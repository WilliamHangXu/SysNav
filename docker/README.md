# SysNav — unified dev container

One image runs the **whole scene-graph pipeline** plus the **ROS 1 ↔ ROS 2
bridge**, against a live ROS 1 Noetic robot or a bag. One command, a few env
knobs.

## Design: bake the stable floor, mount the volatile workspace

| | |
|---|---|
| **Baked into the image** | Ubuntu + CUDA, ROS 2 Jazzy (desktop-full), ROS 1 Noetic + the compiled `ros1_bridge` (grafted from the upstream builder), native SLAM deps (Sophus/Ceres/GTSAM/Livox), the Python/ML stack, and the `sam2` editable install. |
| **NOT baked** | The application source, the colcon build, the model weights. `src/` is bind-mounted at runtime and `colcon build --symlink-install` runs **in-container into a named volume**, so day-to-day pipeline edits need no image rebuild. |

The pipeline is under constant development, so its source is mounted, not baked.
The model weights (SAM2/CLIP/YOLO `.pt`/`.ts`) already live under `src/`, so they
arrive via the same mount. `docker/` is mounted too, so editing `run.sh` /
`supervisor.sh` applies on the next run with no rebuild.

Because the bridge and the pipeline share one container, their DDS traffic is
intra-container shared memory — the old cross-container UDP-only FastDDS hack is
gone.

## Build (two images, once)

```bash
# 1. the upstream bridge compiler -- see docker/ros1_bridge/README.md
git clone --recurse-submodules \
  https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder.git
cd ros-jazzy-ros1-bridge-builder && docker build . -t ros-jazzy-ros1-bridge-builder:latest

# 2. this image (context = repo root)
cd /home/all/AlphaZ/SysNav
docker build -f docker/Dockerfile -t sysnav:latest .
```

The first build is long (CUDA base + desktop-full + the ML stack + GTSAM/Ceres +
detectron2/pytorch3d CUDA kernels). Layers are ordered for cache reuse; the
bridge graft is last, so it only re-runs if the builder image changes.

## Run

`docker/run.sh` is the host launcher. Knobs are env vars:

```bash
# live robot (robot is the ROS 1 master; pipeline runs on wall clock)
MODE=live ROBOT_IP=192.168.123.18 LAPTOP_IP=192.168.123.190 docker/run.sh

# demo: live wiring, but the robot gates the run over /scene_graph_generator/*
MODE=demo ROBOT_IP=192.168.123.18 LAPTOP_IP=192.168.123.190 docker/run.sh

# live/demo + record what the pipeline receives as a ROS 2 bag -> output/recordings/<ts>/
MODE=live ROBOT_IP=192.168.123.18 LAPTOP_IP=192.168.123.190 RECORD=1 docker/run.sh

# ROS 2 bag straight into the pipeline -- no bridge (the fast dev/test path)
MODE=bag-direct BAG=/home/all/AlphaZ/bags/multifloor_test_slam_ros2 docker/run.sh

# ROS 1 bag through the bridge (in-container roscore + rosbag --clock)
MODE=bag BAG=/home/all/AlphaZ/bags/multifloor_test_slam docker/run.sh

# play only a window of the bag: skip 200s in, then play 230s (either knob optional)
MODE=bag-direct BAG=<dir> START_OFFSET=200 DURATION=230 docker/run.sh

# RViz stays up after the bag ends by default so you can inspect the result (Ctrl-C
# to quit); pass HOLD=0 to auto-exit when the bag finishes (e.g. scripted/batch runs)
MODE=bag-direct BAG=<dir> DURATION=230 HOLD=0 docker/run.sh

# debug shell (workspace sourced); any MODE's env still applies
MODE=bag-direct BAG=<dir> docker/run.sh shell

# force a colcon rebuild (e.g. after C++ changes), THEN run the pipeline
BUILD=1 MODE=bag-direct BAG=<dir> docker/run.sh

# just build, no pipeline -- incremental colcon build into the named volume (no MODE/bag)
docker/run.sh build

# just build, no pipeline -- full clean rebuild: wipe build/install/log, build from scratch
docker/run.sh rebuild
```

### Knobs

| Env | Default | Meaning |
|---|---|---|
| `MODE` | `live` | `live` \| `demo` \| `bag` \| `bag-direct` |
| `RVIZ` | `1` | Launch RViz (forwards X; see below) |
| `OBJECTS` | `0` | Default = rooms + navgraph only (also skips the GPU YOLO engine export); `1` adds object detection+mapping |
| `ROS_DOMAIN_ID` | `0` | DDS domain |
| `BUILD` | `0` | `1` forces `colcon build` even if the volume already has one |
| `FORCE_ENGINE_REBUILD` | `0` | `1` re-exports the YOLO TensorRT engines |
| `VOLUME` | `sysnav-build` | Named volume holding `/app/{build,install,log}` |
| `IMAGE` | `sysnav:latest` | Image tag to run |
| `NAME` | `sysnav` | Container name → `docker exec -it sysnav bash` to attach a second terminal (see below). Override to run more than one container at once |
| `INS` | `0` | `1` auto-opens a second host terminal already `docker exec`'d into the container, with sample ROS inspection commands printed (commented out). Scripts the "attach a second terminal" recipe below; needs a host terminal emulator + X |
| `ROBOT_IP` | — | **live**: robot's IP (its `roscore` host) → `ROS_MASTER_URI` |
| `LAPTOP_IP` | — | **live**: your IP on the robot's subnet → `ROS_IP` |
| `BAG` | — | **bag / bag-direct**: host bag directory (mounted ro at `/app/bag`) |
| `START_OFFSET` | — | **bag / bag-direct**: seconds to skip from the bag start (empty = from 0) |
| `DURATION` | — | **bag / bag-direct**: seconds to play, then stop (empty = to the end) |
| `HOLD` | `1` | **bag / bag-direct / demo**: keeps the stack (incl. RViz) up after the bag finishes (or, in demo, after the robot's run completes), for inspection (`Ctrl-C` to quit); set `0` to auto-exit when the run ends (scripted/batch). No effect on live (no end event — runs until `Ctrl-C`) |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` (bag/bag-direct) | Confines ROS 2 discovery to this host so two laptops on the same WiFi don't collide (see below). Set `SUBNET` to opt back into cross-host ROS 2 |
| `RECORD` | `0` | **live / demo**: `1` records a ROS 2 bag of the pipeline inputs to `output/recordings/<ts>/` (see below). No effect on bag / bag-direct |

Cloud-VLM credentials (`GEMINI_API_KEY`, `DASHSCOPE_API_KEY`, `VLM_PROVIDER`,
`QWEN_MODEL`, `QWEN_MODEL_LITE`) pass through from your environment if set.

#### Two laptops on the same WiFi (bag/bag-direct)

`bag` and `bag-direct` are a fully self-contained ROS 2 graph (bag → pipeline, no
robot off-box). But `run.sh` uses `--network host` with the default
`ROS_DOMAIN_ID`, so two people running on the same LAN/WiFi would otherwise
auto-discover each other and cross-wire `/clock`, `/tf`, node names and
`/rosbag2_player` — silently breaking **both** runs. To prevent that, the two bag
modes default `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, isolating each laptop.
(`live`/`demo` reach the robot over **ROS 1**, not ROS 2, so this is left at the
ROS 2 default there and doesn't affect the robot link.) If you genuinely want two
machines to share one ROS 2 graph, set `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` on
both and give them a matching `ROS_DOMAIN_ID`.

#### Recording a bag (live / demo)

`RECORD=1` records **exactly what the pipeline receives** — the bridged sensor/TF
inputs (`<ns>/cloud_registered`, `<ns>/lio/odometry`, `<ns>/camera/image_rect_color`,
`<ns>/camera_rect/camera_info`, `/tf`, `/tf_static`) — to a ROS 2 bag under
`output/recordings/<ts>/`. The topic set is derived from `bridge_topics.yaml`
(rendered for the namespace), minus `/clock` (regenerated on replay) and the demo
control channel. Off by default.

- **No WiFi cost.** It records the *in-container ROS 2* side — data the bridge has
  already pulled to your laptop once — so it adds no robot↔laptop traffic. (A ROS 1
  `rosbag record` would re-subscribe to the robot and roughly double the sensor
  traffic over WiFi; that's why we record ROS 2 only.)
- **Replay** the result directly in `bag-direct` — it *is* a ROS 2 bag of the
  pipeline inputs:
  ```bash
  MODE=bag-direct BAG=output/recordings/<ts> docker/run.sh
  ```
- **demo** records only the **run window** (robot `start` → run done); **live**
  records for the container's lifetime. The bag is finalized cleanly on stop
  (`SIGINT` → rosbag2 flushes and closes its file before teardown).
- **Need a ROS 1 `.bag` later?** Convert offline with `rosbags-convert` (Ternaris
  `rosbags` pip pkg; lossless for these standard types). Not needed to re-run the
  pipeline — `bag-direct` plays the ROS 2 bag as-is.

> Note: `/tf_static` is latched (`transient_local`). rosbag2 captures the latched
> message on subscribe and `ros2 bag play` re-asserts it, so the TF tree comes back
> on replay — the one thing to sanity-check the first time you record.

> **Artifact ownership.** The container runs as root, so anything it writes under
> the bind-mounted `output/` and `runlogs/` (recordings, snapshots, per-run logs)
> would otherwise land root-owned on the host — un-deletable without `sudo` (a lock
> icon in the file manager). On exit the supervisor `chown`s both trees back to your
> uid/gid (`HOST_UID`/`HOST_GID`, set automatically by `run.sh`), recursively, so it
> also reclaims older root-owned runs. If a container is ever hard-killed
> (`docker kill`) and skips that step, reclaim by hand from the repo root:
> ```bash
> sudo chown -R "$(id -u):$(id -g)" output runlogs
> ```

#### Attach a second terminal (`ros2 topic hz`, etc.)

The container is named `sysnav` (the `NAME` knob), so from another terminal you can
drop into the **running** container and inspect its live ROS 2 graph:

```bash
docker exec -it sysnav bash
ros2 topic hz /go2w_026/cloud_registered     # ~/.bashrc already sourced ROS 2 + workspace
ros2 topic list                              # bridged inputs + pipeline-internal topics
```

`docker exec` shares the container's network/IPC namespace, `ROS_DOMAIN_ID` and RMW,
so you see exactly the pipeline's graph — no version/discovery mismatch. The
container's `~/.bashrc` sources ROS 2 + the workspace on startup, so the shell is
ready immediately. For **ROS 1** topics (raw robot topics that aren't bridged),
`source /opt/ros/noetic/setup.bash` in that shell (the `ROS_MASTER_URI` / `ROS_IP`
are already set) and use `rostopic hz`. To attach to a second concurrent container,
launch it with `NAME=...` and `docker exec` into that name.

**`INS=1` automates this.** Add it to any run and, once the container is up (and the
workspace is sourced), `run.sh` opens a **new terminal already inside the container**
with these sample commands pre-printed as comments (copy a line, drop the `#`, run):

```bash
INS=1 MODE=bag-direct BAG=/home/all/AlphaZ/bags/multifloor_test_slam_ros2 docker/run.sh
```

It's off by default (`INS=0`) and works in every mode. It needs a host terminal
emulator (`gnome-terminal` / `konsole` / `xfce4-terminal` / `tilix` / `xterm`, or the
Debian `x-terminal-emulator` alias) and an X session; if none is found it prints the
manual `docker exec -it <NAME> bash` line instead. The sample commands are filled in
with the live `robot_namespace` from `robot.yaml`. The watcher runs in the
background, so the main run keeps the foreground (and `Ctrl-C` behaves as usual); on
the first run it waits out the one-time colcon build before opening, so the shell is
ROS-ready when it appears.

### Finding the live IPs

`ROBOT_IP` / `LAPTOP_IP` are discovered fresh per setup, never baked:

```bash
ROBOT=192.168.123.18                                       # the robot's roscore host
LAPTOP=$(ip route get $ROBOT | grep -oP 'src \K[\d.]+')    # your IP on that subnet (auto)
```

`ip route get` picks the right NIC even with multiple interfaces. Confirm the
robot is the master with `nc -vz $ROBOT 11311`.

## What a run does

`docker/run.sh` starts the container with `--gpus all --network host --ipc host`
and hands off to `docker/supervisor.sh`, which:

1. **Provisions the mounted workspace** (idempotent; slow bits run once):
   - `sam2` import check — baked, so normally instant; self-heals if missing.
   - `colcon build --symlink-install` into the named volume, if absent or `BUILD=1`.
   - YOLO TensorRT engine export on the GPU, if the `.engine` files are missing
     (engines are GPU-arch specific, so they can't be baked).
2. **Runs the chosen MODE** (each backgrounds its pieces, tears them all down on exit):
   - **live** — start the bridge → wait → `scene_graph.launch use_sim_time:=false`.
   - **demo** — start the bridge, then **gate**: nothing else runs until the robot
     publishes `start` on `/scene_graph_generator/request`. Then launch the
     pipeline; on `complete` save the scene graph and stream its JSON back on
     `/scene_graph_generator/response` every 5 s until the robot replies
     `received`; `cancel` saves locally and tears down. Both control topics are
     `std_msgs/String` (see `demo_control.py`).
   - **bag** — in-container roscore → bridge → `rosbag play --clock`, prime `/clock`,
     then `scene_graph.launch use_sim_time:=true`.
   - **bag-direct** — `ros2 bag play --clock --start-paused` → launch the stack →
     resume the bag. No bridge.

Per-run logs land in `runlogs/<timestamp>/{bridge,pipeline,bag,…}.log` (mounted to
the host). `Ctrl-C` → `tearing down (N processes)` → clean exit.

The pipeline is backgrounded, so its output (RViz, planner, VLM, room
segmentation, …) goes to the logfile, not your terminal. Follow it live from
another terminal:

```bash
L=$(ls -dt runlogs/*/ | head -1)   # newest run dir
tail -f "$L"pipeline.log           # whole stack; or bridge.log / bag.log
tail -f "$L"*.log                  # all of them at once
```

Run it from the repo root (else anchor the glob: `runlogs/` → `<repo>/runlogs/`).

### Demo-mode handshake (MODE=demo)

The robot drives the whole run over two `std_msgs/String` topics:

| `/scene_graph_generator/request` | Effect |
|---|---|
| `start` | launch the scene-graph pipeline (it was held down) |
| `complete` | save a snapshot, then stream the JSON on `…/response` every 5 s until `received` |
| `received` | stop streaming, tear the system down |
| `cancel` | save the snapshot locally, then tear down (no `…/response` traffic) |

Optional overrides: `REQ_TOPIC`, `RESP_TOPIC`, `ACK_TIMEOUT` (default 300 s — a
guard so a lost `received` can't stream forever). The JSON is also written to
`output/scene_graph/run_*/` (with a `latest.json` pointer) exactly as in other
modes; the response stream just re-reads the freshest snapshot.

By default (`HOLD=1`) the stack — including RViz — stays up after the run ends
(`received` / `cancel`) for inspection; the live feed keeps updating (wall clock)
until you `Ctrl-C`. Set `HOLD=0` to tear down automatically when the run ends.

## Code changes: rebuild, recompile, or just re-run?

Recap of the design: **the image is a stable base; your workspace is mounted.**
So after editing code — or `git pull`-ing someone else's changes — most of the
time you just re-run. Only specific changes need work. Three levels, cheapest
first:

| You changed… | Do this | Cost |
|---|---|---|
| Python ROS nodes (`.py` logic), `docker/` scripts (`run.sh`/`supervisor.sh`/…), config yamls, model weights | **nothing** — just re-run `docker/run.sh` | instant (mounted live) |
| Pipeline **C++** (tare_planner, bag_slam_bridge, …), or *added* Python nodes / entry points | `BUILD=1 … docker/run.sh` — recompiles into the build volume | incremental colcon (fast) |
| `requirement.txt`, `docker/Dockerfile`, an apt package, or a **vendored native dep** (`src/slam/dependency/{Sophus,ceres-solver,gtsam}`, `Livox-SDK2`) | **rebuild the image** (below) | minutes (layer cache helps) |

```bash
docker build -f docker/Dockerfile -t sysnav:latest .     # rebuild the image
```

After a `git pull`, this tells you which level you're at — it lists any pulled
changes to the files that are *baked* into the image:

```bash
git diff --stat ORIG_HEAD HEAD -- docker/Dockerfile requirement.txt \
  src/slam/dependency src/utilities/livox_ros_driver2/Livox-SDK2
```

Any output → **rebuild the image**. Otherwise: if C++ under `src/` changed →
`BUILD=1`; if only Python/config/scripts changed → just run.

> ⚠️ **The vendored native deps live under `src/` but are compiled at
> image-build time, not by the in-container colcon build** — so changing
> `src/slam/dependency/*` or `Livox-SDK2` needs an **image** rebuild, not just
> `BUILD=1`. Everything else under `src/` is the normal mounted workspace.

### Build without running the pipeline (`build` / `rebuild`)

`BUILD=1 … docker/run.sh` recompiles *and then launches a run*. When you just want
to compile — CI, a pre-flight check, or warming the build volume — use the build-only
subcommands. Both compile into the named volume and **exit** (no robot, no bag, no
pipeline), so they need **no `MODE`/`ROBOT_IP`/`BAG`**:

```bash
docker/run.sh build      # incremental colcon build (same compile BUILD=1 does)
docker/run.sh rebuild    # wipe build/install/log in the volume, then build from scratch
```

- **`build`** — `colcon build --symlink-install` into `sysnav-build`. Incremental:
  only changed packages recompile (instant if nothing changed). This is the plain
  compile that `BUILD=1` performs, minus the run afterward.
- **`rebuild`** — first `rm -rf /app/{build,install,log}` *inside the volume*, then a
  full from-scratch build. Reach for this when incremental is suspect: after an image
  rebuild (so the C++ relinks against the fresh baked libs — see below), after
  changing a package's CMake/dependencies, or to clear a corrupted build tree. It's
  the in-place equivalent of `docker volume rm sysnav-build` followed by a build, but
  it keeps the same volume.

Both honor the `VOLUME` / `IMAGE` knobs and write incrementally, so a later
`docker/run.sh` (any mode) starts immediately against the freshly built volume.

### Your host `colcon build` does not carry into the container

Only `src/` is mounted. The container builds into the **named volume**
`sysnav-build` (`/app/{build,install,log}`), which your host's
`build/`/`install/`/`log/` never touch — and host-compiled binaries are linked
against host libraries, so they wouldn't load in here anyway. The container
always builds its own. The upside: that volume **persists across runs**, so
`BUILD=1` is needed only on the *first* run after a C++ change, and colcon
recompiles incrementally (just the changed packages). For a tight C++ loop, shell
in once and rebuild by hand instead of relaunching the whole stack each time:

```bash
docker/run.sh shell                                   # workspace sourced, volume mounted
colcon build --symlink-install --packages-select tare_planner
ros2 launch tare_planner scene_graph.launch ...       # edit on host -> rebuild pkg -> relaunch
```

### When you rebuild the image, reset the build volume

Wipe the old build so the C++ isn't linked against stale baked libraries — either
drop the volume, or just `rebuild` it in place:

```bash
docker volume rm sysnav-build      # next run does a clean colcon build into a fresh volume
docker/run.sh rebuild              # same effect without a run: wipe build/install/log, build fresh
```

## RViz / X

`RVIZ=1` forwards the host X server and renders on the dGPU. The CUDA base only
requests `compute,utility` driver caps, so `run.sh` adds
`NVIDIA_DRIVER_CAPABILITIES=all` + `--device /dev/dri` (else RViz falls back to
Mesa and dies with `failed to load driver: iris`). The container runs as root, so
`run.sh` also grants it X access (`xhost +local:root`) for the lifetime of the run
and revokes it on exit — without that, RViz aborts with `Authorization required …
could not connect to display`. (Needs `xhost`, i.e. `x11-xserver-utils`, on an X11
session; harmless no-op otherwise.) Set `RVIZ=0` for headless.

## Per-robot / per-bag config

The one file to edit when moving to a different robot or bag is
`src/exploration_planner/tare_planner/config/robot.yaml`:

- `robot_namespace` — the topic/tf prefix (`go2w_026` live, `go2w_016` for the
  multifloor bag). Wrong value → the pipeline waits forever on calibration.
- `registered_scan_source` — `bag` (read the robot/bag's own registered cloud +
  `/lio/odometry`) or `slam_bridge` (register the raw lidar in-container). The
  bridge allowlist (`docker/ros1_bridge/bridge_topics.yaml`) carries whichever
  cloud topic that source needs.

## Files

| File | Purpose |
|---|---|
| `Dockerfile` | The unified image (multi-stage: bridge graft + CUDA/Jazzy/ML floor). |
| `Dockerfile.dockerignore` | Keeps weights/bags/build artifacts out of the build context. |
| `run.sh` | Host launcher — knobs, mounts, GPU/X, per-mode env. |
| `supervisor.sh` | In-container orchestration — provision then run the chosen MODE. |
| `ros1_bridge/` | The bridge: shared launch logic + allowlist, and an optional standalone bridge image. See its README. |

## See also

- `docker/ros1_bridge/README.md` — bridge internals: the upstream builder
  dependency, the topic allowlist + QoS (`bridge_topics.yaml`), `param-bridge`
  vs `dynamic_bridge`, and the live-calibration gotchas.
- The tmuxp runners (`vlm_ros_alphaz_*.tmuxp.yaml`) — the native (non-container)
  equivalent; `bag-direct` mirrors `vlm_ros_alphaz_bag_direct.tmuxp.yaml`.
