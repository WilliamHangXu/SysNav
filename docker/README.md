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

# ROS 2 bag straight into the pipeline -- no bridge (the fast dev/test path)
MODE=bag-direct BAG=/home/all/AlphaZ/bags/multifloor_test_slam_ros2 docker/run.sh

# ROS 1 bag through the bridge (in-container roscore + rosbag --clock)
MODE=bag BAG=/home/all/AlphaZ/bags/multifloor_test_slam docker/run.sh

# debug shell (workspace sourced); any MODE's env still applies
MODE=bag-direct BAG=<dir> docker/run.sh shell

# force a colcon rebuild (e.g. after C++ changes)
BUILD=1 MODE=bag-direct BAG=<dir> docker/run.sh
```

### Knobs

| Env | Default | Meaning |
|---|---|---|
| `MODE` | `live` | `live` \| `bag` \| `bag-direct` |
| `RVIZ` | `1` | Launch RViz (forwards X; see below) |
| `OBJECTS` | `0` | Default = rooms + navgraph only (also skips the GPU YOLO engine export); `1` adds object detection+mapping |
| `ROS_DOMAIN_ID` | `0` | DDS domain |
| `BUILD` | `0` | `1` forces `colcon build` even if the volume already has one |
| `FORCE_ENGINE_REBUILD` | `0` | `1` re-exports the YOLO TensorRT engines |
| `VOLUME` | `sysnav-build` | Named volume holding `/app/{build,install,log}` |
| `IMAGE` | `sysnav:latest` | Image tag to run |
| `ROBOT_IP` | — | **live**: robot's IP (its `roscore` host) → `ROS_MASTER_URI` |
| `LAPTOP_IP` | — | **live**: your IP on the robot's subnet → `ROS_IP` |
| `BAG` | — | **bag / bag-direct**: host bag directory (mounted ro at `/app/bag`) |

Cloud-VLM credentials (`GEMINI_API_KEY`, `DASHSCOPE_API_KEY`, `VLM_PROVIDER`,
`QWEN_MODEL`, `QWEN_MODEL_LITE`) pass through from your environment if set.

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
   - **bag** — in-container roscore → bridge → `rosbag play --clock`, prime `/clock`,
     then `scene_graph.launch use_sim_time:=true`.
   - **bag-direct** — `ros2 bag play --clock --start-paused` → launch the stack →
     resume the bag. No bridge.

Per-run logs land in `runlogs/<timestamp>/{bridge,pipeline,bag,…}.log` (mounted to
the host). `Ctrl-C` → `tearing down (N processes)` → clean exit.

## RViz / X

`RVIZ=1` forwards the host X server and renders on the dGPU. The CUDA base only
requests `compute,utility` driver caps, so `run.sh` adds
`NVIDIA_DRIVER_CAPABILITIES=all` + `--device /dev/dri` (else RViz falls back to
Mesa and dies with `failed to load driver: iris`). You may need `xhost
+local:root` once per host login. Set `RVIZ=0` for headless.

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
