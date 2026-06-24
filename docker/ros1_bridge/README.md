# ROS 1 (Noetic) ↔ ROS 2 (Jazzy) bridge for the scene-graph pipeline

`ros1_bridge` so a **ROS 1 Noetic** robot (or a ROS 1 bag) can feed the **ROS 2
Jazzy** scene-graph pipeline.

**In normal use you don't run anything here directly.** The bridge runs *inside*
the unified dev container (see `../README.md`): `supervisor.sh` calls
`start_bridge.sh` with the `bridge_topics.yaml` allowlist. This directory holds:

- `start_bridge.sh` + `bridge_topics.yaml` — the shared bridge launch logic +
  topic allowlist, consumed by the unified container's supervisor.
- `Dockerfile` + `entrypoint.sh` — an **optional standalone** bridge-only image
  (`sysnav-ros1-bridge:latest`) for inspecting the bridge in isolation.

The rest of this doc is bridge-internals reference: the upstream builder
dependency, the allowlist + QoS, `param-bridge` vs `dynamic_bridge`, and the
live-calibration gotchas.

## Why a prebuilt upstream builder (read before building)

Noetic targets Ubuntu 20.04 and Jazzy targets 24.04, so `ros1_bridge` must be
compiled against two distros built for different Ubuntu releases — a notoriously
finicky build (e.g. the `lttng`/`tracetools` `-llttng-ust-common` linker
failure). Rather than re-solve that, we build on
[TommyChangUMD/ros-jazzy-ros1-bridge-builder], which compiles `ros1_bridge`
against Jazzy + a Focal-Noetic copy inside one image and handles those gotchas.
Our `Dockerfile` adds only a thin entrypoint so the container *runs the bridge*.

> **The bridge only carries standard message types.** The pipeline's inputs are
> `PointCloud2`, `Odometry`, `Image`, `CameraInfo`, `TFMessage` (+ `Clock` for the
> bag test) — all bridged by the built-in mappings. Every custom message
> (`tare_planner/*`, `DetectionResult`, `ObjectNode`, …) is ROS 2-internal and
> never crosses, so there is **no custom-message rebuild** to do.

## Topology

```
   ROS 1 master (robot or bag)        bridge container            HOST (native Jazzy)
   ──────────────────────────         ────────────────            ───────────────────
   /<ns>/cloud_registered   ─┐
   /<ns>/lio/odometry        ├─ roscpp ─►  dynamic_bridge  ─ DDS ─►  scene_graph.launch
   /<ns>/camera/image_rect_color   (selective: only subscribed)      (planner, room_seg,
   /<ns>/camera_rect/camera_info                                      semantic_mapping, vlm,
   /tf, /tf_static  (+ /clock, bag only)                              exporter)
```

`<ns>` is the `robot_namespace` knob in
`src/exploration_planner/tare_planner/config/robot.yaml` (this bag = `go2w_016`).

## Prerequisites

- Docker (no GPU needed for the bridge itself).
- Linux host (`--network host` for DDS discovery).
- **amd64 only** — the upstream builder does not currently publish arm64 artifacts.

## Build

The upstream bridge compiler is a prerequisite for **the unified image** (its
graft `COPY`s from it) and for the optional standalone bridge image:

```bash
# The upstream bridge compiler (≈10 min; needs ~1 GB RAM per CPU core)
git clone --recurse-submodules \
  https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder.git
cd ros-jazzy-ros1-bridge-builder
docker build . -t ros-jazzy-ros1-bridge-builder:latest
```

The unified image grafts the compiled bridge from this builder (see
`../Dockerfile`), so for normal use you stop here. **Optional** — a standalone
bridge-only image:

```bash
cd /home/all/AlphaZ/SysNav
docker build -t sysnav-ros1-bridge:latest docker/ros1_bridge
```

---

## Run the bridge — through the unified container

Both the ROS 1 bag and the live robot run through the unified container, which
starts roscore (bag mode), the bridge, and the pipeline together with the right
`use_sim_time` and start ordering:

```bash
# ROS 1 bag through the bridge (set robot.yaml -> robot_namespace to match the bag)
MODE=bag  BAG=/home/all/AlphaZ/bags/multifloor_test_slam docker/run.sh

# live robot
MODE=live ROBOT_IP=<robot-ip> LAPTOP_IP=<laptop-ip> docker/run.sh
```

See `../README.md` for all knobs. The bridge always runs in `param-bridge` mode
(the explicit allowlist) — eager bridges + per-topic QoS remove the ~30s live
calibration lag (see [Bridging modes](#bridging-modes-selective-vs-allowlist)).

## Standalone bridge (optional, debugging)

To run *only* the bridge in its own container — e.g. to inspect what crosses
without the pipeline — use the optional `sysnav-ros1-bridge:latest` image:

```bash
docker run --rm -it --network host --ipc host \
  -e ROS_MASTER_URI=http://<robot-ip>:11311 \
  -e ROS_IP=<laptop-ip> \
  -e ROS_DOMAIN_ID=0 \
  -v $(pwd)/docker/ros1_bridge/bridge_topics.yaml:/bridge_topics.yaml:ro \
  -v $(pwd)/src/exploration_planner/tare_planner/config/robot.yaml:/robot.yaml:ro \
  sysnav-ros1-bridge:latest param-bridge
```

- **`ROS_IP=<laptop-ip>` is required live** — your laptop's IP on the robot's
  subnet (e.g. `192.168.123.190`); without it the robot's ROS 1 publishers can't
  send data back.
- **No robot name to edit.** `bridge_topics.yaml` uses a `__NS__` placeholder; the
  `param-bridge` entrypoint reads `robot_namespace` from the mounted `robot.yaml`
  (the one place the robot name lives) and renders it before loading.
- **No UDP-only FastDDS profile.** That hack only existed for a root *container* ↔
  non-root *host* process. Run the pipeline in another container on the same
  `--ipc host` (or just use the unified container) and DDS shared memory works.

> ⚠️ **`use_sim_time` flips bag → live.** The unified container sets it for you,
> but if you launch the pipeline by hand: `true` for the bag (it publishes
> `/clock`), **`false`** live (wall clock, no `/clock`). Leave it `true` live and
> the pipeline waits forever for a `/clock` that never arrives. Live also has no
> end-of-bag watchdog — rely on periodic snapshots (`save_interval_s`) or the
> manual trigger (`ros2 topic pub --once /keyboard_input std_msgs/String "{data:
> 'ssg'}"`).

## Bridging modes: selective vs allowlist

The unified container always uses **`param-bridge`** (the explicit allowlist) for
both bag and live — it's strictly better on the live robot and fine for the bag.
The selective **`bridge`** (`dynamic_bridge`) survives only as a standalone-image
subcommand, for ad-hoc inspection. The two compared:

### `bridge` — selective `dynamic_bridge` (standalone image only)

Bridges a topic only when the *other* side has a subscriber, so exactly the
pipeline's inputs cross (`cloud_registered`, `lio/odometry`,
`camera/image_rect_color`, `camera_rect/camera_info`, `/tf`, `/tf_static`, plus
`/clock` under sim time) and direction is automatic. Zero config — good for
quick checks.

- `BRIDGE_ALL=1` bridges *everything* (firehose) — primes `/clock` before the
  planner boots (selective mode otherwise won't bridge `/clock` until subscribed).
- A topic isn't bridged until its subscriber exists, so **start the pipeline first**.
  `ros2 topic echo <topic>` alone won't trigger a bridge — pass the type too.
- **On a live robot it's slow to settle (~30s):** discovery over the robot's 300+
  topic graph is slow to establish each bridge; latched `/tf_static` is delivered
  late (a race), so the camera extrinsic `base→<optical>` can take ~30s to appear;
  and lazy `image_proc` nodelets only wake once the bridge subscribes (late). The
  pipeline sits on `Waiting for camera_info + tf calibration...` the whole time.

### `param-bridge` — explicit allowlist `parameter_bridge` (recommended live)

Reads `bridge_topics.yaml` (mounted; loaded onto the master via `rosparam`) and
creates those bridges **eagerly** with **per-topic QoS**. This fixes both lag causes:

- `/tf_static` is published **`transient_local`**, so latched static transforms reach
  late subscribers instantly — no 30s race.
- eager ROS 1 subscription **wakes the lazy rectify nodelet** at bridge startup, so
  `camera_info` is flowing before the pipeline boots.
- only the ~6 listed topics are bridged — no 300+ graph churn.

Result: calibration resolves in **~1–2s instead of ~30s**, and **start order doesn't
matter** (transient_local + eager wake are order-independent — run the pipeline first
if you like; it just waits until the bridge is up, then snaps to calibrated).

**Caveats:** `parameter_bridge` bridges each topic **bidirectionally** (no per-topic
direction), so the laptop's `/tf`,`/tf_static` (incl. the synthetic `odom→map`) also
go to the robot — harmless (the robot gains an unused frame; selective does this too).
Types in the allowlist must be exact. The robot namespace is **not** in the allowlist —
it uses a `__NS__` placeholder that the entrypoint fills from `robot.yaml` (mounted),
so `robot.yaml` stays the single source of the robot name. Nothing else is hardcoded —
only topic names + QoS.

## Verify the bridge

From inside the running container (`docker exec -it <id> bash`, or a separate
`docker/run.sh … shell`), with `<ns>` = your `robot_namespace`:

```bash
ros2 topic list | grep <ns>                 # cloud_registered, lio/odometry, camera/*, …
ros2 topic hz /<ns>/cloud_registered        # data actually flowing
ros2 run tf2_tools view_frames               # odom -> base -> front_cam tree present
```

Using the standalone bridge image, `--print-pairs` shows what it can map:

```bash
docker run --rm --network host sysnav-ros1-bridge:latest \
  ros2 run ros1_bridge dynamic_bridge --print-pairs
```

## Gotchas

- **`use_sim_time` flips bag → live** (see above). The single most common mistake.
- **Bag test must prime `/clock` before the planner boots.** TARE crashes on a
  0 → bag-time clock jump. Run the bag test with **`BRIDGE_ALL=1`** and start the
  bridge+bag **before** the pipeline, so `/clock` (bridged unconditionally) is
  already at bag time when the planner initializes. In *selective* mode `/clock`
  isn't bridged until the planner subscribes — which reintroduces the jump — so
  selective is for the **live** robot (wall clock, no `/clock`), not the bag test.
- **`/tf_static` is latched.** The live robot republishes it continuously, so the
  bridge re-latches it fine. In the bag test, `rosbag play` re-emits it at bag
  start — don't use `--start-offset` (it skips the latched statics), play from 0.
- **QoS is fine by default.** The pipeline subscribes with default *reliable*
  QoS (not best-effort), so the usual "clouds silently don't arrive over the
  bridge" QoS mismatch does not apply here.
- **`ROS_DOMAIN_ID` must match** between the bridge container and the host
  pipeline (both default `0` here).
- **"Topic is listed but `ros2 topic echo` hangs" (only if you split containers).**
  FastDDS uses shared memory for same-host data; a root *container*'s `/dev/shm`
  segments aren't usable by a non-root *host* process even with `ipc: host`. The
  unified container sidesteps this entirely — bridge and pipeline are in the SAME
  container, so SHM just works. (This is why the old UDP-only FastDDS profile was
  retired.) If you do split bridge and pipeline across two containers, share
  `--ipc host` and run both as the same user.
- **Bandwidth.** A full LiDAR `PointCloud2` stream through `dynamic_bridge` can be
  a CPU/bandwidth bottleneck — confirm `ros2 topic hz /go2w_016/cloud_registered`
  keeps up before trusting the live map. Selective mode (the default) already drops
  everything you don't need; the cloud itself is the heavy one and is required.
- **Bridge can't find `local_setup.bash`?** Then this image wasn't built `FROM`
  the upstream builder, or the builder changed its layout — the entrypoint falls
  back to a `find /`, but check `find_bridge_setup` in `entrypoint.sh`.

## Files

| File | Purpose |
|---|---|
| `start_bridge.sh` | Shared bridge launcher: renders `__NS__` from `robot.yaml`, `rosparam load`s the allowlist, runs `parameter_bridge`. Used by the unified container's supervisor AND the standalone entrypoint. |
| `bridge_topics.yaml` | Explicit topic allowlist + QoS for `param-bridge`; `__NS__` placeholder filled from `robot.yaml`. |
| `Dockerfile` | Optional standalone bridge-only image over the upstream builder. |
| `entrypoint.sh` | Standalone-image subcommands: `bridge`, `param-bridge`, `roscore`, `play`, `shell`. |

## Sources

- [TommyChangUMD/ros-jazzy-ros1-bridge-builder] — the prebuilt Jazzy↔Noetic bridge builder this wraps.
- [ros2/ros1_bridge] — upstream bridge package.
- [Using ros1_bridge with upstream ROS — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html)

[TommyChangUMD/ros-jazzy-ros1-bridge-builder]: https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder
[ros2/ros1_bridge]: https://github.com/ros2/ros1_bridge
