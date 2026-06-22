# ROS 1 (Noetic) ↔ ROS 2 (Jazzy) bridge for the scene-graph pipeline

Runtime `ros1_bridge` so a **ROS 1 Noetic** robot (or a ROS 1 bag) can feed the
**ROS 2 Jazzy** scene-graph pipeline live. The pipeline runs natively on the host;
this directory only stands up the bridge.

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

- Docker (no GPU needed for the bridge itself; the pipeline's GPU needs are
  unchanged and run on the host).
- The scene-graph pipeline built/runnable on the host (native ROS 2 Jazzy).
- Linux host (these compose files use `network_mode: host`).
- **amd64 only** — the upstream builder does not currently publish arm64 artifacts.

## Build (two images, once)

```bash
# 1. The upstream bridge compiler (≈10 min; needs ~1 GB RAM per CPU core)
git clone --recurse-submodules \
  https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder.git
cd ros-jazzy-ros1-bridge-builder
docker build . -t ros-jazzy-ros1-bridge-builder:latest

# 2. Our thin runtime layer
cd /home/all/AlphaZ/SysNav
docker build -t sysnav-ros1-bridge:latest docker/ros1_bridge
```

---

## Use it — Step 2 of your test plan: the ROS 1 bag

This is the closest possible proxy for the live robot: it replays the raw ROS 1
`.bag` through Noetic and across the real bridge.

```bash
# Set robot.yaml -> robot_namespace: go2w_016 first (matches this bag).
ROS1_BAG_DIR=/home/all/AlphaZ/bags/multifloor_test_slam \
ROS_DOMAIN_ID=0 \
docker compose -f docker/ros1_bridge/docker-compose.yml up
```

Then, on the host (native Jazzy):

```bash
source install/setup.bash
ros2 launch tare_planner scene_graph.launch use_sim_time:=true   # sim time: /clock comes from the bag
```

`use_sim_time:=true` here because the bag publishes `/clock` (via `rosbag play
--clock`) and the bridge carries it. The default in `scene_graph.launch` is
already `true`, so you can omit the arg.

## Use it — Step 3: the live robot

No bag, no roscore in a container — the bridge talks to the **robot's** master.
Use **`param-bridge`** (the explicit allowlist) here: on a live robot it removes the
~30s startup calibration lag that selective `dynamic_bridge` causes (see
[Bridging modes](#bridging-modes-selective-vs-allowlist)).

```bash
docker run --rm -it --network host --ipc host \
  -e ROS_MASTER_URI=http://<robot-ip>:11311 \
  -e ROS_IP=<laptop-ip> \
  -e ROS_DOMAIN_ID=0 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp_only.xml \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/fastdds_udp_only.xml \
  -v $(pwd)/docker/ros1_bridge/fastdds_udp_only.xml:/fastdds_udp_only.xml:ro \
  -v $(pwd)/docker/ros1_bridge/bridge_topics.yaml:/bridge_topics.yaml:ro \
  sysnav-ros1-bridge:latest param-bridge
```

- **`ROS_IP=<laptop-ip>` is required live** — your laptop's IP on the robot's subnet
  (e.g. `192.168.123.190`); without it the robot's ROS 1 publishers can't send data
  back. The UDP-only profile is needed for the same root-container ↔ host-pipeline
  reason as the bag test.
- **Edit `bridge_topics.yaml`** so the namespace + topic list match the robot
  (`go2w_026`, …). Nothing is hardcoded beyond topic names + QoS.

Then on the host:

```bash
ros2 launch tare_planner scene_graph.launch use_sim_time:=false   # wall clock — no /clock live
```

> ⚠️ **The one trap:** `use_sim_time` flips from `true` (bag) to **`false`**
> (live). Leave it `true` on the live robot and the pipeline waits forever for a
> `/clock` that never arrives. Live also means **no end-of-bag watchdog** — rely on
> periodic snapshots (`save_interval_s`) or the manual trigger
> (`ros2 topic pub --once /keyboard_input std_msgs/String "{data: 'ssg'}"`).

## Bridging modes: selective vs allowlist

Two modes, two entrypoint subcommands. **Bag test → `bridge`; live robot →
`param-bridge`.**

### `bridge` — selective `dynamic_bridge` (default)

Bridges a topic only when the *other* side has a subscriber, so exactly the
pipeline's inputs cross (`cloud_registered`, `lio/odometry`,
`camera/image_rect_color`, `camera_rect/camera_info`, `/tf`, `/tf_static`, plus
`/clock` under sim time) and direction is automatic. Zero config — good for the
**bag test** and quick checks.

- `BRIDGE_ALL=1` bridges *everything* (firehose) — used in the bag test to prime
  `/clock` before the planner boots.
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
Types in the allowlist must be exact, and the robot namespace appears there too (a
per-robot seam, like `robot.yaml`). No transforms/intrinsics are hardcoded — only
topic names + QoS.

## Verify the bridge

With the bridge up **and the pipeline running** (selective mode bridges on demand),
on the host:

```bash
ros2 topic list | grep go2w_016                 # cloud_registered, lio/odometry, camera/*, …
ros2 topic hz /go2w_016/cloud_registered        # data actually flowing
ros2 run tf2_tools view_frames                   # odom -> base -> front_cam tree present
```

Inside the bridge container, `--print-pairs` shows what it can map:

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
- **"Topic is listed on the host but `ros2 topic echo` hangs."** Discovery works
  (UDP) but data doesn't, because FastDDS uses shared memory for same-host data
  and the root container's `/dev/shm` segments aren't usable by the non-root host
  process — even with `ipc: host`. The compose fixes this by forcing the bridge's
  DDS to **UDP-only** (`fastdds_udp_only.xml` + `FASTRTPS_DEFAULT_PROFILES_FILE`).
  Trade-off: UDP loopback is heavier than SHM for the big LiDAR cloud; if it
  bottlenecks, restore SHM by running the container as your host user
  (`user: "$(id -u):$(id -g)"`) instead of disabling SHM.
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
| `Dockerfile` | Thin runtime layer over the upstream builder image. |
| `entrypoint.sh` | Subcommands: `bridge` (default), `param-bridge`, `roscore`, `play`, `shell`. |
| `bridge_topics.yaml` | Explicit topic allowlist + QoS for `param-bridge` (recommended live). |
| `docker-compose.yml` | Bag-test topology: Noetic master+bag + bridge. |

## Sources

- [TommyChangUMD/ros-jazzy-ros1-bridge-builder] — the prebuilt Jazzy↔Noetic bridge builder this wraps.
- [ros2/ros1_bridge] — upstream bridge package.
- [Using ros1_bridge with upstream ROS — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html)

[TommyChangUMD/ros-jazzy-ros1-bridge-builder]: https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder
[ros2/ros1_bridge]: https://github.com/ros2/ros1_bridge
