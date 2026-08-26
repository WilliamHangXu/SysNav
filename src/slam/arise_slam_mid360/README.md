# arise_slam_mid360 (ARISE SLAM)

ARISE is the stack's **LiDAR-inertial state estimator** for the Livox Mid-360:
a LOAM-family scan-to-map odometry tightly coupled with a GTSAM IMU smoother.
As far as the rest of SysNav is concerned, this whole package exists to produce
exactly **two topics**:

| Topic | Type | What it is |
|---|---|---|
| `/registered_scan` | `sensor_msgs/PointCloud2` | One deskewed LiDAR sweep, transformed into the SLAM world frame (`map`), ~10 Hz |
| `/state_estimation` | `nav_msgs/Odometry` | The LiDAR pose in that same `map` frame, IMU-rate smoothed (~50 Hz) |

Everything downstream — terrain analysis, local planner, TARE, semantic
mapping, room segmentation, the scene-graph exporter — consumes only this pair
(§ *The downstream contract*). That is also why the package can be swapped out
wholesale: when a bag already carries its own LIO,
`src/slam/bag_slam_bridge` replays the bag's odometry onto the same two topics
and ARISE is not launched at all (`system_bag_direct.launch`).

Mental model: **three nodes, three time scales.**

1. `feature_extraction_node` — per *sweep* (~10 Hz): assemble, deskew, extract features.
2. `laser_mapping_node` — per *sweep*, heavier: scan-to-map optimization → the authoritative pose; emits `/registered_scan`.
3. `imu_preintegration_node` — per *IMU sample* (~200 Hz): propagate the latest optimized pose forward with the IMU; emits `/state_estimation`.

```
/<ns>/livox/lidar (CustomMsg, ~10 Hz)        /<ns>/livox/imu (~200 Hz)
  (livox_ros_driver2's msg_MID360_launch.py remaps these to /lidar/scan and
   /imu/data = the config's laser_topic / imu_topic)
        │                                          │
        ▼                                          │ (both nodes consume raw IMU)
┌─ feature_extraction_node ◄───────────────────────┤
│  per-point time, gyro-orientation buffer,        │
│  rotation-only deskew, uniform surface features  │
└──► /feature_info  (LaserFeature: deskewed cloud  │
        │            + features + IMU initial q)   │
        ▼                                          │
┌─ laser_mapping_node  (100 ms timer)              │
│  initial guess → scan-to-map ICP (Ceres)         │
│  against rolling local voxel map                 │
├──► /registered_scan      (sweep × optimized pose)│
├──► /laser_odometry       (optimized pose) ───┐   │
└──► /aft_mapped_to_init_incremental           │   │
                                               ▼   ▼
                              ┌─ imu_preintegration_node
                              │  GTSAM: IMU factors + laser pose factors
                              │  → bias estimate → forward propagation
                              ├──► /state_estimation  (every 4th IMU msg)
                              ├──► /state_estimation_health
                              └──► TF  map → sensor
```

`PROJECT_NAME` is `""` in our configs, so all topic names above are absolute.

---

## Components

| Directory | Contents |
|---|---|
| `src/FeatureExtraction/` | `featureExtraction.cpp` — the front end: sweep buffering, IMU/lidar sync, deskew, feature extraction dispatch. `LidarKeypointExtractor` / `DepthImageKeypointExtractor` — curvature/uniform keypoint pickers. |
| `src/LaserMapping/` | `laserMapping.cpp` — the back-end ROS node: buffers `/feature_info`, picks the initial guess, runs the optimization, publishes everything. `LocalMap.cpp` — rolling voxelized local map. `lidarOptimization.cpp` — Ceres cost functions. |
| `src/LidarProcess/` | `LidarSlam.cpp` — `slam.Localization()`, the actual scan-to-map ICP core (point-to-line/plane residuals, degeneracy analysis). `RollingGrid.cpp`, `pose_local_parameterization.cpp`, `SE3AbsolutatePoseFactor.cpp` — its support code. |
| `src/ImuPreintegration/` | `imuPreintegration.cpp` — GTSAM factor-graph smoother + IMU-rate forward propagation. |
| `src/parameter/` | Global parameter loading; defines `WORLD_FRAME` / `SENSOR_FRAME` used in every header stamp. |
| `src/scanRegistration_node.cpp`, `src/ousterTransform.cpp` | Legacy / other-sensor paths, not part of the go2w launch. |
| `launch/arize_slam_go2w.launch.py` | Starts the three nodes with `config/livox_mid360_go2w.yaml` + `config/livox/livox_mid360_calibration.yaml` (LiDAR↔IMU extrinsic). |

Inactive inputs you will see in the code: a VIO odometry subscription
(`vins_estimator/imu_propagate` — no VINS node runs in this stack, so the VIO
prediction/deskew branches never fire) and RealSense depth fusion (commented
out).

---

## How `/registered_scan` is produced

### 1. Front end (`feature_extraction_node`)

- **Ingest** (`featureExtraction.cpp:1541`, `livoxHandler`): one Livox
  `CustomMsg` = one sweep (~13k points). Each point gets an absolute time via
  `point.time = offset_time / 1e9` (`:1594`); reflectivity becomes `intensity`;
  too-close/too-far returns are dropped (`min_range`/`max_range`).
- **IMU normalization** (`normalizeImuData`, `:1152`): raw IMU samples are
  rotated by `imu_laser_R_Gravity` — the gravity/attitude rotation estimated
  once at startup while the robot is static — and the accelerometer is
  rescaled to ‖g‖ = 9.8105. The node keeps a buffer of gyro-integrated
  orientations `q_w_i` (one per IMU sample).
- **Sync** (`synchronize_measurements`, `:306`): waits until the IMU buffer
  brackets the sweep interval; otherwise the sweep is skipped with a warning.
- **Deskew** (`imuRemovePointDistortion`, `:359-491`): for every point, slerp
  the orientation buffer to the point's timestamp (`:469`), form the rotation
  *relative to sweep start* `q_orig_i = q_w_original⁻¹ · q_w_i` (`:472`), and
  rotate the point by it (`:486`). The translation of this correction is
  **zeroed** (`:473`) — ARISE deskew is **rotation-only**; intra-sweep
  translation is not compensated here (see `shift_undistortion` below).
  Result: all points expressed in the LiDAR frame *at sweep-start time*.
- **Features** (`undistortionAndscanregistration`, `:1063`): for Livox the
  extractor is `uniformfeatureExtraction` — a uniform subsample used as
  *surface* features; the edge set stays empty (`:1120-1132`).
- **Publish** (`publishTopic`, `:1036`): one `LaserFeature` message on
  `/feature_info` carrying the full deskewed cloud (`cloud_nodistortion`,
  stamped `SENSOR_FRAME`), the feature clouds, and the gyro-integrated
  orientation at sweep start as the **IMU initial guess** for mapping. Stamp =
  sweep-start time.

### 2. Back end (`laser_mapping_node`)

Runs `process()` from a 100 ms wall timer (`laserMapping.cpp:113`,`:1138`).
Each cycle takes the oldest buffered frame and **drops any backlog**
(`:1237-1264`) — it keeps up with real time rather than processing every sweep.

- **Initial guess** (`setInitialGuess`, `:460`): the *first* frame hard-sets
  the map orientation from the IMU's roll/pitch with **yaw zeroed**
  (`:464-503`) — this is what makes the `map` frame gravity-aligned and
  defines its origin/heading at the startup pose. After that,
  `selectposePrediction` (`:553`) prefers an external odometry prediction
  (VIO, or LIO on `integrated_to_init5` — neither is published in this
  configuration), and so in practice falls back to **IMU orientation +
  constant-velocity translation**: relative gyro rotation applied to the last
  optimized pose, plus a low-passed per-frame displacement (`shiftX/Y/Z`,
  `:574`, `:932-934`). The active source is published on `/prediction_source`.
- **(Optional) shift undistortion** (`:1187-1235`, `config_.shift_undistortion`):
  re-applies that displacement estimate per point to approximate the missing
  translational deskew.
- **Optimization**: `slam.Localization(...)` (`:1303-1311`) — ICP-style
  scan-to-map registration of the feature clouds against the rolling local
  voxel map (Ceres; eigenvalue-based degeneracy detection sets
  `slam.isDegenerate`). The result is the authoritative pose
  `q_w_curr / t_w_curr` = `T_w_lidar` (`:1316-1320`).
- **Publish** (`publishTopic`, `:825`):
  - `/registered_scan`: every point of the deskewed full-res sweep is mapped
    by `pointAssociateToMap` — `p_w = q_w_curr · p + t_w_curr` (`:398-406`,
    loop at `:868-877`) — and published with `frame_id = WORLD_FRAME`
    (`map`) and **stamp = sweep-start time** (`:879-883`). So a registered
    scan is *exactly* `optimized pose ∘ deskewed raw sweep*, one sweep per
    message, intensity preserved.
  - `/laser_odometry`: the same optimized pose as Odometry (`map` → `sensor`),
    with the degeneracy flag tucked into `pose.covariance[0]` (`:976-980`).
    This is what the IMU smoother consumes.
  - `/aft_mapped_to_init_incremental`: same pose stream, kept for consumers
    that want the mapping output directly (e.g. tare's
    `matterport_bagfile.yaml` uses it as its state-estimation input).

## How `/state_estimation` is produced

`imu_preintegration_node` fuses the ~10 Hz optimized poses with the ~200 Hz IMU
in a GTSAM factor graph, then propagates between corrections:

1. **Correction step** (`laserodometryHandler`, `imuPreintegration.cpp:646`):
   on every `/laser_odometry` message, the system initializes on the first one
   (`initial_system`, `:301` — priors, gravity alignment via its own
   `imu_laser_R_Gravity` estimate), then `process_imu_odometry` (`:498`) adds
   an IMU preintegration factor spanning the inter-sweep interval plus a pose
   factor from the laser pose, optimizes, and updates the anchored state and
   IMU biases (`prevStateOdom`, `prevBiasOdom`). Zero-velocity/no-motion
   priors are added when stationary (`:226`, `:247`). If the IMU stream lags
   the laser time by more than `imu_laser_timedelay`, health is flagged bad
   (`:694-742`).
2. **Propagation step** (`imuHandler`, `:851`): every IMU sample after the
   first optimization is integrated (`:948`), and the current state is
   predicted from the last anchored state + biases (`:957`). The IMU-body pose
   is composed with the `imu2Lidar` extrinsic (`:991-993`) so the published
   pose is the **LiDAR** pose. With `use_imu_roll_pitch: true` the roll/pitch
   would be overwritten by the IMU's own attitude (`:975-981`); our go2w
   config leaves it `false`.
3. **Publish** (`:1033-1096`): every **4th** IMU sample (`:1070` — 200 Hz IMU
   → ~50 Hz output): `/state_estimation` (`frame_id = map`,
   `child_frame_id = sensor`, stamp = IMU time), twist = body-frame linear
   velocity + bias-corrected gyro rates, and a TF `map → sensor` with the same
   pose. **The covariance array is not a covariance**: `[0]` = IMU health
   enum, `[1..3]` accel bias, `[4..6]` gyro bias, `[7]` gravity magnitude
   (`:1059-1067`). `/state_estimation_health` (Bool) mirrors the health flag.

So at any instant `/state_estimation` = *latest scan-to-map pose, IMU-propagated
forward*. It converges back onto the mapping pose at every correction; between
corrections it can differ from the (future) optimized pose by the propagation
error of ≤1 sweep interval.

---

## Frames and conventions

- `WORLD_FRAME` = `map`, `SENSOR_FRAME` = `sensor` (set in
  `config/livox_mid360_go2w.yaml`; code default would be `sensor_init`).
- The `map` frame is **created by ARISE at startup**: origin at the initial
  LiDAR pose, yaw zeroed, roll/pitch leveled from the initial IMU attitude
  estimate. It is gravity-aligned exactly as well as `imu_laser_R_Gravity` is
  accurate — a **per-run, per-robot estimate** printed to stdout at init.
- The published pose is the **LiDAR** pose, not a robot-base pose.
- ARISE's TF output is the single dynamic `map → sensor` leg. Its `map` is its
  own tree root — *disconnected* from any TF tree a bag may carry (`world`,
  `odom`, …). Reconciling the two trees is the job of the consumer (see the
  scene-graph exporter's frozen `world_T_map`, and `ARCHITECTURE.md` →
  *Coordinate frames*), and the recurring source of "slightly off" coordinates
  that motivated the bag-direct path.

---

## The downstream contract

Direct subscribers of the pair (verified):

| Consumer | Subscription |
|---|---|
| `terrain_analysis` (`terrainAnalysis.cpp:259-261`), `terrain_analysis_ext` | both topics |
| `local_planner` (`localPlanner.cpp:662-664`) | both topics |
| `sensor_scan_generation` (`sensorScanGeneration.cpp:128-129`) | message-filter sync of both → `/state_estimation_at_scan`, `/sensor_scan` |
| `semantic_mapping` (`semantic_mapping_node.py:193,210`) | both topics |
| `tare_planner` (incl. room segmentation + Representation) | both, via scenario yaml (`sub_registered_scan_topic_`, `sub_state_estimation_topic_`; the bagfile scenario points the latter at `/aft_mapped_to_init_incremental`) |

What they all assume — i.e. what any replacement (such as `bag_slam_bridge`)
must reproduce:

1. **One shared frame.** Both topics are in the *same* `map` frame with the
   *same* body (`sensor`). Consumers freely mix a pose from one topic with a
   cloud from the other.
2. **Pose ∘ raw = registered.** A registered scan is the raw (deskewed) sweep
   transformed by the pose this estimator would report at the scan's stamp.
   `semantic_mapping` literally inverts this: it pulls `/registered_scan` back
   into the body frame using the nearest `/state_estimation` pose
   (`semantic_map_new.py:214-218`), runs image-space segmentation, and lifts
   object points back to world with the same pose. Any registered-scan /
   state-estimation inconsistency becomes object-position error 1:1. (Two
   built-in tolerances exist: the IMU-propagated pose differs slightly from
   the optimized pose used for registration, and deskew is rotation-only.)
3. **Stamp pairing.** `/state_estimation` runs ~5× faster than the scan rate
   and brackets every scan stamp, so nearest-stamp / interpolated lookups are
   always possible. `sensor_scan_generation` syncs the two exactly;
   `semantic_mapping` matches by nearest stamp.
4. **Gravity-aligned, z-up `map`.** Terrain analysis (slope/step costs), the
   local planner, room segmentation's height slicing, and ceiling-height
   filters all assume z is up and the floor is level in `map`. If the initial
   attitude estimate is off, *everything* downstream is tilted — and the
   semantic-mapping `R_GRAVITY` (platform `go2w`) and scene-graph exporter
   `gravity_matrix` are frozen copies of this per-run estimate that must match
   the printed value for the run being replayed.
5. **Start at the origin.** `map` origin/heading = wherever SLAM started.
   All planner grids, viewpoints, and exported coordinates are relative to
   that, not to any external datum.
6. **Single dense sweep per message, ~10 Hz.** Not an accumulated map: each
   message is one sweep (~13k points, intensity present). The planner's
   coverage updates, room segmentation, and 3-D lifting all depend on this
   density and rate.
7. **Robot position = sensor position.** Keyposes, viewpoints, and "robot
   position" everywhere downstream are this LiDAR pose.
8. **Covariance fields are repurposed.** Nothing downstream may interpret
   `pose.covariance` of either odometry topic as an actual covariance; health
   lives in `/state_estimation_health` and `covariance[0]`.

## Relationship to `bag_slam_bridge`

For bags that already contain a good LIO (e.g. the go2w robot bags),
`system_bag_direct.launch` runs `src/slam/bag_slam_bridge` instead of this
package: it republishes the bag's own odometry as `/state_estimation` and
registers the raw Livox stream into the bag's world frame as
`/registered_scan`, honoring the same contract (one frame labelled `map`,
pose ∘ raw = registered, z-up). ARISE remains the path for live robots and
bags without usable odometry, and for A/B comparison.
