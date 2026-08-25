# rsb_test — Implementation Plan

> **Goal.** A teleoperated-robot scene-graph pipeline that produces **room nodes,
> object nodes, viewpoints, doors and VLM room labels identical to sysnav
> (`rsb`)**, on a **360° panorama camera**, with **no motion planning** (no TARE
> steering), and **no NavGraph / JSON exporter / quadrant / docker / ros1_bridge**.
> Runs in the Unity simulator (teleop) and on the real robot (teleop) with the
> **ARISE SLAM (Livox Mid-360) restored** as the `/state_estimation` +
> `/registered_scan` source. A waypoint producing node comes later and is out of
> scope here.
>
> **Base.** `rsb_test` = `deepclean` + a `.gitignore` line (`9c53269`).
> `deepclean` = `rsb`'s scene-graph core with steering / object-search / nav Q&A /
> `base_autonomy` / `route_planner` / `slam` (ARISE) removed (verified
> bag-over-bag), plus
> additions we do NOT want (NavGraph, exporter, quadrant, best-3 pinhole room
> labeling, docker, ros1_bridge, direct-LIO input). This plan strips the
> additions and restores the sysnav producers verbatim.
>
> **Why this base and not `rsb`.** The deletion work is the expensive, risky part
> (multi-day surgery on a 6k-line monolith with non-obvious dependencies —
> `/freespace_cloud` routes through `viewpoint_manager`, `start_time_` feeds its
> warm-up, link-order breakage when shared libs went away). Everything we want
> gone from `deepclean` is an additive leaf module. See `EXTRACTION_AUDIT.md`
> for the original keep/kill ledger.

---

## Ground truth: what differs between `rsb` and `deepclean` in the producers

Diffed 2026-08-25. The algorithms are the same code; the deltas are short.

### Room nodes — `room_segmentation.cpp`
| Change on `deepclean` | Affects room output? | Action |
|---|---|---|
| `cloud_pose_lag_dist` freshness gate (`cloudPoseStale()`, skips wall-erosion ops when the cloud lags the pose > 0.3 m) | **Yes** (real-robot wall-leak fix) | Decision D1 below |
| `interior_point` (pole of inaccessibility) added to `RoomNode.msg` | No (additive) | Dropped with the `rsb` checkout |
| `/wall_axis` publisher + `wall_axis/*` params | No | Delete (Phase 1) |
| Topic names composed from `robot_namespace` + `topic_suffix.*` | No (empty namespace = `rsb` names) | Keep, harmless |
| `[wall_dbg]` / `[room_pia]` logging | No | Goes with the checkout |

Watershed, `updateRooms`, door extraction, plane tracking, freespace/occupied erosion: unchanged.

### Object nodes — `semantic_mapping`
| Change on `deepclean` | Affects object output? | Action |
|---|---|---|
| Detector `yolov8x-worldv2_cus.engine` (YOLO-World) → `yoloe-26x-seg.engine` | **Yes** | Revert |
| `ObjMapper.confidence_thres` 0.30 → 0.07 | **Yes** | Revert |
| Cross-class merge branch enabled (`rsb` had it commented out) | **Yes** | Revert |
| `generate_seg_cloud(..., image)` → dumps a debug PNG every frame | No (disk/CPU hazard) | Revert |
| `get_dominant_label` vs `get_dominant_label()` fix in the delete rule | Only with a `target_object` set | Irrelevant |
| `objects.yaml` `box` enabled; default `objects_office.yaml`; go2w configs; `camera_info`/tf calibration (only for `platform: go2w_bag`) | Config / inactive for panorama platforms | Goes with the checkout |

`single_object_new.py`: unchanged. Panorama projections `scan2pixels_mecanum`, `_mecanum_sim`, `_wheelchair`: unchanged.

### Planner ingestion (rooms/objects → `Representation`)
`ObjectNodeListCallback`, `UpdateObjectVisibility`, `UpdateViewpointObjectVisibility`, `UpdateViewpointRep`, `RegisteredScanCallback`, `DoorCloudCallback`, `RoomMaskCallback`: identical modulo removed object-search bookkeeping and log lines. `SetCurrentRoomId`: nav shortcuts removed, empty-mask bounds guard added (crash fix). `RoomNodeListCallback`: `rsb`'s anchor-STRIP removed, best-3 orphan re-home added (both handled in Phase 2).

### Room labeling (the one real transplant)
| | `rsb` (sysnav) | `deepclean` |
|---|---|---|
| Trigger | `UpdateRoomLabel`: covered cloud binned into rooms; first coverage, or +20 voxels / +5 m² since last query | `UpdateRoomViews` + `PublishRoomTypeQueries`: best-3 view buffer dirty or object count changed, 3 s rate limit |
| Evidence | one **panorama crop** (`project_pcl_to_image`, 1920 px wide) + room mask, embedded in `RoomType.image` | ≤3 pinhole jpgs by path + object inventory string |
| Answer apply | re-resolve room by sampling `room_mask_` at `anchor_point`; `labels[type] += voxel_num` (vote) | apply by `room_id`, `interior_point` fallback; latest answer wins |
| Lifecycle | per-cycle anchor STRIP clears labels when the anchor samples a foreign id | STRIP removed; orphan views re-homed by mask |
| VLM prompt | closed candidate list, single image | open vocab, ≤3 images, label-stability hint |

`project_pcl_to_image` still exists on `deepclean` as dead code.

---

## Phase 0 — Baseline (½ day)

- [ ] `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` on `rsb_test` as-is.
      **Never** `--cmake-clean-cache` without re-passing the build type — it drops
      to -O0 and the planner starves (no freespace / keypose clouds for minutes;
      looks like a regression, isn't).
- [ ] Run once on the reference bag (`output/recordings/20260628_232101`,
      `objects:=true`). Keep `snapshot_final.json` + the run log as the
      pre-change reference (the exporter is deleted in Phase 1, so this is the
      last chance to get one).
- [ ] Run `rsb`'s `./system_simulation_with_exploration_planner.sh` once to
      confirm the Unity sim (`src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64`,
      untracked, present on disk) and its 1920×640 panorama still work here.

## Phase 1 — Strip the `deepclean`-only additions (1 day)

Pure deletion of leaf modules; nothing here touches room/object output.

| Delete | Then unwire in |
|---|---|
| `src/navgraph/`, `include/navgraph/` (incl. `building_axes.h`) | `sensor_coverage_planner_ground.{cpp,h}`: `navgraph_`, the `execute()` block (`navgraph_room_keys` … `navgraph_->Update`), `navigation_graph/*` params |
| `src/scene_graph_exporter/`, `include/scene_graph_exporter/`, `config/scene_graph_export.yaml` | `SaveSceneGraphSnapshot`, `SceneGraphWatchdogCallback`, `TryFreezeWorldFromOdom`, `scene_graph_cfg_`, both timers, the `manual_save_keyword` branch of `KeyboardInputCallback`; `LogRoomTypeQuery/Answer` (optional keep) |
| `src/quadrant_manager/`, `include/quadrant_manager/`, `msg/WallAxis.msg` | `quadrant_mgr_` + `quadrant/*` params; `room_segmentation.{cpp,h}`: `publishWallAxis`, `pub_wall_axis_`, `wall_axis/*` params (moot after Phase 3 checkout, but needed to compile Phase 1) |
| `docker/`, `vlm_ros_alphaz_bag_direct.tmuxp.yaml`, `src/utilities/domain_bridge` | `launch/scene_graph.launch` (keep as bag-direct bringup or drop in Phase 4), `EXTRACTION_AUDIT.md` (mark superseded by this file) |
| CMake: `add_library(navgraph|scene_graph_exporter|quadrant_manager)` + their link lines; `WallAxis.msg` in `rosidl_generate_interfaces` | `tare_planner_ground.rviz`: navgraph/quadrant displays; `config/go2w_bag_direct.yaml`: `navigation_graph/`, `quadrant/`, `wall_axis/` blocks; stray mentions in `viewpoint_manager.cpp` |

Keep for now: `grid_world` + `GlobalPlanning()` (harmless; its only consumer was
the NavGraph — optional removal later), the keypose graph (cheap; the future
waypoint node will want a traversability roadmap), `viewpoint_manager` /
`planning_env` (feed `/freespace_cloud` and keypose-graph collision healing).

**Verify:** Release build; bag run shows `/room_nodes_list`, `/object_nodes_list`,
`/room_type_query` at the same cadence as Phase 0.

## Phase 2 — Room labeling back to sysnav (1–2 days)

1. `msg/RoomType.msg` ← `rsb` (`sensor_msgs/Image image` instead of
   `string[] image_paths`; no `interior_point`). `msg/RoomNode.msg` ← `rsb`.
2. `include/representation/representation.h` + `src/representation/representation.cpp`
   ← `rsb` verbatim (removes `RoomView`, `best_views_`, `views_dirty_`,
   `last_query_time_`, `objects_at_last_query_`, `interior_point_`; restores
   `image_`, `anchor_point_`, `last_area_`, `voxel_num_` semantics).
3. Monolith `sensor_coverage_planner_ground.{cpp,h}`:
   - **Delete** `UpdateRoomViews`, `PointToCameraView`, `TryCalibrateCamera`,
     `CameraInfoCallback` + `camera_info_sub_` / `camera_tf_buffer_` /
     `cam_*` members, `PublishRoomTypeQueries`, the `room_view.*` /
     `room_type_query.*` params, `room_views_dir_`, the orphan-view re-home
     block in `RoomNodeListCallback`, the `interior_point` fallback in
     `RoomTypeCallback`, `DbgSampleMask`, `ROOM_DBG` lines that reference
     removed fields.
   - **Port from `rsb`**: `UpdateRoomLabel` (minus its three
     `ChangeRoomQuery` early-stop lines and the `ask_vlm_change_room_` tail),
     `RoomTypeCallback` (`labels[type] += voxel_num`), `room_cloud_pub_`, and
     — per decision D2 — the anchor-STRIP block in `RoomNodeListCallback`.
   - Call `UpdateRoomLabel()` where `PublishRoomTypeQueries()` sits in
     `execute()`. `project_pcl_to_image` becomes live again.
     `camera_image_` init back to `cv::Mat::zeros(640, 1920, CV_8UC3)`.
4. `vlm_node/vlm_reasoning_node.py`: keep `deepclean`'s trimmed node (nav Q&A
   already gone) but replace `process_room_type_query` with `rsb`'s (single
   embedded `msg.image` + mask, closed candidate list `self.room_types`). Do
   not take `rsb`'s 1190-line file wholesale. Bring back `vlm_sim_config.yaml`
   if `vlm_node_sim.launch` references it.

**Verify:** compiles; on the sim (Phase 4) or a panorama bag, rooms emit
`/room_type_query` with a crop image and labels accumulate.

## Phase 3 — Producers back to sysnav, verbatim (½ day)

```bash
git checkout rsb -- src/semantic_mapping                                   # whole package
git checkout rsb -- src/exploration_planner/tare_planner/src/room_segmentation \
                    src/exploration_planner/tare_planner/include/room_segmentation
```

Seams to fix afterwards:
- Planner subscriptions must resolve to `/camera/image`, `/state_estimation`
  (`/state_estimation_at_scan` in sim), `/registered_scan` with an empty
  `robot_namespace` (already the fallback; scenario yaml sets the sim odom topic).
- `room_segmentation` params in the scenario yaml: `rsb`'s node reads no
  `cloud_pose_lag_dist` / `wall_axis/*` — remove or re-add per D1.
- Decision D1: re-apply the `cloud_pose_lag_dist` gate as ONE cherry-pick
  (`cloudPoseStale()`, `odom_buf_`, `latest_cloud_stamp_sec_`, the three call
  sites in `occupiedCloudCallback` / `freespaceCloudCallback` / `getWall`,
  the param).

**Verify:** `git diff rsb -- src/semantic_mapping src/exploration_planner/tare_planner/{src,include}/room_segmentation src/exploration_planner/tare_planner/{src,include}/representation`
is empty except the optional gate.

## Phase 4 — Sim + teleop bringup (1 day)

1. `git checkout rsb -- src/base_autonomy` (verbatim, ~30 MB / 50 files, zero
   coupling to the scene graph; builds standalone). `ros_tcp_endpoint`,
   `teleop_rviz_plugin`, `waypoint_rviz_plugin`, `teleop_joy_controller` are
   already in `src/utilities/`.
2. New `tare_planner/launch/scene_graph_sim.launch` =
   `rsb`'s `system_simulation.launch` (vehicle_simulator, sim_image_repub,
   sensor_scan_generation, terrain_analysis + _ext, local_planner with
   `autonomyMode:=false`, joy, ros_tcp_endpoint, visualization_tools)
   **+** `detection_node.launch`, `semantic_mapping_sim.launch`
   (`platform: mecanum_sim`), `room_segmentation.launch scenario:=matterport_sim`,
   `vlm_node_sim.launch`, `keyboard_input.launch`, and the scene-graph node via
   `explore.launch scenario:=matterport_sim` (that yaml already uses
   `/state_estimation_at_scan` + `/registered_scan`).
3. `system_simulation_teleop.sh` = Unity binary → the launch → RViz, with the
   teleop and waypoint panels in `tare_planner_ground.rviz`.
   Driving paths: RViz teleop panel / joystick → `/joy` → `local_planner` →
   `/cmd_vel`; click-waypoint → `/way_point` → `local_planner` (also the
   interface the future waypoint node will publish to);
   `teleop_joy_controller` → `/cmd_vel` directly.
4. Real-robot bringup is Phase 4b (needs SLAM back first).

**Verify:** teleop around the Unity office: rooms segment, objects appear,
panorama crops reach the VLM, labels land, viewpoints drop.

## Phase 4b — SLAM back + real-robot teleop bringup (1 day + build time)

The sim needs no SLAM (`vehicleSimulator` publishes ground-truth
`/state_estimation`, `sensor_scan_generation` derives the rest). The real
robot (mecanum / wheelchair, Livox Mid-360, 360° camera) needs ARISE back.

**What ARISE is, for our purposes.** Three nodes (`feature_extraction_node`
→ `laser_mapping_node` → `imu_preintegration_node`) consuming
`/livox/lidar` (CustomMsg) + `/livox/imu`, producing exactly the pair every
downstream consumer reads: `/registered_scan` (deskewed sweep in `map`,
~10 Hz) and `/state_estimation` (IMU-rate smoothed pose in `map`, ~50 Hz),
plus TF `map → sensor`. Frame convention matches what the planner /
room_segmentation / semantic_mapping already assume (`kWorldFrameID = "map"`).

1. `git checkout rsb -- src/slam` — `arise_slam_mid360`, `arise_slam_mid360_msgs`,
   and the vendored `dependency/{gtsam,ceres-solver,Sophus}` (82.7 MB tracked /
   5799 files; each is a colcon package, so no system installs). Depends on
   `livox_ros_driver2` (still in `src/utilities/`, do **not** prune it) and
   `pcl_conversions`.
2. Restore the ARISE developer README that only ever lived on `deepclean`
   history: `git show fb6ee81^:src/slam/arise_slam_mid360/README.md > src/slam/arise_slam_mid360/README.md`
   (the topic contract, deskew, ICP, IMU smoother — worth keeping).
3. Build once with `colcon build --packages-up-to arise_slam_mid360 --cmake-args -DCMAKE_BUILD_TYPE=Release`.
   gtsam + ceres are the long pole (tens of minutes; ~1.2 GB of build
   artifacts — this is the "1.2 GB deps" the removal commit mentions).
4. Launch wiring already comes with the Phase 4 `base_autonomy` checkout:
   `vehicle_simulator/launch/system_real_robot.launch` includes
   `arise_slam_mid360/launch/arize_slam.launch.py` and the Livox driver
   (`livox_ros_driver2 … msg_MID360_launch.py`); `system_bagfile.launch`
   includes ARISE too (for raw-Livox bags). `system_real_robot_teleop.sh` +
   `scene_graph_real_robot.launch` = `system_real_robot.launch` (SLAM, livox
   driver, terrain, local planner `autonomyMode:=false`, joy, visualization)
   **+** the scene-graph nodes with `platform: mecanum` / `wheelchair` and the
   real camera on `/camera/image`.
5. Config: `arize_slam.launch.py` defaults to `config/livox_mid360_2.yaml` +
   `config/livox/livox_mid360_calibration.yaml` (LiDAR↔IMU extrinsic). Pick /
   verify the yaml for the actual robot; `config/livox_mid360_go2w.yaml` also
   exists.

**Verify:** on the robot (or a raw-Livox bag via `system_bagfile.launch`):
`/state_estimation` ~50 Hz and `/registered_scan` ~10 Hz in frame `map`,
`/state_estimation_health` OK, TF `map → sensor` present; then the scene-graph
nodes see the same pair as in sim.

## Phase 5 — Identity check against sysnav (1 day)

1. Record one sim teleop session as a ROS 2 bag: `/state_estimation`,
   `/state_estimation_at_scan`, `/registered_scan`, `/camera/image`,
   `/terrain_map`, `/tf`, `/tf_static`, `/clock`.
2. Play it into **both** `rsb` (`system_bagfile_with_exploration_planner.sh` —
   TARE cannot move a bag, so it is inert) and `rsb_test`.
3. Compare structurally per cycle: room count / ids / areas / neighbors;
   object count / labels / positions; final room labels. Tolerate VLM answer
   and tracker-id variance; everything geometric must match.
   (Run-to-run nondeterminism exists regardless: VLM, callback timing,
   tracker ids. Identity is at the algorithm level.)

## Phase 6 — Tidy (½ day, optional)

- Rename `SensorCoveragePlanner3D` → `SceneGraphNode` (executable
  `tare_planner_node` → `scene_graph_node`).
- Drop dead params (`kAutoStart`, `kRushHome`, `pub_waypoint_topic_`,
  `sub_terrain_map_topic_`, …), `TargetObjectInstruction.msg` if unreferenced.
- Prune `src/utilities/` (receive_theta, serial; **keep** livox_ros_driver2
  — ARISE depends on it — plus ROS-TCP-Endpoint, rviz plugins,
  teleop_joy_controller).
- Rewrite `ARCHITECTURE.md` / package READMEs: remove exporter / NavGraph /
  quadrant / best-3 references; document the sim + teleop bringup.
- Later, if wanted: the audit's R1/R2 rewiring to drop `viewpoint_manager` /
  `grid_world` entirely (freespace from the rolling occupancy grid; keypose
  healing against the grid). Not needed for function.

---

## Decisions (confirm before Phase 2/3)

| # | Question | Default assumed here |
|---|---|---|
| D1 | Re-apply the `cloud_pose_lag_dist` gate onto `rsb`'s `room_segmentation.cpp`? | **Yes**, default 0.3 m (no-op in sim, protects hardware) |
| D2 | Answer apply: `rsb`'s anchor-mask re-resolve + STRIP verbatim (identical; known to drop / wipe labels on transient mask flicker) vs `deepclean`'s apply-by-`room_id`? | **Verbatim** (identity first; the fix is a one-function swap later) |
| D3 | `semantic_mapping`: whole-package checkout from `rsb` vs keep `deepclean`'s node with four line-reverts? | **Whole package** (provably identical) |
| D4 | Keep `grid_world` / keypose graph? | **Keep** (cheap; roadmap for the future waypoint node) |
| D5 | ARISE config for the target robot (`livox_mid360_2.yaml` vs `livox_mid360_go2w.yaml` vs a new one) + LiDAR↔IMU calibration yaml | **`livox_mid360_2.yaml`** (sysnav's real-robot default) until the robot is confirmed |

## Known gotchas

- Release build type (see Phase 0).
- **Always build with `--symlink-install`** (the workspace convention). The
  `tare_planner` CMake installs only the two executables — its shared libs
  (`librepresentation.so`, `libmisc_utils.so`, `libplanning_env.so`, …) stay in
  `build/` and the installed exe has no RUNPATH — and `semantic_mapping`'s
  `setup.py` does not package the `.engine`/`.pt` weights. A plain
  `colcon build` therefore yields exit-127 C++ nodes and a `FileNotFoundError`
  for `yoloe-26x-seg.engine`. Full form:
  `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- `semantic_mapping_node` **wipes** `output/object_images`, `output/viewpoint_images`
  and `output/debug_mapper` at startup (`annotate_image: true`). Copy anything
  you want to keep from a run before starting the next one.
- `rsb`'s `RoomTypeCallback` samples `room_mask_` at `anchor_point` without a
  `room_mask_.empty()` guard; keep `deepclean`'s bounds checks when porting.
- `rsb`'s `execute()` returned early on zero candidate viewpoints, freezing the
  scene graph when the robot idles — `deepclean`'s no-freeze version stays.
- Sim camera: 1920×640, 360° hfov / 120° vfov, decoded from
  `/camera/image/compressed` by `sim_image_repub`.
- `vehicleSimulator` subscribes `/terrain_map`; `terrain_analysis` is part of
  the sim, not optional.
- `deepclean`'s `ObjectNodeRep::confidence_` is never set (0.0) — irrelevant
  once `PublishRoomTypeQueries` is gone (Phase 2), but do not reintroduce a
  confidence filter on it.
- ARISE estimates `imu_laser_R_Gravity` **once at startup while the robot is
  static** — keep the robot still for the first seconds after launch.
- `arize_slam.launch.py` hard-sets `use_sim_time=false` via `SetParameter`
  and defaults `map_dir` to `~/Desktop/pointcloud_local.txt`; revisit both
  before bag replay / when moving the map dump somewhere sane.
- ARISE deskew is rotation-only (no intra-sweep translation compensation);
  the scan-to-map ICP absorbs the rest. This is the behaviour sysnav had —
  fine for identity, just don't expect the `bag_slam_bridge`-era refinements
  (those were deleted with `deepclean`'s slam removal and are not needed).
- Do not run ARISE on a bag that already carries its own LIO (go2w bags):
  two independent estimators = two drifting `map` frames. Use ARISE only for
  raw-Livox inputs (real robot, raw-Livox bags).
