# SysNav `rsb_test` — Architecture Guide (Scene-Graph Pipeline)

> **What this document is.** A developer / coding-agent onboarding guide to the
> **scene-graph construction pipeline** as it exists on branch `rsb_test`: the
> original sysnav (`rsb`) producers, driven by a **teleoperated** robot, with the
> TARE exploration planner's steering removed. The root [`README.md`](README.md)
> is the paper landing page; the branch's worklist, verification history and
> gotchas are in [`RSB_TEST_PLAN.md`](RSB_TEST_PLAN.md). Verified against
> `rsb_test` @ `e9504a4` (2026-08-29).

---

## TL;DR mental model

You drive the robot (RViz teleop panel, joystick, or a bag replay). Sensors
(Livox Mid-360 + IMU, a Ricoh Theta Z1 **360° panorama** on `/camera/image`)
feed SLAM, and three producers write into one in-memory structure owned by the
scene-graph node, the **`Representation`**:

1. **Objects** — detected in the panorama, lifted to 3D with the lidar, tracked
   over time (`semantic_mapping`).
2. **Viewpoints + keypose graph** — observation keyframes at real robot poses,
   and a traversability roadmap along the trajectory (the scene-graph node).
3. **Rooms** — free space segmented into rooms and doors (`room_segmentation`),
   rooms typed by a VLM (`vlm_node` + the scene-graph node).

A fourth, independent product is the **BEV occupancy map** (`bev_mapper`).

```
 /lidar/scan + /imu/data ─► ARISE SLAM ─► /registered_scan (map frame), /state_estimation (map→sensor)
                                             │
 /camera/image ─► detection_node ─► semantic_mapping_node ─► /object_nodes_list ─┐
 /registered_scan + odom ─► room_segmentation ─► /room_nodes_list, /room_mask, /door_cloud ─┼─► Representation
 /camera/image + odom ─► scene-graph node: viewpoints, keypose graph, room typing (⇄ vlm_node) ─┘  (in memory, RViz)
 /registered_scan + /state_estimation ─► bev_mapper ─► /bev_map/grid, /bev_map/local, /bev_map/frontiers
```

The scene graph lives in memory in the scene-graph node and is inspected
through RViz (`tare_planner_teleop.rviz`); `Representation::ToJSON()` is a stub.
The one on-disk product is the **SemPathBench map export**: the scene-graph
node and `bev_mapper` dump raw snapshots to `output/sempath_export/` (periodic,
`/keyboard_input "export"`, and at shutdown) and the standalone tool
`tools/sempath_export/` converts them into a SemPathBench (ProcTHOR-style)
layered map under `output/sempath_maps/` — see
[`tools/sempath_export/README.md`](tools/sempath_export/README.md).
**Nothing in this stack steers the robot** — `local_planner` follows the
teleop / waypoint commands only.

---

## Repository layout

| Package (`src/…`) | Role on this branch |
|---|---|
| `exploration_planner/tare_planner` | **The scene-graph node** (`tare_planner_node`, class `SensorCoveragePlanner3D` in `sensor_coverage_planner/sensor_coverage_planner_ground.cpp` — TARE's planner with the steering outputs deleted) + `representation` (the scene graph) + `room_segmentation` (separate executable) + `keypose_graph`, `grid_world`, `viewpoint_manager`, `planning_env` (kept: they feed viewpoints, the keypose graph and `/occupied_cloud` + `/freespace_cloud` for room segmentation). Msgs, scenario yamls, the bringup launches. |
| `semantic_mapping` | 3D semantic **objects**: YOLOE + BoT-SORT detection, SAM2 masks, lidar–image fusion, persistent per-instance clouds → `/object_nodes_list`. [README](src/semantic_mapping/README.md). |
| `vlm_node` | VLM Q&A: **room typing** (`/room_type_query` → `/room_type_answer`) and object-label verification for `semantic_mapping` (`/object_type_query` → `/object_type_answer`). Optional `keyboard_input` terminal. |
| `slam` | **ARISE SLAM** (`arise_slam_mid360`): `feature_extraction_node` → `laser_mapping_node` → `imu_preintegration_node`. [README](src/slam/arise_slam_mid360/README.md). |
| `base_autonomy` | sysnav's teleop base, verbatim: `vehicle_simulator` (Unity sim + the `system_*.launch` bringups), `local_planner` (+ `pathFollower` → `/cmd_vel`), `terrain_analysis(_ext)`, `sensor_scan_generation` (`/state_estimation_at_scan`), `visualization_tools`. |
| `bev_mapper` | BEV occupancy map from `/registered_scan` + `/state_estimation`. [README](src/bev_mapper/README.md). |
| `utilities` | `livox_ros_driver2` (lidar), `receive_theta` (panorama camera driver), `ROS-TCP-Endpoint` (Unity), RViz teleop / waypoint / goalpoint plugins, `teleop_joy_controller`, `rviz_2d_overlay_plugins`, `serial`. |

---

## Bringup

Both entry points are root scripts that `cd` to the workspace root (several
nodes resolve `src/…` and `output/…` paths relative to the cwd), source
`install/setup.bash`, and run one launch file in `tare_planner/launch/`.

| | Unity sim | Real robot / bag |
|---|---|---|
| script | `./system_simulation_teleop.sh` (starts the Unity binary, then the launch) | `./system_real_robot_teleop.sh` (traps Ctrl-C and SIGTERMs the ARISE nodes, which ignore SIGINT) |
| launch | `scene_graph_sim.launch` | `scene_graph_real_robot.launch` |
| base autonomy include | `vehicle_simulator/system_simulation.launch` | `system_real_robot.launch` (lidar driver + ARISE) or, with `bagfile:=true`, `system_bagfile.launch` (no driver) |
| scene-graph scenario | `matterport_sim` | `matterport_real` / `matterport_bagfile` |
| semantic_mapping | `semantic_mapping_sim.launch` (platform `mecanum_sim`) | `semantic_mapping_real.launch` (`mecanum`) / `semantic_mapping_bagfile.launch` (`mecanum_bagfile`) |
| common args | `objects:=true` (detection + semantic_mapping), `rviz:=true`, `keyboard:=false`, `bev:=true`, `use_sim_time:=false` | same + `bagfile:=false`, sysnav's `sensorOffsetX/Y`, `cameraOffsetZ`, `vehicleX/Y`, `checkTerrainConn` |

Notes:
- The panorama driver (`receive_theta`, `receive_theta_sensorpod.launch`) is
  **not** part of the real-robot launch — start it separately, as in sysnav.
- `use_sim_time` stays `false` everywhere; play bags **without** `--clock`, and
  in `bagfile:=true` mode play only the raw topics
  (`--topics /lidar/scan /imu/data /camera/image`) so the recorded SLAM output
  does not fight the live ARISE (duplicate publishers = 2× scan rate, dead
  odometry).
- Driving: RViz teleop panel or joystick → `/joy` → `local_planner` →
  `/cmd_vel` (`geometry_msgs/TwistStamped`, consumed by the base's own motor
  driver, outside this repo); RViz waypoint tool → `/way_point` (needs the
  autonomy bit on `/joy`, `local_planner` runs with `autonomyMode=false`).
- Build: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
  (non-symlink builds lose the gitignored `.engine` weights; a non-Release
  planner cannot keep up).

---

## Input contract and frames

Everything lives in one gravity-aligned **`map` frame** (`kWorldFrameID = "map"`
in the scene-graph node and the `Representation`), anchored at the robot's start
pose. There is no `map`/`odom` split: room centroids, object positions,
viewpoints, doors and keypose nodes are directly comparable.

| Topic | Type | Producer (real) | Producer (sim) | Consumers |
|---|---|---|---|---|
| `/registered_scan` | PointCloud2, frame `map`, 10 Hz | ARISE `laser_mapping_node` | Unity via ROS-TCP-Endpoint | scene-graph node, room_segmentation, semantic_mapping, bev_mapper, base_autonomy |
| `/state_estimation` | Odometry `map`→`sensor`, 50 Hz | ARISE `imu_preintegration_node` | `vehicleSimulator` | room_segmentation, semantic_mapping (`mecanum`, `mecanum_sim`), bev_mapper, local_planner |
| `/state_estimation_at_scan` | Odometry, 10 Hz (pose at the scan stamp) | `sensor_scan_generation` | same | scene-graph node in **sim** (`matterport_sim.yaml`) |
| `/aft_mapped_to_init_incremental` | Odometry `map`→`laser`, 10 Hz | ARISE | — | scene-graph node on **real/bagfile** (`matterport_real/bagfile.yaml`), semantic_mapping (`mecanum_bagfile`) |
| `/camera/image` | Image 1920×640 rgb8, ~10 Hz, **equirectangular panorama** | `receive_theta` | Unity `sim_image_repub` | detection_node, semantic_mapping, scene-graph node (room-type crops) |
| TF `map→sensor` (+ `sensor→vehicle`, `map→sensor_at_scan`) | | ARISE + static publishers in the base launches | `vehicleSimulator` | RViz, local_planner |

The camera model is hard-coded per `platform` in
`semantic_mapping/cloud_image_fusion.py` (`CAMERA_PARA`, panorama offsets
relative to the lidar) and in the scene-graph node's `project_pcl_to_image`;
a pinhole front camera is **not** supported (room-type crops fall back to the
full frame with a warning; object fusion assumes the panorama).

---

## The scene graph data model (`Representation`)

`src/exploration_planner/tare_planner/src/representation/` — owned by the
scene-graph node as `representation_`. Details and per-field writers:
[`representation/README.md`](src/exploration_planner/tare_planner/src/representation/README.md).

| Node | Type | Key fields | Meaning |
|---|---|---|---|
| **Room** | `RoomNodeRep` | `id_`, `centroid_`, `polygon_`, `neighbors_`, `viewpoint_indices_`, `object_indices_`; labeling state `labels_` (coverage-weighted **vote map**, `GetRoomLabel()` = argmax), `is_labeled_` (= *asked*), `anchor_point_`, `image_`, `room_mask_`, `voxel_num_`, `last_area_` | A segmented region of free space; owns the viewpoints and objects inside it; `neighbors_` are door-connected rooms. |
| **Object** | `ObjectNodeRep` | `object_id_`, `label_`, `position_`, `bbox3d_`, `cloud_`, `room_id_`, `visible_viewpoint_indices_` | A detected, 3D-localized, persistently tracked object instance. |
| **Viewpoint** | `ViewPointRep` | `id_` (== index), `position_`, `timestamp_`, `cloud_`, `covered_cloud_`, `object_indices_`, `direct_object_indices_`, `room_id_` | An **observation keyframe** at a real robot pose where the camera saw objects. Not a free-space waypoint. |
| **Door** | `door_cloud_` (`PointXYZRGBL`) | `r`/`g` = the two room ids it joins; `x/y/z` = position | The opening between two rooms; fills the room adjacency matrix. |

Relations: Room ↔ Viewpoint / Object membership by voxelizing positions into
the room mask (`UpdateViewpointRoomIdsFromMask`, `SetObjectRoomRelation`);
Viewpoint ↔ Object visibility (`UpdateObjectNode`, `UpdateObjectVisibility`);
Room ↔ Room adjacency via doors.

---

## Construction pipeline (how the graph fills up)

**1. Objects — `semantic_mapping`**
```
/camera/image ─► detection_node (YOLOE + BoT-SORT) ─► /detection_result
/detection_result + odom + /registered_scan ─► semantic_mapping_node
      └─ SAM2 mask ─ lidar–image fusion ─ ObjMapper (merge / grow / prune)
      └─ /object_nodes_list ──────────────────────► Representation::UpdateObjectNode
      └─ /object_type_query ⇄ /object_type_answer (vlm_node label verification)
```
[`semantic_mapping/README.md`](src/semantic_mapping/README.md). Debug images
land in `output/viewpoint_images/`, `output/object_images/` (cwd-relative).

**2. Viewpoints — the scene-graph node**
`UpdateViewpointRep` commits a viewpoint at the robot's real pose when coverage
overlap drops or object interest is high (`obj_score_`), deduplicated at 2 m
(`AddViewPointRep`). On commit it publishes `viewpoint_rep_header`;
`semantic_mapping_node` then saves the closest camera frame for that viewpoint.

**3. Rooms — `room_segmentation` + room typing**
```
/registered_scan + odom + /occupied_cloud + /freespace_cloud ─► room_segmentation
      └─ /room_nodes_list ─► RoomNodeListCallback (AddRoomNode / update / erase; anchor STRIP)
      └─ /room_mask ───────► RoomMaskCallback → UpdateViewpointRoomIdsFromMask
      └─ /door_cloud ──────► DoorCloudCallback → door_cloud_, adjacency
UpdateRoomLabel ─► /room_type_query (RoomType: panorama crop + mask) ─► vlm_node
vlm_node ─► /room_type_answer ─► RoomTypeCallback → labels_[type] += voxel_num
```
`/occupied_cloud` and `/freespace_cloud` come from the scene-graph node
(`planning_env` / `viewpoint_manager`) — the reason those TARE modules survive.
Geometry: [`room_segmentation/README.md`](src/exploration_planner/tare_planner/src/room_segmentation/README.md).
Typing: [`sensor_coverage_planner/ROOM_LABELING.md`](src/exploration_planner/tare_planner/src/sensor_coverage_planner/ROOM_LABELING.md).

**4. Keypose graph — the scene-graph node (leaf)**
A roadmap over explored free space: keypose nodes every 5th registered scan
(`RegisteredScanCallback` → `AddKeyposeNode`), connector nodes from
`grid_world::AddPathsInBetweenCells`, edges = collision-free straight segments,
healed and connectivity-checked each cycle (`UpdateKeyposeGraph`). On this
branch nothing consumes it beyond `grid_world`'s cell bookkeeping and RViz; it
is kept as the base for a future navigation layer.
[`keypose_graph/README.md`](src/exploration_planner/tare_planner/src/keypose_graph/README.md).

**5. BEV map — `bev_mapper` (independent)**
2-D occupancy / explored / trajectory grid (67.2 m @ 0.05 m, centred on the
start pose) from `/registered_scan` + `/state_estimation`; publishes
`/bev_map/grid` (OccupancyGrid), `/bev_map/local` (448×448 image),
`/bev_map/frontiers`. Sets occupancy but never clears it (a person walking with
the robot leaves marks). [`bev_mapper/README.md`](src/bev_mapper/README.md).

The scene-graph node's cycle (`execute()`, on each keypose-cloud update):
`ProcessObjectNodes` → `UpdateRoomLabel` → `SetCurrentRoomId` →
`UpdateObjectVisibility` → `UpdateGlobalRepresentation` → `UpdateViewpointRep`
→ viz markers → `UpdateViewPoints` → `UpdateKeyposeGraph` → connector
injection (`GlobalPlanning`, TSP removed).

---

## Inspecting the scene graph

RViz (`tare_planner_teleop.rviz`, sysnav's `tare_planner_ground.rviz` minus the
planner-only displays):

| What | Topic |
|---|---|
| room ids + VLM labels at centroids | `/room_type_vis` |
| room mask / boundaries / walls / room cloud | `/room_mask_vis`, `/room_boundaries`, `/current_room_boundary`, `/walls`, `/room_cloud`, `/room_map_cloud` |
| doors | `/door_cloud_vis`, `/door_cloud_in_range` |
| objects (clouds, boxes, labels, nodes) | `/obj_points`, `/obj_boxes`, `/obj_labels`, `/object_node_markers`, `/annotated_image` |
| viewpoints, viewpoint→object visibility | `/viewpoint_rep_vis_cloud`, `/viewpoint_room_ids`, `/object_visibility_connections` |
| keypose graph | `/keypose_graph_cloud`, `/keypose_graph_edge_marker` |
| BEV map | `/bev_map/grid`, `/bev_map/local`, `/bev_map/frontiers` |

On disk: `output/viewpoint_images/`, `output/object_images/` (semantic_mapping),
`debug/room_type/<room_id>_<type>.jpg|_mask.jpg|.txt` (vlm_node) — all relative
to the cwd, i.e. the workspace root when started by the scripts. Verbose
scene-graph logging: `#define ROOM_DBG_ENABLED` at the top of the node's cpp
(`[room_dbg] CREATE / DEATH / MASK_UPDATE / QUERY_PUB / ANSWER_APPLY`).

Snapshot export: `output/sempath_export/{scene_graph_latest.json, room_mask_latest.png,
bev_latest.npz}` (params `export.*` in the scenario yamls / `bev_mapper.yaml`), converted by
`python3 -m tools.sempath_export.transform_sysnav_to_map` into a SemPathBench map bundle
(`output/sempath_maps/<split>/<id>/`). The deepclean-era GADM exporter was removed with its
NavGraph dependency; see `RSB_TEST_PLAN.md` for the history.

---

## Leftovers worth knowing

- `tare_planner/launch/scene_graph.launch` + `config/go2w_bag_direct.yaml` are
  deepclean's bag-direct bringup (Go2-W, LIO from the bag); not used by either
  script. `explore_world*.launch` / `explore_tunnel.launch` are sysnav's
  originals with the removed `navigationBoundary` node stripped.
- `matterport_real/bagfile.yaml` point the node at `/room_boundary`, which
  nothing publishes (planner-only input; harmless).
- Config knobs live in three places: scenario yaml (`config/matterport_*.yaml`
  — topics, thresholds, `isDebug`), `config/robot.yaml`, and the hard-coded
  camera model per `platform` in `semantic_mapping`.

---

## Where to look next

| If you're working on… | Go to |
|---|---|
| Bringup, phases, verification history, gotchas | [`RSB_TEST_PLAN.md`](RSB_TEST_PLAN.md) |
| Rooms / doors / the room mask (geometry) | [`room_segmentation/README.md`](src/exploration_planner/tare_planner/src/room_segmentation/README.md) |
| Room *type* labeling (crop → VLM → vote) | [`sensor_coverage_planner/ROOM_LABELING.md`](src/exploration_planner/tare_planner/src/sensor_coverage_planner/ROOM_LABELING.md), `vlm_node/` |
| 3D objects / detection | [`semantic_mapping/README.md`](src/semantic_mapping/README.md) |
| The in-memory scene graph itself | [`representation/README.md`](src/exploration_planner/tare_planner/src/representation/README.md) |
| Traversability roadmap | [`keypose_graph/README.md`](src/exploration_planner/tare_planner/src/keypose_graph/README.md) |
| SLAM contract, topics, config | [`slam/arise_slam_mid360/README.md`](src/slam/arise_slam_mid360/README.md) |
| BEV map | [`bev_mapper/README.md`](src/bev_mapper/README.md) |
| SemPathBench map export (dumps + converter) | [`tools/sempath_export/README.md`](tools/sempath_export/README.md) |
| The node that owns & wires it all | `src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` |
| Why things were removed (history) | [`EXTRACTION_AUDIT.md`](EXTRACTION_AUDIT.md) (deepclean-era, superseded) |
