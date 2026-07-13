# EXTRACTION_AUDIT — scene-graph-only pipeline

> **What this is.** The dependency audit for extracting a **self-contained
> scene-graph pipeline** out of SysNav into a new repo. Decided context: the
> robot runs its **own** TARE planner for steering (ROS1, untouched); our stack
> must be fully decoupled from it, consume **only standard sensor topics**, and
> not duplicate a full planner. This document says, for every node / module /
> callback / publisher / method of the current stack: **KEEP**, **TRIM**,
> **KILL**, or **REPLACE** — with the evidence.
>
> Audited on branch `deepclean` (2026-07-06), main monolith
> `src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp`
> (6156 lines). Line numbers were verified at audit time; expect small drift.

---

## Verdict at a glance

| Component | Verdict | Why |
|---|---|---|
| `representation`, `navgraph`, `quadrant_manager`, `scene_graph_exporter`, `keypose_graph`, `room_segmentation` | **KEEP** | The scene graph itself |
| `planning_env` (+ `rolling_occupancy_grid`, `rolling_grid`, `grid`, `pointcloud_manager`) | **KEEP (untrimmed for now)** | The occupancy spine. The R3 trim is deferred with R1/R2 (see decision note below). |
| `viewpoint_manager` (+ `viewpoint`, `exploration_path`, `lidar_model`) | **KEEP (for now — decision 2026-07-06)** | Zero scene-graph API of its own, but `/freespace_cloud` (→ room_segmentation) and `KeyposeGraph::CheckLocalCollision` route through it. Replacing those (R1, R2) is *rewiring*, deferred until wanted; until then viewpoint_manager + its feeding path (`UpdateViewPoints` collision/LOS/connectivity, `planning_env` collision cloud) stay. `exploration_path`/`lidar_model` stay as its build deps. |
| `grid_world` | **KEEP (decision 2026-07-06)** | **It is the source of the NavGraph's connector nodes**: `UpdateCellStatus` bins candidate viewpoints into cells → `AddPathsInBetweenCells` → `keypose_graph->AddPath(...)` → non-keypose nodes → NavGraph. The user wants connector nodes kept, so grid_world + the candidate/coverage machinery feeding it stay. Only its TSP half (`SolveGlobalTSP`) is steering. |
| `local_coverage_planner`, `tsp_solver`, `tare_visualizer`, `graph`, **or-tools (137 MB)** | **KILL (Phase 2)** | Pure steering *outputs* — local TSP, global TSP ordering, exploration viz. Connector generation runs before/independently of the TSP solve. or-tools can only go if grid_world's then-dead `SolveGlobalTSP` method is also deleted (one dead-method module touch). |
| Terrain (`/terrain_map*` subs, 3 clouds, 2 callbacks) | **KILL** | Only feeds viewpoint collision/height (motion). Already inert: `scene_graph.launch` starts no terrain node |
| Room-typing pipeline (views → VLM → label) | **KEEP, planner-free** | Verified: zero dependency on planning_env / viewpoint_manager / grid_world / ray-casts |
| `UpdateRoomLabel` | **KILL** | Pure navigation bookkeeping + the `SetIsLabeled` corruption bug; room typing does not need it |
| Object↔viewpoint visibility ray-casts | **SPLIT** (R4) | Object→room assignment stays (mask lookup); LOS ray-cast links are nav/viz-only |
| `semantic_mapping` | **KEEP** | Couples to planner by exactly 2 topics: `/viewpoint_rep_header` in, `/object_nodes_list` out |
| `vlm_node` | **TRIM** | Keep room typing + object-type Q&A; kill navigation/target/anchor reasoning |
| `base_autonomy`, `route_planner` | **KILL (whole packages)** | Nothing in `scene_graph.launch` uses them |
| `slam` | **TRIM** | Keep `bag_slam_bridge.launch` (+ its node); arise_slam LIO itself unused in bag-direct/live |
| `utilities` | **TRIM** | Keep what live-robot bringup needs (bridge tooling, rviz_2d_overlay_plugins); kill teleop/waypoint/goalpoint plugins, ROS-TCP-Endpoint (audit per live config) |

**The three biggest findings:**

1. **Room typing is already planner-free.** `UpdateRoomViews` → `PublishRoomTypeQueries` → `RoomTypeCallback` reads only: camera image + odom pose ring buffer (`GetPoseAtTime`/`GetYawAtTime`), `room_mask_`, the instantaneous `registered_cloud_`, `PointToCameraView`, and per-room `best_views_`. The actual VLM gate is `{views_dirty_ ∨ object-count-change} ∧ has-evidence ∧ rate-limit` (`:4695–4720`) — **not** the `room_counts>100` gate (that lives in `UpdateRoomLabel` and only gates nav early-stop).
2. **`viewpoint_manager` has an empty scene-graph API surface** — every direct call is candidate sampling / coverage / waypoint / viz, or a *consumer* of scene-graph state (room mask / room id / transit flags pushed into it). Only two indirect couplings exist, both replaceable from the rolling occupancy grid (R1, R2).
3. **The exporter reads none of the nav flags.** `Build()` iterates all rooms unfiltered and reads only `GetRoomLabel()`, `interior_point_`, `centroid_`, `polygon_`, `neighbors_`, `GetObjectIndices()` (+ NavGraph + door cloud). `is_labeled_ / is_covered_ / is_visited_ / voxel_num_` never reach the JSON. (The ROOM_LABELING.md claim that the exporter gates on `is_labeled_` is stale.)

---

## Node & topic level

Current `scene_graph.launch` already starts only: optional `bag_slam_bridge`, static tf, image republish, `detection_node`, `semantic_mapping`, `room_segmentation`, `vlm_node`, `tare_planner_node` (via `explore.launch`), RViz. **No base_autonomy, no route_planner** — `/terrain_map`, `/terrain_map_ext` have no publisher today.

Inter-node contract that survives extraction:

```
sensors:      /registered_scan (or <ns>/cloud_registered), odom, camera image, camera_info, tf
planner node: /occupied_cloud ──► room_segmentation          (KEEP — from rolling_occupancy_grid)
              /freespace_cloud ─► room_segmentation          (REPLACE source — R1)
              /viewpoint_rep_header ─► semantic_mapping      (KEEP)
              /room_type_query ─► vlm_node                   (KEEP)
semantic:     /object_nodes_list ─► planner node             (KEEP)
              /object_type_query ⇄ /object_type_answer (vlm) (KEEP — object layer)
room_seg:     /room_nodes_list, /room_mask, /door_cloud ─► planner node   (KEEP)
              /wall_axis ─► quadrant_manager                 (KEEP)
vlm:          /room_type_answer ─► planner node              (KEEP)
```

---

## `tare_planner` module inventory

| Module | Verdict | Notes |
|---|---|---|
| `representation` | KEEP | Slim `ViewPointRep`: `cloud_` (keypose-cloud deep copy) and `covered_cloud_` are consumed only by viewpoint-manager coverage / viz — droppable fields |
| `keypose_graph` | KEEP | Feeders after extraction: `AddKeyposeNode` (needs point `InCollision` — one call site, `keypose_graph.cpp:617`), `CheckLocalCollision` (R2), `CheckConnectivity`. `AddPath` callers die with grid_world ⇒ trajectory-only graph (no connector nodes) — accepted |
| `navgraph`, `quadrant_manager`, `scene_graph_exporter` | KEEP | Unchanged; navgraph reads keypose_graph in-process |
| `room_segmentation` | KEEP | Standalone executable; inputs: registered scan, odom, `/occupied_cloud`, `/freespace_cloud` (R1) |
| `planning_env` | KEEP (R3 trim deferred) | The occupancy spine; also feeds viewpoint_manager's collision cloud while that stays |
| `rolling_occupancy_grid`, `rolling_grid`, `grid` | KEEP | The actual occupancy engine (`CheckLineOfSight`, FREE/OCCUPIED cells, updated clouds) |
| `pointcloud_manager` | KEEP | Occupancy persistence beyond the rolling window (`planning_env.h:99–119` stores rolled-out cells and re-feeds rolled-in) — needed so revisited areas keep occupancy |
| `viewpoint_manager`, `viewpoint` | KEEP (for now) | Decision 2026-07-06: removal requires R1/R2 rewiring — deferred. Untouched — grid_world stays too, so no overload prune needed |
| `grid_world` | KEEP | NavGraph connector-node source (`AddPathsInBetweenCells`); user explicitly wants connector nodes kept (2026-07-06) |
| `local_coverage_planner`, `tsp_solver` | KILL (Phase 2) | Local/global TSP (steering only; connector injection is upstream of the TSP solve) |
| `exploration_path` | KEEP | Build dep of viewpoint_manager (and the monolith's `exploration_path_` member while motion members are pruned) |
| `tare_visualizer` | KILL (Phase 2) | Exploration viz |
| `or-tools/` (137 MB vendored) | KILL (Phase 2) | Only tsp_solver uses it |
| `lidar_model` | KEEP | Included by `planning_env.h` and `viewpoint.h` — both stay |
| `graph` | KILL (verify) | Included only by the monolith; likely dead. Verify at build |
| `utils` (misc_utils, pointcloud_utils) | KEEP | Used everywhere |
| `navigation_boundary_publisher` | KILL | Steering-support executable |
| msgs | TRIM | Keep: ObjectNode(List), RoomNode(List), RoomType, ViewpointRep, DetectionResult, ObjectType, WallAxis. Kill: NavigationQuery, VlmAnswer, RoomEarlyStop1, TargetObject\*, TargetObjectInstruction (unless semantic_mapping's use of the instruction is kept — see vlm_node section) |

---

## The planner monolith (`SensorCoveragePlanner3D`)

### Subscriptions

**KEEP:** `registered_scan_sub_`, `state_estimation_sub_`, `camera_image_sub_`, `camera_info_sub_`, `object_node_list_sub_`, `room_node_list_sub_`, `room_mask_sub_`, `door_cloud_sub_`, `room_type_sub_` (`/room_type_answer`), `keyboard_input_sub_` (manual snapshot keyword; strip nav keys).

**KILL:** `exploration_start_sub_`, `terrain_map_sub_`, `terrain_map_ext_sub_`, `coverage_boundary_sub_`, `viewpoint_boundary_sub_`, `nogo_boundary_sub_`, `joystick_sub_`, `reset_waypoint_sub_`, `goal_point_sub_`, `room_navigation_answer_sub_`, `target_object_instruction_sub_`, `target_object_sub_`, `anchor_object_sub_`. (`viewpoint_room_boundary_sub_` is already a dead member — declared, never wired.)

### Publishers

**KEEP:** `viewpoint_rep_pub_`, `room_type_pub_` (`/room_type_query`), `/occupied_cloud` (inside planning_env), `/freespace_cloud` (new source, R1), keypose-graph viz markers/cloud. Viz worth keeping: `viewpoint_room_id_marker_pub_`, `object_node_marker_pub_`, `room_type_vis_pub_`, visibility markers *only if* R4 keeps ray-casts.

**KILL:** all path/waypoint/exploration publishers (`global_path*`, `local_path`, `exploration_path`, `/way_point`, `exploration_finish`, runtime/momentum, `pointcloud_manager_neighbor_cells_origin`), `room_navigation_query_pub_`, `room_early_stop_1_pub_`, `chosen_room_boundary_pub_`, `target_object_pub_`, `anchor_object_pub_`, `target_object_spatial_pub_`, `room_anchor_point_pub_`, `/door_position`, `/door_normal` (door-approach steering; the *door cloud storage* for the exporter stays).

### Methods

**KEEP (scene graph):** `RegisteredScanCallback` (trim: keep pose update, `UpdateRegisteredCloud`, `AddKeyposeNode`, keypose-cloud stacking), `StateEstimationCallback` (pose ring buffer), `CameraImageCallback` → `UpdateRoomViews`, `PublishRoomTypeQueries`, `RoomTypeCallback`, `PointToCameraView` + `TryCalibrateCamera` + `CameraInfoCallback`, `GetPoseAtTime` / `GetYawAtTime`, `ObjectNodeListCallback` + `ProcessObjectNodes`, `RoomNodeListCallback`, `RoomMaskCallback` (drop the pushes into viewpoint_manager/grid_world), `DoorCloudCallback` (uses `planning_env_->DoorInCollision` — keep), `GetDoorCentroid`, `SetCurrentRoomId` (keep the room-mask lookup; drop the 8 pushes into vm/gw and the `SetIsVisited` write), `UpdateObjectVisibility` (split per R4), `UpdateViewpointObjectVisibility` (R4), `UpdateViewpointRep`, `UpdateKeyposeGraph` (with R2), `CheckRayVisibilityInOccupancyGrid` / `InRange` / `Convert2Voxels` (R4), `SaveSceneGraphSnapshot` + `SceneGraphWatchdogCallback` + `TryFreezeWorldFromOdom`, `CreateVisibilityMarkers` / `PublishViewpointRoomIdMarkers` / `PublishRoomTypeVisualization` / `PublishObjectNodeMarkers` (viz), `LogRoomTypeQuery/Answer`.

**KILL (steering/exploration):** `SendInitialWaypoint`, `GlobalPlanning`, `LocalPlanning`, `ConcatenateGlobalLocalPath`, `GetLookAheadPoint`, `PublishWaypoint`, `GetRobotToHomeDistance`, `PublishExplorationState`, `CountDirectionChange`, `PrintExplorationStatus`, `PublishRuntime`, `UpdateViewPoints`, `UpdateViewPointCoverage`, `UpdateRobotViewPointCoverage`, `UpdateCoveredAreas`, `UpdateVisitedPositions`, `UpdateRoomLabel` (see write-deletion list), `SendInRoomWaypoint`, `SetRoomPosition`, `SetStartAndEndRoomId`, `ResetRoomInfo`, `GetToRoomState`, `GetDoorNormal`, `CheckDoorCloudInRange` (door *steering*; door state for export lives in `DoorCloudCallback`), `GetRobotToRoomDistance`, `CheckObjectFound`, `CheckAnchorObjectFound`, `Reset/SetFound*Object*`, `PublishRoomNavigationQuery`, `ChangeRoomQuery`, `GetAnswer`, `RoomNavigationAnswerCallback`, `TargetObject*Callback`, `AnchorObjectCallback`, `GoalPointCallback`, `JoystickCallback`, `ResetWaypointCallback`, boundary callbacks, `PublishGlobal/LocalPlanningVisualization`.

**SPLIT:** `UpdateGlobalRepresentation` — keep only `planning_env_->UpdateRobotPosition` (rolls the occupancy window; inline into `execute()`); everything else (grid_world neighbor cells, home position, frontier flag, viz) dies. `UpdateKeyposeCloud` goes with the coverage machinery (its output feeds viewpoint coverage; the viewpoint-rep trigger runs off the rolling grid, not this) — verify against golden master.

### `execute()` after extraction

```cpp
void execute() {                    // still on keypose_cloud_update_
  ProcessObjectNodes();             // deferred object deletes
  if (!keypose_cloud_update_) return;
  keypose_cloud_update_ = false;

  SetCurrentRoomId();               // room-mask lookup only
  PublishRoomTypeQueries();         // VLM gate: views_dirty/object-change/rate/evidence
  UpdateObjectVisibility();         // trimmed: object→room mask assignment (+R4 choice)
  planning_env_->UpdateRobotPosition(robot_position_);   // roll occupancy window
  UpdateViewpointRep();             // coverage-drift + obj-score trigger (unchanged)
  if (add_viewpoint_rep_) UpdateViewpointObjectVisibility();  // if R4 keeps ray-casts
  UpdateKeyposeGraph();             // heal (R2) + connectivity
  quadrant_mgr_->Update(...); BuildRoomGrids(...);
  navgraph_->Update(keypose_graph_, room_mask_, ..., room_keys, room_grids);
  PublishFreespaceCloud();          // R1 new source
  /* viz markers */                 // snapshots fire from their own timers
}
```

---

## Replacement tasks (the actual engineering)

> **DEFERRED (decision 2026-07-06).** The user chose to keep `viewpoint_manager`
> (and an untrimmed `planning_env`) for now: current scope is **only deletions
> that are completely unnecessary and safe — no rewiring**. R1/R2/R3/R4 below
> stay documented for when the viewpoint_manager removal is picked up again.

### R1 — `/freespace_cloud` for room_segmentation  ⚠ highest behavior risk
Today: `PublishFreespaceCloud()` (`:3377`) emits `viewpoint_manager_->GetFreespaceCloud()` = candidate-viewpoint positions that are `!InCollision && InLineOfSight && Connected` — i.e., a robot-height layer of validated free space (gated to >20 s after start). room_segmentation uses it to **decrement wall_hist** (`updateFreespace`, "the halving") — free-space evidence eroding wall votes.
Replacement: publish FREE-state cell centers from `rolling_occupancy_grid` restricted to a **z-band around robot height** (match the viewpoint layer). Differences to watch: grid FREE comes from ray-tracing (includes cells a viewpoint check would reject near obstacles); no connectivity gating; density differs (viewpoint resolution vs grid resolution). **Tune against the golden master** — wall erosion rate directly shapes rooms.

### R2 — `KeyposeGraph::CheckLocalCollision` without viewpoint_manager
Today (`keypose_graph.cpp:333–424`): deletes nodes/edges whose cells went into collision, using viewpoint_manager's precomputed per-cell flags — resolution, `GetViewPointInd`, `GetViewPointHeight` (same-layer gate), `ViewPointInCollision`.
Replacement: same algorithm against `rolling_occupancy_grid`: interpolate edges at `resolution/2`, query cell state (OCCUPIED ⇒ cut), reproduce the height-layer gate with a fixed z-tolerance. Alternative: `CheckLineOfSight(a, b)` per edge. Feasible, not a drop-in — expect small differences in which edges get healed; NavGraph output is the regression signal.

### R3 — `planning_env` trim (keep exactly this API)
`GetPlannerCloudResolution`, `UpdateRobotPosition`, `UpdateRegisteredCloud<T>` (drives `UpdateOccupancy` + `RayTrace` + `/occupied_cloud` publish), `GetUpdatedCloudInRange`, `GetUpdatedVoxelInds`, `GetCurrentObsVoxelInds`, `UpdateCoveredVoxels`, `Pos2Sub`, `InRange`, `CheckLineOfSightInOccupancyGrid`, `DoorInCollision`, `InCollision` (for `AddKeyposeNode`).
Delete: `UpdateCoverageBoundary`, `GetCollisionCloud`, `GetDiffCloud`, `GetStackedCloud`, `UpdateCoveredArea`, `GetUncoveredArea`, `PublishUncovered*`, `SetUseFrontier`/frontier machinery, viz accessors, and (verify) `UpdateKeyposeCloud` + the LiDARModel coverage path. Keep `pointcloud_manager` (occupancy persistence for revisited areas).

### R4 — `UpdateObjectVisibility` split (a decision, then mechanical)
Must keep: **object→room assignment** — `room_mask_` lookup (+2-voxel dilation fallback) → `SetObjectRoomRelation` (`:2759–2795`). This is what puts objects in the JSON's rooms; no occupancy needed.
Optional: the LOS ray-cast viewpoint↔object links (`CheckLineOfSightInOccupancyGrid`). They feed (a) RViz visibility markers, (b) `obj_score_` "+1.0 if unseen" in the viewpoint-drop trigger. Not exported. **Options:** keep as-is (cheap — grid already kept; zero behavior drift), or drop and rebase `obj_score_` on direct detections only (simpler; slightly different viewpoint placement). Recommendation: **keep for v1** (golden-master fidelity), reconsider later.

### R5 — navigation→Representation write deletions (bug fixes for free)
Delete: `SetIsLabeled(true)` @ `:4916`, `:4966` (**the label-corruption bug** — sets `is_labeled_` from coverage accrual before any VLM answer); `SetIsVisited(true)` @ `:2092`; `SetIsCovered` @ `:4183`/`:4190`; `SetIsAskedValue(2)` @ `:1808`; `SetIsAsked()` @ `:6050–6051`; `SetAnchorPoint` @ `:4938`, `:4990` (nav goals) and `:4730` (dead — answers re-resolve via `interior_point` now); all object `is_considered_` writes (die with object search).
Keep: the one legitimate label write — `RoomTypeCallback` @ `:1678–1681` (`labels_` + `SetIsLabeled(true)`). `is_labeled_` then means exactly "VLM has answered" (its only remaining reader is the RViz room-type marker + the labeled-skip in code being deleted).

---

## `vlm_node` trim

Keep: `/room_type_query` → `/room_type_answer` (room typing); `/object_type_query` → `/object_type_answer` (object-type verification for semantic_mapping — object layer, stays).
Kill: `/room_navigation_query`→`/room_navigation_answer`, `/room_early_stop_1` (answer side already commented out — dead), `/target_object_query`, `/target_object_spatial_query`→`/target_object_answer`, `/anchor_object_query`→`/anchor_object_answer`, `/target_object_instruction` publishing.
⚠ One check before deleting the instruction path: `semantic_mapping` also subscribes `/target_object_instruction` — confirm what it does with it (likely target-vocabulary steering for detection); if it only serves object search, kill on both sides.

---

## Risks / open items

1. **R1 freespace semantics** — the only replacement with real tuning risk (room shapes). Do it first, alone, against the golden master.
2. **R2 edge-healing drift** — compare NavGraph node/edge counts + JSON `edges` distances bag-over-bag.
3. **`UpdateKeyposeCloud` removal** — verify the viewpoint-rep trigger cadence is unchanged (it should be: trigger reads rolling-grid voxel sets, not the keypose cloud).
4. **`graph` / `lidar_model` modules** — expected dead after trim; confirm at build.
5. **Golden-master nondeterminism** — VLM answers and callback timing vary run-to-run; compare structurally (room/object/waypoint counts, positions within tolerance, edge distances), ideally with a VLM-off run for the geometric core.
6. **Sequencing** — finish/merge the `deepclean` quadrant branch first; this surgery touches the same files.

## Suggested phase order

0. Golden harness: run current stack on 1–2 reference bags, keep `snapshot_final.json` + a structural comparator.
1. **DONE (2026-07-06, commits 37f1f59..2b5b3d5)** — R5 write deletions + nav callback/publisher kills; −2014 lines from the monolith pair. Verified on the go2w_008 bag `20260628_232101` (`decompress_camera:=true` needed when launching outside the docker supervisor): room views → VLM query → `office-room_1`, 16+1 waypoints, 23 edges, compass, watchdog final snapshot. Stale doc references to `UpdateRoomLabel` remain in `ROOM_LABELING.md` (docs pass pending).
2. **REVISED ×2 (2026-07-06): steering-output deletions only — NavGraph connector nodes MUST survive.** **2A DONE (2026-07-13, commit 090de56):** monolith steering tail deleted (−972 lines, cpp+h only); `GlobalPlanning` = the three connector calls verbatim; kept `start_time_` (feeds `PublishFreespaceCloud`'s 20 s warm-up gate — discovered dependency) and the odom-ring RPY computation (room-view camera poses); `UpdateCandidateViewPointCellStatus` deleted (sole reader = local_coverage_planner.cpp:157, dies in 2B). Bag-smoke verified (planner alive, freespace/keypose clouds 1 Hz, room_type_query flowing). **2B DONE (2026-07-13, commit 23b3b6d, −861,846 lines / 2,807 files):** local_coverage_planner, tsp_solver, tare_visualizer, graph, navigationBoundary, or-tools (137 MB; package 139 MB → 1.5 MB) removed; dead `grid_world::SolveGlobalTSP` deleted (it contained ALL of grid_world's tsp_solver+ExplorationPath usage; `SetCurKeyposeGraphNodeInd`/`return_home_` kept — monolith still calls the setter); planning_env reordered after the static archives in the monolith link line (`--as-needed` broke once the deleted shared libs stopped re-pulling it); navigationBoundary node blocks stripped from the 3 legacy explore launches. Bag-smoke verified on a **Release** rebuild — NOTE: `colcon build --cmake-clean-cache` silently resets `CMAKE_BUILD_TYPE` to empty (−O0) and the planner then can't keep up (no freespace/keypose clouds for minutes — looks like a pipeline regression, isn't); always rebuild with `--cmake-args -DCMAKE_BUILD_TYPE=Release`. **2C DONE (2026-07-13, commit bf36962, −187,072 lines / 82 files):** terrain + boundary subscriptions/callbacks/clouds/params deleted from the monolith (all provably dead: terrain publishers lived only in base_autonomy's legacy launches; room_segmentation's `/navigation_boundary` publisher `pub_room_boundary_tmp_` is created but never published — README-confirmed placeholder); KEPT `kViewPointHeightFromTerrain*` + `kUseCoverageBoundary*` declares (viewpoint_manager / planning_env read them, both untouched); `base_autonomy/` + `route_planner/` + 6 legacy `system_*.sh` removed (zero surviving references, incl. `visibility_graph_msg`). Also committed separately: room-id viz fix 25587dc (id shows as soon as the room exists; VLM label appended on arrival — the old is_labeled_ gate was masking Phase 1's corruption fix).

**PHASE 2 VERIFIED (2026-07-13, run_2026-07-13_15-59-02 vs baseline run_2026-07-06_18-04-09, reference bag 20260628_232101, robot.yaml temporarily go2w_008):** identical GADM structure, room label `office` in both, waypoints 32→34 and edges 23→26 (run-to-run variance), long connector-fed shortcut edges present in both (max edge 9.4→13.5 m), compass present. NavGraph connector nodes survived the whole phase. KEEP: grid_world, viewpoint_manager, `UpdateViewPoints` (full, incl. `UpdateViewPointVisited`), the coverage updates, and `GlobalPlanning`'s connector prefix (`UpdateCellStatus` → `UpdateCellKeyposeGraphNodes` → `AddPathsInBetweenCells`) — everything that can influence candidate viewpoints / cell state / connector injection. DELETE (provably downstream of connector injection): `SolveGlobalTSP` call + global-path construction, `LocalPlanning` + local_coverage_planner, `ConcatenateGlobalLocalPath`/`GetLookAheadPoint`/`PublishWaypoint`/`SendInitialWaypoint` + lookahead/momentum members, return-home/exploration-termination block, `PublishRuntime` + runtime/waypoint/path publishers, path viz, tare_visualizer, `graph`, navigationBoundary executable; terrain + boundary dead inputs; packages `base_autonomy/` + `route_planner/` (unreferenced; keep `slam` — live tmux launches arise_slam; `utilities` needs a live/docker audit first). or-tools (137 MB) goes only together with deleting grid_world's then-dead `SolveGlobalTSP` method (single module touch) — else it stays. Verification: NavGraph node/edge counts + presence of connector-seeded nodes must match a pre-change bag run.
3. **DEFERRED — viewpoint_manager removal:** R1 (freespace from rolling grid, tune) → R2 (keypose healing vs grid) → R3 planning_env trim → delete viewpoint_manager/viewpoint; then `execute()` final reduction.
4. Trim vlm_node; drop dead msgs; thin launch/tmux.
5. Split to the new repo (clone with history), rename `SensorCoveragePlanner3D` → `SceneGraphNode`, pair with the config-portability cleanup.
