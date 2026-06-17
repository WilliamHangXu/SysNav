# room_segmentation

ROS 2 node that turns the robot's accumulated LiDAR scan + a rolling occupancy/freespace pair into a top-down map of **persistent, id-stable rooms** connected by **doors**. Walls are extracted from two complementary sources (a column-height histogram and region-grown vertical planes), the floor plan is dilated and watershed-segmented, and the resulting room labels are reconciled with the previous frame so that ids survive across cycles. The output is a `RoomNodeList` (one entry per live room: id, centroid, polygon, neighbors, area, room mask), plus a `door_cloud` whose per-point `r`/`g` channels carry the ids of the two rooms the door connects.

A single executable is shipped:

| Executable | File | Purpose |
| --- | --- | --- |
| `room_segmentation` | `src/room_segmentation/room_segmentation.cpp` | Everything below. |

The class lives in `include/room_segmentation/room_segmentation_node.h`. There is no Python side.

---

## Pipeline at a glance

```
/registered_scan  ─►┐
                    │   accumulate + downsample,
                    │   re-estimate normals on
                    │   affected neighbors only
                    ▼
            ceiling-filtered cloud, normals
                    │
                    ▼
            updateVoxelMap()            ─► navigable_voxels_ (3D)
                                          navigable_map_all_  (2D top-down count)
                                          wall_hist_all_      (2D, only z ∈ [wall_thres, ceiling])
                                          bbox_  (current crop, +20 px margin)

/occupied_cloud  ─► occupiedCloudCallback ─► updateStateVoxel()
                                              clears walls where freespace+occupied agree → state_map_all_

/freespace_cloud ─► freespaceCloudCallback ─► updateFreespace()
                                                freespace_indices_, dz-extension to clear navigable_voxels_

                              segment_flag_ ───────────┐
                                                       │ (every ~5 scans)
                                                       ▼
                         100 ms timer  ─►  roomSegmentation()
                                              │
                            ┌─────────────────┼─────────────────┐
                            ▼                 ▼                 ▼
                getWall(in_range_cloud_)   wall_from_hist      outside_boundary
                (region-grow → planes      (threshold on        from navigable_map_
                 → merge across frames)    wall_hist_)          + hole-fill (contour hierarchy)
                            └────────►  wall_from_plane ∪ wall_from_hist   ◄────────┘
                                                       │
                                            dilate, connectedComponents
                                            → seed markers for watershed
                                                       │
                                                cv::watershed
                                                       │
                                            updateRooms() (lifecycle)
                                            ┌──────────┼──────────┐
                                            ▼          ▼          ▼
                                         delete     update      split → new id
                                                       │
                                              door detection from
                                              watershed boundary
                                              (markers == -1)
                                                       │
                                              adjacency_matrix,
                                              isRoomConnected()
                                                       │
                                                       ▼
                          publishRoomNodes(), publishDoorCloud(), publishRoomPolygon()
                                              + /room_mask, /room_mask_vis
```

`/registered_scan`, `/occupied_cloud`, and `/freespace_cloud` are produced upstream (the same `/registered_scan` that drives `semantic_mapping`; the occupied/freespace pair is published by `sensor_coverage_planner_ground` from the rolling occupancy grid).

---

## Files

```
room_segmentation/
├── room_segmentation.cpp     # All callbacks + roomSegmentation() pipeline
include/room_segmentation/
├── room_segmentation_node.h  # RoomSegmentationNode class, PlaneInfo struct
launch/
├── room_segmentation.launch  # ros2 launch entrypoint (loads <scenario>.yaml)
config/
├── matterport_sim.yaml       # Example tuning — see "Configuration" below
└── ...                       # one yaml per scenario (indoor, outdoor, tunnel, etc.)
msg/
├── RoomNode.msg              # One room
└── RoomNodeList.msg          # std_msgs/Header + RoomNode[]
```

The downstream consumer is `src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` (`SensorCoveragePlanner3D::RoomNodeListCallback`, around line 1062), which feeds the data into `representation_ns::Representation` — see `include/representation/representation.h` for `RoomNodeRep`.

---

## ROS interface

### Subscriptions
| dir | topic | type | notes |
|---|---|---|---|
| sub | `/registered_scan` | `sensor_msgs/PointCloud2` | LiDAR scan in world (`map`) frame. The main driver. |
| sub | `/state_estimation` | `nav_msgs/Odometry` | Only the position is read (`robot_position_`); orientation is unused. |
| sub | `/occupied_cloud` | `sensor_msgs/PointCloud2` | Per-voxel state from the rolling occupancy grid; `intensity == 0` means "occupied" in this stream's convention. Used to *clear* false walls in already-known free space. |
| sub | `/freespace_cloud` | `sensor_msgs/PointCloud2` | Known-free voxels from the rolling occupancy grid. Erodes accumulated walls. |
| sub | `/keyboard_input` | `std_msgs/String` | `"demo"` freezes the pipeline (cached results keep being republished at ~2 Hz); `"resume"` thaws it. |

### Publishers (`pub_*` in the header)
| dir | topic | type | notes |
|---|---|---|---|
| pub | `/room_nodes_list` | `tare_planner/RoomNodeList` | **Main output.** One entry per live room. Consumed by the planner. |
| pub | `/room_nodes` | `tare_planner/RoomNode` | Declared but never published in the current code path; kept for tooling that wants individual node updates. |
| pub | `/room_mask` | `sensor_msgs/Image` (`32SC1`) | Integer label image — every pixel carries the room id (0 = background). The planner uses this to map robot/viewpoint positions to room ids. |
| pub | `/room_mask_vis` | `sensor_msgs/Image` (`bgr8`) | Same mask, recolored via `idToColor`, transposed/flipped for human-friendly RViz display. |
| pub | `/door_cloud` | `sensor_msgs/PointCloud2` (`PointXYZRGBL`) | One point per door pixel. `r` and `g` are the **room ids** the door connects (cast to uint8 — see "Gotcha" below); `b = 0`; `label` is a per-door instance id. |
| pub | `/room_boundaries` | `visualization_msgs/MarkerArray` | One `LINE_STRIP` per room polygon for RViz. |
| pub | `/current_room_boundary` | `geometry_msgs/PolygonStamped` | Polygon of the room the robot is currently in. |
| pub | `/navigation_boundary` | `geometry_msgs/PolygonStamped` | (currently unused publish-side — placeholder for downstream navigation) |
| pub | `/walls` | `sensor_msgs/PointCloud2` | Colored point cloud of the persistent `plane_infos_` (one random color per plane) for debugging. |
| pub | `/explore_areas_new` | `sensor_msgs/PointCloud2` | Downsampled, ceiling-filtered, normals-bearing cloud (`PointXYZINormal`). |
| pub | `/room_map_cloud` | `sensor_msgs/PointCloud2` | The room mask re-projected to 3D at `robot.z` and colored by room id. |
| pub | `/debug_cloud` / `/free_cloud_1` | `sensor_msgs/PointCloud2` | In-range cloud and freespace cloud at `robot.z`, for debugging only. |

### `tare_planner/msg/RoomNode`
```
int32 id                                # stable across frames
int32 show_id                           # 1..N, compact, only for visualization
geometry_msgs/PolygonStamped polygon    # outer contour of the largest connected component
geometry_msgs/Point centroid            # mean of the room's points, z = robot.z
int32[] neighbors                       # ids of connected rooms via doors
bool is_connected                       # reachable from the room the robot is currently in
float32 area                            # pixels × room_resolution²    [m²]
sensor_msgs/Image room_mask             # mono8, cropped to the room's bbox (with margin)
```

`RoomNodeList` is just `std_msgs/Header + RoomNode[]`.

---

## How the node ticks

The pipeline is **callback-driven, segmentation-triggered**, not periodic. The 100 ms timer (`timerCallback`, line 193) only acts as a flusher:

```
laserCloudCallback                          // fires on every /registered_scan
  - accumulate into explored_area_cloud_tmp_
  - every  5 * exploredAreaDisplayInterval  scans:
      downsample, height-filter to below ceiling_height_
      re-estimate normals on the newly-added points AND their
        kNN neighbors in the existing cloud  (incremental update)
      updateVoxelMap()                      // navigable_voxels_, *_map_all_, bbox_
      publish /explore_areas_new, /debug_cloud
      segment_flag_ = true                  // <-- the only trigger

occupiedCloudCallback                       // height-filter, updateStateVoxel()
freespaceCloudCallback                      // updateFreespace()
stateEstimationCallback                     // just stash robot_position_

timerCallback @ 10 Hz:
  if demo_frozen_:    republish cached results every 5th tick (~2 Hz)
  elif segment_flag_: segment_flag_ = false; roomSegmentation();
                      publishRoomNodes(); publishDoorCloud(); publishRoomPolygon();
```

So effective segmentation rate ≈ `lidar_rate / (5 × exploredAreaDisplayInterval)`, bounded above by the 10 Hz timer. With `exploredAreaDisplayInterval=1` and 10 Hz LiDAR that's ~2 Hz.

There are no locks — the node is single-threaded, and all callbacks plus the timer share the default callback group.

---

## Maps and voxels — the central data structures

The world is voxelized into a fixed-size 3D grid (`room_x × room_y × room_z`, default 200×200×50; configured up to 3000×3000×80 for Matterport). The grid is **anchored at the origin** via `shift_ = dim / 2`, so voxel (shift) corresponds to world (0,0,0). Anything outside the grid is clipped.

| Member | Type | What it stores |
|---|---|---|
| `navigable_voxels_` | `vector<int>` size `X*Y*Z` | 1 if a LiDAR point has been seen in this voxel (and the column is not flagged as known free space). |
| `state_voxels_` | `vector<int>` size `X*Y*Z` | Initialised to -1. Reserved for the LiDAR-reflection workaround (see `updateStateVoxel`). |
| `freespace_indices_` | `vector<int>` (2D xy indices) | Indices of `(x,y)` columns currently classified as known free space. Rebuilt every freespace callback. |
| `navigable_map_all_` | `cv::Mat` `CV_32F` `X×Y` | Top-down **count** of navigable voxels in each `(x,y)` column. |
| `wall_hist_all_` | `cv::Mat` `CV_32F` `X×Y` | Same, but only counting voxels whose `z ∈ (wall_thres_height_, ceiling_height_)` — this is the "wall-only" histogram. |
| `state_map_all_` | `cv::Mat` `CV_8U`  `X×Y` | 1 where the column has been confirmed free space (and so should never be drawn as a wall). |
| `bbox_` | 2× `Eigen::Vector2i` | Min/max `(row, col)` of `navigable_map_all_` non-zero pixels, expanded by a 20-px margin. All downstream processing operates on this crop. |
| `navigable_map_`, `wall_hist_`, `state_map_` | `cv::Mat` | The cropped views of the above three, sharing memory via `rowRange/colRange`. |
| `room_mask_` | `cv::Mat` `CV_32S` `X×Y` | The persistent room-id image. Each pixel = id of the room owning it; 0 = background. **This is the canonical room state across frames.** |
| `room_mask_vis_` | `cv::Mat` `CV_8UC3` `X×Y` | Recolored version of `room_mask_` (per-id color via `idToColor`). |
| `plane_infos_` | `vector<PlaneInfo>` | Persistent list of detected vertical planes (walls). See "How walls are tracked". |
| `room_nodes_map_` | `map<int, RoomNodeRep>` | **Persistent room list, keyed by id.** Owned by this node; the planner has its own copy. |
| `room_node_counter_` | `int` | Monotonic id allocator. New rooms get `++counter`; ids are never reused. |
| `ceiling_height_`, `wall_thres_height_` | `float` | Recomputed every laser callback as `*_base_ + robot_position_.z`, so the height bands follow the robot vertically. |

Helper conversions live in `utils/misc_utils.h`:
- `point_to_voxel(p, shift, 1/res)` → `Vector3i`
- `voxel_to_point(idx, shift, res)` → `Vector3f` (world XY, z=0)
- `point_to_voxel_cropped / voxel_to_point_cropped` — same, but offset by `bbox_[0]` so callers can work in cropped image coordinates.
- `idToColor(id)` — deterministic id→BGR mapping, also used for the room cloud and `/room_boundaries`.

---

## Stage-by-stage

### 1. `laserCloudCallback` (line 343)
Accumulates the latest scan into `explored_area_cloud_tmp_`. Every `5 × exploredAreaDisplayInterval` calls it flushes:
1. Voxel-downsample `explored_area_cloud_tmp_` into `downsampled_explored_area_cloud_tmp_`.
2. Recompute `ceiling_height_` / `wall_thres_height_` from `robot.z`.
3. Height-filter (`PassThrough`) into `downsampled_ceiling_cloud_tmp_` — anything above the ceiling is dropped.
4. **Incremental normal estimation.** Append the new points to `downsampled_ceiling_cloud_`, then for each new point find its `normal_search_num_` neighbors and *also re-estimate the normals of those old neighbors*. This keeps normals stable as the cloud grows without recomputing everything. Updated `normal_x/y/z/curvature` are written back in place.
5. `updateVoxelMap(ceilingPoint_tmp)` — bumps `navigable_voxels_`, `navigable_map_all_`, and (if the point sits in the wall band) `wall_hist_all_`; recomputes `bbox_` from `navigable_map_all_`'s non-zero hull plus a 20-pixel margin; refreshes the cropped views.
6. Final downsample of `downsampled_explored_area_cloud_` and `downsampled_ceiling_cloud_`.
7. Publish `/explore_areas_new` (ceiling-filtered cloud) and `/debug_cloud` (the `in_range_cloud_` — points within `region_growing_radius_` of the robot, used by wall extraction).
8. **Set `segment_flag_ = true`** so the next timer tick runs `roomSegmentation()`.

### 2. `occupiedCloudCallback` (line 483) → `updateStateVoxel`
Height-filters `/occupied_cloud` to the band `[robot.z - kViewPointCollisionMarginZMinus_, robot.z + kViewPointCollisionMarginZPlus_]`, then walks each occupied point: if its 2×2 neighborhood is fully in `freespace_indices_`, the surrounding 4×4 cell block is force-set to free space — `state_map_all_` ← 1, `navigable_map_all_` ← 1, `wall_hist_all_` ← 1, and the corresponding voxel column in `navigable_voxels_` is zeroed.

This is the LiDAR-reflection workaround: glass / specular surfaces leave fake "navigable" voxels behind a wall; the occupied+freespace agreement votes them out.

### 3. `freespaceCloudCallback` (line 506) → `updateFreespace`
Rebuilds `freespace_indices_` from the latest `/freespace_cloud`. For every freespace voxel and a small dz range (`dz ∈ [-2, 5]`) it decrements `navigable_voxels_` / `navigable_map_all_` / `wall_hist_all_` if a vote was previously cast in that column. Net effect: anything later confirmed as free has its accumulated walls and navigable-count erased.

### 4. `roomSegmentation()` (line 1525) — the main loop
The single function that turns the maps into rooms.

**(a) Outside boundary.** Threshold `navigable_map_` (cropped) → binary mask `outside_boundary` covering everywhere a point has ever been observed.

**(b) Wall extraction — two streams that get OR'd.**
- `getWall(in_range_cloud_)` runs PCL `RegionGrowing` on the points within `region_growing_radius_` of the robot (min cluster 300, neighbors 50, smoothness 3°, curvature 1.0). Each cluster is reduced to a vertical plane if `|normal · ẑ| < cos(80°)`, point-to-plane variance is small (`< 0.1`), and the cluster's vertical extent is `> 1.5 m`. Surviving clusters become `PlaneInfo` records with corners, u/v directions, width, height, voxel footprint. New planes are merged into the persistent `plane_infos_` list (see "How walls are tracked"). Each surviving plane is drawn into a 2D `wall_mask` as two filled quads: the wall footprint dilated by `outward_distance_1_` along its normal, and a "stop on first hit" extension along the wall's `u_dir` by up to `outward_distance_0_` — this connects walls that should butt up against each other but have a noisy gap.
- `wall_hist_` (the cropped column-band histogram) is normalised, thresholded at `0.5 × max`, and OR'd into the plane-derived mask. Then `wall_from_plane.setTo(0, state_map_)` zeros any wall pixel that is now known free space.

**(c) Outside-boundary cleanup.** Find contours of `outside_boundary` with `RETR_CCOMP`. Only outer contours are kept, and only holes with area `≥ 400 px` are punched back out (this throws away pinhole holes from occluded floor patches but preserves real interior obstacles).

Two variants are kept: `outside_boundary` (walls subtracted) and `outside_boundary_connected` (only the histogram walls subtracted; used as the watershed background mask, which has fewer breaks than the plane mask).

Both go through `connectedComponentsWithStats` and components smaller than 100 pixels are dropped.

**(d) Marker seeding + watershed.**
- Dilate `boundary_mask` by `dilation_iteration_` with a 3×3 rect kernel; invert; `connectedComponentsWithStats` on the result.
- Each component of area `> min_room_size_` is taken as a seed for a foreground marker (the eventual room).
- The background marker is the inverse of `full_map_connected`.
- `cv::watershed` (run on a 3-channel cv::Mat) propagates the seeds and writes the room labels into `markers`; the border between rooms (`markers == -1`) is the **door pixels** — this is where the door cloud comes from.

**(e) Lifecycle reconciliation (`updateRooms`, line 1196).** See next section.

**(f) Door detection + adjacency.** For every connected component of the `markers == -1` border:
- Look at the 3×3 neighborhood in `room_mask_cropped`; collect the set of room ids seen.
- 1 distinct id → not a door (drop the pixel). >2 ids → ambiguous junction, blank the 3×3 cell around it. Exactly 2 ids → it's a door between those two rooms.
- Append all of its pixels to `door_cloud_` with `r = id1`, `g = id2`, `label = adjacency_count` (acts as a per-pair door instance id, since two rooms can share multiple doorways).
- Add reciprocal entries to `room_nodes_map_[*]→neighbors_` and bump `adjacency_matrix(id1-1, id2-1)`.

**(g) Connectivity.** Find the room containing `robot_position_` (`current_room_label`); BFS on the adjacency matrix; set `room_node.is_connected_ = (reachable from current room)`. Also publish `/current_room_boundary` for that room.

**(h) Publish.** `room_mask_` as `32SC1` over `/room_mask`; the recolored, transposed/flipped vis over `/room_mask_vis`; then `publishRoomNodes()`, `publishDoorCloud()`, `publishRoomPolygon()` are called from the timer.

### Gotcha — door cloud channel width
`door_point.r = room_label_1; door_point.g = room_label_2;` writes ids into `pcl::PointXYZRGBL`'s **uint8** fields. With more than 255 distinct room ids these will overflow. In practice room counts never get close, but the planner (`sensor_coverage_planner_ground.cpp:DoorCloudCallback`) reads them back as `room_id_0 = point.r; room_id_1 = point.g;` and uses them as 1-based indices into `adjacency_matrix`. Keep that in mind if you push the dimensions.

---

## How rooms are reconciled across frames — `updateRooms`

This is the core lifecycle step. Inputs: the current `room_mask_cropped` (persistent ids from previous frames), `room_mask_new` (raw watershed labels 1..N for this frame), the visualisation crop, and the room count `N`.

Initial scrub: any pixel that is 0 in `room_mask_new` becomes 0 in `room_mask_cropped` too — gone-away rooms can't survive into this cycle.

Then, for each existing `room_node` in `room_nodes_map_`, look at the pixels it owned in the previous frame:

| Case | Detection | Action |
|---|---|---|
| **1. Deleted / merged away** | All of the old pixels map to 0 in the new mask, **or** ≥80% map to 0, **or** the old id is no longer present in the cropped image | `room_node.alive = false`, id added to `room_ids_to_remove`. The room node will be erased from `room_nodes_map_` at the end. |
| **2. One-to-one update** | The old pixels map to exactly one non-zero new label `v` | Replace the room's pixels with the new label's footprint. Calls `room_node.UpdateRoomNode(non_zero_points_new)`. Removes `v` from `room_need_process_ids` so it doesn't become a new room. |
| **3. Split** | The old pixels map to multiple new labels | Pick the new label with the maximum overlap area as the "main" successor — it inherits the old id (case 2 above). For the other new labels: if at least half of that label's pixels are still unclaimed in `room_mask_cropped`, allocate a fresh id (`++room_node_counter_`) and stamp those pixels with it. |

Finally, any new label still in `room_need_process_ids` and present in `room_mask_new` becomes a brand-new room with a fresh `room_node_counter_` id.

`room_nodes_map_` is then garbage-collected: any entry with `alive == false` is erased.

A second pass renumbers `show_id_` 1..N for visualisation (only `show_id_` is compact — `id_` itself is the monotonic counter), recomputes each room's polygon (`computePolygonFromMaskCropped`, which takes the largest contour), `area_`, `centroid_` (mean of pixel positions, `z = robot.z`), clones the cropped per-room mask into `room_node.room_mask_`, and clears `neighbors_` ready for the door pass.

Two consequences worth knowing:
- **Ids are append-only and never reused.** A "Room 47" that gets deleted leaves a hole — the planner can rely on `id` being a stable handle for the lifetime of a room.
- **`show_id` is not stable** — it renumbers every cycle for visualisation. Don't use it as a key.

### `RoomNodeRep` (`include/representation/representation.h`)
The persistent per-room object that lives in `room_nodes_map_`. Key fields:
```
id_                 stable monotonic id
show_id_            1..N, recomputed every cycle
polygon_            outer contour (PolygonStamped)
points_             cropped image pixels owned by this room
centroid_           Vector3f, world coordinates
area_               m²
alive               internal lifecycle flag
neighbors_          set<int> of connected room ids
is_connected_       reachable from robot's current room
room_mask_          mono8 per-room bitmap (cropped to its bbox)

// extra slots used by the planner side, not by this node:
viewpoint_indices_  set<int>  (planner populates)
object_indices_     set<int>  (planner populates)
labels_             map<string, int>  room-type label store (latest answer wins)
best_views_         vector<RoomView>  up to 3 best camera views (the query payload)
anchor_point_       Point   room centroid sent in the query
is_labeled_, is_asked_, last_area_, voxel_num_, views_dirty_, last_query_time_
```

The room-typing fields (`labels_`, `best_views_`, `anchor_point_`, etc.) are written by the planner; they do **not** flow back to this node. The room segmentation node only writes the structural fields and publishes them; the planner mirrors them in its own copy and adds the VLM/labeling state. Full mechanism: [Room labeling](#room-labeling--the-semantic-room-type) below.

---

## How walls are tracked across frames — `plane_infos_`

`getWall` accumulates the persistent `plane_infos_` vector across cycles. Each new candidate plane is compared against every existing plane via `isPlaneSame` (line 243), which checks:
1. Angle between normals `≤ angle_threshold_deg_` (default 6°).
2. Centroid offset along the normal direction `≤ distance_angel_threshold_` (default 0.3 m) — i.e. they're co-planar, not parallel.
3. Edge-to-edge distance `≤ distance_threshold_` (default 2.5 m) — `(centroid_dist) - (width_a + width_b)/2`.

If matched, `mergePlanes` (line 1109) downsamples the concatenated cloud, refits the plane with RANSAC (`SACMODEL_PLANE`, threshold 0.2 m), recomputes corners/width/height/voxel footprint, and marks the new plane `merged = true` so the wall-mask drawing step skips it (the base plane is drawn instead). New planes whose centroid is in range and not matched simply get `id = plane_infos_.size()` and are appended.

Two cleanup steps run on every cycle:
- A self-merge pass: O(n²) over `plane_infos_` to fuse any two alive planes that became co-planar after their updates.
- A "now-free" prune: for each plane, if >33% of its voxel footprint sits on `state_map_all_ == 1` (known free space), it's `alive = false`. Then non-alive planes are erased.

---

## Configuration

Parameters are loaded by `room_segmentation.launch` from the per-scenario yaml that matches the `scenario` launch arg (`matterport_sim.yaml`, `indoor.yaml`, etc., under `tare_planner/config/`). All parameters are read at construction time only — runtime reconfiguration is not supported.

| Param (yaml key) | Default | Meaning |
|---|---|---|
| `room_resolution` | 0.1 m | Voxel + image pixel size in the top-down map. Every 1 px = 10 cm by default. |
| `room_x`, `room_y`, `room_z` | 200, 200, 50 | Voxel grid dimensions. Scenario yamls bump these significantly (3000×3000×80 for Matterport). |
| `exploredAreaVoxelSize` | 0.1 m | Voxel downsample for `explored_area_cloud_`. |
| `rolling_occupancy_grid.resolution_x` | 0.2 m | Sub-param read but currently only stashed; not used in this node. |
| `exploredAreaDisplayInterval` | 1 | Trigger a flush every `5 * this` scans. Larger = slower segmentation but fewer cycles. |
| `ceilingHeight_` | 2.0 m | Above this (relative to robot z) points are cropped. |
| `wall_thres_height_` | 0.1 m | Lower bound (relative to robot z) of the band used for `wall_hist_`. |
| `outward_distance_0` | 0.5 m | Extension along the wall's `u_dir` when drawing the 2D wall mask (helps close noisy gaps). |
| `outward_distance_1` | 0.3 m | Wall thickness drawn perpendicular to the plane. |
| `distance_threshold` | 2.5 m | `isPlaneSame` edge-to-edge tolerance. |
| `distance_angel_threshold` | 0.3 m | `isPlaneSame` along-normal offset tolerance. |
| `angle_threshold_deg` | 6° | `isPlaneSame` normal angle tolerance. |
| `region_growing_radius` | 15.0 m | Crop radius around the robot for wall plane detection. |
| `dilation_iteration` | 4 | Wall mask dilation iterations before watershed marker seeding. |
| `min_room_size` | 40 (px) | A connected component smaller than this after dilation is dropped (not seeded as a room). With `room_resolution=0.1` this is 0.4 m². |
| `normal_search_num` | 50 | kNN size for incremental normal estimation. |
| `normal_search_radius` | 0.5 m | (Set but not currently used — incremental path uses kNN.) |
| `kViewPointCollisionMarginZPlus_` | 0.5 m | Upper height band for filtering `/occupied_cloud`. |
| `kViewPointCollisionMarginZMinus_` | 0.5 m | Lower height band. |
| `isDebug` | false | When true, `saveImageToFile` dumps PNG snapshots of every intermediate mask. **All PNGs are written to the working directory** — make sure that's where you want them, especially if launched via a robot's systemd unit. |

The pipeline is sensitive to a few of these:
- **`dilation_iteration` is the main "how aggressively do we split rooms" knob.** Higher = wider walls before watershed = more rooms split. Drop it if hallways are getting carved into rooms; raise it if two real rooms are bleeding together.
- **`wall_thres_height_` matters in low-ceiling spaces.** It controls the height band for the wall histogram; if you go below where the LiDAR sees mostly furniture, walls become noisy.
- **`distance_angel_threshold_` (along-normal) is the only thing keeping nearby parallel walls from getting merged.** If you see "two parallel walls became one", tighten it.

---

## Demo freeze mode

`/keyboard_input "demo"` sets `demo_frozen_ = true`. The three sensor callbacks early-return, so nothing is updated. The timer keeps firing and, every 5 ticks (~2 Hz), republishes the cached `room_nodes_map_` / `door_cloud_` / polygons so downstream nodes don't lose their inputs. `"resume"` restores normal operation. Useful when recording a deterministic demo or pausing for an external VLM pass without dropping the current map.

---

## How the planner consumes this

`sensor_coverage_planner_ground.cpp`:

1. **`RoomNodeListCallback` (line 1062).** Marks every room in `representation_->GetRoomNodesMapMutable()` as `alive=false`, then for each msg either calls `AddRoomNode(msg)` (new id) or `GetRoomNode(id).UpdateRoomNode(msg)` (existing). Finally, any room still `alive=false` is erased. The "delete this room" signal is therefore implicit: a room is gone exactly when it disappears from the next `RoomNodeList`.

2. **`RoomMaskCallback`.** Receives `/room_mask` (`32SC1`), resizes to `room_voxel_dimension_`, hands it to `viewpoint_manager_`, `grid_world_`, and `representation_->UpdateViewpointRoomIdsFromMask(...)`. This is how *viewpoints* and *objects* get assigned to rooms.

3. **`DoorCloudCallback`.** Decodes the `r`/`g` channels back into room ids and fills `adjacency_matrix` — the planner's own door graph, kept separately from `room_nodes_map_[*].neighbors_`. Doors that would put the planner in collision (`planning_env_->DoorInCollision`) are dropped.

4. The room labels themselves (strings like "office room", "kitchenette") **come from the VLM**, not this node. The planner keeps a per-room buffer of the best few camera frames; whenever that evidence changes it publishes `/room_type_query` (image paths + object inventory + the room's current label) and ingests the answer on `/room_type_answer` into `RoomNodeRep.labels_` — **latest answer wins**. Never round-trips back here. The full mechanism is documented in **["Room labeling — the semantic room type"](#room-labeling--the-semantic-room-type)** below.

---

## Room labeling — the semantic room type

This node produces **structural** rooms (id, polygon, centroid, mask, neighbors). It never assigns a *type* — the strings "office room", "kitchenette", "lobby". That semantic label is added downstream, by `sensor_coverage_planner_ground` together with the VLM (`vlm_node/vlm_reasoning_node.py`), and stored only in the planner's copy of the room (`RoomNodeRep`). It never flows back into this node, but it completes a room's identity, so it is documented here.

> ⚠️ **This pipeline was redesigned for a low, front-facing camera** (Unitree Go2). An earlier design assumed a high 360° panorama where any one frame ≈ a full room overview; a low front camera breaks that (most frames see a wall or a sliver). The current design instead **accumulates evidence over time**: it keeps the **best few camera frames** of each room plus the room's **object inventory**, and lets the VLM type the room from that. The old single-frame `project_pcl_to_image` crop is gone (the function is dead code).

### The model in one paragraph
Each room maintains a rolling **best-3 set of camera views** (chosen for how much of the room they actually see, kept diverse by pose) plus its **detected objects**. Whenever that evidence changes, the planner sends the VLM the three image paths + the object list + the room's *current* label, and the VLM returns one type. **Latest answer wins** — there is no vote histogram; the label only changes when the VLM deliberately changes it. Sending the current label gives stability and prevents synonym churn (e.g. "meeting room" ↔ "conference room").

### Where the state lives — `RoomNodeRep` typing slots
(`include/representation/representation.h`, written only on the planner side)

| Field | Type | Role |
|---|---|---|
| `best_views_` | `vector<RoomView>` | Up to 3 best camera observations. Each `RoomView` = `{image_path, camera_pos, camera_yaw, anchor_xy, coverage_m2}`; the jpgs live on disk, only the path is held. |
| `labels_` | `map<string,int>` | The label store. Latest-answer-wins sets it to `{answer: 1}`, so `GetRoomLabel()` returns that answer. (Kept as a map for the argmax accessor / exporter compatibility.) |
| `GetRoomLabel()` | `string` | The current label. Empty until the first answer. Canonical room type everything downstream reads. |
| `views_dirty_` | `bool` | `best_views_` changed since the last query → re-query. |
| `last_query_time_` | `double` | Wall seconds of the last query (per-room rate limit). |
| `objects_at_last_query_` | `int` | Object count at the last query (new-object re-query trigger). |
| `anchor_point_` | `Point` | Room centroid. Sent in the query and used to re-resolve the room id when the answer returns (via `room_mask_`). |
| `object_indices_` | `set<int>` | The room's objects (assigned via `room_mask_`); their labels form the query's object inventory. |
| `is_asked_` | `int`, starts at 2 | **Unrelated to typing** — the early-stop *navigation* budget. Listed only to avoid confusion. |
| `ClearRoomLabels()` | — | Resets the label **and** `best_views_` + query state when a room is substantially re-segmented. |

### Stage 1 — the per-room best-3 view buffer
`UpdateRoomViews()`, called from `CameraImageCallback`, runs on every `/camera/image` frame (always on). For each frame:

1. **Motion gate** — skip unless the robot moved > `room_view.motion_dist_m` (0.2 m) or turned > `room_view.motion_yaw_deg` (5°) since the last evaluated frame (pose taken at the frame's timestamp via `GetPoseAtTime`). Bounds cost and keeps views diverse.
2. **Yaw-rate (blur) gate** — skip if the instantaneous yaw rate (two latest odom samples) exceeds `room_view.max_yaw_rate_deg_s` (45 °/s). Fast turns ⇒ motion blur.
3. **Coverage = observed floor cells.** Bin the **instantaneous LiDAR sweep** (`registered_cloud_`, *not* the accumulated map) into rooms via `room_mask_`, deduped by cell. Each point is kept only if it projects into the camera image: `PointToCameraView` runs the exact **Go2 pinhole model** (extrinsics + intrinsics + plumb_bob distortion mirrored from `cloud_image_fusion.py:scan2pixels_go2w_bag`), so horizontal **and** vertical FOV come from the real 1280×720 frame, not a hand-set angle; then a `room_view.max_range_m` (8 m) depth gate. A room's coverage = the number of distinct floor cells it returned. Using *observed* returns gives occlusion for free — a wall yields no returns beyond it, so this kills both wall-facing frames and through-wall misattribution. The frame is attributed to the room with the most observed cells, rejected below `room_view.min_coverage_m2` (1.0).
4. **Pose-diversity admission** into that room's `best_views_`: if pose-close (`room_view.pose_dist_m` 1.5 m / `pose_yaw_deg` 40°) to an existing view, keep the higher-coverage one; otherwise fill an empty slot or evict the weakest if beaten. Accepted frames are written to `room_views/room_<id>_slot_<k>.jpg`; the view's `anchor_xy` is the mean of its covered cells. On any change, `views_dirty_ = true`.

### Stage 2 — query emission
`PublishRoomTypeQueries()` runs once per planning cycle (after `SetCurrentRoomId`). For each live room it fires a query when **`views_dirty_` OR the object count changed**, rate-limited by `room_type_query.min_interval_s` (3 s), provided there is ≥ 1 image or object as evidence. This **replaces** the old coverage-growth trigger and is decoupled from `UpdateRoomLabel`, which now keeps only the navigation early-stop logic (`Early Stop 1` / `ChangeRoomQuery`).

The **object inventory** is built from the room's `object_indices_`: each object's `GetLabel()`, confidence-filtered (`room_view.object_conf_min` 0.3), deduped with counts → `"desk x2, monitor x1"`.

### The query payload (`/room_type_query`, `tare_planner/msg/RoomType`)
```
std_msgs/Header   header
geometry_msgs/Point anchor_point  # room centroid — re-resolves the room on answer
string[]          image_paths     # ≤3 best-view jpgs on disk (vlm_node reads them)
sensor_msgs/Image room_mask       # this room's mono8 top-down footprint
string            objects         # "desk x2, monitor x1, ..."  (deduped, conf-filtered)
string            room_type       # the room's CURRENT label (for stability); "" if none yet
int32             room_id
int32             voxel_num        # coverage count (legacy field; no longer a vote weight)
bool              in_room
```

Images are sent **by path**, not embedded — the planner and `vlm_node` share a filesystem. The top-down `room_mask` (a camera-independent shape cue) is still embedded.

### How it is answered — the VLM
`vlm_node/vlm_reasoning_node.py:process_room_type_query`:

1. `/room_type_query` → enqueued; the periodic loop **keeps only the latest query per `room_id`** and runs one thread per room.
2. The handler reads the ≤3 images from `image_paths` (and the `room_mask`), then builds the prompt = a room-type prompt + the **object inventory** + a **current-label hysteresis** instruction (*"return this same label unless the images and objects clearly indicate a different kind of room; never swap it for a synonym"*), and asks the VLM for one `room_type`.
3. Republishes the *same* message on `/room_type_answer` with `room_type` filled (lowercased).

> The active prompt is `ROOM_TYPE_PROMPT_FREE` (open-vocabulary). A closed candidate list (`self.room_types`) still exists in the node but is currently commented out; in the open-vocab setting it is the **current-label hysteresis** that prevents synonym drift.

### How the answer is ingested — latest answer wins
`RoomTypeCallback`:

1. Re-resolve the room id from `anchor_point` → voxel → `room_mask_` (matched by *position*, not by trusting the returned `room_id`).
2. `labels_.clear(); labels_[room_type] = 1` — the newest answer **replaces** the label (no accumulation). `GetRoomLabel()` returns it.
3. `SetIsLabeled(true)`.
4. If the label **changed**, every object in the room is marked `is_considered_ = false` to be re-evaluated against the new context.

### Lifecycle of the saved views (merge / split / delete)
`best_views_` rides the room node's lifecycle. In `RoomNodeListCallback`, just before a dead room is erased its views go to an orphan pool, then each is **re-homed by `anchor_xy → room_mask_`**: it joins whichever live room now owns that cell (so a **merge** survivor pools both rooms' views and keeps the best 3 by coverage), or is dropped if the area became background (a true **delete**). Splits need no special handling — the child fills naturally. No merge/split *detection* is required; the mask says where each view's content went.

### Logging (optional)
The pipeline above is **always on**. Setting `room_type_query_log.enabled: true` additionally dumps, under the per-run output folder:
- `room_views/room_<id>.json` — each room's current best-3 (paths, coverage, poses);
- `room_type_queries/query_<seq>_room<id>.json` — each outgoing query payload;
- `room_type_queries/answer_<seq>_room<id>.json` — each answer + the resulting label.

The best-view jpgs (`room_views/room_<id>_slot_<k>.jpg`) are written **regardless** — they are the query payload, not just debug.

### Net flow
```
/camera/image ─► UpdateRoomViews  (motion+yaw gate, observed-coverage, pose-diversity)
                      │ best_views_ (≤3 jpgs on disk) + views_dirty_
objects ──────────────┤
                      ▼
   PublishRoomTypeQueries ──/room_type_query (paths + objects + current label)──► VLM
                      ▲                                                            │
   GetRoomLabel() = labels_ (latest answer)  ◄────────/room_type_answer───────────┘
                      │
                      ▼
   exporter RoomKey "<label>-room_<id>"
```

The exporter consumes only `GetRoomLabel()` at snapshot time — see the [scene_graph_exporter README](../scene_graph_exporter/README.md).

### Key parameters (planner side; all have defaults, no yaml required)
| Param | Default | Meaning |
|---|---|---|
| `room_view.max_range_m` | 8.0 | Max depth counted for coverage (camera FOV itself comes from the baked-in Go2 pinhole intrinsics, not a param). |
| `room_view.min_coverage_m2` | 1.0 | Reject frames seeing less room than this. |
| `room_view.max_yaw_rate_deg_s` | 30 | Reject frames captured turning faster (blur). |
| `room_view.motion_dist_m` / `.motion_yaw_deg` | 0.2 / 5 | Intake gate (move/turn since last eval). |
| `room_view.pose_dist_m` / `.pose_yaw_deg` | 1.5 / 40 | Pose-diversity admission thresholds. |
| `room_view.object_conf_min` | 0.3 | Object-inventory confidence floor. |
| `room_type_query.min_interval_s` | 3.0 | Per-room re-query rate limit. |
| `room_type_query_log.enabled` | false | Dump the query/answer/view JSON logs above. |

---

## When something breaks — first places to look

- **No rooms appear** → check that `segment_flag_` is being set. Likely culprits: `/registered_scan` not arriving, `exploredAreaDisplayInterval` too high, or `updateVoxelMap` finding no non-zero pixels (the cloud is all above `ceiling_height_`).
- **One real room gets cut into many** → wall mask is too thick. Lower `dilation_iteration_`, or tighten the `wall_hist_` threshold (currently hard-coded at `0.5 × max` in `roomSegmentation`).
- **Two real rooms bleed into one** → walls are missing. Check `wall_thres_height_` vs. the actual furniture height, or that `getWall` is producing planes (`/walls` cloud should show one color per detected plane).
- **Rooms keep flipping ids every cycle** → `updateRooms` cases 2/3 are not finding the right successor. Usually means the room shape is being chewed by `/freespace_cloud` (look at `state_map_` snapshots with `isDebug:=true`). Less commonly, two rooms swap their "most-overlap" pick. Inspect `room_mask_new.png` from successive cycles.
- **Doors point to wrong rooms** → the watershed boundary is being misread. The 3×3 neighborhood sampling assumes the door pixel sits on a 2-room border; if more than two rooms touch at a pixel, the door is dropped (look for `"Door mask has more than two labels"` in the log). Often a sign that `min_room_size_` is too small and a single-pixel room artefact is touching the door.
- **PNG dumps clobber each other / aren't written** → `saveImageToFile` uses bare relative paths. They land in the working directory of whoever launched the node. Set `isDebug:=false` in production; redirect via `cd` before launch when debugging.
- **`/room_nodes_list` is empty but the visual mask looks fine** → `room_nodes_map_` was wiped because every room hit case 1 in `updateRooms`. Usually a sign that the bbox shifted faster than the room ids could keep up — `updateVoxelMap`'s 20-pixel margin should prevent this in steady state; if you scaled the grid down too aggressively it can happen.
- **Stale walls don't go away** → `/occupied_cloud` or `/freespace_cloud` aren't being published, or `kViewPointCollisionMarginZ*` is missing the actual band where the wall sits. `updateFreespace` needs the freespace pass *first*, then `updateStateVoxel` consumes the indices.

---

## Running

The node is part of the `tare_planner` package:
```bash
ros2 launch tare_planner room_segmentation.launch scenario:=matterport_sim use_sim_time:=true
```
Or it's brought up as part of the full `tare_planner` stack — the standard SysNav launch files start it alongside `sensor_coverage_planner_ground`.

Manual demo controls:
```bash
ros2 topic pub /keyboard_input std_msgs/String "data: 'demo'"     # freeze
ros2 topic pub /keyboard_input std_msgs/String "data: 'resume'"   # thaw
```

For inspection without RViz:
```bash
ros2 topic echo /room_nodes_list --field nodes[].id
ros2 topic echo /door_cloud
```
