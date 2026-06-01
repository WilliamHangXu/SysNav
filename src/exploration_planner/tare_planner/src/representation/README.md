# representation

The **canonical in-memory scene graph** of the SysNav planner. `representation_ns::Representation` is a single object owned by `SensorCoveragePlanner3D` (`sensor_coverage_planner_ground.cpp:384`) that fuses three asynchronous data streams — semantic-mapping object updates, room-segmentation room updates, and the planner's own keypose-driven viewpoint samples — into one coherent graph of **viewpoints ↔ rooms ↔ objects**. Every higher-level planning decision (which room to enter next, which viewpoint to ask a VLM about, whether the target object has been seen) reads from this object.

There is no ROS interface here — `Representation` is a pure data structure. All I/O happens in `SensorCoveragePlanner3D` callbacks and timers, which write to `Representation` after deserializing inbound messages.

| File | Purpose |
| --- | --- |
| `include/representation/representation.h` | Class definitions for `ViewPointRep`, `RoomNodeRep`, `ObjectNodeRep`, and the top-level `Representation`. |
| `src/representation/representation.cpp` | Method bodies. ~420 lines. No threading, no locks. |

---

## What's stored

```
                                Representation
                                      │
        ┌─────────────────────────────┼─────────────────────────────┐
        ▼                             ▼                             ▼
 viewpoint_reps_              room_nodes_map_              object_node_rep_map_
 vector<ViewPointRep>         map<int, RoomNodeRep>        unordered_map<int, ObjectNodeRep>
 (index == id_)               (key == id_)                 (key == object_id_[0])
        │                             │                             │
        │  room_id_   ────────────────┤  viewpoint_indices_         │
        │                             │  object_indices_   ─────────┤
        │  object_indices_   ─────────┼──────────────────────────►  │  room_id_
        │  direct_object_indices_ ────┘                             │  visible_viewpoint_indices_
        └───────────────────────────────────────────────────────────┘

         + latest_object_node_indices_  : set<int>   objects updated this cycle, pending visibility resolution
         + viewpoint_rep_vis_cloud_     : pcl PointXYZ — one point per viewpoint position (RViz)
         + covered_points_all_          : pcl PointXYZI — union of all viewpoints' covered_cloud_
```

### `ViewPointRep` — a keypose where the robot stood (line 62, header)

| Field | Type | What it means / where it's written |
|---|---|---|
| `id_` | `int` | Always equal to its index in `viewpoint_reps_` (`representation.cpp:143`). |
| `position_` | `geometry_msgs/Point` | Robot world position at the moment the viewpoint was created. Immutable afterwards. |
| `cloud_` | `PointCloud<XYZRGBNormal>::Ptr` | A **deep copy** of the planner's current `keypose_cloud_` at add time. Used by viewpoint manager / coverage planner. |
| `covered_cloud_` | `PointCloud<XYZI>::Ptr` | Deep copy of `planning_env_->GetUpdatedCloudInRange()` at add time — voxels this viewpoint contributed to the covered set. |
| `timestamp_` | `rclcpp::Time` | Stamp the viewpoint was added (taken from `viewpoint_rep_msg_.header.stamp`). |
| `room_id_` | `int` | Room this viewpoint sits in. Written at creation, then re-resolved every `RoomMaskCallback` (see "Relations" below). `-1` if off-map. |
| `object_indices_` | `set<int>` | All objects geometrically visible from this viewpoint (via occupancy-grid ray-cast in `UpdateObjectVisibility` / `UpdateViewpointObjectVisibility`). Superset of `direct_object_indices_`. |
| `direct_object_indices_` | `set<int>` | Objects whose detection frame was aligned to this viewpoint by `semantic_mapping_node` — i.e. the camera actually saw them here. Populated by `UpdateObjectNode` when `msg->viewpoint_id` matches. |

There are **no setters that delete** — viewpoints append-only forever. `id_ == index` is an implicit invariant; nothing erases or inserts mid-vector.

### `RoomNodeRep` — a persistent room (line 136, header)

Fields split by who writes them:

**Written by `room_segmentation` (via the `RoomNode` ROS message → `UpdateRoomNode(msg)` at `representation.cpp:82`):**
| Field | Type | Meaning |
|---|---|---|
| `id_` | `int` | Monotonic room id (never reused; allocated by `room_segmentation`). |
| `show_id_` | `int` | Compact 1..N display id, *recomputed every cycle by `room_segmentation`* — do not use as a stable key. |
| `polygon_` | `PolygonStamped` | Outer contour of the room (largest connected component). |
| `centroid_` | `Vector3f` | Pixel-mean of the room's footprint, world coords. |
| `neighbors_` | `set<int>` | Room ids connected to this one via doors. Reset on every message. |
| `area_` | `float` | m². |
| `room_mask_` | `cv::Mat` mono8 | Per-room bitmap cropped to its bbox. |
| `is_connected_` | `bool` | Reachable from the room currently containing the robot. |
| `alive` | `bool` | Lifecycle flag set to `true` on update, set `false` by the planner before re-ingest (see `RoomNodeListCallback`). |

**Written by the planner (VLM / room-typing pipeline):**
| Field | Type | Meaning |
|---|---|---|
| `labels_` | `map<string, int>` | Vote histogram for room type, accumulated from `/room_type_answer` (`RoomTypeAnswerCallback`). `GetRoomLabel()` returns the argmax. |
| `is_labeled_` | `bool` | True once any vote has arrived. |
| `is_visited_` | `bool` | Set by `UpdateGlobalRepresentation` / `SetIsVisited` when the robot has entered. |
| `is_covered_` | `bool` | True when coverage planner has explored most of the room. |
| `is_asked_` | `int` | Counts how many more times the room can be queried (default 2, decremented). Used by "early stop" logic. |
| `voxel_num_` | `int` | Number of covered voxels assigned to this room (refreshed every `UpdateRoomLabel`). |
| `anchor_point_` | `Point` | Camera location where the VLM was asked about this room — also the navigation goal for "enter this room". |
| `image_` | `cv::Mat` | The cropped camera frame sent to the VLM for room typing. |
| `last_area_` | `float` | Area at the moment the room was last (re-)queried, used to decide when to re-ask. |

**Written by the planner (graph crosslinks):**
| Field | Type | Meaning |
|---|---|---|
| `viewpoint_indices_` | `set<int>` | Viewpoints whose `room_id_` is this room. Maintained bidirectionally by `SetViewpointRoomRelation`. |
| `object_indices_` | `set<int>` | Objects assigned to this room. Maintained bidirectionally by `SetObjectRoomRelation`. |

### `ObjectNodeRep` — a persistent semantic object (line 319, header)

**Written by `semantic_mapping_node` (via `ObjectNode` → `UpdateObjectNode(msg)` at `representation.cpp:113`):**
| Field | Type | Meaning |
|---|---|---|
| `object_id_` | `vector<int>` | All tracker ids merged into this object (semantic mapping concatenates ids on same-class merges). `GetObjectId()` returns `[0]` — the "primary" id used as map key. |
| `label_` | `string` | Dominant class label, or `"Potential Target"` for unverified target candidates. |
| `confidence_` | `double` | (declared, populated by ctor only) |
| `position_` | `Point` | Centroid (`infer_centroid` of regularized voxels). |
| `bbox3d_` | `array<Point, 8>` | Oriented 3D box corners. |
| `cloud_` | `sensor_msgs/PointCloud2` | Per-object voxel cloud. |
| `status_` | `bool` | True = upsert; **false-status messages never reach here** — they take the `object_ids_to_remove_` path in `ObjectNodeListCallback` and trigger erase from `object_node_rep_map_` later. |
| `timestamp_` | `rclcpp::Time` | Detection timestamp. |
| `img_path_` | `string` | Disk path to best masked crop, for downstream VLM. |
| `is_asked_vlm_` | `bool` | True once VLM has answered for this object. |

**Written by the planner:**
| Field | Type | Meaning |
|---|---|---|
| `room_id_` | `int` | Room containing the object. Maintained by `SetObjectRoomRelation` in `UpdateObjectVisibility`. `-1` until first assignment. |
| `visible_viewpoint_indices_` | `set<int>` | Viewpoints from which this object is visible (ray-cast or direct). |
| `is_considered_` / `is_considered_strong_` | `bool` | "Already inspected as a target candidate" flags. Reset to false on new `target_object_instruction`. |
| `voxels_` | `vector<Vector3i>` | (declared, not used in the current code path) |

`object_id_[0]` is the primary handle used everywhere — `object_node_rep_map_`'s key, `visible_viewpoint_indices_` entries, etc.

### Top-level fields on `Representation`

```cpp
rclcpp::Node::SharedPtr                       nh_;
std::vector<ViewPointRep>                     viewpoint_reps_;
std::map<int, RoomNodeRep>                    room_nodes_map_;       // ordered for deterministic iteration
std::unordered_map<int, ObjectNodeRep>        object_node_rep_map_;  // O(1) lookup by id
std::set<int>                                 latest_object_node_indices_;
pcl::PointCloud<pcl::PointXYZ>::Ptr           viewpoint_rep_vis_cloud_;
pcl::PointCloud<pcl::PointXYZI>::Ptr          covered_points_all_;
```

`viewpoint_rep_vis_cloud_` is appended-to **only** inside `AddViewPointRepNode` (one `PointXYZ` per viewpoint) — the planner publishes it via `viewpoint_rep_vis_cloud_->Publish()` at the end of `UpdateViewpointRep`. `covered_points_all_` accumulates each viewpoint's `covered_cloud_` at creation time.

---

## How the graph gets populated — three feeders

### Feeder 1: `semantic_mapping_node` → objects
`/object_nodes_list` → `SensorCoveragePlanner3D::ObjectNodeListCallback` (`sensor_coverage_planner_ground.cpp:946`):

```cpp
for (const auto& node : msg->nodes) {
  if (node.status == false) {                     // delete
    for (auto obj_id : node.object_id) {
      if (obj_id == found_object_id_) continue;   // never remove the active target
      object_ids_to_remove_.push_back(obj_id);
    }
    continue;
  }
  if (node.cloud.data.empty()) continue;          // skip empty
  representation_->UpdateObjectNode(node_ptr);
  representation_->GetLatestObjectNodeIndicesMutable().insert(node.object_id[0]);
}
```

So adds/updates flow directly into `object_node_rep_map_[id]` via `UpdateObjectNode` (which also stitches the viewpoint↔object back-link if `msg->viewpoint_id >= 0`). Deletes are *deferred* into `object_ids_to_remove_` and processed later in the planning loop (around line 4888: `representation_->GetObjectNodeRepMapMutable().erase(obj_id)` after cleaning up the object's links).

Every entered/updated id also lands in `latest_object_node_indices_` — a per-cycle work queue consumed by `UpdateObjectVisibility`.

### Feeder 2: `room_segmentation` → rooms
`/room_nodes_list` → `RoomNodeListCallback` (line 1062). The pattern is "mark all dead, ingest, sweep":

```cpp
for (auto &kv : representation_->GetRoomNodesMapMutable()) kv.second.SetAlive(false);
for (const auto &room_node_msg : room_node_list_msg->nodes) {
  if (!representation_->HasRoomNode(room_node_msg.id))
    representation_->AddRoomNode(room_node_msg);            // new room
  else
    representation_->GetRoomNode(room_node_msg.id).UpdateRoomNode(room_node_msg);
}
for (auto it = ...; ) {
  if (!it->second.IsAlive()) it = ...erase(it);             // gone-away rooms
  else ++it;
}
```

So "this room disappeared" is signalled implicitly — by simply not being in the latest list. After the sweep, the planner also validates each labeled room's `anchor_point_` still falls inside the (potentially re-shaped) room mask; if not, the room's VLM labels are cleared (lines 1100-1126).

Then `/room_mask` arrives separately and triggers `RoomMaskCallback` (line 1129), which calls `representation_->UpdateViewpointRoomIdsFromMask(room_mask_, shift_, room_resolution_)` — re-binds every viewpoint to its current room. This makes the viewpoint↔room link automatically rebalance when rooms split, merge, or get renumbered.

### Feeder 3: planner itself → viewpoints
`SensorCoveragePlanner3D::UpdateViewpointRep` (line 3726, called every cycle):

A new viewpoint is added when **either** of these fires:
- **Coverage drift**: `intersect(current_obs_voxel_inds_, previous_obs_voxel_inds_)` is small both absolutely (`< rep_threshold_voxel_num_`) and as a fraction of the current observation (`< rep_threshold_`, default 0.1).
- **Object pressure**: `obj_score_ > 4.0`, where `obj_score_` is built in `UpdateObjectVisibility` from how many objects are newly-visible from the current robot position (`+1.0` if no viewpoint has seen them, `+0.2` if already-seen).

`AddViewPointRep` also enforces a **2.0 m proximity floor**: if any existing viewpoint is within 2 m, no new node is created and the closest existing index is returned. `/viewpoint_rep_header` is only published when a *fresh* viewpoint actually got added (`prev_size != curr_size`) — that header is the signal to `semantic_mapping_node` to align one mapping cycle to this viewpoint id.

On a successful add the planner also:
- Sets `room_id_ = current_room_id_`.
- Calls `planning_env_->UpdateCoveredVoxels(origin)`.
- Resets `previous_obs_voxel_inds_` (baseline for next drift check) and clears `latest_object_node_indices_`.

---

## Relations and how they're maintained

There are three relation kinds. All are kept bidirectional. Helper functions live on `Representation`:

### Viewpoint ↔ Room (`SetViewpointRoomRelation`, line 397)
Removes the viewpoint id from the old room's `viewpoint_indices_`, sets `viewpoint.room_id_ = new_room_id`, inserts into the new room's `viewpoint_indices_`. Called from `UpdateViewpointRoomIdsFromMask` (one viewpoint at a time, every `/room_mask` message). Idempotent — calling with `old == new` is a no-op.

### Object ↔ Room (`SetObjectRoomRelation`, line 377)
Same pattern: drop from old room's `object_indices_`, set `object.room_id_`, insert into new room's `object_indices_`. Called from `UpdateObjectVisibility` (line 2341) for every object touched this cycle. The planner first looks up the object's position in `room_mask_`; if the lookup returns 0 (the object's centroid landed on a border / unmapped pixel), it dilates by 2 voxels and uses the first non-zero room id found. Objects whose dilated lookup is still 0 get `room_id_ = -1`.

### Object ↔ Viewpoint
This is the most complex link, written in three places:

1. **`UpdateObjectNode`** (`representation.cpp:215`). When `semantic_mapping_node` stamps `viewpoint_id` on its `ObjectNode` message, the planner adds the object to `viewpoint.direct_object_indices_` (and to `viewpoint.object_indices_` implicitly via the equivalent visibility check next), and adds the viewpoint id to `object.visible_viewpoint_indices_`. This is the *direct* link — "the camera frame aligned to this viewpoint actually contained this detection".

2. **`UpdateObjectVisibility`** (planner line 2200). For every object in `latest_object_node_indices_`, walk every viewpoint:
   - If the viewpoint already lists this object → idempotent re-affirm.
   - Else if the object's `visible_viewpoint_indices_` already contains this viewpoint → trust the prior link (no ray-cast).
   - Else if the viewpoint is within `rep_sensor_range` → run `CheckLineOfSightInOccupancyGrid` against each voxel of the object's cloud; the first that's reachable promotes the pair.
   Hits go into `viewpoint.object_indices_` only (NOT into `direct_object_indices_`).

3. **`UpdateViewpointObjectVisibility`** (planner line 2401). Called once at the end of the cycle if a new viewpoint was just added — ray-casts the fresh viewpoint against *every* existing object. Fills in the new viewpoint's `object_indices_`.

After visibility runs, any object id that was paired with at least one viewpoint is erased from `latest_object_node_indices_`. So an object reappears in that set every time semantic mapping publishes a new update for it.

---

## The per-cycle update sequence

The orchestration lives in the main planning loop, starting at `execute()` (line 3510). When `keypose_cloud_update_` fires:

```
UpdateRoomLabel()                  // walks covered cloud, assigns to rooms via room_mask_;
                                   // emits /room_type_query for unlabeled rooms with image+anchor.

SetCurrentRoomId()                 // updates current_room_id_ from room_mask_ at the robot pose.

(room transit / arrival bookkeeping)

UpdateObjectVisibility()           // object↔viewpoint ray-casts + obj_room_relation;
                                   // builds obj_score_ for the next step.

UpdateGlobalRepresentation()       // grid_world_ accounting (separate subsystem).

UpdateViewpointRep()               // decides whether to AddViewPointRep this cycle.

PublishViewpointRoomIdMarkers()    // RViz markers from viewpoint_reps_.

if (add_viewpoint_rep_)
  UpdateViewpointObjectVisibility()  // populates the brand-new viewpoint's object_indices_.

CreateVisibilityMarkers()          // line markers viewpoint → object for each direct/indirect link.

UpdateViewPoints()                 // viewpoint manager + local coverage planner uses
                                   // representation_->GetViewPointReps() implicitly.
```

Other callbacks plug in asynchronously:
- `ObjectNodeListCallback` writes to `object_node_rep_map_` / queues deletes.
- `RoomNodeListCallback` writes/sweeps `room_nodes_map_`.
- `RoomMaskCallback` re-binds viewpoints to rooms.
- `RoomTypeAnswerCallback` (line 1235): `room_node.labels_[room_type] += voxel_num`, and if the argmax changed, marks all objects in that room as `is_considered_=false` so they'll be re-evaluated.
- `RoomNavigationAnswerCallback` (line 1261): reads `room_node.anchor_point_` to set a goal, or reads a candidate room's anchor as the next exploration destination.
- `TargetObjectInstructionCallback` (line 1342): when the operator names a new target, clears `is_considered*` on every object and resets `is_asked_=2` on every room.
- `TargetObjectCallback` (line 1371) / `AnchorObjectCallback`: reads `representation_->GetObjectNodeRep(id).GetPosition()` / `.room_id_` to set `found_object_*` state.

---

## How the planner consumes the representation

A non-exhaustive map of who reads what:

| Consumer | Reads | For |
|---|---|---|
| `viewpoint_manager_`, `local_coverage_planner_` | `viewpoint_reps_` (positions, clouds, covered clouds) | Candidate viewpoint sampling and coverage scoring. |
| `grid_world_` / global coverage | `room_nodes_map_` | Room-level coverage state, used to gate exploration termination. |
| `UpdateRoomLabel` | `room_nodes_map_`, `room_mask_` | Decides which rooms still need to be VLM-typed; publishes `/room_type_query`. |
| `UpdateObjectVisibility` | `object_node_rep_map_`, `viewpoint_reps_`, `room_mask_` | Maintains all three relation types each cycle. |
| `RoomNavigationAnswerCallback`, `ChangeRoomQuery` | `room_node.anchor_point_`, `room_node.GetObjectIndices()`, `IsCovered/IsVisited/IsLabeled` | Picks the next room to enter / when to ask the VLM about the next room. |
| `TargetObjectCallback`, `AnchorObjectCallback` | `object_node_rep_map_[id].position_`, `.room_id_` | Sets the active target / anchor to navigate to. |
| `KeyboardInputCallback "next"` | `object_node_rep_map_[found_object_id_]` | Marks the current found object as fully considered, freeing the planner to look for the next instance. |
| `CreateVisibilityMarkers`, `PublishViewpointRoomIdMarkers` | `viewpoint_reps_` (`object_indices_`, `room_id_`) | RViz visualization of the live scene graph. |
| `viewpoint_rep_vis_cloud_`, `covered_points_all_` | (managed by `Representation` itself) | Coverage visualisation. |

Plus reads scattered throughout the planning loop:
- `representation_->GetRoomNode(id).IsCovered()`, `.IsLabeled()`, `.GetObjectIndices()` — gating logic for room-by-room exploration.
- `representation_->GetObjectNodeRep(id).IsConsideredStrong()` — skipping objects that were already inspected as candidates.
- `representation_->GetRoomNode(id).SetIsVisited(true)` — checked off when the robot enters.

So the planner's high-level loop is essentially: **"look at the graph, decide what to ask the VLM and where to go next, execute, write the result back into the graph."**

---

## API surface (one-line cheatsheet)

```cpp
// ViewPoints
int     AddViewPointRep(pos, cloud, covered_cloud, t);   // returns new or nearest-existing index
ViewPointRep& GetViewPointRepNode(int index);
Point   GetViewPointRepNodePos(int index) const;
size_t  GetViewPointCount() const;
auto&   GetViewPointReps[Mutable]() const;
auto    GetViewPointRepCloud() const;       // pcl::PointCloud<XYZ>::Ptr, one point per viewpoint
auto    GetCoveredPointsAllCloud() const;   // union of every viewpoint's covered_cloud_

// Objects
void    UpdateObjectNode(ObjectNode::ConstSharedPtr msg);   // upsert; also stitches viewpoint↔object
ObjectNodeRep& GetObjectNodeRep(int object_id);
bool    HasObjectNode(int object_id) const;
size_t  GetObjectNodeCount() const;
auto&   GetObjectNodeRepMap[Mutable]() const;
auto&   GetLatestObjectNodeIndices[Mutable]() const;        // per-cycle work queue

// Rooms
RoomNodeRep& AddRoomNode(const tare_planner::msg::RoomNode& msg);
RoomNodeRep& GetRoomNode(int room_id);
bool    HasRoomNode(int room_id) const;
size_t  GetRoomNodeCount() const;
auto&   GetRoomNodesMap[Mutable]() const;

// Relations (bidirectional)
void    SetObjectRoomRelation(int object_id, int new_room_id);
void    SetViewpointRoomRelation(int viewpoint_id, int new_room_id);
void    UpdateViewpointRoomIdsFromMask(const cv::Mat& mask, const Vector3f& shift, float resolution);

// Serialization
std::string ToJSON() const;   // **STUB** — declared but not implemented (representation.cpp:209-213)
```

---

## Invariants and gotchas

1. **`viewpoint.id_ == index in viewpoint_reps_`.** Baked in at creation, assumed everywhere. Never erase or insert mid-vector. If you ever need viewpoint deletion, the `id_` field must be decoupled from the index and every downstream lookup updated.

2. **Room ids are monotonic and never reused** (`room_segmentation` allocates them via a counter). A deleted-then-recreated room is a *different* room with a fresh id. The planner relies on this for cross-cycle stability of `room_id_` on viewpoints/objects.

3. **`room_node.show_id_` is not stable** — `room_segmentation` renumbers it every cycle for visualisation. The planner uses `id_` everywhere internally.

4. **`object_node_rep_map_` is keyed by `object_id_[0]`**, but `object_id_` is a vector because semantic mapping merges tracker ids on same-class merges. After a merge the object node arrives with a longer `object_id_` list but the primary key stays the same.

5. **`status=false` ObjectNode messages don't reach `Representation`** — they're intercepted in `ObjectNodeListCallback` and processed as deferred deletes via `object_ids_to_remove_`. So `UpdateObjectNode` only ever sees upserts.

6. **Room "deletion" is implicit.** A room is gone when it's missing from the next `/room_nodes_list`. The planner marks everyone dead, ingests, then sweeps non-alive. There is no per-message delete flag.

7. **`/viewpoint_rep_header` is only published on a real add**, never on a proximity-dedup hit. If you grep for `viewpoint_rep_pub_->publish` you'll find exactly one site, behind `if (prev_size != curr_size)`. Don't expect a header per `UpdateViewpointRep` call.

8. **`direct_object_indices_` is a strict subset of `object_indices_`.** A direct detection always promotes both; a ray-cast hit promotes only the indirect one. This distinction matters for the VLM pipeline — you generally want to query a viewpoint whose `direct_object_indices_` contains the target, because that's the viewpoint whose camera frame has the best masked crop.

9. **`ToJSON()` is unimplemented** (`representation.cpp:209-213`). If you want to dump the scene graph for offline analysis, you need to either implement it or iterate the public collections from outside.

10. **No locks.** `Representation` is single-threaded; everything happens on the planner's single executor thread. Don't share `Representation` across threads without adding synchronization.

11. **The implicit identity `viewpoint_reps_[i].id_ == i`** is also assumed by `viewpoint_rep_vis_cloud_->points[i]` — they're appended in lockstep inside `AddViewPointRepNode`. Same caveat about not erasing mid-vector applies to that cloud.

---

## When something looks wrong

- **Objects without a room (`room_id_ == -1`)** → centroid landed outside `room_mask_` or on a 0-pixel even after dilation. Often because the room mask hasn't been published yet for the area where the object lives — wait a cycle. If persistent, the object centroid is in unmapped territory (free-space gap or off-grid).
- **Viewpoint with stale `room_id_`** → check that `/room_mask` is being delivered; `UpdateViewpointRoomIdsFromMask` is the only place that re-binds them after creation.
- **Same physical object appears twice** → not a representation bug. `Representation` keys strictly by `object_id_[0]`; duplicates mean semantic mapping's merging didn't fire (see semantic_mapping README §B).
- **A room never gets typed** → the room hasn't accumulated enough covered voxels for `UpdateRoomLabel` to send a `/room_type_query`, or `is_asked_` already counted down to 0. `room_counts[room_id] > 100` is the gate in `UpdateRoomLabel`.
- **Target object gets erased** → check that the delete-protection at `ObjectNodeListCallback:980` (`if (obj_id == found_object_id_) continue;`) is firing. If `found_object_id_` was reset between detection and deletion, the protection misses.
- **Visibility markers show wrong objects per viewpoint** → that's the live state of `viewpoint.object_indices_`. To debug a specific viewpoint, log its `GetObjectIndices()` and compare against the result of `CheckLineOfSightInOccupancyGrid` for each object — most issues are stale ray-casts after the occupancy grid shifted, not bugs in `Representation` itself.
