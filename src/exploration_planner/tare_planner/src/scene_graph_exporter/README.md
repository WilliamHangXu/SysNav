# Scene Graph Exporter

The scene graph exporter serializes the planner's **in-memory semantic scene
graph** — rooms, the **NavGraph waypoints** and objects inside them, and the doors
between them — into a single **GADM-style JSON document** on disk. It is the
artifact that leaves the running system: a compact, frame-consistent description of
*what rooms exist, what is in them, how they connect, and where everything is*.

> **NavGraph, not viewpoints.** The per-room `waypoints` and the `edges` come from
> the [NavGraph](../navgraph/README.md) (a sparse contraction of the keypose
> graph), **not** from the `Representation`'s viewpoints. The exporter takes the
> NavGraph nodes/edges as plain data and reshapes them; it does not build them.

Mental model: the live `Representation` (rooms / objects / doors) plus the
`NavGraph` (waypoints + edges) are the source of truth the planner grows as it
explores; the exporter is a
**pure snapshot function** that freezes that graph at a moment in time, optionally
re-expresses every coordinate in a stable `world` frame, and writes it out as
JSON. It computes nothing semantic — no clustering, no segmentation — it only
*reshapes and relabels* what `Representation` already holds.

> **Pure transformation, no ROS.** `SceneGraphExporter::Build()` depends only on
> `Representation` data types, Eigen, PCL and `nlohmann::json`. It has **no**
> dependency on `rclcpp` or `SensorCoveragePlanner3D`, so it can be unit-tested in
> isolation. Everything stateful — ROS parameters, timers, TF, file I/O, the
> end-of-bag watchdog — lives on the **planner side** and is described in
> [Planner-side glue](#planner-side-glue) below.

---

## Files

| File | Contents |
|---|---|
| `include/scene_graph_exporter/scene_graph_exporter.h` | `SceneGraphExportConfig` struct, `SceneGraphExporter` class declaration |
| `src/scene_graph_exporter/scene_graph_exporter.cpp` | All JSON-building logic (`Build` and its helpers) |
| `config/scene_graph_export.yaml` | Runtime config (identifiers, save cadence, world transform). Loaded by `tare_planner_node` |
| `config/scene_graph_export_bag_direct.yaml` | Variant config for the bag-direct setup (no arise SLAM; single gravity-aligned TF tree) |

The exporter reads from the `Representation`
(`src/representation/representation.cpp`) and is **driven** by the main planner
(`src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp`), which owns the
`SceneGraphExporter` instance and decides *when* to call `Build()` and *where* to
write the result.

---

## Pipeline at a glance

```
           Representation (live)                       NavGraph (live)
            ┌──────────────┬───────────────┬──────────────┐  ┌──────────────┐
            │   rooms      │   objects     │  door_cloud  │  │ nodes + edges│
            │ (RoomNodeRep)│(ObjectNodeRep)│ XYZRGBL pcl  │  │ (NavNode/Edge)│
            └──────┬───────┴───────┬───────┴──────┬───────┘  └──────┬───────┘
                   │               │               │              │
   SaveSceneGraphSnapshot()  ── reads live state at call time ────┤
                   │                                              │
                   ▼                                              ▼
        world_from_map  (frozen once via TF, or identity)   passed into
                   │                                              │
                   └───────────────► SceneGraphExporter::Build() ◄┘
                                              │
                          per-room JSON  +  flat waypoint list  +  edges
                                              │
                                              ▼
                              snapshot_*.json  (output_root/<run>/)
```

Three things trigger a snapshot — a **periodic timer**, an **end-of-bag
watchdog**, and a **manual keyword** — all of which call the same
`SaveSceneGraphSnapshot()`. See [Save triggers](#save-triggers).

---

## What the exporter consumes

`Build()` takes five data containers plus one transform
(`scene_graph_exporter.h`):

| Parameter | Type | Meaning |
|---|---|---|
| `rooms` | `std::map<int, RoomNodeRep>` | Live rooms keyed by stable id. Ordered map ⇒ deterministic output order. |
| `objects` | `std::unordered_map<int, ObjectNodeRep>` | Live objects keyed by primary object id. |
| `nav_nodes` | `std::map<int, navgraph_ns::NavNode>` | NavGraph nodes keyed by stable id. Each carries a `room_id` (→ which room emits it) and a `name` (its `wp` id). |
| `nav_edges` | `std::vector<navgraph_ns::NavEdge>` | NavGraph edges (`u`, `v` node ids, `meters` = traversable distance) → `layout.edges`. |
| `door_cloud` | `pcl::PointCloud<pcl::PointXYZRGBL>` | One point per door pixel. `r`/`g` channels carry the **two room ids** the door joins (order-agnostic); `x/y/z` is its position. |
| `world_from_source` | `Eigen::Isometry3d` | Rigid map→world transform applied to **every** emitted coordinate. Identity ⇒ coordinates stay in the `map` frame. |

The relevant fields the exporter pulls:

- **`RoomNodeRep`** — `id_`, `GetRoomLabel()`, `centroid_` (room waypoint `wp_0`),
  `neighbors_` (connected room ids → **entrances**; note edges now come from the
  NavGraph, not `neighbors_`), `GetObjectIndices()` (objects in the room),
  `polygon_` (room outline → map dimensions).
- **`ObjectNodeRep`** — `object_id_` (vector; primary = `[0]`), `label_`.
- **`NavNode`** — `position`, `room_id` (groups nodes under rooms), `name`
  (emitted verbatim as the waypoint `id`). The ROS-free `NavNode`/`NavEdge` structs
  live in [`navgraph/navgraph_types.h`](../navgraph/navgraph_types.h), so the
  exporter stays rclcpp/PCL-free.

> Each reference is **validated** before use: a `neighbor_id` not in `rooms`, an
> `object_id` not in `objects`, or a NavGraph node/edge whose room is not alive is
> silently skipped (stale-link tolerance). The snapshot never dangles.

---

## Output schema (GADM-style JSON)

Top-level document (`Build`, `scene_graph_exporter.cpp:266`):

```jsonc
{
  "map_id": "HomeBuilding_test",
  "warehouse_id": "HomeBuilding_test",
  "name": "HomeBuilding_test",
  "client_id": "alphaz",
  "uploaded_by": "WilliamXu",
  "layout": {
    "zones": {
      "all": {                          // single zone bucket for every room
        "rooms": { "<room_key>": { ...room json... }, ... }
      }
    },
    "waypoints": [ "<room_key>-wp_0", "<room_key>-wp_1", ... ],   // flat list of *all* waypoint ids
    "edges":     [ { "u": "...-wp_0", "v": "...-wp_0", "meters": 4.2 }, ... ],
    "metadata": {
      "units": "meters",
      "building": "HomeBuilding",
      "floor_level": 1,
      "floor_id": "floor_04",
      "dimensions": { "width": 12.3, "height": 8.7 }   // world-space bbox over all room polygons
    }
  }
}
```

Per-room object (`BuildRoomJson`, `scene_graph_exporter.cpp:88`):

```jsonc
"kitchen-room_1": {                     // room_key = "<label>-room_<id>"
  "type": "kitchen",                    // GetRoomLabel(); "unknown" if untyped
  "sgid": 1,                            // room id_
  "entrances": [                        // one per door-adjacent neighbor that HAS door geometry
    {
      "id": "1_2_entrance_1",
      "connected_to": "hallway-room_2", // neighbor's room_key
      "x": ..., "y": ..., "z": ...      // door centroid, world frame
    }
  ],
  "waypoints": [                        // wp_0 = room centroid, wp_1..N = the room's NavGraph nodes
    { "id": "kitchen-room_1-wp_0", "x": ..., "y": ..., "z": ... },  // centroid
    { "id": "kitchen-room_1-wp_1", "x": ..., "y": ..., "z": ... }   // NavGraph node
  ],
  "objects": [                          // every object assigned to this room
    { "object_id": "refrigerator_7", "type": "refrigerator", "sgid": 7, "waypoint": {} }
  ]
}
```

Key naming conventions:

- **`room_key`** = `"<label>-room_<id>"`, e.g. `kitchen-room_1`; label falls back
  to `"unknown"`.
- **`wp_0`** is always the room **centroid**; `wp_1..N` are the room's NavGraph
  nodes in ascending node-id order. Each `wp` id is the NavGraph node's `name`,
  emitted verbatim so the RViz labels and the JSON match.
- **`object_id`** = `"<label>_<primary_id>"`; `sgid` is the numeric id.

---

## How `Build()` constructs the JSON

1. **Per room** (`BuildRoomJson`):
   - **Entrances** — for each `neighbor_id` in `room.neighbors_` that still
     exists, call `DoorCentroid()` to average the door pixels tagged with that
     room-pair `(r,g)`. No matching door pixels ⇒ no entrance for that neighbor.
   - **Waypoints** — emit `wp_0` from `room.centroid_`, then one waypoint per
     NavGraph node whose `room_id` is this room (grouped via `nodes_by_room`),
     using the node's `name` as the id and `position` as the coordinate.
   - **Objects** — emit one entry per id in `room.GetObjectIndices()` that exists
     in `objects` (`BuildObjectJson`).
2. **Flatten waypoints** — every room waypoint id is mirrored into the top-level
   `layout.waypoints` list (`Build`, `scene_graph_exporter.cpp:222`).
3. **Edges** — one per **NavGraph edge**: `{u, v, meters}` where `u`/`v` are the
   two nodes' waypoint ids (recorded while emitting room waypoints) and `meters`
   is the NavGraph edge weight (keypose-graph walking distance). An edge whose
   endpoint was not placed in an alive room is skipped. (The old room-centroid
   adjacency edges are no longer emitted.)
4. **Dimensions** — `ComputeDimensions()` takes the world-space XY bounding box
   over **every room polygon vertex** → `width`/`height`.
5. **Wrap** in the top-level identifiers + `layout.metadata` from the config.

> **Entrances vs. edges are different things now.** **Entrances** are per-room
> door geometry (from `neighbors_` + door pixels). **Edges** are the NavGraph's
> node-to-node reachability with real walking distances — a finer graph over the
> waypoints, no longer derived from `neighbors_`.

### Coordinate transform

Every coordinate written to the JSON — room centroids, NavGraph node positions,
door centroids, polygon extents — is passed through `ToWorld(world_from_source, …)`
(`scene_graph_exporter.cpp:20`). For door centroids, **each pixel is transformed
before averaging** so the centroid is correct in the world frame. Pass identity
to leave everything in the source (`map`) frame.

---

## Planner-side glue

Everything below lives in `sensor_coverage_planner_ground.cpp`, not in the
exporter.

### Lifecycle

- **Parameters** loaded into `scene_graph_cfg_` (`:264–322`) from
  `scene_graph_export.yaml`.
- **Construction** (`:611`): if `enabled`, build the `SceneGraphExporter`, create
  a per-run output directory under `output_root` (disabling export rather than
  crashing if that fails), and arm the timers.

### Save triggers

All three call `SaveSceneGraphSnapshot(reason)` (`:4645`), which reads the live
`Representation`, calls `Build()`, and writes `snapshot.dump(2)` to disk.

| Trigger | Mechanism | File name |
|---|---|---|
| **Periodic** | Wall-clock timer every `save_interval_s` (fires under sim time too) | `snapshot_<count>_<sec>.json` |
| **End-of-bag** | `SceneGraphWatchdogCallback` (`:4701`); only when `use_sim_time`. Arms after the sim clock first advances, fires once the clock **stalls** for `bag_end_timeout_s` | `snapshot_final.json` (written once) |
| **Manual** | `/keyboard_input` String equals `manual_save_keyword` (default `"ssg"`) (`:1484`) | `snapshot_<count>_<sec>.json` |

The watchdog's two-phase arming deliberately ignores the pre-playback window: a
clock that is `0` or held constant before the bag starts must **not** be mistaken
for "bag finished."

### World transform (`map` → `world`)

The scene graph lives in arise's gravity-aligned **`map`** frame, which is a
**disjoint TF tree** from the bag's **`world`** frame. They are bridged through
the one physical object both trees see — the LiDAR (`livox_frame` in the bag tree,
`sensor` in arise's tree) — composed in code (`TryFreezeWorldFromMap`, `:4590`):

```
world_T_map = world_T_livox · G · (map_T_sensor)⁻¹
```

where **`G` = the gravity rotation** (`gravity_matrix`, arise's per-run
`imu_laser_R_Gravity`). The transform is **looked up once and frozen** (retried at
1 Hz until both trees are flowing, then the timer cancels). If it is never
available, snapshots are still written — **silently falling back to identity**,
i.e. in the `map` frame.

> ⚠️ **`gravity_matrix` is a brittle per-run constant.** It must match the
> `R_GRAVITY` arise used for *this* bag/IMU init (the same value as
> `cloud_image_fusion.py`). A wrong matrix tilts/displaces every world-frame
> coordinate in the snapshot. Re-read it for any new bag, IMU init, or sensor
> remount. See the `object-mapping-and-map-frame` notes. The bag-direct setup
> uses a single gravity-aligned tree (identity `G`); see
> `scene_graph_export_bag_direct.yaml`.

---

## Configuration

Loaded from `config/scene_graph_export.yaml` (merged into `tare_planner_node`'s
parameters). Field meanings (`SceneGraphExportConfig`, `scene_graph_exporter.h:36`):

| Group | Key | Purpose |
|---|---|---|
| Behavior | `enabled` | Master on/off for the whole exporter |
| | `output_root` | Parent dir; a per-run subfolder is created inside |
| | `save_interval_s` | Periodic snapshot cadence; `<= 0` disables periodic saves |
| | `end_of_bag_save` | Enable the final-snapshot watchdog (sim time only) |
| | `bag_end_timeout_s` | Wall-clock stall before declaring "bag over" |
| | `manual_save_keyword` | `/keyboard_input` string that triggers an on-demand dump |
| World transform | `world_transform.enabled` | Re-express coordinates in `world`; else stay in `map` |
| | `world_transform.{world,livox,map,sensor}_frame` | Frame names for the two-tree bridge (no leading `/`) |
| | `world_transform.gravity_matrix` | Row-major 3×3 `G`; identity disables the rotation |
| Identifiers | `zone`, `map_id`, `warehouse_id`, `name`, `client_id`, `uploaded_by` | Written verbatim into the JSON |
| Metadata | `units`, `building`, `floor_level`, `floor_id` | Written into `layout.metadata` |

> Only the **identifier / metadata / transform** fields reach `Build()`; the
> **behavior** fields are interpreted entirely by the planner glue.

---

## Gotchas

- **Snapshot semantics.** `Build()` reads the live `Representation` at call time;
  it copies nothing in advance. Each snapshot is an independent, self-contained
  view — later snapshots simply reflect a more-grown graph.
- **Identity fallback is silent in the data.** If `world_T_map` isn't latched, the
  snapshot is written in the `map` frame (a warning is logged, but the JSON looks
  identical in shape). World-frame and map-frame snapshots are **not**
  coordinate-comparable.
- **`scene_graph_snapshot_count_` is shared** across periodic and manual saves, so
  numbered file names interleave; only `final` has a stable name.
- **Stale links vanish quietly.** Missing neighbor rooms, missing objects, or
  NavGraph nodes/edges with no alive room are skipped, not errored — a snapshot can
  legitimately omit a referenced neighbor/object/node.
- **End-of-bag save needs sim time.** The watchdog only arms when
  `use_sim_time == true`; under live/wall-clock operation there is no "final"
  snapshot, only periodic/manual ones.
- **Edges/waypoints are the NavGraph.** `edges` and `wp_1..N` are the NavGraph —
  a fine-grained traversability graph with real walking distances (see
  [`navgraph/README.md`](../navgraph/README.md)). The room centroid `wp_0` is an
  isolated waypoint that no edge touches.

---

## Quick reference

```cpp
// Pure build (no ROS): reshape live scene-graph state into JSON.
nlohmann::json SceneGraphExporter::Build(
    const std::map<int, RoomNodeRep>&             rooms,
    const std::unordered_map<int, ObjectNodeRep>& objects,
    const std::map<int, navgraph_ns::NavNode>&    nav_nodes,
    const std::vector<navgraph_ns::NavEdge>&      nav_edges,
    const pcl::PointCloud<pcl::PointXYZRGBL>&     door_cloud,
    const Eigen::Isometry3d& world_from_source = Identity) const;

// Planner-side entry points (sensor_coverage_planner_ground.cpp):
void SaveSceneGraphSnapshot(const std::string& reason);  // "periodic" | "final" | "manual"
bool TryFreezeWorldFromMap();                             // compose & latch world_T_map
void SceneGraphWatchdogCallback();                        // end-of-bag final snapshot
```

| Want to… | Look at |
|---|---|
| Change what fields land in the JSON | `BuildRoomJson` / `BuildObjectJson` / `Build` (`scene_graph_exporter.cpp`) |
| Change *when* snapshots are written | timers + `SaveSceneGraphSnapshot` (`sensor_coverage_planner_ground.cpp:611`, `:4645`) |
| Change the output frame | `world_transform.*` in `scene_graph_export.yaml` + `TryFreezeWorldFromMap` |
| Trigger a dump by hand | publish `manual_save_keyword` (default `"ssg"`) on `/keyboard_input` |
| Find where rooms/objects come from | `Representation` (`src/representation/`), fed by room segmentation + semantic mapping |
| Find where waypoints/edges come from | [`NavGraph`](../navgraph/README.md) (`src/navgraph/`), contracted from the keypose graph |
```
