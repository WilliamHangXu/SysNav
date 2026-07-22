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
| `config/scene_graph_export.yaml` | Runtime config (identifiers, save cadence, world transform). Loaded by `tare_planner_node`. World transform defaults off => coordinates in the bag odom frame. |

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
| `world_from_source` | `Eigen::Isometry3d` | Rigid odom→world transform applied to **every** emitted coordinate. Identity ⇒ coordinates stay in the `odom` frame. |

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

A fourth, snapshot-adjacent trigger shares the channel: publishing
`keypose_dump_keyword` (default `"skg"`) on `/keyboard_input` calls
`SaveKeyposeGraphJson()`, which writes the raw **keypose graph** (not the scene
graph) to `<run_dir>/keypose_graph.json` — nodes as `[x,y,z]` indexed by
keypose `node_ind`, undirected edges as `[u,v,dist]` (`u < v`), and the
collision-pruned `connected` node set verbatim. It is the offline navgraph
builder's input, uses the same frame policy as snapshots, overwrites the same
stable filename on each press (the graph only grows), and never fires on its
own — production runs are unaffected.

### World transform (`odom` → `world`)

The scene graph is built entirely in the bag's **odom** frame (numerically
`kWorldFrameID = "map"`, identity-pinned to the bag odom). To express a snapshot
in a building-fixed **`world`** frame, the planner looks up the **single static
transform** `world_T_source = lookupTransform(world_frame, source_frame)` once and
freezes it (`TryFreezeWorldFromOdom`):

```
world_T_source   (= world ← odom; source_frame defaults to "map" = kWorldFrameID)
```

There is **no gravity bridge and no per-run constant** — the standardized
direct-LIO stack publishes one coherent TF tree, so a single lookup suffices. The
transform is retried at 1 Hz until tf is flowing, then the timer cancels. If it is
never available, snapshots are still written — **falling back to identity** (odom
frame), and `layout.metadata.frame` is set to `odom` accordingly (loud warning).

> The export-time single transform is exact **only because `world ← odom` is
> static** per bag (no online global relocalization during the run). Many bags
> (e.g. the go2w_016 multifloor_test_slam bag) have **no `world` frame at all** —
> there `enabled: false` and coordinates stay in odom.

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
| | `keypose_dump_keyword` | `/keyboard_input` string that dumps the keypose graph to `keypose_graph.json` (offline navgraph input) |
| World transform | `world_transform.enabled` | Re-express coordinates in `world`; else stay in `odom` |
| | `world_transform.world_frame` | Target (building-fixed) tf frame, no leading `/` |
| | `world_transform.source_frame` | Frame the scene-graph coords are in (default `map` = kWorldFrameID), no leading `/` |
| Identifiers | `zone`, `map_id`, `warehouse_id`, `name`, `client_id`, `uploaded_by` | Written verbatim into the JSON |
| Metadata | `units`, `building`, `floor_level`, `floor_id` | Written into `layout.metadata` |

> Only the **identifier / metadata / transform** fields reach `Build()`; the
> **behavior** fields are interpreted entirely by the planner glue.

---

## Gotchas

- **Snapshot semantics.** `Build()` reads the live `Representation` at call time;
  it copies nothing in advance. Each snapshot is an independent, self-contained
  view — later snapshots simply reflect a more-grown graph.
- **Identity fallback is labeled.** If `world_T_odom` isn't latched, the snapshot
  is written in the `odom` frame and `layout.metadata.frame` is set to `odom` (a
  warning is logged). World-frame and odom-frame snapshots are **not**
  coordinate-comparable, but the `frame` field always tells you which one it is.
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
bool TryFreezeWorldFromOdom();                            // look up & latch world_T_odom
void SceneGraphWatchdogCallback();                        // end-of-bag final snapshot
```

| Want to… | Look at |
|---|---|
| Change what fields land in the JSON | `BuildRoomJson` / `BuildObjectJson` / `Build` (`scene_graph_exporter.cpp`) |
| Change *when* snapshots are written | timers + `SaveSceneGraphSnapshot` (`sensor_coverage_planner_ground.cpp:611`, `:4645`) |
| Change the output frame | `world_transform.*` in `scene_graph_export.yaml` + `TryFreezeWorldFromOdom` |
| Trigger a dump by hand | publish `manual_save_keyword` (default `"ssg"`) on `/keyboard_input` |
| Dump the keypose graph for the offline navgraph | publish `keypose_dump_keyword` (default `"skg"`) on `/keyboard_input` → `keypose_graph.json` |
| Find where rooms/objects come from | `Representation` (`src/representation/`), fed by room segmentation + semantic mapping |
| Find where waypoints/edges come from | [`NavGraph`](../navgraph/README.md) (`src/navgraph/`), contracted from the keypose graph |
```
