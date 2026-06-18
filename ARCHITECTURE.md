# SysNav — Architecture Guide (Scene-Graph Pipeline)

> **What this document is.** A developer/coding-agent onboarding guide to the
> part of SysNav that **builds a semantic scene graph and exports it as JSON**.
> The root [`README.md`](README.md) is the paper landing page (SysNav is a full
> three-level object-navigation system); *this* file is the practical map of how
> the scene graph is structured, constructed, and written out. If your task
> touches rooms / objects / viewpoints / doors / the exported JSON / the keypose
> graph, start here.

---

## TL;DR mental model

You run the stack against sensor input (a recorded **rosbag** of camera + LiDAR +
odometry, or a live robot). As the robot moves, three things happen in parallel
and feed one shared, growing in-memory structure called the **`Representation`**
(the scene graph):

1. **Objects** are detected in images, lifted to 3D, and tracked over time.
2. **Free space is explored** by a classical exploration planner (TARE), which
   also drops **viewpoints** (observation keyframes) and maintains a **keypose
   graph** (a traversability roadmap).
3. **Free space is segmented into rooms**, rooms are typed by a **VLM**, and
   **doors** between rooms are detected.

The `Representation` ties these together — *which objects and viewpoints belong to
which room, and how rooms connect* — and the **scene graph exporter** snapshots it
to a **GADM-style JSON** document on disk. **That JSON is the deliverable.**

A fourth structure, the **NavGraph**, is continuously derived from the keypose
graph as a *sparse, room-tagged, LLM-friendly* waypoint graph. It — **not** the
viewpoints — supplies the **waypoints and edges** in the exported JSON, giving the
downstream LLM a coarse navigation graph with real walking distances.

```
 sensors ─► detection ─► 3D semantic objects ─┐
 sensors ─► TARE explore ─► viewpoints + keypose graph ─┼─► Representation ──┐
 sensors ─► room segmentation + VLM typing + doors ─────┘  (the scene graph) ├─► exporter ─► scene_graph.json
                          keypose graph ─► NavGraph (sparse waypoints+edges) ─┘
```

---

## Repository layout

ROS 2 workspace; packages under `src/`:

| Package | Role |
|---|---|
| `exploration_planner/tare_planner` | **The heart of this pipeline.** TARE exploration planner + keypose graph + NavGraph + grid world + room segmentation + the `Representation` (scene graph) + the scene-graph exporter. Most of this guide lives here. |
| `semantic_mapping` | 3D semantic **object** mapping: YOLO-World/NanoOWL detection + SAM2 + LiDAR-image fusion → persistent per-instance object clouds → object nodes. See its [README](src/semantic_mapping/README.md). |
| `vlm_node` | Vision-Language reasoning: **room typing** (labels rooms), target-object / spatial-condition reasoning for navigation queries. |
| `slam` | `arise_slam` LiDAR-inertial odometry. Often **bypassed** when a bag already carries its own odometry/TF (see *Coordinate frames*). See its [README](src/slam/arise_slam_mid360/README.md). |
| `route_planner` | Far/near route planning between waypoints (navigation execution, not graph building). |
| `base_autonomy` | Low-level autonomy: local planner, terrain analysis, motion. The bottom of the three-level stack. |
| `utilities` | Support: `domain_bridge`, Livox driver, ROS-TCP endpoint, RViz overlay plugins. |

> **Scope note.** Navigation *execution* (route_planner, base_autonomy) is mostly
> orthogonal to building/exporting the scene graph. This guide focuses on the
> graph-building packages.

---

## Coordinate frames (read this before trusting any coordinate)

Everything in the scene graph lives in a **single `map` frame**
(`kWorldFrameID = "map"`), anchored at the **robot's start pose** — so the first
pose is the origin `(0,0,0)`. The frame is **gravity-aligned**. There is no
`map`→`odom` split inside the graph: room centroids, object positions, viewpoint
positions, door pixels, and keypose-graph nodes are all directly comparable in
this one frame.

A separate wrinkle exists only at **export** time: a recorded bag may publish its
own `world` TF tree that is **disjoint** from arise's `map` tree. The exporter can
optionally bridge them via the one physical object both trees see — the LiDAR —
composing `world_T_map = world_T_livox · G · (map_T_sensor)⁻¹`, where `G` is a
per-run **gravity rotation** (`R_GRAVITY`). This is **frozen once** and applied to
every emitted coordinate. If you don't need world-frame output, leave it off and
the JSON stays in the `map` frame. Details + caveats:
[`scene_graph_exporter/README.md`](src/exploration_planner/tare_planner/src/scene_graph_exporter/README.md).

> ⚠️ `G`/`R_GRAVITY` is a **brittle per-run constant** that must match the value
> arise used for *this* bag's IMU init. Wrong value ⇒ tilted/displaced world
> coordinates. For the bag-direct setup (no arise SLAM) the tree is already a
> single gravity-aligned tree and `G` is identity.

In the bag-direct setup, `bag_slam_bridge` decides which bag frame `map` is
pinned to via its `anchor_frame` parameter: `world` (default,
building-anchored — the bag's static `world←odom` is composed onto every LIO
pose) or `odom` (start-anchored: origin at the robot start, +x = its initial
heading). The scene graph and the exported JSON inherit that choice verbatim;
record it by setting `scene_graph_export.frame` to match, which is written into
`layout.metadata.frame`.

---

## The scene graph data model (`Representation`)

The live scene graph is the `Representation`
(`src/exploration_planner/tare_planner/src/representation/`). It holds four node
populations plus their relations:

| Node | Type | Key fields | Meaning |
|---|---|---|---|
| **Room** | `RoomNodeRep` | `id_`, label, `centroid_`, `polygon_`, `neighbors_`, `viewpoint_indices_`, object indices | A segmented region of free space. Owns the viewpoints and objects inside it; `neighbors_` are door-connected rooms. |
| **Object** | `ObjectNodeRep` | `object_id_`, `label_`, `position_`, `room_id_`, visible-viewpoints | A detected, 3D-localized, persistently-tracked object instance. |
| **Viewpoint** | `ViewPointRep` | `id_` (== index), position, timestamp, covered cloud, visible-objects, `room_id_` | An **observation keyframe**: a real pose where the camera captured an image and saw objects. *Not* a free-space waypoint. |
| **Door** | `door_cloud` (`PointXYZRGBL`) | `r`/`g` = the two room ids it joins; `x/y/z` = position | The opening between two rooms; its centroid becomes a room "entrance". |

Relations that make it a *graph*:

- **Room ↔ Viewpoint / Object** — membership, assigned by voxelizing positions
  into a room mask (`UpdateViewpointRoomIdsFromMask`).
- **Viewpoint ↔ Object** — bidirectional visibility (which objects were seen from
  which viewpoint), wired in `UpdateObjectNode`.
- **Room ↔ Room** — adjacency via `neighbors_`, geometrically realized by doors.

> **Navigation layer — built.** The "places" layer that was once planned now
> exists as the **NavGraph** (`src/navgraph/`): a sparse, room-tagged contraction
> of the keypose graph's connected component, exported as the per-room waypoints
> and the edges. It is a separate structure, not part of the `Representation`.
> See [`navgraph/README.md`](src/exploration_planner/tare_planner/src/navgraph/README.md).
>
> **Planned extensions (discussed, not yet built):**
> - **Areas** — subdivide each room into four quadrants by axes through its
>   centroid (system/map-frame aligned), and attach viewpoints/objects to the
>   *area* instead of the room. Pure exporter-side geometric bucketing.

---

## Construction pipeline (how the graph fills up)

All of the following run concurrently and write into the one `Representation`,
which is owned by the planner node (`SensorCoveragePlanner3D` in
`sensor_coverage_planner_ground.cpp`).

**1. Object layer — `semantic_mapping`**
```
camera image ─► detection_node (YOLO-World / NanoOWL + tracker) ─► DetectionResult
DetectionResult + odom + registered_scan ─► semantic_mapping_node
      └─ SAM2 mask ─ lift to 3D via LiDAR-image fusion ─ ObjMapper (merge/grow/prune)
      └─ publishes /object_nodes_list  ─────────────────────────────────►  Representation::UpdateObjectNode
```
Objects are persistent instances (deduplicated, grown with new views). Their node
messages carry the object id, label, 3D position, and the viewpoint id they were
seen from. See [`semantic_mapping/README.md`](src/semantic_mapping/README.md).

**2. Viewpoints — the planner**
The planner samples candidate viewpoints during exploration and, in
`UpdateViewpointRep`, commits a **scene-graph viewpoint at the robot's real pose**
when coverage changes substantially or object interest is high (deduped at ~2 m).
On commit it publishes `/viewpoint_rep_header`, which tells `semantic_mapping_node`
to grab the **closest camera frame** and save the image + camera transform for that
viewpoint. So every viewpoint is a genuine captured observation.

**3. Room layer — `room_segmentation` + VLM**
```
free-space / occupancy ─► room_segmentation ─► room nodes (+ room mask, + door cloud)
      └─ Representation::AddRoomNode / UpdateRoomNode  (rooms added/updated/erased over time)
      └─ /door_cloud  ─►  door_cloud_  (room-pair-tagged door pixels)
vlm_node ─► room *type* labels ─► RoomNodeRep label  (e.g. "kitchen")
```
Rooms get a lifecycle (created, updated, deleted as segmentation evolves).
Viewpoints and objects are re-binned into rooms via the room mask as it changes.
See [`room_segmentation/README.md`](src/exploration_planner/tare_planner/src/room_segmentation/README.md).

**4. The keypose graph — the planner (parallel, mostly independent)**
A global topological **roadmap** of the walkable world, built from the robot's
trajectory plus connector waypoints. It answers "traversable distance / path
between A and B" for the planner. It is **not** part of the scene graph (only
loosely coupled — the planner uses it for walking-distance queries to objects),
but it is the **source the NavGraph contracts** (item 5).
Full details: [`keypose_graph/README.md`](src/exploration_planner/tare_planner/src/keypose_graph/README.md).

**5. The NavGraph — the planner (derived from the keypose graph)**
Once per planning cycle the planner calls `navgraph_->Update(...)`, which rebuilds
a **sparse Voronoi contraction** of the keypose graph's connected component: a
distance-gated set of waypoints (nodes), edges meaning *reachability* with real
keypose walking distance, each node tagged with a room and a scene-graph waypoint
id. This is the structure the exporter turns into the JSON waypoints + edges.
Full details: [`navgraph/README.md`](src/exploration_planner/tare_planner/src/navgraph/README.md).

**6. Export — `scene_graph_exporter`**
On a timer / at end-of-bag / on manual trigger, the exporter snapshots the
`Representation` **plus the NavGraph** into GADM-style JSON.

---

## The keypose graph in one paragraph

An undirected graph over explored free space where an edge means "the robot can
travel between these two points in a straight line without hitting a known
obstacle." Two node kinds: **keypose nodes** (dropped along the trajectory) and
**non-keypose / connector nodes** (vertices of inter-cell roadmap paths). A
connectivity flood-fill from the first keypose marks each node *connected* or not;
in the `/keypose_graph_cloud` RViz cloud the **connected** component is one color
and the stale/orphaned (often edgeless) majority is the other. The planner only
routes over the **connected** component — which is also exactly what the
**NavGraph** contracts into the scene graph's navigation layer. Read
[`keypose_graph/README.md`](src/exploration_planner/tare_planner/src/keypose_graph/README.md)
before touching it.

---

## The exporter & the output JSON

The exporter is a **pure transformation** (no ROS deps) from `Representation`
state to `nlohmann::json`. Shape:

```jsonc
{
  "map_id": ..., "warehouse_id": ..., "name": ..., "client_id": ..., "uploaded_by": ...,
  "layout": {
    "zones": { "all": { "rooms": {
      "kitchen-room_1": {
        "type": "kitchen", "sgid": 1,
        "entrances": [ { "id", "connected_to", "x","y","z" } ],   // door centroids
        "waypoints": [ { "id": "...-wp_0" (centroid) }, { "...-wp_1" (NavGraph node) }, ... ],
        "objects":   [ { "object_id", "type", "sgid", "waypoint" } ]
      }
    } } },
    "waypoints": [ ...all waypoint ids... ],
    "edges":     [ { "u": "...-wp_1", "v": "...-wp_3", "meters": N } ],   // NavGraph edges
    "metadata":  { "units", "building", "floor_level", "floor_id", "dimensions" }
  }
}
```

- Snapshots are written under `output_root/<run>/` as `snapshot_<n>_<t>.json`
  (periodic / manual) and `snapshot_final.json` (end-of-bag).
- Triggers: **periodic** wall-clock timer, **end-of-bag** watchdog (sim time
  stalls), and **manual** (publish the keyword, default `"ssg"`, on
  `/keyboard_input`).
- Config: `config/scene_graph_export.yaml` (identifiers, cadence, world
  transform). Bag-direct variant: `config/scene_graph_export_bag_direct.yaml`.

Full schema, build logic, triggers, and gotchas:
[`scene_graph_exporter/README.md`](src/exploration_planner/tare_planner/src/scene_graph_exporter/README.md).

---

## Active & planned work (for the next session)

- **Room areas** — quadrant subdivision of each room (centroid axes in the
  system/map frame), with viewpoints/objects attached to areas instead of the
  room. Exporter-side, additive; needs a schema decision on nesting + empty-area
  handling, and the JSON *consumer* must learn the new `room → area → object`
  nesting.
- **Navigation/places layer — done.** Built as the **NavGraph**
  (`src/navgraph/`): a sparse contraction of the keypose graph's connected
  component, exported as the per-room waypoints (`wp_1..N`) and the `edges`. See
  [`navgraph/README.md`](src/exploration_planner/tare_planner/src/navgraph/README.md).

---

## Where to look next

| If you're working on… | Go to |
|---|---|
| The exported JSON shape / save logic | [`scene_graph_exporter/README.md`](src/exploration_planner/tare_planner/src/scene_graph_exporter/README.md) |
| Rooms / doors / room labels | [`room_segmentation/README.md`](src/exploration_planner/tare_planner/src/room_segmentation/README.md), `vlm_node/` |
| 3D objects / detection | [`semantic_mapping/README.md`](src/semantic_mapping/README.md) |
| Traversability roadmap / routing distances | [`keypose_graph/README.md`](src/exploration_planner/tare_planner/src/keypose_graph/README.md) |
| The exported waypoint/navigation graph (for the LLM) | [`navgraph/README.md`](src/exploration_planner/tare_planner/src/navgraph/README.md) |
| The in-memory scene graph itself | `src/exploration_planner/tare_planner/src/representation/` |
| The node that owns & wires it all | `src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` |
