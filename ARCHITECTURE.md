> **OUT OF DATE (2026-08-25).** Describes the `deepclean` pipeline. On this branch
> the NavGraph, JSON exporter and quadrant/room-area tagging have been removed
> (see [`RSB_TEST_PLAN.md`](RSB_TEST_PLAN.md), Phase 1); the room-labeling and
> producer sections will change again in Phases 2-3. Rewritten in Phase 6.

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
2. **Free space is mapped as the robot moves** (the robot is driven externally —
   its own onboard planner, teleop, or a bag replay; this stack does **not**
   steer). The pipeline drops **viewpoints** (observation keyframes) and
   maintains a **keypose graph** (a traversability roadmap).
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
 sensors ─► viewpoints + keypose graph ─────────┼─► Representation ──┐
 sensors ─► room segmentation + VLM typing + doors ─────┘  (the scene graph) ├─► exporter ─► scene_graph.json
                          keypose graph ─► NavGraph (sparse waypoints+edges) ─┘
```

---

## Repository layout

ROS 2 workspace; packages under `src/`:

| Package | Role |
|---|---|
| `exploration_planner/tare_planner` | **The heart of this pipeline.** The scene-graph builder (grown out of the TARE planner, steering removed) + keypose graph + NavGraph + grid world + room segmentation + the `Representation` (scene graph) + the scene-graph exporter. Most of this guide lives here. |
| `semantic_mapping` | 3D semantic **object** mapping: YOLO-World/NanoOWL detection + SAM2 + LiDAR-image fusion → persistent per-instance object clouds → object nodes. See its [README](src/semantic_mapping/README.md). |
| `vlm_node` | Vision-Language reasoning: **room typing** (labels rooms) and object-label verification for `semantic_mapping`. |
| `utilities` | Support: ROS-TCP endpoint, RViz overlay plugins, and other tooling (pending its own audit). |

> **Scope note.** This branch is the **scene-graph-construction-only** reduction
> of SysNav: navigation execution (the TARE steering outputs, `route_planner`,
> `base_autonomy`) and the in-repo SLAM (`arise_slam`) have been **removed**. The
> robot is driven by its own onboard planner (or teleop / bag replay), and every
> node consumes the robot/bag's own registered cloud + odometry
> (`/<ns>/cloud_registered` + `/<ns>/lio/odometry`). The reduction worklist and
> its verification history live in [`EXTRACTION_AUDIT.md`](EXTRACTION_AUDIT.md).

---

## Coordinate frames (read this before trusting any coordinate)

Everything in the scene graph lives in a **single `map` frame**
(`kWorldFrameID = "map"`), anchored at the **robot's start pose** — so the first
pose is the origin `(0,0,0)`. The frame is **gravity-aligned**. There is no
`map`→`odom` split inside the graph: room centroids, object positions, viewpoint
positions, door pixels, and keypose-graph nodes are all directly comparable in
this one frame.

A separate wrinkle exists only at **export** time: by default coordinates are
emitted in the bag's **odom** frame. To express them in a building-fixed `world`
frame instead, the exporter applies a **single static** `world_T_odom` transform
looked up once from tf (`source_frame` defaults to `map` = `kWorldFrameID`). There
is **no gravity bridge and no per-run constant** — the standardized direct-LIO
stack publishes one coherent TF tree, so a single lookup suffices. If you don't
need world-frame output, leave it off and the JSON stays in `odom`. Details +
caveats:
[`scene_graph_exporter/README.md`](src/exploration_planner/tare_planner/src/scene_graph_exporter/README.md).

> ⚠️ The single export-time transform is exact **only because `world ← odom` is
> static** per bag (no online global relocalization mid-run). Many bags have no
> `world` frame at all — there the export falls back to `odom`, and
> `layout.metadata.frame` is set to `odom` to match.

In the bag-direct setup the bag's own LIO feeds `/<ns>/lio/odometry` +
`/<ns>/cloud_registered` directly, and a static identity `<ns>/odom → map` pins
the stack's `map` frame to the bag odom. The scene graph is therefore in odom; the
exporter sets `layout.metadata.frame` automatically to `odom` (or `world`, when
the optional `world_transform` is enabled and its `world ← odom` lookup
succeeds).

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

**2. Viewpoints — the planner node**
The planner node samples candidate viewpoints as the robot moves and, in
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
See [`room_segmentation/README.md`](src/exploration_planner/tare_planner/src/room_segmentation/README.md)
for the (purely geometric) mask, and
[`sensor_coverage_planner/ROOM_LABELING.md`](src/exploration_planner/tare_planner/src/sensor_coverage_planner/ROOM_LABELING.md)
for how rooms get *typed* (view-image admission → VLM query → answer applied).

**4. The keypose graph — the planner node (parallel, mostly independent)**
A global topological **roadmap** of the walkable world, built from the robot's
trajectory plus connector waypoints (injected by `grid_world` from candidate
viewpoints). It is **not** part of the scene graph, but it is the **source the
NavGraph contracts** (item 5) — which is why the connector-node machinery
survives in the reduced pipeline.
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
and the stale/orphaned (often edgeless) majority is the other. The **connected**
component is exactly what the **NavGraph** contracts into the scene graph's
navigation layer. Read
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
  transform). Default world transform is off (coordinates stay in odom).

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
| Rooms / doors / the room mask (geometry) | [`room_segmentation/README.md`](src/exploration_planner/tare_planner/src/room_segmentation/README.md) |
| Room *type* labeling (views → VLM → apply) | [`sensor_coverage_planner/ROOM_LABELING.md`](src/exploration_planner/tare_planner/src/sensor_coverage_planner/ROOM_LABELING.md), `vlm_node/` |
| 3D objects / detection | [`semantic_mapping/README.md`](src/semantic_mapping/README.md) |
| Traversability roadmap / routing distances | [`keypose_graph/README.md`](src/exploration_planner/tare_planner/src/keypose_graph/README.md) |
| The exported waypoint/navigation graph (for the LLM) | [`navgraph/README.md`](src/exploration_planner/tare_planner/src/navgraph/README.md) |
| The in-memory scene graph itself | `src/exploration_planner/tare_planner/src/representation/` |
| The node that owns & wires it all | `src/exploration_planner/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` |
