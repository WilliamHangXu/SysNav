# NavGraph

The NavGraph is a **persistent, lightweight topological graph** maintained live
during a run as a **Voronoi contraction of the [keypose graph](../keypose_graph/README.md)**.
Where the keypose graph is a dense (~0.3–0.5 m spacing) traversability roadmap
the *planner* uses, the NavGraph is a sparse (~1.25 m spacing) version meant to be
**exported into the scene-graph JSON and handed to an LLM for path planning**:

- **Nodes** are a distance-spread subset of keypose-node positions — "in-room
  waypoints" — each tagged with a room and a scene-graph waypoint id.
- **Edges** mean *reachability* (you can walk from one node to the other), with
  weight = the keypose-graph shortest-path distance. They are **not** straight-line
  segments — there is zero geometry/collision checking in the NavGraph itself.

Mental model: take the keypose graph, drop a sparse set of representative nodes
over it, and collapse the dense mesh between them into single edges that carry the
true walking distance. The result is a "subway map" of the explored building.

> **It is a coarsened *view* of the keypose graph, computed read-only.** The dense
> keypose graph is never modified — the planner still routes on it. The NavGraph
> only *reads* the keypose graph (connected component + adjacency + shortest
> paths) each cycle and rebuilds itself. The single coupling added to the keypose
> graph is one read accessor, `KeyposeGraph::GetNodeNeighbors` (keypose_graph.h).

---

## Files

| File | Contents |
|---|---|
| `include/navgraph/navgraph_types.h` | `NavNode` / `NavEdge` POD structs. **ROS-free** (only `geometry_msgs::Point`) so the exporter can consume them without pulling in rclcpp/PCL. |
| `include/navgraph/navgraph.h` | `NavGraph` class declaration. |
| `src/navgraph/navgraph.cpp` | All construction / maintenance / naming / visualization logic. |

Owned and driven by the main planner
(`SensorCoveragePlanner3D` in `sensor_coverage_planner_ground.cpp`): member
`navgraph_` (`.h:204`), constructed in `InitializeData` (`.cpp:436`), updated once
per planning cycle (`.cpp:3909`). Consumed by the
[scene graph exporter](../scene_graph_exporter/README.md) (`.cpp:5227`).

---

## Where it sits in the pipeline

```
                 KeyposeGraph (dense, planner's roadmap)
                          │   read-only each cycle
   GetConnectedGraphNodeIndices / GetNodePosition /
   GetNodeNeighbors / GetShortestPath
                          ▼
   execute(): after UpdateKeyposeGraph()  ──►  navgraph_->Update(...)
                          │
            ┌─────────────┴───────────────────────────────────┐
            │  Reconcile():  seed → label → delete → edges     │  (throttled
            │  TagRooms():   voxelize into room_mask           │   every N
            │  AssignNames(): "<room_key>-wp_<n>"              │   calls)
            │  PublishVisualization(): nodes/edges/labels      │
            └─────────────┬───────────────────────────────────┘
                          ▼
        GetNodes() / GetEdges()  ──►  SceneGraphExporter::Build()  ──►  JSON
                          ▼
        RViz: navgraph/{node_marker, edge_marker, label_marker}
```

---

## Core data structures

### `NavNode` (navgraph_types.h)

| Field | Meaning |
|---|---|
| `id` | Stable, monotonic, **never reused** node id. |
| `position` | xyz, **copied from a keypose node** at birth and frozen forever (never moved/re-centered). |
| `room_id` | Room affiliation (`-1` = unknown), re-tagged each pass from the room mask. |
| `name` | Scene-graph waypoint id, e.g. `"kitchen-room_1-wp_3"`. Exactly the id the exporter emits, so RViz labels and the JSON stay identical. Empty if the node has no room. |

### `NavEdge` (navgraph_types.h)

| Field | Meaning |
|---|---|
| `u`, `v` | The two node ids (canonical `u < v`). |
| `meters` | Keypose-graph shortest-path distance between the two nodes (honest traversable distance, summed keypose edge lengths). |

### `NavGraph` (navgraph.h)

| Member | Role |
|---|---|
| `nodes_` | `std::map<int, NavNode>` — **id-keyed** (ordered ⇒ deterministic output, and ids stay valid through deletion). |
| `edges_` | `std::vector<NavEdge>` — **rebuilt from scratch every reconcile** (idempotent; no stale edges). |
| `next_id_` | Monotonic id counter; only ever increments. |
| `update_call_count_` | Throttle counter for `Update`. |
| `nodes_cloud_` + `kdtree_nodes_` | Spatial index over current node positions; the node `id` is smuggled through PCL's intensity channel (`navgraph.cpp:90`, mirrors the keypose graph). |
| `node_marker_pub_` / `edge_marker_pub_` / `label_marker_pub_` | RViz publishers the module owns. |

---

## The reconcile pass — `Update()` → `Reconcile()`

`Update` (`navgraph.cpp:50`) is the **single entry point** the planner calls each
cycle. It **self-throttles**: the full reconcile runs on every
`kNavGraphUpdateInterval`-th call (the first call runs immediately), so the planner
side is one line. A run does: `Reconcile` → `TagRooms` → `AssignNames` →
`PublishVisualization` (+ a temporary count log).

`Reconcile` (`navgraph.cpp:121`) operates **only on the keypose graph's connected
component** (`GetConnectedGraphNodeIndices`), so disconnected/edgeless keypose junk
never enters the NavGraph. Four phases:

1. **Seed** (`:132`) — greedy distance-gated coverage. For each connected keypose
   position, if it is farther than `kNavNodeMinDist` from every existing node (via
   the kdtree) **and** every node seeded earlier this pass (checked linearly — the
   set is small), mint a new `NavNode` there with a fresh id. **Seeding is purely
   additive**: existing nodes are never moved or touched. Seeds come from **both**
   keypose (trajectory) and connector nodes, since the bulk read makes no
   distinction. Invariant afterwards: every connected keypose node is within
   `kNavNodeMinDist` of some NavGraph node.
2. **Label** (`:169`) — nearest-node Voronoi. Each connected keypose node is
   assigned to its nearest NavGraph node (its *region*); a per-node member count is
   tallied. No orphan/distance-cap case is needed thanks to the Phase-1 invariant.
3. **Hard-delete** (`:188`) — any NavGraph node whose region got **no** connected
   members this pass is erased. (Newly seeded nodes always keep ≥ 1 member — their
   own seed — so they survive.) Ids are retired, never reused.
4. **Edges** (`:203`) — region adjacency. For each connected keypose edge `(a, b)`,
   if `region(a) ≠ region(b)` those two NavGraph nodes are adjacent. Each unique
   pair gets one edge, weighted by `keypose_graph->GetShortestPath(pos_u, pos_v,
   …, use_connected_nodes=true)`. Edges that return no path (≥ `INF`) are skipped.

### `TagRooms()` (`navgraph.cpp:253`)

Voxelizes each node's position into the planner's `room_mask_` and reads the room
id — mirroring `Representation::UpdateViewpointRoomIdsFromMask` (same
`misc_utils_ns::point_to_voxel`). Out-of-bounds or no-mask ⇒ `room_id = -1`.

### `AssignNames()` (`navgraph.cpp:281`)

Assigns each node its scene-graph waypoint id `"<room_key>-wp_<n>"`. Nodes are
grouped by room and numbered **`wp_1..N` in ascending node-id order** — exactly the
order the exporter emits, so the names line up. **`wp_0` is reserved for the room
centroid** (emitted by the exporter, not a NavGraph node). The `room_key` strings
(e.g. `"kitchen-room_1"`) are passed in from the planner, built with the exporter's
own `SceneGraphExporter::RoomKey`, so RViz labels and JSON ids cannot drift. A node
with no alive room gets an **empty name** and is not exported.

---

## How it reaches the JSON

The [exporter](../scene_graph_exporter/README.md) takes `GetNodes()` /
`GetEdges()` and, per room, emits:

- `waypoints`: `wp_0` = room **centroid**, then `wp_1..N` = that room's NavGraph
  nodes (using `node.name` verbatim as the id).
- `edges`: the **NavGraph edges only**, referencing node waypoint ids with their
  real `meters`. (The centroid `wp_0` is therefore an isolated waypoint that no
  edge touches.)

Nodes with `room_id < 0` (and edges touching them) are dropped — the schema is
strictly room-organized, with no "no-room" bucket.

---

## Visualization

`PublishVisualization` (`navgraph.cpp:302`) owns all three RViz outputs (the
planner is not involved):

| Topic | Type | Style |
|---|---|---|
| `navgraph/node_marker` | `Marker` `POINTS` | **orange** (1.0/0.5/0.0), 0.35. A marker (not a `PointCloud2` intensity field) so the color is fixed regardless of RViz's color transformer. |
| `navgraph/edge_marker` | `Marker` `LINE_LIST` | **green**, 0.1 — distinct from the keypose graph's yellow edges. |
| `navgraph/label_marker` | `MarkerArray` of `TEXT_VIEW_FACING` | white node-name labels floating 0.3 m above each node. A `DELETEALL` is sent first each cycle so deleted nodes' labels don't linger. A node with no room shows `"(no room) #<id>"`. |

All published in `world_frame_id_` (default `"map"`, = the keypose graph's frame).

---

## Parameters (`navigation_graph/...`)

Declared by the planner (`sensor_coverage_planner_ground.cpp:120`), read by
`NavGraph::ReadParameters` (`navgraph.cpp:39`). Override per scenario in the
scenario yaml next to `keypose_graph/*` (set in `config/go2w_bag_direct.yaml`).

| Param | Role | Default |
|---|---|---|
| `kNavNodeMinDist` | Node spacing — the in-room-waypoint granularity knob. | 1.25 |
| `kNavGraphUpdateInterval` | Run the full reconcile every Nth `Update()` call (clamped ≥ 1). | 2 |
| `world_frame_id` | Frame for the RViz markers. | `map` |

---

## Gotchas

- **Edges are reachability, not geometry.** A NavGraph edge says "you can walk
  between these regions" with the true keypose walking distance — it is *not* a
  collision-checked straight line. Two nodes a short Euclidean distance apart but
  separated by a wall are correctly *not* edged; two far nodes down a corridor are.
- **Id churn at a re-created location is accepted.** Nodes are hard-deleted when
  their keypose support disconnects; if the spot is revisited later it gets a
  **new** id (ids are never reused). Within a node's lifetime its id is stable.
- **Names can lag a snapshot by a label change.** `name` is computed at reconcile
  time from the current room label; if the VLM relabels a room between a reconcile
  and a snapshot, the prefix may briefly differ from the room's dict key. Resolves
  on the next reconcile.
- **The centroid (`wp_0`) is JSON-only.** It is not a NavGraph node, so it never
  appears in the RViz labels and no NavGraph edge touches it. Expect RViz to show
  *N* labels per room and the JSON to have *N+1* waypoints.
- **Frame.** Node positions are copied verbatim from keypose nodes, so the NavGraph
  inherits the keypose graph's frame (the bag's `odom`/`world`, still labelled
  `kWorldFrameID = "map"`). The NavGraph applies no transforms of its own.

---

## Quick API reference

```cpp
// --- the only entry point the planner calls (once per cycle, self-throttled) ---
void Update(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
            const cv::Mat& room_mask, const Eigen::Vector3f& shift,
            float room_resolution,
            const std::map<int, std::string>& room_keys);

// --- read API for the exporter / other consumers ---
const std::map<int, NavNode>& GetNodes() const;   // id-keyed
const std::vector<NavEdge>&   GetEdges() const;
int GetNodeNum() const;
int GetEdgeNum() const;
```
