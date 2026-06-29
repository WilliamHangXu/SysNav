# NavGraph

The NavGraph is a **persistent, lightweight topological graph** maintained live
during a run as a **Voronoi contraction of the [keypose graph](../keypose_graph/README.md)**.
Where the keypose graph is a dense (~0.3–0.5 m spacing) traversability roadmap
the *planner* uses, the NavGraph is a sparse (~1.25 m spacing) version meant to be
**exported into the scene-graph JSON and handed to an LLM for path planning**:

- **Nodes** are a distance-spread subset of keypose-node positions — "in-room
  waypoints" — each tagged with a room and a scene-graph waypoint id.
- **Edges** mean *reachability* (you can walk from one node to the other), with
  weight = the keypose-graph traversable distance across the corridor that joins
  the two regions. They are **not** straight-line segments — there is zero
  geometry/collision checking in the NavGraph itself.

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
`navgraph_` (`.h:205`), constructed in `InitializeData` (`.cpp:519`), updated once
per planning cycle (`.cpp:4081`). Consumed by the
[scene graph exporter](../scene_graph_exporter/README.md) (`.cpp:5546`).

---

## Where it sits in the pipeline

```
                 KeyposeGraph (dense, planner's roadmap)
                          │   read-only each cycle
   GetConnectedGraphNodeIndices / GetNodePosition /
   GetNodeNeighbors / GetNeighborDistances
                          ▼
   execute(): after UpdateKeyposeGraph()  ──►  navgraph_->Update(...)
                          │
            ┌─────────────┴───────────────────────────────────┐
            │  Reconcile(): seed→reanchor→label→delete→edges  │  (throttled
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
| `seed_keypose_ind` | Keypose-node index this waypoint was seeded from, **frozen at birth**. It is the BFS source for the geodesic region labeling (Phase 2) and the anchor the re-anchor salvage (Phase 1.5) rebinds when that keypose node drops out of the connected component. `-1` before assignment. |
| `room_id` | Room affiliation (`-1` = unknown), re-tagged each pass from the room mask. |
| `name` | Scene-graph waypoint id, e.g. `"kitchen-room_1-wp_3"`. Exactly the id the exporter emits, so RViz labels and the JSON stay identical. Empty if the node has no room. |

### `NavEdge` (navgraph_types.h)

| Field | Meaning |
|---|---|
| `u`, `v` | The two node ids (canonical `u < v`). |
| `meters` | Traversable distance between the two nodes: the shortest crossing distance over the keypose edges joining their regions (summed keypose edge lengths, not straight-line). |

### `NavGraph` (navgraph.h)

| Member | Role |
|---|---|
| `nodes_` | `std::map<int, NavNode>` — **id-keyed** (ordered ⇒ deterministic output, and ids stay valid through deletion). |
| `edges_` | `std::vector<NavEdge>` — **rebuilt from scratch every reconcile** (idempotent; no stale edges). |
| `next_id_` | Monotonic id counter; only ever increments. |
| `update_call_count_` | Throttle counter for `Update`. |
| `nodes_cloud_` + `kdtree_nodes_` | Spatial index over current node positions; the node `id` is smuggled through PCL's intensity channel (`navgraph.cpp:99`, mirrors the keypose graph). |
| `node_marker_pub_` / `edge_marker_pub_` / `label_marker_pub_` | RViz publishers the module owns. |

---

## The reconcile pass — `Update()` → `Reconcile()`

`Update` (`navgraph.cpp:52`) is the **single entry point** the planner calls each
cycle. It **self-throttles**: the full reconcile runs on every
`kNavGraphUpdateInterval`-th call (the first call runs immediately), so the planner
side is one line. A run does: `Reconcile` → `TagRooms` → `AssignNames` →
`PublishVisualization` (+ a temporary per-phase timing log).

`Reconcile` (`navgraph.cpp:130`) operates **only on the keypose graph's connected
component** (`GetConnectedGraphNodeIndices`), so disconnected/edgeless keypose junk
never enters the NavGraph. Five phases (numbered to match the code):

- **Phase 1 — Seed** (`:146`) — greedy distance-gated coverage. For each connected
  keypose position, if it is farther than `kNavNodeMinDist` from every existing node
  (via the kdtree) **and** every node seeded earlier this pass (checked linearly —
  the set is small), mint a new `NavNode` there with a fresh id, stashing the keypose
  index it came from in `seed_keypose_ind` (the anchor Phases 1.5 and 2 use).
  **Seeding is purely additive**: existing nodes are never moved or touched. Seeds
  come from **both** keypose (trajectory) and connector nodes, since the bulk read
  makes no distinction. Invariant afterwards: every connected keypose node is within
  `kNavNodeMinDist` of some NavGraph node.
- **Phase 1.5 — Re-anchor** (`:192`) — orphan salvage to keep ids stable across
  connectivity blips. A node whose `seed_keypose_ind` has dropped out of the
  connected component would get zero members in Phase 2 and be hard-deleted, churning
  its id. Before that, for each such node, scan the **dead anchor's own keypose
  neighbors** for a still-connected, unclaimed one within `kNavNodeReanchorDist` of
  the node's (frozen) position; if one exists, rebind `seed_keypose_ind` to it so the
  node — and its id — survive. The position never moves; only the BFS source shifts
  (by ≤ `kNavNodeReanchorDist`, far below the node spacing, so the salvaged node
  rejoins through a real edge). No near connected neighbor ⇒ the node stays orphaned
  and is hard-deleted in Phase 3. Each keypose anchor is claimed by at most one node.
- **Phase 2 — Label** (`:250`) — **geodesic Voronoi via multi-source BFS** (this
  replaced an Euclidean nearest-node Voronoi). Each node's `seed_keypose_ind` is a
  BFS source; the search floods the connected component along **real keypose edges
  only**, and the first source to reach a keypose node (fewest hops) claims it as a
  member of that node's *region*; per-node member counts are tallied. Because keypose
  edges are collision-checked, a region label can **never leak across a wall** —
  which is exactly what produced false cross-wall NavGraph edges under the old
  Euclidean labeling (two same-room keypose nodes whose nearest representatives sat
  on opposite sides of a wall). It also costs less than the kdtree build + N
  nearest-queries it replaced.
- **Phase 3 — Hard-delete** (`:309`) — any NavGraph node whose region got **no**
  connected members this pass is erased. (Newly seeded nodes always keep ≥ 1 member —
  their own seed — so they survive.) Ids are retired, never reused.
- **Phase 4 — Edges** (`:325`) — region adjacency. For each connected keypose edge
  `(a, b)`, if `region(a) ≠ region(b)` those two NavGraph nodes are adjacent. The
  weight is the shortest **crossing distance**
  `‖navnode_u − a‖ + len(a,b) + ‖b − navnode_v‖` over all keypose edges that cross
  that region pair (`len(a,b)` read from `GetNeighborDistances`). Both endpoints sit
  within `kNavNodeMinDist` of their nav node (Phase-1 invariant), so for adjacent
  regions this is a tight estimate of the traversable distance — computed with **no
  per-edge A\* search**. (This used to call `GetShortestPath` once per nav edge,
  which is `O(nav_edges × keypose_nodes)` per reconcile and stalled the planning loop
  as the map grew; see the gotchas.)

### `TagRooms()` (`navgraph.cpp:417`)

Voxelizes each node's position into the planner's `room_mask_` and reads the room
id — mirroring `Representation::UpdateViewpointRoomIdsFromMask` (same
`misc_utils_ns::point_to_voxel`). Out-of-bounds or no-mask ⇒ `room_id = -1`.

### `AssignNames()` (`navgraph.cpp:445`)

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

`PublishVisualization` (`navgraph.cpp:466`) owns all three RViz outputs (the
planner is not involved):

| Topic | Type | Style |
|---|---|---|
| `navgraph/node_marker` | `Marker` `POINTS` | **orange** (1.0/0.5/0.0), 0.35. A marker (not a `PointCloud2` intensity field) so the color is fixed regardless of RViz's color transformer. |
| `navgraph/edge_marker` | `Marker` `LINE_LIST` | **green**, 0.1 — distinct from the keypose graph's yellow edges. |
| `navgraph/label_marker` | `MarkerArray` of `TEXT_VIEW_FACING` | white node-name labels floating 0.3 m above each node. A `DELETEALL` is sent first each cycle so deleted nodes' labels don't linger. A node with no room shows `"(no room) #<id>"`. |

All published in `world_frame_id_` (default `"map"`, = the keypose graph's frame).

---

## Parameters (`navigation_graph/...`)

Declared by the planner (`sensor_coverage_planner_ground.cpp:162`), read by
`NavGraph::ReadParameters` (`navgraph.cpp:40`). Override per scenario in the
scenario yaml next to `keypose_graph/*` (set in `config/go2w_bag_direct.yaml`).

| Param | Role | Default |
|---|---|---|
| `kNavNodeMinDist` | Node spacing — the in-room-waypoint granularity knob. | 1.25 |
| `kNavNodeReanchorDist` | Max distance to salvage an orphaned node by re-anchoring it to a still-connected keypose neighbor (Phase 1.5), instead of deleting it. Kept well below `kNavNodeMinDist`. | 0.2 |
| `kNavGraphUpdateInterval` | Run the full reconcile every Nth `Update()` call (clamped ≥ 1). | 2 |
| `world_frame_id` | Frame for the RViz markers. | `map` |

---

## Gotchas

- **Edges are reachability, not geometry.** A NavGraph edge says "you can walk
  between these regions" with the true keypose walking distance — it is *not* a
  collision-checked straight line. Two nodes a short Euclidean distance apart but
  separated by a wall are correctly *not* edged; two far nodes down a corridor are.
- **Id churn at a re-created location is accepted (but transient blips are
  salvaged).** A brief anchor disconnect is repaired by the re-anchor pass (Phase
  1.5) when a still-connected keypose node sits within `kNavNodeReanchorDist` of the
  frozen node position, so the id survives. Only a *sustained* disconnect (no
  still-connected keypose node that close) hard-deletes the node; if the spot is
  revisited later it gets a **new** id (ids are never reused). Within a node's
  lifetime its id is stable.
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
- **Edge weighting must stay A\*-free.** `Update()` runs synchronously inside the
  planner's `execute()` loop, so anything it does delays planning *and* every viz
  marker the loop publishes (keypose + navgraph markers, room labels) — room masks
  keep flowing only because segmentation is a separate node. The original Phase 4
  called `GetShortestPath` once per nav edge — `O(nav_edges × keypose_nodes)` per
  reconcile (each call does an `O(N)` endpoint scan + a full position-vector copy +
  A\*) — which grew with the map and froze the loop for seconds (long stalls, then a
  burst). Edge weights are now accumulated from crossing keypose-edge lengths in one
  `O(total keypose edges)` pass. Keep it that way; don't reintroduce a per-edge
  graph search.

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
