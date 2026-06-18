# Keypose Graph

The keypose graph is the planner's **global topological roadmap**: an undirected
graph of waypoints laid over the explored free space, where an edge means *"the
robot can travel between these two points in a straight line without hitting a
known obstacle."* It is the structure the planner uses to answer, efficiently and
repeatedly:

- **"How far apart are A and B along an actually-traversable route?"** (graph
  distance, not the lying Euclidean distance that cuts through walls)
- **"Give me a concrete waypoint path from A to B."**

Mental model: it is the *skeleton of the walkable world*. Nodes ≈ breadcrumbs the
robot drops along its trajectory, plus extra connector waypoints; edges ≈
collision-free straight segments between nearby, mutually-visible breadcrumbs.

> **Not part of the scene graph — but the NavGraph contracts it.** Semantic
> mapping and room segmentation never touch this structure, and its own
> nodes/edges never enter the `Representation`. Couplings to the scene-graph side:
> the planner calls `GetShortestPath(robot, object)` for a *walking* distance to a
> found target/anchor object; the per-keypose accumulated scan (`keypose_cloud_`)
> is reused when sampling viewpoint reps; and the **[NavGraph](../navgraph/README.md)**
> reads this graph each cycle (connected component + `GetNodeNeighbors` adjacency +
> `GetShortestPath`) to build the sparse waypoint graph that *is* exported. The
> NavGraph is the only consumer that turns this roadmap into scene-graph output.

---

## Files

| File | Contents |
|---|---|
| `include/keypose_graph/keypose_graph.h` | `KeyposeNode` struct, `KeyposeGraph` class declaration |
| `src/keypose_graph/keypose_graph.cpp` | All construction / maintenance / query logic |

Driven from the main planner
(`src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp`) and consumed
heavily by the grid world (`src/grid_world/grid_world.cpp`). Path search uses the
generic A\* in `src/utils/misc_utils.cpp` (`AStarSearch`,
`AStarSearchWithMaxPathLength`).

---

## Where it sits in the pipeline

```
 RegisteredScanCallback (every 5th scan)            GlobalPlanning()
        robot_position_  ─┐                      grid_world_->AddPathsInBetweenCells
                          │                                  │  (nav_msgs::Path)
                          ▼                                  ▼
                 AddKeyposeNode()                       AddPath()
              (keypose nodes, is_keypose=true)   (connector nodes, is_keypose=false)
                          │                                  │
                          └──────────────┬───────────────────┘
                                         ▼
                          ┌─────────────────────────────┐
                          │        KeyposeGraph          │
                          │  nodes_ / graph_ / dist_     │
                          │  + kdtrees (all / connected) │
                          └─────────────────────────────┘
                                         │
            UpdateKeyposeGraph() once per planning cycle:
              CheckLocalCollision()  → prune nodes/edges now in collision
              CheckConnectivity()    → flood-fill reachable component
                                         │
                  ┌──────────────────────┼───────────────────────────┐
                  ▼                      ▼                            ▼
         GetShortestPath()      IsPositionReachable()      GetClosest*Node*()
        (grid-world TSP,        (reject unreachable        (bind robot / objects
         return-home, object     goals)                     to graph nodes)
         distance)
```

---

## Core data structures

### `KeyposeNode` (keypose_graph.h:40)

A single waypoint plus bookkeeping.

| Field | Meaning |
|---|---|
| `position_` | xyz in the `map` frame |
| `keypose_id_` | which keypose this node belongs to (a monotonic counter, **not** a SLAM id) |
| `node_ind_` | this node's index into `nodes_` |
| `cell_ind_` | grid-world cell index (set by callers; informational) |
| `is_keypose_` | `true` = dropped from the robot trajectory; `false` = a connector node from a roadmap path |
| `is_connected_` | recomputed every cycle by `CheckConnectivity` — reachable from the anchor? |
| `offset_to_keypose_` | offset to the parent keypose at creation time — **vestigial** (see Gotchas) |

### `KeyposeGraph` (keypose_graph.h:75)

Plain index-parallel adjacency lists plus spatial indices:

| Member | Role |
|---|---|
| `nodes_` | `vector<KeyposeNode>` — the nodes; index == `node_ind_` |
| `graph_[i]` | neighbor indices of node `i` (adjacency list) |
| `dist_[i]` | edge weights parallel to `graph_[i]` (Euclidean edge length) |
| `nodes_cloud_` + `kdtree_nodes_` | **all** nodes, for nearest-node queries |
| `connected_nodes_cloud_` + `kdtree_connected_nodes_` | only the connected component, for reachability queries |
| `connected_node_indices_` | cached list of node indices in the connected component |
| `current_keypose_id_`, `current_keypose_position_` | the most recently added keypose (used to stamp connector nodes) |
| `allow_vertical_edge_` | if false, edges spanning a large z-gap are forbidden (keeps floors separate); set false by the planner |

Key implementation details:

- **Edges are stored symmetrically.** `AddEdge` (keypose_graph.cpp:74) pushes into
  both `graph_[from]` and `graph_[to]`, and the same weight into both `dist_`
  lists.
- **The kdtrees smuggle the node index through PCL's intensity channel**
  (`point.intensity = i`, line 435). Almost every query is *"find the node nearest
  to this arbitrary point,"* so nearest-neighbour search returns a cloud index,
  and the intensity recovers the real `node_ind_`.
- `UpdateNodes()` (line 426) rebuilds `nodes_cloud_` + `kdtree_nodes_` from
  scratch. It is called after any batch that adds nodes.

---

## Two kinds of nodes

The single `nodes_` vector holds two populations that are created by different
mechanisms and deduplicated against different radii:

| | **Keypose nodes** | **Connector (non-keypose) nodes** |
|---|---|---|
| `is_keypose_` | `true` | `false` |
| Source | the robot's own trajectory | grid-world roadmap paths between cells |
| Added by | `AddKeyposeNode` | `AddPath` → `AddNonKeyposeNode` |
| Dedup radius | `kAddNodeMinDist` | `kAddNonKeyposeNodeMinDist` |
| Pruned by collision? | **No** — trajectory is trusted | **Yes** — `CheckLocalCollision` may delete them |
| Role | continuous backbone tracing where the robot went | stitch separate regions/cells together so the global router can cross between them |

---

## Construction

### 1. Keypose nodes — from the robot trajectory

There is **no external SLAM "keypose" topic** in this stack. The planner mints
keyposes itself inside `RegisteredScanCallback`
(sensor_coverage_planner_ground.cpp:918):

- Registered scans are accumulated; on **every 5th scan**
  (`registered_cloud_count_ == 0`, line 943) the planner:
  1. takes the robot's current position as a candidate keypose
     (`keypose_.pose.pose.position = robot_position_`, line 945),
  2. stamps it with a monotonic id —
     `keypose_.pose.covariance[0] = keypose_count_++` (line 946). The id is hacked
     through the unused covariance field of the `Odometry` message.
  3. calls `keypose_graph_->AddKeyposeNode(keypose_, *planning_env_)` (line 948),
  4. snapshots the accumulated downsampled scan into `keypose_cloud_` and raises
     `keypose_cloud_update_`.

**`AddKeyposeNode` (keypose_graph.cpp:514)** is the heart of construction:

1. Record `current_keypose_position_` / `current_keypose_id_` from the message.
2. **First keypose ever** (`nodes_` empty or no keypose nodes yet, line 528):
   just add it and return. This node becomes the permanent **connectivity
   anchor** and also serves as *home*.
3. Otherwise scan all existing nodes, honouring the vertical constraint
   (`kAddEdgeVerticalThreshold`, line 546 — skipped only if
   `allow_vertical_edge_`), and find:
   - `min_dist_ind` — nearest *keypose* node,
   - `last_keypose_ind` — most recent keypose node (highest `keypose_id_`),
   - every node within `kAddEdgeConnectDistThr` → `in_range_node_indices`.
4. **Dedup:** if the nearest keypose is closer than `kAddNodeMinDist` (line 574),
   do **not** add a node — return that existing node's index (line 634). This is
   what keeps keyposes spaced ≈ `kAddNodeMinDist` apart so a stationary robot
   doesn't spam nodes.
5. Otherwise add the node and wire its first edge:
   - to the **last keypose** if it is within `kAddEdgeToLastKeyposeDistThr`
     (preserves trajectory continuity, line 577), **else**
   - to the **nearest** node (line 587).
6. **Cross-linking with collision checks (lines 590–628):** for every other
   in-range node, ray-march a straight line from the new node to it at
   `kAddEdgeCollisionCheckResolution` steps and add an edge **only if
   `planning_env_->InCollision(...)` is false along the whole segment**. This is
   what turns a 1-D trajectory thread into a 2-D mesh: it connects nearby branches
   of the path that are mutually visible, giving A\* real shortcuts.

> `allow_vertical_edge_` is set `false` by the planner
> (sensor_coverage_planner_ground.cpp:602), so nodes more than
> `kAddEdgeVerticalThreshold` apart in z are never linked — different floors stay
> topologically separate.

### 2. Connector nodes — from grid-world roadmap paths

During `GlobalPlanning`, `grid_world_->AddPathsInBetweenCells(...)`
(sensor_coverage_planner_ground.cpp:2934) computes candidate paths between
explored subspace cells and injects each into the graph via
`keypose_graph_->AddPath(path)` (grid_world.cpp:1404).

**`AddPath` (keypose_graph.cpp:188)** walks the polyline:

- For each vertex it calls **`AddNonKeyposeNode`** (line 160), which dedups
  against `kAddNonKeyposeNodeMinDist` (returns the existing node if close enough,
  line 171) and otherwise appends a new `is_keypose_ = false` node stamped with
  the current keypose id.
- It chains edges between consecutive path vertices (skipping duplicates via
  `HasEdgeBetween`).
- Calls `UpdateNodes()` at the end to refresh the kdtree.

These connector nodes stitch together regions the raw trajectory never directly
linked, so the global router can plan a route that crosses from one cell/room to
another.

---

## Maintenance — once per planning cycle

`UpdateKeyposeGraph()` (sensor_coverage_planner_ground.cpp:2229, called at
line 3761 inside `execute()`) refreshes markers and then **prunes and re-labels**
the graph:

### `CheckLocalCollision(robot_position, viewpoint_manager)` (keypose_graph.cpp:333)

The healing step. For each **non-keypose** node inside the local planning horizon
(keypose nodes are skipped — the trajectory is trusted, line 345):

- If the node now sits on a viewpoint that is `InCollision`, **cut the node out**
  entirely — erase it from every neighbour's adjacency list and clear its own
  (lines 364–378).
- Otherwise re-walk each of its edges: interpolate points along the segment
  (`LinInterpPoints`) and **delete the edge if any sample lands in a colliding
  viewpoint** (lines 385–420).

This is how the roadmap stays honest as exploration reveals obstacles: edges that
used to pass through now-known walls get removed.

### `CheckConnectivity(robot_position)` (keypose_graph.cpp:444)

- Rebuilds the node kdtree (`UpdateNodes`).
- Sets every node `is_connected_ = false`, then **DFS flood-fills from the first
  keypose node** (the anchor, always connected, lines 456–475) via
  `GetConnectedNodeIndices` (an explicit-stack DFS, line 292). If for some reason
  there is no keypose node, it falls back to flooding from the node nearest the
  robot (lines 477–493).
- Rebuilds `connected_nodes_cloud_` + `kdtree_connected_nodes_` from the connected
  set.

Everything that needs *reachable* answers
(`IsPositionReachable`, `GetClosestConnectedNodeIndAndDistance`,
`GetShortestPath(..., use_connected_nodes=true)`) consults the connected kdtree,
so the planner never routes to an island it cannot actually reach.

---

## Querying / consumption

The graph exposes two families of queries.

**Spatial binding (snap a free point to the graph):**

| Method | Returns |
|---|---|
| `GetClosestNodeInd` / `GetClosestNodeIndAndDistance` | nearest node over **all** nodes |
| `GetClosestConnectedNodeIndAndDistance` | nearest node in the **connected** component |
| `GetClosestKeyposeID` | keypose id of the nearest node |
| `IsPositionReachable(point[, thr])` | is `point` within `thr` (default `kAddNonKeyposeNodeMinDist`) of a connected node? |
| `GetFirstKeyposePosition` | the anchor keypose ≈ *home* / start pose |

**Routing (A\* over `graph_`/`dist_`):**

| Method | Behaviour |
|---|---|
| `GetShortestPath(start, target, get_path, path, use_connected_nodes)` | snaps `start`/`target` to nearest nodes (within a 1.5 m z-window when vertical edges are disallowed), runs `AStarSearch`, returns the distance; fills `path` if `get_path`. Degenerate `<2` nodes → straight line. |
| `GetShortestPathWithMaxLength(...)` | same, but aborts paths exceeding `max_path_length` (`AStarSearchWithMaxPathLength`) |

Returned path poses carry extra metadata: `orientation.w = keypose_id`,
`orientation.x = node_ind` (lines 974–976), so callers can recover which
keypose/graph node each waypoint maps to.

**Who consumes it (grid_world.cpp & planner):**

- `UpdateCellKeyposeGraphNodes` (grid_world.cpp:267) bins connected graph nodes
  into EXPLORING cells, giving each cell its graph "ports."
- `AddPathsInBetweenCells` / `SolveGlobalTSP` call `GetShortestPath` /
  `GetShortestPathWithMaxLength` repeatedly to get **traversable** inter-cell
  distances and to materialize the global path the robot follows.
- Reachability gating, return-home (`GetFirstKeyposePosition` + A\*), and
  **walking distance to a found object/anchor**
  (sensor_coverage_planner_ground.cpp:1550, 1603, 4776, 5039) all go through the
  graph — the lone touchpoint between this roadmap and the scene-graph search.

---

## Visualization

Built by two helpers and published from `UpdateKeyposeGraph`:

| Helper | Produces | Topic | Style |
|---|---|---|---|
| `GetMarker(node_marker, edge_marker)` (keypose_graph.cpp:229) | a `POINTS` marker of all node positions and a `LINE_LIST` marker of all unique edges | node → `keypose_graph_node_marker`, edge → `keypose_graph_edge_marker` | nodes red `POINTS` 0.4; edges yellow `LINE_LIST` 0.05 (set at sensor_coverage_planner_ground.cpp:406–416) |
| `GetVisualizationCloud(cloud)` (keypose_graph.cpp:271) | a `PointXYZI` cloud of all nodes, intensity `10` if connected, `-1` if not | `keypose_graph_cloud` | colour-codes connectivity |

Wiring in `UpdateKeyposeGraph` (sensor_coverage_planner_ground.cpp:2233–2241):

- The **edge marker is published** (line 2236) → the yellow edge mesh you see in
  RViz.
- The **node marker is built but *not* published** (line 2235 is commented out).
  Nodes are instead visualized through the `keypose_graph_cloud` point cloud, so
  you can tell connected (bright) from disconnected (dim) nodes.
- `GetMarker` is called *before* the collision/connectivity prune, while the vis
  cloud is generated *after* — so on any given frame the edge marker reflects the
  pre-prune graph and the cloud reflects the freshly-pruned connectivity. Usually
  invisible, but worth knowing when debugging.

---

## Parameters (`keypose_graph/...`)

Declared in the planner (sensor_coverage_planner_ground.cpp:101–114), read by
`KeyposeGraph::ReadParameters` (keypose_graph.cpp:46). The hardcoded constructor
defaults (lines 31–37) are immediately overwritten by `ReadParameters`, so the
declared/yaml values are what actually take effect.

| Param | Role | Declared default |
|---|---|---|
| `kAddNodeMinDist` | min spacing between keypose nodes (dedup) | 0.5 |
| `kAddNonKeyposeNodeMinDist` | min spacing for connector nodes; also the reachability radius | (yaml) |
| `kAddEdgeConnectDistThr` | max distance to *attempt* a cross-link edge | 0.5 |
| `kAddEdgeToLastKeyposeDistThr` | prefer first edge to last keypose if within this | (yaml) |
| `kAddEdgeVerticalThreshold` | max z-gap for any edge (floor separation) | 1.0 |
| `kAddEdgeCollisionCheckResolution` | ray-march step for edge collision tests | 0.5 |
| `kAddEdgeCollisionCheckRadius` | collision-check radius | (yaml) |
| `kAddEdgeCollisionCheckPointNumThr` | point-count threshold for a segment to count as in-collision | 1 |

---

## Gotchas

- **`offset_to_keypose_` is vestigial in this fork.** In upstream TARE, connector
  nodes store an offset to their parent keypose so that when SLAM loop closure
  corrects keypose positions, the attached roadmap nodes rigidly follow.
  `AddNonKeyposeNode` still computes the offset
  (`SetCurrentKeyposePosition`, line 178) but **nothing ever re-applies it** —
  `UpdateNodes` only rebuilds the kdtree. Since keyposes here are raw robot
  positions in a single drift-prone frame with no loop-closure feedback into the
  graph, the re-anchoring machinery is dead code. Fine as long as odometry
  doesn't jump.
- **The anchor is the first keypose, not the robot.** `CheckConnectivity` always
  floods from the first-ever keypose (≈ origin). Keypose nodes are never pruned,
  so that anchor is effectively permanent; the robot-nearest fallback
  (lines 477–493) rarely fires.
- **`keypose_id` rides in `covariance[0]`** — a monotonic counter incremented
  every 5th registered scan, not a SLAM keyframe id.
- **Two node classes, two dedup radii, one id space.** Keypose nodes dedup on
  `kAddNodeMinDist`; connector nodes on `kAddNonKeyposeNodeMinDist`. They share
  the same `nodes_` vector and `keypose_id_` numbering.
- **Distances are XYZ for edges but queries use an XY + z-window snap.** When
  vertical edges are disallowed, `GetShortestPath` only snaps to nodes within a
  hardcoded 1.5 m z-band (keypose_graph.cpp:839, 936 — flagged `TODO:
  parameterize`).
- **Coordinates are in the single drift-prone `map`/odom frame** anchored at the
  robot's start pose (the first keypose is ≈ `(0,0,0)`), same frame as everything
  else in the stack.

---

## Quick API reference

```cpp
// --- construction ---
int  AddKeyposeNode(const Odometry& keypose, const PlanningEnv& env);  // trajectory node
int  AddNonKeyposeNode(const Point& pos);                             // connector node
void AddPath(const nav_msgs::msg::Path& path);                        // batch of connector nodes+edges
void AddEdge(int from, int to, double dist);

// --- maintenance (call once per cycle) ---
void CheckLocalCollision(const Point& robot, const ViewPointManagerPtr& vpm);  // prune
void CheckConnectivity(const Point& robot);                                    // flood-fill is_connected_
void UpdateNodes();                                                            // rebuild node kdtree

// --- spatial binding ---
int    GetClosestNodeInd(const Point& p);
void   GetClosestConnectedNodeIndAndDistance(const Point& p, int& ind, double& d);
bool   IsPositionReachable(const Point& p[, double thr]);
Point  GetFirstKeyposePosition();   // home / anchor

// --- routing ---
double GetShortestPath(const Point& s, const Point& t, bool get_path,
                       nav_msgs::msg::Path& path, bool use_connected_nodes = false);
bool   GetShortestPathWithMaxLength(const Point& s, const Point& t, double max_len,
                                    bool get_path, nav_msgs::msg::Path& path);

// --- visualization ---
void GetMarker(Marker& node_marker, Marker& edge_marker);
void GetVisualizationCloud(PointCloud<PointXYZI>::Ptr cloud);  // intensity 10=connected, -1=not
```
