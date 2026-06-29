# Quadrant / Room-Area pipeline

Buckets every scene-graph **waypoint** into one of four room **quadrants** —
`northeast` / `northwest` / `southeast` / `southwest` — so the exported JSON (and
its LLM / frontend consumers) can reason about "the NE corner of the kitchen". The
quadrants are defined by a **single global building orientation** fit automatically
from the room geometry (`cv::minAreaRect` over all room polygons) and **frozen**
once stable, with each room's axes centered at its **centroid**.

The feature spans three places but is small and additive:

1. **`QuadrantManager`** (this module) fits + freezes the global axes and draws the
   per-room cross-lines in RViz.
2. The **[NavGraph](../navgraph/README.md)** tags each node's `area` live (for node
   coloring), reading the frozen axes.
3. The **[scene-graph exporter](../scene_graph_exporter/README.md)** writes
   `"area"` onto every waypoint and a `compass` into `layout.metadata`.

All three read the **same** frozen `BuildingAxes` and the **same** room centroids
(handed in by the planner each cycle), so the live RViz color and the JSON `area`
are literally the same field — they can never disagree.

---

## Files

| File | Contents |
|---|---|
| `include/navgraph/building_axes.h` | The shared primitives: `enum class Area`, `AreaName()`, `struct BuildingAxes`, and the pure `AssignArea()`. **Eigen-only, ROS-free, OpenCV-free** — the lowest layer, so the ROS-free `NavNode` and the rclcpp/OpenCV-free exporter can both use it. Lives in `navgraph_ns`. |
| `include/quadrant_manager/quadrant_manager.h` | `QuadrantManager` class declaration (`quadrant_ns`). |
| `src/quadrant_manager/quadrant_manager.cpp` | The fit (`minAreaRect`), canonicalization, freeze state machine, and cross-line visualization. The **only** translation unit that pulls in `<opencv2/imgproc.hpp>`. |

Touched elsewhere (see those modules' code):

| File | Change |
|---|---|
| `navgraph_types.h` | `NavNode` gains `Area area`. |
| `navgraph.{h,cpp}` | `Update()` takes `room_centroids` + `axes`; new `TagAreas()`; node marker recolored per quadrant. |
| `scene_graph_exporter.{h,cpp}` | `Build()`/`BuildRoomJson()` take `axes`; `"area"` per waypoint; `compass` in metadata; `compass_radius_m` config; `ComputeAabbCenterSource()`. |
| `sensor_coverage_planner_ground.{cpp,h}` | Owns `quadrant_mgr_`; declares `quadrant/*` params; builds `room_centroids`; orders `quadrant_mgr_->Update()` → `navgraph_->Update()`; passes axes into `Build()`. |

---

## Where it sits in the pipeline

```
   Representation rooms (polygons + centroids)
            │  once per planning cycle (in execute())
            ▼
   QuadrantManager::Update(rooms)
      ├─ FitAxes(): minAreaRect over ALL room polygons → canonicalize
      ├─ freeze on warmup stability (angle stable for K cycles, hard cap)
      └─ PublishCrossLines(): per-room axis cross  ──►  RViz /quadrant/cross_marker
            │
            │  GetAxes()  (frozen BuildingAxes; invalid until freeze)
            ├───────────────────────────────┐
            ▼                                ▼
   NavGraph::Update(..., room_centroids, axes)     SceneGraphExporter::Build(..., axes)
      └─ TagAreas(): node.area =                      ├─ waypoint "area" (wp_0 computed,
         AssignArea(node.pos, centroid, axes)         │   wp_1..N read node.area)
      └─ PublishVisualization(): nodes colored        └─ layout.metadata.compass
         by area  ──► RViz /navgraph/node_marker          (center + N/S/E/W)
```

---

## The model in one paragraph

The robot-start `map` frame is at an arbitrary yaw to the building walls, so raw map
axes would cut rooms diagonally and "NE of the kitchen" would mean nothing physical.
Instead we fit a single **oriented rectangle** to the whole explored floor plan; its
orientation recovers the building's principal directions automatically (no reliance
on the hand-tuned per-bag `world←odom` yaw). That orientation is a *static* property,
so it is **frozen once** and reused for the rest of the run — assignment is then a
pure, stateless function of (point, room centroid, frozen axes), recomputed wherever
it is needed. The room centroid is the axis **origin**, so a point's quadrant is just
the sign of its offset projected onto the two axes.

---

## Core data structures (`building_axes.h`)

### `enum class Area`
`kUnknown(-1)`, `kNorthEast(0)`, `kNorthWest(1)`, `kSouthEast(2)`, `kSouthWest(3)`.
`AreaName(Area)` maps to the canonical strings emitted into the JSON and the RViz
labels (`"northeast"`…, `"unknown"`), so the two cannot drift.

### `struct BuildingAxes`
```cpp
Eigen::Vector2d east{1,0}, north{0,1};   // XY only (the stack is gravity-aligned)
bool valid = false;                       // false until the warmup freeze
```
`east` is canonicalized to within ±45° of map +X and `north = east` rotated +90° CCW
(so `north.y > 0`, "up"). A **default-constructed `BuildingAxes{}` is the "not ready"
sentinel** — `AssignArea` returns `kUnknown` and the exporter omits the compass.

### `Area AssignArea(p, origin, axes)` (pure)
`d = p − origin`; `east_pos = d·east ≥ 0`; `north_pos = d·north ≥ 0`; map
`(north_pos, east_pos)` → NE/NW/SE/SW. Tiebreak `≥ 0 → positive` (deterministic). 2D
only — pass the x,y of 3D positions.

### `NavNode::area` (`navgraph_types.h`)
The per-node quadrant, tagged each reconcile from the node's room centroid + the
frozen axes. Stored on the node so it is available **live** (RViz color) and read
verbatim by the exporter. `kUnknown` until the axes freeze / a node has no room.

---

## QuadrantManager — fit, freeze, draw

`Update(rooms, debug_log)` is called once per planning cycle by the planner (right
before `navgraph_->Update`). It self-throttles to every `kUpdateInterval`-th call.

### Fit — `FitAxes` (`cv::minAreaRect`)
Gathers every alive room's polygon vertices into one point set and fits a rotated
rectangle. The axes are taken from the rect **corners** (`boxPoints`), *not*
`RotatedRect::angle` (whose range flipped between OpenCV 4.4 and 4.5):

```
e = the rect side more aligned with map +X (larger |x|), flipped so e.x ≥ 0
north = (−e.y, e.x)                                   // +90° CCW
```

Choosing the larger-|x| side and flipping to `e.x ≥ 0` puts `east` deterministically
within ±45° of +X. This **absorbs the 90° rect ambiguity**: a 90° rotation swaps
which side is "more horizontal", and canonicalization re-selects the within-±45° one,
yielding an identical `east`. `FitAxes` returns false (no freeze) on too few /
collinear / degenerate vertices, so an early sparse map never latches a garbage axis.

### Freeze — warmup stability
The building orientation is static, so it is a **latch-once** quantity (mirrors the
intent of the planner's `TryFreezeWorldFromOdom`, but triggered on geometry stability
rather than a TF lookup). Each successful fit, the east-axis angle is compared to the
previous one (mod 90°); after `kFreezeStableCycles` consecutive fits within
`kFreezeAngleEpsDeg`, the axes **freeze** and are never refit. A hard cap
(`kMaxWarmupCycles`) guarantees a freeze even if the angle keeps wobbling. The
one-time freeze line is always logged:

```
[quadrant] axes FROZEN after 13 cycles (stable=5): east=(0.951,-0.309) north=(0.309,0.951) angle=-18.0 deg
```

Before freeze, `GetAxes().valid == false` → every node is `kUnknown` (grey), no
cross-lines, no compass. Freeze normally happens within the first ~tens of cycles, so
live quadrants appear early in the run; periodic snapshots (60 s apart) and the
end-of-bag snapshot therefore essentially always have valid axes.

---

## How nodes get tagged — `NavGraph::TagAreas`

Runs after `TagRooms` (needs `room_id`) and before `AssignNames`. For each node with
a known room and centroid:
`node.area = AssignArea({pos.x, pos.y}, {centroid.x, centroid.y}, axes)`. No room /
unknown centroid / unfrozen axes → `kUnknown`. The planner builds the
`room_id → centroid` map from `Representation::GetRoomNodesMap()` in the same loop
that builds the room keys, so it covers exactly the alive rooms the exporter emits.

---

## How it reaches the JSON — exporter

Each waypoint object gains an `"area"`:

```jsonc
{ "id": "office-room_22-wp_3", "x": .., "y": .., "z": .., "area": "northeast" }
```

- **`wp_1..N`** (NavGraph nodes) read `AreaName(node->area)` — the field tagged
  upstream. No recomputation.
- **`wp_0`** (the room interior point) is assigned the same way, from the room
  centroid: `AreaName(AssignArea(interior_point, centroid, axes))`. It is **not** a
  special "center" — the interior point lands wherever it lands in the quadrants.

A **compass** is added to `layout.metadata` so a frontend can draw an oriented
compass at the right place:

```jsonc
"compass": {
  "center": { "x", "y", "z" },        // environment-AABB center
  "north":  { … }, "south": { … },    // center ± R · north
  "east":   { … }, "west":  { … }     // center ± R · east
}
```

The 5 points are computed in the **source frame** (`ComputeAabbCenterSource`) then
each transformed by `world_from_source` (like every other coordinate), so the compass
aligns with the waypoints in whatever frame the snapshot is in. Radius
`R = compass_radius_m` if > 0, else **auto** = ½·max(width, height). The compass is
**omitted** when the axes are not yet frozen or there are no rooms (a frontend treats
absence as "not ready").

---

## Visualization

| Topic | Type | Style |
|---|---|---|
| `quadrant/cross_marker` | `Marker` `LINE_LIST` | Per alive room, two lines through the centroid spanning the room (extents = polygon vertices projected onto the axes). **East line red, north line blue.** One marker, rewritten each publish so dead rooms vanish. Width `kCrossLineWidth`. Only published once the axes are valid. |
| `navgraph/node_marker` | `Marker` `POINTS` | NavGraph nodes, **per-point colored by quadrant**: NE green, NW blue, SE orange, SW magenta, `kUnknown` grey. (Replaces the old single orange.) Same topic as before — an existing RViz display recolors automatically. |

All in `world_frame_id` (`map`). To see the cross-lines add a `Marker` display on
`/quadrant/cross_marker`.

---

## Parameters

Declared planner-side (`ReadParameters`, slash style like `navigation_graph/*`),
read by `QuadrantManager::ReadParameters`. Override in the scenario yaml
(`config/go2w_bag_direct.yaml`).

| Param | Default | Meaning |
|---|---|---|
| `quadrant/kUpdateInterval` | 4 | Self-throttle: fit + viz every Nth `Update()` call (clamped ≥ 1). |
| `quadrant/kWarmupMinRooms` | 2 | Min alive rooms before fitting. |
| `quadrant/kWarmupMinVertices` | 8 | Min total polygon vertices before fitting. |
| `quadrant/kFreezeStableCycles` | 5 | Consecutive angle-stable fits → freeze. |
| `quadrant/kFreezeAngleEpsDeg` | 2.0 | Angle-stability tolerance (converted to rad). |
| `quadrant/kMaxWarmupCycles` | 60 | Hard cap → freeze the last fit no matter what. |
| `quadrant/kCrossLineWidth` | 0.08 | Cross-line marker width (m). |
| `quadrant/world_frame_id` | `map` | Marker frame. |
| `quadrant/debug_log` | false | **Runtime** toggle for verbose warmup/fit logs (re-read each cycle by the planner; the FREEZE line is always shown). |
| `scene_graph_export.compass_radius_m` | 0.0 | Compass arm length (m); `≤ 0` → auto (½·max extent). Lives in `scene_graph_export.yaml` because the compass is built in the exporter. |

---

## Cost

Designed to stay well under the NavGraph reconcile (which is `O(keypose edges)` and
runs synchronously in `execute()`):

- `minAreaRect` + `boxPoints`: `O(V log V)` over a few hundred polygon vertices, and
  runs **only until frozen** (an early return short-circuits all fitting after the
  latch). Throttled by `kUpdateInterval`.
- `TagAreas`: `O(nodes)` (~hundreds), one map lookup + 2 dot products each. Observed
  `tag_areas 0.00 ms` in the NavGraph Update log.
- Node recolor / cross-lines: `O(nodes)` / `O(Σ vertices)`, piggyback existing
  throttles.

---

## Gotchas

- **Pre-freeze is the only path to `"area":"unknown"`** for a roomed, exported node
  (and the only time `compass` is omitted). Warmup is short and hard-capped, so real
  snapshots have valid axes. After freeze, every exported waypoint is a real
  NE/NW/SE/SW.
- **One global orientation.** A single rect cannot represent a building with wings at
  different angles; minority-oriented rooms are bucketed by the dominant axes. Only
  `QuadrantManager` would change for a future per-room fit — `AssignArea`, the JSON
  schema, and the viz are already per-room-origin.
- **Near-square / exactly-45° environments.** Canonicalization removes the 90° flip;
  the residual exact-45° pathology is mitigated by the stability gate + hard cap.
- **Uneven buckets are expected.** Rooms aren't symmetric about their centroids, so a
  room can have many NW nodes and no SE node — that's geometry, not a bug.
- **`wp_0` is bucketed, not "center".** Per design the interior point gets a real
  quadrant relative to the centroid; there is no `"center"` area value.
- **Frame.** Axes + assignment live in the source/map frame; the exporter transforms
  the *finished* compass points through `world_from_source`. Node positions /
  centroids carry no transform of their own.

---

## Verifying a run

1. Toggle the log live: `ros2 param set /tare_planner_node quadrant/debug_log true`.
   Expect exactly one `axes FROZEN …` line, after which fit logs stop (proves the
   hard throttle).
2. NavGraph Update log shows the per-quadrant tally:
   `… | areas NE=5 NW=8 SE=3 SW=3 unk=0` (all `unk` before freeze, none after).
3. Inspect a snapshot (`output/scene_graph/<run>/snapshot_*.json`): every waypoint
   incl. `wp_0` has `"area"` ∈ the four values; objects have **no** `area`;
   `layout.metadata.compass` has `center/north/south/east/west`. Sanity check:
   `(east − center)` normalized equals the frozen `east`, and all four arms share one
   radius.
4. RViz: each room shows a red/blue cross at its centroid; a node visually NE of its
   centroid is green and its JSON `area` is `"northeast"` — same field, so they can't
   disagree.

---

## Quick API reference

```cpp
// building_axes.h (Eigen-only, ROS-free)
enum class Area { kUnknown=-1, kNorthEast, kNorthWest, kSouthEast, kSouthWest };
const char* AreaName(Area);
struct BuildingAxes { Eigen::Vector2d east, north; bool valid; };
Area AssignArea(const Eigen::Vector2d& p, const Eigen::Vector2d& origin, const BuildingAxes&);

// QuadrantManager (quadrant_ns) — owned and driven by the planner
explicit QuadrantManager(rclcpp::Node::SharedPtr nh);
void Update(const std::map<int, representation_ns::RoomNodeRep>& rooms, bool debug_log = false);
const navgraph_ns::BuildingAxes& GetAxes() const;   // valid == false until frozen
bool IsFrozen() const;
```
