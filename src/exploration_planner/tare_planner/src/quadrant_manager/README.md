# Quadrant / Room-Area pipeline

Buckets every scene-graph **waypoint** into one cell of a **3×3 grid** of its room —
the `center` cell plus the eight compass directions (`north`, `northeast`, `east`,
`southeast`, `south`, `southwest`, `west`, `northwest`) — so the exported JSON (and
its LLM / frontend consumers) can reason about "the NE corner of the kitchen". The
grid is aligned to a **single global building orientation** that is **frozen once
stable**; each room's **oriented bounding box** is split into equal thirds along each
axis, and a point's cell is which third it falls in on each axis.

The orientation comes from the building's **walls**. `room_segmentation` already
detects persistent wall planes, so it estimates the **dominant Manhattan wall
direction** and publishes it on `/wall_axis`. `QuadrantManager` consumes that as the
**primary** source. Only if no confident wall axis ever arrives does it fall back —
**at a hard cap** — to a `cv::minAreaRect` over the room polygons (a bounding-box
orientation that does *not* track walls; the original method, now demoted to a
last-resort).

The feature spans four places but is small and additive:

1. **`room_segmentation::publishWallAxis()`** estimates the dominant wall orientation
   from `plane_infos_` and publishes `tare_planner/msg/WallAxis` on `/wall_axis`.
2. **`QuadrantManager`** (this module) subscribes, **freezes** the global axes (wall
   primary, min-rect fallback at the cap), builds each room's 3×3 grid, and draws the
   per-room "#" glyphs.
3. The **[NavGraph](../navgraph/README.md)** tags each node's `area` live (node
   coloring), reading the frozen axes.
4. The **[scene-graph exporter](../scene_graph_exporter/README.md)** writes `"area"`
   onto every waypoint and a `compass` into `layout.metadata`.

All consumers read the **same** per-room `RoomGrid`s (built once by `QuadrantManager`),
so the live RViz node color and the JSON `area` are literally the same field — they
can never disagree.

---

## Files

| File | Contents |
|---|---|
| `include/navgraph/building_axes.h` | Shared primitives: `enum class Area` (9 cells), `AreaName()`, `struct BuildingAxes`, `struct RoomGrid` + `MakeRoomGrid()`, pure `AssignArea(p, grid)`. **Eigen-only, ROS-free, OpenCV-free** — the lowest layer, usable by the ROS-free `NavNode` and the OpenCV-free exporter. `navgraph_ns`. |
| `include/quadrant_manager/quadrant_manager.h` | `QuadrantManager` class declaration (`quadrant_ns`). |
| `src/quadrant_manager/quadrant_manager.cpp` | `/wall_axis` subscription, `AxesFromAngle` (wall yaw → axes), the freeze state machine (wall primary, min-rect at cap), `FitAxes` (the min-rect fallback), `BuildRoomGrids` (per-room 3×3 grids), and the "#" grid viz. The **only** TU pulling `<opencv2/imgproc.hpp>`. |
| `msg/WallAxis.msg` | The wall-orientation message: `yaw_rad`, `confidence`, `support_length`, `valid`. |

Touched elsewhere:

| File | Change |
|---|---|
| `room_segmentation.cpp` / `room_segmentation_node.h` | `publishWallAxis()` (the wall-orientation estimator) + `/wall_axis` publisher + `wall_axis/*` params. |
| `navgraph_types.h` | `NavNode` gains `Area area`. |
| `navgraph.{h,cpp}` | `Update()` takes `room_grids`; `TagAreas()` buckets each node into its room's 3×3 grid; node marker colored on the 4 corner cells only. |
| `scene_graph_exporter.{h,cpp}` | `Build()` takes `axes` + `room_grids`; `BuildRoomJson()` takes the room's grid (wp_0 area); `"area"` per waypoint; `compass`; `ComputeAabbCenterSource()`. |
| `sensor_coverage_planner_ground.{cpp,h}` | Owns `quadrant_mgr_`; declares `quadrant/*` params; orders `quadrant_mgr_->Update()` → `BuildRoomGrids()` → `navgraph_->Update()`; passes the grids into `Build()`. |

---

## Where it sits in the pipeline

```
 room_segmentation: plane_infos_ (wall planes)
        │  publishWallAxis()  (~2 Hz)
        ▼
   /wall_axis  {yaw_rad, confidence, support_length, valid}
        │  (subscription; latest cached, no mutex)
        ▼
   QuadrantManager::Update(rooms)        ◄── Representation rooms (polygons + centroids)
      │   once per planning cycle (execute()), self-throttled
      ├─ PRIMARY : freeze on a CONFIDENT + STABLE wall yaw  → source=wall
      ├─ FALLBACK: min-rect FitAxes ONLY at the hard cap    → source=minrect_cap
      └─ PublishCrossLines(): per-room "#" glyph ──► RViz /quadrant/cross_marker
            │  GetAxes() + BuildRoomGrids(rooms)  (per-room 3×3 grids; invalid until freeze)
            ├───────────────────────────────┐
            ▼                                ▼
   NavGraph::Update(..., room_grids)   SceneGraphExporter::Build(..., axes, room_grids)
      └─ TagAreas(): node.area =          ├─ waypoint "area" (wp_0 via its grid)
         AssignArea(node.pos, grid)       └─ layout.metadata.compass (uses axes)
      └─ corner cells colored ──► /navgraph/node_marker
```

---

## The model in one paragraph

The robot-start `map` frame is at an arbitrary yaw to the building walls, so raw map
axes would cut rooms diagonally and "NE of the kitchen" would mean nothing physical —
we want axes that **align with the walls**. `cv::minAreaRect` over room polygons
*doesn't* deliver that: it fits a bounding box of room *extent* (L-rooms, corridors,
unions of rooms), not wall *direction*. So the primary source is the **dominant
Manhattan wall orientation** estimated from the wall planes: assuming ~90% of walls
share two orthogonal directions, the wall set has a single grid angle θ∈[0,90°), and
the axes are {θ, θ+90°}. Building orientation is a *static* property, so it is
**frozen once** (when a confident wall estimate has held steady) and reused. Each room
is then divided into a **3×3 grid**: its polygon is projected onto the frozen axes to
get an oriented bounding box, each axis extent is split into equal thirds, and a
point's cell is which third it lands in on each axis (middle × middle = `center`).
Assignment is a pure, stateless function of (point, that room's `RoomGrid`).

---

## Core data structures (`building_axes.h`)

### `enum class Area`
`kUnknown(-1)`, then the nine 3×3 cells: `kCenter(0)`, `kNorth`, `kNorthEast`, `kEast`,
`kSouthEast`, `kSouth`, `kSouthWest`, `kWest`, `kNorthWest`. `AreaName(Area)` maps to
the canonical strings (`"center"`, `"north"`, … / `"unknown"`) emitted into the JSON
and used in the RViz labels, so the two cannot drift.

### `struct BuildingAxes`
```cpp
Eigen::Vector2d east{1,0}, north{0,1};   // XY only (the stack is gravity-aligned)
bool valid = false;                       // false until the freeze
```
`east` is canonicalized to within ±45° of map +X and `north = east` rotated +90° CCW
(so `north.y > 0`, "up"). A **default-constructed `BuildingAxes{}` is the "not ready"
sentinel** — `AssignArea` returns `kUnknown` and the exporter omits the compass.

### `struct RoomGrid` + `MakeRoomGrid(...)`
A room's 3×3 grid: the global `axes`, the projection `origin` (room centroid), and the
two third-boundaries on each axis (`e_lo/e_hi`, `n_lo/n_hi`, origin-relative).
`MakeRoomGrid(origin, axes, e_min, e_max, n_min, n_max, center_fraction)` (pure) splits
each oriented-bbox extent into bands: the center band spans `center_fraction` of the
extent (`1/3` ⇒ equal thirds). Invalid axes / degenerate extent ⇒ an invalid grid.

### `Area AssignArea(p, grid)` (pure)
Project `d = p − origin` onto the axes; classify `e = d·east` and `n = d·north` into
low / middle / high by the grid's third-boundaries; map the pair to a cell
(middle × middle = `center`). Boundary points fall into the middle (center-ward) band.
2D only. `kUnknown` if the grid is invalid (axes not frozen).

### `NavNode::area` (`navgraph_types.h`)
The per-node 3×3 cell, tagged each reconcile from the node's room `RoomGrid`. Stored on
the node so it is available **live** (RViz color) and read verbatim by the exporter.
`kUnknown` until the axes freeze / a node has no room.

---

## The wall-axis estimator (`room_segmentation::publishWallAxis`)

Runs once per segmentation cycle (~2 Hz), after the wall planes are finalized, and
publishes `/wall_axis`. Robust to the ~10% non-Manhattan walls by construction.

**Votes.** Each persistent wall plane that is `alive`, **not** `merged` (one entry =
one real wall — no double-count), and at least `min_wall_len` long contributes its
along-wall angle `a = atan2(u_dir.y, u_dir.x)` weighted by its length `w = width`.
`total_w = Σ w`.

**Fold to the grid (×4).** `φ = wrap2π(4·a)`. Multiplying by 4 collapses the two
orthogonal Manhattan families *and* the 180° line ambiguity onto one cluster on the
circle; the off-grid minority scatters elsewhere.

**Mode, fully circular.** A length-weighted histogram over `[0,2π)` is **3-tap
ring-smoothed** then `argmax`'d for a seed. *Why circular matters:* an axis-aligned
(~0°) building — the common case — has its cluster straddling the φ=0/2π seam; a
*linear* histogram/mean would split it and return ~45° garbage. Ring smoothing +
circular inlier distance + circular mean handle the seam correctly.

**Refine.** `refine_iters` times: gather inliers within `±window` (`inlier_window_deg`
in θ, ×4 in φ) of the seed by circular distance, take their length-weighted **circular
mean** (`atan2(Σw·sinφ, Σw·cosφ)`). Then `θ = φ̂/4`, folded to `[0,π/2)`.

**Outputs.** Mode (not mean) rejects the off-grid minority; length-weighting suppresses
short strays.
- `support_length = Σ w over the inliers` — total length of wall *on* the dominant
  grid (m). The absolute evidence behind the orientation.
- `confidence = support_length / total_w` — the **fraction of total wall length aligned
  to the grid**, [0,1]. ~1 = a clean single grid; ~0.5 = two competing grids / lots of
  off-axis wall.
- `valid = (total_w ≥ min_total_len && support_length > 0)`.

A two-grid building yields two φ peaks → the mode picks one and `confidence ≈ 0.5`,
which the consumer rejects (→ min-rect cap).

---

## QuadrantManager — source, freeze, draw

`Update(rooms, debug_log)` runs once per planning cycle (before `navgraph_->Update`),
self-throttled to every `kUpdateInterval`-th call. The latest `/wall_axis` is cached
by the subscription callback into a small POD (`wall_axis_`); the planner and
room_segmentation are both single-threaded `rclcpp::spin`, so **no mutex** is needed.

### Source — wall primary, min-rect only at the cap
- The **primary** source is the cached `/wall_axis`, *trusted* when
  `valid && confidence ≥ kWallMinConfidence && support_length ≥ kWallMinSupportM`.
  `AxesFromAngle(yaw)` turns the grid angle into a `BuildingAxes` (canonicalized to
  east within ±45° of +X via `std::remainder`; `north = east + 90° CCW`).
- The **min-rect** `FitAxes` (`cv::minAreaRect` over room polygons, corner/`boxPoints`
  canonicalization that absorbs the 90° rect ambiguity, OpenCV-version-robust) is the
  original method, now used **only** as the hard-cap fallback — never during normal
  warmup.

### Freeze — three triggers
Each eligible cycle past the **geometry gate** (≥ `kWarmupMinRooms` alive room with
≥ `kWarmupMinVertices` polygon vertices) advances `warmup_cycles_`. Then:

1. **`source=wall`** (normal) — while the wall axis is trusted, its yaw is tracked for
   stability (within `kFreezeAngleEpsDeg`, mod 90°); after `kFreezeStableCycles`
   **consecutive** stable cycles it freezes. A non-confident cycle resets the streak
   (and `have_last_angle_`), so a flickering wall can't accumulate a false streak.
2. **`source=wall_cap`** — at the hard cap (`kMaxWarmupCycles`), if a trusted wall axis
   is available but never stabilized, freeze it anyway.
3. **`source=minrect_cap`** — at the cap with no trusted wall axis, freeze the min-rect
   fit. This is the only place `FitAxes` runs.

`warmup_cycles_` advances on *geometry alone*, so the cap is a real timeout even if a
wall axis never arrives (and single-room environments no longer hang). The one-time
freeze line names the trigger (`stable < kFreezeStableCycles` on a `*_cap` line ⇒ the
timeout fired, not stability):

```
[quadrant] axes FROZEN source=wall after 7 cycles (stable=5): yaw=18.0 deg conf=0.86 support=24.3 m | east=(0.951,0.309) north=(-0.309,0.951)
```

Before freeze, `GetAxes().valid == false` → grey nodes, no "#" glyphs, no compass;
the markers appear at the freeze instant. Once frozen the direction is latched and
never refit (only each room's "#" *position / size* keeps tracking the live
centroid/polygon).

---

## How nodes get tagged — `NavGraph::TagAreas`

Runs after `TagRooms` (needs `room_id`) and before `AssignNames`. For each node with a
known room and grid: `node.area = AssignArea({pos.x, pos.y}, room_grids[room_id])`. No
room / no grid / unfrozen axes → `kUnknown`. The planner builds `room_grids` once via
`QuadrantManager::BuildRoomGrids(GetRoomNodesMap())` and hands the **same** map to both
the NavGraph and the exporter — so node cells and `wp_0` cells agree by construction.

---

## How it reaches the JSON — exporter

Each waypoint object gains an `"area"`:

```jsonc
{ "id": "office-room_22-wp_3", "x": .., "y": .., "z": .., "area": "northeast" }
```

- **`wp_1..N`** (NavGraph nodes) read `AreaName(node->area)` — the field tagged
  upstream. No recomputation.
- **`wp_0`** (the room interior point) is assigned the same way, from the same room
  grid: `AreaName(AssignArea(interior_point, room_grids[room.id_]))`. It can be
  `"center"` or any of the eight directions — wherever the interior point falls.

A **compass** is added to `layout.metadata` so a frontend can draw an oriented compass
at the right place:

```jsonc
"compass": {
  "center": { "x", "y", "z" },        // environment-AABB center
  "north":  { … }, "south": { … },    // center ± R · north
  "east":   { … }, "west":  { … }     // center ± R · east
}
```

The 5 points are computed in the **source frame** (`ComputeAabbCenterSource`) then each
transformed by `world_from_source` (like every other coordinate), so the compass aligns
with the waypoints in whatever frame the snapshot is in. Radius `R = compass_radius_m`
if > 0, else **auto** = ½·max(width, height). The compass is **omitted** when the axes
are not yet frozen or there are no rooms (a frontend treats absence as "not ready").

---

## Visualization

| Topic | Type | Style |
|---|---|---|
| `quadrant/cross_marker` | `Marker` `LINE_LIST` | Per alive room, a small **"#" glyph** at the center of its 3×3 grid — the four third-boundary segments (`e_lo/e_hi/n_lo/n_hi`) drawn short with a 0.45×cell overhang, **not** spanning the room. North-running strokes blue, east-running strokes red. One marker, rewritten each publish so dead rooms vanish. Width `kCrossLineWidth`. Only published once the axes are valid. |
| `navgraph/node_marker` | `Marker` `POINTS` | NavGraph nodes, colored only on the **four corner cells** (NE green, NW blue, SE orange, SW magenta); `center` + the four edge cells (N/E/S/W) + `kUnknown` are grey. The full 9-way area is still in the JSON — the node viz just highlights corners. Same topic — an existing RViz display recolors automatically. |

All in `world_frame_id` (`map`). To see the "#" glyphs add a `Marker` display on
`/quadrant/cross_marker`.

---

## Parameters

room_seg params declared in `room_segmentation` (`wall_axis/*`); the rest declared
planner-side and read by `QuadrantManager::ReadParameters`. Override in the scenario
yaml (`config/go2w_bag_direct.yaml`).

| Param | Default | Meaning |
|---|---|---|
| `wall_axis/min_wall_len` (room_seg) | 0.5 | drop walls shorter than this (m) from voting |
| `wall_axis/hist_bins` (room_seg) | 90 | φ histogram bins (4°/bin = 1° in θ) |
| `wall_axis/inlier_window_deg` (room_seg) | 6.0 | inlier half-window in θ-degrees (×4 in φ) |
| `wall_axis/min_total_len` (room_seg) | 3.0 | min total wall length for a `valid` estimate (m) |
| `wall_axis/refine_iters` (room_seg) | 2 | circular-mean refine iterations |
| `quadrant/kWallMinConfidence` | 0.5 | min `/wall_axis` confidence to trust it as primary |
| `quadrant/kWallMinSupportM` | 3.0 | min `/wall_axis` aligned wall length to trust it (m) |
| `quadrant/kCenterFraction` | 0.333 | 3×3 center-band size as a fraction of each room extent (1/3 = equal thirds) |
| `quadrant/kWarmupMinRooms` | 1 | geometry gate (≥ this many alive rooms); fixes single-room/timeout |
| `quadrant/kWarmupMinVertices` | 8 | geometry gate (≥ total polygon vertices) |
| `quadrant/kFreezeStableCycles` | 5 | consecutive stable wall-yaw cycles → freeze |
| `quadrant/kFreezeAngleEpsDeg` | 2.0 | yaw-stability tolerance (mod 90°) |
| `quadrant/kMaxWarmupCycles` | 60 | hard cap → freeze (`wall_cap`, else `minrect_cap`) |
| `quadrant/kUpdateInterval` | 4 | self-throttle: do work every Nth `Update()` call (clamped ≥ 1) |
| `quadrant/kCrossLineWidth` | 0.08 | cross-line marker width (m) |
| `quadrant/world_frame_id` | `map` | marker frame |
| `quadrant/debug_log` | false | **runtime** toggle for verbose warmup/fit logs (FREEZE line always shown) |
| `scene_graph_export.compass_radius_m` | 0.0 | compass arm length (m); `≤ 0` → auto (½·max extent). Lives in `scene_graph_export.yaml`. |

---

## Cost

- **Estimator** (room_seg): a handful of planes × (atan2 + ring-smoothed histogram + 2
  refine passes) at ~2 Hz — negligible.
- **QuadrantManager**: one POD copy + a couple of dot products per cycle. `FitAxes`
  (min-rect) now runs at most a few times near the cap, not every warmup cycle.
- **TagAreas / node recolor / "#" glyphs**: `O(nodes)` / `O(Σ vertices)`, piggyback
  existing throttles. Observed `tag_areas 0.00 ms` in the NavGraph Update log.

---

## Gotchas

- **Min-rect only at the cap.** Normal operation freezes on the wall axis; the
  bounding-box min-rect is the timeout fallback for "no confident wall ever" (sparse
  walls / large open spaces / strongly non-Manhattan). It does **not** track walls — so
  a `source=minrect_cap` freeze is the degraded case, by design.
- **Axis-aligned (~0°) buildings** are handled by the circular estimator (the φ=0/2π
  seam). Regression test: such a building must freeze near 0°, **not** ~45°.
- **Two-grid / non-Manhattan buildings.** One global orientation; the dominant grid
  wins and the minority is bucketed by it. Detected as low `confidence` → min-rect cap.
  A future per-room fit would change only the estimator/source (`AssignArea`, the JSON
  schema, and the viz are already per-room-origin).
- **Pre-freeze is the only path to `"area":"unknown"`** for a roomed, exported node (and
  the only time `compass` is omitted). Freeze is early once walls + rooms exist.
- **9-cell vocabulary.** Each waypoint's `"area"` is one of nine values (`center` + 8
  directions) — a vocabulary change the JSON/LLM consumer must learn (`center` is now a
  real value). `wp_0` is bucketed like any other point and can be `center`.
- **Uneven cell occupancy is expected.** Rooms aren't filled uniformly, so a room can
  have several `northwest` nodes and no `southeast` node — geometry, not a bug.
- **Frame.** Axes + assignment live in the source/map frame; the exporter transforms
  only the *finished* compass points through `world_from_source`.

---

## Verifying a run

1. `ros2 topic echo /wall_axis` → `yaw_rad / confidence / support_length` evolve as
   walls accumulate; `valid` flips true past `min_total_len`.
2. `ros2 param set /tare_planner_node quadrant/debug_log true` → repeated
   `fitting: wall_ok=1 yaw=… stable=x/5 …`, then exactly one `axes FROZEN source=wall …`
   (or `source=wall_cap` / `source=minrect_cap` on a no-wall/ambiguous run).
3. NavGraph Update log: `… | areas C=.. N=.. NE=.. E=.. SE=.. S=.. SW=.. W=.. NW=.. unk=0`
   (all `unk` before freeze, none after).
4. RViz: post-freeze a **"#" glyph** appears at each room's center, aligned to the wall
   grid (eyeball vs `/walls`); the four corner cells' nodes are colored, the rest grey.
5. Snapshot JSON: `"area"` per waypoint + `layout.metadata.compass` present, **identical
   shape** to the min-rect version — only the axis angle differs.

---

## Quick API reference

```cpp
// building_axes.h (Eigen-only, ROS-free)
enum class Area { kUnknown=-1, kCenter, kNorth, kNorthEast, kEast, kSouthEast, kSouth, kSouthWest, kWest, kNorthWest };
const char* AreaName(Area);
struct BuildingAxes { Eigen::Vector2d east, north; bool valid; };
struct RoomGrid { Eigen::Vector2d origin; BuildingAxes axes; double e_lo,e_hi,n_lo,n_hi; bool valid; };
RoomGrid MakeRoomGrid(origin, axes, e_min, e_max, n_min, n_max, center_fraction);  // oriented-bbox thirds
Area AssignArea(const Eigen::Vector2d& p, const RoomGrid& grid);                   // -> one of 9 cells

// tare_planner/msg/WallAxis   (room_segmentation -> /wall_axis, the primary source)
//   std_msgs/Header header; float64 yaw_rad; float64 confidence; float64 support_length; bool valid

// QuadrantManager (quadrant_ns) — owned and driven by the planner; subscribes /wall_axis
explicit QuadrantManager(rclcpp::Node::SharedPtr nh);
void Update(const std::map<int, representation_ns::RoomNodeRep>& rooms, bool debug_log = false);
std::map<int, navgraph_ns::RoomGrid> BuildRoomGrids(const std::map<int, RoomNodeRep>& rooms) const;
const navgraph_ns::BuildingAxes& GetAxes() const;   // valid == false until frozen
bool IsFrozen() const;
```
