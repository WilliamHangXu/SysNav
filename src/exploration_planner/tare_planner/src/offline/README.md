# Offline scene-graph pipeline

Builds a complete per-floor **scene graph** from one training-session folder — no
live robot, no streaming: `scans.pcd` (full-building FAST-LIO map) + `blueprint.yaml`
(per-floor robot z) + per-floor `keypose_graph.json` dumps go in; per-floor
`scene_graph.json` (plus every layer artifact and debug image) comes out. Room
*labels* are the one missing piece (`unknown-room_<id>` until the offline labeling
stage exists); objects are a future layer.

The DAG: **room segmentation** (rooms layer) → **navgraph** (downsampled keypose
graph, per floor) → **assembly** (scene_graph.json, per floor); labeling will slot in
before assembly. The segmentation core — two-source wall extraction (region-grown
vertical planes ∪ wall-band column histogram) → dilate → `cv::watershed` → door
detection → per-room polygon / centroid / interior point — is **lifted** from the
online [`room_segmentation`](../room_segmentation/README.md) node (copy-first, the
online node is untouched); most of this README is the deep dive into that stage.

**Naming contract (keep strict):** a *navgraph* is ONLY the downsampled keypose graph
(nodes + reachability edges). The *scene graph* is the assembled whole — rooms +
navgraph + future objects. Every cross-layer relationship (waypoint∈room tagging,
`wp_<n>` naming, 3×3 areas, compass) lives in the **assembler**; the layer producers
stay relationship-free.

All stages live in one ROS-free static library (`offline_scene_graph_core`), one
module per layer:

| Module (`include/offline/` + `src/offline/`) | Owns |
| --- | --- |
| `offline_types.h` | shared PODs + voxel/color helpers |
| `offline_room_segmentation.{h,cpp}` | rooms layer (this README's deep-dive subject) |
| `offline_navgraph.{h,cpp}` | keypose-dump loader, downsampler, navgraph.json ([below](#offline_navgraph)) |
| `offline_scene_graph.{h,cpp}` | rooms-layer readers, assembler, debug overlay ([below](#offline_scene_graph)) |
| `offline_pipeline.{h,cpp}` | the whole DAG as one in-process call |

Config: **`config/offline_scene_graph.yaml`** — the pipeline's own flat yaml (same
key names as the online scenario yamls so tuning transfers; values copied from
`go2w_bag_direct.yaml`).

Two entry points, both thin: the **production `offline_scene_graph_node`** — the only
ROS code in the pipeline — listens for a signal and runs the whole DAG
([below](#offline_scene_graph_node-production)); `offline_cli` is the ROS-free debug
CLI with one subcommand per stage (`run | seg | navgraph | assemble`).

---

## Running

```bash
colcon build --packages-select tare_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The whole pipeline, one command (production runs the same thing through
[the node](#offline_scene_graph_node-production) instead):

```bash
./install/tare_planner/lib/tare_planner/offline_cli run \
    --session /path/to/training/20260722_060723 \
    --config src/exploration_planner/tare_planner/config/offline_scene_graph.yaml \
    --building AlphaZ \
    [--out <dir>] [--floor floor_1]
```

### Rooms layer alone (segmentation tuning loop)

```bash
./install/tare_planner/lib/tare_planner/offline_cli seg \
    --pcd scans.pcd \
    --floors blueprint.yaml \
    --config src/exploration_planner/tare_planner/config/offline_scene_graph.yaml \
    --out output/offline/alphaz_building \
    [--floor floor_1]
```

| Flag | Required | Meaning |
| --- | --- | --- |
| `--pcd` | yes | Full-building point cloud (only `x y z` fields are read). Must be gravity-aligned; all outputs inherit this cloud's frame. |
| `--floors` | yes | Per-floor z index (see below). |
| `--out` | yes | Output root; one subdirectory per floor is created. |
| `--config` | no | Scenario yaml or flat yaml with tuning parameters. Without it, all defaults apply. |
| `--floor` | no | Process a single floor by name (tuning loop). |

Runtime on the AlphaZ building maps (~7–9 M points, 2 floors): ~1 s per floor after
the one-time PCD load; the whole pipeline (`run`) is ~1 s total per building.

Exit code is non-zero if any floor fails (empty slab) or no floor was processed.

### The floors yaml (`--floors`)

Any yaml with a `floors:` sequence works; only two fields per entry are read —
everything else is ignored (the on-robot blueprint tool's `blueprint.yaml` can be
passed as-is):

```yaml
floors:
  - name: floor_1
    collision_range: [-3.886, -2.386]   # [0] = the robot's z on that floor
  - name: floor_2
    collision_range: [0.4, 1.3]
```

`collision_range[0]` is the **robot's z** on that floor (roughly 0.5 m above the floor
surface for the Go2), not the floor surface itself. From it, each floor's crop bands
are derived:

```
slab      = [robot_z − slab_below,          robot_z + ceilingHeight_]   # points used at all
wall band = [robot_z + wall_thres_height_,  robot_z + ceilingHeight_]   # histogram source
```

Floors are sorted by `robot_z`, and each slab's top is additionally clamped to the
next floor's slab bottom (`next robot_z − slab_below`) so the slabs stay disjoint in
stairwells.

---

## Pipeline at a glance

```
scans.pcd (whole building)
    │  per floor:
    ▼
PassThrough z-slab  ─►  VoxelGrid (exploredAreaVoxelSize)
    │
    ▼
NormalEstimationOMP (kSearch = normal_search_num)      ── one shot, not incremental
    │
    ▼
auto-sized 2D grid from cloud XY bounds + margin        ── room_x/y/z params ignored
    │
    ▼
updateVoxelMap()  ─►  navigable_map_ (top-down count)
                      wall_hist_     (count, wall band only)
    │
    ├──────────────────────────────┐
    ▼                              ▼
getWall()                      wall_from_hist
(RegionGrowing → vertical      (float threshold:
 planes → same-pass merge       hist ≥ factor × max)
 → 2D footprint quads)
    └──────────►  wall ∪ hist  ◄───┘
                      │
        outside-boundary cleanup (hole punching ≥ min_hole_area,
        components > min_component_area; two variants kept — see below)
                      │
        dilate ×dilation_iteration → seeds > min_room_size
                      │
                cv::watershed
                      │
        room ids = watershed labels 1..N (one shot, no lifecycle)
                      │
    ┌─────────────────┼──────────────────┐
    ▼                 ▼                  ▼
per-room polygon,  door detection     adjacency
centroid, area,    (borders touching  (rooms.json
interior point     exactly 2 rooms)    neighbors)
(PIA)
```

As in the online node, **two** boundary variants are deliberately kept: the
walls-subtracted one seeds the rooms; the histogram-only-subtracted one is the
watershed background + flood image. Don't collapse them.

### Deliberately absent relative to the online node

- **Freespace/occupancy machinery** (`updateFreespace`, `updateStateVoxel`,
  `state_map_`, the latched-free wall masking, the plane free-cull). It corrects
  *online accumulation* artifacts — dynamics, glass reflections — that a clean
  one-shot map does not have. If a pcd turns out dirty, fix it at map-build time
  (hit-count filter, batch ray-carving), don't port the online machinery.
- **All incremental state**: room lifecycle reconciliation (`updateRooms`),
  monotonic id allocation, cross-frame plane merge/prune, incremental normal
  estimation, demo freeze, callbacks/timer. One shot: **room ids are the raw
  watershed labels 1..N** and `show_id == id`. Ids are only stable for a fixed
  input + config; rerunning after retuning renumbers rooms.
- **`robot_position_`**: there is no robot. The in-range plane kill and the
  current-room / `is_connected` BFS are gone; every output z (room centroid,
  interior point, door centroid) is the floor's `robot_z`.

The only plane merging left is the same-pass self-merge (`isPlaneSame` /
`mergePlanes`), which fuses one physical wall that region-growing split into two
clusters.

---

## Outputs

```
<out>/<floor_name>/
├── room_mask.png       # CV_16U label image, 0 = background, pixel value = room id
├── room_mask_vis.png   # idToColor-colored mask, door pixels in red
├── mask_meta.json      # the pixel↔world contract (below)
├── rooms.json
├── doors.json
└── debug/              # every intermediate, saved unconditionally
```

### Orientation — read this before consuming any image

- `room_mask.png` and `room_mask_vis.png` are in **raw grid orientation**:
  `row = world +x`, `col = world +y`. These are the machine artifacts; downstream
  stages (offline navgraph, labeling, export) consume these.
- `debug/*.png` are **transposed + vertically flipped** for human viewing, matching
  the online node's `saveImageToFile` dumps so side-by-side comparison works.

### `mask_meta.json`

Everything needed to map pixels to world coordinates and to reproduce the run:

- `frame` — the coordinate frame label (from config, default `map`). All world
  coordinates in all three jsons are in the input pcd's frame; this field only
  *names* it.
- `robot_z`, `slab_z`, `wall_band_z` — the resolved z bands for this floor.
- `resolution`, `grid_dims`, `origin_shift`, `bbox`, `image_dims` — grid geometry.
  The pngs are cropped to `bbox` (the non-zero hull of the navigable map + margin).
- `pixel_to_world` — the formula, spelled out:
  `x = (row + bbox.row[0] − origin_shift[0]) × resolution`,
  `y = (col + bbox.col[0] − origin_shift[1]) × resolution`
  (cell corner; add `resolution/2` for the cell center).
- `params` — the fully resolved parameter set actually used.

### `rooms.json`

```jsonc
{
  "floor": "floor_1",
  "robot_z": -3.886,
  "rooms": [
    {
      "id": 3,                      // == pixel value in room_mask.png
      "show_id": 3,                 // == id offline (kept for online-schema parity)
      "area_m2": 53.7,
      "pixel_count": 5370,
      "centroid": [x, y, robot_z],        // mean of the room's cells
      "interior_point": [x, y, robot_z],  // pole of inaccessibility — guaranteed inside
      "polygon": [[x, y], ...],           // largest outer contour, world XY
      "neighbors": [1, 2, 4]              // door-connected room ids
    }
  ]
}
```

Use `interior_point`, not `centroid`, as a room's representative point — the centroid
of an L-shaped or ring-shaped room can fall outside it. The interior point is the
medoid of the distance-transform near-max ridge (same PIA as online).

### `doors.json`

```jsonc
{
  "floor": "floor_1",
  "doors": [
    {
      "id": 0,                 // global sequential index
      "door_id": 0,            // per-room-pair instance (two rooms can share 2+ doorways)
      "rooms": [3, 5],         // room_a < room_b
      "centroid": [x, y, robot_z],
      "pixel_count": 14
    }
  ]
}
```

A door component that touches ≠ 2 rooms is skipped (logged to stderr), same rule as
online.

Border components connecting the **same room pair** whose nearest pixels are within
`door_merge_gap_m` (default 0.4 m) are merged into one door before output — the 3×3
label filter can chop one physical opening into several fragments (typically a
normal-size component plus a 1-px shard a couple of pixels away). The merged
centroid is the mean over the union of pixels, so it is pixel-weighted
automatically. A genuine double doorway — two openings with a real wall pier
between them — stays split, since piers are essentially never that narrow.

On an empty slab or zero watershed seeds, the tool still writes `rooms.json` /
`doors.json` with an `"error"` field and empty arrays, so downstream globbing never
sees a half-missing floor.

### `debug/` images

Saved unconditionally (offline = cheap), in pipeline order. Names shared with the
online `isDebug` dumps mean the same thing there:

| File | What it shows |
| --- | --- |
| `full_map_1.png` | Raw observed-area mask (thresholded navigable map). |
| `wall_planes_color.png` | Per-plane footprints, one color per surviving plane (offline stand-in for the `/walls` cloud). |
| `wall_mask_from_planes.png` | 2D wall mask drawn from the plane quads (+ gap-closing extensions). |
| `walls_skeleton_hist_1_raw.png` | Normalized wall-band histogram. |
| `wall_from_plane.png` / `wall_from_hist.png` | The two wall sources, binarized. |
| `wall_all.png` | Their union. |
| `full_map.png` / `full_map_connected.png` | The two boundary variants (room-seeding vs. watershed-background). |
| `new_boundary.png` | Dilated boundary used for seed extraction. |
| `seed_mask.png` | Colored watershed seeds that survived `min_room_size`. |
| `full_map_color.png` | The 3-channel flood image fed to `cv::watershed`. |
| `watershed_markers_vis.png` | Watershed result; red = borders (door candidates). |
| `door_mask_raw.png` / `door_mask_filtered.png` | Borders before/after the 3×3 label filter. |
| `room_mask_new.png` | Binary any-room mask. |
| `room_segmentation_visualization.png` | Colored rooms + red door pixels. |
| `final_annotated.png` | **Start here.** Rooms with contours, id at the interior point (black dot), door centroids circled in blue. |

---

## Configuration

The pipeline's config is **`config/offline_scene_graph.yaml`** — a flat yaml with the
same key names as the online scenario yamls (so tuning transfers) and values copied
from `go2w_bag_direct.yaml` where it overrode a default. **Pass it explicitly to
`offline_cli`**: the compiled defaults below are the *online node's* defaults, and
several differ from the pipeline yaml's tuned values (`dilation_iteration` 4 vs 3,
`ceilingHeight_` 2.0 vs 2.3, `distance_threshold` 2.5 vs 2.0) — running config-less
gives different rooms. Only `offline_scene_graph_node` auto-defaults to the installed
copy of the yaml.

`--config` also accepts an online **scenario yaml** directly (keys under
`room_segmentation: ros__parameters:` and/or the `/**: ros__parameters:` wildcard).
Lookup priority per key: `room_segmentation:` scope → `/**:` scope → top level
(`navigation_graph/kNavNodeMinDist` uses the `tare_planner_node:` scope instead).

### Shared with the online node (same keys, same defaults, same meaning)

| Key | Default | Notes |
| --- | --- | --- |
| `room_resolution` | 0.1 m | Grid/pixel size. |
| `exploredAreaVoxelSize` | 0.1 m | Slab downsample leaf. |
| `ceilingHeight_` | 2.0 m | Slab/wall-band top, relative to `robot_z`. |
| `wall_thres_height_` | 0.1 m | Wall-band bottom, relative to `robot_z`. |
| `dilation_iteration` | 4 | The main "how aggressively to split rooms" knob. |
| `outward_distance_0` / `outward_distance_1` | 0.5 / 0.3 m | Wall-quad gap-closing / thickness. |
| `distance_threshold`, `distance_angel_threshold`, `angle_threshold_deg` | 2.5 m / 0.3 m / 6° | `isPlaneSame` merge tolerances. |
| `min_room_size` | 40 px | Minimum watershed seed area. |
| `normal_search_num` | 50 | kNN for normal estimation. |

Ignored offline (documented so nobody hunts for them): `room_x/y/z` (grid is
auto-sized from the cloud bounds), `region_growing_radius` (the whole floor is fed
to `getWall`), `exploredAreaDisplayInterval`, `kViewPointCollisionMarginZ*`,
`isDebug` (debug output is always on).

### Formerly hard-coded online, exposed here (defaults = the online constants)

| Key | Default | Meaning |
| --- | --- | --- |
| `rg_min_cluster_size` | 300 | RegionGrowing minimum cluster. |
| `rg_neighbors` | 50 | RegionGrowing neighbor count. |
| `rg_smoothness_deg` | 3.0 | RegionGrowing smoothness threshold. |
| `rg_curvature` | 1.0 | RegionGrowing curvature threshold. |
| `plane_min_height` | 1.5 m | Minimum vertical extent for a wall plane. |
| `min_hole_area` | 400 px | Holes smaller than this are filled in the boundary. |
| `min_component_area` | 100 px | Boundary components smaller than this are dropped. |
| `hist_threshold_factor` | 0.5 | Wall-histogram gate, `hist ≥ factor × max`, applied on the **float** histogram (online it was an implicit 0.5 after normalization — here it's a real knob). |

### Offline-only

| Key | Default | Meaning |
| --- | --- | --- |
| `slab_below` | 1.0 m | Slab bottom = `robot_z − slab_below`. |
| `grid_margin_px` | 20 | Margin around the auto-sized grid. |
| `door_merge_gap_m` | 0.4 m | Same-pair door fragments whose nearest pixels are closer than this are merged into one door (see `doors.json` above). The 3×3 junction blanking leaves gaps up to ~3–4 px, so don't go below ~0.35 m at 0.1 m resolution. |
| `wall_stage_leaf_size` | 0 (off) | If > `exploredAreaVoxelSize`, the normal/plane stage runs on a coarser copy of the cloud (perf escape hatch for huge maps; normals are re-estimated on the coarse copy). |
| `frame` | `"map"` | Frame label written into `mask_meta.json`. |

---

## When something breaks — first places to look

- **"slab z=[a, b] has only N points"** → wrong `robot_z` in the floors yaml, or the
  pcd is in a different frame/units than expected. Check the pcd's z histogram
  against `collision_range[0]`.
- **One real room cut into many** → wall mask too thick: lower `dilation_iteration`,
  or raise `hist_threshold_factor`. Compare `wall_from_hist.png` vs
  `wall_from_plane.png` to see which source is over-firing.
- **Two real rooms bleed into one** → walls missing. Check `wall_planes_color.png`
  (are the shared walls detected as planes at all?) and the wall band: with low
  partitions, `wall_thres_height_` may sit above them; with high ceilings,
  `ceilingHeight_` may dilute the histogram.
- **Sparse areas drop out entirely** (e.g. regions seen only through glass) →
  they fail `min_component_area` / `min_room_size`. Decide whether to lower those
  or crop the pcd; do not re-add freespace machinery for this.
- **No room seeds survived** → `dilation_iteration` ate everything (tiny floor?) or
  the slab caught almost no wall band. The stderr message prints both knobs.
- **A downstream consumer sees mirrored/rotated rooms** → it read a `debug/` image
  instead of `room_mask.png`, or ignored the `orientation_note` in `mask_meta.json`.

---

## offline_navgraph

Downsamples a keypose-graph dump into the scene graph's navigation layer. The input
`keypose_graph.json` is written by the ONLINE planner on the `skg` `/keyboard_input`
trigger (see the [exporter README](../scene_graph_exporter/README.md)); its `connected`
array is replayed verbatim (deduped, first-occurrence order — it contains duplicates by
contract and must never be recomputed from adjacency, which overstates traversability).

```bash
./install/tare_planner/lib/tare_planner/offline_cli navgraph \
    --graph <session>/floor_1/keypose_graph.json \
    --config config/offline_scene_graph.yaml \    # navigation_graph/kNavNodeMinDist
    --rooms <seg_out>/floor_1 \                   # OPTIONAL: debug overlay only
    --out <seg_out>/floor_1
```

One-shot port of the live `NavGraph::Reconcile` (`src/navgraph/navgraph.cpp`), minus
the incremental machinery: **seed** (greedy distance-gated coverage in flood order) →
**label** (geodesic Voronoi, multi-source BFS along collision-checked keypose edges —
wall-leak-proof) → **edges** (region adjacency, weight = min crossing
`‖node_u−a‖ + len(a,b) + ‖b−node_v‖`). Deterministic: same input → same output.

Output `navgraph.json`: `metadata` (frame/stamp/`nav_node_min_dist`/counts), `nodes`
(`{id, position, seed_keypose_ind}`, ids dense 0..N−1 in seed order), `edges`
(`[u,v,meters]`, canonical `u<v`). **No room ids** — by the naming contract, rooms are
the assembler's business. `--rooms` adds `debug/navgraph_overlay.png` (nodes/edges over
`room_mask_vis.png`, debug orientation; untagged nodes draw grey) and prints the
nodes-inside-a-room coverage — the frame-consistency check between the robot-side dump
and the pcd-side masks.

## offline_scene_graph

Assembles one floor's layers into a GADM-style `scene_graph.json` with the **same
schema as the online exporter's snapshots** (`SceneGraphExporter::Build`), so
downstream consumers work unchanged. Room labels are `"unknown"` until the offline
labeling stage exists; `objects` arrays are empty until an object layer exists.

```bash
./install/tare_planner/lib/tare_planner/offline_cli assemble \
    --rooms <seg_out>/floor_1 \                   # rooms.json + doors.json + mask
    --navgraph <seg_out>/floor_1/navgraph.json \
    --building AlphaZ \                           # optional metadata knobs; also
    --out <seg_out>/floor_1/scene_graph.json      #   --floor-level --floor-id --map-name
```

All cross-layer relationships happen here: nav nodes are tagged into rooms via the
mask (`RoomIdAt`, the cropped-mask port of the online `TagRooms`), named
`<room_key>-wp_<n>` (wp_0 = the room's interior point), and given 3×3 `area` tags from
per-room grids built on building axes fitted from all room polygons
(`cv::minAreaRect`, canonicalized exactly like `QuadrantManager::FitAxes` — the
compass in `layout.metadata` comes from the same axes). Doors become `entrances`,
navgraph edges become `layout.edges` between waypoint ids (edges touching an untagged
node are dropped and counted in the console summary). `--floor-level` defaults to the
trailing number of the floor name in `mask_meta.json`.

## offline_scene_graph_node (production)

The orchestrator: a ROS 2 node wrapping the ROS-free
`RunOfflinePipeline()` (`include/offline/offline_pipeline.h`) — room segmentation
(all floors, one pcd pass) → per floor navgraph → per floor scene-graph assembly,
in-process on a worker thread.

```bash
ros2 run tare_planner offline_scene_graph_node --ros-args \
    -p session_dir:=/path/to/training/20260722_060723 \
    -p building:=AlphaZ
# trigger:
ros2 topic pub -1 /scene_graph_generator/request std_msgs/msg/String "{data: generate}"
```

`config_yaml` defaults to the installed `offline_scene_graph.yaml` (resolved via the
package share directory); pass the parameter to override. The same run is available
without ROS as `offline_cli run --session <dir> [--config <yaml>] [--building NAME]
[--out <dir>] [--floor <name>]`.

The session folder must contain `scans.pcd`, `blueprint.yaml` and per-floor
`<floor>/keypose_graph.json` dumps. Signal semantics on `request_topic`
(param, default `/scene_graph_generator/request`): `trigger_keyword` (param, default
`"generate"`) runs on the `session_dir` parameter; a message whose payload is an
existing directory path runs on that folder instead. One run at a time (a second
trigger gets `{"status":"busy"}`).

Outputs land in `output_dir` (param, default `<session>/scene_graph/<floor>/` —
everything this README documents per floor, plus `navgraph.json` +
`scene_graph.json`). One JSON response per run on `response_topic` (default
`/scene_graph_generator/response`): `status` (`complete`/`error`/`busy`), per-floor
stats (`rooms`, `nav_nodes`, `nav_edges`, `nodes_in_rooms`), paths, and the full
scene graphs inline. A floor without a keypose dump is reported in the response
(`skipped_reason`) and keeps its rooms layer — dump the graph (`skg` on
`/keyboard_input`) and re-trigger.

## Frame consistency

Everything is in the input pcd's (FAST-LIO map) frame — keypose dumps label it
`odom`, the segmentation labels it `map`; same physical frame, and the assembler
prints a note rather than failing on the label mismatch. Keep every future artifact
(image poses, objects) in that same frame. The navgraph overlay's
nodes-inside-a-room count is the cheap end-to-end check that a session's dumps and
masks actually agree.
