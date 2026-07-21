# offline_room_segmentation

Batch, **non-ROS** CLI that runs the room segmentation pipeline on a complete map:
a full-building `.pcd` (e.g. a FAST-LIO map) plus a per-floor z index go in; per-floor
`room_mask.png` / `rooms.json` / `doors.json` and a full set of debug images come out.

This is stage 1 of the **offline scene-graph pipeline** (offline room segmentation →
offline navgraph → offline room labeling → assembler/export), the batch counterpart of
the online [`room_segmentation`](../room_segmentation/README.md) node. The segmentation
core — two-source wall extraction (region-grown vertical planes ∪ wall-band column
histogram) → dilate → `cv::watershed` → door detection → per-room polygon / centroid /
interior point — is **lifted** from the online node (copy-first, the online node is
untouched).

| Executable | Files |
| --- | --- |
| `offline_room_segmentation` | `src/offline/offline_room_segmentation.cpp`, `include/offline/offline_types.h` |

---

## Running

```bash
colcon build --packages-select tare_planner --cmake-args -DCMAKE_BUILD_TYPE=Release

./install/tare_planner/lib/tare_planner/offline_room_segmentation \
    --pcd scans.pcd \
    --floors blueprint.yaml \
    --config src/exploration_planner/tare_planner/config/go2w_bag_direct.yaml \
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

Runtime on the AlphaZ building map (8.6 M points, 2 floors): ~1 s per floor after the
one-time PCD load.

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

`--config` accepts either an existing **scenario yaml** (keys under
`room_segmentation: ros__parameters:` and/or the `/**: ros__parameters:` wildcard,
e.g. `config/go2w_bag_direct.yaml`) or a **flat yaml** with top-level keys. Lookup
priority per key: `room_segmentation:` scope → `/**:` scope → top level. So a
scenario file tunes the online node and this tool identically.

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

## Downstream consumers

The `room_mask.png` + `mask_meta.json` pair is the contract for the later offline
stages: the offline navgraph builder tags nav nodes with room ids by projecting node
positions through the `pixel_to_world` inverse, and the labeling/export stages take
room identity (`id`), geometry (`polygon`, `interior_point`) and topology
(`neighbors`, `doors.json`) from here. Everything is in the input pcd's frame — keep
every other artifact (keypose-graph dump, image poses) in that same frame.
