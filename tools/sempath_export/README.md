# tools/sempath_export — SysNav run → SemPathBench map

Turns one real-robot SysNav run into a **SemPathBench ProcTHOR-style layered map** without touching the
SemPathBench repo: the two SysNav nodes dump their state to `output/sempath_export/`, this tool converts the
dump with **vendored copies** of SemPathBench's own writer code (`vendor/`, pinned to SemPathBench commit
`06d570d`), and you copy the finished directory into SemPathBench by hand.

```
tare_planner_node ─► output/sempath_export/scene_graph_latest.json   (rooms+VLM labels, objects, doors, room-grid geometry)
                     output/sempath_export/room_mask_latest.png      (uint16 crop of the global room-id mask)
bev_mapper        ─► output/sempath_export/bev_latest.npz            (occupancy / explored / trajectory + origin)
                                   │
   python3 -m tools.sempath_export.transform_sysnav_to_map --dump-dir output/sempath_export --map-id 001_train
                                   │
                     output/sempath_maps/train/001_train/001_train.{json,png,ppm}  _maps.npz  _metadata.json
                                                          _thinggraph.json  _overview.png  template_instruction.json
                                                          001_train_metric_cache/{manifest.json,map_arrays.npz}
                                   │
   cp -a output/sempath_maps/train/001_train /home/all/SemPathBench/resources/maps/sysnav/train/   → map key sysnav/001_train
```

## 1. Producing a dump (SysNav side)

Both nodes write on three triggers: a periodic timer, a manual keyword on `/keyboard_input`, and once more at
shutdown (Ctrl-C of the teleop script). Writes are atomic (temp file + rename), so `*_latest.*` is always complete.

| Node | Params (`export.*`) | Files |
|---|---|---|
| `tare_planner_node` (`config/matterport_{real,bagfile,sim}.yaml`) | `enabled`, `output_dir` (`output/sempath_export`, cwd-relative → absolute), `interval_s` (30, 0=off), `keyword` (`export`), `keep_history`, `include_clouds`, `cloud_voxel_m` (0.05), `mask_crop_margin_cells` (20) | `scene_graph_latest.json`, `room_mask_latest.png`, `latest.json` |
| `bev_mapper` (`src/bev_mapper/config/bev_mapper.yaml`) | `enabled`, `output_dir`, `interval_s`, `keyword`, `keep_history` | `bev_latest.npz` |

```bash
./system_real_robot_teleop.sh bagfile:=true objects:=true rviz:=false        # or the real robot / sim bringup
ros2 bag play /path/to/bag --topics /lidar/scan /imu/data /camera/image      # no --clock
ros2 topic pub --once /keyboard_input std_msgs/msg/String "{data: export}"  # manual snapshot any time
# Ctrl-C → both nodes write a final snapshot (reason "final")
```

Room *types* come from `vlm_node` (needs a Gemini/Qwen key); without it every room is exported as `unknown_room`.

### Dump schemas

`scene_graph_latest.json` (`sysnav_scene_graph_dump/1`):
`stamp{ros_sec,wall_unix,wall_iso}`, `reason`, `frame:"map"`, `robot_position{x,y,z}`,
`room_grid{room_resolution, shift[3], dims{rows,cols}, layout:"row=x_index, col=y_index", index_formula}`,
`room_mask{file, dtype:"uint16", crop{row0,col0,rows,cols}, nonzero_cells, max_id}|null`,
`rooms[]{id, show_id, label, label_votes{}, is_labeled, is_connected, centroid[3], anchor_point[3], area_m2, polygon_xy[[x,y]…], neighbors[]}`,
`objects[]{id, ids[], label, confidence, status, room_id, img_path, timestamp, position[3], bbox3d[8×[x,y,z]], cloud_xyz[]|null, cloud_points_raw}`,
`doors[]{x,y,z, room_a, room_b, label}`, `room_adjacency[[a,b]…]`.

`bev_latest.npz` (`sysnav_bev_dump/1`): `occupancy`, `explored`, `trajectory` (uint8 `[row=+Y, col=+X]`),
`map_origin_x/y`, `resolution`, `map_size`, `global_cells`, `start_z`, `robot_pose[x,y,z,yaw]`, `stamp_unix`,
`stamp_ros_sec`, `frame`, `reason`, `scan_seq`, `layout`.

## 2. Converting

```bash
cd /home/all/AlphaZ/SysNav
python3 -m tools.sempath_export.transform_sysnav_to_map --dump-dir output/sempath_export --map-id 001_train
python3 -m tools.sempath_export.vendor.view_procthor_map --prefix output/sempath_maps/train/001_train/001_train --save
```

| Option | Default | Meaning |
|---|---|---|
| `--map-id` | required | must end in `_train` / `_valunseen` (SemPathBench's `--set` filtering keys off the suffix) |
| `--output-dir` | `output/sempath_maps` | writes `<out>/<split>/<id>/<id>.*` (leaf dir == JSON stem, as SemPathBench's discovery requires) |
| `--resolution` | 0.05 | must equal the BEV resolution (no resampling in v1) |
| `--padding` | 1.0 m | margin around the explored area |
| `--footprint cloud\|bbox` | cloud | object footprint from the projected voxel cloud (bbox-hull fallback when an object has no cloud), or from the XY hull of the 8 `bbox3d` corners |
| `--merge-gap` | 0.10 m | single-linkage merge of same-type footprints whose cells come within this gap (class-agnostic; collapses fragmented tracks of one object; 0 = off) |
| `--footprint-close` | 0.10 m | per-object cleanup radius: connected-component grouping on the k-dilated mask keeps the dominant blob (drops stray depth-bleed cells), then a morphological closing seals holes/seams (0 = off) |
| `--object-margin` | 0.15 m | inflate occupancy around obstacles and blocked objects, mimicking ProcTHOR's agent-radius reachability (its maps carry a ~3-cell solid obstacle contour around furniture); the contour is unconditional (the robot's own driven path is inflated over too, matching ProcTHOR's conservatism) and also consumes adjacent unknown cells so it stays complete against unexplored space; 0 = off |
| `--object-separation / --no-object-separation` | on | carve a 1-cell unlabeled (black) seam where two different instances touch (carved from the larger side, so small objects keep their footprint) |
| `--objects-block / --no-objects-block` | on | non-traversable object cells become obstacles (mimics ProcTHOR's reachable-position semantics) |
| `--unknown-as exterior\|obstacle\|unknown` | exterior | unexplored cells: `exterior` keeps occupancy `2` only outside the building (border-connected unknown; square padding is also `2`) and turns enclosed pockets — furniture interiors, sealed occlusion pockets — into obstacle `1`; `obstacle` = fully binary; `unknown` keeps every unexplored cell as `2` |
| `--absorb-pockets / --no-absorb-pockets` | on | geodesic competition (multi-source BFS) assigns leftover non-navigable cells to the nearest object: occlusion bays, enclosed unexplored patches, and *inaccessible floor* — explored-free cells unreachable from the robot's trajectory through a ≥0.15 m corridor once object cells count as walls (floor seen under chairs/beds through leg gaps); navigable space competes, so wall runs stay unlabeled; a final pass seals anything fully enclosed by one object's cells |
| `--contain-merge` | 0.8 | post-growth merge: a same-category object whose final footprint is ≥ this fraction contained in a bigger one is folded into it (fragments swallowed by fill/absorb/box; touching neighbours are nowhere near contained and survive; 0 = off) |
| `--footprint-box guarded\|full\|off` | guarded | finalize each footprint as its axis-aligned bounding rectangle (absorbs open bays no topological fill can claim); `guarded` keeps explored-free cells out of the box |
| `--footprint-fill / --no-footprint-fill` | on | claim regions enclosed by an object's shell + adjacent BEV-occupied cells for that object (fills a bed/sofa's occluded interior with its instance id), then fill any region enclosed by the object's own final cells (its body returns would otherwise leave the mask hollow); guarded: explored-free cells are never claimed, pockets bounded mostly by walls are left alone, and a region's claimant must own at least max(0.5 m, 5%) of its boundary |
| `--clear-trajectory-radius` | 0.4 m | cells the robot drove through are forced free (removes the BEV "person walking alongside" trail) |
| `--doors-as-objects` | on | door clusters become `Doorway` objects (traversable for the evaluator) |
| `--door-thickness` | 0.2 m | the watershed door line (spanning the opening) is padded along its thin axis to this depth, giving ProcTHOR-style filled door rectangles |
| `--room-fill-radius` | 0.5 m | free cells with no room id take the nearest room within this distance |
| `--min-room-cells` | 25 | drop mask ids with fewer export cells |
| `--drop-labels` | `person` | YOLOE labels to ignore |
| `--label-aliases` | – | yaml `{objects: {label: ObjectType}, rooms: {label: category}}` merged over `label_aliases.py` |
| `--skip-overview`, `--overwrite` | | |

Pipeline (`convert_sysnav_scene.py` → `layered_map.py` → vendored writers):
1. export grid = BEV frame cropped to explored ∪ room cells (+padding); `map_info.x_min/z_min` are the **centres** of
   column 0 / row 0, so export `[r,c]` ≡ BEV `[r0+r, c0+c]` exactly;
2. traversibility = explored ∧ ¬occupied (after trajectory clearing), unknown = ¬explored ∧ ¬occupied;
3. rooms: the 0.1 m mask is sampled at every 0.05 m cell centre (X-major formula), ids renumbered 1..N by ascending
   SysNav id, nearest-room fill, simplified contour polygon, VLM label → category via `label_aliases.py`;
4. objects: footprint cells (cloud projection by default, bbox hull as fallback/mode) → same-type single-linkage
   merge (`--merge-gap`) → dilated-CC stray-cell drop + closing (`--footprint-close`) → per-cell winner by
   ProcTHOR's `(priority, area, −height)` rule; metadata mirrors ProcTHOR (`objectId = sysnav|<id>`, merged
   instances record `merged_sysnav_ids`), doors → `Doorway`;
5. writes `_maps.npz` + `_metadata.json` (+ `sysnav` provenance), the layered bundle, the overview, then
   `migrate_map_payload` (`object_footprints`, `cell_object_ids`), `grid_coordinate_frame{row_axis: ros_y, col_axis: ros_x}`,
   validation, and the metric cache **last** (its validity is keyed on the JSON's size + mtime).

## 3. Hand-off to SemPathBench

```bash
mkdir -p /home/all/SemPathBench/resources/maps/sysnav/train
cp -a output/sempath_maps/train/001_train /home/all/SemPathBench/resources/maps/sysnav/train/   # -a keeps mtimes → cache stays valid
# SemPathBench: load_map_state("sysnav/001_train"); instructions go to resources/instructions/sysnav/001_train/instruction_files/
```
Read-only cross-check with SemPathBench's own validator (imports its module, writes nothing):
```bash
cd /home/all/SemPathBench && python -c "import json; from scripts.make_instruction.make_instruction import validate_map_payload; \
validate_map_payload(json.load(open('/home/all/AlphaZ/SysNav/output/sempath_maps/train/001_train/001_train.json')), '001_train'); print('ok')"
```

## 4. Tests

```bash
python3 -m unittest tools.sempath_export.test_convert_sysnav_scene -v
```
Synthetic 10×10 m dump (two rooms, three objects with scrambled box corners, one door, a trajectory with a
fake "person" streak); covers index round-trips, BEV slicing, renumbering + nearest fill, priority/aliases,
doorway traversability, occupancy values, the full bundle + validators + metric-cache freshness, and a hash
guard that the vendored files are unmodified (`vendor/MANIFEST.json`).

## Conventions & limitations

- Frames: everything is the ROS `map` frame; SemPathBench's "z" is ROS **y** (`row = y`, `col = x`, recorded in
  `metadata.grid_coordinate_frame`). ROS is right-handed, ProcTHOR left-handed — the map is mirrored relative
  to a ProcTHOR house viewed the same way; harmless for planners, keep it in mind for "left/right" in instructions.
- The room mask is anchored at world (0,0) (`shift = dims/2`, X-major) while the BEV is anchored at the start pose
  (Y-major); the converter never transposes arrays, it always goes through world coordinates.
- Real maps are partial and estimated (only objects the camera saw, YOLOE vocabulary, VLM room types); the benchmark
  treats the export as ground truth, so instructions must be authored against the exported map.
- One floor per export; `--resolution` must equal the BEV resolution; door/room ids > 255 would overflow the door
  cloud's uint8 channels (never happens in practice).
- `vendor/` is byte-identical to SemPathBench `06d570d` apart from import-path rewrites (`vendor/VENDORED.md`).
