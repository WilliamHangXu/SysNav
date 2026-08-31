# Vendored SemPathBench writer modules

Source: `/home/all/SemPathBench` @ `06d570d039c8c669f73758feef96cbd9ec00564a` (see `MANIFEST.json`
for per-file SHA-256 of the *source* files and the exact import rewrites applied).

| File | Source | Local change |
|---|---|---|
| `convert_procthor_scene.py` | `scripts/make_maps/procthor/convert_procthor_scene.py` | none |
| `transform_procthor_to_map.py` | `scripts/make_maps/procthor/transform_procthor_to_map.py` | 3 import lines → `tools.sempath_export.vendor.*` |
| `view_procthor_map.py` | `scripts/make_maps/procthor/view_procthor_map.py` | none |
| `regenerate_procthor_maps.py` | `scripts/make_maps/procthor/regenerate_procthor_maps.py` | 2 import lines → `tools.sempath_export.vendor.*` |
| `metric_cache.py` | `scripts/evaluation/metric_cache.py` | 2 import lines → `tools.sempath_export.vendor.metric_geometry_slim` |
| `metric_geometry_slim.py` | verbatim function/constant copies from `scripts/evaluation/hyparameter.py` (`TRAVERSABLE_OBJECT_CATEGORIES`), `scripts/methods/util/grid_astar.py` (`TraversableGrid`, `free_occupancy_value`, `build_traversable_grid`), `scripts/evaluation/metric_geometry.py` (`_map_free_occupancy_value`, `_object_categories_by_id`, `clearance_obstacle_cells`, `clearance_occupancy_shape`, `clearance_distance_field_from_obstacles`) | composed file; every segment byte-identical to the source (`MANIFEST.json` → `segments_sha256`) |

## Policy

- These files are **never edited for behaviour**. SysNav-specific behaviour lives in
  `tools/sempath_export/{convert_sysnav_scene,layered_map,transform_sysnav_to_map,label_aliases}.py`.
- `test_convert_sysnav_scene.py::test_vendored_files_unmodified` re-derives the source hash from each vendored file
  (reverting the import rewrites listed in `MANIFEST.json`) and compares it with `source_sha256`; a behaviour edit fails
  the test. Update `MANIFEST.json` only when deliberately re-syncing.
- The ProcTHOR-only code paths (`prior`, `ai2thor`, `MAP_ROOT = resources/maps/procthor`, batch generation) are kept
  verbatim but unused; `prior`/`ai2thor` are imported lazily inside functions and never at module import.

## Re-sync

```bash
# from the SysNav root
python3 - <<'PY'
# re-run the vendoring script recorded in git history for tools/sempath_export/vendor (commit that added it),
# or: copy the 5 files, apply MANIFEST.json["rewrites"] to `from X import` lines, regenerate metric_geometry_slim.py
# with ast.get_source_segment for the listed names, then refresh MANIFEST.json hashes.
PY
diff -u /home/all/SemPathBench/scripts/make_maps/procthor/convert_procthor_scene.py tools/sempath_export/vendor/convert_procthor_scene.py
```
