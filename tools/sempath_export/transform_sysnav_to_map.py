#!/usr/bin/env python3
"""Convert a SysNav dump (``output/sempath_export/``) into a SemPathBench map bundle.

    python -m tools.sempath_export.transform_sysnav_to_map --dump-dir output/sempath_export --map-id 001_train

Writes ``<output-dir>/<split>/<map_id>/<map_id>.{json,png,ppm}``, ``_maps.npz``, ``_metadata.json``,
``_thinggraph.json``, ``_overview.png``, ``template_instruction.json`` and ``<map_id>_metric_cache/``.
Copy the finished directory into SemPathBench with ``cp -a`` (the metric cache is keyed on the JSON's mtime).
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

from tools.sempath_export.convert_sysnav_scene import (
    ConvertOptions,
    SysNavDump,
    build_scene_representation,
    load_sysnav_dump,
)
from tools.sempath_export.label_aliases import load_label_aliases
from tools.sempath_export.layered_map import save_layered_map
from tools.sempath_export.vendor.convert_procthor_scene import save_scene_representation
from tools.sempath_export.vendor.metric_cache import build_map_metric_cache
from tools.sempath_export.vendor.regenerate_procthor_maps import (
    migrate_map_payload,
    validate_map_shape,
    validate_rich_object_schema,
    write_json_atomic,
)
from tools.sempath_export.vendor.transform_procthor_to_map import expected_simple_demo_output_paths, save_map_overview

MAP_SPLITS = ("train", "valunseen")
DEFAULT_OUTPUT_DIR = Path("output") / "sempath_maps"
SEMPATHBENCH_MAPS_HINT = "/home/all/SemPathBench/resources/maps/sysnav"


def map_split_from_id(map_id: str) -> str:
    split = map_id.rsplit("_", 1)[-1] if "_" in map_id else ""
    if split not in MAP_SPLITS:
        raise ValueError(f"map id must end in _train or _valunseen (SemPathBench --set filtering): {map_id!r}")
    return split


def build_output_prefix(output_dir: str | Path, map_id: str) -> Path:
    split = map_split_from_id(map_id)
    return Path(output_dir) / split / map_id / map_id


def validate_instance_ids(payload: dict) -> list[str]:
    """Ids used in layers.room / layers.object_instance must exist; occupancy values must be 0/1/2."""
    errors: list[str] = []
    layers = payload.get("layers", {})
    room_ids = {0} | {int(i["id"]) for i in payload.get("room_instances", [])}
    object_ids = {0} | {int(i["id"]) for i in payload.get("object_instances", [])}
    used_rooms = {int(v) for row in layers.get("room", []) for v in row}
    used_objects = {int(v) for row in layers.get("object_instance", []) for v in row}
    occupancy_values = {int(v) for row in layers.get("occupancy", []) for v in row}
    if used_rooms - room_ids:
        errors.append(f"layers.room uses unknown room ids {sorted(used_rooms - room_ids)}")
    if used_objects - object_ids:
        errors.append(f"layers.object_instance uses unknown object ids {sorted(used_objects - object_ids)}")
    if occupancy_values - {0, 1, 2}:
        errors.append(f"layers.occupancy has values outside 0/1/2: {sorted(occupancy_values - {0, 1, 2})}")
    return errors


def finalize_map_json(json_path: Path, metadata_path: Path, thinggraph_path: Path, *, map_info: dict) -> dict:
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    thinggraph = json.loads(thinggraph_path.read_text(encoding="utf-8"))
    migrated = migrate_map_payload(payload, metadata_payload=metadata, thinggraph_payload=thinggraph)
    migrated.setdefault("metadata", {})["grid_coordinate_frame"] = {
        "index_order": "[row, col]",
        "row_axis": "ros_y",
        "col_axis": "ros_x",
        "world_frame": "map",
        "resolution": float(map_info["resolution"]),
        "x_min": float(map_info["x_min"]),
        "z_min": float(map_info["z_min"]),
        "note": "x_min/z_min are the centres of column 0 / row 0; world y of SysNav is SemPathBench 'z'",
    }
    errors = validate_map_shape(migrated) + validate_rich_object_schema(migrated) + validate_instance_ids(migrated)
    if errors:
        raise ValueError("map validation failed:\n" + "\n".join(errors[:50]))
    write_json_atomic(json_path, migrated)
    build_map_metric_cache(json_path, migrated, overwrite=True)
    return migrated


def transform_sysnav_to_map(
    dump: SysNavDump,
    output_prefix: str | Path,
    map_id: str,
    opts: ConvertOptions,
    *,
    skip_overview: bool = False,
    overwrite: bool = False,
    map_index: int | None = None,
) -> dict[str, Path]:
    # Absolute: the vendored metric-cache builder resolves relative paths against ITS package root.
    prefix = Path(output_prefix).resolve()
    json_path = expected_simple_demo_output_paths(prefix)[0]
    if json_path.exists() and not overwrite:
        raise FileExistsError(f"{json_path} exists; pass --overwrite to replace the map")
    if prefix.parent.exists() and overwrite:
        shutil.rmtree(prefix.parent)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    rep = build_scene_representation(dump, opts, map_id=map_id, map_split=map_split_from_id(map_id), map_index=map_index)

    # 1. raw layers + metadata (vendored), then append provenance
    maps_path, metadata_path = save_scene_representation(rep, prefix)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["source"] = "sysnav"
    metadata["sysnav"] = rep["source_metadata"]
    write_json_atomic(metadata_path, metadata)

    # 2. layered map bundle (json / png / ppm / template / thinggraph / first metric cache)
    outputs = save_layered_map(rep, prefix, map_id=map_id)

    # 3. overview figure (matplotlib, optional)
    overview_path = None
    if not skip_overview:
        overview_path = save_map_overview(rep, prefix.with_name(f"{prefix.name}_overview.png"))

    # 4. migrate (object_footprints / cell_object_ids) -> validate -> rewrite -> rebuild metric cache LAST
    finalize_map_json(outputs["json_path"], metadata_path, outputs["thinggraph_path"], map_info=rep["map_info"])

    result = {"maps_path": maps_path, "metadata_path": metadata_path, **outputs}
    if overview_path is not None:
        result["overview_path"] = Path(overview_path)
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dump-dir", type=Path, default=None, help="directory with scene_graph_latest.json / room_mask_latest.png / bev_latest.npz")
    parser.add_argument("--snapshot", type=Path, default=None)
    parser.add_argument("--room-mask", type=Path, default=None)
    parser.add_argument("--bev", type=Path, default=None)
    parser.add_argument("--map-id", required=True, help="e.g. 001_train or 003_valunseen")
    parser.add_argument("--map-index", type=int, default=None)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--footprint", choices=("cloud", "bbox"), default="cloud")
    parser.add_argument("--merge-gap", type=float, default=0.10,
                        help="single-linkage merge of same-type footprints within this gap in metres (0 = off)")
    parser.add_argument("--footprint-close", type=float, default=0.10,
                        help="morphological-closing radius in metres for object footprints (0 = off)")
    parser.add_argument("--objects-block", dest="objects_block", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--unknown-as", choices=("exterior", "obstacle", "unknown"), default="exterior",
                        help="unexplored cells: 'exterior' keeps occupancy 2 only outside the building (border-connected) "
                             "and turns enclosed pockets (furniture interiors) into obstacle; 'obstacle' = fully binary; "
                             "'unknown' keeps every unexplored cell as 2")
    parser.add_argument("--absorb-pockets", dest="absorb_pockets", action=argparse.BooleanOptionalAction, default=True,
                        help="assign leftover non-navigable cells (occlusion bays, enclosed unexplored patches, and "
                             "observed floor unreachable through a 0.15 m corridor from the robot's trajectory, e.g. under "
                             "chairs/beds) to the geodesically nearest object; navigable space competes, so walls stay unlabeled")
    parser.add_argument("--footprint-box", choices=("guarded", "full", "off"), default="guarded",
                        help="finalize each object footprint as its axis-aligned bounding rectangle; 'guarded' keeps "
                             "explored-free cells out of the box, 'full' paints the whole rectangle")
    parser.add_argument("--footprint-fill", dest="footprint_fill", action=argparse.BooleanOptionalAction, default=True,
                        help="claim regions enclosed by an object's shell + occupied cells (fills bed/sofa interiors)")
    parser.add_argument("--clear-trajectory-radius", type=float, default=0.4)
    parser.add_argument("--doors-as-objects", dest="doors_as_objects", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--room-fill-radius", type=float, default=0.5)
    parser.add_argument("--min-room-cells", type=int, default=25)
    parser.add_argument("--drop-labels", default="person", help="comma-separated YOLOE labels to ignore")
    parser.add_argument("--label-aliases", type=Path, default=None, help="yaml {objects: {...}, rooms: {...}} merged over the defaults")
    parser.add_argument("--skip-overview", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def options_from_args(args: argparse.Namespace) -> ConvertOptions:
    object_aliases, room_aliases = load_label_aliases(args.label_aliases)
    return ConvertOptions(
        resolution=args.resolution,
        padding_m=args.padding,
        footprint=args.footprint,
        objects_block=args.objects_block,
        unknown_as=args.unknown_as,
        footprint_fill=args.footprint_fill,
        footprint_box=args.footprint_box,
        absorb_pockets=args.absorb_pockets,
        clear_trajectory_radius_m=args.clear_trajectory_radius,
        doors_as_objects=args.doors_as_objects,
        room_fill_radius_m=args.room_fill_radius,
        min_room_cells=args.min_room_cells,
        merge_gap_m=args.merge_gap,
        footprint_close_m=args.footprint_close,
        drop_labels=tuple(label.strip() for label in args.drop_labels.split(",") if label.strip()),
        object_aliases=object_aliases,
        room_aliases=room_aliases,
    )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    dump = load_sysnav_dump(args.dump_dir, snapshot=args.snapshot, room_mask=args.room_mask, bev=args.bev)
    prefix = build_output_prefix(args.output_dir, args.map_id)
    outputs = transform_sysnav_to_map(
        dump, prefix, args.map_id, options_from_args(args),
        skip_overview=args.skip_overview, overwrite=args.overwrite, map_index=args.map_index,
    )
    payload = json.loads(outputs["json_path"].read_text(encoding="utf-8"))
    split = map_split_from_id(args.map_id)
    print(f"map {args.map_id}: grid {payload['grid_size']}x{payload['grid_size']} @ {payload['metadata']['map_info']['resolution']} m, "
          f"{len(payload['room_instances'])} rooms, {len(payload['object_instances'])} objects -> {prefix.parent}")
    for key, path in outputs.items():
        print(f"  {key:26s} {path}")
    print("hand-off (preserve mtimes so the metric cache stays valid):")
    print(f"  mkdir -p {SEMPATHBENCH_MAPS_HINT}/{split} && cp -a {prefix.parent} {SEMPATHBENCH_MAPS_HINT}/{split}/")
    print(f"  -> SemPathBench map key: sysnav/{args.map_id}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
