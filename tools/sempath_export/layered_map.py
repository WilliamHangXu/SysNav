"""SysNav variant of SemPathBench's simple-demo state builder / saver.

SemPathBench's ``scene_representation_to_simple_demo_state`` re-rasterises room polygons, cannot emit the
``unknown`` occupancy value and hard-codes ProcTHOR metadata, so this module builds the layered state from
the converter's ``room_instance_map`` / ``unknown_map`` and then writes the *same file set* with the upstream
helpers (json / png / ppm / template_instruction.json / thinggraph / metric cache).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from tools.sempath_export import spb  # noqa: F401  (sys.path bootstrap for the embedded checkout)

from scripts.evaluation.metric_cache import build_map_metric_cache
from scripts.make_maps.procthor.transform_procthor_to_map import (
    SIMPLE_DEMO_OBJECT_COLORS,
    SIMPLE_DEMO_OCCUPANCY_TILES,
    SIMPLE_DEMO_ROOM_COLORS,
    _attribute,
    _export_simple_demo_png,
    _export_simple_demo_ppm,
    _generate_simple_demo_template_instructions,
    _json_ready,
    _legend_for_categories,
    _normalize_simple_category,
    _object_instances_from_metadata,
    _pad_to_square,
    _simple_demo_payload,
    build_thinggraph_payload,
    expected_simple_demo_output_paths,
)

SOURCE_LABELS = {
    "procthor": ("ProcTHOR", "ProcTHOR scene converted into simple_demo layered map format."),
    "sysnav": ("SysNav", "SysNav real-robot scene graph + BEV occupancy converted into simple_demo layered map format."),
}


def room_instances_from_metadata(room_metadata: list[dict]) -> list[dict]:
    """Same ids / names / attributes as SemPathBench's ``_room_instances_and_layer``, without rasterisation.

    Instance id == 1-based position in ``room_metadata`` (the converter's ``room_instance_map`` uses the same ids).
    The ``procthor_room_id`` attribute name is kept because ``build_thinggraph_payload`` reads it to link rooms
    back to ``room_metadata``; for SysNav it carries ``sysnav|room|<id>``.
    """
    instances: list[dict] = []
    category_counts: dict[str, int] = {}
    for record in room_metadata:
        raw_room_type = record.get("room_type")
        category = _normalize_simple_category(raw_room_type, fallback="room")
        category_counts[category] = category_counts.get(category, 0) + 1
        attributes = [
            item
            for item in (
                _attribute("procthor_room_id", record.get("room_id")),
                _attribute("procthor_room_type", raw_room_type),
                _attribute("area", record.get("area")),
                _attribute("grid_area", record.get("grid_area")),
                _attribute("sysnav_room_id", record.get("sysnav_room_id")),
                _attribute("sysnav_label", record.get("sysnav_label")),
            )
            if item is not None
        ]
        instances.append({
            "id": len(instances) + 1,
            "category": category,
            "name": f"{category}_{category_counts[category]}",
            "attributes": attributes,
        })
    return instances


def scene_representation_to_layered_state(scene_representation: dict, map_id: str) -> dict:
    traversibility = np.asarray(scene_representation["traversibility_map"])
    object_instance_map = np.asarray(scene_representation["object_instance_map"])
    room_layer = np.asarray(scene_representation["room_instance_map"]).astype(np.int32)
    tiles = SIMPLE_DEMO_OCCUPANCY_TILES
    occupancy = np.where(traversibility > 0, tiles["free"]["value"], tiles["obstacle"]["value"]).astype(np.int32)
    unknown = scene_representation.get("unknown_map")
    if unknown is not None:
        occupancy[(np.asarray(unknown) > 0) & (traversibility == 0)] = tiles["unknown"]["value"]

    room_instances = room_instances_from_metadata(scene_representation.get("room_metadata", []))
    if room_layer.size and int(room_layer.max()) > len(room_instances):
        raise ValueError("room_instance_map contains ids without a room_metadata record")
    if room_layer.shape != traversibility.shape:
        raise ValueError("room_instance_map shape != traversibility_map shape")
    object_instances = _object_instances_from_metadata(scene_representation)

    # square padding is outside the building: grey when the map distinguishes unknown, obstacle otherwise
    pad_value = tiles["unknown"]["value"] if unknown is not None else tiles["obstacle"]["value"]
    occupancy = _pad_to_square(occupancy, pad_value)
    room_layer = _pad_to_square(room_layer, 0)
    object_instance_map = _pad_to_square(object_instance_map, 0)
    grid_size = int(occupancy.shape[0])

    source = str(scene_representation.get("source") or "procthor")
    label, description = SOURCE_LABELS.get(source, (source, f"{source} scene converted into simple_demo layered map format."))
    room_categories = {str(i["category"]) for i in room_instances if isinstance(i.get("category"), str)}
    object_categories = {str(i["category"]) for i in object_instances if isinstance(i.get("category"), str)}
    metadata = {
        "map_id": map_id,
        "name": f"{label} {map_id}",
        "description": description,
        "source": source,
        "map_split": scene_representation.get("map_split"),
        "map_index": scene_representation.get("map_index"),
        "map_assignment_basis": scene_representation.get("map_assignment_basis"),
        "scene_size": scene_representation.get("scene_size"),
        "original_grid_shape": [int(traversibility.shape[0]), int(traversibility.shape[1])],
        "padded_grid_size": grid_size,
        "map_info": _json_ready(scene_representation.get("map_info", {})),
    }
    if scene_representation.get("source_metadata") is not None:
        metadata["source_metadata"] = _json_ready(scene_representation["source_metadata"])
    return {
        "metadata": metadata,
        "layers": {
            "occupancy": occupancy.astype(int).tolist(),
            "room": room_layer.astype(int).tolist(),
            "object_instance": object_instance_map.astype(int).tolist(),
        },
        "room_instances": room_instances,
        "object_instances": object_instances,
        "layer_legends": {
            "occupancy": tiles,
            "room_categories": _legend_for_categories(room_categories, SIMPLE_DEMO_ROOM_COLORS, saturation=0.32, value=0.95),
            "object_categories": _legend_for_categories(object_categories, SIMPLE_DEMO_OBJECT_COLORS, saturation=0.52, value=0.86),
        },
        "grid_size": grid_size,
    }


def save_layered_map(scene_representation: dict, output_prefix: str | Path, *, map_id: str) -> dict[str, Path]:
    """Mirror of SemPathBench's ``save_simple_demo_scene_representation`` using the SysNav state builder."""
    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    json_path, png_path, ppm_path, template_path, thinggraph_path = expected_simple_demo_output_paths(prefix)
    safe_map_id = _normalize_simple_category(map_id, fallback=prefix.name)
    state = scene_representation_to_layered_state(scene_representation, safe_map_id)
    json_path.write_text(json.dumps(_json_ready(_simple_demo_payload(state)), indent=2, ensure_ascii=False), encoding="utf-8")
    _export_simple_demo_ppm(state, ppm_path)
    _export_simple_demo_png(state, png_path)
    template_path.write_text(
        json.dumps(_json_ready(_generate_simple_demo_template_instructions(safe_map_id, state)), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    thinggraph_path.write_text(
        json.dumps(
            _json_ready(build_thinggraph_payload(
                scene_representation, state, safe_map_id,
                files={
                    "simple_map_json": json_path.name,
                    "metadata_json": f"{prefix.name}_metadata.json",
                    "maps_npz": f"{prefix.name}_maps.npz",
                    "template_instruction_json": template_path.name,
                },
            )),
            indent=2, ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    build_map_metric_cache(json_path, state)
    return {
        "json_path": json_path, "png_path": png_path, "ppm_path": ppm_path,
        "template_instruction_path": template_path, "thinggraph_path": thinggraph_path,
    }
