"""Slim, verbatim subset of SemPathBench's evaluation geometry helpers.

Vendored from /home/all/SemPathBench @ 06d570d:
  - scripts/evaluation/hyparameter.py      : TRAVERSABLE_OBJECT_CATEGORIES
  - scripts/methods/util/grid_astar.py     : TraversableGrid, free_occupancy_value, build_traversable_grid
  - scripts/evaluation/metric_geometry.py  : _map_free_occupancy_value, _object_categories_by_id,
                                             clearance_obstacle_cells, clearance_occupancy_shape,
                                             clearance_distance_field_from_obstacles
Only what tools/sempath_export/vendor/metric_cache.py imports. Function bodies are byte-identical
to the source (extracted with ast.get_source_segment); do not edit them here.
"""

from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np
from scipy.ndimage import distance_transform_edt

TRAVERSABLE_OBJECT_CATEGORIES = {"doorframe", "doorway"}


TraversableGrid = list[list[bool]]


def free_occupancy_value(map_state: Mapping[str, object]) -> int:
    legends = map_state.get("layer_legends")
    if isinstance(legends, Mapping):
        occupancy = legends.get("occupancy")
        if isinstance(occupancy, Mapping):
            free = occupancy.get("free")
            if isinstance(free, Mapping) and isinstance(free.get("value"), int):
                return int(free["value"])
    return 0


def build_traversable_grid(map_state: Mapping[str, object]) -> TraversableGrid:
    """Convert the map's occupancy layer into a boolean traversability grid."""
    grid_size = int(map_state["grid_size"])
    layers = map_state["layers"]
    if not isinstance(layers, Mapping):
        return [[False for _col in range(grid_size)] for _row in range(grid_size)]
    occupancy = layers["occupancy"]
    if not isinstance(occupancy, Sequence):
        return [[False for _col in range(grid_size)] for _row in range(grid_size)]
    free_value = free_occupancy_value(map_state)
    traversable: TraversableGrid = []
    for row in range(grid_size):
        traversable_row: list[bool] = []
        for col in range(grid_size):
            occupancy_value = int(occupancy[row][col])  # type: ignore[index]
            traversable_row.append(occupancy_value == free_value)
        traversable.append(traversable_row)
    return traversable


def _map_free_occupancy_value(map_state: Mapping[str, object]) -> int:
    legends = map_state.get("layer_legends")
    if isinstance(legends, Mapping):
        occupancy_legends = legends.get("occupancy")
        if isinstance(occupancy_legends, Mapping):
            free_tile = occupancy_legends.get("free")
            if isinstance(free_tile, Mapping):
                value = free_tile.get("value")
                if isinstance(value, int) and not isinstance(value, bool):
                    return value
    return 0


def _object_categories_by_id(map_state: Mapping[str, object]) -> dict[int, str]:
    object_instances = map_state.get("object_instances", [])
    if not isinstance(object_instances, Sequence) or isinstance(
        object_instances, (str, bytes)
    ):
        return {}
    categories: dict[int, str] = {}
    for instance in object_instances:
        if not isinstance(instance, Mapping):
            continue
        object_id = instance.get("id")
        category = instance.get("category")
        if isinstance(object_id, int) and not isinstance(object_id, bool) and isinstance(category, str):
            categories[object_id] = category
    return categories


def clearance_obstacle_cells(
    map_state: Mapping[str, object],
) -> list[tuple[int, int, int]]:
    """Return clearance obstacle source cells as (row, col, object_id)."""

    layers = map_state.get("layers")
    if not isinstance(layers, Mapping):
        return []
    occupancy = layers.get("occupancy")
    object_grid = layers.get("object_instance")
    if (
        not isinstance(occupancy, Sequence)
        or isinstance(occupancy, (str, bytes))
        or not isinstance(object_grid, Sequence)
        or isinstance(object_grid, (str, bytes))
    ):
        return []

    free_value = _map_free_occupancy_value(map_state)
    object_categories = _object_categories_by_id(map_state)
    obstacles: list[tuple[int, int, int]] = []
    for row_index, row in enumerate(occupancy):
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)):
            continue
        object_row = object_grid[row_index] if row_index < len(object_grid) else None
        for col_index, value in enumerate(row):
            try:
                occupancy_value = int(value)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                occupancy_value = free_value + 1
            object_id = 0
            if (
                isinstance(object_row, Sequence)
                and not isinstance(object_row, (str, bytes))
                and col_index < len(object_row)
            ):
                try:
                    object_id = int(object_row[col_index])  # type: ignore[index]
                except (TypeError, ValueError):
                    object_id = 0
            object_category = object_categories.get(object_id)
            object_is_obstacle = (
                object_id != 0 and object_category not in TRAVERSABLE_OBJECT_CATEGORIES
            )
            if occupancy_value != free_value or object_is_obstacle:
                obstacles.append((row_index, col_index, object_id))
    return obstacles


def clearance_occupancy_shape(map_state: Mapping[str, object]) -> tuple[int, int] | None:
    layers = map_state.get("layers")
    occupancy = layers.get("occupancy") if isinstance(layers, Mapping) else None
    if not isinstance(occupancy, Sequence) or isinstance(occupancy, (str, bytes)):
        return None
    row_count = len(occupancy)
    if row_count == 0:
        return None
    col_count = max(
        len(row)
        for row in occupancy
        if isinstance(row, Sequence) and not isinstance(row, (str, bytes))
    )
    return row_count, col_count


def clearance_distance_field_from_obstacles(
    map_state: Mapping[str, object],
    obstacles: Sequence[tuple[int, int, int]],
    excluded_ids: frozenset[int] = frozenset(),
) -> np.ndarray | None:
    shape = clearance_occupancy_shape(map_state)
    if shape is None:
        return None

    obstacle_mask = np.zeros(shape, dtype=bool)
    for row, col, object_id in obstacles:
        if object_id in excluded_ids:
            continue
        if 0 <= row < shape[0] and 0 <= col < shape[1]:
            obstacle_mask[row, col] = True

    if not obstacle_mask.any():
        return np.full(shape, np.inf, dtype=np.float64)
    return distance_transform_edt(~obstacle_mask)
