"""Pure geometry helpers for sempath_planner (unit-testable without ROS).

Pixel <-> world uses the exporter's ``metadata.grid_coordinate_frame``
(``tools/sempath_export``): index order [row, col], row axis = ROS y, col axis = ROS x,
``x_min`` / ``z_min`` are the world coordinates of the CENTRE of col 0 / row 0.
"""

from __future__ import annotations

import math


def world_to_pixel(x: float, y: float, frame: dict) -> tuple[int, int]:
    """World (x, y) in the map frame -> (row, col), plain Python ints.

    GroundPlan's start_pose check is ``isinstance(v, int)``, so numpy scalars must not leak in.
    """
    res = float(frame["resolution"])
    row = int(round((y - float(frame["z_min"])) / res))
    col = int(round((x - float(frame["x_min"])) / res))
    return row, col


def pixel_to_world(row: float, col: float, frame: dict) -> tuple[float, float]:
    """(row, col) -> world (x, y) at the cell centre."""
    res = float(frame["resolution"])
    x = float(frame["x_min"]) + float(col) * res
    y = float(frame["z_min"]) + float(row) * res
    return x, y


def trajectory_to_world(trajectory: list[list[int]], frame: dict) -> list[tuple[float, float]]:
    return [pixel_to_world(r, c, frame) for r, c in trajectory]


def semantic_map_cells(
    rgb,
    occupancy,
    frame: dict,
    skip_value: int = 2,
) -> list[tuple[float, float, tuple[int, int, int]]]:
    """World-centred colored cells of a SemPathBench map render, for an RViz overlay.

    ``rgb`` is the map PNG indexed ``[row][col]`` -> (r, g, b) (one pixel per grid cell, same
    row/col order as the layers); ``occupancy`` is the matching layer, and cells equal to
    ``skip_value`` (2 = unknown, i.e. the unexplored padding around the building) are dropped.
    """
    res = float(frame["resolution"])
    x_min = float(frame["x_min"])
    z_min = float(frame["z_min"])
    cells: list[tuple[float, float, tuple[int, int, int]]] = []
    for row, occ_row in enumerate(occupancy):
        y = z_min + row * res
        for col, occ in enumerate(occ_row):
            if occ == skip_value:
                continue
            r, g, b = rgb[row][col]
            cells.append((x_min + col * res, y, (int(r), int(g), int(b))))
    return cells


def _point_segment_distance(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    seg_sq = dx * dx + dy * dy
    if seg_sq == 0.0:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg_sq))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def rdp(points: list[tuple[float, float]], epsilon: float) -> list[tuple[float, float]]:
    """Ramer-Douglas-Peucker simplification (iterative; keeps first and last point)."""
    if len(points) < 3 or epsilon <= 0.0:
        return list(points)
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        lo, hi = stack.pop()
        max_dist, max_idx = 0.0, -1
        for i in range(lo + 1, hi):
            d = _point_segment_distance(points[i], points[lo], points[hi])
            if d > max_dist:
                max_dist, max_idx = d, i
        if max_dist > epsilon:
            keep[max_idx] = True
            stack.append((lo, max_idx))
            stack.append((max_idx, hi))
    return [p for p, k in zip(points, keep) if k]


def decimate_path(
    points: list[tuple[float, float]],
    epsilon_m: float,
    max_spacing_m: float,
) -> list[tuple[float, float]]:
    """RDP-simplify, then subdivide long segments so consecutive waypoints are <= max_spacing_m apart.

    Corners survive (RDP keeps them); straight runs become evenly spaced waypoints the follower can
    hand to the local planner one at a time. The first point (the robot start) is kept so callers
    can decide to skip it.
    """
    simplified = rdp(points, epsilon_m)
    if max_spacing_m <= 0.0 or len(simplified) < 2:
        return simplified
    out: list[tuple[float, float]] = [simplified[0]]
    for a, b in zip(simplified, simplified[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        pieces = max(1, math.ceil(dist / max_spacing_m))
        for k in range(1, pieces + 1):
            t = k / pieces
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out
