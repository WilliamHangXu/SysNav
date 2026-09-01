"""Turn a SysNav scene-graph + BEV dump into a SemPathBench ``scene_representation`` dict.

Inputs (written by the SysNav nodes into ``output/sempath_export/``):
  * ``scene_graph_latest.json`` — rooms (+VLM label), objects (8-corner boxes, clouds), doors, room-grid geometry
  * ``room_mask_latest.png``    — uint16 crop of the planner's global room-id mask (X-major, see below)
  * ``bev_latest.npz``          — occupancy / explored / trajectory grids from ``bev_mapper`` (Y-major)

Grid conventions (all in the ROS ``map`` frame; "z" in SemPathBench names == ROS y):
  * room mask: ``ix = floor(x/room_res + shift[0])``, ``iy = floor(y/room_res + shift[1])``,
    ``id = png[ix - row0, iy - col0]``; cell centre ``x = (ix + 0.5 - shift[0]) * room_res``.
  * BEV: ``col = floor((x - origin_x)/res)``, ``row = floor((y - origin_y)/res)``.
  * export (SemPathBench): ``row = round((y - z_min)/res)``, ``col = round((x - x_min)/res)`` where ``x_min``/``z_min``
    are the *centres* of column 0 / row 0.  With crop offset ``(r0, c0)`` we set ``x_min = origin_x + (c0+0.5)*res`` so
    export ``[r, c]`` is exactly BEV ``[r0 + r, c0 + c]``.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage

from tools.sempath_export.label_aliases import (
    alias_object_label,
    alias_room_label,
    normalize_label,
    object_priority,
)
from tools.sempath_export.vendor.convert_procthor_scene import (
    _center_fallback_cells,
    _convex_hull,
    _rectangle_from_bounds,
    rasterize_polygon_to_grid,
    world_to_grid,
)
from tools.sempath_export.vendor.transform_procthor_to_map import _normalize_simple_category

SNAPSHOT_FILE = "scene_graph_latest.json"
BEV_FILE = "bev_latest.npz"
TRAVERSABLE_OBJECT_TYPES = {"doorway", "doorframe"}  # normalised categories the evaluator lets paths cross


@dataclass
class ConvertOptions:
    resolution: float = 0.05
    padding_m: float = 1.0
    footprint: str = "cloud"             # "cloud" (projected voxel cloud) | "bbox" (XY hull of bbox3d)
    objects_block: bool = True           # non-traversable object cells become obstacles (ProcTHOR semantics)
    unknown_as: str = "exterior"         # "exterior": grey only outside the building (border-connected unknown);
                                         #   enclosed unknown (furniture interiors, sealed pockets) becomes obstacle.
                                         # "obstacle": binary map | "unknown": keep all unknown as occupancy 2
    footprint_fill: bool = True          # claim enclosed non-free regions (e.g. a bed's unobserved interior)
    absorb_pockets: bool = True          # give leftover non-free pockets to the geodesically nearest object
    footprint_box: str = "guarded"       # finalize each footprint as its axis-aligned bounding rectangle:
                                         #   "guarded" = rectangle ∩ not-explored-free (observed floor stays out)
                                         #   "full" = the whole rectangle | "off"
    clear_trajectory_radius_m: float = 0.4
    doors_as_objects: bool = True
    room_fill_radius_m: float = 0.5
    min_room_cells: int = 25
    min_object_cells: int = 1
    merge_gap_m: float = 0.10            # single-linkage merge of same-type footprints within this gap (0 = off)
    footprint_close_m: float = 0.10      # morphological-closing radius sealing holes/seams in a footprint (0 = off)
    drop_labels: tuple[str, ...] = ("person",)
    object_aliases: dict[str, str] | None = None
    room_aliases: dict[str, str] | None = None

    def public_dict(self) -> dict[str, object]:
        return {
            "resolution": self.resolution, "padding_m": self.padding_m, "footprint": self.footprint,
            "objects_block": self.objects_block, "unknown_as": self.unknown_as,
            "footprint_fill": self.footprint_fill, "footprint_box": self.footprint_box,
            "absorb_pockets": self.absorb_pockets,
            "clear_trajectory_radius_m": self.clear_trajectory_radius_m, "doors_as_objects": self.doors_as_objects,
            "room_fill_radius_m": self.room_fill_radius_m, "min_room_cells": self.min_room_cells,
            "min_object_cells": self.min_object_cells, "merge_gap_m": self.merge_gap_m,
            "footprint_close_m": self.footprint_close_m, "drop_labels": list(self.drop_labels),
            "custom_aliases": bool(self.object_aliases or self.room_aliases),
        }


@dataclass
class SysNavDump:
    snapshot: dict
    room_mask: np.ndarray | None          # uint16 crop, X-major
    mask_crop: tuple[int, int]            # (row0, col0) of the crop inside the full mask
    bev: dict
    paths: dict[str, Path] = field(default_factory=dict)

    @property
    def room_grid(self) -> dict:
        return self.snapshot["room_grid"]


@dataclass
class RoomLayers:
    room_instance_map: np.ndarray
    room_type_map: np.ndarray
    room_metadata: list[dict]
    room_type_to_id: dict[str, int]
    id_map: dict[int, int]                # sysnav room id -> export instance id (1..N)


@dataclass
class ObjectLayers:
    object_category_map: np.ndarray
    object_instance_map: np.ndarray
    object_metadata: list[dict]
    category_to_id: dict[str, int]
    blocked: np.ndarray                   # bool, cells that objects_block turns into obstacles


# --------------------------------------------------------------------------- loading

def _npz_to_dict(path: Path) -> dict:
    out: dict = {}
    with np.load(path, allow_pickle=False) as archive:
        for key in archive.files:
            value = archive[key]
            out[key] = value.item() if value.ndim == 0 else value
    return out


def load_sysnav_dump(
    dump_dir: str | Path | None = None,
    *,
    snapshot: str | Path | None = None,
    room_mask: str | Path | None = None,
    bev: str | Path | None = None,
) -> SysNavDump:
    if dump_dir is None and (snapshot is None or bev is None):
        raise ValueError("give --dump-dir or both --snapshot and --bev")
    base = Path(dump_dir) if dump_dir is not None else None
    snapshot_path = Path(snapshot) if snapshot is not None else base / SNAPSHOT_FILE
    bev_path = Path(bev) if bev is not None else base / BEV_FILE
    snapshot_payload = json.loads(snapshot_path.read_text(encoding="utf-8"))
    if snapshot_payload.get("schema", "").split("/")[0] != "sysnav_scene_graph_dump":
        raise ValueError(f"{snapshot_path}: not a sysnav_scene_graph_dump")

    mask_info = snapshot_payload.get("room_mask")
    mask_array = None
    crop = (0, 0)
    if isinstance(mask_info, dict):
        mask_path = Path(room_mask) if room_mask is not None else snapshot_path.parent / mask_info["file"]
        mask_array = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        if mask_array is None:
            raise FileNotFoundError(mask_path)
        if mask_array.ndim != 2:
            raise ValueError(f"{mask_path}: expected a single-channel image, got shape {mask_array.shape}")
        mask_array = mask_array.astype(np.int32)
        crop_info = mask_info["crop"]
        crop = (int(crop_info["row0"]), int(crop_info["col0"]))
        expected = (int(crop_info["rows"]), int(crop_info["cols"]))
        if tuple(mask_array.shape) != expected:
            raise ValueError(f"{mask_path}: shape {mask_array.shape} != crop {expected} declared in the snapshot")
    bev_payload = _npz_to_dict(bev_path)
    if str(bev_payload.get("schema", "")).split("/")[0] != "sysnav_bev_dump":
        raise ValueError(f"{bev_path}: not a sysnav_bev_dump")
    return SysNavDump(
        snapshot=snapshot_payload, room_mask=mask_array, mask_crop=crop, bev=bev_payload,
        paths={"snapshot": snapshot_path, "bev": bev_path},
    )


# --------------------------------------------------------------------------- room-mask geometry

def room_mask_indices(xs: np.ndarray, ys: np.ndarray, room_grid: dict) -> tuple[np.ndarray, np.ndarray]:
    res = float(room_grid["room_resolution"])
    shift = room_grid["shift"]
    ix = np.floor(np.asarray(xs, dtype=np.float64) / res + float(shift[0])).astype(np.int64)
    iy = np.floor(np.asarray(ys, dtype=np.float64) / res + float(shift[1])).astype(np.int64)
    return ix, iy


def room_mask_cell_center(ix: np.ndarray, iy: np.ndarray, room_grid: dict) -> tuple[np.ndarray, np.ndarray]:
    res = float(room_grid["room_resolution"])
    shift = room_grid["shift"]
    return (ix + 0.5 - float(shift[0])) * res, (iy + 0.5 - float(shift[1])) * res


def sample_room_ids(dump: SysNavDump, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    xs = np.asarray(xs)
    out = np.zeros(xs.shape, dtype=np.int32)
    if dump.room_mask is None:
        return out
    ix, iy = room_mask_indices(xs, ys, dump.room_grid)
    r = ix - dump.mask_crop[0]
    c = iy - dump.mask_crop[1]
    rows, cols = dump.room_mask.shape
    valid = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
    out[valid] = dump.room_mask[r[valid], c[valid]]
    return out


def room_mask_world_centers(dump: SysNavDump) -> tuple[np.ndarray, np.ndarray]:
    """World xy of every non-zero room-mask cell centre."""
    if dump.room_mask is None:
        return np.empty(0), np.empty(0)
    r, c = np.nonzero(dump.room_mask)
    return room_mask_cell_center(r + dump.mask_crop[0], c + dump.mask_crop[1], dump.room_grid)


# --------------------------------------------------------------------------- export grid

def compute_export_grid(dump: SysNavDump, opts: ConvertOptions) -> tuple[dict, tuple[int, int]]:
    bev = dump.bev
    res = float(bev["resolution"])
    if abs(opts.resolution - res) > 1e-9:
        raise ValueError(f"--resolution {opts.resolution} != BEV resolution {res} (resampling is not supported)")
    occupancy = np.asarray(bev["occupancy"]) > 0
    explored = np.asarray(bev["explored"]) > 0
    n_rows, n_cols = occupancy.shape
    ox, oy = float(bev["map_origin_x"]), float(bev["map_origin_y"])

    rows, cols = np.nonzero(occupancy | explored)
    wx, wy = room_mask_world_centers(dump)
    if len(wx):
        rc = np.floor((wx - ox) / res).astype(np.int64)
        rr = np.floor((wy - oy) / res).astype(np.int64)
        inside = (rr >= 0) & (rr < n_rows) & (rc >= 0) & (rc < n_cols)
        rows = np.concatenate([rows, rr[inside]])
        cols = np.concatenate([cols, rc[inside]])
    if len(rows) == 0:
        raise ValueError("dump has no explored cells and no room cells inside the BEV grid")

    pad = int(math.ceil(opts.padding_m / res))
    r0 = max(0, int(rows.min()) - pad)
    r1 = min(n_rows - 1, int(rows.max()) + pad)
    c0 = max(0, int(cols.min()) - pad)
    c1 = min(n_cols - 1, int(cols.max()) + pad)
    height, width = r1 - r0 + 1, c1 - c0 + 1
    x_min = ox + (c0 + 0.5) * res
    z_min = oy + (r0 + 0.5) * res
    map_info = {
        "x_min": x_min, "x_max": x_min + (width - 1) * res,
        "z_min": z_min, "z_max": z_min + (height - 1) * res,
        "resolution": res, "H": height, "W": width,
    }
    return map_info, (r0, c0)


def export_cell_centers(map_info: dict) -> tuple[np.ndarray, np.ndarray]:
    res = float(map_info["resolution"])
    xs = float(map_info["x_min"]) + np.arange(int(map_info["W"])) * res
    zs = float(map_info["z_min"]) + np.arange(int(map_info["H"])) * res
    return np.meshgrid(xs, zs)  # X[r, c] = xs[c], Z[r, c] = zs[r]


def _disk(radius_cells: int) -> np.ndarray:
    size = 2 * radius_cells + 1
    yy, xx = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
    return ((yy * yy + xx * xx) <= radius_cells * radius_cells).astype(np.uint8).reshape(size, size)


# --------------------------------------------------------------------------- occupancy

def build_traversibility_layers(
    dump: SysNavDump, map_info: dict, rc0: tuple[int, int], opts: ConvertOptions
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray]:
    """Return (traversibility uint8 1=free, unknown bool|None, occupied bool) on the export grid."""
    r0, c0 = rc0
    height, width = int(map_info["H"]), int(map_info["W"])
    res = float(map_info["resolution"])
    sl = (slice(r0, r0 + height), slice(c0, c0 + width))
    occ = (np.asarray(dump.bev["occupancy"]) > 0)[sl].copy()
    exp = (np.asarray(dump.bev["explored"]) > 0)[sl].copy()
    traj = (np.asarray(dump.bev["trajectory"]) > 0)[sl]
    if opts.clear_trajectory_radius_m > 0 and traj.any():
        radius = int(math.ceil(opts.clear_trajectory_radius_m / res))
        trail = cv2.dilate(traj.astype(np.uint8), _disk(radius)) > 0
        occ[trail] = False
        exp[trail] = True
    traversibility = (exp & ~occ).astype(np.uint8)
    unknown = None
    if opts.unknown_as != "obstacle":
        unknown = ~exp & ~occ
        if opts.unknown_as == "exterior":
            unknown = _exterior_unknown(unknown)
    return traversibility, unknown, occ


# --------------------------------------------------------------------------- rooms

def room_contour_polygon_xz(cell_mask: np.ndarray, map_info: dict, eps_cells: float = 2.0) -> list[tuple[float, float]]:
    """Simplified outer contour of a room's cells, in world (x, z=y) coordinates."""
    res = float(map_info["resolution"])
    x_min, z_min = float(map_info["x_min"]), float(map_info["z_min"])
    rows, cols = np.nonzero(cell_mask)
    if len(rows) == 0:
        return []
    fallback = _rectangle_from_bounds(
        x_min + cols.min() * res, x_min + cols.max() * res, z_min + rows.min() * res, z_min + rows.max() * res
    )
    contours, _ = cv2.findContours(cell_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return fallback
    largest = max(contours, key=cv2.contourArea)
    approx = cv2.approxPolyDP(largest, eps_cells, True).reshape(-1, 2)
    if len(approx) < 3:
        return fallback
    return [(x_min + float(col) * res, z_min + float(row) * res) for col, row in approx]


def build_room_layers(
    dump: SysNavDump, map_info: dict, rc0: tuple[int, int], traversibility: np.ndarray, opts: ConvertOptions
) -> RoomLayers:
    del rc0
    height, width = int(map_info["H"]), int(map_info["W"])
    res = float(map_info["resolution"])
    xs, zs = export_cell_centers(map_info)
    ids = sample_room_ids(dump, xs, zs)

    snapshot_rooms = {int(room["id"]): room for room in dump.snapshot.get("rooms", [])}
    counts = np.bincount(ids.ravel())
    present = [int(i) for i in np.nonzero(counts)[0] if i > 0 and counts[i] >= opts.min_room_cells]
    if snapshot_rooms:
        present = [i for i in present if i in snapshot_rooms]
    ordered = sorted(present)
    id_map = {sysnav_id: index + 1 for index, sysnav_id in enumerate(ordered)}

    lut = np.zeros(int(ids.max()) + 1 if ids.size else 1, dtype=np.int32)
    for sysnav_id, new_id in id_map.items():
        lut[sysnav_id] = new_id
    room_instance_map = lut[ids]

    if opts.room_fill_radius_m > 0 and room_instance_map.any():
        empty = room_instance_map == 0
        distance, (nearest_r, nearest_c) = ndimage.distance_transform_edt(empty, return_indices=True)
        fill = empty & (traversibility > 0) & (distance <= opts.room_fill_radius_m / res)
        room_instance_map[fill] = room_instance_map[nearest_r[fill], nearest_c[fill]]

    room_types = {
        sysnav_id: alias_room_label(snapshot_rooms.get(sysnav_id, {}).get("label", ""), opts.room_aliases)
        for sysnav_id in ordered
    }
    room_type_to_id = {room_type: index for index, room_type in enumerate(sorted(set(room_types.values())))}
    room_type_map = np.full((height, width), -1, dtype=np.int32)
    room_metadata: list[dict] = []
    for sysnav_id in ordered:
        new_id = id_map[sysnav_id]
        mask = room_instance_map == new_id
        rows, cols = np.nonzero(mask)
        if len(rows) == 0:
            continue
        room_type = room_types[sysnav_id]
        room_type_map[mask] = room_type_to_id[room_type]
        polygon = room_contour_polygon_xz(mask, map_info)
        wx = float(map_info["x_min"]) + cols * res
        wz = float(map_info["z_min"]) + rows * res
        source = snapshot_rooms.get(sysnav_id, {})
        grid_area = float(len(rows)) * res * res
        room_metadata.append({
            "room_index": new_id - 1,
            "room_id": f"sysnav|room|{sysnav_id}",
            "room_type": room_type,
            "room_type_id": room_type_to_id[room_type],
            "floor_polygon_xz": [list(point) for point in polygon],
            "area": grid_area,
            "num_grid_cells": int(len(rows)),
            "grid_area": grid_area,
            "bbox": {
                "x_min": float(wx.min()), "x_max": float(wx.max()), "z_min": float(wz.min()), "z_max": float(wz.max()),
                "width_x": float(wx.max() - wx.min()), "depth_z": float(wz.max() - wz.min()),
            },
            "centroid_xz": [float(wx.mean()), float(wz.mean())],
            "sysnav_room_id": sysnav_id,
            "sysnav_show_id": source.get("show_id"),
            "sysnav_label": source.get("label", ""),
            "sysnav_label_votes": source.get("label_votes", {}),
            "sysnav_area_m2": source.get("area_m2"),
            "sysnav_neighbors": [id_map[n] for n in source.get("neighbors", []) if n in id_map],
            "is_connected": source.get("is_connected"),
        })
    return RoomLayers(room_instance_map, room_type_map, room_metadata, room_type_to_id, id_map)


# --------------------------------------------------------------------------- objects

def _bbox_polygon_xy(obj: dict) -> list[tuple[float, float]] | None:
    corners = obj.get("bbox3d") or []
    points = sorted({(round(float(p[0]), 5), round(float(p[1]), 5)) for p in corners if len(p) >= 2})
    if len(points) >= 3:
        hull = _convex_hull(points)
        if len(hull) >= 3:
            return hull
    if len(points) >= 2:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        if max(xs) > min(xs) and max(ys) > min(ys):
            return _rectangle_from_bounds(min(xs), max(xs), min(ys), max(ys))
    return None


def _in_bounds(row: int, col: int, map_info: dict) -> bool:
    return 0 <= row < int(map_info["H"]) and 0 <= col < int(map_info["W"])


def _object_cells(obj: dict, map_info: dict, opts: ConvertOptions) -> tuple[list[tuple[int, int]], str, list | None]:
    x_min, z_min, res = float(map_info["x_min"]), float(map_info["z_min"]), float(map_info["resolution"])
    polygon = _bbox_polygon_xy(obj)
    cells: list[tuple[int, int]] = []
    source = "sysnav_bbox3d"
    if opts.footprint == "cloud" and obj.get("cloud_xyz"):
        seen: set[tuple[int, int]] = set()
        for point in obj["cloud_xyz"]:
            row, col = world_to_grid(float(point[0]), float(point[1]), x_min, z_min, res)
            if _in_bounds(row, col, map_info):
                seen.add((row, col))
        cells = sorted(seen)
        source = "sysnav_cloud"
    if not cells and polygon:
        cells = rasterize_polygon_to_grid(polygon, map_info)
        source = "sysnav_bbox3d"
    if not cells:
        position = obj.get("position") or [0.0, 0.0, 0.0]
        cells = _center_fallback_cells({"x": float(position[0]), "z": float(position[1])}, map_info, radius_cells=1)
        source = "center_fallback"
    return cells, source, polygon


def _postprocess_cells(cells: list[tuple[int, int]], close_cells: int) -> list[tuple[int, int]]:
    """Drop stray cells, then seal holes/seams — both class-agnostic.

    1. Connected components on the mask dilated by ``close_cells`` (so cells within ``2*close_cells``
       of each other count as one group); keep the group holding the most *original* cells. This
       removes isolated depth-bleed cells without ever growing the footprint.
    2. Morphological closing (same radius) on the kept cells to fill interior holes and narrow seams.
       Closing cannot bridge sparse dots or end-to-end gaps of thin strips — that is what step 1's
       dilated grouping is for — and it is the identity on solid shapes.
    """
    if not cells:
        return cells
    rows = np.fromiter((c[0] for c in cells), dtype=np.int64, count=len(cells))
    cols = np.fromiter((c[1] for c in cells), dtype=np.int64, count=len(cells))
    margin = 2 * max(1, close_cells) + 2   # dilation must never touch the ROI border (cv2 border handling)
    r0, c0 = int(rows.min()) - margin, int(cols.min()) - margin
    mask = np.zeros((int(rows.max()) - r0 + margin + 1, int(cols.max()) - c0 + margin + 1), dtype=np.uint8)
    mask[rows - r0, cols - c0] = 1
    kernel = None
    if close_cells > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * close_cells + 1, 2 * close_cells + 1))
    grouped = cv2.dilate(mask, kernel) if kernel is not None else mask
    count, labels = cv2.connectedComponents(grouped, connectivity=8)
    if count > 2:
        cell_labels = labels[rows - r0, cols - c0]
        best = int(np.bincount(cell_labels).argmax())
        mask &= (labels == best).astype(np.uint8)
    if kernel is not None:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    out_rows, out_cols = np.nonzero(mask)
    return sorted((int(r) + r0, int(c) + c0) for r, c in zip(out_rows, out_cols))


def _exterior_unknown(unknown: np.ndarray) -> np.ndarray:
    """Keep only unknown regions connected to the export-grid border ("outside the building").

    Enclosed unknown pockets — a bed's unobserved interior, occlusion pockets sealed by walls — are
    dropped from the mask and therefore end up as obstacle in the occupancy layer.
    """
    count, labels = cv2.connectedComponents(unknown.astype(np.uint8), connectivity=4)
    if count <= 1:
        return unknown
    border = np.zeros(count, dtype=bool)
    border[labels[0, :]] = True
    border[labels[-1, :]] = True
    border[labels[:, 0]] = True
    border[labels[:, -1]] = True
    border[0] = False
    return border[labels]


def _fill_enclosed_regions(
    cell_sets: list[list[tuple[int, int]]], occupied: np.ndarray, free: np.ndarray, near_cells: int = 3
) -> tuple[list[list[tuple[int, int]]], list[int]]:
    """Jointly claim enclosed regions for the objects whose shells bound them.

    A lidar at chest height sees a bed/sofa as a ring in BEV occupancy while the camera-masked cloud
    shell may cover only one side; the top surface is occluded, so the interior stays unexplored.
    Structure = occupied ∪ every object's shell; each background region not touching the grid border
    is a candidate interior. Every boundary cell of the region is attributed to the object whose
    shell is nearest within ``near_cells`` (so an object's own BEV edge — typically 1-2 cells from
    its cloud shell — counts as that object, while distant walls stay neutral). The region goes to
    the object with the largest attributed share, and only if that share is at least
    ``max(10 cells, 5%)`` of the boundary — i.e. at least ~0.5 m of the ring must be this object's
    own edge; an observed furniture side is tens of cells, a small object on the rim is a handful — so a cup on a bed
    it can neither swallow the bed's interior nor claim it when the bed went undetected. Per cell,
    only NOT-explored-free cells are claimed (observed floor is never relabelled as furniture).
    Class-agnostic; no per-class priors.
    """
    added_counts = [0] * len(cell_sets)
    if not any(cell_sets):
        return cell_sets, added_counts
    shells = np.zeros(occupied.shape, dtype=np.int32)          # cell -> object index + 1
    for index, cells in enumerate(cell_sets):
        for row, col in cells:
            shells[row, col] = index + 1
    # nearest-shell attribution: distance + per-pixel label of the nearest shell cell
    dist, nearest = cv2.distanceTransformWithLabels(
        (shells == 0).astype(np.uint8), cv2.DIST_L2, 3, labelType=cv2.DIST_LABEL_PIXEL)
    label_to_object = np.zeros(int(nearest.max()) + 1, dtype=np.int32)
    shell_rows, shell_cols = np.nonzero(shells)
    label_to_object[nearest[shell_rows, shell_cols]] = shells[shell_rows, shell_cols]
    attribution = np.where(dist <= near_cells, label_to_object[nearest], 0)

    structure = (occupied | (shells > 0)).astype(np.uint8)
    count, labels = cv2.connectedComponents((structure == 0).astype(np.uint8), connectivity=4)
    if count <= 1:
        return cell_sets, added_counts
    border_labels = set(labels[0, :]) | set(labels[-1, :]) | set(labels[:, 0]) | set(labels[:, -1])
    kernel = np.ones((3, 3), dtype=np.uint8)
    out_sets = [set(cells) for cells in cell_sets]
    for label in range(1, count):
        if label in border_labels:
            continue
        component = (labels == label).astype(np.uint8)
        rows, cols = np.nonzero(component)
        r0, c0 = max(0, rows.min() - 1), max(0, cols.min() - 1)
        r1, c1 = min(component.shape[0], rows.max() + 2), min(component.shape[1], cols.max() + 2)
        roi = (slice(r0, r1), slice(c0, c1))
        comp_roi = component[roi].astype(bool)
        ring = cv2.dilate(component[roi], kernel).astype(bool) & ~comp_roi
        ring_owners = attribution[roi][ring]
        owners = np.bincount(ring_owners[ring_owners > 0], minlength=len(cell_sets) + 1)
        if not owners.any():
            continue                                            # bounded by walls only: leave it alone
        winner = int(owners.argmax())
        if owners[winner] < max(10, int(math.ceil(0.05 * int(ring.sum())))):
            continue                                            # touches an object but is not bounded by it
        claim = comp_roi & ~(free[roi] > 0)
        claim_rows, claim_cols = np.nonzero(claim)
        if len(claim_rows) == 0:
            continue
        out_sets[winner - 1].update((int(r) + r0, int(c) + c0) for r, c in zip(claim_rows, claim_cols))
        added_counts[winner - 1] += len(claim_rows)
    return [sorted(cells) for cells in out_sets], added_counts


def _fill_mask_holes(cells: list[tuple[int, int]], free: np.ndarray | None) -> tuple[list[tuple[int, int]], int]:
    """Fill regions fully enclosed by the object's OWN cells (its body returns show up as occupied
    cells inside the footprint and are structure, not background, for the joint fill — leaving the
    mask hollow). With a ``free`` mask, claims only NOT-explored-free cells; with ``free=None``,
    claims everything enclosed (a cell fully surrounded by one object is unreachable by definition)."""
    if not cells:
        return cells, 0
    rows = np.fromiter((c[0] for c in cells), dtype=np.int64, count=len(cells))
    cols = np.fromiter((c[1] for c in cells), dtype=np.int64, count=len(cells))
    r0, c0 = int(rows.min()) - 1, int(cols.min()) - 1
    mask = np.zeros((int(rows.max()) - r0 + 2, int(cols.max()) - c0 + 2), dtype=np.uint8)
    mask[rows - r0, cols - c0] = 1
    count, labels = cv2.connectedComponents((mask == 0).astype(np.uint8), connectivity=4)
    if count <= 2:
        return cells, 0
    border_labels = set(labels[0, :]) | set(labels[-1, :]) | set(labels[:, 0]) | set(labels[:, -1])
    added: list[tuple[int, int]] = []
    for label in range(1, count):
        if label in border_labels:
            continue
        hole_rows, hole_cols = np.nonzero(labels == label)
        for r, c in zip(hole_rows, hole_cols):
            gr, gc = int(r) + r0, int(c) + c0
            if free is None or (0 <= gr < free.shape[0] and 0 <= gc < free.shape[1] and not free[gr, gc]):
                added.append((gr, gc))
    if not added:
        return cells, 0
    return sorted(set(cells) | set(added)), len(added)


def _reachable_free(free_mask: np.ndarray, blocked: np.ndarray, seed: np.ndarray) -> np.ndarray:
    """Free cells reachable from ``seed`` through corridors at least 3 cells (0.15 m) wide,
    treating object cells as walls. Floor under a chair connects to the room only through leg
    gaps 1-2 cells wide, so it fails this test; a walkable alcove or an L-sofa's corner passes."""
    open_space = free_mask & ~blocked
    eroded = cv2.erode(open_space.astype(np.uint8), np.ones((3, 3), dtype=np.uint8)).astype(bool)
    count, labels = cv2.connectedComponents(eroded.astype(np.uint8), connectivity=8)
    if count <= 1:
        return open_space if seed.any() else open_space  # nothing wide enough: treat all as reachable
    keep = np.zeros(count, dtype=bool)
    seed_labels = labels[seed & eroded]
    keep[seed_labels] = True
    keep[0] = False
    core = keep[labels] & eroded
    reachable = cv2.dilate(core.astype(np.uint8), np.ones((3, 3), dtype=np.uint8)).astype(bool) & open_space
    return reachable


def _absorb_enclosed_pockets(
    cell_sets: list[list[tuple[int, int]]], occupied: np.ndarray, free: np.ndarray,
    trajectory: np.ndarray | None = None,
) -> tuple[list[list[tuple[int, int]]], list[int]]:
    """Assign leftover non-navigable cells to objects by geodesic competition (multi-source BFS).

    Domain = cells that are neither navigable, nor exterior unknown (border-connected), nor already
    part of an object: unlabeled walls, occlusion pockets, enclosed unexplored patches — and, when
    the robot trajectory is given, "inaccessible free" cells: explored floor that cannot be reached
    from the trajectory through a corridor at least 3 cells (0.15 m) wide once object cells are
    treated as walls (floor seen under a chair/bed through leg gaps). Sources: every
    navigable/exterior-adjacent domain cell (background) and every object-adjacent domain cell
    (that object); BFS through the domain, first arrival wins, background wins ties. A pocket
    behind/inside an object is geodesically closer to the object than to navigable space, so it is
    absorbed; a wall run is within a cell or two of navigable space almost everywhere, so the
    background reclaims it and object creep along walls stops on its own. No per-class priors.
    """
    from collections import deque

    height, width = free.shape
    free_mask = free > 0
    exterior = _exterior_unknown(~free_mask & ~occupied)
    inaccessible = np.zeros((height, width), dtype=bool)
    if trajectory is not None and trajectory.any():
        blocked = np.zeros((height, width), dtype=bool)
        for cells in cell_sets:
            for row, col in cells:
                blocked[row, col] = True
        seed = cv2.dilate(trajectory.astype(np.uint8), np.ones((3, 3), dtype=np.uint8)).astype(bool) & free_mask
        navigable = _reachable_free(free_mask, blocked, seed)
        inaccessible = free_mask & ~navigable                    # floor the robot cannot actually reach
        free_mask = navigable                                    # the rest joins the domain
    label = np.zeros((height, width), dtype=np.int32)          # 0 unvisited, -1 background, i+1 object
    for index, cells in enumerate(cell_sets):
        for row, col in cells:
            label[row, col] = index + 1
    domain = ~free_mask & ~exterior & (label == 0)          # free_mask here = navigable floor
    background = free_mask | exterior
    queue: deque = deque()
    rows, cols = np.nonzero(domain)
    object_seeds = []
    for row, col in zip(rows, cols):
        seed = 0
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row + dr, col + dc
            if not (0 <= r < height and 0 <= c < width) or background[r, c]:
                seed = -1                                       # grid border / free / exterior neighbour
                break
            if label[r, c] > 0 and seed == 0:
                seed = label[r, c]
        if seed == -1 and not inaccessible[row, col]:
            label[row, col] = -1
            queue.append((row, col))                            # background seeds first: they win ties
        elif seed > 0:
            object_seeds.append((row, col, seed))
    for row, col, seed in object_seeds:
        if label[row, col] == 0:
            label[row, col] = seed
            queue.append((row, col))
    while queue:
        row, col = queue.popleft()
        value = label[row, col]
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row + dr, col + dc
            if 0 <= r < height and 0 <= c < width and domain[r, c] and label[r, c] == 0:
                if value == -1 and inaccessible[r, c]:
                    continue                                    # navigable space never flows into sealed-off floor
                label[r, c] = value
                queue.append((r, c))
    out_sets = [set(cells) for cells in cell_sets]
    added_counts = [0] * len(cell_sets)
    for index in range(len(cell_sets)):
        claim_rows, claim_cols = np.nonzero(domain & (label == index + 1))
        if len(claim_rows):
            out_sets[index].update((int(r), int(c)) for r, c in zip(claim_rows, claim_cols))
            added_counts[index] = len(claim_rows)
    return [sorted(cells) for cells in out_sets], added_counts


def _box_cells(cells: list[tuple[int, int]], free: np.ndarray, mode: str) -> tuple[list[tuple[int, int]], int]:
    """Finalize a footprint as its axis-aligned bounding rectangle (class-agnostic shape prior).

    Open bays in a ragged mask (the object's body seen by the BEV but not the camera cloud, connected
    to the outside, so no topological fill can claim them) are absorbed by the rectangle. "guarded"
    keeps every explored-free cell out, so walkable floor — an L-sofa's real inner corner, a walkway
    clipped by the box of a diagonal object — is never turned into furniture.
    """
    if not cells or mode == "off":
        return cells, 0
    rows = np.fromiter((c[0] for c in cells), dtype=np.int64, count=len(cells))
    cols = np.fromiter((c[1] for c in cells), dtype=np.int64, count=len(cells))
    r0, r1 = int(rows.min()), int(rows.max())
    c0, c1 = int(cols.min()), int(cols.max())
    take = np.ones((r1 - r0 + 1, c1 - c0 + 1), dtype=bool)
    if mode == "guarded":
        take &= ~(free[r0:r1 + 1, c0:c1 + 1] > 0)
    take[rows - r0, cols - c0] = True
    out_rows, out_cols = np.nonzero(take)
    out = sorted((int(r) + r0, int(c) + c0) for r, c in zip(out_rows, out_cols))
    return out, len(out) - len(set(cells))


def _merge_same_type_records(records: list[dict], gap_m: float, resolution: float) -> list[dict]:
    """Single-linkage merge of same-objectType footprints whose cells come within ``gap_m`` of each other.

    Class-agnostic: one global gap. Distance is Chebyshev on the grid (a cell dilated by
    ``ceil(gap/res)`` cells), so the threshold is approximate on diagonals.
    """
    if gap_m <= 0 or len(records) < 2:
        return records
    reach = max(1, int(math.ceil(gap_m / resolution)))
    offsets = [(dr, dc) for dr in range(-reach, reach + 1) for dc in range(-reach, reach + 1)]
    parent = list(range(len(records)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    by_type: dict[str, list[int]] = {}
    for index, record in enumerate(records):
        by_type.setdefault(record["object_type"], []).append(index)
    for indices in by_type.values():
        if len(indices) < 2:
            continue
        cell_sets = {i: set(records[i]["cells"]) for i in indices}
        for pos, i in enumerate(indices):
            grown = {(r + dr, c + dc) for r, c in cell_sets[i] for dr, dc in offsets}
            for j in indices[pos + 1:]:
                if find(i) != find(j) and not grown.isdisjoint(cell_sets[j]):
                    parent[find(j)] = find(i)

    groups: dict[int, list[dict]] = {}
    for index, record in enumerate(records):
        groups.setdefault(find(index), []).append(record)
    merged: list[dict] = []
    for group in groups.values():
        if len(group) == 1:
            merged.append(group[0])
            continue
        group.sort(key=lambda rec: -len(rec["cells"]))     # primary evidence = largest fragment
        primary = group[0]
        cells = sorted({cell for rec in group for cell in rec["cells"]})
        weights = np.array([max(1, len(rec["cells"])) for rec in group], dtype=np.float64)
        positions = np.array([rec["position"] for rec in group], dtype=np.float64)
        mean_pos = (positions * weights[:, None]).sum(axis=0) / weights.sum()
        fragment_ids = sorted(int(rec["sysnav"]["id"]) for rec in group)
        record = dict(primary)
        record["cells"] = cells
        record["polygon"] = None                            # a merged footprint is no longer one rectangle
        record["position"] = [float(v) for v in mean_pos]
        record["object_id"] = f"sysnav|{fragment_ids[0]}"
        record["merged_ids"] = fragment_ids
        record["all_sysnav_ids"] = sorted({int(i) for rec in group for i in rec["sysnav"].get("ids", [rec["sysnav"]["id"]])})
        record["cloud_points_total"] = sum(len(rec["sysnav"].get("cloud_xyz") or []) for rec in group)
        record["confidence"] = max((rec["sysnav"].get("confidence") or 0.0) for rec in group)
        merged.append(record)
    merged.sort(key=lambda rec: int(str(rec["object_id"]).rsplit("|", 1)[-1]))
    return merged


def _door_groups(dump: SysNavDump) -> list[dict]:
    groups: dict[tuple, dict] = {}
    for door in dump.snapshot.get("doors", []):
        key = (int(door.get("room_a", 0)), int(door.get("room_b", 0)), int(door.get("label", 0)))
        entry = groups.setdefault(key, {"room_a": key[0], "room_b": key[1], "label": key[2], "points": []})
        entry["points"].append((float(door["x"]), float(door["y"]), float(door.get("z", 0.0))))
    return [groups[key] for key in sorted(groups)]


def build_object_layers(
    dump: SysNavDump, map_info: dict, rc0: tuple[int, int], rooms: RoomLayers, opts: ConvertOptions,
    traversibility: np.ndarray | None = None, occupied: np.ndarray | None = None,
    trajectory: np.ndarray | None = None,
) -> ObjectLayers:
    del rc0
    height, width = int(map_info["H"]), int(map_info["W"])
    res = float(map_info["resolution"])
    x_min, z_min = float(map_info["x_min"]), float(map_info["z_min"])
    drop = {normalize_label(label) for label in opts.drop_labels}

    records: list[dict] = []
    objects = [o for o in dump.snapshot.get("objects", []) if o.get("status", True) is not False]
    for obj in sorted(objects, key=lambda o: int(o["id"])):
        label = obj.get("label", "")
        if normalize_label(label) in drop:
            continue
        cells, source, polygon = _object_cells(obj, map_info, opts)
        if len(cells) < opts.min_object_cells:
            continue
        position = [float(v) for v in (obj.get("position") or [0.0, 0.0, 0.0])]
        records.append({
            "object_type": alias_object_label(label, opts.object_aliases),
            "raw_label": str(label),
            "cells": cells,
            "source": source,
            "polygon": polygon,
            "position": position,
            "object_id": f"sysnav|{int(obj['id'])}",
            "sysnav": obj,
        })
    records = _merge_same_type_records(records, opts.merge_gap_m, res)
    close_cells = int(round(opts.footprint_close_m / res))
    for record in records:
        record["cells"] = _postprocess_cells(record["cells"], close_cells)
    if opts.footprint_fill and traversibility is not None and occupied is not None and records:
        cell_sets, added = _fill_enclosed_regions([r["cells"] for r in records], occupied.astype(bool), traversibility)
        for record, cells, added_count in zip(records, cell_sets, added):
            cells, hole_count = _fill_mask_holes(cells, traversibility > 0)
            record["cells"] = cells
            record["filled_cells"] = added_count + hole_count
    if opts.absorb_pockets and traversibility is not None and occupied is not None and records:
        cell_sets, absorbed = _absorb_enclosed_pockets(
            [r["cells"] for r in records], occupied.astype(bool), traversibility, trajectory=trajectory)
        for record, cells, absorbed_count in zip(records, cell_sets, absorbed):
            record["cells"] = cells
            if absorbed_count > 0:
                record["absorbed_cells"] = absorbed_count
    if opts.footprint_box != "off" and traversibility is not None:
        for record in records:
            record["cells"], boxed = _box_cells(record["cells"], traversibility, opts.footprint_box)
            if boxed > 0:
                record["boxed_cells"] = boxed
                rows = [cell[0] for cell in record["cells"]]
                cols = [cell[1] for cell in record["cells"]]
                record["polygon"] = _rectangle_from_bounds(
                    x_min + min(cols) * res, x_min + max(cols) * res,
                    z_min + min(rows) * res, z_min + max(rows) * res)
    if opts.footprint_fill and traversibility is not None:
        for record in records:                                  # last pass: seal anything the later stages enclosed
            record["cells"], sealed = _fill_mask_holes(record["cells"], None)
            if sealed > 0:
                record["filled_cells"] = record.get("filled_cells", 0) + sealed
    if opts.doors_as_objects:
        for group in _door_groups(dump):
            seen: set[tuple[int, int]] = set()
            for x, y, _z in group["points"]:
                row, col = world_to_grid(x, y, x_min, z_min, res)
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if _in_bounds(row + dr, col + dc, map_info):
                            seen.add((row + dr, col + dc))
            if not seen:
                continue
            pts = np.asarray(group["points"], dtype=np.float64)
            records.append({
                "object_type": "Doorway",
                "raw_label": "door",
                "cells": sorted(seen),
                "source": "sysnav_door_cloud",
                "polygon": None,
                "position": [float(pts[:, 0].mean()), float(pts[:, 1].mean()), float(pts[:, 2].mean())],
                "object_id": f"sysnav|door|{group['room_a']}_{group['room_b']}_{group['label']}",
                "sysnav": {"door_rooms": [rooms.id_map.get(group["room_a"]), rooms.id_map.get(group["room_b"])],
                           "door_label": group["label"], "points": len(group["points"])},
            })

    categories = sorted({record["object_type"] for record in records})
    category_to_id = {category: index + 1 for index, category in enumerate(categories)}
    object_category_map = np.zeros((height, width), dtype=np.int32)
    object_instance_map = np.zeros((height, width), dtype=np.int32)
    priority_grid = np.full((height, width), -1, dtype=np.int32)
    area_grid = np.full((height, width), -1.0, dtype=np.float32)
    height_grid = np.full((height, width), np.inf, dtype=np.float32)
    blocked = np.zeros((height, width), dtype=bool)
    type_counts: dict[str, int] = {}
    object_metadata: list[dict] = []

    for instance_id, record in enumerate(records, start=1):
        object_type = record["object_type"]
        cells = record["cells"]
        priority = object_priority(object_type)
        footprint_area = len(cells) * res * res
        object_height = record["position"][2]
        type_counts[object_type] = type_counts.get(object_type, 0) + 1
        center_row, center_col = world_to_grid(record["position"][0], record["position"][1], x_min, z_min, res)
        if not _in_bounds(center_row, center_col, map_info):
            center_row, center_col = -1, -1
        entry: dict = {
            "instance_id": instance_id,
            "objectId": record["object_id"],
            "objectType": object_type,
            "name": f"{object_type}_{type_counts[object_type]}",
            "position": {"x": record["position"][0], "y": record["position"][2], "z": record["position"][1]},
            "category_id": category_to_id[object_type],
            "has_obb": record["polygon"] is not None,
            "has_aabb": record["polygon"] is not None,
            "bbox_polygon_xz": [list(p) for p in record["polygon"]] if record["polygon"] else None,
            "wall_segment_xz": None,
            "grid_row": center_row,
            "grid_col": center_col,
            "num_grid_cells": len(cells),
            "priority": priority,
            "footprint_area": footprint_area,
            "footprint_source": record["source"],
            "sysnav_label": record["raw_label"],
        }
        sysnav = record["sysnav"]
        if record["source"] == "sysnav_door_cloud":
            entry["door_rooms"] = sysnav["door_rooms"]
            entry["door_label"] = sysnav["door_label"]
        else:
            entry["sysnav_object_id"] = int(str(record["object_id"]).rsplit("|", 1)[-1])
            entry["sysnav_object_ids"] = record.get("all_sysnav_ids") or [int(i) for i in sysnav.get("ids", [sysnav["id"]])]
            entry["sysnav_room_id"] = rooms.id_map.get(int(sysnav.get("room_id", -1)))
            entry["confidence"] = record.get("confidence", sysnav.get("confidence"))
            entry["img_path"] = sysnav.get("img_path")
            entry["cloud_points"] = record.get("cloud_points_total", len(sysnav.get("cloud_xyz") or []))
            if record.get("merged_ids"):
                entry["merged_sysnav_ids"] = record["merged_ids"]
        if record.get("filled_cells"):
            entry["filled_cells"] = int(record["filled_cells"])
        if record.get("boxed_cells"):
            entry["boxed_cells"] = int(record["boxed_cells"])
        if record.get("absorbed_cells"):
            entry["absorbed_cells"] = int(record["absorbed_cells"])
        if len(cells) <= 128:
            entry["grid_cells"] = [list(cell) for cell in cells]
        object_metadata.append(entry)

        candidate = (priority, footprint_area, -object_height)
        for row, col in cells:
            existing = (int(priority_grid[row, col]), float(area_grid[row, col]), -float(height_grid[row, col]))
            if candidate > existing:
                object_category_map[row, col] = category_to_id[object_type]
                object_instance_map[row, col] = instance_id
                priority_grid[row, col] = priority
                area_grid[row, col] = footprint_area
                height_grid[row, col] = object_height
        if opts.objects_block and _normalize_simple_category(object_type) not in TRAVERSABLE_OBJECT_TYPES:
            rows = np.fromiter((c[0] for c in cells), dtype=np.int64, count=len(cells))
            cols = np.fromiter((c[1] for c in cells), dtype=np.int64, count=len(cells))
            blocked[rows, cols] = True

    return ObjectLayers(object_category_map, object_instance_map, object_metadata, category_to_id, blocked)


# --------------------------------------------------------------------------- assembly

def _plain(value):
    """numpy -> json-friendly python scalars/lists."""
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


def build_scene_representation(
    dump: SysNavDump, opts: ConvertOptions, *, map_id: str, map_split: str, map_index: int | None = None
) -> dict:
    map_info, rc0 = compute_export_grid(dump, opts)
    traversibility, unknown, occupied = build_traversibility_layers(dump, map_info, rc0, opts)
    rooms = build_room_layers(dump, map_info, rc0, traversibility, opts)
    r0, c0 = rc0
    trajectory = (np.asarray(dump.bev["trajectory"]) > 0)[r0:r0 + int(map_info["H"]), c0:c0 + int(map_info["W"])]
    objects = build_object_layers(dump, map_info, rc0, rooms, opts,
                                  traversibility=traversibility, occupied=occupied, trajectory=trajectory)
    if opts.objects_block:
        traversibility = traversibility.copy()
        traversibility[objects.blocked] = 0
        if unknown is not None:
            unknown = unknown & ~objects.blocked
    bev = dump.bev
    snapshot = dump.snapshot
    return {
        "traversibility_map": traversibility,
        "room_type_map": rooms.room_type_map,
        "object_category_map": objects.object_category_map,
        "object_instance_map": objects.object_instance_map,
        "object_metadata": objects.object_metadata,
        "room_metadata": rooms.room_metadata,
        "scene_size": float(sum(r["grid_area"] for r in rooms.room_metadata)),
        "map_info": map_info,
        "room_type_to_id": rooms.room_type_to_id,
        "category_to_id": objects.category_to_id,
        "map_id": map_id,
        "map_split": map_split,
        "map_index": map_index,
        "map_assignment_basis": "manual",
        # converter-only keys (consumed by layered_map.py, ignored by the vendored writers)
        "room_instance_map": rooms.room_instance_map,
        "unknown_map": unknown,
        "occupied_map": occupied,
        "source": "sysnav",
        "source_metadata": _plain({
            "schema": snapshot.get("schema"),
            "dump_stamp": snapshot.get("stamp"),
            "dump_reason": snapshot.get("reason"),
            "frame": snapshot.get("frame", "map"),
            "robot_position": snapshot.get("robot_position"),
            "room_grid": snapshot.get("room_grid"),
            "room_mask_crop": snapshot.get("room_mask", {}).get("crop") if snapshot.get("room_mask") else None,
            "bev": {k: bev.get(k) for k in ("schema", "stamp_unix", "stamp_ros_sec", "reason", "map_origin_x",
                                            "map_origin_y", "resolution", "global_cells", "start_z", "scan_seq")},
            "bev_robot_pose": bev.get("robot_pose"),
            "crop_offset": list(rc0),
            "room_id_map": {str(k): v for k, v in rooms.id_map.items()},
            "options": opts.public_dict(),
            "counts": {"rooms": len(rooms.room_metadata), "objects": len(objects.object_metadata),
                       "snapshot_objects": len(snapshot.get("objects", [])), "snapshot_doors": len(snapshot.get("doors", []))},
        }),
    }
