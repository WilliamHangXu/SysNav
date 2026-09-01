"""Unit tests for the SysNav -> SemPathBench map converter (run from the SysNav root):

    python3 -m unittest tools.sempath_export.test_convert_sysnav_scene -v
"""

from __future__ import annotations

import hashlib
import json
import os
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from tools.sempath_export import convert_sysnav_scene as cs
from tools.sempath_export.layered_map import scene_representation_to_layered_state
from tools.sempath_export.transform_sysnav_to_map import (
    build_output_prefix,
    transform_sysnav_to_map,
    validate_instance_ids,
)
from tools.sempath_export.vendor.regenerate_procthor_maps import validate_map_shape, validate_rich_object_schema
from tools.sempath_export.vendor.transform_procthor_to_map import expected_simple_demo_output_paths

VENDOR_DIR = Path(__file__).resolve().parent / "vendor"

# ---- synthetic world: 10 m x 10 m BEV @ 0.05 m, origin (-5, -5); rooms on a 0.1 m mask anchored at (0, 0)
BEV_RES, BEV_N, BEV_OX, BEV_OY = 0.05, 200, -5.0, -5.0
ROOM_RES, ROOM_DIM = 0.1, 200            # shift = 100 -> world (0,0) at index 100; covers [-10, 10) m
FREE_X = (-4.5, 3.0)                     # explored/free interior
FREE_Y = (-3.0, 3.0)
ROOM_A = (3, "Office Room", (-2.9, 0.0))   # id, label, x-range
ROOM_B = (7, "", (0.0, 3.0))


def bev_index(x: float, y: float) -> tuple[int, int]:
    return int(math.floor((y - BEV_OY) / BEV_RES)), int(math.floor((x - BEV_OX) / BEV_RES))


def box_corners(cx: float, cy: float, sx: float, sy: float, z0: float = 0.0, z1: float = 0.8, yaw: float = 0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for dx in (-sx / 2, sx / 2):
        for dy in (-sy / 2, sy / 2):
            for z in (z0, z1):
                pts.append([cx + c * dx - s * dy, cy + s * dx + c * dy, z])
    rng = np.random.default_rng(7)
    rng.shuffle(pts)                      # corner order is unspecified in the real dump
    return pts


def make_synthetic_dump(tmp: Path) -> Path:
    occ = np.zeros((BEV_N, BEV_N), np.uint8)
    exp = np.zeros((BEV_N, BEV_N), np.uint8)
    traj = np.zeros((BEV_N, BEV_N), np.uint8)
    r_lo, c_lo = bev_index(FREE_X[0], FREE_Y[0])
    r_hi, c_hi = bev_index(FREE_X[1], FREE_Y[1])
    exp[r_lo - 1:r_hi + 2, c_lo - 1:c_hi + 2] = 1
    # walls around the free area + a partition at x=0 with a door gap y in [-0.3, 0.3]
    occ[r_lo - 1, c_lo - 1:c_hi + 2] = 1
    occ[r_hi + 1, c_lo - 1:c_hi + 2] = 1
    occ[r_lo - 1:r_hi + 2, c_lo - 1] = 1
    occ[r_lo - 1:r_hi + 2, c_hi + 1] = 1
    # bed at (1.8, 1.8), 1.0 x 0.8: BEV shows an occupied ring with an unexplored (occluded) interior
    b_r0, b_c0 = bev_index(1.3, 1.4)
    b_r1, b_c1 = bev_index(2.3, 2.2)
    occ[b_r0:b_r1 + 1, b_c0] = 1
    occ[b_r0:b_r1 + 1, b_c1] = 1
    occ[b_r0, b_c0:b_c1 + 1] = 1
    occ[b_r1, b_c0:b_c1 + 1] = 1
    exp[b_r0 + 1:b_r1, b_c0 + 1:b_c1] = 0
    wall_col = bev_index(0.0, 0.0)[1]
    occ[r_lo:r_hi + 1, wall_col] = 1
    gap_lo, gap_hi = bev_index(0.0, -0.3)[0], bev_index(0.0, 0.3)[0]
    occ[gap_lo:gap_hi + 1, wall_col] = 0
    # robot trail along y = 1.5 through the gap? no: along y = 0 through the door, with an occupied streak (artefact)
    trail_row = bev_index(0.0, 0.0)[0]
    traj[trail_row, bev_index(-2.5, 0)[1]:bev_index(2.5, 0)[1]] = 1
    occ[trail_row, bev_index(-2.0, 0)[1]:bev_index(-1.5, 0)[1]] = 1   # "person walking with the robot"

    # room mask (X-major: [ix, iy]); rooms only cover x >= -2.9 so cells at x in [-4.5,-2.9) are unlabeled
    mask = np.zeros((ROOM_DIM, ROOM_DIM), np.uint16)
    shift = ROOM_DIM // 2
    for room_id, _label, (x0, x1) in (ROOM_A, ROOM_B):
        ix0, ix1 = int(math.floor(x0 / ROOM_RES + shift)), int(math.floor(x1 / ROOM_RES + shift))
        iy0, iy1 = int(math.floor(FREE_Y[0] / ROOM_RES + shift)), int(math.floor(FREE_Y[1] / ROOM_RES + shift))
        mask[ix0:ix1, iy0:iy1] = room_id
    rows, cols = np.nonzero(mask)
    row0, col0 = int(rows.min()) - 5, int(cols.min()) - 5
    crop = mask[row0:int(rows.max()) + 6, col0:int(cols.max()) + 6]
    cv2.imwrite(str(tmp / "room_mask_latest.png"), crop)

    snapshot = {
        "schema": "sysnav_scene_graph_dump/1", "reason": "manual",
        "stamp": {"ros_sec": 100.0, "wall_unix": 1.0e9, "wall_iso": "2026-08-29T00:00:00Z"},
        "frame": "map", "robot_position": {"x": 1.0, "y": 0.0, "z": 0.3},
        "room_grid": {"room_resolution": ROOM_RES, "shift": [shift, shift, 10], "dims": {"rows": ROOM_DIM, "cols": ROOM_DIM},
                      "layout": "row=x_index, col=y_index"},
        "room_mask": {"file": "room_mask_latest.png", "dtype": "uint16",
                      "crop": {"row0": row0, "col0": col0, "rows": int(crop.shape[0]), "cols": int(crop.shape[1])},
                      "nonzero_cells": int(np.count_nonzero(crop)), "max_id": 7},
        "rooms": [
            {"id": 3, "show_id": 1, "label": "Office Room", "label_votes": {"office room": 40}, "is_labeled": True,
             "is_connected": True, "centroid": [-1.45, 0, 0.3], "anchor_point": [-1.45, 0, 0.3], "area_m2": 17.4,
             "polygon_xy": [[-2.9, -3], [0, -3], [0, 3], [-2.9, 3]], "neighbors": [7]},
            {"id": 7, "show_id": 2, "label": "", "label_votes": {}, "is_labeled": False, "is_connected": True,
             "centroid": [1.5, 0, 0.3], "anchor_point": [1.5, 0, 0.3], "area_m2": 18.0,
             "polygon_xy": [[0, -3], [3, -3], [3, 3], [0, 3]], "neighbors": [3]},
        ],
        "objects": [
            {"id": 11, "ids": [11], "label": "chair", "confidence": 0.8, "status": True, "room_id": 3, "img_path": "",
             "timestamp": 90.0, "position": [-1.5, 1.0, 0.4], "bbox3d": box_corners(-1.5, 1.0, 0.5, 0.5), "cloud_xyz": None},
            {"id": 12, "ids": [12, 15], "label": "sofa", "confidence": 0.9, "status": True, "room_id": 3, "img_path": "",
             "timestamp": 91.0, "position": [-1.3, 1.0, 0.4], "bbox3d": box_corners(-1.3, 1.0, 1.6, 0.8, yaw=0.2),
             "cloud_xyz": [[-1.3 + 0.05 * i, 1.0, 0.4] for i in range(-10, 11)]
                          + [[-1.3 + 0.05 * i, 1.05, 0.4] for i in range(-10, 11)]
                          + [[0.5, -2.6, 0.4]]},                                  # stray depth-bleed point
            {"id": 18, "ids": [18], "label": "sofa", "confidence": 0.85, "status": True, "room_id": 3, "img_path": "",
             "timestamp": 95.0, "position": [-0.55, 1.0, 0.4], "bbox3d": box_corners(-0.55, 1.0, 0.3, 0.3),
             "cloud_xyz": [[-0.72 + 0.05 * i, 1.0, 0.4] for i in range(6)]
                          + [[-0.72 + 0.05 * i, 1.05, 0.4] for i in range(6)]},   # fragment, 0.08 m gap to sofa 12
            {"id": 13, "ids": [13], "label": "trash can", "confidence": 0.7, "status": True, "room_id": 7, "img_path": "",
             "timestamp": 92.0, "position": [2.0, -2.0, 0.3], "bbox3d": box_corners(2.0, -2.0, 0.3, 0.3), "cloud_xyz": None},
            {"id": 14, "ids": [14], "label": "person", "confidence": 0.9, "status": True, "room_id": 7, "img_path": "",
             "timestamp": 93.0, "position": [2.0, 2.0, 0.9], "bbox3d": box_corners(2.0, 2.0, 0.5, 0.5, 0, 1.7), "cloud_xyz": None},
            {"id": 19, "ids": [19], "label": "bed", "confidence": 0.9, "status": True, "room_id": 7, "img_path": "",
             "timestamp": 96.0, "position": [1.8, 1.8, 0.3], "bbox3d": box_corners(1.8, 1.8, 1.0, 0.8),
             "cloud_xyz": [[1.3 + 0.05 * i, 1.4, 0.4] for i in range(21)]           # three sides of the rim;
                          + [[1.3 + 0.05 * i, 2.2, 0.4] for i in range(21)]         # the x = 2.3 side is unobserved
                          + [[1.3, 1.4 + 0.05 * i, 0.4] for i in range(17)]},
            {"id": 20, "ids": [20], "label": "table", "confidence": 0.6, "status": True, "room_id": 3, "img_path": "",
             "timestamp": 97.0, "position": [-2.2, -2.2, 0.4], "bbox3d": box_corners(-2.2, -2.2, 0.6, 0.6),
             "cloud_xyz": [[-2.5 + 0.05 * i, -2.5, 0.4] for i in range(13)]         # closed ring around floor the
                          + [[-2.5 + 0.05 * i, -1.9, 0.4] for i in range(13)]       # robot has actually observed
                          + [[-2.5, -2.5 + 0.05 * i, 0.4] for i in range(13)]
                          + [[-1.9, -2.5 + 0.05 * i, 0.4] for i in range(13)]},
            {"id": 16, "ids": [16], "label": "vase", "confidence": 0.5, "status": False, "room_id": 7, "img_path": "",
             "timestamp": 94.0, "position": [2.5, 2.5, 0.9], "bbox3d": box_corners(2.5, 2.5, 0.2, 0.2), "cloud_xyz": None},
        ],
        "doors": [{"x": 0.0, "y": y, "z": 0.3, "room_a": 3, "room_b": 7, "label": 1} for y in np.linspace(-0.25, 0.25, 6)],
        "room_adjacency": [[3, 7]],
    }
    (tmp / "scene_graph_latest.json").write_text(json.dumps(snapshot))
    with open(tmp / "bev_latest.npz", "wb") as handle:
        np.savez_compressed(
            handle, occupancy=occ, explored=exp, trajectory=traj, map_origin_x=BEV_OX, map_origin_y=BEV_OY,
            resolution=BEV_RES, global_cells=BEV_N, start_z=0.3, robot_pose=np.array([1.0, 0.0, 0.3, 0.0]),
            stamp_unix=1.0e9, stamp_ros_sec=100.0, frame="map", reason="manual", scan_seq=42,
            schema="sysnav_bev_dump/1",
        )
    return tmp


class ConverterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.tmp = Path(cls._tmp.name)
        make_synthetic_dump(cls.tmp)
        cls.dump = cs.load_sysnav_dump(cls.tmp)
        cls.opts = cs.ConvertOptions()
        cls.map_info, cls.rc0 = cs.compute_export_grid(cls.dump, cls.opts)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def bev_cell(self, x: float, y: float) -> tuple[int, int]:
        """Export index of the BEV cell containing (x, y) — exact even on cell edges."""
        r, c = bev_index(x, y)
        return r - self.rc0[0], c - self.rc0[1]

    def export_cell(self, x: float, y: float) -> tuple[int, int]:
        res = self.map_info["resolution"]
        return int(round((y - self.map_info["z_min"]) / res)), int(round((x - self.map_info["x_min"]) / res))

    def test_room_mask_index_roundtrip(self):
        grid = self.dump.room_grid
        xs = np.array([-1.234, 0.0, 2.999]); ys = np.array([0.5, -2.05, 2.0])
        ix, iy = cs.room_mask_indices(xs, ys, grid)
        cx, cy = cs.room_mask_cell_center(ix, iy, grid)
        self.assertTrue(np.all(np.abs(cx - xs) <= ROOM_RES / 2 + 1e-9))
        self.assertTrue(np.all(np.abs(cy - ys) <= ROOM_RES / 2 + 1e-9))
        # X-major mask vs BEV Y-major export grid agree on a known cell: (-1.5, 2.0) is in room 3, (1.5, -2.0) in room 7
        self.assertEqual(int(cs.sample_room_ids(self.dump, np.array([-1.5]), np.array([2.0]))[0]), 3)
        self.assertEqual(int(cs.sample_room_ids(self.dump, np.array([1.5]), np.array([-2.0]))[0]), 7)
        self.assertEqual(int(cs.sample_room_ids(self.dump, np.array([-4.0]), np.array([0.0]))[0]), 0)

    def test_export_grid_matches_bev_slicing(self):
        r0, c0 = self.rc0
        res = self.map_info["resolution"]
        self.assertAlmostEqual(self.map_info["x_min"], BEV_OX + (c0 + 0.5) * res)
        self.assertAlmostEqual(self.map_info["z_min"], BEV_OY + (r0 + 0.5) * res)
        trav, unknown, occ = cs.build_traversibility_layers(self.dump, self.map_info, self.rc0, cs.ConvertOptions(clear_trajectory_radius_m=0))
        bev_occ = np.asarray(self.dump.bev["occupancy"]) > 0
        r, c = self.bev_cell(0.0, 2.0)                         # partition wall cell (BEV index -> export index)
        self.assertTrue(occ[r, c]); self.assertTrue(bev_occ[r0 + r, c0 + c])
        self.assertEqual(occ.shape, (self.map_info["H"], self.map_info["W"]))
        np.testing.assert_array_equal(occ, bev_occ[r0:r0 + self.map_info["H"], c0:c0 + self.map_info["W"]])

    def test_trajectory_clearing(self):
        trav_raw, _, occ_raw = cs.build_traversibility_layers(self.dump, self.map_info, self.rc0, cs.ConvertOptions(clear_trajectory_radius_m=0))
        trav, _, occ = cs.build_traversibility_layers(self.dump, self.map_info, self.rc0, self.opts)
        r, c = self.bev_cell(-1.75, 0.0)
        self.assertTrue(occ_raw[r, c]); self.assertFalse(occ[r, c]); self.assertEqual(trav[r, c], 1)
        r, c = self.bev_cell(0.0, 2.0)                         # real wall untouched
        self.assertTrue(occ[r, c])

    def test_room_renumbering_and_nearest_fill(self):
        trav, _, _ = cs.build_traversibility_layers(self.dump, self.map_info, self.rc0, self.opts)
        rooms = cs.build_room_layers(self.dump, self.map_info, self.rc0, trav, self.opts)
        self.assertEqual(rooms.id_map, {3: 1, 7: 2})
        rim = rooms.room_instance_map
        self.assertEqual(rim[self.export_cell(-1.5, 2.0)], 1)
        self.assertEqual(rim[self.export_cell(1.5, -2.0)], 2)
        self.assertEqual(rim[self.export_cell(-3.1, 0.0)], 1)     # 0.2 m outside the mask -> filled
        self.assertEqual(rim[self.export_cell(-4.2, 0.0)], 0)     # > 0.5 m away -> stays 0
        self.assertEqual(len(rooms.room_metadata), 2)
        self.assertEqual(rooms.room_metadata[0]["room_id"], "sysnav|room|3")

    def test_room_polygon_and_type(self):
        trav, _, _ = cs.build_traversibility_layers(self.dump, self.map_info, self.rc0, self.opts)
        rooms = cs.build_room_layers(self.dump, self.map_info, self.rc0, trav, self.opts)
        office, other = rooms.room_metadata
        self.assertEqual(office["room_type"], "office")
        self.assertEqual(other["room_type"], "unknown_room")
        self.assertGreaterEqual(len(office["floor_polygon_xz"]), 3)
        self.assertAlmostEqual(office["bbox"]["x_max"], 0.0, delta=0.1)
        self.assertAlmostEqual(office["bbox"]["z_max"], 3.0, delta=0.1)
        self.assertIn(2, office["sysnav_neighbors"])

    def _objects(self, opts=None):
        opts = opts or self.opts
        trav, _, _ = cs.build_traversibility_layers(self.dump, self.map_info, self.rc0, opts)
        rooms = cs.build_room_layers(self.dump, self.map_info, self.rc0, trav, opts)
        return rooms, cs.build_object_layers(self.dump, self.map_info, self.rc0, rooms, opts)

    def test_object_priority_and_aliases(self):
        # separation off: this test probes the raw per-cell priority rule at the sofa/chair interface,
        # which the seam carve (tested separately) would blank
        rooms, objects = self._objects(cs.ConvertOptions(separate_objects=False))
        by_id = {m["objectId"]: m for m in objects.object_metadata}
        self.assertEqual(by_id["sysnav|11"]["objectType"], "Chair")
        self.assertEqual(by_id["sysnav|12"]["objectType"], "Sofa")
        self.assertEqual(by_id["sysnav|13"]["objectType"], "GarbageCan")
        self.assertNotIn("sysnav|14", by_id)                    # person dropped
        self.assertNotIn("sysnav|16", by_id)                    # status False dropped
        self.assertNotIn("sysnav|18", by_id)                    # sofa fragment merged into sysnav|12
        sofa_id = by_id["sysnav|12"]["instance_id"]
        self.assertEqual(objects.object_instance_map[self.export_cell(-1.5, 1.0)], sofa_id)   # Sofa (100) beats Chair (90)
        self.assertEqual(by_id["sysnav|12"]["sysnav_room_id"], 1)
        # scrambled corners: hull has 4 vertices and covers the 0.5 x 0.5 chair (~100 cells)
        self.assertEqual(len(by_id["sysnav|11"]["bbox_polygon_xz"]), 4)
        self.assertAlmostEqual(by_id["sysnav|11"]["num_grid_cells"], 100, delta=25)

    def test_footprint_merge_and_stray(self):
        rooms, objects = self._objects()
        by_id = {m["objectId"]: m for m in objects.object_metadata}
        sofa = by_id["sysnav|12"]
        self.assertEqual(sofa["merged_sysnav_ids"], [12, 18])
        self.assertEqual(sofa["sysnav_object_ids"], [12, 15, 18])
        sofa_cells = {tuple(c) for c in sofa["grid_cells"]}
        self.assertIn(self.export_cell(-1.7, 1.0), sofa_cells)      # main fragment
        self.assertIn(self.export_cell(-0.5, 1.0), sofa_cells)      # merged fragment
        self.assertNotIn(self.export_cell(0.5, -2.6), sofa_cells)   # stray bleed cell dropped
        self.assertEqual(objects.object_instance_map[self.export_cell(0.5, -2.6)], 0)
        # merging off -> the fragment stays a separate instance
        _, objects_off = self._objects(cs.ConvertOptions(merge_gap_m=0.0))
        ids_off = {m["objectId"] for m in objects_off.object_metadata}
        self.assertIn("sysnav|18", ids_off)

    def test_postprocess_cells_semantics(self):
        # solid shapes are untouched; a stray cell is dropped; a 2-cell seam between blobs is sealed
        rect = [(r, c) for r in range(20, 30) for c in range(40, 50)]
        self.assertEqual(cs._postprocess_cells(rect, 2), sorted(rect))
        self.assertEqual(cs._postprocess_cells(rect + [(80, 80)], 2), sorted(rect))
        two_blobs = rect + [(r, c) for r in range(20, 30) for c in range(52, 60)]
        sealed = set(cs._postprocess_cells(two_blobs, 2))
        self.assertTrue({(r, 50) for r in range(22, 28)} <= sealed)  # seam filled
        self.assertEqual(cs._postprocess_cells(rect, 0), sorted(rect))

    def test_doorway_traversable(self):
        rooms, objects = self._objects()
        doors = [m for m in objects.object_metadata if m["objectType"] == "Doorway"]
        self.assertEqual(len(doors), 1)
        self.assertEqual(doors[0]["door_rooms"], [1, 2])
        # the watershed line became a filled rectangle: cells == bbox area, thin axis >= 0.2 m / 0.05 = 4 cells
        cells = doors[0]["grid_cells"]
        rows = {c[0] for c in cells}; cols = {c[1] for c in cells}
        h = max(rows) - min(rows) + 1; w = max(cols) - min(cols) + 1
        self.assertEqual(len(cells), h * w)
        self.assertGreaterEqual(min(h, w), 4)
        self.assertEqual(len(doors[0]["bbox_polygon_xz"]), 4)
        rep = cs.build_scene_representation(self.dump, self.opts, map_id="001_train", map_split="train")
        r, c = self.export_cell(0.0, 0.0)
        self.assertEqual(rep["object_instance_map"][r, c], doors[0]["instance_id"])
        self.assertEqual(rep["traversibility_map"][r, c], 1)                      # doorway stays free
        self.assertEqual(rep["traversibility_map"][self.export_cell(-1.3, 1.0)], 0)  # sofa blocks
        rep_nb = cs.build_scene_representation(self.dump, cs.ConvertOptions(objects_block=False), map_id="001_train", map_split="train")
        self.assertEqual(rep_nb["traversibility_map"][self.export_cell(-1.3, 1.0)], 1)

    def test_object_margin(self):
        rep = cs.build_scene_representation(self.dump, self.opts, map_id="001_train", map_split="train")
        r, c = self.export_cell(-1.3, 1.35)                          # 0.2 m off the sofa's edge, off the trail
        self.assertEqual(rep["traversibility_map"][r, c], 0)          # inside the inflated margin
        self.assertEqual(rep["object_instance_map"][r, c], 0)         # margin blocks, but never labels
        rep0 = cs.build_scene_representation(self.dump, cs.ConvertOptions(object_margin_m=0.0),
                                             map_id="001_train", map_split="train")
        self.assertEqual(rep0["traversibility_map"][r, c], 1)
        r, c = self.export_cell(0.0, 0.0)                            # doorway centre is beyond both walls' margins
        self.assertEqual(rep["traversibility_map"][r, c], 1)
        # the contour is unconditional: no free cell may touch a blocked object's mask anywhere
        inst = rep["object_instance_map"]; trav = rep["traversibility_map"]
        for dr, dc in ((0, 1), (1, 0)):
            a = inst[:inst.shape[0] - dr, :inst.shape[1] - dc]; b = inst[dr:, dc:]
            ta = trav[:inst.shape[0] - dr, :inst.shape[1] - dc]; tb = trav[dr:, dc:]
            doors = {m["instance_id"] for m in rep["object_metadata"] if m["objectType"] == "Doorway"}
            blocked_a = (a > 0) & ~np.isin(a, list(doors)); blocked_b = (b > 0) & ~np.isin(b, list(doors))
            self.assertFalse(bool((blocked_a & (tb > 0)).any()))
            self.assertFalse(bool((blocked_b & (ta > 0)).any()))

    def test_object_separation_and_unknown_margin(self):
        rep = cs.build_scene_representation(self.dump, self.opts, map_id="001_train", map_split="train")
        inst = rep["object_instance_map"]
        # invariant: no two different instances are 4-adjacent anywhere
        for dr, dc in ((0, 1), (1, 0)):
            a = inst[:inst.shape[0] - dr, :inst.shape[1] - dc]
            b = inst[dr:, dc:]
            self.assertFalse(bool(((a > 0) & (b > 0) & (a != b)).any()))
        # margin against unexplored space: just outside the building wall turns obstacle, far exterior stays grey
        state = scene_representation_to_layered_state(rep, "001_train")
        r, c = self.export_cell(0.0, 3.15)
        self.assertEqual(state["layers"]["occupancy"][r][c], 1)
        self.assertEqual(state["layers"]["occupancy"][0][0], 2)

    def test_occupancy_values(self):
        rep = cs.build_scene_representation(self.dump, self.opts, map_id="001_train", map_split="train")
        state = scene_representation_to_layered_state(rep, "001_train")
        occ = state["layers"]["occupancy"]
        self.assertEqual({v for row in occ for v in row}, {0, 1, 2})
        self.assertEqual(occ[0][0], 2)                                    # outside the building stays grey
        r, c = self.export_cell(1.8, 1.8)
        self.assertEqual(occ[r][c], 1)                                    # enclosed bed interior is obstacle
        self.assertEqual(state["grid_size"], len(state["layers"]["room"]))
        self.assertEqual(state["metadata"]["source"], "sysnav")
        rep2 = cs.build_scene_representation(self.dump, cs.ConvertOptions(unknown_as="obstacle"), map_id="001_train", map_split="train")
        values2 = {v for row in scene_representation_to_layered_state(rep2, "001_train")["layers"]["occupancy"] for v in row}
        self.assertEqual(values2, {0, 1})

    def test_fill_one_sided_shell_and_competition(self):
        # camera shell on ONE side only; BEV occupancy closes the ring -> interior assigned via boundary share
        occupied = np.zeros((100, 100), bool); free = np.zeros((100, 100), bool)
        occupied[40, 40:71] = True; occupied[70, 40:71] = True
        occupied[40:71, 40] = True; occupied[40:71, 70] = True       # closed 30x30 ring
        free[:38, :] = True                                          # observed floor elsewhere
        shell = [(40, c) for c in range(40, 71)]                     # bed shell = the top side only
        cup = [(70, 55), (70, 56)]                                   # tiny object standing on the rim
        sets, added = cs._fill_enclosed_regions([shell, cup], occupied, free)
        self.assertGreater(added[0], 700)                            # bed claims its ~29x29 interior
        self.assertIn((55, 55), set(sets[0]))
        self.assertEqual(added[1], 0)                                # the cup cannot swallow it
        # cup alone (bed undetected): boundary share < 5% -> interior stays unclaimed
        sets2, added2 = cs._fill_enclosed_regions([cup], occupied, free)
        self.assertEqual(added2[0], 0)
        # a second, separate ring away from any shell is bounded by walls only -> untouched
        occupied3 = occupied.copy()
        occupied3[40:71, 82] = True; occupied3[40:71, 92] = True
        occupied3[40, 82:93] = True; occupied3[70, 82:93] = True
        sets3, _ = cs._fill_enclosed_regions([shell], occupied3, free)
        self.assertNotIn((55, 87), set(sets3[0]))

    def test_contain_merge(self):
        def make_record(oid, otype, cells, **extra):
            base = {"object_type": otype, "raw_label": otype.lower(), "cells": sorted(cells), "source": "sysnav_cloud",
                    "polygon": None, "position": [0.0, 0.0, 0.4], "object_id": f"sysnav|{oid}",
                    "sysnav": {"id": oid, "ids": [oid], "confidence": 0.5, "cloud_xyz": None}}
            base.update(extra)
            return base
        host = make_record(4, "Sofa", [(r, c) for r in range(10, 30) for c in range(10, 30)], boxed_cells=50)
        fragment = make_record(6, "Sofa", [(r, c) for r in range(15, 18) for c in range(15, 18)])
        neighbour = make_record(7, "Sofa", [(r, c) for r in range(10, 30) for c in range(28, 45)])  # touches, ~10% overlap
        chair = make_record(9, "Chair", [(r, c) for r in range(20, 23) for c in range(20, 23)])     # inside, other type
        out = cs._merge_contained_records([host, fragment, neighbour, chair], 0.8)
        by_id = {r["object_id"]: r for r in out}
        self.assertEqual(len(out), 3)
        self.assertIn("sysnav|4", by_id)
        self.assertNotIn("sysnav|6", by_id)                          # fragment folded into the host
        self.assertEqual(by_id["sysnav|4"]["merged_ids"], [4, 6])
        self.assertEqual(by_id["sysnav|4"]["boxed_cells"], 50)
        self.assertIn("sysnav|7", by_id)                             # touching neighbour survives
        self.assertIn("sysnav|9", by_id)                             # stacked other-type object survives
        self.assertEqual(len(cs._merge_contained_records([host, fragment], 0.0)), 2)   # 0 = off

    def test_absorb_pockets(self):
        # building: walls rows/cols 10 & 40; sofa against the top wall with an unexplored strip behind it
        occupied = np.zeros((60, 60), bool); free = np.zeros((60, 60), np.uint8)
        occupied[10, 10:41] = occupied[40, 10:41] = True
        occupied[10:41, 10] = occupied[10:41, 40] = True
        free[15:40, 11:40] = 1                                        # explored floor
        free[11:15, 11:14] = 1                                        # explored gap left of the sofa
        sofa = [(r, c) for r in range(12, 15) for c in range(15, 31)]
        sets, added = cs._absorb_enclosed_pockets([sofa], occupied, free)
        claimed = set(sets[0])
        self.assertIn((11, 20), claimed)                              # strip behind the sofa absorbed
        self.assertGreaterEqual(added[0], 16)                         # the full 1x16 strip (+ a few edge cells)
        self.assertNotIn((10, 20), claimed)                           # wall itself is exterior-adjacent -> background
        self.assertNotIn((10, 35), claimed)                           # wall far along: background reclaims
        self.assertNotIn((40, 20), claimed)                           # opposite wall untouched
        self.assertNotIn((9, 20), claimed)                            # exterior unknown never claimed

    def test_inaccessible_free_absorbed(self):
        # chair ring with 1-cell leg gaps around observed floor: interior unreachable (corridors < 3 cells)
        occupied = np.zeros((60, 60), bool)
        free = np.zeros((60, 60), np.uint8); free[5:55, 5:55] = 1
        traj = np.zeros((60, 60), bool); traj[30, 5:55] = True
        ring = [(r, c) for r in range(10, 21) for c in range(10, 21) if r in (10, 20) or c in (10, 20)]
        ring = [cell for cell in ring if cell not in {(10, 15), (15, 10), (20, 15), (15, 20)}]   # leg gaps
        sets, added = cs._absorb_enclosed_pockets([ring], occupied, free, trajectory=traj)
        self.assertIn((15, 15), set(sets[0]))                        # under-chair floor claimed
        self.assertGreater(added[0], 60)
        # a wide-open alcove (>= 3-cell corridor) stays free
        alcove = [(r, c) for r in range(40, 51) for c in range(40, 51) if (r in (40, 50) or c in (40, 50))]
        alcove = [cell for cell in alcove if not (cell[0] == 40 and 43 <= cell[1] <= 47)]        # 5-cell opening
        sets2, added2 = cs._absorb_enclosed_pockets([alcove], occupied, free, trajectory=traj)
        self.assertNotIn((45, 45), set(sets2[0]))

    def test_guarded_box(self):
        free = np.ones((40, 40), np.uint8)
        free[10:20, 10:26] = 0                                        # non-free zone
        free[14, 14] = 1                                              # observed floor speck inside it
        cells = [(r, 10) for r in range(10, 20)] + [(10, c) for c in range(10, 26)]   # open C/L shape
        boxed, added = cs._box_cells(cells, free, "guarded")
        self.assertGreater(added, 100)
        self.assertIn((19, 25), set(boxed))                           # open bay absorbed by the rectangle
        self.assertNotIn((14, 14), set(boxed))                        # observed floor stays out
        full, _ = cs._box_cells(cells, free, "full")
        self.assertIn((14, 14), set(full))
        same, zero = cs._box_cells(cells, free, "off")
        self.assertEqual(zero, 0)

    def test_mask_holes_filled(self):
        ring = [(r, c) for r in range(10, 21) for c in range(10, 21) if r in (10, 20) or c in (10, 20)]
        free = np.zeros((40, 40), bool)
        cells, added = cs._fill_mask_holes(ring, free)                # interior non-free -> filled solid
        self.assertEqual(added, 81)
        self.assertIn((15, 15), set(cells))
        free[11:20, 11:20] = True                                     # interior is observed floor -> guarded
        cells2, added2 = cs._fill_mask_holes(ring, free)
        self.assertEqual(added2, 0)

    def test_bed_interior_filled_and_guard(self):
        rep = cs.build_scene_representation(self.dump, self.opts, map_id="001_train", map_split="train")
        meta = {m["objectId"]: m for m in rep["object_metadata"]}
        bed = meta["sysnav|19"]
        self.assertEqual(bed["objectType"], "Bed")
        self.assertGreater(bed.get("filled_cells", 0), 100)               # occluded interior claimed
        r, c = self.export_cell(1.8, 1.8)
        self.assertEqual(rep["object_instance_map"][r, c], bed["instance_id"])
        self.assertEqual(rep["traversibility_map"][r, c], 0)
        # ring around observed floor: that floor is unreachable once the ring stands -> now claimed
        table = meta["sysnav|20"]
        r, c = self.export_cell(-2.2, -2.2)
        self.assertEqual(rep["object_instance_map"][r, c], table["instance_id"])
        self.assertGreater(table.get("absorbed_cells", 0), 0)

    def test_end_to_end_outputs(self):
        with tempfile.TemporaryDirectory() as out:
            prefix = build_output_prefix(out, "001_train")
            outputs = transform_sysnav_to_map(self.dump, prefix, "001_train", self.opts, skip_overview=False)
            for path in expected_simple_demo_output_paths(prefix):
                self.assertTrue(path.exists(), path)
            for key in ("maps_path", "metadata_path", "overview_path"):
                self.assertTrue(outputs[key].exists(), key)
            payload = json.loads(outputs["json_path"].read_text())
            self.assertEqual(validate_map_shape(payload), [])
            self.assertEqual(validate_rich_object_schema(payload), [])
            self.assertEqual(validate_instance_ids(payload), [])
            self.assertEqual(payload["metadata"]["grid_coordinate_frame"]["row_axis"], "ros_y")
            self.assertEqual(payload["metadata"]["source"], "sysnav")
            self.assertIn("object_footprints", payload)
            self.assertIn("cell_object_ids", payload)
            self.assertEqual(len(payload["room_instances"]), 2)
            metadata = json.loads(outputs["metadata_path"].read_text())
            self.assertEqual(metadata["sysnav"]["room_id_map"], {"3": 1, "7": 2})
            thinggraph = json.loads(outputs["thinggraph_path"].read_text())
            self.assertEqual(thinggraph["summary"]["unassigned_object_count"], 0)
            self.assertEqual(thinggraph["summary"]["room_count"], 2)
            manifest = json.loads((prefix.parent / "001_train_metric_cache" / "manifest.json").read_text())
            stat = outputs["json_path"].stat()
            self.assertEqual(manifest["source"]["size"], stat.st_size)
            self.assertEqual(manifest["source"]["mtime_ns"], stat.st_mtime_ns)
            with self.assertRaises(FileExistsError):
                transform_sysnav_to_map(self.dump, prefix, "001_train", self.opts, skip_overview=True)

    def test_relative_output_dir(self):
        # The vendored metric-cache builder resolves relative paths against its own package root; the
        # converter must therefore hand it absolute paths even when --output-dir is relative.
        with tempfile.TemporaryDirectory() as out:
            cwd = os.getcwd()
            try:
                os.chdir(out)
                outputs = transform_sysnav_to_map(self.dump, build_output_prefix("rel_out", "002_train"), "002_train",
                                                  self.opts, skip_overview=True)
            finally:
                os.chdir(cwd)
            self.assertTrue(outputs["json_path"].is_absolute())
            self.assertTrue((Path(out) / "rel_out/train/002_train/002_train_metric_cache/manifest.json").exists())

    def test_cli_build_output_prefix(self):
        self.assertEqual(build_output_prefix("out", "001_train"), Path("out/train/001_train/001_train"))
        self.assertEqual(build_output_prefix("out", "003_valunseen"), Path("out/valunseen/003_valunseen/003_valunseen"))
        with self.assertRaises(ValueError):
            build_output_prefix("out", "office_map")

    def test_vendored_files_unmodified(self):
        manifest = json.loads((VENDOR_DIR / "MANIFEST.json").read_text())
        for name, record in manifest["files"].items():
            digest = hashlib.sha256((VENDOR_DIR / name).read_bytes()).hexdigest()
            self.assertEqual(digest, record["vendored_sha256"], f"{name} was modified; re-sync per VENDORED.md")


if __name__ == "__main__":
    unittest.main()
