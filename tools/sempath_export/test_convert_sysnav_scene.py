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
             "cloud_xyz": [[-1.3 + 0.1 * i, 1.0, 0.4] for i in range(-5, 6)]},
            {"id": 13, "ids": [13], "label": "trash can", "confidence": 0.7, "status": True, "room_id": 7, "img_path": "",
             "timestamp": 92.0, "position": [2.0, -2.0, 0.3], "bbox3d": box_corners(2.0, -2.0, 0.3, 0.3), "cloud_xyz": None},
            {"id": 14, "ids": [14], "label": "person", "confidence": 0.9, "status": True, "room_id": 7, "img_path": "",
             "timestamp": 93.0, "position": [2.0, 2.0, 0.9], "bbox3d": box_corners(2.0, 2.0, 0.5, 0.5, 0, 1.7), "cloud_xyz": None},
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
        rooms, objects = self._objects()
        by_id = {m["objectId"]: m for m in objects.object_metadata}
        self.assertEqual(by_id["sysnav|11"]["objectType"], "Chair")
        self.assertEqual(by_id["sysnav|12"]["objectType"], "Sofa")
        self.assertEqual(by_id["sysnav|13"]["objectType"], "GarbageCan")
        self.assertNotIn("sysnav|14", by_id)                    # person dropped
        self.assertNotIn("sysnav|16", by_id)                    # status False dropped
        sofa_id = by_id["sysnav|12"]["instance_id"]
        self.assertEqual(objects.object_instance_map[self.export_cell(-1.5, 1.0)], sofa_id)   # Sofa (100) beats Chair (90)
        self.assertEqual(by_id["sysnav|12"]["sysnav_room_id"], 1)
        # scrambled corners: hull has 4 vertices and covers the 0.5 x 0.5 chair (~100 cells)
        self.assertEqual(len(by_id["sysnav|11"]["bbox_polygon_xz"]), 4)
        self.assertAlmostEqual(by_id["sysnav|11"]["num_grid_cells"], 100, delta=25)

    def test_doorway_traversable(self):
        rooms, objects = self._objects()
        doors = [m for m in objects.object_metadata if m["objectType"] == "Doorway"]
        self.assertEqual(len(doors), 1)
        self.assertEqual(doors[0]["door_rooms"], [1, 2])
        rep = cs.build_scene_representation(self.dump, self.opts, map_id="001_train", map_split="train")
        r, c = self.export_cell(0.0, 0.0)
        self.assertEqual(rep["object_instance_map"][r, c], doors[0]["instance_id"])
        self.assertEqual(rep["traversibility_map"][r, c], 1)                      # doorway stays free
        self.assertEqual(rep["traversibility_map"][self.export_cell(-1.3, 1.0)], 0)  # sofa blocks
        rep_nb = cs.build_scene_representation(self.dump, cs.ConvertOptions(objects_block=False), map_id="001_train", map_split="train")
        self.assertEqual(rep_nb["traversibility_map"][self.export_cell(-1.3, 1.0)], 1)

    def test_occupancy_values(self):
        rep = cs.build_scene_representation(self.dump, self.opts, map_id="001_train", map_split="train")
        state = scene_representation_to_layered_state(rep, "001_train")
        values = {v for row in state["layers"]["occupancy"] for v in row}
        self.assertEqual(values, {0, 1, 2})
        self.assertEqual(state["grid_size"], len(state["layers"]["room"]))
        self.assertEqual(state["metadata"]["source"], "sysnav")
        rep2 = cs.build_scene_representation(self.dump, cs.ConvertOptions(unknown_as="obstacle"), map_id="001_train", map_split="train")
        values2 = {v for row in scene_representation_to_layered_state(rep2, "001_train")["layers"]["occupancy"] for v in row}
        self.assertEqual(values2, {0, 1})

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
