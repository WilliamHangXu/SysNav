"""Pure-function tests: pixel<->world inversion and path decimation (no ROS needed).

    python3 -m unittest discover src/sempath_planner/test
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sempath_planner.path_utils import (
    cmdline_matches, decimate_path, pixel_to_world, rdp, semantic_map_cells,
    trajectory_to_world, world_to_pixel)

FRAME = {"resolution": 0.05, "x_min": -3.475, "z_min": 1.025}  # centre of col 0 / row 0


class PixelWorldTest(unittest.TestCase):
    def test_roundtrip_is_exact_on_cell_centres(self):
        for row, col in [(0, 0), (17, 3), (240, 511)]:
            x, y = pixel_to_world(row, col, FRAME)
            self.assertEqual(world_to_pixel(x, y, FRAME), (row, col))

    def test_world_to_pixel_rounds_to_nearest_centre(self):
        x, y = pixel_to_world(10, 20, FRAME)
        self.assertEqual(world_to_pixel(x + 0.02, y - 0.02, FRAME), (10, 20))
        self.assertEqual(world_to_pixel(x + 0.03, y, FRAME), (10, 21))

    def test_returns_plain_ints(self):
        import numpy as np
        row, col = world_to_pixel(np.float64(0.4), np.float64(1.6), FRAME)
        self.assertIs(type(row), int)  # GroundPlan rejects numpy ints via isinstance(x, int)
        self.assertIs(type(col), int)

    def test_trajectory_to_world_matches_grid_formula(self):
        world = trajectory_to_world([[0, 0], [2, 4]], FRAME)
        self.assertAlmostEqual(world[1][0], FRAME["x_min"] + 4 * 0.05)
        self.assertAlmostEqual(world[1][1], FRAME["z_min"] + 2 * 0.05)


class SemanticMapCellsTest(unittest.TestCase):
    def test_cells_match_pixel_to_world_and_skip_unknown(self):
        rgb = [[(255, 255, 255), (10, 20, 30)],
               [(40, 50, 60), (0, 0, 0)]]
        occupancy = [[0, 2],   # (0,1) unknown -> dropped
                     [1, 0]]
        cells = semantic_map_cells(rgb, occupancy, FRAME)
        self.assertEqual(len(cells), 3)
        by_color = {c: (x, y) for x, y, c in cells}
        self.assertNotIn((10, 20, 30), by_color)                       # the unknown cell
        self.assertEqual(by_color[(40, 50, 60)], pixel_to_world(1, 0, FRAME))  # row 1 = +y, col 0
        self.assertEqual(by_color[(0, 0, 0)], pixel_to_world(1, 1, FRAME))

    def test_plain_int_colors_from_numpy_image(self):
        import numpy as np
        rgb = np.full((1, 1, 3), 200, dtype=np.uint8)
        cells = semantic_map_cells(rgb, [[0]], FRAME)
        self.assertEqual(cells[0][2], (200, 200, 200))
        self.assertTrue(all(type(v) is int for v in cells[0][2]))


class DecimationTest(unittest.TestCase):
    def test_rdp_keeps_corners_drops_collinear(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (2.0, 2.0)]
        self.assertEqual(rdp(pts, 0.05), [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])

    def test_rdp_keeps_small_detour_when_above_epsilon(self):
        pts = [(0.0, 0.0), (1.0, 0.3), (2.0, 0.0)]
        self.assertEqual(len(rdp(pts, 0.1)), 3)
        self.assertEqual(len(rdp(pts, 0.5)), 2)

    def test_decimate_enforces_max_spacing(self):
        pts = [(0.0, 0.0), (6.0, 0.0)]
        out = decimate_path(pts, 0.1, 1.5)
        self.assertEqual(out[0], (0.0, 0.0))
        self.assertEqual(out[-1], (6.0, 0.0))
        for a, b in zip(out, out[1:]):
            self.assertLessEqual(math.hypot(b[0] - a[0], b[1] - a[1]), 1.5 + 1e-9)

    def test_decimate_preserves_endpoints_and_corner(self):
        pts = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0, 0.5), (1.0, 1.0)]
        out = decimate_path(pts, 0.05, 10.0)
        self.assertEqual(out, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])


MAPPING = {"tare_planner_node", "room_segmentation", "detection_node",
           "semantic_mapping_node", "vlm_reasoning_node", "bev_mapper_node"}


class CmdlineMatchesTest(unittest.TestCase):
    def test_matches_python_node_shim_and_cpp_binary(self):
        py = ["/usr/bin/python3", "/x/install/semantic_mapping/lib/semantic_mapping/detection_node",
              "--ros-args", "--params-file", "/x/share/mapping_mecanum_sim.yaml"]
        cpp = ["/x/install/tare_planner/lib/tare_planner/tare_planner_node",
               "--ros-args", "-p", "scenario:=matterport_sim"]
        self.assertEqual(cmdline_matches(py, MAPPING), "detection_node")
        self.assertEqual(cmdline_matches(cpp, MAPPING), "tare_planner_node")

    def test_spares_package_mates_and_path_substrings(self):
        # the keyboard terminal lives in the vlm_node PACKAGE: its argv contains "vlm_node"
        # both as a path component and as a bare `ros2 run` token — must never match
        terminal = ["bash", "-c", "ros2 run vlm_node keyboard_input"]
        run = ["ros2", "run", "vlm_node", "keyboard_input"]
        shim = ["/usr/bin/python3", "/x/install/vlm_node/lib/vlm_node/keyboard_input", "--ros-args"]
        for tokens in (terminal, run, shim):
            self.assertIsNone(cmdline_matches(tokens, MAPPING))

    def test_basename_must_match_exactly(self):
        # a params-file named after a node must not drag an unrelated process in
        tokens = ["/usr/bin/python3", "/x/lib/foo/other_node",
                  "--params-file", "/x/share/detection_node.yaml"]
        self.assertIsNone(cmdline_matches(tokens, MAPPING))


if __name__ == "__main__":
    unittest.main()
