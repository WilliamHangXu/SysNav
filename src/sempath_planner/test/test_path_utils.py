"""Pure-function tests: pixel<->world inversion and path decimation (no ROS needed).

    python3 -m unittest discover src/sempath_planner/test
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sempath_planner.path_utils import (
    decimate_path, pixel_to_world, rdp, semantic_map_cells, trajectory_to_world, world_to_pixel)

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


if __name__ == "__main__":
    unittest.main()
