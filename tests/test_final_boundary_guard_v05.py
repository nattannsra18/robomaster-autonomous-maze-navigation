"""Offline guards against looking beyond the low physical maze wall.

These tests do not initialize the RoboMaster SDK or issue motion commands.
"""
import sys
import types
import unittest

if "libmedia_codec" not in sys.modules:
    media = types.ModuleType("libmedia_codec")
    class H264Decoder:
        def decode(self, _data):
            return []
    class OpusDecoder:
        def decode(self, _data):
            return None
    media.H264Decoder = H264Decoder
    media.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media

from classwork8.config import Classwork8Config
from classwork8.occupancy_grid import OccupancyGrid, FREE, UNKNOWN
from classwork8.tof_camera_round1_v05 import (
    _closed_maze_completion_v04, _flanked_by_confirmed_walls,
    _plan_frontier_move, _ray_limit_to_cell_face,
    _set_edge_state, _update_tof_ray,
)


class BoundaryMapTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Classwork8Config()
        self.cfg.cell_size_m = self.cfg.exploration_step_m = 0.60

    def grid(self):
        return OccupancyGrid(8.0, 8.0, 0.05)

    def test_long_stationary_ray_does_not_paint_exterior(self):
        grid = self.grid()
        limit = _ray_limit_to_cell_face(self.cfg, (0, 0), 0.0, 0.0, 0)
        self.assertGreater(limit, 0.10)
        self.assertLess(limit, 0.18)
        _update_tof_ray(grid, self.cfg, 0.0, 0.0, 0, 280.0,
                        mapped_until_m=limit)
        inside = grid.world_to_cell(0.16, 0.0)
        outside = grid.world_to_cell(0.60, 0.0)
        self.assertEqual(grid.state_at(*inside), FREE)
        self.assertEqual(grid.state_at(*outside), UNKNOWN)

    def test_real_close_hit_still_records_occupied(self):
        grid = self.grid()
        limit = _ray_limit_to_cell_face(self.cfg, (0, 0), 0.0, 0.0, 0)
        _update_tof_ray(grid, self.cfg, 0.0, 0.0, 0, 8.0,
                        mapped_until_m=limit)
        from classwork8.occupancy_grid import OCCUPIED
        cell = grid.world_to_cell(0.20, 0.0)
        self.assertEqual(grid.state_at(*cell), OCCUPIED)

    def test_moving_ray_is_limited_to_entered_destination_cell(self):
        grid = self.grid()
        # Robot centre has physically crossed the face of target cell +1.
        limit = _ray_limit_to_cell_face(self.cfg, (1, 0), 0.4, 0.0, 0)
        self.assertGreater(limit, 0.25)
        self.assertLess(limit, 0.40)
        _update_tof_ray(grid, self.cfg, 0.4, 0.0, 0, 280.0,
                        mapped_until_m=limit)
        self.assertEqual(grid.state_at(*grid.world_to_cell(1.2, 0.0)), UNKNOWN)

    def test_no_default_map_expansion_from_far_sighting(self):
        from classwork8.target_map_visual import target_plot_geometry
        hint, observer, sighting = target_plot_geometry({
            "localization_status": "SIGHTING_ONLY",
            "range_confirmed_wall": False,
            "sighting_cell_hint": [4, 0],
            "observation_cells": [[0, 0]],
            "view_directions": [0],
        }, 0.60)
        self.assertTrue(sighting)
        self.assertEqual(observer, (0.0, 0.0))
        self.assertLess(hint[0], 0.30)


class CompletionAndRouteTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Classwork8Config()
        self.cfg.closed_maze_min_rows = 2
        self.cfg.closed_maze_min_cols = 2
        self.visited = {(0, 0), (0, 1), (1, 0), (1, 1)}
        self.edges = {}
        for y in (0, 1):
            _set_edge_state(self.edges, (1, y), 0, "WALL")
            _set_edge_state(self.edges, (0, y), 2, "WALL")
        for x in (0, 1):
            _set_edge_state(self.edges, (x, 0), 1, "WALL")
            _set_edge_state(self.edges, (x, 1), 3, "WALL")

    def test_do_not_finish_when_a_boundary_edge_is_open(self):
        _set_edge_state(self.edges, (1, 0), 0, "OPEN")
        result = _closed_maze_completion_v04(
            self.visited, self.edges, set(), self.cfg
        )
        self.assertEqual(result["boundary_open_edges"], 1)
        self.assertFalse(result["complete"])

    def test_deferred_edge_is_not_a_fake_perimeter_wall(self):
        _set_edge_state(self.edges, (1, 0), 0, "OPEN")
        deferred = (1, 0), (2, 0)
        result = _closed_maze_completion_v04(
            self.visited, self.edges, {((1, 0), 0)}, self.cfg,
            {deferred}
        )
        self.assertFalse(result["complete"])

    def test_stable_far_echo_cannot_open_continuous_observed_wall(self):
        visited = {(0, 0), (0, 1), (0, -1)}
        edges = {}
        _set_edge_state(edges, (0, 1), 0, "WALL")
        _set_edge_state(edges, (0, -1), 0, "WALL")
        self.assertTrue(_flanked_by_confirmed_walls(
            (0, 0), 0, visited, edges,
        ))
        # One missing flank must not invent a wall.
        self.assertFalse(_flanked_by_confirmed_walls(
            (0, 0), 0, visited - {(0, -1)}, edges,
        ))

    def test_confirmed_open_neighbor_still_plannable(self):
        _set_edge_state(self.edges, (1, 0), 0, "OPEN")
        plan = _plan_frontier_move(
            (1, 0), self.visited, self.edges, set(), self.cfg, 0
        )
        self.assertIsNotNone(plan)
        self.assertEqual(plan["next_cell"], (2, 0))


if __name__ == "__main__":
    unittest.main()
