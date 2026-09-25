import sys
import types
import unittest


# Unit tests exercise planner/completion logic only.  RoboMaster SDK imports
# media.py on Windows even though these tests never open a camera or connect
# to a robot, so provide a tiny compatibility stub before importing classwork8.
if "libmedia_codec" not in sys.modules:
    media_codec = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    media_codec.H264Decoder = H264Decoder
    media_codec.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media_codec


from classwork8.config import Classwork8Config
from classwork8.tof_only_v03 import (
    _plan_frontier_move,
    _set_edge_state,
)


class FrontierPlannerV03Tests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.config.map_width_m = 20.0
        self.config.map_height_m = 20.0
        self.blocked = set()

    def test_local_unvisited_open_neighbor_is_expanded_first(self):
        visited = {(0, 0)}
        edges = {}
        _set_edge_state(edges, (0, 0), 0, "OPEN")

        plan = _plan_frontier_move(
            (0, 0),
            visited,
            edges,
            self.blocked,
            self.config,
            0,
        )

        self.assertIsNotNone(plan)
        self.assertTrue(plan["is_new"])
        self.assertEqual(plan["next_cell"], (1, 0))
        self.assertEqual(plan["mode"], "EXPAND_LOCAL_FRONTIER")

    def test_relocates_to_nearest_frontier_not_parent_stack(self):
        # Visited T-shaped graph:
        #   frontier at (0,1) -> (0,2)
        #       |
        # (0,0)-(1,0)-(2,0) current
        #
        # From (2,0), nearest frontier route is (2,0)->(1,0)->(0,0)->(0,1).
        visited = {(0, 0), (1, 0), (2, 0), (0, 1)}
        edges = {}

        _set_edge_state(edges, (0, 0), 0, "OPEN")
        _set_edge_state(edges, (1, 0), 0, "OPEN")
        _set_edge_state(edges, (0, 0), 3, "OPEN")
        _set_edge_state(edges, (0, 1), 3, "OPEN")

        plan = _plan_frontier_move(
            (2, 0),
            visited,
            edges,
            self.blocked,
            self.config,
            0,
        )

        self.assertIsNotNone(plan)
        self.assertFalse(plan["is_new"])
        self.assertEqual(plan["next_cell"], (1, 0))
        self.assertEqual(plan["frontier_cell"], (0, 1))
        self.assertEqual(plan["frontier_target"], (0, 2))
        self.assertEqual(plan["mode"], "RELOCATE_TO_FRONTIER")

    def test_shorter_frontier_route_wins(self):
        visited = {(0, 0), (1, 0), (2, 0), (0, 1)}
        edges = {}

        _set_edge_state(edges, (0, 0), 0, "OPEN")
        _set_edge_state(edges, (1, 0), 0, "OPEN")
        _set_edge_state(edges, (0, 0), 3, "OPEN")

        # Near frontier from (2,0): (1,0) -> (1,-1)
        _set_edge_state(edges, (1, 0), 1, "OPEN")

        # Farther frontier from start branch: (0,1) -> (0,2)
        _set_edge_state(edges, (0, 1), 3, "OPEN")

        plan = _plan_frontier_move(
            (2, 0),
            visited,
            edges,
            self.blocked,
            self.config,
            0,
        )

        self.assertIsNotNone(plan)
        self.assertEqual(plan["frontier_cell"], (1, 0))
        self.assertEqual(plan["frontier_target"], (1, -1))
        self.assertEqual(plan["next_cell"], (1, 0))


if __name__ == "__main__":
    unittest.main()
