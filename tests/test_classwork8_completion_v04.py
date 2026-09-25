import unittest

from classwork8.config import Classwork8Config
from classwork8.tof_only_v04 import (
    _closed_maze_completion_v04,
    _set_edge_state,
)


class ClosedMazeCompletionV04Tests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.config.closed_maze_auto_stop = True
        self.config.closed_maze_perimeter_wall_ratio = 0.70
        self.config.closed_maze_min_rows = 2
        self.config.closed_maze_min_cols = 2

    def _filled_rectangle(self, rows, cols):
        return {
            (x, -y)
            for x in range(rows)
            for y in range(cols)
        }

    def _mark_perimeter_walls(self, edges, rows, cols):
        min_x, max_x = 0, rows - 1
        min_y, max_y = -(cols - 1), 0

        for y in range(min_y, max_y + 1):
            _set_edge_state(edges, (max_x, y), 0, "WALL")
            _set_edge_state(edges, (min_x, y), 2, "WALL")

        for x in range(min_x, max_x + 1):
            _set_edge_state(edges, (x, min_y), 1, "WALL")
            _set_edge_state(edges, (x, max_y), 3, "WALL")

    def test_latest_run_shape_tolerates_one_missed_wall_on_four_cell_side(self):
        visited = self._filled_rectangle(5, 4)
        edges = {}
        self._mark_perimeter_walls(edges, 5, 4)

        # Reproduce the latest field pattern: one of four BACK boundary
        # readings is a ToF miss, leaving that edge OPEN.
        _set_edge_state(edges, (0, -1), 2, "OPEN")

        result = _closed_maze_completion_v04(
            visited,
            edges,
            set(),
            self.config,
        )

        self.assertTrue(result["filled"])
        self.assertEqual(result["rows"], 5)
        self.assertEqual(result["cols"], 4)
        self.assertAlmostEqual(result["ratios"]["BACK"], 0.75)
        self.assertTrue(result["complete"])

    def test_does_not_complete_with_unvisited_hole(self):
        visited = self._filled_rectangle(5, 4)
        visited.remove((2, -2))

        edges = {}
        self._mark_perimeter_walls(edges, 5, 4)

        result = _closed_maze_completion_v04(
            visited,
            edges,
            set(),
            self.config,
        )

        self.assertFalse(result["filled"])
        self.assertFalse(result["complete"])

    def test_does_not_complete_when_outer_side_is_still_open(self):
        visited = self._filled_rectangle(5, 4)
        edges = {}
        self._mark_perimeter_walls(edges, 5, 4)

        # Miss two of four BACK edges -> only 50%, below 70%.
        _set_edge_state(edges, (0, -1), 2, "OPEN")
        _set_edge_state(edges, (0, -2), 2, "OPEN")

        result = _closed_maze_completion_v04(
            visited,
            edges,
            set(),
            self.config,
        )

        self.assertAlmostEqual(result["ratios"]["BACK"], 0.50)
        self.assertFalse(result["complete"])


if __name__ == "__main__":
    unittest.main()
