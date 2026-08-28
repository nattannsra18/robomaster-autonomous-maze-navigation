import tempfile
import unittest
from pathlib import Path

from robomaster_mission.configuration import HybridConfig
from robomaster_mission.grid_map import GridMazeMap
from robomaster_mission.reporting import export_run_artifacts


class WallEvidenceTests(unittest.TestCase):
    def test_one_open_reading_does_not_remove_confirmed_wall(self):
        maze = GridMazeMap(2, 2)
        maze.observe_edge((0, 0), 1, True)
        maze.observe_edge((0, 0), 1, False)
        self.assertTrue(maze.has_wall((0, 0), 1))

    def test_three_consecutive_open_readings_remove_wall(self):
        maze = GridMazeMap(2, 2)
        maze.observe_edge((0, 0), 1, True)
        for _ in range(3):
            maze.observe_edge((0, 0), 1, False)
        self.assertFalse(maze.has_wall((0, 0), 1))

    def test_blocked_reading_resets_open_streak(self):
        maze = GridMazeMap(2, 2)
        maze.observe_edge((0, 0), 1, True)
        maze.observe_edge((0, 0), 1, False)
        maze.observe_edge((0, 0), 1, False)
        maze.observe_edge((0, 0), 1, True)
        maze.observe_edge((0, 0), 1, False)
        self.assertTrue(maze.has_wall((0, 0), 1))

    def test_traversal_is_immediate_strong_open_evidence(self):
        maze = GridMazeMap(2, 2)
        maze.observe_edge((0, 0), 1, True)
        maze.confirm_traversed_open_edge((0, 0), 1)
        self.assertFalse(maze.has_wall((0, 0), 1))
        self.assertTrue(maze.is_traversed_open((0, 0), 1))

    def test_artifacts_include_sensor_graph(self):
        maze = GridMazeMap(2, 2)
        maze.start, maze.drop, maze.exit = (0, 0), (0, 1), (1, 1)
        maze.record_pose((0, 0), 1)
        maze.record_sensor_snapshot((0, 0), 1, 42.0, 15.0, 31.0)
        config = HybridConfig(rows=2, cols=2, output_prefix="test_run")
        with tempfile.TemporaryDirectory() as directory:
            old_prefix = config.output_prefix
            config.output_prefix = str(Path(directory) / old_prefix)
            paths = export_run_artifacts(config, maze)
            self.assertEqual(len(paths), 3)
            for path in paths:
                self.assertTrue(path.is_file())


if __name__ == "__main__":
    unittest.main()
