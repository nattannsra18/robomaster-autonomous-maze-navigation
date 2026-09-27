import json
import tempfile
import unittest
from pathlib import Path

from classwork8.config import Classwork8Config
from classwork8.evaluate import calculate_metrics
from classwork8.ground_truth_editor import GroundTruthLayout
from classwork8.occupancy_grid import FREE, UNKNOWN, OCCUPIED as WALL


class GroundTruthEditorModelTests(unittest.TestCase):
    def test_wall_toggle_mirrors_neighbour_and_outer_walls_stay_closed(self):
        maze = GroundTruthLayout(3, 4)
        self.assertTrue(maze.has_wall(0, 0, 0))  # outer FRONT
        self.assertTrue(maze.has_wall(0, 0, 3))  # outer LEFT
        maze.toggle_wall(0, 0, 1)
        self.assertTrue(maze.has_wall(0, 0, 1))
        self.assertTrue(maze.has_wall(0, 1, 3))
        maze.toggle_wall(0, 1, 3)
        self.assertFalse(maze.has_wall(0, 0, 1))
        maze.toggle_wall(0, 0, 0)
        self.assertTrue(maze.has_wall(0, 0, 0))

    def test_requires_start_before_export(self):
        with self.assertRaisesRegex(ValueError, "START"):
            GroundTruthLayout(5, 5).crop_coordinates(Classwork8Config())

    def test_raster_shape_and_internal_wall(self):
        maze = GroundTruthLayout(3, 4, (1, 2))
        maze.set_wall(1, 1, 1, True)
        grid = maze.rasterize()
        self.assertEqual((len(grid), len(grid[0])), (48, 36))
        self.assertTrue(all(value in (FREE, WALL) for row in grid for value in row))
        # RIGHT of physical editor cell (1,1) is a line at raster row 24;
        # its segment occupies x columns 12..23 in occupancy-grid frame.
        self.assertEqual(grid[23][17], WALL)
        self.assertEqual(grid[24][17], WALL)
        self.assertEqual(grid[21][17], FREE)
        self.assertEqual(grid[23][5], FREE)

    def test_crop_coordinates_are_calculated_from_real_start_not_slam(self):
        maze = GroundTruthLayout(7, 7, (3, 3))
        top, left = maze.crop_coordinates(Classwork8Config())
        self.assertEqual((top, left), (38, 38))
        maze2 = GroundTruthLayout(3, 4, (1, 2))
        self.assertEqual(maze2.crop_coordinates(Classwork8Config()), (50, 62))

    def test_field_must_fit_working_canvas(self):
        maze = GroundTruthLayout(15, 15, (7, 7))
        with self.assertRaisesRegex(ValueError, "outside"):
            maze.crop_coordinates(Classwork8Config())

    def test_end_to_end_save_and_preknown_alignment(self):
        maze = GroundTruthLayout(3, 4, (1, 2))
        maze.set_wall(0, 1, 2, True)
        config = Classwork8Config()
        with tempfile.TemporaryDirectory() as d:
            target, meta = maze.save(Path(d) / "ground_truth.csv", config)
            self.assertEqual((meta["crop_top"], meta["crop_left"]), (50, 62))
            self.assertTrue(target.is_file())
            self.assertTrue(target.with_suffix(".json").is_file())
            self.assertTrue(target.with_suffix(".svg").is_file())
            self.assertEqual(json.loads(target.with_suffix(".json").read_text())["start"], [1, 2])
            restored = GroundTruthLayout.from_payload(meta)
            self.assertEqual(restored.walls, maze.walls)
            gt = maze.rasterize()
            canvas = [[UNKNOWN] * 160 for _ in range(160)]
            top, left = maze.crop_coordinates(config)
            for r, row in enumerate(gt):
                canvas[top+r][left:left+len(row)] = row[:]
            aligned = [row[left:left+len(gt[0])] for row in canvas[top:top+len(gt)]]
            metrics = calculate_metrics(aligned, gt)
            self.assertEqual(metrics["map_accuracy_percent"], 100.0)
            self.assertEqual(metrics["coverage_percent"], 100.0)

    def test_rejects_mismatched_resolution_and_bad_start(self):
        maze = GroundTruthLayout(3, 3, (1, 1))
        with self.assertRaisesRegex(ValueError, "whole number"):
            maze.rasterize(.6, .07)
        with self.assertRaisesRegex(ValueError, "START"):
            maze.set_start(-1, 0)


if __name__ == "__main__":
    unittest.main()
