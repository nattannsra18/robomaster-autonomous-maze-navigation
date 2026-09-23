import unittest

from classwork8.occupancy_grid import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    OccupancyGrid,
    calculate_accuracy,
    calculate_coverage,
)


class OccupancyGridTests(unittest.TestCase):
    def test_ray_marks_free_and_wall(self):
        grid = OccupancyGrid(2.0, 2.0, 0.1)
        for _ in range(2):
            grid.update_ray(
                0.0, 0.0, 0.0, 0.5,
                max_range_m=1.0,
                hit=True,
            )
        start = grid.world_to_cell(0.0, 0.0)
        wall = grid.world_to_cell(0.5, 0.0)
        self.assertIsNotNone(start)
        self.assertIsNotNone(wall)
        self.assertEqual(grid.state_at(*start), FREE)
        self.assertEqual(grid.state_at(*wall), OCCUPIED)

    def test_max_range_does_not_create_fake_wall(self):
        grid = OccupancyGrid(2.0, 2.0, 0.1)
        grid.update_ray(
            0.0, 0.0, 0.0, 1.0,
            max_range_m=1.0,
            hit=False,
        )
        endpoint = grid.world_to_cell(0.9, 0.0)
        self.assertIsNotNone(endpoint)
        self.assertNotEqual(
            grid.state_at(*endpoint), OCCUPIED
        )

    def test_unknown_cells_remain_unknown(self):
        grid = OccupancyGrid(2.0, 2.0, 0.1)
        far = grid.world_to_cell(-0.8, -0.8)
        self.assertEqual(grid.state_at(*far), UNKNOWN)

    def test_coverage_formula(self):
        predicted = [[0, 100], [-1, -1]]
        self.assertAlmostEqual(
            calculate_coverage(predicted), 50.0
        )

    def test_accuracy_formula(self):
        predicted = [[0, 100], [-1, 0]]
        truth = [[0, 100], [100, 0]]
        self.assertAlmostEqual(
            calculate_accuracy(predicted, truth), 75.0
        )


if __name__ == "__main__":
    unittest.main()
