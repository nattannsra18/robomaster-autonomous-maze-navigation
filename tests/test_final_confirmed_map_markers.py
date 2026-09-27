"""The logical map must show only confirmed signs, not tentative sightings.

PENDING_RECHECK must remain available in targets.json and revisit logic.
These pure rendering-data tests never need RoboMaster hardware or Tk.
"""
import unittest

from classwork8.target_map_visual import (
    confirmed_map_targets,
    target_marker_offsets,
    target_plot_geometry,
)


class ConfirmedOnlyMapTests(unittest.TestCase):
    def setUp(self):
        self.confirmed_near = {
            "target_id": "T01", "color": "green", "shape": "square",
            "status": "POSITION_CANDIDATE", "localization_status": "NEAR_WALL_ESTIMATE",
            "range_confirmed_wall": True, "estimated_target_xy_m": [0.5, 0.0],
        }
        self.confirmed_distant = {
            "target_id": "T02", "color": "red", "shape": "circle",
            "status": "SIGHTING_ONLY", "localization_status": "SIGHTING_ONLY",
            "range_confirmed_wall": False, "observation_cells": [[0, 0]],
            "view_directions": [0], "sighting_cell_hint": [5, 0],
        }
        self.pending = {
            "target_id": "P01", "color": "green", "shape": "square",
            "status": "PENDING_RECHECK", "confirmed": False,
            "observation_cells": [[0, 0]], "view_directions": [1],
        }

    def test_only_confirmed_signs_are_displayed(self):
        source = [self.confirmed_near, self.pending, self.confirmed_distant]
        displayed = confirmed_map_targets(source)
        self.assertEqual([item["target_id"] for item in displayed], ["T01", "T02"])
        self.assertEqual(len(source), 3)  # no mutation / no data loss

    def test_pending_legacy_entry_is_hidden_even_without_status(self):
        self.assertEqual(confirmed_map_targets([dict(self.pending, status=None)]), [])

    def test_both_confirmed_localized_and_distant_signs_remain_plottable(self):
        displayed = confirmed_map_targets(
            [self.confirmed_near, self.pending, self.confirmed_distant]
        )
        self.assertEqual(target_plot_geometry(displayed[0], 0.6)[0], (0.5, 0.0))
        hint, origin, sighting_only = target_plot_geometry(displayed[1], 0.6)
        self.assertTrue(sighting_only)
        self.assertIsNotNone(hint)
        self.assertIsNotNone(origin)
        self.assertEqual(len(target_marker_offsets(displayed, 0.6)), 2)

    def test_none_and_empty_are_safe(self):
        self.assertEqual(confirmed_map_targets(None), [])
        self.assertEqual(confirmed_map_targets([]), [])


if __name__ == "__main__":
    unittest.main()
