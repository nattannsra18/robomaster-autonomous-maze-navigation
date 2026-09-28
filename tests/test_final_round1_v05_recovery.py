"""Offline regression checks for Final Round-1 stationary recovery and map badges.

Does not connect to or command RoboMaster hardware.
"""

import unittest

from classwork8.config import Classwork8Config
from classwork8.motion_safety_v05 import side_checkpoint_decision
from classwork8.target_map_visual import target_marker_offsets, target_plot_geometry


class WallRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()

    def decide(self, walls, readings, baseline, opposite):
        cfg = self.config
        return side_checkpoint_decision(
            0, walls, readings,
            hard_stop_cm=cfg.midcell_side_hard_stop_cm,
            soft_margin_cm=cfg.midcell_side_soft_margin_cm,
            wall_max_cm=cfg.scan_side_wall_max_cm,
            gain_mps_per_cm=cfg.scan_side_kp_mps_per_cm,
            max_bias_mps=cfg.midcell_side_max_bias_mps,
            baseline_cm=baseline,
            max_baseline_drop_cm=cfg.midcell_side_max_baseline_drop_cm,
            recenter_deadband_cm=cfg.midcell_side_recenter_deadband_cm,
            allow_soft_recovery=True,
            opposite_clearance_cm=opposite,
        )

    def test_left_wall_soft_approach_biases_right(self):
        ok, reason, bias = self.decide({3}, {3: 14.0}, {3: 20.0}, 40.0)
        self.assertTrue(ok)
        self.assertEqual(reason, "MIDCELL_SOFT_WALL_RECOVERY_AWAY_LEFT")
        self.assertAlmostEqual(bias, self.config.midcell_side_max_bias_mps)

    def test_right_wall_soft_approach_biases_left(self):
        ok, reason, bias = self.decide({1}, {1: 14.0}, {1: 20.0}, 40.0)
        self.assertTrue(ok)
        self.assertEqual(reason, "MIDCELL_SOFT_WALL_RECOVERY_AWAY_RIGHT")
        self.assertLess(bias, 0.0)

    def test_no_fresh_opposite_or_insufficient_space_never_recovers(self):
        for opposite in (None, 14.0, 18.0):
            ok, reason, bias = self.decide({3}, {3: 14.0}, {3: 20.0}, opposite)
            self.assertFalse(ok)
            self.assertEqual(reason, "SIDE_RANGE_DROP_LEFT")
            self.assertEqual(bias, 0.0)

    def test_critical_distance_stops_despite_open_opposite(self):
        ok, reason, bias = self.decide({3}, {3: 9.0}, {3: 20.0}, 120.0)
        self.assertFalse(ok)
        self.assertEqual(reason, "SIDE_CRITICAL_RANGE_LEFT")
        self.assertEqual(bias, 0.0)

    def test_both_walls_approaching_stops(self):
        ok, reason, bias = self.decide(
            {1, 3}, {1: 13.0, 3: 13.0}, {1: 20.0, 3: 20.0}, 40.0
        )
        self.assertFalse(ok)
        self.assertEqual(reason, "SIDE_RANGE_DROP_BOTH")
        self.assertEqual(bias, 0.0)


class BadgeTests(unittest.TestCase):
    def test_coincident_estimates_get_different_display_offsets_only(self):
        targets = [
            {"estimated_target_xy_m": [0.5, 0.0],
             "localization_status": "NEAR_WALL_ESTIMATE",
             "range_confirmed_wall": True},
            {"estimated_target_xy_m": [0.5, 0.0],
             "localization_status": "NEAR_WALL_ESTIMATE",
             "range_confirmed_wall": True},
        ]
        offsets = target_marker_offsets(targets, 0.6)
        self.assertNotEqual(offsets[0], offsets[1])
        self.assertEqual(target_plot_geometry(targets[0], 0.6)[0], (0.5, 0.0))
        self.assertEqual(targets[0]["estimated_target_xy_m"], [0.5, 0.0])

    def test_different_positions_are_not_shifted(self):
        targets = [
            {"estimated_target_xy_m": [0.5, 0.0],
             "localization_status": "NEAR_WALL_ESTIMATE",
             "range_confirmed_wall": True},
            {"estimated_target_xy_m": [1.5, 0.0],
             "localization_status": "NEAR_WALL_ESTIMATE",
             "range_confirmed_wall": True},
        ]
        self.assertEqual(target_marker_offsets(targets, 0.6),
                         [(0.0, 0.0), (0.0, 0.0)])


if __name__ == "__main__":
    unittest.main()
