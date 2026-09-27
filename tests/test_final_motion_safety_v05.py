"""Offline checks for V05 lateral wall safety and the stationary checkpoint.

No RoboMaster connection, camera, chassis, or physical gimbal is used.
"""

import inspect
import sys
import types
import unittest
from unittest import mock


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
from classwork8.motion_safety_v05 import (
    adjacent_wall_sides,
    bound_travel_lateral,
    critical_start_side_recheck,
    side_checkpoint_decision,
)
from classwork8 import tof_camera_round1_v05 as mission


class SideCheckLogicTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()

    def decide(self, direction, walls, readings, baseline=None):
        return side_checkpoint_decision(
            direction,
            walls,
            readings,
            hard_stop_cm=self.config.midcell_side_hard_stop_cm,
            soft_margin_cm=self.config.midcell_side_soft_margin_cm,
            wall_max_cm=self.config.scan_side_wall_max_cm,
            gain_mps_per_cm=self.config.scan_side_kp_mps_per_cm,
            max_bias_mps=self.config.midcell_side_max_bias_mps,
            baseline_cm=baseline,
            max_baseline_drop_cm=self.config.midcell_side_max_baseline_drop_cm,
            recenter_deadband_cm=self.config.midcell_side_recenter_deadband_cm,
        )

    def test_only_confirmed_wall_sides_need_stationary_scan(self):
        edges = {
            (0, 0, 3): "WALL",
            (1, 0, 1): "OPEN",
        }
        self.assertEqual(
            adjacent_wall_sides(0, (0, 0), (1, 0), edges), {3}
        )
        self.assertEqual(
            adjacent_wall_sides(1, (0, 0), (0, -1), {}), set()
        )

    def test_two_walls_use_correct_lateral_sign(self):
        # FRONT: LEFT is 3, RIGHT is 1; left is closer -> move right.
        ok, label, bias = self.decide(
            0, {3, 1}, {3: 24.0, 1: 33.0},
            baseline={3: 27.0, 1: 32.0},
        )
        self.assertTrue(ok)
        self.assertEqual(label, "MIDCELL_RESTORE_SIDE_BALANCE")
        self.assertGreater(bias, 0.0)
        self.assertLessEqual(bias, 0.012)
        ok, _, reverse = self.decide(
            0, {3, 1}, {3: 33.0, 1: 24.0},
            baseline={3: 32.0, 1: 27.0},
        )
        self.assertTrue(ok)
        self.assertLess(reverse, 0.0)

    def test_single_close_wall_biases_away(self):
        ok, label, bias = self.decide(
            0, {3}, {3: 24.0}, baseline={3: 27.0}
        )
        self.assertTrue(ok)
        self.assertEqual(label, "MIDCELL_BIAS_AWAY_LEFT")
        self.assertGreater(bias, 0.0)

        ok, label, bias = self.decide(
            0, {1}, {1: 24.0}, baseline={1: 27.0}
        )
        self.assertTrue(ok)
        self.assertEqual(label, "MIDCELL_BIAS_AWAY_RIGHT")
        self.assertLess(bias, 0.0)

    def test_single_wall_normal_baseline_does_not_blindly_strafe(self):
        correction, reason = mission._scan_side_guidance_v02(
            0, {3: 162.8, 1: 14.5}, self.config
        )
        self.assertEqual(correction, 0.0)
        self.assertEqual(reason, "SINGLE_WALL_BASELINE_ONLY")

    def test_recheck_confirmed_6_5cm_still_stops(self):
        cleared, reason, value = critical_start_side_recheck(
            (6.4, 6.6),
            hard_stop_cm=10.0,
            release_margin_cm=2.0,
            max_spread_cm=2.0,
        )
        self.assertFalse(cleared)
        self.assertEqual(reason, "SIDE_START_CRITICAL_CONFIRMED")
        self.assertAlmostEqual(value, 6.5)

    def test_recheck_can_clear_one_spurious_short_side_echo(self):
        cleared, reason, value = critical_start_side_recheck(
            (14.5, 14.7),
            hard_stop_cm=10.0,
            release_margin_cm=2.0,
            max_spread_cm=2.0,
        )
        self.assertTrue(cleared)
        self.assertEqual(reason, "SIDE_START_TRANSIENT_CLEARED")
        self.assertAlmostEqual(value, 14.6)

    def test_recheck_single_high_reflection_cannot_clear_stop(self):
        cleared, reason, value = critical_start_side_recheck(
            (6.5, 14.7),
            hard_stop_cm=10.0,
            release_margin_cm=2.0,
            max_spread_cm=2.0,
        )
        self.assertFalse(cleared)
        self.assertEqual(reason, "SIDE_START_RECHECK_INCONSISTENT")
        self.assertIsNone(value)

    def test_recheck_missing_to_f_cannot_clear_stop(self):
        cleared, reason, value = critical_start_side_recheck(
            (None, 15.0),
            hard_stop_cm=10.0,
            release_margin_cm=2.0,
            max_spread_cm=2.0,
        )
        self.assertFalse(cleared)
        self.assertEqual(reason, "SIDE_START_RECHECK_NO_FRESH_TOF")
        self.assertIsNone(value)

    def test_real_field_14_5_to_14_7_cm_is_stable(self):
        # 2026-09-27 log: start RIGHT 14.5, halfway RIGHT 14.7 cm.
        # It must not be treated as collision solely because it is below
        # the obsolete 22 cm stop threshold.
        ok, label, bias = self.decide(
            0, {1}, {1: 14.7}, baseline={1: 14.5}
        )
        self.assertTrue(ok)
        self.assertEqual(label, "MIDCELL_SIDE_BASELINE_STABLE")
        self.assertEqual(bias, 0.0)

    def test_real_field_baseline_drop_still_stops(self):
        ok, label, bias = self.decide(
            0, {1}, {1: 10.4}, baseline={1: 14.5}
        )
        self.assertFalse(ok)
        self.assertEqual(label, "SIDE_RANGE_DROP_RIGHT")
        self.assertEqual(bias, 0.0)

    def test_without_baseline_no_lateral_autosteer(self):
        ok, label, bias = self.decide(0, {1}, {1: 14.7})
        self.assertTrue(ok)
        self.assertEqual(label, "MIDCELL_NO_BASELINE_NO_AUTO_STEER")
        self.assertEqual(bias, 0.0)

    def test_combined_lateral_command_is_bounded_without_changing_forward(self):
        # FRONT travel: chassis +Y moves right.
        x, y = bound_travel_lateral(0.10, 0.07, (0.0, 1.0), 0.028)
        self.assertAlmostEqual(x, 0.10)
        self.assertAlmostEqual(y, 0.028)
        # RIGHT travel: right-relative is backward (-X).
        x, y = bound_travel_lateral(-0.07, 0.10, (-1.0, 0.0), 0.028)
        self.assertAlmostEqual(x, -0.028)
        self.assertAlmostEqual(y, 0.10)

    def test_dangerous_side_range_forces_stop_not_correction(self):
        ok, label, bias = self.decide(0, {3, 1}, {3: 9.0, 1: 35.0})
        self.assertFalse(ok)
        self.assertEqual(label, "SIDE_CRITICAL_RANGE_LEFT")
        self.assertEqual(bias, 0.0)

    def test_missing_or_far_side_data_never_produces_steering(self):
        ok, label, bias = self.decide(0, {3}, {3: None})
        self.assertTrue(ok)
        self.assertEqual(label, "SIDE_WALL_NOT_VISIBLE_NO_AUTO_STEER")
        self.assertEqual(bias, 0.0)
        ok, label, bias = self.decide(0, {3}, {3: 200.0})
        self.assertTrue(ok)
        self.assertEqual(bias, 0.0)

    def test_checkpoint_code_is_only_inside_drive_not_four_way_scan(self):
        scan_source = inspect.getsource(mission._scan_four_directions)
        drive_source = inspect.getsource(mission._drive_one_cell)
        # Regression: the previous commit inserted the motion checkpoint into
        # the stationary map scanner and crashed with NameError before a move.
        self.assertNotIn("checkpoint_enabled", scan_source)
        self.assertNotIn("checkpoint_done", scan_source)
        self.assertIn("checkpoint_enabled", drive_source)
        self.assertIn("_midcell_wall_checkpoint(", drive_source)

    def test_new_default_is_fast_on_open_routes_but_limited_near_walls(self):
        self.assertAlmostEqual(self.config.travel_speed_mps, 0.20)
        self.assertAlmostEqual(
            self.config.motion_wall_adjacent_speed_cap_mps, 0.12
        )
        self.config.validate()
        self.config.travel_speed_mps = 0.08
        # Lowering travel speed during tuning must remain valid.
        self.config.validate()

    def test_config_rejects_invalid_side_thresholds(self):
        self.config.midcell_side_soft_margin_cm = 9.0
        with self.assertRaises(ValueError):
            self.config.validate()

    def test_config_rejects_invalid_cross_track_guard(self):
        self.config.motion_cross_track_abort_m = 0.025
        with self.assertRaises(ValueError):
            self.config.validate()


class StationarySideCheckpointTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.chassis = object()
        self.gimbal = object()
        self.sensor = object()
        self.tracker = mock.Mock()
        self.tracker.get_pitch.return_value = 0.0
        self.recorder = mock.Mock()

    def run_checkpoint(self, samples, baseline=None):
        def get_tof(_sensors, _config, _stop):
            return next(samples)

        with mock.patch.object(mission, "stop_chassis") as stop, \
             mock.patch.object(mission, "_point_gimbal", return_value=True) as point, \
             mock.patch.object(mission, "_sample_tof", side_effect=get_tof), \
             mock.patch.object(
                 mission, "_wait_for_move_tof_v03", return_value=105.0
             ) as fresh:
            result = mission._midcell_wall_checkpoint(
                self.chassis,
                self.gimbal,
                self.sensor,
                self.tracker,
                self.recorder,
                self.config,
                direction=0,
                wall_sides={3, 1},
                current_cell=(0, 0),
                target_cell=(1, 0),
                progress_m=0.30,
                stop_event=None,
                baseline_ranges=baseline,
            )
            return result, stop, point, fresh

    def test_stops_before_side_check_and_restores_forward(self):
        result, stop, point, fresh = self.run_checkpoint(
            iter((25.0, 32.0)), baseline={3: 28.0, 1: 32.0}
        )
        self.assertTrue(result[0])
        self.assertGreater(result[2], 0.0)
        stop.assert_called_once_with(self.chassis)
        self.assertEqual(
            [call.args[3] for call in point.call_args_list],
            [3, 1, 0],
        )
        fresh.assert_called_once()
        self.recorder.event.assert_called_once()

    def test_real_field_stable_side_range_resumes_forward_tof(self):
        # Only RIGHT is mapped as a wall for this leg.
        with mock.patch.object(mission, "stop_chassis") as stop, \
             mock.patch.object(mission, "_point_gimbal", return_value=True) as point, \
             mock.patch.object(mission, "_sample_tof", return_value=14.7), \
             mock.patch.object(
                 mission, "_wait_for_move_tof_v03", return_value=90.0
             ) as fresh:
            ok, label, bias = mission._midcell_wall_checkpoint(
                self.chassis, self.gimbal, self.sensor, self.tracker,
                self.recorder, self.config,
                direction=0, wall_sides={1},
                current_cell=(0, 0), target_cell=(1, 0),
                progress_m=0.29, stop_event=None,
                baseline_ranges={1: 14.5},
            )
            self.assertTrue(ok)
            self.assertEqual(label, "MIDCELL_SIDE_BASELINE_STABLE")
            self.assertEqual(bias, 0.0)
            stop.assert_called_once_with(self.chassis)
            self.assertEqual(
                [call.args[3] for call in point.call_args_list], [1, 0]
            )
            fresh.assert_called_once()

    def test_side_too_close_stays_stopped_without_new_forward_motion(self):
        result, stop, point, fresh = self.run_checkpoint(iter((9.0, 32.0)))
        self.assertFalse(result[0])
        self.assertEqual(result[1], "SIDE_CRITICAL_RANGE_LEFT")
        stop.assert_called_once_with(self.chassis)
        fresh.assert_not_called()


if __name__ == "__main__":
    unittest.main()
