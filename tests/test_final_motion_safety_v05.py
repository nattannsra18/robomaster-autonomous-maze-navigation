"""Offline checks for V05 lateral wall safety and the stationary checkpoint.

No RoboMaster connection, camera, chassis, or physical gimbal is used.
"""

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
    side_checkpoint_decision,
)
from classwork8 import tof_camera_round1_v05 as mission


class SideCheckLogicTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()

    def decide(self, direction, walls, readings):
        return side_checkpoint_decision(
            direction,
            walls,
            readings,
            hard_stop_cm=self.config.midcell_side_hard_stop_cm,
            soft_margin_cm=self.config.midcell_side_soft_margin_cm,
            wall_max_cm=self.config.scan_side_wall_max_cm,
            gain_mps_per_cm=self.config.scan_side_kp_mps_per_cm,
            max_bias_mps=self.config.midcell_side_max_bias_mps,
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
        ok, label, bias = self.decide(0, {3, 1}, {3: 24.0, 1: 33.0})
        self.assertTrue(ok)
        self.assertEqual(label, "MIDCELL_BETWEEN_WALLS")
        self.assertGreater(bias, 0.0)
        self.assertLessEqual(bias, 0.012)
        ok, _, reverse = self.decide(0, {3, 1}, {3: 33.0, 1: 24.0})
        self.assertTrue(ok)
        self.assertLess(reverse, 0.0)

    def test_single_close_wall_biases_away(self):
        ok, label, bias = self.decide(0, {3}, {3: 24.0})
        self.assertTrue(ok)
        self.assertEqual(label, "MIDCELL_BIAS_AWAY_LEFT")
        self.assertGreater(bias, 0.0)

        ok, label, bias = self.decide(0, {1}, {1: 24.0})
        self.assertTrue(ok)
        self.assertEqual(label, "MIDCELL_BIAS_AWAY_RIGHT")
        self.assertLess(bias, 0.0)

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
        ok, label, bias = self.decide(0, {3, 1}, {3: 21.0, 1: 35.0})
        self.assertFalse(ok)
        self.assertEqual(label, "SIDE_CLEARANCE_LOW_LEFT")
        self.assertEqual(bias, 0.0)

    def test_missing_or_far_side_data_never_produces_steering(self):
        ok, label, bias = self.decide(0, {3}, {3: None})
        self.assertTrue(ok)
        self.assertEqual(label, "SIDE_WALL_NOT_VISIBLE_NO_AUTO_STEER")
        self.assertEqual(bias, 0.0)
        ok, label, bias = self.decide(0, {3}, {3: 200.0})
        self.assertTrue(ok)
        self.assertEqual(bias, 0.0)

    def test_config_rejects_invalid_side_thresholds(self):
        self.config.midcell_side_soft_margin_cm = 20.0
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

    def run_checkpoint(self, samples):
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
            )
            return result, stop, point, fresh

    def test_stops_before_side_check_and_restores_forward(self):
        result, stop, point, fresh = self.run_checkpoint(iter((25.0, 32.0)))
        self.assertTrue(result[0])
        self.assertGreater(result[2], 0.0)
        stop.assert_called_once_with(self.chassis)
        self.assertEqual(
            [call.args[3] for call in point.call_args_list],
            [3, 1, 0],
        )
        fresh.assert_called_once()
        self.recorder.event.assert_called_once()

    def test_side_too_close_stays_stopped_without_new_forward_motion(self):
        result, stop, point, fresh = self.run_checkpoint(iter((19.0, 32.0)))
        self.assertFalse(result[0])
        self.assertEqual(result[1], "SIDE_CLEARANCE_LOW_LEFT")
        stop.assert_called_once_with(self.chassis)
        fresh.assert_not_called()


if __name__ == "__main__":
    unittest.main()
