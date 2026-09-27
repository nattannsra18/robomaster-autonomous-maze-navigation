"""Offline regression test of V05 gimbal pitch/yaw feedback control.

Does not connect to RoboMaster or command a physical gimbal.
"""

import sys
import types
import unittest


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
from classwork8.tof_camera_round1_v05 import GimbalTracker, _point_gimbal


class FakeSensors:
    def __init__(self):
        self.resets = 0

    def reset_filters(self):
        self.resets += 1


class FakeGimbal:
    def __init__(self, tracker):
        self.tracker = tracker
        self.commands = []

    def drive_speed(self, pitch_speed=0.0, yaw_speed=0.0):
        self.commands.append((float(pitch_speed), float(yaw_speed)))
        with self.tracker._lock:
            # Simulate a very small bounded motion during each 30 ms tick.
            # Missing telemetry is not magically restored by a zero command.
            if self.tracker.pitch is not None:
                self.tracker.pitch += float(pitch_speed) * 0.03
            if self.tracker.yaw is not None:
                self.tracker.yaw += float(yaw_speed) * 0.03
        return True


class PitchHoldTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.config.gimbal_turn_timeout_sec = 5.0
        self.config.gimbal_stable_samples = 2
        self.config.gimbal_settle_sec = 0.01
        self.tracker = GimbalTracker()
        self.sensors = FakeSensors()
        self.gimbal = FakeGimbal(self.tracker)

    def test_right_scan_relevels_tilted_pitch(self):
        # Simulate the field case: yaw is already RIGHT but pitch is +12 deg.
        self.tracker.pitch = 12.0
        self.tracker.yaw = 90.0

        success = _point_gimbal(
            self.gimbal, self.sensors, self.tracker, 1, self.config, None
        )
        self.assertTrue(success)
        self.assertLessEqual(
            abs(self.tracker.get_pitch() - self.config.gimbal_scan_pitch_deg),
            self.config.gimbal_pitch_tolerance_deg,
        )
        self.assertEqual(self.sensors.resets, 1)
        self.assertTrue(any(abs(pitch) > 0.0 for pitch, _yaw in self.gimbal.commands))

    def test_previous_plus_1_7_degree_end_point_is_not_accepted(self):
        # This was accepted by the previous +/-2 degree threshold. The
        # current tolerance should actively correct instead of reporting OK.
        self.tracker.pitch = 1.7
        self.tracker.yaw = 90.0

        success = _point_gimbal(
            self.gimbal, self.sensors, self.tracker, 1, self.config, None
        )
        self.assertTrue(success)
        self.assertLessEqual(
            abs(self.tracker.get_pitch() - self.config.gimbal_scan_pitch_deg),
            self.config.gimbal_pitch_tolerance_deg,
        )
        self.assertTrue(
            any(abs(pitch) > 0.0 for pitch, _yaw in self.gimbal.commands)
        )

    def test_yaw_sweep_never_commands_both_axes_at_once(self):
        self.config.gimbal_turn_timeout_sec = 10.0
        self.tracker.pitch = 0.0
        self.tracker.yaw = 0.0

        success = _point_gimbal(
            self.gimbal, self.sensors, self.tracker, 1, self.config, None
        )
        self.assertTrue(success)
        self.assertEqual(self.sensors.resets, 1)
        self.assertTrue(
            any(abs(yaw) > 0.0 for _pitch, yaw in self.gimbal.commands)
        )
        self.assertTrue(all(
            abs(pitch) == 0.0 or abs(yaw) == 0.0
            for pitch, yaw in self.gimbal.commands
        ))

    def test_transient_pitch_during_yaw_is_relevelled_before_tof(self):
        self.config.gimbal_turn_timeout_sec = 10.0
        self.tracker.pitch = 0.0
        self.tracker.yaw = 0.0

        class CoupledGimbal(FakeGimbal):
            def drive_speed(self, pitch_speed=0.0, yaw_speed=0.0):
                result = super().drive_speed(
                    pitch_speed=pitch_speed, yaw_speed=yaw_speed
                )
                if abs(yaw_speed) > 0.0:
                    with self.tracker._lock:
                        self.tracker.pitch = 7.0
                return result

        coupled = CoupledGimbal(self.tracker)
        success = _point_gimbal(
            coupled, self.sensors, self.tracker, 1, self.config, None
        )
        self.assertTrue(success)
        self.assertEqual(self.sensors.resets, 1)
        self.assertLessEqual(
            abs(self.tracker.pitch - self.config.gimbal_scan_pitch_deg),
            self.config.gimbal_pitch_tolerance_deg,
        )
        self.assertTrue(all(
            abs(pitch) == 0.0 or abs(yaw) == 0.0
            for pitch, yaw in coupled.commands
        ))

    def test_pitch_that_cannot_be_relevelled_still_blocks_tof(self):
        self.config.gimbal_turn_timeout_sec = 0.18
        self.tracker.pitch = 7.0
        self.tracker.yaw = 90.0

        class StuckPitchGimbal(FakeGimbal):
            def drive_speed(self, pitch_speed=0.0, yaw_speed=0.0):
                self.commands.append((float(pitch_speed), float(yaw_speed)))
                return True

        stuck = StuckPitchGimbal(self.tracker)
        success = _point_gimbal(
            stuck, self.sensors, self.tracker, 1, self.config, None
        )
        self.assertFalse(success)
        self.assertEqual(self.sensors.resets, 0)

    def test_no_pitch_feedback_fails_safely(self):
        self.config.gimbal_turn_timeout_sec = 0.12
        self.tracker.pitch = None
        self.tracker.yaw = 90.0

        success = _point_gimbal(
            self.gimbal, self.sensors, self.tracker, 1, self.config, None
        )
        self.assertFalse(success)
        self.assertEqual(self.sensors.resets, 0)


if __name__ == "__main__":
    unittest.main()
