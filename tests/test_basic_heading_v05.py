"""Offline post-scan heading alignment and gimbal/chassis yaw diagnostics."""
import sys
import types
import unittest

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
from classwork8.tof_camera_round1_v05 import (
    GimbalTracker, _align_chassis_after_scan, _heading_error,
    _fixed_heading_control_v02,
)


class FakePose:
    def __init__(self, yaw):
        self.yaw = yaw

    def get_yaw(self):
        return self.yaw


class FakeChassis:
    def __init__(self, pose, response_sign=1):
        self.pose = pose
        self.response_sign = response_sign
        self.commands = []

    def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None):
        self.commands.append((float(x), float(y), float(z)))
        # Approximate 50 ms of angular response per controller iteration.
        self.pose.yaw += self.response_sign * float(z) * 0.05
        return True


class HeadingTests(unittest.TestCase):
    def test_shortest_angle_wrap(self):
        self.assertAlmostEqual(_heading_error(179.0, -179.0), -2.0)
        self.assertAlmostEqual(_heading_error(-179.0, 179.0), 2.0)

    def test_gimbal_relative_yaw_and_ground_yaw_are_separate(self):
        tracker = GimbalTracker()
        tracker.callback((0.0, 90.0, 0.0, 94.0))
        self.assertEqual(tracker.get_yaws(), (90.0, 94.0))
        self.assertEqual(tracker.get_yaw(), 90.0)

    def test_small_error_never_commands_stationary_rotation(self):
        config = Classwork8Config()
        pose = FakePose(0.5)
        chassis = FakeChassis(pose)
        ok, reason = _align_chassis_after_scan(chassis, pose, config, 0.0, None)
        self.assertTrue(ok)
        self.assertEqual(reason, "ALREADY_ALIGNED")
        self.assertTrue(all(abs(z) == 0.0 for _x, _y, z in chassis.commands))

    def test_correction_converges_when_sign_matches(self):
        config = Classwork8Config()
        pose = FakePose(-4.0)
        chassis = FakeChassis(pose, response_sign=1)
        ok, reason = _align_chassis_after_scan(chassis, pose, config, 0.0, None)
        self.assertTrue(ok, reason)
        self.assertLessEqual(abs(pose.yaw), config.heading_align_tolerance_deg)
        self.assertTrue(any(z > 0.0 for _x, _y, z in chassis.commands))
        self.assertTrue(all(x == y == 0.0 for x, y, _z in chassis.commands))

    def test_wrong_z_sign_aborts_not_spins_forever(self):
        config = Classwork8Config()
        pose = FakePose(-4.0)
        chassis = FakeChassis(pose, response_sign=-1)
        ok, reason = _align_chassis_after_scan(chassis, pose, config, 0.0, None)
        self.assertFalse(ok)
        self.assertEqual(reason, "HEADING_SIGN_MISMATCH")
        self.assertEqual(chassis.commands[-1], (0.0, 0.0, 0.0))

    def test_heading_steering_does_not_reduce_forward_speed(self):
        config = Classwork8Config()
        x, y, z, _mode, _error = _fixed_heading_control_v02(
            config, 0.0, 4.0, config.travel_speed_mps, 0.0, "BASIC_MOVE"
        )
        self.assertAlmostEqual(x, 0.30)
        self.assertEqual(y, 0.0)
        self.assertNotEqual(z, 0.0)


if __name__ == "__main__":
    unittest.main()
