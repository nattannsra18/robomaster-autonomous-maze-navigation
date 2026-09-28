"""Offline post-scan heading alignment and gimbal/chassis yaw diagnostics."""
import inspect
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
    _fixed_heading_control_v02, _point_gimbal, V05PoseTracker,
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
        self.wheel_commands = []
        self.stop_calls = 0

    def stop(self):
        self.stop_calls += 1

    def drive_wheels(self, w1=0, w2=0, w3=0, w4=0, timeout=None):
        self.wheel_commands.append((w1, w2, w3, w4))
        # Preserve the existing command trace convention while recording
        # the new V05-specific zero-wheel stop independently.
        self.commands.append((0.0, 0.0, 0.0))
        return True

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
        self.assertEqual(chassis.wheel_commands, [(0, 0, 0, 0)])
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

    def test_gimbal_has_independent_pitch_and_yaw_time_budgets(self):
        source = inspect.getsource(_point_gimbal)
        self.assertIn("pitch_deadline = time.monotonic()", source)
        self.assertIn("yaw_deadline = started_yaw +", source)
        self.assertIn("_allow_endpoint_retry=False", source)

    def test_requested_new_defaults(self):
        config = Classwork8Config()
        config.validate()
        self.assertAlmostEqual(config.travel_speed_mps, 0.30)
        self.assertAlmostEqual(config.target_camera_pitch_deg, -20.0)
        self.assertAlmostEqual(config.gimbal_scan_pitch_deg, 0.0)
        self.assertAlmostEqual(config.gimbal_yaw_speed_dps, 170.0)
        self.assertAlmostEqual(config.gimbal_min_yaw_speed_dps, 9.0)
        self.assertAlmostEqual(config.gimbal_pitch_max_speed_dps, 38.0)
        self.assertAlmostEqual(config.gimbal_pitch_kp, 2.2)
        self.assertAlmostEqual(config.gimbal_settle_sec, 0.08)
        self.assertAlmostEqual(config.target_camera_settle_sec, 0.08)

    def test_attitude_age_detects_missing_feedback(self):
        pose = V05PoseTracker()
        self.assertIsNone(pose.attitude_age_sec())
        pose.attitude_callback((3.0, 0.0, 0.0))
        self.assertAlmostEqual(pose.get_yaw(), 3.0)
        self.assertIsNotNone(pose.attitude_age_sec())
        self.assertLess(pose.attitude_age_sec(), 0.3)
        pose.attitude_callback((float("nan"), 0.0, 0.0))
        self.assertAlmostEqual(pose.get_yaw(), 3.0)

    def test_yaw_isolation_preserves_translation_and_forces_zero_z(self):
        from classwork8.tof_camera_round1_v05 import _basic_motion_command, DIR_VEC_DRIVE
        config = Classwork8Config()
        config.yaw_isolation_mode = True
        config.heading_hold_enabled = True  # Hard zero even if misconfigured.
        for direction in range(4):
            for actual_yaw in (-8.0, 0.0, 8.0):
                with self.subTest(direction=direction, yaw=actual_yaw):
                    x, y, z, _error = _basic_motion_command(
                        config, direction, 0.0, actual_yaw
                    )
                    ux, uy = DIR_VEC_DRIVE[direction]
                    self.assertAlmostEqual(x * ux + y * uy, 0.30)
                    self.assertEqual(z, 0.0)

    def test_yaw_isolation_disables_stationary_auto_alignment(self):
        config = Classwork8Config()
        config.yaw_isolation_mode = True
        pose = FakePose(-5.0)
        chassis = FakeChassis(pose)
        ok, reason = _align_chassis_after_scan(chassis, pose, config, 0.0, None)
        self.assertTrue(ok)
        self.assertEqual(reason, "YAW_ISOLATION")
        self.assertTrue(all(x == y == z == 0.0 for x, y, z in chassis.commands))

    def test_scan_checks_free_mode_and_sends_explicit_zero(self):
        from classwork8.tof_camera_round1_v05 import run
        source = inspect.getsource(run)
        self.assertIn("FREE_MODE_FAILED", source)
        self.assertIn("SCAN_STOP_SENT_", source)
        self.assertIn("stop_chassis(chassis)", source)

    def test_each_gimbal_direction_captures_chassis_attitude(self):
        from classwork8.tof_camera_round1_v05 import _scan_four_directions
        source = inspect.getsource(_scan_four_directions)
        self.assertIn("PRE_GIMBAL_", source)
        self.assertIn("POST_GIMBAL_", source)

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
