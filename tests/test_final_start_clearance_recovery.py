"""Offline simulation of short start-side clearance recovery.

No robot connection; a fake odometry model is used for the motion checks.
"""
import types
import sys
import unittest
from unittest import mock

if "libmedia_codec" not in sys.modules:
    media = types.ModuleType("libmedia_codec")
    class H264Decoder:
        def decode(self, _data): return []
    class OpusDecoder:
        def decode(self, _data): return None
    media.H264Decoder = H264Decoder
    media.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media

from classwork8.config import Classwork8Config
from classwork8.motion_safety_v05 import side_start_recovery_preflight
from classwork8 import tof_camera_round1_v05 as mission


class PreflightTests(unittest.TestCase):
    def test_opposite_clearance_and_release_hysteresis(self):
        args = dict(hard_stop_cm=10.0, release_margin_cm=2.0,
                    opposite_min_cm=25.0, step_m=0.025)
        self.assertTrue(side_start_recovery_preflight(6.5, 30.0, **args)[0])
        self.assertEqual(
            side_start_recovery_preflight(6.5, 15.0, **args),
            (False, "RECOVERY_OPPOSITE_TOO_CLOSE"),
        )
        self.assertEqual(
            side_start_recovery_preflight(6.5, None, **args),
            (False, "RECOVERY_NO_FRESH_OPPOSITE_RANGE"),
        )
        self.assertEqual(
            side_start_recovery_preflight(6.5, 30.0, **dict(args, step_m=0.0)),
            (False, "RECOVERY_INVALID_STEP"),
        )


class FakePose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
    def get_xy(self): return self.x, self.y
    def get_yaw(self): return 0.0


class FakeChassis:
    def __init__(self, pose):
        self.pose = pose
        self.commands = []
    def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=0.2):
        self.commands.append((x, y, z))
        self.pose.x += x * 0.04
        self.pose.y += y * 0.04
        return True


class FakeSensor:
    def __init__(self, pose, opposite=40.0):
        self.pose = pose
        self.direction = 0
        self.opposite = opposite
    def reset_filters(self): pass
    def get_front_cm(self):
        if self.direction == 0:    # FRONT wall gets farther as we reverse
            return 6.5 - self.pose.x * 100.0
        if self.direction == 2:    # BACK range shrinks during reversal
            return self.opposite + self.pose.x * 100.0
        return 50.0


class FakeTracker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.direction = 0
    def get_angles(self):
        return 0.0, self.cfg.gimbal_yaw_for_direction(self.direction)
    def get_pitch(self): return 0.0


class PhysicalNudgeSimulation(unittest.TestCase):
    def setUp(self):
        self.cfg = Classwork8Config()
        self.cfg.side_start_auto_recovery_enabled = True
        self.cfg.odom_scale_x = 1.0
        self.cfg.odom_scale_y = 1.0
        self.pose = FakePose()
        self.chassis = FakeChassis(self.pose)
        self.sensor = FakeSensor(self.pose)
        self.tracker = FakeTracker(self.cfg)
        self.recorder = mock.Mock()

    def run_recovery(self):
        def point(_gimbal, _sensor, _tracker, direction, _config, _stop):
            self.sensor.direction = direction
            self.tracker.direction = direction
            return True
        with mock.patch.object(mission, "_point_gimbal", side_effect=point), \
             mock.patch.object(mission.time, "sleep", return_value=None):
            return mission._recover_critical_start_side(
                self.chassis, object(), self.pose, self.sensor,
                self.tracker, self.recorder, self.cfg,
                side=0, travel_direction=1, current_cell=(0, 0),
                start_x=0.0, start_y=0.0, start_yaw_deg=0.0,
                confirmed_side_cm=6.5, stop_event=None,
            )

    def test_confirmed_front_side_moves_back_in_short_steps_then_rechecks(self):
        ok, reason, final_cm = self.run_recovery()
        self.assertTrue(ok, reason)
        self.assertEqual(reason, "RECOVERY_CLEARANCE_CONFIRMED")
        self.assertGreaterEqual(final_cm, 12.0)
        self.assertLessEqual(abs(self.pose.x), 0.085 + 0.005)
        self.assertTrue(any(x < 0 and y == 0 for x, y, _ in self.chassis.commands))
        self.assertTrue(all(z == 0 for _, _, z in self.chassis.commands))
        self.assertEqual(self.chassis.commands[-1], (0.0, 0.0, 0.0))

    def test_blocked_opposite_never_issues_movement(self):
        self.sensor.opposite = 15.0
        ok, reason, _ = self.run_recovery()
        self.assertFalse(ok)
        self.assertEqual(reason, "RECOVERY_OPPOSITE_TOO_CLOSE")
        self.assertFalse(any(x != 0 or y != 0 for x, y, _ in self.chassis.commands))

    def test_disabled_never_issues_movement(self):
        self.cfg.side_start_auto_recovery_enabled = False
        ok, reason, _ = self.run_recovery()
        self.assertFalse(ok)
        self.assertEqual(reason, "SIDE_START_AUTORECOVERY_DISABLED")
        self.assertFalse(any(x != 0 or y != 0 for x, y, _ in self.chassis.commands))

    def test_no_opposite_sensor_never_issues_movement(self):
        self.sensor.opposite = None
        original = self.sensor.get_front_cm
        self.sensor.get_front_cm = lambda: (
            None if self.sensor.direction == 2 else original()
        )
        ok, reason, _ = self.run_recovery()
        self.assertFalse(ok)
        self.assertEqual(reason, "RECOVERY_OPPOSITE_TOF_STALE")
        self.assertFalse(any(x != 0 or y != 0 for x, y, _ in self.chassis.commands))


if __name__ == "__main__":
    unittest.main()
