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
from classwork8.motion_safety_v05 import (
    side_start_recovery_preflight,
    heading_alignment_preflight,
    supervised_recheck_pose_ok,
)
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


class HeadingAlignmentDecisionTests(unittest.TestCase):
    def rays(self, critical=10.7, opposite=30.0):
        return {0: opposite, 1: 32.0, 2: critical, 3: 35.0}

    def plan(self, yaw_error, rays):
        return heading_alignment_preflight(
            yaw_error, rays, 2,
            min_error_deg=1.5, max_step_deg=1.0,
            max_initial_error_deg=8.0,
            critical_side_min_cm=10.0,
            other_side_min_cm=22.0,
        )

    def test_right_lean_corrects_only_towards_original_heading(self):
        valid, status, correction = self.plan(4.5, self.rays())
        self.assertTrue(valid, status)
        self.assertEqual(correction, 1.0)
        valid, status, correction = self.plan(-4.5, self.rays())
        self.assertTrue(valid, status)
        self.assertEqual(correction, -1.0)

    def test_real_front_back_tight_log_forbids_rotation(self):
        valid, status, correction = self.plan(
            4.5, self.rays(critical=10.7, opposite=15.8),
        )
        self.assertFalse(valid)
        self.assertEqual(status, "HEADING_RECOVERY_NO_ROTATION_CLEARANCE_0")
        self.assertEqual(correction, 0.0)

    def test_no_critical_clearance_or_stale_ray_forbids_rotation(self):
        self.assertFalse(self.plan(4.0, self.rays(critical=9.0))[0])
        rays = self.rays()
        rays[1] = None
        self.assertEqual(self.plan(4.0, rays)[1],
                         "HEADING_RECOVERY_RANGE_UNAVAILABLE")

    def test_peak_error_is_not_live_error(self):
        valid, status, _ = self.plan(0.6, self.rays())
        self.assertFalse(valid)
        self.assertEqual(status, "HEADING_RECOVERY_ALREADY_ALIGNED")


class SupervisedRecheckTests(unittest.TestCase):
    def test_stationary_aligned_robot_can_request_fresh_scan(self):
        self.assertEqual(
            supervised_recheck_pose_ok(
                (0.6, 0.0), (0.609, -0.008), 0.7,
            ),
            (True, "SUPERVISED_FRESH_RECHECK_ALLOWED"),
        )

    def test_physically_displaced_robot_must_relocalize(self):
        self.assertEqual(
            supervised_recheck_pose_ok(
                (0.6, 0.0), (0.72, 0.0), 0.0,
            ),
            (False, "RELOCALIZATION_REQUIRED"),
        )

    def test_bad_heading_and_missing_pose_block_resumption(self):
        self.assertEqual(
            supervised_recheck_pose_ok(
                (0.6, 0.0), (0.6, 0.0), 5.0,
            ),
            (False, "SUPERVISED_HEADING_NOT_ALIGNED"),
        )
        self.assertEqual(
            supervised_recheck_pose_ok(None, (0.6, 0.0), 0.0),
            (False, "SUPERVISED_POSE_UNAVAILABLE"),
        )


class FakePose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
    def get_xy(self): return self.x, self.y
    def get_yaw(self): return getattr(self, "yaw", 0.0)


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

    def run_recovery(self, confirmed_side_cm=6.5):
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
                confirmed_side_cm=confirmed_side_cm, stop_event=None,
            )

    def test_transient_heading_spike_stops_rechecks_and_retries(self):
        # First translation makes a fake yaw spike. Stopping the motors makes
        # feedback settle; the next pulse must repeat the opposite ToF gate.
        self.pose.yaw = 0.0
        original_drive = self.chassis.drive_speed
        injected = {"value": False}
        count = {"translations": 0}

        def transient_drive(x=0.0, y=0.0, z=0.0, timeout=0.2):
            result = original_drive(x=x, y=y, z=z, timeout=timeout)
            if x < 0:
                count["translations"] += 1
                if not injected["value"]:
                    injected["value"] = True
                    self.pose.yaw = 5.2
            elif injected["value"] and x == y == z == 0.0:
                self.pose.yaw = 0.0
            return result

        self.chassis.drive_speed = transient_drive
        self.sensor.get_front_cm = lambda: (
            8.0 - self.pose.x * 100.0 if self.sensor.direction == 0
            else 40.0 + self.pose.x * 100.0
        )
        ok, reason, final_cm = self.run_recovery(confirmed_side_cm=8.0)
        self.assertTrue(ok, reason)
        self.assertEqual(reason, "RECOVERY_CLEARANCE_CONFIRMED")
        self.assertGreaterEqual(final_cm, 12.0)
        self.assertTrue(injected["value"])
        self.assertGreater(count["translations"], 2)
        self.assertTrue(all(z == 0 for _, _, z in self.chassis.commands))
        events = [
            call.args[2] for call in self.recorder.event.call_args_list
            if len(call.args) > 2 and call.args[1] == "CLEARANCE_HEADING"
        ]
        self.assertIn("RECOVERY_HEADING_SETTLED", events)

    def test_persistent_heading_error_never_keeps_driving(self):
        self.pose.yaw = 0.0
        original_drive = self.chassis.drive_speed
        injected = {"value": False}
        count = {"translations": 0}

        def persistent_drive(x=0.0, y=0.0, z=0.0, timeout=0.2):
            result = original_drive(x=x, y=y, z=z, timeout=timeout)
            if x < 0:
                count["translations"] += 1
                if not injected["value"]:
                    injected["value"] = True
                    self.pose.yaw = 5.2
            return result

        self.chassis.drive_speed = persistent_drive
        ok, reason, _ = self.run_recovery()
        self.assertFalse(ok)
        self.assertEqual(reason, "RECOVERY_HEADING_PERSISTENT")
        self.assertEqual(count["translations"], 1)
        self.assertEqual(self.chassis.commands[-1], (0.0, 0.0, 0.0))

    def test_preexisting_large_yaw_does_not_start_escape(self):
        self.pose.yaw = 5.0
        ok, reason, _ = self.run_recovery()
        self.assertFalse(ok)
        self.assertEqual(reason, "RECOVERY_HEADING_PRE_PULSE_UNSAFE")
        self.assertFalse(any(
            x != 0 or y != 0 for x, y, _ in self.chassis.commands
        ))

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
