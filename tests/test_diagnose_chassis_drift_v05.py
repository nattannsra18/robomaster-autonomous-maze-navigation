"""Static safety regression tests for the standalone stationary drift probe."""
import ast
import threading
import time
import unittest
from pathlib import Path


SOURCE = (Path(__file__).resolve().parents[1] / "diagnose_chassis_drift_v05.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)


class DriftDiagnosticTests(unittest.TestCase):
    def test_all_chassis_speed_commands_are_zero(self):
        commands = [
            node for node in ast.walk(TREE)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "drive_speed"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "chassis"
        ]
        self.assertTrue(commands)
        for call in commands:
            values = {keyword.arg: ast.literal_eval(keyword.value)
                      for keyword in call.keywords}
            self.assertEqual(values["x"], 0.0)
            self.assertEqual(values["y"], 0.0)
            self.assertEqual(values["z"], 0.0)
        self.assertNotIn("drive_wheels(", SOURCE)
        self.assertNotIn("chassis.move(", SOURCE)

    def test_gimbal_motion_repeats_commands_and_uses_feedback(self):
        self.assertIn('"--with-gimbal"', SOURCE)
        self.assertIn('if stop.is_set() or not args.with_gimbal:', SOURCE)
        self.assertIn("gimbal.resume()", SOURCE)
        self.assertIn('speed = +30.0 if initial < -5.0', SOURCE)
        self.assertIn('stop.wait(0.03)', SOURCE)
        self.assertIn('if delta >= 12.0:', SOURCE)
        self.assertIn('if now - began > 0.8 and delta < 1.0:', SOURCE)
        self.assertIn('if result is False:', SOURCE)
        self.assertNotIn('GIMBAL_LEFT_PULSE', SOURCE)
        self.assertNotIn('GIMBAL_RIGHT_PULSE', SOURCE)

    def test_extreme_negative_angle_commands_inward_positive_yaw(self):
        # Actual previous run reported -269 deg. Positive command moves
        # inward, not farther into the negative limit.
        for initial in (-269.0, -250.0, -90.0):
            speed = +30.0 if initial < -5.0 else -30.0 if initial > 5.0 else +30.0
            self.assertGreater(speed, 0.0)
        self.assertIn('if not gimbal_ready:', SOURCE)
        self.assertIn('if command_ok is False:', SOURCE)
        self.assertNotIn('if not command_ok:', SOURCE)
        self.assertIn('None is normal for SDK async send', SOURCE)

    def test_probe_with_none_async_return_repeats_until_feedback_moves(self):
        from diagnose_chassis_drift_v05 import _gimbal_closed_loop_probe

        class Gimbal:
            yaw = -269.0

            def __init__(self):
                self.nonzero_commands = 0

            def drive_speed(self, pitch_speed=0.0, yaw_speed=0.0):
                if yaw_speed:
                    self.nonzero_commands += 1
                    self.yaw += 0.8
                return None  # Official SDK can return None for async commands.

        class Chassis:
            def __init__(self):
                self.commands = []

            def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None):
                self.commands.append((x, y, z))
                return True

        class Telemetry:
            def __init__(self, gimbal):
                self.gimbal = gimbal
                self.stamps = {}

            def snapshot(self):
                self.stamps["gimbal_relative"] = time.monotonic()
                return {"gimbal_relative": self.gimbal.yaw,
                        "yaw": -25.0, "esc": (0.0, 0.0, 0.0, 0.0)}

        gimbal = Gimbal()
        chassis = Chassis()
        stop = threading.Event()
        ok = _gimbal_closed_loop_probe(
            chassis, gimbal, Telemetry(gimbal), time.monotonic(), stop
        )
        self.assertTrue(ok)
        self.assertGreaterEqual(gimbal.nonzero_commands, 10)
        self.assertTrue(all(x == y == z == 0.0 for x, y, z in chassis.commands))

    def test_probe_stops_when_yaw_feedback_never_moves(self):
        from diagnose_chassis_drift_v05 import _gimbal_closed_loop_probe

        class Gimbal:
            def __init__(self):
                self.nonzero_commands = 0
                self.stopped = False

            def drive_speed(self, pitch_speed=0.0, yaw_speed=0.0):
                if yaw_speed:
                    self.nonzero_commands += 1
                else:
                    self.stopped = True
                return None

        class Chassis:
            def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None):
                self_zero = (x, y, z)
                assert self_zero == (0.0, 0.0, 0.0)
                return True

        class Telemetry:
            stamps = {}

            def snapshot(self):
                self.stamps["gimbal_relative"] = time.monotonic()
                return {"gimbal_relative": -269.0, "yaw": -25.0,
                        "esc": (0.0, 0.0, 0.0, 0.0)}

        gimbal = Gimbal()
        stop = threading.Event()
        ok = _gimbal_closed_loop_probe(
            Chassis(), gimbal, Telemetry(), time.monotonic(), stop
        )
        self.assertFalse(ok)
        self.assertGreater(gimbal.nonzero_commands, 1)
        self.assertTrue(gimbal.stopped)

    def test_esc_mode_and_imu_are_logged(self):
        for token in ('sub_esc(', 'sub_imu(', 'sub_mode(', 'stick_overlay(0)',
                      'CONNECTED_NO_COMMAND', 'FREE_WITH_ZERO_CHASSIS',
                      'OVERLAY_OFF_WITH_ZERO_CHASSIS', 'esc_rpm=', 'gyro_z='):
            with self.subTest(token=token):
                self.assertIn(token, SOURCE)


if __name__ == "__main__":
    unittest.main()
