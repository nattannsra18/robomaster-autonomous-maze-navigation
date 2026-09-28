"""Static safety regression tests for the standalone stationary drift probe."""
import ast
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

    def test_gimbal_motion_is_opt_in_inward_and_feedback_verified(self):
        self.assertIn('"--with-gimbal"', SOURCE)
        self.assertIn('if stop.is_set() or not args.with_gimbal:', SOURCE)
        self.assertIn("gimbal.resume()", SOURCE)
        self.assertIn('speed = +30.0 if initial < -5.0', SOURCE)
        self.assertIn('_stage("GIMBAL_INWARD_PULSE", 1.2', SOURCE)
        self.assertIn('delta * speed <= 3.0', SOURCE)
        self.assertIn('"[GIMBAL_FAIL] no verified inward gimbal motion', SOURCE)
        self.assertNotIn('GIMBAL_LEFT_PULSE', SOURCE)
        self.assertNotIn('GIMBAL_RIGHT_PULSE', SOURCE)

    def test_extreme_negative_angle_commands_inward_positive_yaw(self):
        # Actual previous run reported -269 deg. Positive command moves
        # inward, not farther into the negative limit.
        for initial in (-269.0, -250.0, -90.0):
            speed = +30.0 if initial < -5.0 else -30.0 if initial > 5.0 else +30.0
            self.assertGreater(speed, 0.0)
        self.assertIn('if not gimbal_ready:', SOURCE)
        self.assertIn('if not command_ok:', SOURCE)

    def test_esc_mode_and_imu_are_logged(self):
        for token in ('sub_esc(', 'sub_imu(', 'sub_mode(', 'stick_overlay(0)',
                      'CONNECTED_NO_COMMAND', 'FREE_WITH_ZERO_CHASSIS',
                      'OVERLAY_OFF_WITH_ZERO_CHASSIS', 'esc_rpm=', 'gyro_z='):
            with self.subTest(token=token):
                self.assertIn(token, SOURCE)


if __name__ == "__main__":
    unittest.main()
