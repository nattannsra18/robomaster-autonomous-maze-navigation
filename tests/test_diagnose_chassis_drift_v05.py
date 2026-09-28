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

    def test_gimbal_motion_is_opt_in_and_short(self):
        self.assertIn('"--with-gimbal"', SOURCE)
        self.assertIn('if stop.is_set() or not args.with_gimbal:', SOURCE)
        self.assertIn('("GIMBAL_LEFT_PULSE", -12.0)', SOURCE)
        self.assertIn('("GIMBAL_RIGHT_PULSE", +12.0)', SOURCE)
        self.assertIn('_stage(name, 0.6, telem, started, stop, True)', SOURCE)

    def test_esc_mode_and_imu_are_logged(self):
        for token in ('sub_esc(', 'sub_imu(', 'sub_mode(', 'stick_overlay(0)',
                      'CONNECTED_NO_COMMAND', 'FREE_WITH_ZERO_CHASSIS',
                      'OVERLAY_OFF_WITH_ZERO_CHASSIS', 'esc_rpm=', 'gyro_z='):
            with self.subTest(token=token):
                self.assertIn(token, SOURCE)


if __name__ == "__main__":
    unittest.main()
