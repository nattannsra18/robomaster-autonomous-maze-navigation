"""Offline safety checks for the no-motion zero-command differential probe."""
import ast
import unittest
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[1] / "diagnose_zero_command_v05.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)


class ZeroCommandDiagnosticTests(unittest.TestCase):
    def test_only_zero_chassis_drive_commands(self):
        commands = [
            node for node in ast.walk(TREE)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "drive_speed"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "chassis"
        ]
        self.assertEqual(len(commands), 4)
        for node in commands:
            vals = {kw.arg: ast.literal_eval(kw.value) for kw in node.keywords}
            self.assertEqual(vals["x"], 0.0)
            self.assertEqual(vals["y"], 0.0)
            self.assertEqual(vals["z"], 0.0)
        self.assertNotIn("drive_wheels(", SOURCE)
        self.assertNotIn("gimbal.drive_speed", SOURCE)
        self.assertNotIn("chassis.move(", SOURCE)

    def test_stages_differentiate_free_from_zero_speed(self):
        names = [
            "CONNECTED_NO_CHASSIS_COMMAND",
            "FREE_NO_CHASSIS_COMMAND",
            "FREE_SINGLE_ZERO_NO_TIMEOUT",
            "FREE_SINGLE_ZERO_WITH_TIMEOUT",
            "FREE_PERIODIC_ZERO_WITH_TIMEOUT",
        ]
        positions = [SOURCE.index('"' + name + '"') for name in names]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("ep.get_robot_mode()", SOURCE)
        self.assertIn('timeout=0.2', SOURCE)
        self.assertIn("abs(overall_delta) > 2.0", SOURCE)

    def test_yaw_wrap(self):
        from diagnose_zero_command_v05 import angle_change
        self.assertAlmostEqual(angle_change(179.0, -179.0), 2.0)
        self.assertAlmostEqual(angle_change(-179.0, 179.0), -2.0)


if __name__ == "__main__":
    unittest.main()
