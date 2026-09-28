"""Offline safety guard for a strictly zero-RPM wheel-control diagnostic."""
import ast
import unittest
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[1] / "diagnose_wheel_zero_v05.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)


class WheelZeroDiagnosticTests(unittest.TestCase):
    def test_all_wheel_commands_exactly_zero(self):
        calls = [
            node for node in ast.walk(TREE)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "drive_wheels"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "chassis"
        ]
        self.assertEqual(len(calls), 2)
        for call in calls:
            values = {kw.arg: ast.literal_eval(kw.value) for kw in call.keywords}
            self.assertEqual(values, {"w1": 0, "w2": 0, "w3": 0, "w4": 0})
        self.assertNotIn("chassis.drive_speed(", SOURCE)
        self.assertNotIn("gimbal.drive_speed(", SOURCE)
        self.assertNotIn("chassis.move(", SOURCE)

    def test_compare_before_and_after_free_and_wheel_zero(self):
        stages = ["CONNECTED_NO_DRIVE", "FREE_NO_DRIVE",
                  "FREE_AFTER_SINGLE_WHEEL_ZERO"]
        positions = [SOURCE.index('"' + x + '"') for x in stages]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("abs(delta) > 1.0", SOURCE)
        self.assertIn("v[\"age\"] > 0.5", SOURCE)
        self.assertIn("ep.get_robot_mode()", SOURCE)
        self.assertIn("[WHEEL_ZERO_RESULT]", SOURCE)
        self.assertIn("[WHEEL_ZERO_ACK_WARN]", SOURCE)


if __name__ == "__main__":
    unittest.main()
