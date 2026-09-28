"""Offline constraints for stationary wheel-zero + bounded gimbal motion."""
import ast
import unittest
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[1] / "diagnose_wheel_zero_gimbal_v05.py").read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)


class WheelZeroGimbalTests(unittest.TestCase):
    def test_no_chassis_speed_or_nonzero_wheels(self):
        calls = [
            node for node in ast.walk(TREE)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "drive_wheels"
        ]
        self.assertEqual(len(calls), 1)
        for call in calls:
            self.assertIsInstance(call.func.value, ast.Name)
            self.assertEqual(call.func.value.id, "chassis")
            kwargs = {kw.arg: ast.literal_eval(kw.value) for kw in call.keywords}
            self.assertEqual(kwargs, {"w1": 0, "w2": 0, "w3": 0, "w4": 0})
        chassis_calls = [
            node.func.attr for node in ast.walk(TREE)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "chassis"
        ]
        self.assertNotIn("drive_speed", chassis_calls)
        self.assertNotIn("move", chassis_calls)
        self.assertNotIn("drive_wheels(", SOURCE.split("def _wheel_zero(chassis):", 1)[0])

    def test_gimbal_is_bounded_and_feedback_guarded(self):
        self.assertIn("time.monotonic() - started_move < 2.0", SOURCE)
        self.assertIn("if inward >= 12.0:", SOURCE)
        self.assertIn("if abs(body_delta) > 1.0:", SOURCE)
        self.assertIn("stop.wait(0.03)", SOURCE)
        self.assertIn("v[\"age\"] > 0.5", SOURCE)
        self.assertIn("stop.wait(0.30)", SOURCE)
        self.assertIn('pitch_speed=0.0, yaw_speed=0.0', SOURCE)

    def test_direct_zero_precedes_gimbal(self):
        self.assertLess(
            SOURCE.index('print("[WHEEL_ZERO] result='),
            SOURCE.index('resume = gimbal.resume()')
        )
        self.assertLess(
            SOURCE.index('resume = gimbal.resume()'),
            SOURCE.index('if not _sweep(')
        )
        self.assertIn('WHEEL_ZERO_AFTER_GIMBAL_STOP', SOURCE)


if __name__ == "__main__":
    unittest.main()
