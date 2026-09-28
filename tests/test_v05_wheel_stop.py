"""V05-only zero-wheel stop regression: no physical robot connection."""
import ast
import inspect
import sys
import types
import unittest
from pathlib import Path

if "libmedia_codec" not in sys.modules:
    codec = types.ModuleType("libmedia_codec")
    class H264Decoder:
        def decode(self, _data):
            return []
    class OpusDecoder:
        def decode(self, _data):
            return None
    codec.H264Decoder = H264Decoder
    codec.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = codec

from classwork8 import tof_camera_round1_v05 as v05
from robomaster_mission import mission as legacy

SOURCE = (Path(__file__).resolve().parents[1] / "classwork8" /
          "tof_camera_round1_v05.py").read_text(encoding="utf-8")


class DummyChassis:
    def __init__(self, result=True):
        self.result = result
        self.wheel_calls = []
        self.speed_calls = []

    def drive_wheels(self, w1=0, w2=0, w3=0, w4=0):
        self.wheel_calls.append((w1, w2, w3, w4))
        return self.result

    def drive_speed(self, x=0, y=0, z=0, timeout=None):
        self.speed_calls.append((x, y, z, timeout))
        raise AssertionError("V05 STOP MUST NOT use drive_speed")


class V05WheelStopTests(unittest.TestCase):
    def test_stop_uses_acknowledged_four_wheel_zero(self):
        chassis = DummyChassis(True)
        self.assertIsNone(v05.stop_chassis(chassis))
        self.assertEqual(chassis.wheel_calls, [(0, 0, 0, 0)])
        self.assertEqual(chassis.speed_calls, [])
        self.assertIsNone(v05.stop_chassis(None))

    def test_no_ack_refuses_to_continue_or_fallback(self):
        for result in (False, None, 0):
            with self.subTest(result=result):
                chassis = DummyChassis(result)
                with self.assertRaisesRegex(
                    RuntimeError, "V05_WHEEL_STOP_NOT_ACKNOWLEDGED"
                ):
                    v05.stop_chassis(chassis)
                self.assertEqual(chassis.wheel_calls, [(0, 0, 0, 0)])
                self.assertEqual(chassis.speed_calls, [])

    def test_v05_helper_is_local_legacy_is_unchanged(self):
        self.assertIsNot(v05.stop_chassis, legacy.stop_chassis)
        func = inspect.getsource(v05.stop_chassis)
        self.assertIn("drive_wheels(w1=0, w2=0, w3=0, w4=0)", func)
        self.assertNotIn("chassis.drive_speed(", func)
        tree = ast.parse(SOURCE)
        from_imports = [
            alias.name for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            and node.module == "robomaster_mission.mission"
            for alias in node.names
        ]
        self.assertNotIn("stop_chassis", from_imports)

    def test_v05_motion_still_uses_speed_and_scan_calls_wheel_stop(self):
        move = inspect.getsource(v05._drive_one_cell)
        scan = inspect.getsource(v05._scan_four_directions)
        main = inspect.getsource(v05.run)
        self.assertIn("chassis.drive_speed(", move)
        self.assertIn("stop_chassis(chassis)", main)
        self.assertNotIn("chassis.drive_wheels(", move)
        self.assertNotIn("chassis.drive_speed(x=0.0, y=0.0, z=0.0", main)


if __name__ == "__main__":
    unittest.main()
