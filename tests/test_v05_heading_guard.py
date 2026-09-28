"""Offline regression for bounded V05 heading control and one-cell test limits."""
import inspect
import sys
import types
import unittest
from types import SimpleNamespace

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
from classwork8 import tof_camera_round1_v05 as mission
from final_round1_tof_camera_01 import _apply_cli_overrides, main


class MovingHeadingGuardTests(unittest.TestCase):
    def test_large_heading_errors_stop_before_next_drive(self):
        config = Classwork8Config()
        config.heading_hold_enabled = True
        config.yaw_isolation_mode = False
        for err in (-41.0, -5.5, -4.01, 4.01, 5.5, 41.0):
            with self.subTest(err=err):
                self.assertTrue(
                    mission._moving_heading_over_limit(config, -67.5, -67.5 - err)
                )
        for err in (-4.0, -0.35, 0.0, 0.35, 4.0):
            with self.subTest(err=err):
                self.assertFalse(
                    mission._moving_heading_over_limit(config, -67.5, -67.5 - err)
                )

    def test_no_feedback_or_disabled_hold_does_not_inject_rotation(self):
        config = Classwork8Config()
        self.assertFalse(mission._moving_heading_over_limit(config, 0.0, None))
        config.yaw_isolation_mode = True
        self.assertFalse(mission._moving_heading_over_limit(config, 0.0, 12.0))
        config.yaw_isolation_mode = False
        config.heading_hold_enabled = False
        self.assertFalse(mission._moving_heading_over_limit(config, 0.0, 12.0))

    def test_stop_guard_precedes_remaining_and_drive_command(self):
        source = inspect.getsource(mission._drive_one_cell)
        self.assertLess(
            source.index("if _moving_heading_over_limit("),
            source.index("if remaining <= float(config.step_tolerance_m):"),
        )
        self.assertLess(
            source.index("if _moving_heading_over_limit("),
            source.index("chassis.drive_speed("),
        )
        self.assertIn('return False, "MOVING_YAW_LIMIT", moved', source)
        self.assertIn("stop_chassis(chassis)", source)


class HardDiagnosticLimitsTests(unittest.TestCase):
    def test_cli_constraints_override_gui_values(self):
        config = Classwork8Config()
        config.max_moves = 500
        config.heading_max_z_dps = 18.0
        config.heading_align_max_z_dps = 10.0
        config.travel_speed_mps = 0.30
        config.target_detection_enabled = True
        args = SimpleNamespace(
            max_moves=1, max_yaw_correction=5.0,
            travel_speed=0.10, no_camera=True, yaw_isolation=False,
        )
        _apply_cli_overrides(config, args)
        self.assertEqual(config.max_moves, 1)
        self.assertEqual(config.heading_max_z_dps, 5.0)
        self.assertEqual(config.heading_align_max_z_dps, 5.0)
        self.assertEqual(config.travel_speed_mps, 0.10)
        self.assertFalse(config.target_detection_enabled)
        self.assertTrue(config.heading_hold_enabled)
        config.validate()

    def test_main_reapplies_cli_bounds_after_gui(self):
        source = inspect.getsource(main)
        self.assertEqual(source.count("_apply_cli_overrides(config, args)"), 2)
        self.assertIn('"--max-moves"', source)
        self.assertIn('"--max-yaw-correction"', source)
        self.assertIn("[DIAG_LIMITS]", source)


if __name__ == "__main__":
    unittest.main()
