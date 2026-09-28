"""V05 BASIC motion regression tests, no physical robot commands."""
import inspect
import sys
import types
import unittest

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


class BasicMotionTests(unittest.TestCase):
    def test_v05_entrypoint_keeps_requested_config_speed(self):
        from final_round1_tof_camera_01 import _defaults
        config = Classwork8Config()
        for speed in (0.10, 0.20, 0.30):
            with self.subTest(speed=speed):
                config.travel_speed_mps = speed
                _defaults(config)
                self.assertAlmostEqual(config.travel_speed_mps, speed)

    def test_requested_speed_along_all_cardinal_directions(self):
        config = Classwork8Config()
        for speed in (0.10, 0.20, 0.30):
            config.travel_speed_mps = speed
            config.validate()
            for direction in range(4):
                with self.subTest(speed=speed, direction=direction):
                    x, y, z, _ = mission._basic_motion_command(
                        config, direction, 0.0, 0.0
                    )
                    ux, uy = mission.DIR_VEC_DRIVE[direction]
                    self.assertAlmostEqual(x * ux + y * uy, speed)
                    self.assertAlmostEqual(z, 0.0)

    def test_legacy_caps_cannot_change_command(self):
        config = Classwork8Config()
        config.travel_speed_mps = 0.25
        config.motion_wall_adjacent_speed_cap_mps = 0.01
        config.motion_slow_cross_track_speed_mps = 0.01
        config.stop_front_cm = 299.0
        config.slow_front_cm = 300.0
        for yaw in (-12.0, -4.0, 0.0, 4.0, 12.0):
            x, y, z, _ = mission._basic_motion_command(
                config, 0, 0.0, yaw
            )
            self.assertAlmostEqual(x, 0.25)
            self.assertAlmostEqual(y, 0.0)
            self.assertLessEqual(abs(z), config.heading_max_z_dps)

    def test_heading_steering_does_not_stop_translation(self):
        config = Classwork8Config()
        for direction in range(4):
            x, y, z, _ = mission._basic_motion_command(
                config, direction, 0.0, 8.0
            )
            ux, uy = mission.DIR_VEC_DRIVE[direction]
            self.assertAlmostEqual(x * ux + y * uy, config.travel_speed_mps)
            self.assertNotEqual(z, 0.0)

    def test_no_environmental_stop_or_speed_scaling_in_runtime(self):
        source = inspect.getsource(mission._drive_one_cell)
        for token in (
            "_confirm_front_blocked(", "_midcell_wall_checkpoint(",
            "critical_start_side_recheck(", "side_checkpoint_decision(",
            "motion_wall_adjacent_speed_cap_mps", "motion_cross_track_abort_m",
            "motion_slow_cross_track_speed_mps", "slow_front_cm", "stop_front_cm",
            "bound_travel_lateral(", "_scan_side_guidance_v02(",
        ):
            with self.subTest(token=token):
                self.assertNotIn(token, source)
        self.assertIn("chassis.drive_speed(", source)
        self.assertIn("stop_event.is_set()", source)
        self.assertIn("remaining <= float(config.step_tolerance_m)", source)

    def test_yaw_controller_never_pauses_longitudinal_motion(self):
        source = inspect.getsource(mission._fixed_heading_control_v02)
        self.assertNotIn("HEADING_RECOVER", source)
        self.assertNotIn("return 0.0, 0.0", source)
        self.assertIn("return x_cmd, y_cmd, z_cmd", source)


if __name__ == "__main__":
    unittest.main()
