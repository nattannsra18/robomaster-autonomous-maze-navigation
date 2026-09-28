"""Offline V05 four-side clearance planning, GUI and real-command safety."""
import inspect
import threading
import time
import unittest
from unittest.mock import patch

from classwork8.config import Classwork8Config
from classwork8.wall_clearance_v05 import choose_clearance_plan
from classwork8 import tof_camera_round1_v05 as v05
from pathlib import Path


def enabled_config():
    config = Classwork8Config()
    config.wall_clearance_enabled = True
    config.wall_clearance_front_cm = 18.0
    config.wall_clearance_right_cm = 18.0
    config.wall_clearance_back_cm = 18.0
    config.wall_clearance_left_cm = 18.0
    return config


class WallClearancePlannerTests(unittest.TestCase):
    def test_all_four_wall_directions_move_opposite(self):
        cfg = enabled_config()
        for wall in range(4):
            with self.subTest(wall=wall):
                readings = {0: 100.0, 1: 100.0, 2: 100.0, 3: 100.0}
                readings[wall] = 12.0
                plan = choose_clearance_plan(readings, cfg)
                self.assertIsNotNone(plan)
                self.assertEqual(plan.wall_direction, wall)
                self.assertEqual(plan.away_direction, (wall + 2) % 4)
                self.assertAlmostEqual(plan.shift_cm, 4.0)

    def test_opposite_wall_budget_and_unsatisfiable_narrow_pair(self):
        cfg = enabled_config()
        # RIGHT too close, LEFT has only 3.5 cm room above its own target.
        readings = {0: 100.0, 1: 12.0, 2: 100.0, 3: 23.0}
        plan = choose_clearance_plan(readings, cfg)
        self.assertEqual(plan.wall_direction, 1)
        self.assertAlmostEqual(plan.shift_cm, 3.5)
        readings[3] = 17.0
        self.assertIsNone(choose_clearance_plan(readings, cfg))

    def test_missing_stale_or_nonwall_readings_do_not_drive(self):
        cfg = enabled_config()
        self.assertIsNone(choose_clearance_plan({0: 12.0, 2: None}, cfg))
        self.assertIsNone(choose_clearance_plan({0: 12.0, 2: float("nan")}, cfg))
        self.assertIsNone(choose_clearance_plan({0: 19.0, 2: 100.0}, cfg))
        self.assertIsNone(choose_clearance_plan({0: 56.0, 2: 100.0}, cfg))

    def test_target_field_validation(self):
        cfg = enabled_config()
        cfg.validate()
        cfg.wall_clearance_front_cm = 100.0
        with self.assertRaisesRegex(ValueError, "wall_clearance_front_cm"):
            cfg.validate()
        cfg.wall_clearance_front_cm = 18.0
        cfg.wall_clearance_max_step_cm = 10.0
        with self.assertRaisesRegex(ValueError, "wall_clearance_max_step_cm"):
            cfg.validate()

    def test_first_gui_tab_lists_all_four_independent_settings(self):
        source = (Path(__file__).resolve().parents[1] / "classwork8" /
                  "config_gui_v05.py").read_text(encoding="utf-8")
        first = source.split('"Mission Settings": [', 1)[1].split('"Motion": [', 1)[0]
        self.assertLess(first.index("wall_clearance_enabled"),
                        first.index("travel_speed_mps"))
        for side in ("front", "right", "back", "left"):
            self.assertIn("wall_clearance_{}_cm".format(side), first)
        self.assertIn('"wall_clearance_enabled": False', source)

    def test_checkpoint_is_opt_in_and_rescans_after_movement(self):
        source = inspect.getsource(v05.run)
        self.assertIn("_maintain_wall_clearance_checkpoint(", source)
        self.assertIn("if corrected:", source)
        self.assertIn("rescanned = _scan_four_directions(", source)
        self.assertIn("not config.wall_clearance_enabled", source)


class WallClearanceMotionTests(unittest.TestCase):
    def test_right_wall_triggers_only_short_left_translation_with_z_zero(self):
        cfg = enabled_config()
        cfg.odom_scale_x = 1.0
        cfg.odom_scale_y = 1.0
        cfg.wall_clearance_speed_mps = 0.035
        class Pose:
            y = 0.0
            def get_xy(self):
                return 0.0, self.y
            def get_yaw(self):
                return 0.0
            def attitude_age_sec(self):
                return 0.01
        class Chassis:
            def __init__(self, pose, sensors):
                self.pose = pose
                self.sensors = sensors
                self.commands = []
                self.stop_count = 0
            def stop(self):
                self.stop_count += 1
            def drive_wheels(self, w1=0, w2=0, w3=0, w4=0):
                self.commands.append(("stop", w1, w2, w3, w4))
                return True
            def drive_speed(self, x, y, z, timeout):
                self.commands.append(("move", x, y, z))
                self.pose.y += y * 0.10
                self.sensors.distance += abs(y * 0.10) * 100.0
                return None
        class Sensors:
            distance = 14.0
            def reset_filters(self):
                pass
            @property
            def tof_last_update(self):
                return time.monotonic()
            def get_front_cm(self):
                return self.distance
        class Tracker:
            def get_angles(self):
                return 0.0, 90.0
        pose, sensors = Pose(), Sensors()
        chassis = Chassis(pose, sensors)
        ranges = {0: 200.0, 1: 14.0, 2: 200.0, 3: 35.0}
        with patch.object(v05, "_point_gimbal", return_value=True):
            moved, reason = v05._maintain_wall_clearance_checkpoint(
                chassis, object(), pose, sensors, Tracker(), cfg, ranges,
                0.0, 0.0, 0.0, threading.Event(),
            )
        self.assertTrue(moved)
        self.assertIsNone(reason)
        motion = [c for c in chassis.commands if c[0] == "move"]
        self.assertGreater(len(motion), 1)
        self.assertTrue(all(x == z == 0.0 and y < 0.0
                            for _, x, y, z in motion))
        self.assertLessEqual(abs(pose.y), 0.052)
        self.assertEqual(chassis.commands[-1], ("stop", 0, 0, 0, 0))

    def test_feature_disabled_does_not_command_chassis(self):
        cfg = Classwork8Config()
        moved, reason = v05._maintain_wall_clearance_checkpoint(
            None, None, None, None, None, cfg,
            {0: 12.0, 2: 100.0}, 0.0, 0.0, 0.0, None,
        )
        self.assertEqual((moved, reason), (False, None))


if __name__ == "__main__":
    unittest.main()
