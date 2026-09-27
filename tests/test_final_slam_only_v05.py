"""Offline checks for the dedicated V05 SLAM-only launcher (no real robot)."""

import unittest
from unittest import mock

from classwork8.config import Classwork8Config
import classwork8_slam_05 as slam


class SlamOnlyLauncherTests(unittest.TestCase):
    def test_defaults_keep_current_v05_recovery_and_separate_exports(self):
        cfg = Classwork8Config()
        slam.apply_slam_defaults(cfg)
        self.assertEqual(cfg.cell_size_m, 0.60)
        self.assertTrue(cfg.side_start_auto_recovery_enabled)
        self.assertTrue(cfg.supervised_hold_on_safety_dead_end)
        self.assertEqual(cfg.side_start_recovery_opposite_min_cm, 20.0)
        self.assertEqual(cfg.stop_front_cm, 13.0)
        self.assertEqual(cfg.output_dir, "classwork8_output_slam")
        cfg.validate()

    def test_camera_and_ground_truth_are_never_required(self):
        cfg = Classwork8Config()
        slam.apply_slam_defaults(cfg)
        self.assertFalse(cfg.target_detection_enabled)
        self.assertFalse(cfg.target_survey_open_directions)
        self.assertFalse(cfg.vision_enabled)
        self.assertFalse(cfg.vision_steering_enabled)
        self.assertFalse(hasattr(cfg, "ground_truth_path"))
        cfg.validate()

    def test_gui_cannot_reenable_camera_before_mission(self):
        cfg = Classwork8Config()
        slam.apply_slam_defaults(cfg)
        # Simulate setting target controls in the shared V05 config GUI.
        cfg.target_detection_enabled = True
        cfg.target_survey_open_directions = True
        cfg.vision_enabled = True
        cfg.vision_steering_enabled = True
        marker = object()
        with mock.patch.object(slam, "run_v05", return_value=marker) as runner:
            result = slam.run_slam_only(config=cfg)
        self.assertIs(result, marker)
        self.assertFalse(cfg.target_detection_enabled)
        self.assertFalse(cfg.target_survey_open_directions)
        self.assertFalse(cfg.vision_enabled)
        self.assertFalse(cfg.vision_steering_enabled)
        runner.assert_called_once_with(config=cfg)

    def test_gui_runner_compatible_with_publish_and_preconnected_robot(self):
        cfg = Classwork8Config()
        slam.apply_slam_defaults(cfg)
        callback = mock.Mock()
        robot_stub = object()
        with mock.patch.object(slam, "run_v05") as runner:
            slam.run_slam_only(config=cfg, publish=callback, ep_robot=robot_stub)
        runner.assert_called_once_with(
            config=cfg, publish=callback, ep_robot=robot_stub,
        )


if __name__ == "__main__":
    unittest.main()
