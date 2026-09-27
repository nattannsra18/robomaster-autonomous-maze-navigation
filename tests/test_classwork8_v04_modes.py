"""Check both V04 entry points without connecting to a physical robot."""

import importlib.util
import json
import runpy
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from classwork8.config import Classwork8Config
from classwork8.occupancy_grid import OccupancyGrid
from classwork8.reporting import RunRecorder


ROOT = Path(__file__).resolve().parents[1]


class V04ModeTests(unittest.TestCase):
    def test_gui_entry_points_select_the_correct_workflow(self):
        for ground_truth in (False, True):
            with self.subTest(ground_truth=ground_truth):
                observed = []

                class FakeRobot:
                    def initialize(self, conn_type):
                        return True

                fake_robot = types.ModuleType("robomaster")
                fake_robot.robot = types.SimpleNamespace(Robot=FakeRobot)
                fake_runner = types.ModuleType("classwork8.tof_only_v04")
                fake_runner.run = lambda **kwargs: None
                fake_config_gui = types.ModuleType("classwork8.config_gui_v04")
                fake_config_gui.configure_before_run = (
                    lambda config, *, with_ground_truth: observed.append(
                        ("configuration", with_ground_truth, config.auto_evaluate_on_save)
                    ) or True
                )
                fake_live_gui = types.ModuleType("classwork8.gui_v04")
                fake_live_gui.run_with_gui = lambda run, config, robot: observed.append(
                    ("run", config.auto_evaluate_on_save)
                )
                modules = {
                    "robomaster": fake_robot,
                    "classwork8.tof_only_v04": fake_runner,
                    "classwork8.config_gui_v04": fake_config_gui,
                    "classwork8.gui_v04": fake_live_gui,
                }
                with patch.dict(sys.modules, modules):
                    spec = importlib.util.spec_from_file_location(
                        "classwork8_slam_04", ROOT / "classwork8_slam_04.py"
                    )
                    entry = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(entry)
                    with patch.dict(sys.modules, {"classwork8_slam_04": entry}):
                        with patch.object(sys, "argv", ["classwork8_slam_04.py"]):
                            if ground_truth:
                                runpy.run_path(
                                    str(ROOT / "classwork8_slam_04_ground_truth.py"),
                                    run_name="__main__",
                                )
                            else:
                                entry.main()

                self.assertEqual(observed, [
                    ("configuration", ground_truth, ground_truth),
                    ("run", ground_truth),
                ])

    def test_slam_only_save_skips_evaluation_even_if_reference_exists(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Classwork8Config()
            config.output_dir = directory
            config.ground_truth_csv = str(Path(directory) / "ground_truth.csv")
            Path(config.ground_truth_csv).write_text("0\n", encoding="utf-8")
            config.auto_evaluate_on_save = False
            recorder = RunRecorder(config)
            grid = OccupancyGrid(0.2, 0.2, 0.1)
            grid.mark_free((0, 0))
            output = recorder.export(
                grid, reason="USER_STOP", start_pose=(0, 0, 0), end_pose=(0, 0, 0)
            )
            self.assertTrue((output / "map.csv").is_file())
            self.assertTrue((output / "trajectory.csv").is_file())
            self.assertFalse((output / "evaluation").exists())
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["evaluation_status"], "disabled")


if __name__ == "__main__":
    unittest.main()
