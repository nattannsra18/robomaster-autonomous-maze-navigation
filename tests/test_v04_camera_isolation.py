"""Hardware-free regression guards for Final Round 1 V04 + camera isolation."""
import ast
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
V04 = ROOT / "classwork8" / "tof_only_v04.py"
HYBRID = ROOT / "classwork8" / "tof_only_v04_camera.py"
CAMERA = ROOT / "classwork8" / "v04_target_survey.py"
RUNNER = ROOT / "final_round1_v04_slam_v05_camera_01.py"
GUI = ROOT / "classwork8" / "gui_v04_camera.py"

MOTION_AND_SLAM = (
    "_point_gimbal",
    "_scan_four_directions",
    "_drive_one_cell",
    "_plan_frontier_move",
    "_closed_maze_completion_v04",
    "_update_tof_ray",
    "_frontier_options",
    "_shortest_open_path",
)


def top_level_function(path, name):
    module = ast.parse(path.read_text(encoding="utf-8"))
    return next(node for node in module.body
                if isinstance(node, ast.FunctionDef) and node.name == name)


class CameraIsolationTests(unittest.TestCase):
    def test_v04_navigation_and_mapper_are_unmodified(self):
        for name in MOTION_AND_SLAM:
            with self.subTest(function=name):
                self.assertEqual(
                    ast.dump(top_level_function(V04, name), include_attributes=False),
                    ast.dump(top_level_function(HYBRID, name), include_attributes=False),
                )

    def test_hybrid_imports_no_v05_motion(self):
        for path in (HYBRID, CAMERA, RUNNER):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    self.assertNotIn("tof_camera_round1_v05", node.module or "")
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        self.assertNotIn("tof_camera_round1_v05", alias.name)

    def test_v04_runner_fixes_important_movement_defaults(self):
        body = RUNNER.read_text(encoding="utf-8")
        for assignment in (
            "config.travel_speed_mps = 0.10",
            "config.stop_front_cm = 18.0",
            "config.cell_size_m = 0.60",
            "config.side_start_auto_recovery_enabled = False",
            "config.wall_follow_recovery_enabled = False",
            "config.supervised_hold_on_safety_dead_end = False",
            "config.vision_steering_enabled = False",
        ):
            with self.subTest(assignment=assignment):
                self.assertIn(assignment, body)

    def test_camera_adapter_never_commands_chassis_translation_or_yaw(self):
        tree = ast.parse(CAMERA.read_text(encoding="utf-8"))
        for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
            if isinstance(call.func, ast.Attribute) and call.func.attr == "drive_speed":
                self.assertIsInstance(call.func.value, ast.Name)
                self.assertEqual("gimbal", call.func.value.id)
            if isinstance(call.func, ast.Attribute) and call.func.attr == "drive_wheels":
                self.fail("Camera module must never issue drive_wheels")
        self.assertIn("stop_chassis(chassis)", CAMERA.read_text(encoding="utf-8"))

    def test_gui_hides_v05_only_motion_controls(self):
        ui = GUI.read_text(encoding="utf-8")
        self.assertNotIn('text="RESCAN CURRENT CELL"', ui)
        self.assertNotIn('text="Skip 4-way scan at fully scanned visited cells"', ui)
        self.assertIn('V04 SLAM + V05 Camera', ui)

    def test_camera_and_v04_scan_are_separate(self):
        core = HYBRID.read_text(encoding="utf-8")
        self.assertIn("ranges, open_dirs = scan", core)
        self.assertIn("target_survey.survey_cell(", core)
        self.assertLess(core.index("ranges, open_dirs = scan"),
                        core.index("target_survey.survey_cell("))
        self.assertIn("CAMERA_HORIZONTAL_TOF_RESTORE_FAILED",
                      CAMERA.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
