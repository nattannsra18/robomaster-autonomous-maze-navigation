import tempfile
import unittest
from pathlib import Path

from classwork8.evaluate import align_to_truth, auto_evaluate_saved_map, calculate_metrics, evaluate, read_map, rotate_clockwise
from classwork8.config import Classwork8Config
from classwork8.occupancy_grid import OccupancyGrid
from classwork8.reporting import RunRecorder


class AssignmentEvaluationTests(unittest.TestCase):
    def test_known_example_exact_formulas(self):
        pred = [[0, 100], [-1, 0]]
        truth = [[0, 100], [100, 100]]
        result = calculate_metrics(pred, truth)
        self.assertEqual(result["correct_cells"], 2)
        self.assertEqual(result["explored_cells"], 3)
        self.assertEqual(result["map_accuracy_percent"], 50.0)
        self.assertEqual(result["coverage_percent"], 75.0)

    def test_all_unknown_is_zero_not_free_points(self):
        result = calculate_metrics([[-1, -1]], [[0, 100]])
        self.assertEqual(result["map_accuracy_percent"], 0)
        self.assertEqual(result["coverage_percent"], 0)
        self.assertIsNone(result["observed_cell_accuracy_percent"])

    def test_crop_explicit_only(self):
        pred = [[-1, -1, -1], [-1, 100, 0], [-1, 0, 0]]
        truth = [[100, 0], [0, 0]]
        with self.assertRaises(ValueError):
            align_to_truth(pred, truth)
        self.assertEqual(align_to_truth(pred, truth, top=1, left=1), truth)
        with self.assertRaises(ValueError):
            align_to_truth(pred, truth, top=2, left=1)

    def test_rotation_clockwise(self):
        self.assertEqual(rotate_clockwise([[0, 100], [-1, 0]], 90), [[-1, 0], [0, 100]])

    def test_ground_truth_must_be_fully_labeled(self):
        with tempfile.TemporaryDirectory() as d:
            source = Path(d) / "truth.csv"
            source.write_text("0,-1\n100,0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Ground Truth"):
                read_map(source, ground_truth=True)

    def test_save_without_ground_truth_produces_pending_report(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            predicted = root / "map.csv"
            predicted.write_text("0,100\n-1,0\n", encoding="utf-8")
            config = Classwork8Config()
            config.ground_truth_csv = str(root / "missing_ground_truth.csv")
            out = root / "evaluation"
            result = auto_evaluate_saved_map(predicted, out, config)
            self.assertEqual(result["evaluation_status"], "ground_truth_missing")
            self.assertIsNone(result["map_accuracy_percent"])
            self.assertIsNone(result["coverage_percent"])
            self.assertEqual(result["working_canvas_coverage_percent"], 75.0)
            self.assertIn("NOT AVAILABLE", (out / "evaluation.txt").read_text(encoding="utf-8"))
            self.assertTrue((out / "evaluation.json").is_file())

    def test_save_with_ground_truth_computes_both_scores(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            predicted = root / "map.csv"
            truth = root / "ground_truth.csv"
            predicted.write_text("0,100\n-1,0\n", encoding="utf-8")
            truth.write_text("0,100\n100,100\n", encoding="utf-8")
            config = Classwork8Config()
            config.ground_truth_csv = str(truth)
            out = root / "evaluation"
            result = auto_evaluate_saved_map(predicted, out, config)
            self.assertEqual(result["evaluation_status"], "complete")
            self.assertEqual(result["map_accuracy_percent"], 50)
            self.assertEqual(result["coverage_percent"], 75)
            self.assertTrue((out / "comparison.svg").is_file())
            self.assertTrue((out / "ground_truth.csv").is_file())

    def test_export_automatically_writes_evaluation_and_summary(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            config = Classwork8Config()
            config.output_dir = str(root / "runs")
            config.ground_truth_csv = str(root / "ground_truth.csv")
            grid = OccupancyGrid(0.2, 0.2, 0.1)
            grid.mark_free((0, 0))
            grid.mark_occupied((0, 1))
            ground_truth = [
                [0 if value == -1 else value for value in row]
                for row in reversed(grid.matrix())
            ]
            with (root / "ground_truth.csv").open("w", encoding="utf-8") as fp:
                fp.write("\n".join(",".join(map(str, row)) for row in ground_truth) + "\n")
            recorder = RunRecorder(config)
            run_dir = recorder.export(grid, reason="USER_STOP",
                                      start_pose=(0.0, 0.0, 0.0),
                                      end_pose=(0.0, 0.0, 0.0))
            result = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(result["evaluation_status"], "complete")
            self.assertEqual(result["assignment_coverage_percent"], 50.0)
            self.assertTrue((run_dir / "evaluation" / "evaluation.json").exists())

    def test_end_to_end_exports(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            pred = root / "map.csv"
            truth = root / "truth.csv"
            pred.write_text("0,100\n-1,0\n", encoding="utf-8")
            truth.write_text("0,100\n100,100\n", encoding="utf-8")
            out = root / "evaluation"
            result = evaluate(pred, truth, out)
            self.assertEqual(result["correct_cells"], 2)
            for name in ("aligned_map.csv", "comparison.svg", "evaluation.json", "evaluation.txt"):
                self.assertTrue((out / name).is_file(), name)
            self.assertIn("50.00%", (out / "evaluation.txt").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
