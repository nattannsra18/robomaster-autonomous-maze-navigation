import tempfile
import unittest
from pathlib import Path

from classwork8.evaluate import align_to_truth, calculate_metrics, evaluate, read_map, rotate_clockwise


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
