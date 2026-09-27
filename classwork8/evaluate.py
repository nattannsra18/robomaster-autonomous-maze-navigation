"""Evaluate a Classwork 8 occupancy map against independently drawn ground truth.

Both maps must use the same cell resolution and orientation. The default
comparison requires equal dimensions; an explicit crop locates a smaller
truth region within the exported working canvas. No best-fit auto-alignment
is performed (that would bias the reported assignment scores).
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import List, Optional, Sequence
from xml.sax.saxutils import escape

UNKNOWN, FREE, WALL = -1, 0, 100
VALID = {UNKNOWN, FREE, WALL}


def read_map(path: Path, *, ground_truth: bool = False) -> List[List[int]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = [[int(item.strip()) for item in row] for row in csv.reader(stream) if row]
    if not rows or not rows[0] or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError(f"{path}: CSV must contain a nonempty rectangular map")
    invalid = sorted(set(value for row in rows for value in row) - VALID)
    if invalid:
        raise ValueError(f"{path}: invalid cell values {invalid}; use -1, 0, or 100")
    if ground_truth and any(UNKNOWN in row for row in rows):
        raise ValueError(f"{path}: Ground Truth must label every evaluated cell as 0 or 100")
    return rows


def rotate_clockwise(grid: Sequence[Sequence[int]], degrees: int) -> List[List[int]]:
    result = [list(row) for row in grid]
    for _ in range(degrees // 90):
        result = [list(row) for row in zip(*result[::-1])]
    return result


def align_to_truth(
    predicted: Sequence[Sequence[int]],
    truth: Sequence[Sequence[int]],
    *,
    top: Optional[int] = None,
    left: Optional[int] = None,
) -> List[List[int]]:
    p_rows, p_cols = len(predicted), len(predicted[0])
    t_rows, t_cols = len(truth), len(truth[0])
    if top is None and left is None:
        if (p_rows, p_cols) != (t_rows, t_cols):
            raise ValueError(
                f"Map {p_rows}x{p_cols} differs from Ground Truth {t_rows}x{t_cols}. "
                "Verify both use the same resolution; then provide --top and --left "
                "for the Ground Truth's top-left cell within map.csv."
            )
        return [list(row) for row in predicted]
    if top is None or left is None:
        raise ValueError("Provide both --top and --left, or neither")
    if top < 0 or left < 0 or top + t_rows > p_rows or left + t_cols > p_cols:
        raise ValueError("Ground Truth crop lies outside the exported map")
    return [list(row[left : left + t_cols]) for row in predicted[top : top + t_rows]]


def calculate_metrics(predicted: Sequence[Sequence[int]], truth: Sequence[Sequence[int]]) -> dict:
    if len(predicted) != len(truth) or any(len(a) != len(b) for a, b in zip(predicted, truth)):
        raise ValueError("Aligned map and Ground Truth must have identical dimensions")
    total = sum(len(row) for row in truth)
    if total == 0:
        raise ValueError("Evaluation grid is empty")
    if any(value not in {FREE, WALL} for row in truth for value in row):
        raise ValueError("Ground Truth must contain only FREE (0) or WALL (100)")
    if any(value not in VALID for row in predicted for value in row):
        raise ValueError("Predicted map contains invalid cell values")

    explored = correct = correct_explored = 0
    for p_row, t_row in zip(predicted, truth):
        for prediction, expected in zip(p_row, t_row):
            if prediction != UNKNOWN:
                explored += 1
                if prediction == expected:
                    correct_explored += 1
            if prediction == expected:
                correct += 1
    return {
        "total_cells": total,
        "explored_cells": explored,
        "unknown_cells": total - explored,
        "correct_cells": correct,
        "incorrect_cells": total - correct,
        "map_accuracy_percent": 100.0 * correct / total,
        "coverage_percent": 100.0 * explored / total,
        "observed_cell_accuracy_percent": (100.0 * correct_explored / explored if explored else None),
        "rows": len(truth),
        "cols": len(truth[0]),
        "note": "Assignment accuracy treats predicted UNKNOWN as incorrect; observed-cell accuracy is diagnostic only.",
    }


def write_csv(path: Path, grid: Sequence[Sequence[int]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        csv.writer(stream).writerows(grid)


def write_comparison_svg(path: Path, predicted: Sequence[Sequence[int]], truth: Sequence[Sequence[int]], metrics: dict) -> None:
    rows, cols = metrics["rows"], metrics["cols"]
    cell = max(2, min(16, 800 // max(rows, cols)))
    gap, margin, header = 24, 20, 84
    board_width, board_height = cols * cell, rows * cell
    width = max(960, 3 * board_width + 2 * gap + 2 * margin)
    height = board_height + header + margin + 28
    palette = {UNKNOWN: "#b6bbc5", FREE: "#ffffff", WALL: "#222222"}
    blocks = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {} {}" width="{}" height="{}">'.format(width, height, width, height),
        '<rect width="100%" height="100%" fill="#f4f5f7"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#202020}</style>',
        f'<text x="{margin}" y="25" font-size="18" font-weight="bold">Classwork 8 - Map Evaluation</text>',
        f'<text x="{margin}" y="45" font-size="13">Accuracy {metrics["map_accuracy_percent"]:.2f}% | Coverage {metrics["coverage_percent"]:.2f}% | N={metrics["total_cells"]}</text>',
    ]
    for panel, title in enumerate(("Robot map", "Ground truth", "Agreement")):
        x0 = margin + panel * (board_width + gap)
        blocks.append(f'<text x="{x0}" y="{header-12}" font-size="14" font-weight="bold">{escape(title)}</text>')
        blocks.append(f'<rect x="{x0}" y="{header}" width="{board_width}" height="{board_height}" fill="white" stroke="#777"/>')
        for row_index, (p_row, t_row) in enumerate(zip(predicted, truth)):
            for col_index, (value, expected) in enumerate(zip(p_row, t_row)):
                fill = (palette[value] if panel == 0 else palette[expected] if panel == 1 else
                        "#16a34a" if value == expected else "#eab308" if value == UNKNOWN else "#dc2626")
                blocks.append(f'<rect x="{x0+col_index*cell}" y="{header+row_index*cell}" width="{cell}" height="{cell}" fill="{fill}"/>')
    blocks.append(f'<text x="{margin}" y="{height-8}" font-size="12">Agreement: green=correct; yellow=unknown; red=wrong prediction. Map: gray=unknown, white=free, black=wall.</text>')
    blocks.append('</svg>')
    path.write_text("\n".join(blocks), encoding="utf-8")


def evaluate(
    predicted_path: Path,
    truth_path: Path,
    output_dir: Path,
    *,
    top: Optional[int] = None,
    left: Optional[int] = None,
    rotate: int = 0,
) -> dict:
    predicted = rotate_clockwise(read_map(predicted_path), rotate)
    truth = read_map(truth_path, ground_truth=True)
    aligned = align_to_truth(predicted, truth, top=top, left=left)
    metrics = calculate_metrics(aligned, truth)
    metrics["predicted_file"] = str(predicted_path)
    metrics["ground_truth_file"] = str(truth_path)
    metrics["rotation_clockwise_degrees"] = rotate
    metrics["crop_top_row"] = 0 if top is None else top
    metrics["crop_left_col"] = 0 if left is None else left
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "aligned_map.csv", aligned)
    write_comparison_svg(output_dir / "comparison.svg", aligned, truth, metrics)
    (output_dir / "evaluation.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
    (output_dir / "evaluation.txt").write_text(
        "Classwork 8 - Map Evaluation\n"
        f"Map Accuracy = {metrics['correct_cells']} / {metrics['total_cells']} x 100 = {metrics['map_accuracy_percent']:.2f}%\n"
        f"Coverage = {metrics['explored_cells']} / {metrics['total_cells']} x 100 = {metrics['coverage_percent']:.2f}%\n"
        f"Unknown = {metrics['unknown_cells']} cells; Incorrect = {metrics['incorrect_cells']} cells\n",
        encoding="utf-8",
    )
    return metrics


def auto_evaluate_saved_map(predicted_path: Path, output_dir: Path, config) -> dict:
    """Always save a clearly labelled evaluation status; score only against real GT.

    This intentionally never guesses a crop/orientation or treats the unknown
    8x8-m working canvas as the lecturer's actual field area.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    ground_truth_path = Path(str(getattr(config, "ground_truth_csv", "")).strip() or "ground_truth.csv").expanduser()
    result = {
        "evaluation_status": "ground_truth_missing",
        "map_accuracy_percent": None,
        "coverage_percent": None,
        "ground_truth_file": str(ground_truth_path),
        "note": "Place a measured ground_truth.csv at the configured path to calculate assignment scores. The working-canvas percentage is diagnostic, not assignment Coverage.",
    }
    try:
        predicted = read_map(predicted_path)
        total = sum(map(len, predicted))
        explored = sum(value != UNKNOWN for row in predicted for value in row)
        result.update({
            "working_canvas_explored_cells": explored,
            "working_canvas_total_cells": total,
            "working_canvas_coverage_percent": round(100.0 * explored / total, 3),
        })
        if ground_truth_path.is_file():
            top = int(getattr(config, "evaluation_crop_top", -1))
            left = int(getattr(config, "evaluation_crop_left", -1))
            rotate = int(getattr(config, "evaluation_rotate_deg", 0))
            result = evaluate(
                predicted_path, ground_truth_path, output_dir,
                top=None if top == -1 else top,
                left=None if left == -1 else left,
                rotate=rotate,
            )
            result.update({
                "evaluation_status": "complete",
                "working_canvas_coverage_percent": round(100.0 * explored / total, 3),
            })
            write_csv(output_dir / "ground_truth.csv", read_map(ground_truth_path, ground_truth=True))
        else:
            (output_dir / "evaluation.txt").write_text(
                "Assignment Map Accuracy: NOT AVAILABLE (Ground Truth missing)\n"
                "Assignment Coverage: NOT AVAILABLE (field evaluation bounds unknown)\n"
                f"Working canvas coverage ONLY: {result['working_canvas_coverage_percent']:.2f}% "
                f"({explored}/{total} fine-grid cells)\n"
                f"Expected Ground Truth: {ground_truth_path}\n",
                encoding="utf-8",
            )
    except (OSError, ValueError, TypeError) as exc:
        result["evaluation_status"] = "evaluation_error"
        result["map_accuracy_percent"] = None
        result["coverage_percent"] = None
        result["error"] = str(exc)
        (output_dir / "evaluation.txt").write_text(
            "Assignment evaluation could not be computed.\n"
            f"Reason: {exc}\n"
            "Map/log/trajectory remain saved; verify Ground Truth resolution, "
            "orientation and explicit crop coordinates.\n",
            encoding="utf-8",
        )
    (output_dir / "evaluation.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate an aligned occupancy-grid map against independently prepared Ground Truth")
    parser.add_argument("predicted", type=Path, help="exported map.csv (5-cm occupancy grid by default)")
    parser.add_argument("ground_truth", type=Path, help="ground_truth.csv at the SAME cell resolution/orientation")
    parser.add_argument("--top", type=int, help="Ground Truth top-left row in map.csv (0-based)")
    parser.add_argument("--left", type=int, help="Ground Truth top-left column in map.csv (0-based)")
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0, help="rotate predicted CSV clockwise, before cropping")
    parser.add_argument("--output", type=Path, help="output directory (default: next to map.csv in evaluation/)")
    args = parser.parse_args()
    try:
        result = evaluate(args.predicted, args.ground_truth, args.output or args.predicted.parent / "evaluation",
                          top=args.top, left=args.left, rotate=args.rotate)
    except (ValueError, FileNotFoundError) as exc:
        parser.error(str(exc))
    print(f"Map Accuracy = {result['correct_cells']} / {result['total_cells']} x 100 = {result['map_accuracy_percent']:.2f}%")
    print(f"Coverage = {result['explored_cells']} / {result['total_cells']} x 100 = {result['coverage_percent']:.2f}%")
    print(f"Results saved to: {args.output or args.predicted.parent / 'evaluation'}")


if __name__ == "__main__":
    main()
