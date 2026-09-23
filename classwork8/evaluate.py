from __future__ import annotations

import argparse
from pathlib import Path

from .occupancy_grid import calculate_accuracy, calculate_coverage, load_grid_csv


def main() -> None:
    parser = argparse.ArgumentParser(description="Calculate Classwork 8 map accuracy")
    parser.add_argument("predicted", type=Path, help="generated map.csv")
    parser.add_argument("ground_truth", type=Path, help="ground-truth CSV with same dimensions")
    args = parser.parse_args()

    predicted = load_grid_csv(args.predicted)
    truth = load_grid_csv(args.ground_truth)
    accuracy = calculate_accuracy(predicted, truth)
    coverage = calculate_coverage(predicted)
    print(f"Map Accuracy = {accuracy:.2f}%")
    print(f"Coverage = {coverage:.2f}%")


if __name__ == "__main__":
    main()
