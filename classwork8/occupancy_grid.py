from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


UNKNOWN = -1
FREE = 0
OCCUPIED = 100


class OccupancyGrid:
    """Small dependency-free occupancy grid using integer evidence scores."""

    def __init__(
        self,
        width_m: float,
        height_m: float,
        resolution_m: float,
        *,
        free_delta: int = -2,
        occupied_delta: int = 5,
        min_score: int = -20,
        max_score: int = 20,
        free_threshold: int = -2,
        occupied_threshold: int = 3,
    ) -> None:
        self.resolution_m = float(resolution_m)
        self.cols = int(math.ceil(float(width_m) / self.resolution_m))
        self.rows = int(math.ceil(float(height_m) / self.resolution_m))
        self.width_m = self.cols * self.resolution_m
        self.height_m = self.rows * self.resolution_m
        self.origin_x_m = -self.width_m / 2.0
        self.origin_y_m = -self.height_m / 2.0
        self.free_delta = int(free_delta)
        self.occupied_delta = int(occupied_delta)
        self.min_score = int(min_score)
        self.max_score = int(max_score)
        self.free_threshold = int(free_threshold)
        self.occupied_threshold = int(occupied_threshold)
        self._score = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        self._observed = [[False for _ in range(self.cols)] for _ in range(self.rows)]

    def world_to_cell(self, x_m: float, y_m: float) -> Optional[Tuple[int, int]]:
        c = int(math.floor((float(x_m) - self.origin_x_m) / self.resolution_m))
        r = int(math.floor((float(y_m) - self.origin_y_m) / self.resolution_m))
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return r, c
        return None

    def cell_to_world(self, row: int, col: int) -> Tuple[float, float]:
        x = self.origin_x_m + (col + 0.5) * self.resolution_m
        y = self.origin_y_m + (row + 0.5) * self.resolution_m
        return x, y

    def _apply(self, cell: Tuple[int, int], delta: int) -> None:
        r, c = cell
        self._observed[r][c] = True
        self._score[r][c] = max(
            self.min_score,
            min(self.max_score, self._score[r][c] + int(delta)),
        )

    def mark_free(self, cell: Tuple[int, int]) -> None:
        self._apply(cell, self.free_delta)

    def mark_occupied(self, cell: Tuple[int, int]) -> None:
        self._apply(cell, self.occupied_delta)

    def state_at(self, row: int, col: int) -> int:
        if not self._observed[row][col]:
            return UNKNOWN
        score = self._score[row][col]
        if score >= self.occupied_threshold:
            return OCCUPIED
        if score <= self.free_threshold:
            return FREE
        return UNKNOWN

    def update_ray(
        self,
        origin_x_m: float,
        origin_y_m: float,
        angle_rad: float,
        distance_m: float,
        *,
        max_range_m: float,
        hit: bool,
    ) -> None:
        if distance_m <= 0.0 or max_range_m <= 0.0:
            return
        ray_len = min(float(distance_m), float(max_range_m))
        if ray_len <= 0.0:
            return

        step = max(self.resolution_m * 0.5, 0.01)
        free_end = ray_len - (self.resolution_m * 0.75 if hit else 0.0)
        free_end = max(0.0, free_end)
        seen = set()
        d = 0.0
        while d <= free_end + 1e-9:
            x = origin_x_m + math.cos(angle_rad) * d
            y = origin_y_m + math.sin(angle_rad) * d
            cell = self.world_to_cell(x, y)
            if cell is not None and cell not in seen:
                self.mark_free(cell)
                seen.add(cell)
            d += step

        if hit and distance_m <= max_range_m:
            x = origin_x_m + math.cos(angle_rad) * distance_m
            y = origin_y_m + math.sin(angle_rad) * distance_m
            cell = self.world_to_cell(x, y)
            if cell is not None:
                self.mark_occupied(cell)

    def matrix(self) -> List[List[int]]:
        return [[self.state_at(r, c) for c in range(self.cols)] for r in range(self.rows)]

    def explored_count(self) -> int:
        return sum(
            1
            for r in range(self.rows)
            for c in range(self.cols)
            if self.state_at(r, c) != UNKNOWN
        )

    def coverage_percent(self) -> float:
        total = self.rows * self.cols
        return 0.0 if total == 0 else 100.0 * self.explored_count() / total

    def save_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        matrix = self.matrix()
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(reversed(matrix))

    def save_svg(
        self,
        path: Path,
        trajectory_xy: Sequence[Tuple[float, float]] = (),
        start_xy: Optional[Tuple[float, float]] = None,
        end_xy: Optional[Tuple[float, float]] = None,
    ) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        scale = 4
        w, h = self.cols * scale, self.rows * scale
        parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
            '<rect width="100%" height="100%" fill="#b8b8b8"/>',
        ]
        for r in range(self.rows):
            for c in range(self.cols):
                state = self.state_at(r, c)
                if state == UNKNOWN:
                    continue
                fill = "#ffffff" if state == FREE else "#111111"
                x = c * scale
                y = (self.rows - 1 - r) * scale
                parts.append(
                    f'<rect x="{x}" y="{y}" width="{scale}" height="{scale}" fill="{fill}"/>'
                )

        if trajectory_xy:
            pts = []
            for x_m, y_m in trajectory_xy:
                cell = self.world_to_cell(x_m, y_m)
                if cell is None:
                    continue
                r, c = cell
                pts.append(
                    f"{(c + 0.5) * scale:.2f},{(self.rows - 1 - r + 0.5) * scale:.2f}"
                )
            if len(pts) >= 2:
                parts.append(
                    f'<polyline points="{" ".join(pts)}" fill="none" stroke="#2563eb" stroke-width="2"/>'
                )

        def marker(xy: Optional[Tuple[float, float]], fill: str, label: str) -> None:
            if xy is None:
                return
            cell = self.world_to_cell(*xy)
            if cell is None:
                return
            r, c = cell
            cx = (c + 0.5) * scale
            cy = (self.rows - 1 - r + 0.5) * scale
            parts.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="5" fill="{fill}"/>')
            parts.append(
                f'<text x="{cx + 6:.2f}" y="{cy - 6:.2f}" font-size="10" fill="{fill}">{label}</text>'
            )

        marker(start_xy, "#16a34a", "START")
        marker(end_xy, "#dc2626", "END")
        parts.append("</svg>")
        path.write_text("\n".join(parts), encoding="utf-8")


def load_grid_csv(path: Path) -> List[List[int]]:
    rows: List[List[int]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        for row in csv.reader(f):
            if not row:
                continue
            rows.append([int(value) for value in row])
    return rows


def calculate_coverage(predicted: Sequence[Sequence[int]]) -> float:
    total = sum(len(row) for row in predicted)
    if total == 0:
        return 0.0
    explored = sum(
        1 for row in predicted for value in row if int(value) != UNKNOWN
    )
    return 100.0 * explored / total


def calculate_accuracy(
    predicted: Sequence[Sequence[int]],
    truth: Sequence[Sequence[int]],
) -> float:
    if len(predicted) != len(truth) or any(
        len(a) != len(b) for a, b in zip(predicted, truth)
    ):
        raise ValueError(
            "predicted and ground-truth maps must have the same dimensions"
        )
    total = sum(len(row) for row in truth)
    if total == 0:
        return 0.0
    correct = sum(
        1
        for pred_row, truth_row in zip(predicted, truth)
        for pred, expected in zip(pred_row, truth_row)
        if int(pred) == int(expected)
    )
    return 100.0 * correct / total
