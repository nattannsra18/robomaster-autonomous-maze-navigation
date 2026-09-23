from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

from .config import Classwork8Config
from .occupancy_grid import OccupancyGrid


class RunRecorder:
    def __init__(self, config: Classwork8Config) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(config.output_dir) / f"run_{stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.config = config
        self.samples: List[dict] = []
        self.events: List[dict] = []
        self.start_monotonic: Optional[float] = None

    def start(self, now: float) -> None:
        self.start_monotonic = float(now)

    def _elapsed(self, now: float) -> float:
        if self.start_monotonic is None:
            self.start(now)
        return float(now) - float(self.start_monotonic)

    def record_sample(
        self,
        now: float,
        x_m: Optional[float],
        y_m: Optional[float],
        yaw_deg: Optional[float],
        heading_index: int,
        front_cm: Optional[float],
        left_cm: Optional[float],
        right_cm: Optional[float],
        ir_left: Optional[bool],
        ir_right: Optional[bool],
        mode: str,
        vision_error: Optional[float] = None,
        vision_confidence: Optional[float] = None,
        vision_correction_mps: Optional[float] = None,
    ) -> None:
        self.samples.append({
            "t_sec": round(self._elapsed(now), 4),
            "x_m": x_m,
            "y_m": y_m,
            "yaw_deg": yaw_deg,
            "heading_index": int(heading_index) % 4,
            "front_cm": front_cm,
            "left_cm": left_cm,
            "right_cm": right_cm,
            "ir_left_blocked": ir_left,
            "ir_right_blocked": ir_right,
            "mode": mode,
            "vision_error": vision_error,
            "vision_confidence": vision_confidence,
            "vision_correction_mps": vision_correction_mps,
        })

    def event(self, now: float, event_type: str, detail: str, **extra) -> None:
        row = {"t_sec": round(self._elapsed(now), 4), "event": event_type, "detail": detail}
        row.update(extra)
        self.events.append(row)

    def trajectory_xy(self) -> List[Tuple[float, float]]:
        result: List[Tuple[float, float]] = []
        for row in self.samples:
            x, y = row.get("x_m"), row.get("y_m")
            if x is None or y is None:
                continue
            point = (float(x), float(y))
            if not result or point != result[-1]:
                result.append(point)
        return result

    @staticmethod
    def _distance(points: List[Tuple[float, float]]) -> float:
        import math
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))

    def _write_csv(self, path: Path, rows: List[dict]) -> None:
        if not rows:
            path.write_text("", encoding="utf-8")
            return
        keys = []
        seen = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    keys.append(key)
                    seen.add(key)
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)

    def export(
        self,
        grid: OccupancyGrid,
        *,
        reason: str,
        start_pose: Optional[Tuple[float, float, float]],
        end_pose: Optional[Tuple[float, float, float]],
    ) -> Path:
        trajectory = self.trajectory_xy()
        self._write_csv(self.run_dir / "sensor_and_pose_log.csv", self.samples)
        self._write_csv(self.run_dir / "exploration_log.csv", self.events)
        self._write_csv(
            self.run_dir / "trajectory.csv",
            [{"sequence": i, "x_m": x, "y_m": y} for i, (x, y) in enumerate(trajectory)],
        )
        grid.save_csv(self.run_dir / "map.csv")
        grid.save_svg(
            self.run_dir / "map.svg",
            trajectory_xy=trajectory,
            start_xy=None if start_pose is None else start_pose[:2],
            end_xy=None if end_pose is None else end_pose[:2],
        )
        summary = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "reason": reason,
            "start_pose": start_pose,
            "end_pose": end_pose,
            "distance_travelled_m": round(self._distance(trajectory), 4),
            "explored_cells": grid.explored_count(),
            "total_cells": grid.rows * grid.cols,
            "working_canvas_coverage_percent": round(grid.coverage_percent(), 3),
            "coverage_note": "For the final classwork Coverage, align/crop map.csv to the Ground Truth evaluation area and run classwork8.evaluate.",
            "map_accuracy_percent": None,
            "map_accuracy_note": "Provide a same-size ground_truth.csv and run classwork8.evaluate to calculate this value.",
            "config": self.config.to_dict(),
        }
        (self.run_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return self.run_dir
