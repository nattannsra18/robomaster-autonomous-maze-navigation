"""Lighting-robust color + shape target detection for Final Assignment Round 1.

OpenCV only; no deep learning.

Pipeline:
raw BGR -> CLAHE on Lab-L -> HSV candidate masks -> morphology -> contours ->
shape score + color score -> temporal verification -> TargetRegistry.

The detector intentionally does not aim or fire a blaster.  Round 1 only
records target type and navigation-relevant observation position.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


# Broad candidate ranges.  Lighting robustness comes from CLAHE, confidence
# scoring, and multi-frame verification rather than a very narrow inRange box.
COLOR_RANGES = {
    "red": [
        ((0, 75, 45), (12, 255, 255)),
        ((168, 75, 45), (179, 255, 255)),
    ],
    "green": [
        ((35, 55, 35), (90, 255, 255)),
    ],
    "blue": [
        ((88, 65, 35), (138, 255, 255)),
    ],
    "yellow": [
        ((18, 65, 55), (39, 255, 255)),
    ],
    "orange": [
        ((5, 80, 50), (24, 255, 255)),
    ],
}

# Circular hue centres used only as a soft confidence term.
HUE_CENTRES = {
    "red": 0.0,
    "green": 62.0,
    "blue": 112.0,
    "yellow": 29.0,
    "orange": 15.0,
}

DRAW_COLOURS = {
    "red": (0, 0, 255),
    "green": (0, 220, 0),
    "blue": (255, 80, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 165, 255),
}

DIR_NAME = {0: "FRONT", 1: "RIGHT", 2: "BACK", 3: "LEFT"}
DIR_VEC = {0: (1.0, 0.0), 1: (0.0, -1.0), 2: (-1.0, 0.0), 3: (0.0, 1.0)}


@dataclass
class TargetDetection:
    color: str
    shape: str
    bbox: Tuple[int, int, int, int]
    centroid: Tuple[int, int]
    area: float
    perimeter: float
    vertices: int
    aspect_ratio: float
    circularity: float
    rectangularity: float
    median_hsv: Tuple[float, float, float]
    median_lab: Tuple[float, float, float]
    color_confidence: float
    shape_confidence: float
    confidence: float
    contour: np.ndarray = field(repr=False, compare=False)


@dataclass
class VerifiedTarget:
    detection: TargetDetection
    verified_frames: int
    confidence: float


class TargetDetector:
    def __init__(self, config) -> None:
        self.config = config
        self._clahe = cv2.createCLAHE(
            clipLimit=float(config.target_clahe_clip_limit),
            tileGridSize=(
                int(config.target_clahe_grid),
                int(config.target_clahe_grid),
            ),
        )

    def normalize_lighting(self, frame: np.ndarray) -> np.ndarray:
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_chan, a_chan, b_chan = cv2.split(lab)
        l_norm = self._clahe.apply(l_chan)
        normalized = cv2.merge((l_norm, a_chan, b_chan))
        return cv2.cvtColor(normalized, cv2.COLOR_LAB2BGR)

    @staticmethod
    def _hue_distance(h1: float, h2: float) -> float:
        raw = abs(float(h1) - float(h2))
        return min(raw, 180.0 - raw)

    def _make_mask(self, hsv: np.ndarray, color_name: str) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in COLOR_RANGES[color_name]:
            part = cv2.inRange(
                hsv,
                np.asarray(lower, dtype=np.uint8),
                np.asarray(upper, dtype=np.uint8),
            )
            mask = cv2.bitwise_or(mask, part)

        k = max(3, int(self.config.target_morph_kernel))
        if k % 2 == 0:
            k += 1
        kernel = np.ones((k, k), dtype=np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def _shape(self, contour: np.ndarray) -> Tuple[str, float, dict]:
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        if area <= 0.0 or perimeter <= 0.0:
            return "unknown", 0.0, {}

        approx = cv2.approxPolyDP(
            contour,
            float(self.config.target_approx_epsilon_ratio) * perimeter,
            True,
        )
        vertices = len(approx)
        x, y, w, h = cv2.boundingRect(contour)
        aspect = float(w) / float(h) if h > 0 else 0.0
        bbox_area = float(max(1, w * h))
        rectangularity = area / bbox_area
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)

        shape = "unknown"
        score = 0.0

        if vertices == 4 and cv2.isContourConvex(approx):
            rect_score = max(
                0.0,
                min(
                    1.0,
                    (rectangularity - float(self.config.target_rectangularity_min))
                    / max(1e-6, 1.0 - float(self.config.target_rectangularity_min)),
                ),
            )

            square_error = abs(math.log(max(1e-6, aspect)))
            square_score = max(0.0, 1.0 - square_error / math.log(1.45))

            if (
                float(self.config.target_square_aspect_min)
                <= aspect
                <= float(self.config.target_square_aspect_max)
            ):
                shape = "square"
                score = 0.55 * rect_score + 0.45 * square_score
            elif rectangularity >= float(self.config.target_rectangularity_min):
                shape = "rectangle"
                score = max(0.55, 0.75 * rect_score + 0.25 * min(1.0, abs(aspect - 1.0)))

        if shape == "unknown":
            circle_score = max(
                0.0,
                min(
                    1.0,
                    (circularity - float(self.config.target_circle_circularity_min))
                    / max(1e-6, 1.0 - float(self.config.target_circle_circularity_min)),
                ),
            )
            if (
                vertices >= int(self.config.target_circle_min_vertices)
                and circularity >= float(self.config.target_circle_circularity_min)
            ):
                shape = "circle"
                score = max(0.55, circle_score)

        return shape, float(score), {
            "area": area,
            "perimeter": perimeter,
            "vertices": vertices,
            "aspect_ratio": aspect,
            "circularity": circularity,
            "rectangularity": rectangularity,
        }

    def _inside_statistics(
        self,
        contour: np.ndarray,
        hsv: np.ndarray,
        lab: np.ndarray,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        region_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(region_mask, [contour], -1, 255, thickness=-1)

        ys, xs = np.where(region_mask > 0)
        if len(xs) == 0:
            return (0.0, 0.0, 0.0), (0.0, 128.0, 128.0)

        hsv_pixels = hsv[ys, xs]
        lab_pixels = lab[ys, xs]

        median_hsv = tuple(float(v) for v in np.median(hsv_pixels, axis=0))
        median_lab = tuple(float(v) for v in np.median(lab_pixels, axis=0))
        return median_hsv, median_lab

    def _color_confidence(
        self,
        color_name: str,
        median_hsv: Tuple[float, float, float],
        median_lab: Tuple[float, float, float],
    ) -> float:
        h, s, v = median_hsv

        hue_distance = self._hue_distance(h, HUE_CENTRES[color_name])
        hue_score = max(0.0, 1.0 - hue_distance / 30.0)

        saturation_score = max(0.0, min(1.0, (s - 45.0) / 100.0))
        brightness_score = max(0.0, min(1.0, (v - 25.0) / 90.0))

        # Lab a/b provide a weak illumination-resistant consistency check.
        _l, a, b = median_lab
        if color_name == "green":
            lab_score = max(0.0, min(1.0, (128.0 - a) / 45.0 + 0.25))
        elif color_name == "red":
            lab_score = max(0.0, min(1.0, (a - 128.0) / 45.0 + 0.25))
        elif color_name in ("yellow", "orange"):
            lab_score = max(0.0, min(1.0, (b - 128.0) / 55.0 + 0.20))
        elif color_name == "blue":
            lab_score = max(0.0, min(1.0, (128.0 - b) / 55.0 + 0.20))
        else:
            lab_score = 0.5

        return float(
            0.50 * hue_score
            + 0.22 * saturation_score
            + 0.08 * brightness_score
            + 0.20 * lab_score
        )

    @staticmethod
    def _iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        ax2, ay2 = ax + aw, ay + ah
        bx2, by2 = bx + bw, by + bh

        ix1 = max(ax, bx)
        iy1 = max(ay, by)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)

        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = float(iw * ih)
        union = float(aw * ah + bw * bh) - inter
        return 0.0 if union <= 0.0 else inter / union

    def detect(self, frame: np.ndarray) -> Tuple[List[TargetDetection], np.ndarray]:
        normalized = self.normalize_lighting(frame)
        hsv = cv2.cvtColor(normalized, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(normalized, cv2.COLOR_BGR2LAB)

        frame_h, frame_w = frame.shape[:2]
        frame_area = float(frame_h * frame_w)
        min_area = max(
            float(self.config.target_min_contour_area_px),
            float(self.config.target_min_contour_area_ratio) * frame_area,
        )
        max_area = float(self.config.target_max_contour_area_ratio) * frame_area

        # The real-camera sample showed seven signs on the foam wall plus a
        # false YELLOW RECTANGLE on a reflective floor object.  Restrict
        # candidates to an adjustable vertical sign band.  Reject clipped
        # contours at its boundaries instead of classifying partial objects.
        roi_top = int(frame_h * float(self.config.target_roi_top_ratio))
        roi_bottom = int(frame_h * float(self.config.target_roi_bottom_ratio))
        roi_bottom = max(roi_top + 1, min(frame_h, roi_bottom))
        roi_margin = int(self.config.target_roi_border_margin_px)

        detections: List[TargetDetection] = []

        for color_name in COLOR_RANGES:
            mask = self._make_mask(hsv, color_name)
            mask[:roi_top, :] = 0
            mask[roi_bottom:, :] = 0
            found = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            contours = found[0] if len(found) == 2 else found[1]

            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < min_area or area > max_area:
                    continue

                moments = cv2.moments(contour)
                if abs(moments["m00"]) < 1e-9:
                    continue

                bbox = tuple(int(v) for v in cv2.boundingRect(contour))
                bx, by, bw, bh = bbox

                # A clipped sign (or a wall/background patch cut at the ROI
                # edge) does not have a trustworthy geometric shape.
                if (
                    bx <= roi_margin
                    or bx + bw >= frame_w - roi_margin
                    or by <= roi_top + roi_margin
                    or by + bh >= roi_bottom - roi_margin
                ):
                    continue

                shape, shape_conf, metrics = self._shape(contour)
                if shape == "unknown":
                    continue

                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])

                median_hsv, median_lab = self._inside_statistics(contour, hsv, lab)
                color_conf = self._color_confidence(
                    color_name,
                    median_hsv,
                    median_lab,
                )

                total_conf = 0.62 * color_conf + 0.38 * shape_conf
                if total_conf < float(self.config.target_min_confidence):
                    continue

                detections.append(
                    TargetDetection(
                        color=color_name,
                        shape=shape,
                        bbox=bbox,
                        centroid=(cx, cy),
                        area=metrics["area"],
                        perimeter=metrics["perimeter"],
                        vertices=metrics["vertices"],
                        aspect_ratio=metrics["aspect_ratio"],
                        circularity=metrics["circularity"],
                        rectangularity=metrics["rectangularity"],
                        median_hsv=median_hsv,
                        median_lab=median_lab,
                        color_confidence=float(color_conf),
                        shape_confidence=float(shape_conf),
                        confidence=float(total_conf),
                        contour=contour,
                    )
                )

        # Suppress duplicate detections caused by overlapping color ranges.
        detections.sort(key=lambda item: item.confidence, reverse=True)
        kept: List[TargetDetection] = []

        for detection in detections:
            duplicate = False
            for other in kept:
                if self._iou(detection.bbox, other.bbox) >= 0.65:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(detection)

        debug = frame.copy()
        cv2.rectangle(
            debug,
            (0, roi_top),
            (frame_w - 1, roi_bottom - 1),
            (180, 180, 180),
            1,
        )
        cv2.putText(
            debug,
            "TARGET ROI",
            (8, max(14, roi_top + 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (180, 180, 180),
            1,
            cv2.LINE_AA,
        )
        for index, detection in enumerate(kept, 1):
            colour = DRAW_COLOURS.get(detection.color, (255, 255, 255))
            cv2.drawContours(debug, [detection.contour], -1, colour, 2)
            x, y, w, h = detection.bbox
            cv2.rectangle(debug, (x, y), (x + w, y + h), colour, 2)
            cv2.circle(debug, detection.centroid, 4, colour, -1)
            # Full color/shape labels overlap badly when many signs share
            # one foam-wall strip. Keep numbered markers here and put a
            # matching detailed legend in the stationary camera-test window.
            cv2.putText(
                debug,
                "#{}".format(index),
                (x, max(18, y - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                colour,
                1,
                cv2.LINE_AA,
            )

        return kept, debug

    def verify_latest(
        self,
        camera_service,
        not_before: Optional[float] = None,
    ) -> Tuple[List[VerifiedTarget], Optional[np.ndarray]]:
        """Require consecutive matches on distinct fresh camera frames.

        The old version sampled CameraService.latest() several times and
        could count one cached frame repeatedly.  This uses capture timestamps
        and one-to-one associations so nearby identical signs stay separate.
        """
        tracks: List[dict] = []
        last_debug = None

        sample_count = max(
            int(self.config.target_verify_frames),
            int(self.config.target_sample_frames),
        )
        interval_sec = float(self.config.target_frame_interval_sec)
        deadline = time.monotonic() + max(
            1.0,
            sample_count * max(0.06, interval_sec) * 3.0,
        )
        used_frame_count = 0
        last_capture_timestamp: Optional[float] = None

        while used_frame_count < sample_count and time.monotonic() < deadline:
            if hasattr(camera_service, "latest_with_timestamp"):
                sample = camera_service.latest_with_timestamp(
                    max_age_sec=float(self.config.target_max_frame_age_sec)
                )
            else:
                frame = camera_service.latest(
                    max_age_sec=float(self.config.target_max_frame_age_sec)
                )
                sample = None if frame is None else (frame, time.monotonic())

            if sample is None:
                time.sleep(0.01)
                continue

            frame, capture_timestamp = sample
            # The camera looks down AFTER the horizontal ToF scan. Frames
            # decoded before pitch settled must never count toward target
            # verification or falsely turn a real floor sign into zero hits.
            if not_before is not None and float(capture_timestamp) <= float(not_before):
                time.sleep(0.01)
                continue
            if (
                last_capture_timestamp is not None
                and float(capture_timestamp) <= last_capture_timestamp
            ):
                time.sleep(0.01)
                continue

            last_capture_timestamp = float(capture_timestamp)
            used_frame_count += 1
            detections, debug = self.detect(frame)
            last_debug = debug

            next_tracks: List[dict] = []
            used_previous = set()

            for detection in detections:
                best_index = None
                best_distance = None

                for index, track in enumerate(tracks):
                    if index in used_previous:
                        continue
                    if (
                        track["color"] != detection.color
                        or track["shape"] != detection.shape
                    ):
                        continue

                    px, py = track["last_centroid"]
                    distance = math.hypot(
                        detection.centroid[0] - px,
                        detection.centroid[1] - py,
                    )
                    if distance > float(self.config.target_verify_max_jump_px):
                        continue

                    if best_distance is None or distance < best_distance:
                        best_index = index
                        best_distance = distance

                if best_index is None:
                    next_tracks.append({
                        "color": detection.color,
                        "shape": detection.shape,
                        "count": 1,
                        "confidence_sum": float(detection.confidence),
                        "last_centroid": detection.centroid,
                        "last_detection": detection,
                    })
                else:
                    used_previous.add(best_index)
                    previous = tracks[best_index]
                    next_tracks.append({
                        "color": detection.color,
                        "shape": detection.shape,
                        "count": int(previous["count"]) + 1,
                        "confidence_sum": (
                            float(previous["confidence_sum"])
                            + float(detection.confidence)
                        ),
                        "last_centroid": detection.centroid,
                        "last_detection": detection,
                    })

            # Dropping unmatched tracks enforces consecutive observations.
            tracks = next_tracks
            time.sleep(max(0.005, interval_sec))

        verified: List[VerifiedTarget] = []

        for track in tracks:
            if track["count"] < int(self.config.target_verify_frames):
                continue

            confidence = float(track["confidence_sum"]) / float(track["count"])
            if confidence < float(self.config.target_save_confidence):
                continue

            verified.append(VerifiedTarget(
                detection=track["last_detection"],
                verified_frames=int(track["count"]),
                confidence=float(confidence),
            ))

        verified.sort(key=lambda item: item.confidence, reverse=True)
        return verified, last_debug


class TargetRegistry:
    def __init__(self, config) -> None:
        self.config = config
        self.targets: List[dict] = []
        self.observations: List[dict] = []
        self._next_id = 1

    def _estimate_target_xy(
        self,
        approach_cell: Tuple[int, int],
        direction: int,
        tof_cm: Optional[float],
    ) -> Tuple[float, float]:
        cell_size = float(self.config.cell_size_m)
        base_x = float(approach_cell[0]) * cell_size
        base_y = float(approach_cell[1]) * cell_size

        dx, dy = DIR_VEC[int(direction) % 4]

        if tof_cm is None:
            distance_m = cell_size * 0.5
        else:
            distance_m = (
                float(self.config.tof_forward_offset_m)
                + max(0.0, float(tof_cm)) / 100.0
            )

        return (
            base_x + dx * distance_m,
            base_y + dy * distance_m,
        )

    def add_verified(
        self,
        verified: VerifiedTarget,
        approach_cell: Tuple[int, int],
        direction: int,
        tof_cm: Optional[float],
        range_confirmed_wall: bool = True,
        camera_pitch_deg: Optional[float] = None,
    ) -> dict:
        detection = verified.detection
        target_x, target_y = self._estimate_target_xy(
            approach_cell,
            direction,
            tof_cm,
        )

        observation = {
            "t_monotonic": time.monotonic(),
            "color": detection.color,
            "shape": detection.shape,
            "approach_cell": [int(approach_cell[0]), int(approach_cell[1])],
            "view_direction": int(direction) % 4,
            "view_direction_name": DIR_NAME[int(direction) % 4],
            "tof_cm": None if tof_cm is None else float(tof_cm),
            "range_confirmed_wall": bool(range_confirmed_wall),
            "camera_pitch_deg": (
                None if camera_pitch_deg is None else float(camera_pitch_deg)
            ),
            "estimated_target_xy_m": [target_x, target_y],
            "centroid_px": [
                int(detection.centroid[0]),
                int(detection.centroid[1]),
            ],
            "bbox_px": [int(v) for v in detection.bbox],
            "verified_frames": int(verified.verified_frames),
            "confidence": float(verified.confidence),
            "color_confidence": float(detection.color_confidence),
            "shape_confidence": float(detection.shape_confidence),
            "median_hsv": [round(float(v), 2) for v in detection.median_hsv],
            "median_lab": [round(float(v), 2) for v in detection.median_lab],
        }
        self.observations.append(observation)

        match = None

        for target in self.targets:
            if target["color"] != detection.color or target["shape"] != detection.shape:
                continue

            # A ToF reading measures distance along the gimbal ray; it cannot
            # distinguish the lateral positions of several signs on one wall.
            # Merge only repeat detections from the SAME approach cell and
            # SAME viewing direction with a sufficiently close image centroid.
            # Cross-cell association needs calibrated camera geometry and is
            # deliberately deferred instead of silently losing real targets.
            for view in target.get("reference_views", []):
                if (
                    view["approach_cell"] != observation["approach_cell"]
                    or int(view["view_direction"]) != int(direction) % 4
                ):
                    continue
                old_cx, old_cy = view["centroid_px"]
                distance_px = math.hypot(
                    detection.centroid[0] - old_cx,
                    detection.centroid[1] - old_cy,
                )
                if distance_px <= float(self.config.target_merge_centroid_px):
                    match = target
                    break

            if match is not None:
                break

        if match is None:
            match = {
                "target_id": "T{:02d}".format(self._next_id),
                "color": detection.color,
                "shape": detection.shape,
                "estimated_target_xy_m": [target_x, target_y],
                "approach_cells": [list(observation["approach_cell"])],
                "view_directions": [int(direction) % 4],
                "view_direction_names": [DIR_NAME[int(direction) % 4]],
                "reference_views": [{
                    "approach_cell": list(observation["approach_cell"]),
                    "view_direction": int(direction) % 4,
                    "centroid_px": list(observation["centroid_px"]),
                }],
                "observations": 1,
                "confidence": float(verified.confidence),
                "range_confirmed_wall": bool(range_confirmed_wall),
                "status": (
                    "DETECTED" if range_confirmed_wall else "NEEDS_RANGE_REVIEW"
                ),
            }
            self._next_id += 1
            self.targets.append(match)
        else:
            count = int(match["observations"])
            old_x, old_y = match["estimated_target_xy_m"]

            match["estimated_target_xy_m"] = [
                (old_x * count + target_x) / float(count + 1),
                (old_y * count + target_y) / float(count + 1),
            ]
            match["observations"] = count + 1
            if range_confirmed_wall:
                match["range_confirmed_wall"] = True
                match["status"] = "DETECTED"
            match["confidence"] = max(
                float(match["confidence"]),
                float(verified.confidence),
            )

            if list(observation["approach_cell"]) not in match["approach_cells"]:
                match["approach_cells"].append(list(observation["approach_cell"]))

            if int(direction) % 4 not in match["view_directions"]:
                match["view_directions"].append(int(direction) % 4)
                match["view_direction_names"].append(
                    DIR_NAME[int(direction) % 4]
                )

        return match

    def public_targets(self) -> List[dict]:
        return json.loads(json.dumps(self.targets))

    def save(self, run_dir: Path) -> None:
        run_dir = Path(run_dir)
        payload = {
            "version": 1,
            "target_count": len(self.targets),
            "targets": self.targets,
            "observations": self.observations,
        }
        (run_dir / "targets.json").write_text(
            json.dumps(payload, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )


def save_topology(
    run_dir: Path,
    *,
    cell_size_m: float,
    start_cell: Tuple[int, int],
    final_cell: Tuple[int, int],
    visited: Iterable[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    traversed_edges: Iterable[Tuple[Tuple[int, int], Tuple[int, int]]],
    start_scan_ranges: Optional[Dict[int, Optional[float]]],
    finish_reason: str,
) -> None:
    open_edges = []
    wall_edges = []

    for (x, y, direction), state in sorted(edge_states.items()):
        row = [int(x), int(y), int(direction)]
        if state == "OPEN":
            open_edges.append(row)
        elif state == "WALL":
            wall_edges.append(row)

    payload = {
        "version": 1,
        "cell_size_m": float(cell_size_m),
        "start_cell": [int(start_cell[0]), int(start_cell[1])],
        "final_cell": [int(final_cell[0]), int(final_cell[1])],
        "finish_reason": str(finish_reason),
        "visited_cells": [
            [int(cell[0]), int(cell[1])]
            for cell in sorted(set(visited))
        ],
        "open_edges": open_edges,
        "wall_edges": wall_edges,
        "traversed_edges": [
            [
                [int(a[0]), int(a[1])],
                [int(b[0]), int(b[1])],
            ]
            for a, b in sorted(set(traversed_edges))
        ],
        "start_scan_ranges_cm": (
            None
            if start_scan_ranges is None
            else {
                DIR_NAME[int(direction) % 4]: (
                    None if value is None else float(value)
                )
                for direction, value in start_scan_ranges.items()
            }
        ),
    }

    Path(run_dir, "topology.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
