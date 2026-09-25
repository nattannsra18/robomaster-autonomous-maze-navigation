"""Robust visual verification that a previously observed target moved/downed.

This module does not control the blaster, chassis or gimbal. It only compares
camera evidence before and after an operator-triggered physical action.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import cv2
import numpy as np


STATUS_IDLE = "IDLE"
STATUS_BASELINE_READY = "BASELINE_READY"
STATUS_WAIT_SETTLE = "WAIT_SETTLE"
STATUS_CHECKING = "CHECKING"
STATUS_STILL_UP = "STILL_UP"
STATUS_DOWN_CONFIRMED = "DOWN_CONFIRMED"
STATUS_UNCERTAIN = "UNCERTAIN"


@dataclass
class TargetDownConfig:
    roi_margin_ratio: float = 0.35
    settle_sec: float = 0.60
    timeout_sec: float = 4.0

    # Temporal confirmation. One bad frame is never enough.
    down_confirm_frames: int = 6
    still_up_confirm_frames: int = 4

    # Relative to the target-color pixel count captured before the action.
    low_color_fraction: float = 0.22

    # Exact target geometry still overlapping its former position => still up.
    exact_iou_still_up: float = 0.22
    exact_area_ratio_min: float = 0.42
    exact_area_ratio_max: float = 1.90

    # Geometry-change cues for a down/moved sign.
    area_ratio_down: float = 0.45
    center_shift_height_ratio: float = 0.35
    aspect_log_change: float = 0.45

    # Lighting-normalized Lab-patch difference in the baseline ROI.
    patch_change_min: float = 0.10

    # If ambient brightness changed strongly, color disappearance becomes weak
    # evidence and structural change is required before confirming DOWN.
    lighting_change_warn: float = 0.14


@dataclass
class TargetDownResult:
    status: str
    confidence: float
    elapsed_sec: float
    color_remaining_fraction: float
    exact_target_present: bool
    best_iou: float
    best_area_ratio: float
    center_shift_ratio: float
    aspect_log_change: float
    patch_change: float
    lighting_change: float
    down_streak: int
    still_up_streak: int
    detail: str


class TargetDownVerifier:
    """Temporal verifier for TARGET STILL UP / TARGET DOWN / UNCERTAIN."""

    def __init__(self, config: Optional[TargetDownConfig] = None) -> None:
        self.config = config or TargetDownConfig()
        self.reset()

    def reset(self) -> None:
        self.status = STATUS_IDLE
        self.target_color: Optional[str] = None
        self.target_shape: Optional[str] = None
        self.baseline_bbox: Optional[Tuple[int, int, int, int]] = None
        self.roi: Optional[Tuple[int, int, int, int]] = None
        self.baseline_area = 0.0
        self.baseline_aspect = 1.0
        self.baseline_center = (0.0, 0.0)
        self.baseline_color_pixels = 0
        self.baseline_patch: Optional[np.ndarray] = None
        self.baseline_light = 0.0
        self.verify_started = 0.0
        self.down_streak = 0
        self.still_up_streak = 0
        self.last_result: Optional[TargetDownResult] = None

    @staticmethod
    def _bbox_iou(a, b) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        ax2, ay2 = ax + aw, ay + ah
        bx2, by2 = bx + bw, by + bh

        ix1, iy1 = max(ax, bx), max(ay, by)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
        inter = float(iw * ih)
        union = float(aw * ah + bw * bh) - inter
        return inter / union if union > 0.0 else 0.0

    @staticmethod
    def _normalize_patch(patch: np.ndarray) -> np.ndarray:
        """Return a lighting-normalized Lab patch for structural comparison.

        L is CLAHE-normalized while a/b retain chroma. This makes the change
        score sensitive to a colored sign disappearing even when its grayscale
        brightness is close to the background, while remaining less sensitive
        to global illumination changes.
        """
        lab = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB)
        l_chan, a_chan, b_chan = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_chan = clahe.apply(l_chan)
        merged = cv2.merge((l_chan, a_chan, b_chan))
        return cv2.GaussianBlur(merged, (5, 5), 0)

    @staticmethod
    def _median_light(frame: np.ndarray) -> float:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return float(np.median(hsv[:, :, 2])) / 255.0

    def capture_baseline(
        self,
        frame: np.ndarray,
        target_mask: np.ndarray,
        detection,
    ) -> None:
        """Store the upright target's visual state before a manual action."""
        h, w = frame.shape[:2]
        x, y, bw, bh = [int(v) for v in detection.bbox]
        margin = int(round(max(bw, bh) * float(self.config.roi_margin_ratio)))

        x1 = max(0, x - margin)
        y1 = max(0, y - margin)
        x2 = min(w, x + bw + margin)
        y2 = min(h, y + bh + margin)

        if x2 <= x1 or y2 <= y1:
            raise ValueError("Target baseline ROI is empty")

        self.target_color = str(detection.color)
        self.target_shape = str(detection.shape)
        self.baseline_bbox = (x, y, bw, bh)
        self.roi = (x1, y1, x2, y2)
        self.baseline_area = max(1.0, float(detection.area))
        self.baseline_aspect = max(1e-6, float(detection.aspect_ratio))
        self.baseline_center = (
            float(detection.centroid[0]),
            float(detection.centroid[1]),
        )
        self.baseline_color_pixels = max(
            1,
            int(cv2.countNonZero(target_mask[y1:y2, x1:x2])),
        )
        self.baseline_patch = self._normalize_patch(frame[y1:y2, x1:x2])
        self.baseline_light = self._median_light(frame)
        self.status = STATUS_BASELINE_READY
        self.verify_started = 0.0
        self.down_streak = 0
        self.still_up_streak = 0
        self.last_result = None

    def begin_verification(self, now: Optional[float] = None) -> None:
        """Start/restart the observation window after a manual action."""
        if self.status not in (
            STATUS_BASELINE_READY,
            STATUS_STILL_UP,
            STATUS_UNCERTAIN,
        ):
            raise RuntimeError("Capture a baseline before starting verification")

        self.verify_started = float(time.monotonic() if now is None else now)
        self.status = STATUS_WAIT_SETTLE
        self.down_streak = 0
        self.still_up_streak = 0

    def _make_result(
        self,
        *,
        status: str,
        confidence: float,
        elapsed: float,
        color_fraction: float,
        exact_present: bool,
        best_iou: float,
        area_ratio: float,
        center_shift: float,
        aspect_change: float,
        patch_change: float,
        lighting_change: float,
        detail: str,
    ) -> TargetDownResult:
        result = TargetDownResult(
            status=status,
            confidence=max(0.0, min(1.0, float(confidence))),
            elapsed_sec=float(elapsed),
            color_remaining_fraction=float(color_fraction),
            exact_target_present=bool(exact_present),
            best_iou=float(best_iou),
            best_area_ratio=float(area_ratio),
            center_shift_ratio=float(center_shift),
            aspect_log_change=float(aspect_change),
            patch_change=float(patch_change),
            lighting_change=float(lighting_change),
            down_streak=int(self.down_streak),
            still_up_streak=int(self.still_up_streak),
            detail=str(detail),
        )
        self.last_result = result
        self.status = status
        return result

    def update(
        self,
        frame: np.ndarray,
        target_mask: np.ndarray,
        detections: Iterable,
        now: Optional[float] = None,
    ) -> TargetDownResult:
        """Evaluate one frame without making any robot-control decision."""
        if self.roi is None or self.baseline_bbox is None or self.baseline_patch is None:
            raise RuntimeError("Baseline has not been captured")
        if self.verify_started <= 0.0:
            raise RuntimeError("Verification has not been started")

        now_value = float(time.monotonic() if now is None else now)
        elapsed = max(0.0, now_value - self.verify_started)

        x1, y1, x2, y2 = self.roi
        roi_mask = target_mask[y1:y2, x1:x2]
        current_color_pixels = int(cv2.countNonZero(roi_mask))
        color_fraction = current_color_pixels / float(self.baseline_color_pixels)

        current_patch = self._normalize_patch(frame[y1:y2, x1:x2])
        if current_patch.shape != self.baseline_patch.shape:
            current_patch = cv2.resize(
                current_patch,
                (self.baseline_patch.shape[1], self.baseline_patch.shape[0]),
                interpolation=cv2.INTER_AREA,
            )

        patch_change = float(
            np.mean(cv2.absdiff(self.baseline_patch, current_patch)) / 255.0
        )
        lighting_change = abs(self._median_light(frame) - self.baseline_light)

        roi_exact = []
        roi_same_color = []

        for detection in detections:
            cx, cy = detection.centroid
            inside = x1 <= cx < x2 and y1 <= cy < y2
            if not inside:
                continue

            if str(detection.color) == self.target_color:
                roi_same_color.append(detection)
                if str(detection.shape) == self.target_shape:
                    roi_exact.append(detection)

        best_exact = None
        best_iou = 0.0

        if roi_exact:
            best_exact = max(
                roi_exact,
                key=lambda d: self._bbox_iou(self.baseline_bbox, d.bbox),
            )
            best_iou = self._bbox_iou(self.baseline_bbox, best_exact.bbox)

        candidate = best_exact
        if candidate is None and roi_same_color:
            candidate = max(roi_same_color, key=lambda d: float(d.area))

        area_ratio = 0.0
        center_shift = 0.0
        aspect_change = 0.0

        if candidate is not None:
            area_ratio = float(candidate.area) / self.baseline_area

            dx = float(candidate.centroid[0]) - self.baseline_center[0]
            dy = float(candidate.centroid[1]) - self.baseline_center[1]
            baseline_h = max(1.0, float(self.baseline_bbox[3]))
            center_shift = math.hypot(dx, dy) / baseline_h

            current_aspect = max(1e-6, float(candidate.aspect_ratio))
            aspect_change = abs(math.log(current_aspect / self.baseline_aspect))

        exact_present = best_exact is not None

        still_up = (
            exact_present
            and best_iou >= self.config.exact_iou_still_up
            and self.config.exact_area_ratio_min
            <= area_ratio
            <= self.config.exact_area_ratio_max
        )

        geometry_changed = (
            candidate is None
            or area_ratio <= self.config.area_ratio_down
            or center_shift >= self.config.center_shift_height_ratio
            or aspect_change >= self.config.aspect_log_change
        )

        color_low = color_fraction <= self.config.low_color_fraction
        structural_change = patch_change >= self.config.patch_change_min

        # Color loss alone is not enough. This specifically prevents a sudden
        # lighting/HSV failure from being interpreted as a fallen target.
        if lighting_change >= self.config.lighting_change_warn:
            down_evidence = (
                (not exact_present)
                and geometry_changed
                and structural_change
            )
        else:
            down_evidence = (
                (not exact_present)
                and geometry_changed
                and structural_change
                and color_low
            )

        if elapsed < self.config.settle_sec:
            self.down_streak = 0
            self.still_up_streak = 0
            return self._make_result(
                status=STATUS_WAIT_SETTLE,
                confidence=0.0,
                elapsed=elapsed,
                color_fraction=color_fraction,
                exact_present=exact_present,
                best_iou=best_iou,
                area_ratio=area_ratio,
                center_shift=center_shift,
                aspect_change=aspect_change,
                patch_change=patch_change,
                lighting_change=lighting_change,
                detail="Waiting for scene to settle",
            )

        if down_evidence:
            self.down_streak += 1
            self.still_up_streak = 0
        elif still_up:
            self.still_up_streak += 1
            self.down_streak = 0
        else:
            self.down_streak = max(0, self.down_streak - 1)
            self.still_up_streak = max(0, self.still_up_streak - 1)

        down_score = (
            0.30 * min(1.0, max(0.0, 1.0 - color_fraction))
            + 0.30
            * min(
                1.0,
                patch_change
                / max(1e-6, self.config.patch_change_min * 2.0),
            )
            + 0.20 * (1.0 if geometry_changed else 0.0)
            + 0.20 * (0.0 if exact_present else 1.0)
        )

        if self.down_streak >= self.config.down_confirm_frames:
            return self._make_result(
                status=STATUS_DOWN_CONFIRMED,
                confidence=down_score,
                elapsed=elapsed,
                color_fraction=color_fraction,
                exact_present=exact_present,
                best_iou=best_iou,
                area_ratio=area_ratio,
                center_shift=center_shift,
                aspect_change=aspect_change,
                patch_change=patch_change,
                lighting_change=lighting_change,
                detail=(
                    "Target no longer matches its original "
                    "visual/geometry state"
                ),
            )

        if self.still_up_streak >= self.config.still_up_confirm_frames:
            still_score = min(
                1.0,
                0.45
                + 0.30 * best_iou
                + 0.25 * min(1.0, area_ratio),
            )
            return self._make_result(
                status=STATUS_STILL_UP,
                confidence=still_score,
                elapsed=elapsed,
                color_fraction=color_fraction,
                exact_present=exact_present,
                best_iou=best_iou,
                area_ratio=area_ratio,
                center_shift=center_shift,
                aspect_change=aspect_change,
                patch_change=patch_change,
                lighting_change=lighting_change,
                detail=(
                    "Original target is still visible "
                    "near its baseline position"
                ),
            )

        if elapsed >= self.config.timeout_sec:
            return self._make_result(
                status=STATUS_UNCERTAIN,
                confidence=max(down_score, 0.25),
                elapsed=elapsed,
                color_fraction=color_fraction,
                exact_present=exact_present,
                best_iou=best_iou,
                area_ratio=area_ratio,
                center_shift=center_shift,
                aspect_change=aspect_change,
                patch_change=patch_change,
                lighting_change=lighting_change,
                detail="Timeout without enough consistent evidence",
            )

        return self._make_result(
            status=STATUS_CHECKING,
            confidence=down_score,
            elapsed=elapsed,
            color_fraction=color_fraction,
            exact_present=exact_present,
            best_iou=best_iou,
            area_ratio=area_ratio,
            center_shift=center_shift,
            aspect_change=aspect_change,
            patch_change=patch_change,
            lighting_change=lighting_change,
            detail="Collecting temporal evidence",
        )
