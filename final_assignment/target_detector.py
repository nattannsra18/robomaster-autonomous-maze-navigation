"""Lighting-robust color + shape detection for the RoboMaster final assignment.

This module is independent from chassis, gimbal and blaster code. It can be
reused by Round 1 target mapping and Round 2 target selection.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


COLOR_RANGES: Dict[
    str,
    List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]],
] = {
    "red": [
        ((0, 80, 45), (12, 255, 255)),
        ((168, 80, 45), (179, 255, 255)),
    ],
    "green": [
        ((35, 55, 35), (90, 255, 255)),
    ],
    "blue": [
        ((90, 65, 35), (135, 255, 255)),
    ],
    "yellow": [
        ((20, 65, 55), (38, 255, 255)),
    ],
    "orange": [
        ((7, 80, 50), (22, 255, 255)),
    ],
}

DRAW_COLORS = {
    "red": (0, 0, 255),
    "green": (0, 220, 0),
    "blue": (255, 80, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 165, 255),
}

MIN_CONTOUR_AREA_PX = 90.0
MIN_CONTOUR_AREA_RATIO = 0.00015
MAX_CONTOUR_AREA_RATIO = 0.35

MORPH_KERNEL_SIZE = 3
APPROX_EPSILON_RATIO = 0.030
RECTANGULARITY_MIN = 0.58
SQUARE_ASPECT_MIN = 0.72
SQUARE_ASPECT_MAX = 1.38
CIRCLE_CIRCULARITY_MIN = 0.72
CIRCLE_MIN_VERTICES = 6


@dataclass
class Detection:
    color: str
    shape: str
    contour: np.ndarray
    bbox: Tuple[int, int, int, int]
    centroid: Tuple[int, int]
    area: float
    perimeter: float
    vertices: int
    aspect_ratio: float
    circularity: float
    rectangularity: float
    color_fill_ratio: float = 0.0


def normalize_lighting(frame: np.ndarray) -> np.ndarray:
    """Normalize brightness while preserving chromatic information.

    CLAHE is applied only to the L channel in Lab. Equalizing B/G/R channels
    independently would distort the target color and is intentionally avoided.
    """
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l_chan, a_chan, b_chan = cv2.split(lab)

    clahe = cv2.createCLAHE(
        clipLimit=2.0,
        tileGridSize=(8, 8),
    )
    l_chan = clahe.apply(l_chan)

    balanced = cv2.merge(
        (l_chan, a_chan, b_chan)
    )
    return cv2.cvtColor(
        balanced,
        cv2.COLOR_LAB2BGR,
    )


def _raw_color_mask(
    hsv: np.ndarray,
    color_name: str,
) -> np.ndarray:
    mask = np.zeros(
        hsv.shape[:2],
        dtype=np.uint8,
    )

    for lower, upper in COLOR_RANGES[color_name]:
        part = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = cv2.bitwise_or(
            mask,
            part,
        )

    return mask


def make_color_mask(
    frame: np.ndarray,
    color_name: str,
) -> np.ndarray:
    """Build a color mask from original + normalized images.

    Using both versions makes detection less brittle when the same sign moves
    between brighter and darker parts of the arena.
    """
    hsv_raw = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV,
    )

    normalized = normalize_lighting(frame)
    hsv_normalized = cv2.cvtColor(
        normalized,
        cv2.COLOR_BGR2HSV,
    )

    mask = cv2.bitwise_or(
        _raw_color_mask(hsv_raw, color_name),
        _raw_color_mask(hsv_normalized, color_name),
    )

    kernel_size = max(
        3,
        int(MORPH_KERNEL_SIZE),
    )

    if kernel_size % 2 == 0:
        kernel_size += 1

    kernel = np.ones(
        (kernel_size, kernel_size),
        dtype=np.uint8,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=1,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=2,
    )

    return mask


def contour_centroid(
    contour: np.ndarray,
) -> Optional[Tuple[int, int]]:
    moments = cv2.moments(contour)

    if abs(moments["m00"]) < 1e-9:
        return None

    return (
        int(moments["m10"] / moments["m00"]),
        int(moments["m01"] / moments["m00"]),
    )


def classify_shape(
    contour: np.ndarray,
) -> Tuple[str, dict]:
    area = float(
        cv2.contourArea(contour)
    )

    perimeter = float(
        cv2.arcLength(
            contour,
            True,
        )
    )

    if area <= 0.0 or perimeter <= 0.0:
        return "unknown", {}

    approx = cv2.approxPolyDP(
        contour,
        APPROX_EPSILON_RATIO * perimeter,
        True,
    )

    vertices = len(approx)

    x, y, w, h = cv2.boundingRect(contour)

    aspect_ratio = (
        float(w) / float(h)
        if h > 0
        else 0.0
    )

    bbox_area = float(w * h)

    rectangularity = (
        area / bbox_area
        if bbox_area > 0.0
        else 0.0
    )

    circularity = (
        4.0
        * math.pi
        * area
        / (perimeter * perimeter)
    )

    shape = "unknown"

    if (
        vertices == 4
        and cv2.isContourConvex(approx)
        and rectangularity >= RECTANGULARITY_MIN
    ):
        shape = (
            "square"
            if SQUARE_ASPECT_MIN
            <= aspect_ratio
            <= SQUARE_ASPECT_MAX
            else "rectangle"
        )

    elif (
        vertices >= CIRCLE_MIN_VERTICES
        and circularity
        >= CIRCLE_CIRCULARITY_MIN
    ):
        shape = "circle"

    return shape, {
        "area": area,
        "perimeter": perimeter,
        "vertices": vertices,
        "aspect_ratio": aspect_ratio,
        "circularity": circularity,
        "rectangularity": rectangularity,
    }


def detect_objects(
    frame: np.ndarray,
) -> Tuple[List[Detection], Dict[str, np.ndarray]]:
    frame_area = float(
        frame.shape[0]
        * frame.shape[1]
    )

    min_area = max(
        MIN_CONTOUR_AREA_PX,
        MIN_CONTOUR_AREA_RATIO
        * frame_area,
    )

    max_area = (
        MAX_CONTOUR_AREA_RATIO
        * frame_area
    )

    detections: List[Detection] = []
    masks: Dict[str, np.ndarray] = {}

    for color_name in COLOR_RANGES:
        mask = make_color_mask(
            frame,
            color_name,
        )

        masks[color_name] = mask

        found = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        contours = (
            found[0]
            if len(found) == 2
            else found[1]
        )

        for contour in contours:
            area = float(
                cv2.contourArea(contour)
            )

            if (
                area < min_area
                or area > max_area
            ):
                continue

            centroid = contour_centroid(
                contour
            )

            if centroid is None:
                continue

            shape, metrics = classify_shape(
                contour
            )

            if shape == "unknown":
                continue

            x, y, w, h = cv2.boundingRect(
                contour
            )

            bbox_area = max(
                1,
                w * h,
            )

            fill_ratio = (
                float(
                    cv2.countNonZero(
                        mask[
                            y : y + h,
                            x : x + w,
                        ]
                    )
                )
                / float(bbox_area)
            )

            detections.append(
                Detection(
                    color=color_name,
                    shape=shape,
                    contour=contour,
                    bbox=(x, y, w, h),
                    centroid=centroid,
                    area=metrics["area"],
                    perimeter=metrics[
                        "perimeter"
                    ],
                    vertices=metrics[
                        "vertices"
                    ],
                    aspect_ratio=metrics[
                        "aspect_ratio"
                    ],
                    circularity=metrics[
                        "circularity"
                    ],
                    rectangularity=metrics[
                        "rectangularity"
                    ],
                    color_fill_ratio=fill_ratio,
                )
            )

    return detections, masks


def is_exact_target(
    detection: Detection,
    target_color: str,
    target_shape: str,
) -> bool:
    return (
        detection.color == target_color
        and detection.shape == target_shape
    )


def choose_target_match(
    detections: List[Detection],
    target_color: str,
    target_shape: str,
    frame_shape,
) -> Optional[Detection]:
    matches = [
        detection
        for detection in detections
        if is_exact_target(
            detection,
            target_color,
            target_shape,
        )
    ]

    if not matches:
        return None

    frame_h, frame_w = frame_shape[:2]
    center_x = frame_w / 2.0
    center_y = frame_h / 2.0

    matches.sort(
        key=lambda detection: (
            math.hypot(
                detection.centroid[0] - center_x,
                detection.centroid[1] - center_y,
            ),
            -detection.area,
        )
    )

    return matches[0]


def draw_detection(
    frame: np.ndarray,
    detection: Detection,
    thickness: int = 2,
) -> None:
    color = DRAW_COLORS.get(
        detection.color,
        (255, 255, 255),
    )

    cv2.drawContours(
        frame,
        [detection.contour],
        -1,
        color,
        thickness,
    )

    x, y, w, h = detection.bbox

    cv2.rectangle(
        frame,
        (x, y),
        (x + w, y + h),
        color,
        thickness,
    )

    cv2.circle(
        frame,
        detection.centroid,
        4,
        color,
        -1,
    )

    label = "{} {}".format(
        detection.color.upper(),
        detection.shape.upper(),
    )

    cv2.putText(
        frame,
        label,
        (
            x,
            max(18, y - 6),
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
        cv2.LINE_AA,
    )
