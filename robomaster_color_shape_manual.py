"""RoboMaster color + shape target search with manual final aiming/firing.

What this program automates:
- runtime target selection (color + shape)
- runtime strafe direction selection
- RoboMaster camera streaming through OpenCV/FFmpeg
- HSV color segmentation
- morphology + contour extraction
- circle / rectangle / square classification
- multi-frame verification
- chassis stop when the requested target is verified
- duplicate-target suppression after the operator marks a target handled
- HSV calibration mode

What this program intentionally does NOT automate:
- it does not point the gimbal at a projectile target
- it does not call blaster.fire()
- it does not issue any firing command

When a target is VERIFIED the chassis stops and the operator handles final
gimbal positioning / firing manually. SPACE only marks the target as handled
and resumes the mission; it never fires the blaster.
"""

from __future__ import annotations

import math
import sys
import threading
import time
import types
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Optional RoboMaster media-codec shim.
#
# We use the SDK only to enable the H.264 stream, then OpenCV/FFmpeg decodes
# the raw TCP stream directly. This matches the Classwork 8 camera test.
# ---------------------------------------------------------------------------

def _prepare_optional_media_codec() -> None:
    try:
        __import__("libmedia_codec")
        return
    except ModuleNotFoundError:
        pass

    media_codec = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    media_codec.H264Decoder = H264Decoder
    media_codec.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media_codec


_prepare_optional_media_codec()

from robomaster import robot


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CONNECTION_TYPE = "ap"

CAMERA_RESOLUTION = "360p"
CAMERA_START_TIMEOUT_SEC = 5.0
CAMERA_WARMUP_SEC = 2.5

STRAFE_SPEED_MPS = 0.10

# RoboMaster chassis +Y is normally rightward strafe. If the physical result
# is reversed on your setup, change only this sign.
STRAFE_Y_SIGN = 1.0

CHASSIS_COMMAND_TIMEOUT_SEC = 0.30
CHASSIS_COMMAND_PERIOD_SEC = 0.10

# Mission ends after this much actual SEARCH/CLEAR travel time.
MAX_TRAVEL_TIME = 45.0

# Color / contour filtering.
MIN_CONTOUR_AREA_PX = 450.0
MIN_CONTOUR_AREA_RATIO = 0.0010
MAX_CONTOUR_AREA_RATIO = 0.40

MORPH_KERNEL_SIZE = 5
APPROX_EPSILON_RATIO = 0.020

RECTANGULARITY_MIN = 0.68
SQUARE_ASPECT_MIN = 0.82
SQUARE_ASPECT_MAX = 1.22

CIRCLE_CIRCULARITY_MIN = 0.72
CIRCLE_MIN_VERTICES = 6

# Verification must persist across multiple camera frames.
MATCH_VERIFY_FRAMES = 4
VERIFY_MAX_CENTER_JUMP_PX = 130.0

# After SPACE/N, ignore the same target until it disappears for several
# consecutive frames and the robot has moved for a short time.
CLEAR_MISSING_FRAMES = 8
CLEAR_MIN_TRAVEL_SEC = 0.50

# Neutral gimbal return after a handled/skipped target.
RECENTER_KP = 1.8
RECENTER_MAX_SPEED_DPS = 80.0
RECENTER_MIN_SPEED_DPS = 10.0
RECENTER_TOLERANCE_DEG = 1.5
RECENTER_STABLE_SAMPLES = 4
RECENTER_TIMEOUT_SEC = 4.0


# OpenCV HSV:
# H: 0..179
# S: 0..255
# V: 0..255
#
# These are intentionally broad starting values. Tune in the real room.
COLOR_RANGES: Dict[
    str,
    List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]],
] = {
    "red": [
        ((0, 90, 70), (10, 255, 255)),
        ((170, 90, 70), (179, 255, 255)),
    ],
    "green": [
        ((35, 65, 55), (85, 255, 255)),
    ],
    "blue": [
        ((90, 80, 55), (132, 255, 255)),
    ],
    "yellow": [
        ((22, 80, 80), (36, 255, 255)),
    ],
    "orange": [
        ((8, 100, 80), (21, 255, 255)),
    ],
}

DRAW_COLORS = {
    "red": (0, 0, 255),
    "green": (0, 220, 0),
    "blue": (255, 80, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 165, 255),
}

STATE_SEARCH = "SEARCH"
STATE_READY_MANUAL = "READY_MANUAL"
STATE_CLEAR = "CLEAR"


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


# ---------------------------------------------------------------------------
# Runtime menus
# ---------------------------------------------------------------------------

def _select_number(prompt: str, choices: Dict[str, str]) -> str:
    while True:
        for key, label in choices.items():
            print("{}. {}".format(key, label))

        value = input(prompt).strip()

        if value in choices:
            return choices[value].lower()

        print("Invalid input. Choose: {}".format(", ".join(choices.keys())))
        print()


def select_target() -> Tuple[str, str]:
    print()
    print("==============================")
    print("RoboMaster Target Selection")
    print("==============================")
    print()
    print("Select target color:")

    target_color = _select_number(
        "Enter color: ",
        {
            "1": "Red",
            "2": "Green",
            "3": "Blue",
            "4": "Yellow",
            "5": "Orange",
        },
    )

    print()
    print("Select target shape:")

    target_shape = _select_number(
        "Enter shape: ",
        {
            "1": "Circle",
            "2": "Rectangle",
            "3": "Square",
        },
    )

    print()
    print("Selected target:")
    print("{} + {}".format(target_color.upper(), target_shape.upper()))

    return target_color, target_shape


def select_direction() -> Tuple[int, str]:
    print()
    print("==============================")
    print("Mission Direction")
    print("==============================")
    print("1. Left to Right")
    print("2. Right to Left")

    while True:
        value = input("Enter direction: ").strip()

        if value == "1":
            return 1, "LEFT -> RIGHT"

        if value == "2":
            return -1, "RIGHT -> LEFT"

        print("Invalid input. Enter 1 or 2.")


# ---------------------------------------------------------------------------
# Latest-frame RoboMaster camera
# ---------------------------------------------------------------------------

class LatestFrameCamera:
    def __init__(self, ep_robot, resolution: str) -> None:
        self.robot = ep_robot
        self.camera = ep_robot.camera
        self.resolution = str(resolution)

        self._cap = None
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_timestamp = 0.0

        self._stream_enabled = False
        self._running = False

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> None:
        robot_ip = getattr(self.robot, "ip", None) or "192.168.2.1"
        port = int(self.camera.conf.video_stream_port)

        print(
            "[CAMERA] Enabling H.264 stream at {}:{} ({})...".format(
                robot_ip,
                port,
                self.resolution,
            ),
            flush=True,
        )

        if not self.camera._stream_sdk(1, self.resolution):
            raise RuntimeError("RoboMaster SDK stream-mode command failed")

        if not self.camera._video_stream(1, self.resolution):
            try:
                self.camera._stream_sdk(0, self.resolution)
            except Exception:
                pass
            raise RuntimeError("RoboMaster SDK video-stream command failed")

        self._stream_enabled = True

        url = "tcp://{}:{}".format(robot_ip, port)

        params = []
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            params.extend([cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000])
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            params.extend([cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000])

        try:
            if params:
                cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
            else:
                cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        except TypeError:
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

        if not cap.isOpened():
            cap.release()
            self.stop()
            raise RuntimeError("OpenCV could not open {}".format(url))

        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        first_frame = None
        deadline = time.monotonic() + CAMERA_START_TIMEOUT_SEC

        while time.monotonic() < deadline:
            ok, frame = cap.read()
            if ok and frame is not None and frame.size:
                first_frame = frame
                break

        if first_frame is None:
            cap.release()
            self.stop()
            raise RuntimeError("Camera opened but no decoded frame arrived")

        self._cap = cap

        with self._lock:
            self._latest_frame = first_frame.copy()
            self._latest_timestamp = time.monotonic()

        self._stop_event.clear()
        self._running = True

        self._thread = threading.Thread(
            target=self._capture_loop,
            name="robomaster-target-camera",
            daemon=True,
        )
        self._thread.start()

        h, w = first_frame.shape[:2]
        print("[CAMERA] Active: {}x{}".format(w, h), flush=True)

    def _capture_loop(self) -> None:
        while not self._stop_event.is_set():
            cap = self._cap
            if cap is None:
                break

            try:
                ok, frame = cap.read()
            except Exception:
                ok, frame = False, None

            if not ok or frame is None or not frame.size:
                time.sleep(0.005)
                continue

            with self._lock:
                self._latest_frame = frame.copy()
                self._latest_timestamp = time.monotonic()

        self._running = False

    def read_latest(self, max_age_sec: float = 0.5) -> Optional[np.ndarray]:
        with self._lock:
            frame = self._latest_frame
            timestamp = self._latest_timestamp

        if frame is None:
            return None

        if time.monotonic() - timestamp > max_age_sec:
            return None

        return frame.copy()

    def stop(self) -> None:
        self._stop_event.set()
        self._running = False

        cap = self._cap
        self._cap = None

        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

        thread = self._thread
        self._thread = None

        if (
            thread is not None
            and thread.is_alive()
            and threading.current_thread() is not thread
        ):
            thread.join(timeout=1.5)

        if self._stream_enabled:
            try:
                self.camera._video_stream(0, self.resolution)
            except Exception:
                pass

            try:
                self.camera._stream_sdk(0, self.resolution)
            except Exception:
                pass

            self._stream_enabled = False


# ---------------------------------------------------------------------------
# Gimbal neutral-position helper
# ---------------------------------------------------------------------------

class GimbalAngleTracker:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.pitch = None
        self.yaw = None

    def callback(self, data) -> None:
        try:
            if data is None or len(data) < 2:
                return

            pitch = float(data[0])
            yaw = float(data[1])

            with self._lock:
                self.pitch = pitch
                self.yaw = yaw
        except Exception:
            pass

    def get(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.pitch, self.yaw


def _bounded_speed(
    error: float,
    kp: float,
    min_speed: float,
    max_speed: float,
) -> float:
    if abs(error) < 1e-9:
        return 0.0

    magnitude = min(max_speed, max(min_speed, abs(error) * kp))
    return math.copysign(magnitude, error)


def stop_gimbal(gimbal) -> None:
    try:
        gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
    except Exception:
        pass


def recenter_gimbal(gimbal, tracker: GimbalAngleTracker) -> bool:
    start = time.monotonic()
    stable = 0

    while time.monotonic() - start < RECENTER_TIMEOUT_SEC:
        pitch, yaw = tracker.get()

        if pitch is None or yaw is None:
            time.sleep(0.03)
            continue

        pitch_error = -pitch
        yaw_error = -yaw

        if (
            abs(pitch_error) <= RECENTER_TOLERANCE_DEG
            and abs(yaw_error) <= RECENTER_TOLERANCE_DEG
        ):
            stable += 1
            stop_gimbal(gimbal)

            if stable >= RECENTER_STABLE_SAMPLES:
                return True
        else:
            stable = 0

            pitch_cmd = _bounded_speed(
                pitch_error,
                RECENTER_KP,
                RECENTER_MIN_SPEED_DPS,
                RECENTER_MAX_SPEED_DPS,
            )

            yaw_cmd = _bounded_speed(
                yaw_error,
                RECENTER_KP,
                RECENTER_MIN_SPEED_DPS,
                RECENTER_MAX_SPEED_DPS,
            )

            gimbal.drive_speed(
                pitch_speed=pitch_cmd,
                yaw_speed=yaw_cmd,
            )

        time.sleep(0.03)

    stop_gimbal(gimbal)
    print("[GIMBAL] Recenter timed out.", flush=True)
    return False


# ---------------------------------------------------------------------------
# Computer vision
# ---------------------------------------------------------------------------

def make_color_mask(hsv: np.ndarray, color_name: str) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in COLOR_RANGES[color_name]:
        part = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = cv2.bitwise_or(mask, part)

    kernel_size = max(3, int(MORPH_KERNEL_SIZE))
    if kernel_size % 2 == 0:
        kernel_size += 1

    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    return mask


def contour_centroid(contour: np.ndarray) -> Optional[Tuple[int, int]]:
    moments = cv2.moments(contour)

    if abs(moments["m00"]) < 1e-9:
        return None

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    return cx, cy


def classify_shape(contour: np.ndarray) -> Tuple[str, dict]:
    area = float(cv2.contourArea(contour))
    perimeter = float(cv2.arcLength(contour, True))

    if area <= 0.0 or perimeter <= 0.0:
        return "unknown", {}

    approx = cv2.approxPolyDP(
        contour,
        APPROX_EPSILON_RATIO * perimeter,
        True,
    )

    vertices = len(approx)

    x, y, w, h = cv2.boundingRect(contour)

    aspect_ratio = float(w) / float(h) if h > 0 else 0.0
    bbox_area = float(w * h)
    rectangularity = area / bbox_area if bbox_area > 0 else 0.0

    circularity = (
        4.0 * math.pi * area / (perimeter * perimeter)
    )

    shape = "unknown"

    if (
        vertices == 4
        and cv2.isContourConvex(approx)
        and rectangularity >= RECTANGULARITY_MIN
    ):
        if SQUARE_ASPECT_MIN <= aspect_ratio <= SQUARE_ASPECT_MAX:
            shape = "square"
        else:
            shape = "rectangle"

    elif (
        vertices >= CIRCLE_MIN_VERTICES
        and circularity >= CIRCLE_CIRCULARITY_MIN
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
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame_area = float(frame.shape[0] * frame.shape[1])

    min_area = max(
        MIN_CONTOUR_AREA_PX,
        MIN_CONTOUR_AREA_RATIO * frame_area,
    )
    max_area = MAX_CONTOUR_AREA_RATIO * frame_area

    detections: List[Detection] = []
    masks: Dict[str, np.ndarray] = {}

    for color_name in COLOR_RANGES.keys():
        mask = make_color_mask(hsv, color_name)
        masks[color_name] = mask

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

            centroid = contour_centroid(contour)
            if centroid is None:
                continue

            shape, metrics = classify_shape(contour)
            if shape == "unknown":
                continue

            x, y, w, h = cv2.boundingRect(contour)

            detections.append(
                Detection(
                    color=color_name,
                    shape=shape,
                    contour=contour,
                    bbox=(x, y, w, h),
                    centroid=centroid,
                    area=metrics["area"],
                    perimeter=metrics["perimeter"],
                    vertices=metrics["vertices"],
                    aspect_ratio=metrics["aspect_ratio"],
                    circularity=metrics["circularity"],
                    rectangularity=metrics["rectangularity"],
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


# ---------------------------------------------------------------------------
# HSV calibration
# ---------------------------------------------------------------------------

def _noop(_value) -> None:
    pass


def run_hsv_calibration(
    camera_stream: LatestFrameCamera,
    target_color: str,
) -> bool:
    window = "HSV Calibration"
    mask_window = "HSV Calibration Mask"

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.namedWindow(mask_window, cv2.WINDOW_NORMAL)

    current = COLOR_RANGES[target_color]
    first_low, first_high = current[0]

    if len(current) >= 2:
        second_low, second_high = current[1]
        use_h2 = 1
    else:
        second_low = (0, first_low[1], first_low[2])
        second_high = (0, first_high[1], first_high[2])
        use_h2 = 0

    cv2.createTrackbar("H1 Min", window, int(first_low[0]), 179, _noop)
    cv2.createTrackbar("H1 Max", window, int(first_high[0]), 179, _noop)
    cv2.createTrackbar("H2 Min", window, int(second_low[0]), 179, _noop)
    cv2.createTrackbar("H2 Max", window, int(second_high[0]), 179, _noop)
    cv2.createTrackbar("Use H2", window, use_h2, 1, _noop)
    cv2.createTrackbar("S Min", window, int(first_low[1]), 255, _noop)
    cv2.createTrackbar("S Max", window, int(first_high[1]), 255, _noop)
    cv2.createTrackbar("V Min", window, int(first_low[2]), 255, _noop)
    cv2.createTrackbar("V Max", window, int(first_high[2]), 255, _noop)

    print("[CALIBRATION] S=save, Q/ESC=cancel", flush=True)

    saved = False

    while True:
        frame = camera_stream.read_latest(max_age_sec=1.0)

        if frame is None:
            time.sleep(0.01)
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h1_min = cv2.getTrackbarPos("H1 Min", window)
        h1_max = cv2.getTrackbarPos("H1 Max", window)
        h2_min = cv2.getTrackbarPos("H2 Min", window)
        h2_max = cv2.getTrackbarPos("H2 Max", window)
        use_h2 = cv2.getTrackbarPos("Use H2", window)

        s_min = cv2.getTrackbarPos("S Min", window)
        s_max = cv2.getTrackbarPos("S Max", window)
        v_min = cv2.getTrackbarPos("V Min", window)
        v_max = cv2.getTrackbarPos("V Max", window)

        h1_lo, h1_hi = sorted((h1_min, h1_max))
        h2_lo, h2_hi = sorted((h2_min, h2_max))
        s_lo, s_hi = sorted((s_min, s_max))
        v_lo, v_hi = sorted((v_min, v_max))

        mask = cv2.inRange(
            hsv,
            np.array((h1_lo, s_lo, v_lo), dtype=np.uint8),
            np.array((h1_hi, s_hi, v_hi), dtype=np.uint8),
        )

        if use_h2:
            mask2 = cv2.inRange(
                hsv,
                np.array((h2_lo, s_lo, v_lo), dtype=np.uint8),
                np.array((h2_hi, s_hi, v_hi), dtype=np.uint8),
            )
            mask = cv2.bitwise_or(mask, mask2)

        kernel = np.ones(
            (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE),
            dtype=np.uint8,
        )

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        preview = frame.copy()

        cv2.putText(
            preview,
            "CALIBRATE {}".format(target_color.upper()),
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            preview,
            "S=SAVE   Q/ESC=CANCEL",
            (12, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(window, preview)
        cv2.imshow(mask_window, mask)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            new_ranges = [
                (
                    (h1_lo, s_lo, v_lo),
                    (h1_hi, s_hi, v_hi),
                )
            ]

            if use_h2:
                new_ranges.append(
                    (
                        (h2_lo, s_lo, v_lo),
                        (h2_hi, s_hi, v_hi),
                    )
                )

            COLOR_RANGES[target_color] = new_ranges
            saved = True

            print(
                "[CALIBRATION] {} = {}".format(
                    target_color.upper(),
                    COLOR_RANGES[target_color],
                ),
                flush=True,
            )
            break

        if key in (ord("q"), 27):
            break

    cv2.destroyWindow(window)
    cv2.destroyWindow(mask_window)

    return saved


def pre_mission_screen(
    camera_stream: LatestFrameCamera,
    target_color: str,
    target_shape: str,
    direction_label: str,
) -> bool:
    window = "RoboMaster Pre-Mission"

    while True:
        frame = camera_stream.read_latest(max_age_sec=1.0)

        if frame is None:
            time.sleep(0.01)
            continue

        display = frame.copy()

        cv2.putText(
            display,
            "TARGET: {} {}".format(
                target_color.upper(),
                target_shape.upper(),
            ),
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            display,
            "DIRECTION: {}".format(direction_label),
            (12, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            display,
            "ENTER=START  C=HSV CALIBRATION  Q/ESC=QUIT",
            (12, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.47,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(window, display)

        key = cv2.waitKey(1) & 0xFF

        if key in (10, 13):
            cv2.destroyWindow(window)
            return True

        if key == ord("c"):
            run_hsv_calibration(camera_stream, target_color)

        if key in (ord("q"), 27):
            cv2.destroyWindow(window)
            return False


# ---------------------------------------------------------------------------
# Robot movement helpers
# ---------------------------------------------------------------------------

def stop_chassis(chassis) -> None:
    try:
        chassis.drive_speed(x=0.0, y=0.0, z=0.0)
    except Exception:
        pass


def command_strafe(chassis, runtime_direction: int) -> None:
    y_speed = (
        float(runtime_direction)
        * float(STRAFE_Y_SIGN)
        * float(STRAFE_SPEED_MPS)
    )

    chassis.drive_speed(
        x=0.0,
        y=y_speed,
        z=0.0,
        timeout=CHASSIS_COMMAND_TIMEOUT_SEC,
    )


def draw_detection(frame: np.ndarray, detection: Detection) -> None:
    color = DRAW_COLORS.get(detection.color, (255, 255, 255))

    cv2.drawContours(frame, [detection.contour], -1, color, 2)

    x, y, w, h = detection.bbox

    cv2.rectangle(
        frame,
        (x, y),
        (x + w, y + h),
        color,
        2,
    )

    cx, cy = detection.centroid

    cv2.circle(frame, (cx, cy), 5, color, -1)

    label = "{} {}  A={:.0f} circ={:.2f}".format(
        detection.color.upper(),
        detection.shape.upper(),
        detection.area,
        detection.circularity,
    )

    cv2.putText(
        frame,
        label,
        (x, max(18, y - 7)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
        cv2.LINE_AA,
    )


def put_status_line(
    frame: np.ndarray,
    text: str,
    index: int,
    color=(255, 255, 255),
    scale=0.52,
) -> None:
    y = 22 + index * 23

    cv2.putText(
        frame,
        text,
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        2,
        cv2.LINE_AA,
    )


# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------

def run_mission(
    ep_robot,
    camera_stream: LatestFrameCamera,
    target_color: str,
    target_shape: str,
    runtime_direction: int,
    direction_label: str,
    gimbal_tracker: GimbalAngleTracker,
) -> None:
    chassis = ep_robot.chassis
    gimbal = ep_robot.gimbal

    state = STATE_SEARCH
    paused = False

    verify_count = 0
    verify_previous_center = None

    clear_missing_count = 0
    clear_started_time = 0.0

    handled_targets = 0

    last_chassis_command = 0.0

    mission_start = time.monotonic()
    last_loop_time = mission_start
    travel_elapsed = 0.0

    print()
    print("============================================================")
    print("MISSION START")
    print("Target    : {} + {}".format(target_color.upper(), target_shape.upper()))
    print("Direction : {}".format(direction_label))
    print("Speed     : {:.2f} m/s".format(STRAFE_SPEED_MPS))
    print("============================================================")
    print("SPACE = mark current verified target handled and resume")
    print("N     = skip current verified target")
    print("P     = pause/resume")
    print("R     = return gimbal to neutral")
    print("Q/ESC = emergency program stop")
    print("============================================================")

    stop_chassis(chassis)
    recenter_gimbal(gimbal, gimbal_tracker)

    while True:
        now = time.monotonic()
        dt = max(0.0, now - last_loop_time)
        last_loop_time = now

        if not paused and state in (STATE_SEARCH, STATE_CLEAR):
            travel_elapsed += dt

        if travel_elapsed >= MAX_TRAVEL_TIME:
            print("[MISSION] MAX_TRAVEL_TIME reached.", flush=True)
            stop_chassis(chassis)
            stop_gimbal(gimbal)
            break

        frame = camera_stream.read_latest(max_age_sec=0.5)

        if frame is None:
            stop_chassis(chassis)
            print("[CAMERA] No fresh frame. Chassis stopped.", flush=True)
            time.sleep(0.05)
            continue

        detections, masks = detect_objects(frame)
        target_mask = masks[target_color]

        display = frame.copy()

        for detection in detections:
            draw_detection(display, detection)

        match_candidate = choose_target_match(
            detections,
            target_color,
            target_shape,
            frame.shape,
        )

        exact_match_present = match_candidate is not None

        primary_detection = None
        if detections:
            primary_detection = max(detections, key=lambda item: item.area)

        # ----------------------------------------------------
        # SEARCH + VERIFY
        # ----------------------------------------------------
        if state == STATE_SEARCH:
            if (
                not paused
                and now - last_chassis_command >= CHASSIS_COMMAND_PERIOD_SEC
            ):
                command_strafe(chassis, runtime_direction)
                last_chassis_command = now

            if match_candidate is None:
                verify_count = 0
                verify_previous_center = None
            else:
                if verify_previous_center is None:
                    verify_count = 1
                else:
                    jump = math.hypot(
                        match_candidate.centroid[0] - verify_previous_center[0],
                        match_candidate.centroid[1] - verify_previous_center[1],
                    )

                    if jump <= VERIFY_MAX_CENTER_JUMP_PX:
                        verify_count += 1
                    else:
                        verify_count = 1

                verify_previous_center = match_candidate.centroid

                if verify_count >= MATCH_VERIFY_FRAMES:
                    stop_chassis(chassis)

                    state = STATE_READY_MANUAL

                    verify_count = 0
                    verify_previous_center = None

                    print()
                    print("TARGET VERIFIED")
                    print("{} {}".format(target_color.upper(), target_shape.upper()))
                    print("CHASSIS STOPPED")
                    print("Final gimbal positioning/firing is manual.")
                    print("SPACE = mark handled and continue")
                    print("N     = skip and continue")
                    print()

        # ----------------------------------------------------
        # READY_MANUAL
        # ----------------------------------------------------
        elif state == STATE_READY_MANUAL:
            stop_chassis(chassis)

        # ----------------------------------------------------
        # CLEAR
        # ----------------------------------------------------
        elif state == STATE_CLEAR:
            if (
                not paused
                and now - last_chassis_command >= CHASSIS_COMMAND_PERIOD_SEC
            ):
                command_strafe(chassis, runtime_direction)
                last_chassis_command = now

            if exact_match_present:
                clear_missing_count = 0
            else:
                clear_missing_count += 1

            if (
                clear_missing_count >= CLEAR_MISSING_FRAMES
                and now - clear_started_time >= CLEAR_MIN_TRAVEL_SEC
            ):
                state = STATE_SEARCH
                verify_count = 0
                verify_previous_center = None
                print("[CLEAR] Previous target cleared. SEARCH resumed.", flush=True)

        detected_text = "NONE"
        if primary_detection is not None:
            detected_text = "{} {}".format(
                primary_detection.color.upper(),
                primary_detection.shape.upper(),
            )

        match_text = "YES" if exact_match_present else "NO"

        state_text = state if not paused else "PAUSED / {}".format(state)

        put_status_line(
            display,
            "TARGET : {} {}".format(target_color.upper(), target_shape.upper()),
            0,
            (0, 255, 255),
        )
        put_status_line(display, "DETECT : {}".format(detected_text), 1)
        put_status_line(display, "STATE  : {}".format(state_text), 2)
        put_status_line(
            display,
            "MATCH  : {}".format(match_text),
            3,
            (0, 255, 0) if exact_match_present else (0, 0, 255),
        )
        put_status_line(
            display,
            "VERIFY : {}/{}".format(verify_count, MATCH_VERIFY_FRAMES),
            4,
        )
        put_status_line(
            display,
            "TRAVEL : {:.1f}/{:.1f}s".format(
                travel_elapsed,
                MAX_TRAVEL_TIME,
            ),
            5,
        )
        put_status_line(
            display,
            "HANDLED: {}".format(handled_targets),
            6,
        )
        put_status_line(
            display,
            "SPACE=HANDLED  N=SKIP  P=PAUSE  R=RECENTER  Q/ESC=STOP",
            7,
            (180, 220, 255),
            0.42,
        )

        if state == STATE_READY_MANUAL:
            cv2.putText(
                display,
                "TARGET VERIFIED - CHASSIS STOPPED",
                (20, frame.shape[0] - 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.68,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                display,
                "Manual final action; SPACE marks handled",
                (20, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow("RoboMaster Target Mission", display)
        cv2.imshow(
            "HSV Mask - {}".format(target_color.upper()),
            target_mask,
        )

        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            print("[EMERGENCY] Q/ESC pressed.", flush=True)
            stop_chassis(chassis)
            stop_gimbal(gimbal)
            break

        if key == ord("p"):
            paused = not paused

            if paused:
                stop_chassis(chassis)
                print("[PAUSE] Mission paused.", flush=True)
            else:
                last_chassis_command = 0.0
                print("[PAUSE] Mission resumed.", flush=True)

        if key == ord("r"):
            stop_chassis(chassis)
            print("[GIMBAL] Returning to neutral.", flush=True)
            recenter_gimbal(gimbal, gimbal_tracker)
            last_chassis_command = 0.0

        if (
            key in (32, ord("n"))
            and state == STATE_READY_MANUAL
        ):
            stop_chassis(chassis)

            if key == 32:
                handled_targets += 1
                print(
                    "[TARGET] Operator marked target handled. No firing command was sent.",
                    flush=True,
                )
            else:
                print("[TARGET] Target skipped.", flush=True)

            # Return camera/gimbal to neutral for continued searching.
            recenter_gimbal(gimbal, gimbal_tracker)

            state = STATE_CLEAR
            clear_missing_count = 0
            clear_started_time = time.monotonic()
            last_chassis_command = 0.0

        if paused:
            stop_chassis(chassis)

    stop_chassis(chassis)
    stop_gimbal(gimbal)

    print()
    print("============================================================")
    print("MISSION COMPLETE")
    print("Travel time : {:.1f}s".format(travel_elapsed))
    print("Handled     : {}".format(handled_targets))
    print("============================================================")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ep_robot = None
    camera_stream = None

    chassis = None
    gimbal = None

    gimbal_tracker = GimbalAngleTracker()
    gimbal_subscribed = False

    try:
        target_color, target_shape = select_target()
        runtime_direction, direction_label = select_direction()

        print()
        print("==============================")
        print("Selected Mission")
        print("==============================")
        print("TARGET    : {} + {}".format(target_color.upper(), target_shape.upper()))
        print("DIRECTION : {}".format(direction_label))
        print()
        input("Press ENTER to connect and prepare mission...")

        print("Connecting to RoboMaster...", flush=True)

        ep_robot = robot.Robot()

        ok = ep_robot.initialize(conn_type=CONNECTION_TYPE)

        print("RoboMaster initialize returned: {!r}".format(ok), flush=True)

        if not ok:
            raise RuntimeError("Could not connect to RoboMaster")

        ep_robot.set_robot_mode(mode=robot.FREE)

        chassis = ep_robot.chassis
        gimbal = ep_robot.gimbal

        stop_chassis(chassis)
        stop_gimbal(gimbal)

        gimbal_subscribed = bool(
            gimbal.sub_angle(
                freq=20,
                callback=gimbal_tracker.callback,
            )
        )

        print(
            "Gimbal angle subscription: {!r}".format(gimbal_subscribed),
            flush=True,
        )

        deadline = time.monotonic() + 2.0

        while time.monotonic() < deadline:
            pitch, yaw = gimbal_tracker.get()
            if pitch is not None and yaw is not None:
                break
            time.sleep(0.03)

        pitch, yaw = gimbal_tracker.get()

        if pitch is None or yaw is None:
            raise RuntimeError("No gimbal angle feedback")

        recenter_gimbal(gimbal, gimbal_tracker)

        camera_stream = LatestFrameCamera(
            ep_robot,
            CAMERA_RESOLUTION,
        )
        camera_stream.start()

        print(
            "[CAMERA] Warm-up {:.1f}s...".format(CAMERA_WARMUP_SEC),
            flush=True,
        )

        warmup_deadline = time.monotonic() + CAMERA_WARMUP_SEC

        while time.monotonic() < warmup_deadline:
            camera_stream.read_latest(max_age_sec=1.0)
            time.sleep(0.02)

        if not pre_mission_screen(
            camera_stream,
            target_color,
            target_shape,
            direction_label,
        ):
            print("Mission cancelled before start.")
            return

        run_mission(
            ep_robot=ep_robot,
            camera_stream=camera_stream,
            target_color=target_color,
            target_shape=target_shape,
            runtime_direction=runtime_direction,
            direction_label=direction_label,
            gimbal_tracker=gimbal_tracker,
        )

    except KeyboardInterrupt:
        print("\n[EMERGENCY] Ctrl+C received.", flush=True)

    finally:
        if chassis is not None:
            stop_chassis(chassis)

        if gimbal is not None:
            stop_gimbal(gimbal)

        if camera_stream is not None:
            try:
                camera_stream.stop()
            except Exception:
                pass

        if gimbal is not None and gimbal_subscribed:
            try:
                gimbal.unsub_angle()
            except Exception:
                pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        if ep_robot is not None:
            try:
                ep_robot.close()
            except Exception:
                pass

        print("[CLEANUP] Robot stopped and resources closed.", flush=True)


if __name__ == "__main__":
    main()
