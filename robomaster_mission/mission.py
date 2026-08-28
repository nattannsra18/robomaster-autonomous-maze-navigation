"""
RoboMaster Basic Fixed-Grid Maze - Pickup + 40 cm Drop + Exit
================================================================
Default mode opens a Tkinter editor where the operator sets rows, columns,
physical cell size, internal walls, START, DROP and EXIT.  The mission then:

    1. picks up and ToF-verifies the object before entering the maze,
    2. follows an orientation-aware A* route to DROP,
    3. physically lowers and releases the object,
    4. replans to EXIT and drives beyond the selected border,
    5. records the path, wall evidence, final map and sensor graph.

This basic version is intended for a fixed maze with known cell dimensions.
It uses orientation-aware A* and RoboMaster odometry to move exactly one cell
at a time.  At DROP it faces a configured wall/corner, aligns the front ToF and
one side Sharp to configurable targets (40 cm by default), physically releases
the object, then continues to EXIT.

Original exploration architecture retained below
------------------------------------------------
Architecture
    Wall sensing
      - Front ToF
      - Left/Right Sharp IR distance
      - Optional digital IR sensors on front-left/front-right chassis corners

    Localization
      - RoboMaster odometry (sub_position cs=1)
      - Discrete N/E/S/W heading from attitude yaw
      - Small metric grid quantization for junction memory

    Exploration
      - Persistent topological junction graph
      - Unvisited exits are frontiers
      - Trémaux-style edge traversal marks
      - Local DFS preference; BFS is used only to transit through already-known
        corridors to the nearest node that still owns an unexplored frontier
      - Loop closure uses position + expected graph target + incoming heading

    Junction handling
      - Side opening is tracked as a physical opening window
      - Robot passes the opening, then reverses to the estimated centre
      - Front blocked corners/dead-ends creep to a repeatable front distance
      - Junction node is registered ONLY after centering

    Exit handling
      - Optional sustained open-space detector. It requires FRONT + LEFT + RIGHT
        to remain open for a configurable travelled distance, so a normal short
        intersection does not immediately count as the maze exit.

IMPORTANT HARDWARE NOTE
    The old code only provided one digital IR adapter ID (1). This clean version
    therefore enables the known front-left IR and leaves IR_FRONT_RIGHT_ID=None.
    If you really have a second digital IR on the right front edge, set its
    adapter ID below. Also verify IR_BLOCKED_LEVEL on your hardware.

Requires RoboMaster Python SDK.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import queue
import statistics
import threading
import time
import traceback
from collections import Counter
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Set, Tuple

from .version import PROGRAM_VERSION

try:
    from robomaster import conn as robomaster_conn
    from robomaster import robot
except ModuleNotFoundError:
    # Planner-only/simulation mode can still be used without the SDK.
    robot = None
    robomaster_conn = None


# ============================================================
# CONFIG - mostly preserved from the supplied working values
# ============================================================

ENABLE_MOTION = True
# ---------------- Sensor IDs ----------------
IR_FRONT_LEFT_ID = 1
IR_FRONT_RIGHT_ID = 4

IR_FRONT_LEFT_PORT = 1
IR_FRONT_RIGHT_PORT = 1

SHARP_LEFT_ID = 2
SHARP_RIGHT_ID = 3
SENSOR_PORT = 1

# Digital IR polarity
# 0 = obstacle detected
# 1 = clear
IR_BLOCKED_LEVEL = 0
IR_CONFIRM_SAMPLES = 2

SHARP_FILTER_SIZE = 7
TOF_FILTER_SIZE = 3
# The supplied calibration test uses a 7-sample median. Do not add a second
# slow EMA that would make a reading from the previous position linger.
SHARP_EMA_NEW_WEIGHT = 1.0
SHARP_EMA_OLD_WEIGHT = 0.0
TOF_STALE_SEC = 0.40

# ---------------- Distances ----------------
TARGET_LEFT_CM = 12.0
TARGET_RIGHT_CM = 12.0


SLOW_FRONT_CM = 18.0
STOP_FRONT_CM = 15.0
SIDE_TOO_CLOSE_CM = 8

SIDE_WALL_ENTER_CM = 28.0
SIDE_WALL_EXIT_CM = 32.0

SIDE_OPEN_ENTER_CM = 18.0
SIDE_OPEN_EXIT_CM = 15.0
EXPLORATION_FRONT_OPEN_CM = 45.0

# ---------------- Speed ----------------
FORWARD_SPEED = 0.25
MIN_FORWARD_SPEED = 0.05
UNKNOWN_FRONT_SPEED = 0.0
ESCAPE_FORWARD_SPEED = 0.00
ESCAPE_Y_SPEED = 0.08

# Proactive side safety: start moving away BEFORE reaching SIDE_TOO_CLOSE_CM.
SIDE_WARNING_CM = 11
SIDE_WARNING_FORWARD_SPEED = 0.08
SIDE_WARNING_Y_SPEED = 0.05

# Front-corner IR has priority over normal wall following.
IR_ESCAPE_Y_SPEED = 0.15
IR_ESCAPE_FORWARD_SPEED = 0.00

Y_DIR_SIGN = 1

# ---------------- Wall controller ----------------
SIDE_KP_STRAFE = 0.012
SIDE_MAX_Y = 0.05
SIDE_DEADBAND_CM = 1.0

CENTER_TRIGGER_CM = 2.0
CENTER_RELEASE_CM = 0.7
CENTER_HOLD_SEC = 0.30
CENTER_KP_STRAFE = 0.009
CENTER_MAX_Y = 0.04

# ---------------- Heading / turn ----------------
DEFAULT_MOVE_TO_YAW_SIGN = -1
DEFAULT_DRIVE_TO_YAW_SIGN = +1

ENABLE_HEADING_HOLD = True
HEADING_KP_Z = 1.5
HEADING_MAX_Z_SPEED = 12.0
HEADING_DEADBAND_DEG = 0.8
HEADING_RECOVER_TRIGGER_DEG = 8.0
HEADING_RECOVER_RELEASE_DEG = 2.0
HEADING_RECOVER_MAX_Z_SPEED = 18.0

TURN_FEEDBACK_KP = 1.20
TURN_FEEDBACK_MIN_Z_SPEED = 10.0
TURN_FEEDBACK_MAX_Z_SPEED = 55.0
TURN_FEEDBACK_TOLERANCE_DEG = 2.0
TURN_FEEDBACK_STABLE_SAMPLES = 3
TURN_FEEDBACK_LOOP_SEC = 0.03
TURN_FEEDBACK_DRIVE_TIMEOUT_SEC = 0.20
TURN_FEEDBACK_TIMEOUT_90_SEC = 3.50
TURN_FEEDBACK_TIMEOUT_180_SEC = 6.00
TURN_PRE_SETTLE_SEC = 0.10
YAW_SETTLE_SEC = 0.08

ATTITUDE_FREQ_HZ = 20
POSE_FREQ_HZ = 20
POSE_WAIT_SEC = 1.0

HEADING_ALIGN_TOLERANCE_DEG = 2.0
HEADING_ALIGN_TIMEOUT_SEC = 1.20
HEADING_ALIGN_LOOP_SEC = 0.04

# ---------------- Junction detector ----------------
OPENING_ENTER_SAMPLES = 3
OPENING_EXIT_SAMPLES = 3
OPENING_MIN_LENGTH_M = 0.10
OPENING_MAX_LENGTH_M = 0.70
OPENING_LOOKAHEAD_M = 0.12
OPENING_MIN_EVIDENCE_SAMPLES = 2

OPENING_CENTER_SPEED = 0.07
OPENING_CENTER_MAX_BACKTRACK_M = 0.40
OPENING_CENTER_TIMEOUT_SEC = 6.5
OPENING_CENTER_LOOP_SEC = 0.04

# For front-blocked corner/dead-end: creep to a repeatable turn centre.
CORNER_CENTER_SPEED = 0.05
CORNER_CENTER_FRONT_TARGET_CM = 11.0
CORNER_CENTER_FRONT_HARD_STOP_CM = 10.5
CORNER_CENTER_MAX_DISTANCE_M = 0.14
CORNER_CENTER_TIMEOUT_SEC = 3.0
CORNER_CENTER_LOOP_SEC = 0.04

DECISION_SCAN_SAMPLES = 5
DECISION_SCAN_INTERVAL_SEC = 0.04
JUNCTION_SETTLE_SEC = 0.15
JUNCTION_CONFIRM_SAMPLES = 3

# Lock the same physical junction after a decision until the robot leaves it.
JUNCTION_REARM_MIN_DISTANCE_M = 0.14
JUNCTION_REARM_DISTANCE_M = 0.25
JUNCTION_REARM_TIMEOUT_SEC = 2.50
JUNCTION_REARM_SAMPLES = 4
JUNCTION_REARM_EMERGENCY_SEC = 0.25

# ---------------- Junction memory / loop closure ----------------
NODE_MATCH_RADIUS_M = 0.18
EXPECTED_TARGET_MATCH_RADIUS_M = 0.26

# Extended loop-closure radius is only used when incoming topology agrees.
# It is deliberately NOT used as a blind distance-only merge radius.
LOOP_MATCH_RADIUS_M = 0.30

# If a new junction event fires only a few centimetres after leaving a node,
# it is usually the SAME wide intersection being detected again rather than a
# real neighbouring junction.  The old V1 deliberately excluded pending_from
# from matching, which created fake nodes such as J10 in the field log.
SAME_NODE_RETRIGGER_RADIUS_M = 0.32
# A departure that re-triggers the same node after only a short distance and
# has a hard front block is a false/blocked exit, not a completed traversal.
FAILED_EDGE_MAX_PROGRESS_M = 0.30
NODE_POSITION_UPDATE_ALPHA = 0.15

# Quantization resolution for the local metric grid (not maze-cell size).
GRID_RESOLUTION_M = 0.05

EXPLORATION_PREFERENCE = ("FRONT", "LEFT", "RIGHT", "BACK")

# ---------------- Post-turn corner clearance ----------------
ENABLE_POST_TURN_CLEARANCE = True
POST_TURN_TRIGGER_CM = 7.0
POST_TURN_RELEASE_CM = 8.5
POST_TURN_FORWARD_SPEED = 0.04
POST_TURN_Y_SPEED = 0.045
POST_TURN_MAX_DISTANCE_M = 0.10
POST_TURN_MAX_SEC = 1.50
POST_TURN_FRONT_STOP_CM = 12.0
POST_TURN_LOOP_SEC = 0.04

# ---------------- Exit detector ----------------
# Outside the maze should remain open longer than an ordinary intersection.
ENABLE_EXIT_DETECTION = True
EXIT_FRONT_OPEN_CM = 50.0
EXIT_SIDE_OPEN_CM = 35.0
EXIT_CONFIRM_DISTANCE_M = 0.60
EXIT_CONFIRM_MIN_SEC = 1.5
EXIT_RESET_SAMPLES = 3

# ---------------- Main loop / logging ----------------
LOOP_DELAY_SEC = 0.05
DRIVE_TIMEOUT_SEC = 0.15
AFTER_TURN_DELAY_SEC = 0.12
SAVE_MAZE_MEMORY = True
MAZE_MEMORY_FILE = "maze_memory_clean.json"
PRINT_EVERY_SEC = 0.20

# ---------------- Sharp calibration: ADC -> cm ----------------
CALIBRATION_SHARP_LEFT = [
    # Calibrated on the supplied LEFT/RIGHT hardware test.
    (450, 10.0),
    (360, 20.0),
    (300, 30.0),
    (240, 40.0),
    (200, 50.0),
]
CALIBRATION_SHARP_RIGHT = list(CALIBRATION_SHARP_LEFT)
_DEFAULT_CALIBRATION_SHARP_LEFT = list(CALIBRATION_SHARP_LEFT)
_DEFAULT_CALIBRATION_SHARP_RIGHT = list(CALIBRATION_SHARP_RIGHT)
_SHARP_CALIBRATION_LOADED_FROM: Dict[str, Optional[str]] = {
    "LEFT": None,
    "RIGHT": None,
}


def _validated_sharp_calibration_table(raw_table, label: str):
    """Return an adc-descending calibration table or raise ValueError."""
    if not isinstance(raw_table, list) or len(raw_table) < 2:
        raise ValueError(f"{label} calibration needs at least two [ADC, cm] points")

    points = []
    for index, pair in enumerate(raw_table, start=1):
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError(f"{label} point {index} must be [ADC, cm]")
        adc, cm = float(pair[0]), float(pair[1])
        if not math.isfinite(adc) or not math.isfinite(cm) or adc <= 0.0 or cm <= 0.0:
            raise ValueError(f"{label} point {index} contains an invalid value")
        points.append((adc, cm))

    # adc_to_cm() expects the largest ADC (nearest wall) first.
    points.sort(key=lambda item: item[0], reverse=True)
    for previous, current in zip(points, points[1:]):
        if previous[0] <= current[0]:
            raise ValueError(f"{label} ADC values must be unique")
        if previous[1] >= current[1]:
            raise ValueError(
                f"{label} is not monotonic: distance must increase as ADC decreases"
            )
    return points


def load_sharp_calibration_file(path_value: str, side: str) -> Optional[Path]:
    """Load a calibration JSON, resolving relative paths beside script or CWD."""
    side = str(side).strip().upper()
    if side not in ("LEFT", "RIGHT"):
        raise ValueError("Sharp calibration side must be LEFT or RIGHT")

    value = str(path_value or "").strip()
    if not value:
        print(f"[CALIBRATION] {side} Sharp: no JSON selected; using built-in table")
        return None

    requested = Path(value).expanduser()
    candidates = [requested] if requested.is_absolute() else [
        Path.cwd() / requested,
        Path(__file__).resolve().parent / requested,
    ]
    resolved = next((candidate for candidate in candidates if candidate.is_file()), None)
    if resolved is None:
        print(
            f"[CALIBRATION] WARNING: {side} Sharp file '{value}' was not found; "
            "using built-in table"
        )
        return None

    try:
        payload = json.loads(resolved.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("calibration JSON root must be an object")
        raw_table = payload.get("table", payload.get("calibration_table"))
        table = _validated_sharp_calibration_table(raw_table, f"{side} Sharp")
        declared_side = str(payload.get("side", side)).strip().upper()
        if declared_side not in (side, ""):
            raise ValueError(
                f"file declares side={declared_side}, but GUI requested {side}"
            )
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        print(
            f"[CALIBRATION] WARNING: cannot use {side} Sharp file "
            f"'{resolved}': {exc}; using built-in table"
        )
        return None

    global CALIBRATION_SHARP_LEFT, CALIBRATION_SHARP_RIGHT
    if side == "LEFT":
        CALIBRATION_SHARP_LEFT = table
    else:
        CALIBRATION_SHARP_RIGHT = table
    _SHARP_CALIBRATION_LOADED_FROM[side] = str(resolved)
    print(
        f"[CALIBRATION] Loaded {side} Sharp: {len(table)} points from {resolved}"
    )
    return resolved


# ============================================================
# Utility
# ============================================================

HEADINGS = ("N", "E", "S", "W")
RELATIVE_OFFSET = {"FRONT": 0, "RIGHT": 1, "BACK": 2, "LEFT": -1}
RELATIVE_TURN_DEG = {"FRONT": 0.0, "LEFT": 90.0, "RIGHT": -90.0, "BACK": -180.0}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def shortest_angle_error_deg(target: float, current: float) -> float:
    return normalize_angle_deg(float(target) - float(current))


def distance_xy(a: Tuple[Optional[float], Optional[float]],
                b: Tuple[Optional[float], Optional[float]]) -> Optional[float]:
    if None in (a[0], a[1], b[0], b[1]):
        return None
    return math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))


def median_or_none(values: List[float]) -> Optional[float]:
    return statistics.median(values) if values else None


def fmt(v: Optional[float]) -> str:
    return "---" if v is None else f"{v:5.1f}"


# ============================================================
# Pose tracker
# ============================================================

class PoseTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.z: Optional[float] = None
        self.yaw: Optional[float] = None
        self.pitch: Optional[float] = None
        self.roll: Optional[float] = None

    def position_callback(self, data):
        try:
            if data is None or len(data) < 3:
                return
            x, y, z = data[:3]
            with self._lock:
                self.x = float(x)
                self.y = float(y)
                self.z = float(z)
        except Exception as exc:
            print("Position callback error:", exc)

    def attitude_callback(self, data):
        try:
            if data is None or len(data) < 3:
                return
            yaw, pitch, roll = data[:3]
            with self._lock:
                self.yaw = normalize_angle_deg(yaw)
                self.pitch = float(pitch)
                self.roll = float(roll)
        except Exception as exc:
            print("Attitude callback error:", exc)

    def get_xy(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.x, self.y

    def get_yaw(self) -> Optional[float]:
        with self._lock:
            return self.yaw

    def has_position(self) -> bool:
        x, y = self.get_xy()
        return x is not None and y is not None


# ============================================================
# Sensor manager
# ============================================================

class SensorManager:
    def __init__(self, sensor_adapter):
        self.sensor_adapter = sensor_adapter

        self.left_adc_buf = deque(maxlen=SHARP_FILTER_SIZE)
        self.right_adc_buf = deque(maxlen=SHARP_FILTER_SIZE)
        self.tof_buf = deque(maxlen=TOF_FILTER_SIZE)

        self.left_ema: Optional[float] = None
        self.right_ema: Optional[float] = None
        self.front_cm: Optional[float] = None
        self.tof_last_update: Optional[float] = None

        self.ir_left_hist = deque(maxlen=IR_CONFIRM_SAMPLES)
        self.ir_right_hist = deque(maxlen=IR_CONFIRM_SAMPLES)

    def tof_callback(self, data):
        try:
            if not data or data[0] is None:
                return
            mm = float(data[0])
            if mm < 20.0 or mm > 4000.0:
                return
            cm = mm / 10.0
            self.tof_buf.append(cm)
            self.front_cm = statistics.median(self.tof_buf)
            self.tof_last_update = time.monotonic()
        except Exception as exc:
            print("ToF callback error:", exc)

    def get_front_cm(self) -> Optional[float]:
        if self.front_cm is None or self.tof_last_update is None:
            return None
        if time.monotonic() - self.tof_last_update > TOF_STALE_SEC:
            return None
        return self.front_cm

    @staticmethod
    def adc_to_cm(adc: float, table) -> float:
        if adc >= table[0][0]:
            return float(table[0][1])
        if adc <= table[-1][0]:
            return float(table[-1][1])

        for i in range(len(table) - 1):
            adc1, cm1 = table[i]
            adc2, cm2 = table[i + 1]
            if adc1 >= adc >= adc2:
                ratio = (adc1 - adc) / (adc1 - adc2)
                return cm1 + ratio * (cm2 - cm1)
        return float(table[-1][1])

    def _read_sharp(self, sensor_id: int, left: bool):
        try:
            raw = self.sensor_adapter.get_adc(id=sensor_id, port=SENSOR_PORT)
            raw = float(raw)
        except Exception as exc:
            print(f"Sharp {sensor_id} read error: {exc}")
            return 0, None

        buf = self.left_adc_buf if left else self.right_adc_buf
        buf.append(raw)
        med = statistics.median(buf)

        if left:
            self.left_ema = med if self.left_ema is None else (
                SHARP_EMA_NEW_WEIGHT * med + SHARP_EMA_OLD_WEIGHT * self.left_ema
            )
            ema = self.left_ema
            table = CALIBRATION_SHARP_LEFT
        else:
            self.right_ema = med if self.right_ema is None else (
                SHARP_EMA_NEW_WEIGHT * med + SHARP_EMA_OLD_WEIGHT * self.right_ema
            )
            ema = self.right_ema
            table = CALIBRATION_SHARP_RIGHT

        return int(round(raw)), self.adc_to_cm(ema, table)

    def read_left(self):
        return self._read_sharp(SHARP_LEFT_ID, left=True)

    def read_right(self):
        return self._read_sharp(SHARP_RIGHT_ID, left=False)

    def reset_sharp_filters(self) -> None:
        """Discard readings that belong to the robot's previous orientation."""
        self.left_adc_buf.clear()
        self.right_adc_buf.clear()
        self.left_ema = None
        self.right_ema = None

    def _read_digital_ir(self, sensor_id: Optional[int], history: deque) -> Optional[bool]:
        if sensor_id is None:
            return None

        level = None

        try:
            if sensor_id == IR_FRONT_LEFT_ID:
                level = self.sensor_adapter.get_io(
                    id=sensor_id,
                    port=IR_FRONT_LEFT_PORT,
                )
            elif sensor_id == IR_FRONT_RIGHT_ID:
                level = self.sensor_adapter.get_io(
                    id=sensor_id,
                    port=IR_FRONT_RIGHT_PORT,
                )
        except Exception as exc:
            print(f"Digital IR {sensor_id} read error: {exc}")
            return None

        if level is None:
            return None

        # Hardware convention:
        # raw 0 -> obstacle
        # raw 1 -> clear
        blocked = int(level) == int(IR_BLOCKED_LEVEL)
        history.append(bool(blocked))
        if len(history) < history.maxlen:
            return None
        return all(history)

    def read_front_corner_ir(self) -> Tuple[Optional[bool], Optional[bool]]:
        left = self._read_digital_ir(IR_FRONT_LEFT_ID, self.ir_left_hist)
        right = self._read_digital_ir(IR_FRONT_RIGHT_ID, self.ir_right_hist)
        return left, right

    def reset_filters(self):
        self.reset_sharp_filters()
        self.tof_buf.clear()
        self.ir_left_hist.clear()
        self.ir_right_hist.clear()
        self.front_cm = None
        self.tof_last_update = None


# ============================================================
# Heading manager and motion control
# ============================================================

class HeadingManager:
    """Maps logical N/E/S/W to absolute RoboMaster attitude yaw."""

    def __init__(self):
        self.base_yaw: Optional[float] = None
        self.target_yaw: Optional[float] = None
        self.heading_index = 0
        # With supplied robot logs: logical RIGHT causes +90 attitude yaw.
        right_command_sign = 1 if (-90.0) > 0 else -1
        self.right_yaw_step_sign = right_command_sign * DEFAULT_MOVE_TO_YAW_SIGN
        self.recovering = False

    def initialize(self, yaw: Optional[float]):
        if yaw is None:
            return False
        self.base_yaw = normalize_angle_deg(yaw)
        self.target_yaw = self.base_yaw
        self.heading_index = 0
        return True

    def initialize_for_heading(self, yaw: Optional[float], heading_index: int) -> bool:
        """Declare which map direction the robot physically faces right now."""
        if yaw is None:
            return False
        heading_index %= 4
        step = {0: 0, 1: 1, 2: 2, 3: -1}[heading_index]
        self.base_yaw = normalize_angle_deg(
            float(yaw) - self.right_yaw_step_sign * 90.0 * step
        )
        self.heading_index = heading_index
        self.target_yaw = normalize_angle_deg(float(yaw))
        self.recovering = False
        return True

    def set_heading_index(self, index: int):
        self.heading_index = index % 4
        if self.base_yaw is None:
            return
        step = {0: 0, 1: 1, 2: 2, 3: -1}[self.heading_index]
        self.target_yaw = normalize_angle_deg(
            self.base_yaw + self.right_yaw_step_sign * 90.0 * step
        )

    def error(self, current_yaw: Optional[float]) -> Optional[float]:
        if self.target_yaw is None or current_yaw is None:
            return None
        return shortest_angle_error_deg(self.target_yaw, current_yaw)

    def correction_z(self, current_yaw: Optional[float], recover=False) -> Tuple[float, Optional[float]]:
        if not ENABLE_HEADING_HOLD:
            return 0.0, None
        error = self.error(current_yaw)
        if error is None:
            return 0.0, None
        if abs(error) <= HEADING_DEADBAND_DEG:
            return 0.0, error

        max_z = HEADING_RECOVER_MAX_Z_SPEED if recover else HEADING_MAX_Z_SPEED
        desired_yaw_rate = clamp(error * HEADING_KP_Z, -max_z, max_z)
        z_cmd = desired_yaw_rate / DEFAULT_DRIVE_TO_YAW_SIGN
        return z_cmd, error

    def apply(self, x: float, y: float, current_yaw: Optional[float], mode: str):
        error = self.error(current_yaw)
        if error is None:
            return x, y, 0.0, mode, None

        if self.recovering:
            if abs(error) <= HEADING_RECOVER_RELEASE_DEG:
                self.recovering = False
        elif abs(error) >= HEADING_RECOVER_TRIGGER_DEG:
            self.recovering = True

        if self.recovering:
            z, error = self.correction_z(current_yaw, recover=True)
            return 0.0, 0.0, z, "HEADING_RECOVER", error

        z, error = self.correction_z(current_yaw, recover=False)
        return x, y, z, mode, error


class WallController:
    def __init__(self):
        self.left_wall = False
        self.right_wall = False
        self.center_owner = "NONE"
        self.center_owner_since = 0.0

    @staticmethod
    def forward_speed(front_cm: Optional[float]) -> float:
        if front_cm is None:
            return UNKNOWN_FRONT_SPEED
        if front_cm >= SLOW_FRONT_CM:
            return FORWARD_SPEED
        if front_cm <= STOP_FRONT_CM:
            return 0.0
        ratio = (front_cm - STOP_FRONT_CM) / (SLOW_FRONT_CM - STOP_FRONT_CM)
        return MIN_FORWARD_SPEED + ratio * (FORWARD_SPEED - MIN_FORWARD_SPEED)

    @staticmethod
    def _wall_state(distance_cm: Optional[float], current: bool) -> bool:
        if distance_cm is None:
            return False
        # A physically open side is never simultaneously treated as a wall.
        if distance_cm >= SIDE_OPEN_ENTER_CM:
            return False
        if current:
            return distance_cm < SIDE_WALL_EXIT_CM
        return distance_cm < SIDE_WALL_ENTER_CM

    def reset(self):
        self.left_wall = False
        self.right_wall = False
        self.center_owner = "NONE"
        self.center_owner_since = 0.0

    def _center_between_walls(self, left_cm: float, right_cm: float):
        now = time.monotonic()
        delta = left_cm - right_cm
        abs_delta = abs(delta)

        if self.center_owner == "NONE":
            if abs_delta < CENTER_TRIGGER_CM:
                return 0.0, "CENTER_STABLE"
            self.center_owner = "LEFT" if delta < 0 else "RIGHT"
            self.center_owner_since = now

        if now - self.center_owner_since >= CENTER_HOLD_SEC:
            if abs_delta <= CENTER_RELEASE_CM:
                self.center_owner = "NONE"
                return 0.0, "CENTER_RELEASE"
            if self.center_owner == "LEFT" and delta >= CENTER_TRIGGER_CM:
                self.center_owner = "RIGHT"
                self.center_owner_since = now
            elif self.center_owner == "RIGHT" and delta <= -CENTER_TRIGGER_CM:
                self.center_owner = "LEFT"
                self.center_owner_since = now

        correction = clamp(abs_delta * CENTER_KP_STRAFE, 0.0, CENTER_MAX_Y)
        if self.center_owner == "LEFT":
            return +correction * Y_DIR_SIGN, "CENTER_LEFT_OWNER"
        return -correction * Y_DIR_SIGN, "CENTER_RIGHT_OWNER"

    def lateral(self, left_cm: Optional[float], right_cm: Optional[float]):
        if left_cm is None or right_cm is None:
            self.reset()
            return 0.0, "NO_SIDE_SENSOR"

        if left_cm <= SIDE_TOO_CLOSE_CM and right_cm <= SIDE_TOO_CLOSE_CM:
            self.center_owner = "NONE"
            return 0.0, "BOTH_TOO_CLOSE"
        if left_cm <= SIDE_TOO_CLOSE_CM:
            self.center_owner = "NONE"
            return +ESCAPE_Y_SPEED * Y_DIR_SIGN, "ESCAPE_LEFT"
        if right_cm <= SIDE_TOO_CLOSE_CM:
            self.center_owner = "NONE"
            return -ESCAPE_Y_SPEED * Y_DIR_SIGN, "ESCAPE_RIGHT"

        # Start avoidance earlier. The V2 field log repeatedly reached the
        # Sharp calibration floor (~5 cm) before ESCAPE engaged, which is late
        # enough for a front chassis corner to scrape the wall.
        if left_cm < SIDE_WARNING_CM and right_cm >= SIDE_WARNING_CM:
            self.center_owner = "NONE"
            return +SIDE_WARNING_Y_SPEED * Y_DIR_SIGN, "AVOID_LEFT"
        if right_cm < SIDE_WARNING_CM and left_cm >= SIDE_WARNING_CM:
            self.center_owner = "NONE"
            return -SIDE_WARNING_Y_SPEED * Y_DIR_SIGN, "AVOID_RIGHT"

        self.left_wall = self._wall_state(left_cm, self.left_wall)
        self.right_wall = self._wall_state(right_cm, self.right_wall)

        if self.left_wall and self.right_wall:
            return self._center_between_walls(left_cm, right_cm)

        self.center_owner = "NONE"

        if self.left_wall:
            error = left_cm - TARGET_LEFT_CM
            if abs(error) <= SIDE_DEADBAND_CM:
                return 0.0, "FOLLOW_LEFT"
            y = clamp(-error * SIDE_KP_STRAFE * Y_DIR_SIGN, -SIDE_MAX_Y, SIDE_MAX_Y)
            return y, "FOLLOW_LEFT"

        if self.right_wall:
            error = right_cm - TARGET_RIGHT_CM
            if abs(error) <= SIDE_DEADBAND_CM:
                return 0.0, "FOLLOW_RIGHT"
            y = clamp(error * SIDE_KP_STRAFE * Y_DIR_SIGN, -SIDE_MAX_Y, SIDE_MAX_Y)
            return y, "FOLLOW_RIGHT"

        return 0.0, "OPEN_SPACE"


def apply_front_corner_ir_guard(
    x: float,
    y: float,
    ir_left_blocked: Optional[bool],
    ir_right_blocked: Optional[bool],
) -> Tuple[float, float, str]:
    """Use the front-edge digital IRs only as collision guards, not topology."""
    left = ir_left_blocked is True
    right = ir_right_blocked is True

    if left and right:
        return 0.0, 0.0, "IR_BOTH_FRONT_STOP"
    if left:
        return min(x, IR_ESCAPE_FORWARD_SPEED), +IR_ESCAPE_Y_SPEED * Y_DIR_SIGN, "IR_LEFT_ESCAPE"
    if right:
        return min(x, IR_ESCAPE_FORWARD_SPEED), -IR_ESCAPE_Y_SPEED * Y_DIR_SIGN, "IR_RIGHT_ESCAPE"
    return x, y, ""


# ============================================================
# Junction detector
# ============================================================

@dataclass
class JunctionEvent:
    kind: str  # SIDE_WINDOW or FRONT_BLOCKED
    observed_front: bool = False
    observed_left: bool = False
    observed_right: bool = False
    start_xy: Optional[Tuple[float, float]] = None
    last_open_xy: Optional[Tuple[float, float]] = None
    end_xy: Optional[Tuple[float, float]] = None
    backtrack_m: float = 0.0


class JunctionDetector:
    def __init__(self):
        self.locked = False
        self.lock_xy: Optional[Tuple[float, float]] = None
        self.lock_time: Optional[float] = None
        self.clear_samples = 0

        self.front_block_count = 0
        self.left_enter_count = 0
        self.right_enter_count = 0
        self.side_close_count = 0

        self.window_active = False
        self.window_start_xy: Optional[Tuple[float, float]] = None
        self.last_open_xy: Optional[Tuple[float, float]] = None
        self.lookahead_start_xy: Optional[Tuple[float, float]] = None
        self.front_evidence = 0
        self.left_evidence = 0
        self.right_evidence = 0

    def reset_window(self):
        self.front_block_count = 0
        self.left_enter_count = 0
        self.right_enter_count = 0
        self.side_close_count = 0
        self.window_active = False
        self.window_start_xy = None
        self.last_open_xy = None
        self.lookahead_start_xy = None
        self.front_evidence = 0
        self.left_evidence = 0
        self.right_evidence = 0

    @staticmethod
    def side_still_open(cm: Optional[float]) -> bool:
        return cm is not None and cm >= SIDE_OPEN_EXIT_CM

    @staticmethod
    def side_enter_open(cm: Optional[float]) -> bool:
        return cm is not None and cm >= SIDE_OPEN_ENTER_CM

    @staticmethod
    def front_is_open(cm: Optional[float]) -> bool:
        return cm is not None and cm >= EXPLORATION_FRONT_OPEN_CM

    @staticmethod
    def front_is_blocked(cm: Optional[float]) -> bool:
        return cm is not None and 0.0 < cm <= STOP_FRONT_CM

    def lock_here(self, pose_xy: Tuple[Optional[float], Optional[float]]):
        self.locked = True
        if None not in pose_xy:
            self.lock_xy = (float(pose_xy[0]), float(pose_xy[1]))
        else:
            self.lock_xy = None
        self.lock_time = time.monotonic()
        self.clear_samples = 0
        self.reset_window()

    def _maybe_unlock(self, pose_xy, front_cm, left_cm, right_cm) -> bool:
        if not self.locked:
            return True

        now = time.monotonic()
        elapsed = now - self.lock_time if self.lock_time is not None else 0.0
        moved = distance_xy(self.lock_xy or (None, None), pose_xy)

        side_clear = not self.side_still_open(left_cm) and not self.side_still_open(right_cm)
        normal_corridor = side_clear and not self.front_is_blocked(front_cm)
        self.clear_samples = self.clear_samples + 1 if normal_corridor else 0

        by_corridor = (
            self.clear_samples >= JUNCTION_REARM_SAMPLES
            and (moved is None or moved >= JUNCTION_REARM_MIN_DISTANCE_M)
        )
        by_distance = (
            moved is not None and moved >= JUNCTION_REARM_DISTANCE_M and side_clear
        )
        by_timeout = (
            elapsed >= JUNCTION_REARM_TIMEOUT_SEC
            and moved is not None
            and moved >= JUNCTION_REARM_MIN_DISTANCE_M
            and side_clear
        )
        by_emergency_front = (
            self.front_is_blocked(front_cm)
            and elapsed >= JUNCTION_REARM_EMERGENCY_SEC
        )

        if by_corridor or by_distance or by_timeout or by_emergency_front:
            self.locked = False
            self.lock_xy = None
            self.lock_time = None
            self.clear_samples = 0
            self.reset_window()
            return True
        return False

    def _start_window(self, pose_xy):
        self.window_active = True
        self.window_start_xy = pose_xy if None not in pose_xy else None
        self.last_open_xy = self.window_start_xy
        self.lookahead_start_xy = None
        self.side_close_count = 0
        print(">>> JUNCTION WINDOW START")

    def update(self, front_cm, left_cm, right_cm, pose_xy) -> Optional[JunctionEvent]:
        if not self._maybe_unlock(pose_xy, front_cm, left_cm, right_cm):
            return None

        front_blocked = self.front_is_blocked(front_cm)
        self.front_block_count = self.front_block_count + 1 if front_blocked else 0

        left_enter = self.side_enter_open(left_cm)
        right_enter = self.side_enter_open(right_cm)
        self.left_enter_count = self.left_enter_count + 1 if left_enter else 0
        self.right_enter_count = self.right_enter_count + 1 if right_enter else 0

        if not self.window_active:
            if (
                self.left_enter_count >= OPENING_ENTER_SAMPLES
                or self.right_enter_count >= OPENING_ENTER_SAMPLES
            ):
                self._start_window(pose_xy)
                self.left_evidence = self.left_enter_count
                self.right_evidence = self.right_enter_count

            elif self.front_block_count >= JUNCTION_CONFIRM_SAMPLES:
                event = JunctionEvent(kind="FRONT_BLOCKED")
                self.lock_here(pose_xy)
                return event
            else:
                return None

        # Window is active.
        if self.front_is_open(front_cm):
            self.front_evidence += 1
        if left_enter:
            self.left_evidence += 1
        if right_enter:
            self.right_evidence += 1

        left_still = self.side_still_open(left_cm)
        right_still = self.side_still_open(right_cm)

        if left_still or right_still:
            if None not in pose_xy:
                self.last_open_xy = (float(pose_xy[0]), float(pose_xy[1]))
            self.lookahead_start_xy = None
            self.side_close_count = 0
        else:
            self.side_close_count += 1
            if self.side_close_count >= OPENING_EXIT_SAMPLES and self.lookahead_start_xy is None:
                self.lookahead_start_xy = pose_xy if None not in pose_xy else None

        # A real front stop while inside the opening window is a valid decision.
        if self.front_block_count >= JUNCTION_CONFIRM_SAMPLES:
            return self._finalize_window(pose_xy, "FRONT_BLOCKED_IN_WINDOW")

        # Finalize after a short look-ahead past the far edge.
        if self.lookahead_start_xy is not None:
            lookahead = distance_xy(self.lookahead_start_xy, pose_xy)
            if lookahead is not None and lookahead >= OPENING_LOOKAHEAD_M:
                return self._finalize_window(pose_xy, "LOOKAHEAD_COMPLETE")

        # Hard maximum prevents a never-ending opening window.
        total = distance_xy(self.window_start_xy or (None, None), pose_xy)
        if total is not None and total >= OPENING_MAX_LENGTH_M:
            return self._finalize_window(pose_xy, "MAX_WINDOW")

        return None

    def _finalize_window(self, pose_xy, reason: str) -> JunctionEvent:
        total = distance_xy(self.window_start_xy or (None, None), pose_xy) or 0.0
        span = distance_xy(
            self.window_start_xy or (None, None),
            self.last_open_xy or pose_xy,
        ) or total

        # Current position is after the opening. Reverse to its estimated centre.
        backtrack = max(0.0, total - 0.5 * span)
        backtrack = min(backtrack, OPENING_CENTER_MAX_BACKTRACK_M)

        event = JunctionEvent(
            kind="SIDE_WINDOW",
            observed_front=self.front_evidence >= OPENING_MIN_EVIDENCE_SAMPLES,
            observed_left=self.left_evidence >= OPENING_MIN_EVIDENCE_SAMPLES,
            observed_right=self.right_evidence >= OPENING_MIN_EVIDENCE_SAMPLES,
            start_xy=self.window_start_xy,
            last_open_xy=self.last_open_xy,
            end_xy=pose_xy if None not in pose_xy else None,
            backtrack_m=backtrack,
        )

        print(
            f">>> JUNCTION WINDOW END reason={reason} total={total:.3f}m "
            f"span={span:.3f}m backtrack={backtrack:.3f}m "
            f"evidence F/L/R={self.front_evidence}/{self.left_evidence}/{self.right_evidence}"
        )
        self.lock_here(pose_xy)
        return event


# ============================================================
# Open-space exit detector
# ============================================================

class ExitDetector:
    def __init__(self):
        self.active = False
        self.start_xy: Optional[Tuple[float, float]] = None
        self.start_time: Optional[float] = None
        self.reset_count = 0

    def reset(self):
        self.active = False
        self.start_xy = None
        self.start_time = None
        self.reset_count = 0

    def update(
        self,
        front_cm: Optional[float],
        left_cm: Optional[float],
        right_cm: Optional[float],
        pose_xy,
        ir_left_blocked: Optional[bool],
        ir_right_blocked: Optional[bool],
    ) -> bool:
        if not ENABLE_EXIT_DETECTION:
            return False

        open_space = (
            front_cm is not None
            and left_cm is not None
            and right_cm is not None
            and front_cm >= EXIT_FRONT_OPEN_CM
            and left_cm >= EXIT_SIDE_OPEN_CM
            and right_cm >= EXIT_SIDE_OPEN_CM
            and ir_left_blocked is not True
            and ir_right_blocked is not True
        )

        if open_space:
            self.reset_count = 0
            if not self.active:
                if None in pose_xy:
                    return False
                self.active = True
                self.start_xy = (float(pose_xy[0]), float(pose_xy[1]))
                self.start_time = time.monotonic()
                print(">>> EXIT CANDIDATE: sustained open space started")
                return False

            moved = distance_xy(self.start_xy or (None, None), pose_xy)
            elapsed = time.monotonic() - self.start_time if self.start_time is not None else 0.0
            if (
                moved is not None
                and moved >= EXIT_CONFIRM_DISTANCE_M
                and elapsed >= EXIT_CONFIRM_MIN_SEC
            ):
                return True
            return False

        if self.active:
            self.reset_count += 1
            if self.reset_count >= EXIT_RESET_SAMPLES:
                self.reset()
        return False


# ============================================================
# Junction graph / Trémaux memory
# ============================================================

@dataclass
class EdgeState:
    # confirmed traversals only: incremented after arriving at a DIFFERENT node
    traversals: int = 0
    target: Optional[str] = None
    observed: bool = False
    attempts: int = 0
    blocked: bool = False


@dataclass
class JunctionNode:
    node_id: str
    x: float
    y: float
    gx: int
    gy: int
    exits: Dict[int, EdgeState] = field(default_factory=dict)
    seen_count: int = 1


@dataclass
class Plan:
    relative: str
    absolute: int
    reason: str


class MazeGraphExplorer:
    def __init__(self):
        self.nodes: Dict[str, JunctionNode] = {}
        self.next_index = 0
        self.current_node: Optional[str] = None
        self.start_node: Optional[str] = None
        self.first_junction_node: Optional[str] = None
        # The first corridor (start -> first junction) is the entrance corridor.
        # Once we have entered the maze, graph routing must never use this edge
        # as a transit route back outside.
        self.entry_edge_nodes: Optional[Tuple[str, str]] = None
        self.heading_index = 0

        # Corridor currently being traversed from a known node.
        self.pending_from: Optional[str] = None
        self.pending_abs: Optional[int] = None
        self.pending_start_xy: Optional[Tuple[float, float]] = None

        self.history: List[dict] = []
        self.graph_events: List[dict] = []

    # ---------------- headings ----------------
    @staticmethod
    def opposite(abs_dir: int) -> int:
        return (abs_dir + 2) % 4

    def absolute_for_relative(self, relative: str) -> int:
        return (self.heading_index + RELATIVE_OFFSET[relative]) % 4

    def relative_for_absolute(self, abs_dir: int) -> str:
        diff = (abs_dir - self.heading_index) % 4
        return {0: "FRONT", 1: "RIGHT", 2: "BACK", 3: "LEFT"}[diff]

    # ---------------- grid ----------------
    @staticmethod
    def grid_xy(x: float, y: float) -> Tuple[int, int]:
        return (
            int(round(x / GRID_RESOLUTION_M)),
            int(round(y / GRID_RESOLUTION_M)),
        )

    def _new_node(self, x: float, y: float) -> str:
        node_id = f"J{self.next_index}"
        self.next_index += 1
        gx, gy = self.grid_xy(x, y)
        self.nodes[node_id] = JunctionNode(node_id, float(x), float(y), gx, gy)
        return node_id

    def initialize_start(self, x: float, y: float) -> str:
        node_id = self._new_node(x, y)
        self.start_node = node_id
        self.current_node = node_id
        return node_id

    def _edge(self, node_id: str, abs_dir: int) -> EdgeState:
        node = self.nodes[node_id]
        abs_dir %= 4
        if abs_dir not in node.exits:
            node.exits[abs_dir] = EdgeState()
        return node.exits[abs_dir]

    def _touch_node(self, node_id: str, x: float, y: float):
        node = self.nodes[node_id]
        alpha = NODE_POSITION_UPDATE_ALPHA
        node.x = (1.0 - alpha) * node.x + alpha * x
        node.y = (1.0 - alpha) * node.y + alpha * y
        node.gx, node.gy = self.grid_xy(node.x, node.y)
        node.seen_count += 1

    def _expected_target(self, x: float, y: float, incoming_abs: Optional[int]) -> Optional[str]:
        if self.pending_from is None or self.pending_abs is None:
            return None
        if self.pending_from not in self.nodes:
            return None
        state = self.nodes[self.pending_from].exits.get(self.pending_abs)
        if state is None or state.target is None or state.target not in self.nodes:
            return None

        target = self.nodes[state.target]
        d = math.hypot(x - target.x, y - target.y)
        if d > EXPECTED_TARGET_MATCH_RADIUS_M:
            return None

        # The expected node should expose the reverse corridor.
        if incoming_abs is not None:
            incoming_state = target.exits.get(incoming_abs)
            if incoming_state is not None and incoming_state.target not in (None, self.pending_from):
                return None
        return target.node_id

    def _topology_compatible(self, node: JunctionNode, observed_abs: set, incoming_abs: Optional[int]) -> bool:
        # Incoming corridor is the strongest identity feature for a revisit.
        if incoming_abs is not None:
            incoming_state = node.exits.get(incoming_abs)
            if incoming_state is None:
                # For extended-distance loop closure, require that the candidate
                # at least already knows the corridor by which we are arriving.
                return False

        known_open = set(node.exits.keys())
        if not known_open:
            return True

        # Require at least one non-incoming overlap if the candidate has one.
        candidate_other = known_open - ({incoming_abs} if incoming_abs is not None else set())
        observed_other = observed_abs - ({incoming_abs} if incoming_abs is not None else set())
        if candidate_other and observed_other and not (candidate_other & observed_other):
            return False
        return True

    def _match_node(self, x: float, y: float, observed_abs: set, incoming_abs: Optional[int]):
        expected = self._expected_target(x, y, incoming_abs)
        if expected is not None:
            return expected, "EXPECTED_TARGET"

        candidates = []
        for node_id, node in self.nodes.items():
            d = math.hypot(x - node.x, y - node.y)

            # V2: allow a very-close event to collapse back into the node we
            # just departed.  This is intentionally a *special* match, not a
            # normal node merge.  It handles a wide junction/corner that
            # re-triggers the detector 10-30 cm after departure.
            if self.pending_from is not None and node_id == self.pending_from:
                if d <= SAME_NODE_RETRIGGER_RADIUS_M:
                    candidates.append((d, -1, node_id))
                continue

            if d <= NODE_MATCH_RADIUS_M:
                candidates.append((d, 0, node_id))
            elif d <= LOOP_MATCH_RADIUS_M and self._topology_compatible(node, observed_abs, incoming_abs):
                candidates.append((d, 1, node_id))

        if not candidates:
            return None, None

        candidates.sort(key=lambda item: (item[1], item[0]))
        d, match_class, node_id = candidates[0]
        if match_class == -1:
            return node_id, "SAME_NODE_RETRIGGER"
        if match_class == 1:
            return node_id, "TOPOLOGY_LOOP_MATCH"
        return node_id, "NEAR_POSITION"

    def _connect(self, a: str, abs_a: int, b: str):
        if a == b:
            return
        abs_a %= 4
        abs_b = self.opposite(abs_a)
        ea = self._edge(a, abs_a)
        eb = self._edge(b, abs_b)

        if ea.target not in (None, b):
            self.graph_events.append({
                "kind": "EDGE_CONFLICT",
                "from": a,
                "heading": HEADINGS[abs_a],
                "existing": ea.target,
                "attempted": b,
            })
            return
        if eb.target not in (None, a):
            self.graph_events.append({
                "kind": "EDGE_CONFLICT_REVERSE",
                "from": b,
                "heading": HEADINGS[abs_b],
                "existing": eb.target,
                "attempted": a,
            })
            return

        ea.target = b
        eb.target = a
        shared = max(ea.traversals, eb.traversals)
        ea.traversals = shared
        eb.traversals = shared
        ea.observed = True
        eb.observed = True
        ea.blocked = False
        eb.blocked = False

    def _confirm_arrival_traversal(self, from_node: str, abs_dir: int, to_node: str):
        """Count an edge only after the robot reaches a different node."""
        if from_node == to_node:
            return
        ea = self._edge(from_node, abs_dir)
        eb = self._edge(to_node, self.opposite(abs_dir))
        new_count = max(ea.traversals, eb.traversals) + 1
        ea.traversals = new_count
        eb.traversals = new_count
        ea.blocked = False
        eb.blocked = False

    def pending_progress_m(self, x: float, y: float) -> Optional[float]:
        if self.pending_start_xy is None:
            return None
        sx, sy = self.pending_start_xy
        return math.hypot(float(x) - sx, float(y) - sy)

    def cancel_pending_as_blocked(self, reason: str, x: float, y: float):
        """Cancel a short failed departure without pretending it was traversed."""
        if self.pending_from is None or self.pending_abs is None:
            return
        state = self._edge(self.pending_from, self.pending_abs)
        state.blocked = True
        progress = self.pending_progress_m(x, y)
        self.graph_events.append({
            "kind": "FAILED_DEPARTURE_BLOCKED",
            "node": self.pending_from,
            "heading": HEADINGS[self.pending_abs],
            "progress_m": progress,
            "reason": reason,
        })
        self.current_node = self.pending_from
        self.pending_from = None
        self.pending_abs = None
        self.pending_start_xy = None

    def arrive(self, x: float, y: float, observed_abs: set) -> Tuple[str, bool, str]:
        previous_from = self.pending_from
        previous_abs = self.pending_abs
        incoming_abs = self.opposite(previous_abs) if previous_abs is not None else None
        matched, match_reason = self._match_node(x, y, observed_abs, incoming_abs)
        is_new = matched is None
        node_id = self._new_node(x, y) if is_new else matched

        same_node_retrigger = (
            not is_new
            and previous_from is not None
            and node_id == previous_from
            and match_reason == "SAME_NODE_RETRIGGER"
        )

        # Do not drag the canonical node position toward the far edge of the
        # same physical intersection on a re-trigger.
        if not is_new and not same_node_retrigger:
            self._touch_node(node_id, x, y)

        if previous_from is not None and previous_abs is not None:
            if same_node_retrigger:
                progress = self.pending_progress_m(x, y)
                self.graph_events.append({
                    "kind": "SAME_NODE_RETRIGGER",
                    "node": node_id,
                    "heading": HEADINGS[previous_abs],
                    "distance_m": math.hypot(
                        x - self.nodes[node_id].x,
                        y - self.nodes[node_id].y,
                    ),
                    "departure_progress_m": progress,
                })
                # IMPORTANT V3: do NOT clear pending_from/pending_abs and do NOT
                # increment traversal here. This is not a successful corridor
                # traversal. Main decides whether to keep moving through a wide
                # junction or cancel a short hard-blocked false exit.
                self.current_node = node_id
                return node_id, False, "SAME_NODE_RETRIGGER"
            else:
                self._connect(previous_from, previous_abs, node_id)
                self._confirm_arrival_traversal(previous_from, previous_abs, node_id)

                # First real junction reached from the synthetic start node.
                if (
                    self.start_node is not None
                    and previous_from == self.start_node
                    and node_id != self.start_node
                    and self.entry_edge_nodes is None
                ):
                    self.first_junction_node = node_id
                    self.entry_edge_nodes = (self.start_node, node_id)
                    self.graph_events.append({
                        "kind": "ENTRY_EDGE_LOCKED",
                        "start": self.start_node,
                        "first_junction": node_id,
                    })

        self.current_node = node_id
        self.pending_from = None
        self.pending_abs = None
        self.pending_start_xy = None

        return node_id, is_new, ("NEW" if is_new else match_reason or "MATCH")

    def observe_openings(self, observed_abs: set):
        if self.current_node is None:
            return
        for abs_dir in observed_abs:
            state = self._edge(self.current_node, abs_dir)
            state.observed = True

    def frontiers(self, node_id: Optional[str] = None) -> List[int]:
        if node_id is None:
            node_id = self.current_node
        if node_id is None:
            return []
        result = []
        for abs_dir, state in self.nodes[node_id].exits.items():
            if (
                state.observed
                and state.traversals == 0
                and state.target is None
                and not state.blocked
            ):
                result.append(abs_dir)
        return sorted(result)

    def all_frontiers(self) -> List[Tuple[str, int]]:
        items = []
        for node_id in sorted(self.nodes, key=lambda n: int(n[1:]) if n[1:].isdigit() else 999999):
            for abs_dir in self.frontiers(node_id):
                items.append((node_id, abs_dir))
        return items

    def _is_entry_edge(self, a: str, b: str) -> bool:
        if self.entry_edge_nodes is None:
            return False
        s, j = self.entry_edge_nodes
        return (a == s and b == j) or (a == j and b == s)

    def _neighbors(self, node_id: str):
        for abs_dir, state in self.nodes[node_id].exits.items():
            if state.target is None or state.target not in self.nodes:
                continue
            # V2: the start corridor is one-way for exploration.  It was used
            # once to enter the maze, but BFS must not route back through it.
            if self._is_entry_edge(node_id, state.target):
                continue
            yield state.target, abs_dir

    def _path_to_nearest_frontier(self) -> Optional[List[str]]:
        if self.current_node is None:
            return None
        q = deque([[self.current_node]])
        visited = {self.current_node}

        while q:
            path = q.popleft()
            node_id = path[-1]
            if node_id != self.current_node and self.frontiers(node_id):
                return path
            for nxt, _ in self._neighbors(node_id):
                if nxt in visited:
                    continue
                visited.add(nxt)
                q.append(path + [nxt])
        return None

    def _abs_to_neighbor(self, node_id: str, neighbor_id: str) -> Optional[int]:
        for abs_dir, state in self.nodes[node_id].exits.items():
            if state.target == neighbor_id:
                return abs_dir
        return None

    def plan(self, current_observed_abs: Optional[set] = None) -> Optional[Plan]:
        if self.current_node is None:
            raise RuntimeError("No current node")

        # 1) DFS/Trémaux: prefer a frontier physically confirmed by the CURRENT
        # centred scan. V2 could remember FRONT as open from an older moving
        # window, then drive straight even while current ToF was ~10 cm.
        local_frontiers = self.frontiers(self.current_node)
        rank = {name: i for i, name in enumerate(EXPLORATION_PREFERENCE)}

        if local_frontiers and current_observed_abs is not None:
            physical = [d for d in local_frontiers if d in current_observed_abs]
            if physical:
                choices = []
                for abs_dir in physical:
                    rel = self.relative_for_absolute(abs_dir)
                    choices.append((rank.get(rel, 99), rel, abs_dir))
                choices.sort()
                _, rel, abs_dir = choices[0]
                return Plan(rel, abs_dir, "LOCAL_CONFIRMED_UNVISITED_EXIT")

            # A remembered side branch can still be useful if the stopped Sharp
            # misses the mouth by a few cm. Turning toward it is safe because
            # front ToF validates the branch after the turn. Never force a
            # remembered FRONT when current ToF says it is blocked.
            side_memory = []
            for abs_dir in local_frontiers:
                rel = self.relative_for_absolute(abs_dir)
                if rel in ("LEFT", "RIGHT"):
                    side_memory.append((rank.get(rel, 99), rel, abs_dir))
            if side_memory:
                side_memory.sort()
                _, rel, abs_dir = side_memory[0]
                return Plan(rel, abs_dir, "REMEMBERED_SIDE_FRONTIER")

        elif local_frontiers:
            choices = []
            for abs_dir in local_frontiers:
                rel = self.relative_for_absolute(abs_dir)
                choices.append((rank.get(rel, 99), rel, abs_dir))
            choices.sort()
            _, rel, abs_dir = choices[0]
            return Plan(rel, abs_dir, "LOCAL_UNVISITED_EXIT")

        # 2) If no frontier exists anywhere, graph exploration is complete.
        if not self.all_frontiers():
            return None

        # 3) Transit through known graph to nearest unexplored frontier.
        path = self._path_to_nearest_frontier()
        if path and len(path) >= 2:
            abs_dir = self._abs_to_neighbor(self.current_node, path[1])
            if abs_dir is not None:
                return Plan(
                    self.relative_for_absolute(abs_dir),
                    abs_dir,
                    "ROUTE_TO_NEAREST_FRONTIER",
                )

        raise RuntimeError(
            "Frontiers exist but none are reachable in the current graph. "
            "This indicates a node/edge matching error."
        )

    def commit_departure(self, plan: Plan, start_xy: Optional[Tuple[float, float]] = None):
        if self.current_node is None:
            raise RuntimeError("No current node")

        state = self._edge(self.current_node, plan.absolute)
        state.attempts += 1

        # V3: traversal count is NOT incremented here. A corridor is counted only
        # after arrive() reaches a different node and confirms the connection.
        self.pending_from = self.current_node
        self.pending_abs = plan.absolute
        self.pending_start_xy = start_xy
        self.heading_index = plan.absolute
        self.history.append({
            "time": time.time(),
            "node": self.current_node,
            "relative": plan.relative,
            "absolute": HEADINGS[plan.absolute],
            "reason": plan.reason,
            "edge_traversals_confirmed": state.traversals,
            "edge_attempts": state.attempts,
            "frontiers_remaining": len(self.all_frontiers()),
        })

    def describe_node(self, node_id: Optional[str] = None) -> str:
        node_id = node_id or self.current_node
        if node_id is None or node_id not in self.nodes:
            return "NO_NODE"
        node = self.nodes[node_id]
        parts = []
        for abs_dir in range(4):
            if abs_dir not in node.exits:
                continue
            e = node.exits[abs_dir]
            frontier = e.observed and e.traversals == 0 and e.target is None and not e.blocked
            tag = "*" if frontier else ("[X]" if e.blocked else "")
            parts.append(
                f"{HEADINGS[abs_dir]}:{e.traversals}->{e.target or '?'}"
                f"(a{e.attempts}){tag}"
            )
        return " | ".join(parts) if parts else "NO_EXITS"

    def save(self, filepath=MAZE_MEMORY_FILE):
        data = {
            "version": PROGRAM_VERSION,
            "current_node": self.current_node,
            "start_node": self.start_node,
            "first_junction_node": self.first_junction_node,
            "entry_edge_nodes": list(self.entry_edge_nodes) if self.entry_edge_nodes else None,
            "heading": HEADINGS[self.heading_index],
            "pending_from": self.pending_from,
            "pending_heading": HEADINGS[self.pending_abs] if self.pending_abs is not None else None,
            "frontiers": [
                {"node": n, "heading": HEADINGS[h]} for n, h in self.all_frontiers()
            ],
            "nodes": {},
            "history": self.history,
            "graph_events": self.graph_events,
        }
        for node_id, node in self.nodes.items():
            data["nodes"][node_id] = {
                "x": node.x,
                "y": node.y,
                "grid": [node.gx, node.gy],
                "seen_count": node.seen_count,
                "exits": {
                    HEADINGS[h]: {
                        "traversals": e.traversals,
                        "target": e.target,
                        "observed": e.observed,
                        "attempts": e.attempts,
                        "blocked": e.blocked,
                        "frontier": (
                            e.observed
                            and e.traversals == 0
                            and e.target is None
                            and not e.blocked
                        ),
                    }
                    for h, e in node.exits.items()
                },
            }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)


# ============================================================
# Physical actions
# ============================================================

def stop_chassis(chassis):
    if chassis is None or not ENABLE_MOTION:
        return
    try:
        chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
    except Exception:
        pass


def wait_for_position(pose: PoseTracker) -> Tuple[float, float]:
    deadline = time.monotonic() + POSE_WAIT_SEC
    while time.monotonic() < deadline:
        if pose.has_position():
            x, y = pose.get_xy()
            return float(x), float(y)
        time.sleep(0.05)
    print("WARNING: odometry not ready; using (0,0)")
    return 0.0, 0.0


def wait_for_yaw(pose: PoseTracker) -> Optional[float]:
    deadline = time.monotonic() + POSE_WAIT_SEC
    while time.monotonic() < deadline:
        yaw = pose.get_yaw()
        if yaw is not None:
            return yaw
        time.sleep(0.05)
    print("WARNING: attitude yaw not ready")
    return None


def travelled_from(start_xy, pose: PoseTracker) -> Optional[float]:
    return distance_xy(start_xy, pose.get_xy())


def feedback_turn(chassis, sensors: SensorManager, pose: PoseTracker, relative: str) -> bool:
    angle = RELATIVE_TURN_DEG[relative]
    if abs(angle) < 0.001:
        return True
    if not ENABLE_MOTION:
        return True

    start_yaw = pose.get_yaw()
    if start_yaw is None:
        print("TURN ERROR: yaw unavailable")
        return False

    # Preserve supplied robot convention:
    target_yaw = normalize_angle_deg(start_yaw + angle * DEFAULT_MOVE_TO_YAW_SIGN)
    timeout = TURN_FEEDBACK_TIMEOUT_180_SEC if abs(angle) > 135 else TURN_FEEDBACK_TIMEOUT_90_SEC

    stop_chassis(chassis)
    time.sleep(TURN_PRE_SETTLE_SEC)
    started = time.monotonic()
    stable = 0

    print(f">>> TURN {relative}: yaw {start_yaw:+.1f} -> {target_yaw:+.1f}")

    while time.monotonic() - started < timeout:
        yaw = pose.get_yaw()
        if yaw is None:
            time.sleep(TURN_FEEDBACK_LOOP_SEC)
            continue

        # --- ส่วนที่เพิ่มใหม่: ระบบ IR หยุดรถและสไลด์หนีขณะเลี้ยว ---
        ir_left, ir_right = sensors.read_front_corner_ir()
        if ir_left is True or ir_right is True:
            y_cmd = 0.0
            if ir_left:
                y_cmd = +IR_ESCAPE_Y_SPEED * Y_DIR_SIGN
                print(">>> TURN PAUSED: Left IR blocked! หยุดหมุนแล้วสไลด์หนีขวา...")
            elif ir_right:
                y_cmd = -IR_ESCAPE_Y_SPEED * Y_DIR_SIGN
                print(">>> TURN PAUSED: Right IR blocked! หยุดหมุนแล้วสไลด์หนีซ้าย...")

            # สั่งสไลด์ (y) โดยหยุดการเดินหน้า (x) และการหมุน (z) ทั้งหมด
            chassis.drive_speed(x=0.0, y=y_cmd, z=0.0, timeout=TURN_FEEDBACK_DRIVE_TIMEOUT_SEC)
            time.sleep(TURN_FEEDBACK_LOOP_SEC)
            
            # ชดเชยเวลา timeout ออกไป เพื่อไม่ให้การเลี้ยวล้มเหลวเพราะเสียเวลาหลบกำแพง
            started = time.monotonic()
            continue # ย้อนกลับไปเช็คใหม่จนกว่า IR จะว่าง ถึงจะยอมให้โค้ดลงไปหมุนต่อ
        # --------------------------------------------------

        err = shortest_angle_error_deg(target_yaw, yaw)
        if abs(err) <= TURN_FEEDBACK_TOLERANCE_DEG:
            stable += 1
            stop_chassis(chassis)
            if stable >= TURN_FEEDBACK_STABLE_SAMPLES:
                time.sleep(YAW_SETTLE_SEC)
                return True
        else:
            stable = 0
            speed = clamp(
                abs(err) * TURN_FEEDBACK_KP,
                TURN_FEEDBACK_MIN_Z_SPEED,
                TURN_FEEDBACK_MAX_Z_SPEED,
            )
            z_cmd = math.copysign(speed, err) / DEFAULT_DRIVE_TO_YAW_SIGN
            chassis.drive_speed(
                x=0.0,
                y=0.0,
                z=z_cmd,
                timeout=TURN_FEEDBACK_DRIVE_TIMEOUT_SEC,
            )
        time.sleep(TURN_FEEDBACK_LOOP_SEC)

    stop_chassis(chassis)
    print("TURN TIMEOUT")
    return False


def align_heading(chassis, pose: PoseTracker, heading: HeadingManager):
    if not ENABLE_MOTION or heading.target_yaw is None:
        return
    deadline = time.monotonic() + HEADING_ALIGN_TIMEOUT_SEC
    while time.monotonic() < deadline:
        yaw = pose.get_yaw()
        err = heading.error(yaw)
        if err is None or abs(err) <= HEADING_ALIGN_TOLERANCE_DEG:
            break
        z, _ = heading.correction_z(yaw, recover=True)
        chassis.drive_speed(x=0.0, y=0.0, z=z, timeout=DRIVE_TIMEOUT_SEC)
        time.sleep(HEADING_ALIGN_LOOP_SEC)
    stop_chassis(chassis)


def backtrack_to_opening_center(chassis, pose: PoseTracker, heading: HeadingManager, meters: float):
    if not ENABLE_MOTION or meters <= 0.005:
        return

    target = min(max(0.0, meters), OPENING_CENTER_MAX_BACKTRACK_M)
    start = pose.get_xy()
    started = time.monotonic()
    print(f">>> CENTER SIDE OPENING: reverse {target:.3f}m")

    while time.monotonic() - started < OPENING_CENTER_TIMEOUT_SEC:
        moved = travelled_from(start, pose)
        if moved is not None and moved >= target:
            break
        x, y, z, _, _ = heading.apply(
            -OPENING_CENTER_SPEED,
            0.0,
            pose.get_yaw(),
            "OPENING_CENTER",
        )
        chassis.drive_speed(x=x, y=y, z=z, timeout=DRIVE_TIMEOUT_SEC)
        time.sleep(OPENING_CENTER_LOOP_SEC)
    stop_chassis(chassis)


def center_front_blocked(chassis, sensors: SensorManager, pose: PoseTracker, heading: HeadingManager):
    """Move slowly toward the front wall until a repeatable turn distance."""
    if not ENABLE_MOTION:
        return

    start = pose.get_xy()
    started = time.monotonic()
    print(f">>> CENTER FRONT BLOCKED: target ToF={CORNER_CENTER_FRONT_TARGET_CM:.1f}cm")

    while time.monotonic() - started < CORNER_CENTER_TIMEOUT_SEC:
        front = sensors.get_front_cm()
        if front is None:
            break

        ir_left, ir_right = sensors.read_front_corner_ir()
        if ir_left is True or ir_right is True:
            print(
                "FRONT_CENTER stop: front-corner IR blocked "
                f"(L={ir_left}, R={ir_right})"
            )
            break

        if front <= CORNER_CENTER_FRONT_HARD_STOP_CM:
            break
        if front <= CORNER_CENTER_FRONT_TARGET_CM:
            break
        moved = travelled_from(start, pose)
        if moved is not None and moved >= CORNER_CENTER_MAX_DISTANCE_M:
            break

        x, y, z, _, _ = heading.apply(
            CORNER_CENTER_SPEED,
            0.0,
            pose.get_yaw(),
            "FRONT_CENTER",
        )
        chassis.drive_speed(x=x, y=y, z=z, timeout=DRIVE_TIMEOUT_SEC)
        time.sleep(CORNER_CENTER_LOOP_SEC)
    stop_chassis(chassis)


def post_turn_clearance(
    chassis,
    sensors: SensorManager,
    pose: PoseTracker,
    heading: HeadingManager,
    relative: str,
):
    """Crawl out of an inside corner before returning to full corridor speed."""
    if not ENABLE_POST_TURN_CLEARANCE or relative not in ("LEFT", "RIGHT"):
        return
    if not ENABLE_MOTION:
        return

    sensors.reset_filters()
    start = pose.get_xy()
    started = time.monotonic()

    # After LEFT turn the old inside corner is on the left; vice versa.
    read_inner = sensors.read_left if relative == "LEFT" else sensors.read_right
    y_out = (
        +POST_TURN_Y_SPEED * Y_DIR_SIGN
        if relative == "LEFT"
        else -POST_TURN_Y_SPEED * Y_DIR_SIGN
    )

    inner = None
    for _ in range(3):
        _, inner = read_inner()
        time.sleep(POST_TURN_LOOP_SEC)

    ir_left, ir_right = sensors.read_front_corner_ir()
    inner_ir = ir_left if relative == "LEFT" else ir_right

    if (
        inner is not None
        and inner >= POST_TURN_TRIGGER_CM
        and inner_ir is not True
    ):
        return

    print(
        f">>> POST_TURN_CLEARANCE {relative} "
        f"inner={fmt(inner)} IR={inner_ir}"
    )

    while time.monotonic() - started < POST_TURN_MAX_SEC:
        front = sensors.get_front_cm()
        if front is None:
            stop_chassis(chassis)
            time.sleep(POST_TURN_LOOP_SEC)
            continue
        if front <= POST_TURN_FRONT_STOP_CM:
            break

        _, inner = read_inner()
        ir_left, ir_right = sensors.read_front_corner_ir()
        inner_ir = ir_left if relative == "LEFT" else ir_right

        if (
            inner is not None
            and inner >= POST_TURN_RELEASE_CM
            and inner_ir is not True
        ):
            break

        moved = travelled_from(start, pose)
        if moved is not None and moved >= POST_TURN_MAX_DISTANCE_M:
            break

        x, y, z, _, _ = heading.apply(
            POST_TURN_FORWARD_SPEED,
            y_out,
            pose.get_yaw(),
            "POST_TURN_CLEARANCE",
        )
        chassis.drive_speed(x=x, y=y, z=z, timeout=DRIVE_TIMEOUT_SEC)
        time.sleep(POST_TURN_LOOP_SEC)

    stop_chassis(chassis)


def scan_junction(sensors: SensorManager, event: JunctionEvent):
    time.sleep(JUNCTION_SETTLE_SEC)
    front_values, left_values, right_values = [], [], []

    for i in range(DECISION_SCAN_SAMPLES):
        _, left = sensors.read_left()
        _, right = sensors.read_right()
        front = sensors.get_front_cm()
        if front is not None:
            front_values.append(front)
        if left is not None:
            left_values.append(left)
        if right is not None:
            right_values.append(right)
        if i + 1 < DECISION_SCAN_SAMPLES:
            time.sleep(DECISION_SCAN_INTERVAL_SEC)

    front = median_or_none(front_values)
    left = median_or_none(left_values)
    right = median_or_none(right_values)

    raw_front_open = front is not None and front >= EXPLORATION_FRONT_OPEN_CM
    raw_left_open = left is not None and left >= SIDE_OPEN_ENTER_CM
    raw_right_open = right is not None and right >= SIDE_OPEN_ENTER_CM

    front_open = raw_front_open or event.observed_front
    left_open = raw_left_open or event.observed_left
    right_open = raw_right_open or event.observed_right

    # A true close front reading always wins for safety.
    if front is not None and front <= STOP_FRONT_CM:
        front_open = False

    print(
        f">>> SCAN F={fmt(front)} {'OPEN' if front_open else 'BLOCK'} | "
        f"L={fmt(left)} {'OPEN' if left_open else 'BLOCK'} | "
        f"R={fmt(right)} {'OPEN' if right_open else 'BLOCK'}"
    )

    return {
        "front_cm": front,
        "left_cm": left,
        "right_cm": right,
        "front_open": front_open,
        "left_open": left_open,
        "right_open": right_open,
    }


def observed_absolute_openings(explorer: MazeGraphExplorer, scan) -> set:
    result = set()
    if scan["front_open"]:
        result.add(explorer.absolute_for_relative("FRONT"))
    if scan["left_open"]:
        result.add(explorer.absolute_for_relative("LEFT"))
    if scan["right_open"]:
        result.add(explorer.absolute_for_relative("RIGHT"))

    # BACK is physically open because the robot just arrived through it.
    if explorer.pending_abs is not None:
        result.add(explorer.opposite(explorer.pending_abs))
    elif explorer.current_node is not None:
        # At the synthetic start node there may not be a meaningful BACK yet.
        pass
    return result


from .configuration import DIR_DELTA, DIR_FROM_NAME, HybridConfig
from .grid_map import GridMazeMap
from .planning import *
from .reporting import export_run_artifacts

# Shared ToF + pickup/drop mission
# ============================================================

@dataclass(frozen=True)
class HybridToFEvent:
    sequence: int
    status: str
    distance_cm: Optional[float]
    raw_mm: Optional[float]
    callback_time: Optional[float]


@dataclass
class HybridToFObservation:
    valid_values_cm: List[float]
    status_counts: Dict[str, int]
    callback_count: int

    @property
    def median_cm(self) -> Optional[float]:
        return float(statistics.median(self.valid_values_cm)) if self.valid_values_cm else None

    @property
    def no_return_ratio(self) -> float:
        if self.callback_count <= 0:
            return 0.0
        return self.status_counts.get("NO_RETURN", 0) / self.callback_count


class HybridToFMonitor:
    """One ToF stream serves pickup verification and maze front safety."""

    def __init__(self, filter_size: int = 5, stale_sec: float = 0.60):
        self.filter_size = max(1, int(filter_size))
        self.stale_sec = float(stale_sec)
        self.paused = False
        self._lock = threading.RLock()
        self._sequence = 0
        self._status = "STARTING"
        self._distance_cm: Optional[float] = None
        self._raw_mm: Optional[float] = None
        self._callback_time: Optional[float] = None
        self._recent: deque = deque(maxlen=self.filter_size)

    def callback(self, data) -> None:
        if self.paused:
            return
        now = time.monotonic()
        status = "BAD_PACKET"
        raw_mm = None
        distance_cm = None
        try:
            if data is not None and len(data) >= 1 and data[0] is not None:
                raw_mm = float(data[0])
                if raw_mm <= 0.0 or raw_mm > 4000.0:
                    status = "NO_RETURN"
                elif raw_mm < 20.0:
                    status = "TOO_CLOSE"
                else:
                    status = "VALID"
                    distance_cm = raw_mm / 10.0
        except (TypeError, ValueError, IndexError):
            status = "BAD_PACKET"

        with self._lock:
            self._sequence += 1
            self._status = status
            self._raw_mm = raw_mm
            self._distance_cm = distance_cm
            self._callback_time = now
            if distance_cm is not None:
                self._recent.append((now, distance_cm))

    def latest(self) -> HybridToFEvent:
        with self._lock:
            return HybridToFEvent(
                self._sequence,
                self._status,
                self._distance_cm,
                self._raw_mm,
                self._callback_time,
            )

    def filtered(self) -> Tuple[Optional[float], str, Optional[float]]:
        now = time.monotonic()
        with self._lock:
            if self._callback_time is None:
                return None, "STARTING", self._raw_mm
            if now - self._callback_time > self.stale_sec:
                return None, "STALE", self._raw_mm
            if self._status != "VALID":
                return None, self._status, self._raw_mm
            values = [value for timestamp, value in self._recent if now - timestamp <= self.stale_sec]
            if not values:
                return None, "STALE", self._raw_mm
            return float(statistics.median(values)), "VALID", self._raw_mm

    def clear(self) -> None:
        with self._lock:
            self._status = "ARM_SETTLING"
            self._distance_cm = None
            self._raw_mm = None
            self._callback_time = None
            self._recent.clear()

    def wait_first(self, timeout_sec: float, stop_event: threading.Event) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not stop_event.is_set():
            if self.latest().sequence > 0:
                return True
            time.sleep(0.02)
        return False

    def wait_beam_clear(self, timeout_sec: float, stop_event: threading.Event) -> bool:
        """Require several live callbacks that are not arm-occlusion readings."""
        deadline = time.monotonic() + timeout_sec
        last_sequence = self.latest().sequence
        consecutive = 0
        while time.monotonic() < deadline and not stop_event.is_set():
            event = self.latest()
            if event.sequence != last_sequence:
                last_sequence = event.sequence
                if event.status in ("VALID", "NO_RETURN"):
                    consecutive += 1
                    if consecutive >= 3:
                        return True
                else:
                    consecutive = 0
            time.sleep(0.02)
        return False

    def collect(self, duration_sec: float, stop_event: threading.Event) -> HybridToFObservation:
        deadline = time.monotonic() + duration_sec
        last_sequence = self.latest().sequence
        valid: List[float] = []
        statuses: Counter = Counter()
        callbacks = 0
        while time.monotonic() < deadline and not stop_event.is_set():
            event = self.latest()
            if event.sequence != last_sequence:
                last_sequence = event.sequence
                callbacks += 1
                statuses[event.status] += 1
                if event.status == "VALID" and event.distance_cm is not None:
                    valid.append(event.distance_cm)
            time.sleep(0.01)
        return HybridToFObservation(valid, dict(statuses), callbacks)


def wait_sdk_action(action, timeout_sec: float, stop_event: threading.Event, label: str) -> None:
    deadline = time.monotonic() + max(0.2, timeout_sec)
    while not stop_event.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(f"{label} timed out after {timeout_sec:.1f}s")
        if action.wait_for_completed(timeout=min(0.10, remaining)):
            return
    raise RuntimeError(f"{label} stopped by user")


def hybrid_move_arm(
    arm,
    x: int,
    y: int,
    config: HybridConfig,
    stop_event: threading.Event,
    label: str,
) -> None:
    print(f"[ARM] {label}: x={x}, y={y}")
    action = arm.moveto(x=int(x), y=int(y))
    wait_sdk_action(action, config.arm_action_timeout_sec, stop_event, label)
    time.sleep(config.arm_settle_sec)


def hybrid_open_gripper(gripper, config: HybridConfig, label: str) -> None:
    print(f"[GRIPPER] {label}: OPEN power={config.gripper_power}%")
    if gripper.open(power=int(config.gripper_power)) is False:
        raise RuntimeError("gripper.open() returned False")
    time.sleep(config.gripper_open_sec)
    gripper.pause()


def hybrid_close_gripper(gripper, config: HybridConfig, label: str, hold: bool = True) -> None:
    print(f"[GRIPPER] {label}: CLOSE power={config.gripper_power}%")
    if gripper.close(power=int(config.gripper_power)) is False:
        raise RuntimeError("gripper.close() returned False")
    time.sleep(config.gripper_close_sec)
    if not hold:
        gripper.pause()


def hybrid_hard_stop(chassis) -> None:
    if chassis is None:
        return
    try:
        chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    except Exception:
        try:
            chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.1)
        except Exception:
            pass


def pickup_speed(config: HybridConfig, distance_cm: float) -> float:
    if distance_cm > 18.0:
        return config.pickup_fast_speed_mps
    if distance_cm > max(config.pickup_target_cm + 2.0, 10.0):
        return config.pickup_slow_speed_mps
    return config.pickup_crawl_speed_mps


def approach_pickup_object(
    chassis,
    pose: PoseTracker,
    tof: HybridToFMonitor,
    config: HybridConfig,
    stop_event: threading.Event,
    status: Callable[[str], None],
) -> Optional[float]:
    status("PICKUP: waiting for object")
    started = None
    lost_since = None
    target_yaw = None
    min_seen = float("inf")
    last_print = 0.0

    while not stop_event.is_set():
        now = time.monotonic()
        distance_cm, state, raw_mm = tof.filtered()
        if state != "VALID" or distance_cm is None:
            hybrid_hard_stop(chassis)
            if started is not None:
                lost_since = lost_since or now
                if now - lost_since >= config.tof_lost_abort_sec:
                    print("[PICKUP] ToF lost while approaching")
                    return None
            if now - last_print >= 0.4:
                print(f"[WAIT OBJECT] status={state} raw_mm={raw_mm}")
                last_print = now
            time.sleep(config.control_period_sec)
            continue

        lost_since = None
        if distance_cm > config.object_detect_max_cm:
            hybrid_hard_stop(chassis)
            time.sleep(config.control_period_sec)
            continue

        if started is None:
            started = now
            target_yaw = pose.get_yaw()
            min_seen = distance_cm
            status(f"PICKUP: object {distance_cm:.1f} cm")
        else:
            min_seen = min(min_seen, distance_cm)
            if distance_cm > min_seen + 10.0:
                hybrid_hard_stop(chassis)
                print("[PICKUP] ToF beam slipped away from object")
                return None
            if now - started > config.pickup_timeout_sec:
                hybrid_hard_stop(chassis)
                return None

        if distance_cm <= config.pickup_target_cm:
            hybrid_hard_stop(chassis)
            sample = tof.collect(0.40, stop_event)
            if (
                sample.median_cm is not None
                and len(sample.valid_values_cm) >= 3
                and sample.median_cm <= config.pickup_target_cm + config.pickup_tolerance_cm
            ):
                return sample.median_cm
            time.sleep(config.control_period_sec)
            continue

        speed = pickup_speed(config, distance_cm)
        z_cmd = 0.0
        current_yaw = pose.get_yaw()
        if target_yaw is not None and current_yaw is not None:
            error = shortest_angle_error_deg(target_yaw, current_yaw)
            if abs(error) > 2.0:
                z_cmd = clamp(error, -8.0, 8.0) / DEFAULT_DRIVE_TO_YAW_SIGN
        chassis.drive_speed(x=speed, y=0.0, z=z_cmd, timeout=config.drive_timeout_sec)
        if now - last_print >= 0.25:
            print(f"[APPROACH] ToF={distance_cm:.1f} cm x={speed:.3f} z={z_cmd:+.1f}")
            last_print = now
        time.sleep(config.control_period_sec)

    hybrid_hard_stop(chassis)
    return None


def pickup_and_verify(
    chassis,
    arm,
    gripper,
    pose: PoseTracker,
    tof: HybridToFMonitor,
    config: HybridConfig,
    stop_event: threading.Event,
    status: Callable[[str], None],
) -> bool:
    """Stable pickup sequence adapted from the supplied pickup V3 file."""
    maximum = max(1, int(config.pickup_attempts))
    for attempt in range(1, maximum + 1):
        if stop_event.is_set():
            return False
        status(f"PICKUP attempt {attempt}/{maximum}")
        hybrid_hard_stop(chassis)

        # Carry position keeps the arm outside the ToF beam while detecting.
        hybrid_move_arm(arm, config.carry_x, config.carry_y, config, stop_event, "RESET/CARRY")
        hybrid_close_gripper(gripper, config, "RESET CLOSE", hold=False)
        tof.clear()
        if not tof.wait_beam_clear(3.0, stop_event):
            print("[PICKUP] ToF remains blocked after arm moved to CARRY")
            return False

        reached = approach_pickup_object(chassis, pose, tof, config, stop_event, status)
        if reached is None:
            continue

        hybrid_open_gripper(gripper, config, "OPEN FOR OBJECT")
        tof.paused = True
        hybrid_move_arm(arm, config.pickup_x, config.pickup_y, config, stop_event, "LOWER TO PICK")
        hybrid_close_gripper(gripper, config, "GRAB", hold=True)
        hybrid_move_arm(arm, config.carry_x, config.carry_y, config, stop_event, "LIFT/CARRY")
        tof.paused = False
        tof.clear()
        observation = tof.collect(config.verify_window_sec, stop_event)
        clear_by_distance = (
            observation.median_cm is not None
            and observation.median_cm >= reached + config.lift_clear_delta_cm
            and len(observation.valid_values_cm) >= 3
        )
        clear_by_no_return = observation.callback_count >= 4 and observation.no_return_ratio >= 0.60
        if clear_by_distance or clear_by_no_return:
            status("PICKUP SUCCESS: carrying object")
            return True

        print(
            "[PICKUP VERIFY FAILED] "
            f"before={reached:.1f} after={observation.median_cm} states={observation.status_counts}"
        )
        hybrid_open_gripper(gripper, config, "RECOVERY RELEASE")
        if config.retry_backoff_cm > 0.0:
            duration = (config.retry_backoff_cm / 100.0) / 0.05
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline and not stop_event.is_set():
                chassis.drive_speed(x=-0.05, y=0.0, z=0.0, timeout=config.drive_timeout_sec)
                time.sleep(config.control_period_sec)
            hybrid_hard_stop(chassis)
    return False


def place_object(
    arm,
    gripper,
    tof: HybridToFMonitor,
    config: HybridConfig,
    stop_event: threading.Event,
    status: Callable[[str], None],
) -> None:
    status("DROP: lowering object")
    tof.paused = True
    hybrid_move_arm(arm, config.drop_x, config.drop_y, config, stop_event, "LOWER TO DROP")
    hybrid_open_gripper(gripper, config, "RELEASE OBJECT")
    hybrid_move_arm(arm, config.carry_x, config.carry_y, config, stop_event, "RETURN TO CARRY")
    tof.paused = False
    tof.clear()
    status("DROP COMPLETE")


def align_drop_position(
    navigator,
    sensors: SensorManager,
    pose: PoseTracker,
    heading: HeadingManager,
    config: HybridConfig,
    stop_event: threading.Event,
    status: Callable[[str], None],
) -> bool:
    """Align front ToF and one side Sharp to the configured drop distances."""
    target_heading = DIR_FROM_NAME[config.drop_heading]
    status(
        f"DROP ALIGN: face {config.drop_heading}, use {config.drop_side} wall, "
        f"object goal={config.drop_object_front_wall_cm:.1f}/"
        f"{config.drop_object_side_wall_cm:.1f}cm, sensor targets "
        f"F={config.drop_front_sensor_target_cm:.1f}cm "
        f"S={config.drop_side_sensor_target_cm:.1f}cm"
    )
    if not navigator.turn_to(target_heading):
        status("DROP ALIGN FAILED: could not turn to drop heading")
        return False

    hybrid_hard_stop(navigator.chassis)
    sensors.reset_sharp_filters()
    time.sleep(JUNCTION_SETTLE_SEC)
    side_reader = sensors.read_left if config.drop_side == "LEFT" else sensors.read_right
    started = time.monotonic()
    start_xy = pose.get_xy()
    stable = 0
    missing_since = None
    last_print = 0.0

    while not stop_event.is_set():
        now = time.monotonic()
        if now - started >= config.drop_align_timeout_sec:
            status("DROP ALIGN FAILED: timeout")
            break

        moved = travelled_from(start_xy, pose)
        if (
            moved is not None
            and moved >= config.drop_align_max_travel_cm / 100.0
        ):
            status(
                f"DROP ALIGN FAILED: travelled {moved * 100.0:.0f}cm "
                "without finding the 40cm target"
            )
            break

        front_cm = sensors.get_front_cm()
        raw_side, side_cm = side_reader()
        if front_cm is None or side_cm is None:
            hybrid_hard_stop(navigator.chassis)
            missing_since = missing_since or now
            if now - missing_since >= 1.2:
                status("DROP ALIGN FAILED: ToF or side Sharp unavailable")
                break
            time.sleep(config.control_period_sec)
            continue
        missing_since = None

        front_error = front_cm - config.drop_front_sensor_target_cm
        side_error = side_cm - config.drop_side_sensor_target_cm
        within_front = abs(front_error) <= config.drop_distance_tolerance_cm
        within_side = abs(side_error) <= config.drop_distance_tolerance_cm
        stable = stable + 1 if within_front and within_side else 0
        if stable >= int(config.drop_stable_samples):
            hybrid_hard_stop(navigator.chassis)
            status(
                f"DROP POSITION READY: front={front_cm:.1f}cm, "
                f"{config.drop_side.lower()}={side_cm:.1f}cm"
            )
            return True

        max_speed = config.drop_align_max_speed_mps
        kp = config.drop_align_kp_mps_per_cm
        x_cmd = 0.0 if within_front else clamp(front_error * kp, -max_speed, max_speed)
        lateral = 0.0 if within_side else clamp(side_error * kp, -max_speed, max_speed)
        # Existing chassis convention: +Y moves away from the LEFT wall.
        y_cmd = (
            -lateral * Y_DIR_SIGN
            if config.drop_side == "LEFT"
            else +lateral * Y_DIR_SIGN
        )

        # Never command farther toward a dangerously close wall.
        if front_cm <= config.hard_stop_front_cm and x_cmd > 0.0:
            x_cmd = 0.0
        if side_cm <= SIDE_TOO_CLOSE_CM:
            if config.drop_side == "LEFT" and y_cmd < 0.0:
                y_cmd = 0.0
            if config.drop_side == "RIGHT" and y_cmd > 0.0:
                y_cmd = 0.0

        ir_left, ir_right = sensors.read_front_corner_ir()
        x_cmd, y_cmd, _ = apply_front_corner_ir_guard(
            x_cmd, y_cmd, ir_left, ir_right
        )
        x_cmd, y_cmd, z_cmd, _, _ = heading.apply(
            x_cmd,
            y_cmd,
            pose.get_yaw(),
            "DROP_ALIGN",
        )
        navigator.chassis.drive_speed(
            x=x_cmd,
            y=y_cmd,
            z=z_cmd,
            timeout=config.drive_timeout_sec,
        )
        if now - last_print >= 0.25:
            print(
                f"[DROP ALIGN] F={front_cm:.1f}/{config.drop_front_sensor_target_cm:.1f} "
                f"{config.drop_side[0]}={side_cm:.1f}/{config.drop_side_sensor_target_cm:.1f} "
                f"ADC={raw_side} stable={stable}/{config.drop_stable_samples} "
                f"x={x_cmd:+.3f} y={y_cmd:+.3f}"
            )
            last_print = now
        time.sleep(config.control_period_sec)

    hybrid_hard_stop(navigator.chassis)
    return False


# ============================================================
# Hybrid robot navigator
# ============================================================

def apply_runtime_config(config: HybridConfig) -> None:
    """Feed GUI values into the proven low-level controllers kept above."""
    global FORWARD_SPEED, MIN_FORWARD_SPEED, SIDE_MAX_Y, STOP_FRONT_CM
    global EXPLORATION_FRONT_OPEN_CM, SIDE_WALL_ENTER_CM, DRIVE_TIMEOUT_SEC
    global SIDE_OPEN_ENTER_CM, SIDE_OPEN_EXIT_CM
    global LOOP_DELAY_SEC
    FORWARD_SPEED = config.forward_speed_mps
    MIN_FORWARD_SPEED = config.minimum_speed_mps
    SIDE_MAX_Y = config.lateral_max_mps
    STOP_FRONT_CM = config.hard_stop_front_cm
    EXPLORATION_FRONT_OPEN_CM = config.front_wall_cm
    SIDE_WALL_ENTER_CM = config.side_wall_cm
    # Match the proven robomaster_maze behaviour: topology/opening detection
    # uses ~18 cm, independently from the wall-follow controller threshold.
    SIDE_OPEN_ENTER_CM = config.side_topology_open_cm
    SIDE_OPEN_EXIT_CM = max(5.0, config.side_topology_open_cm - 3.0)
    DRIVE_TIMEOUT_SEC = config.drive_timeout_sec
    LOOP_DELAY_SEC = config.control_period_sec
    # Always reset before loading so repeated GUI runs cannot retain a table
    # selected by a previous mission.
    global CALIBRATION_SHARP_LEFT, CALIBRATION_SHARP_RIGHT
    CALIBRATION_SHARP_LEFT = list(_DEFAULT_CALIBRATION_SHARP_LEFT)
    CALIBRATION_SHARP_RIGHT = list(_DEFAULT_CALIBRATION_SHARP_RIGHT)
    _SHARP_CALIBRATION_LOADED_FROM["LEFT"] = None
    _SHARP_CALIBRATION_LOADED_FROM["RIGHT"] = None
    if str(config.sharp_left_calibration_file).strip():
        load_sharp_calibration_file(config.sharp_left_calibration_file, "LEFT")
    if str(config.sharp_right_calibration_file).strip():
        load_sharp_calibration_file(config.sharp_right_calibration_file, "RIGHT")


def sdk_connection_type(value: str):
    """Return the SDK's own constant object for AP/STA/RNDIS.

    RoboMaster SDK 0.1.1 compares conn_type with ``is`` instead of ``==`` in
    SdkConnection.request_connection().  A string read from Tkinter therefore
    may skip every branch and leave ``proxy_addr`` unassigned.  Returning the
    exact constant object avoids that upstream SDK bug.
    """
    if robomaster_conn is None:
        raise RuntimeError("RoboMaster SDK is not installed")
    normalized = str(value).strip().lower()
    choices = {
        "ap": robomaster_conn.CONNECTION_WIFI_AP,
        "sta": robomaster_conn.CONNECTION_WIFI_STA,
        "rndis": robomaster_conn.CONNECTION_USB_RNDIS,
    }
    if normalized not in choices:
        raise ValueError("Connection must be ap, sta, or rndis")
    return choices[normalized]


class HybridNavigator:
    def __init__(
        self,
        chassis,
        sensors: SensorManager,
        pose: PoseTracker,
        heading: HeadingManager,
        maze: GridMazeMap,
        config: HybridConfig,
        events: queue.Queue,
        stop_event: threading.Event,
    ):
        self.chassis = chassis
        self.sensors = sensors
        self.pose = pose
        self.heading = heading
        self.maze = maze
        self.config = config
        self.events = events
        self.stop_event = stop_event
        self.cell = maze.start
        self.heading_index = DIR_FROM_NAME[config.start_heading]
        self.walls = WallController()
        self.edge_traversals: Counter = Counter()
        self.visited: Counter = Counter()
        # DROP alignment intentionally moves away from the cell centre.  The
        # first departure must follow the fixed map without converting those
        # off-centre side readings into cell-edge topology.
        self.manual_drop_departure_pending = False
        # ``cell_anchor_xy`` is the odometry position at which the logical
        # grid cell was committed. DROP ALIGN may move away from this anchor;
        # the next centre-to-centre distance must compensate for that offset.
        self.cell_anchor_xy = pose.get_xy()
        self.last_arrival_previous: Optional[Tuple[int, int]] = None
        self.last_arrival_direction: Optional[int] = None
        self.manual_drop_return_cell: Optional[Tuple[int, int]] = None
        self.manual_drop_return_direction: Optional[int] = None
        if self.cell is not None:
            self.visited[self.cell] += 1

    def status(self, message: str) -> None:
        self.maze.status = message
        print(f"[MISSION] {message}")
        self.events.put(("status", message))

    def publish_pose(self, fraction: float = 0.0, direction: Optional[int] = None) -> None:
        if self.cell is None:
            return
        self.maze.record_pose(self.cell, self.heading_index)
        payload = {
            "cell": self.cell,
            "heading": self.heading_index,
            "fraction": float(fraction),
            "direction": self.heading_index if direction is None else direction,
        }
        self.events.put(("pose", payload))

    def sense_current_cell(self, samples: int = 5) -> Dict[int, Optional[bool]]:
        """Return absolute direction -> blocked; None means no reliable sample."""
        if self.cell is None:
            return {}
        # A turn changes what LEFT/RIGHT physically point at.  Keeping the old
        # three-sample median and EMA can paint the previous wall onto the new
        # absolute direction.  Rebuild only the Sharp filters here; do not
        # clear ToF because its callback state is independent of orientation.
        self.sensors.reset_sharp_filters()
        time.sleep(0.06)
        fronts, lefts, rights = [], [], []
        ir_left_values, ir_right_values = [], []
        for _ in range(max(1, samples)):
            front = self.sensors.get_front_cm()
            _, left = self.sensors.read_left()
            _, right = self.sensors.read_right()
            ir_left, ir_right = self.sensors.read_front_corner_ir()
            if front is not None:
                fronts.append(front)
            if left is not None:
                lefts.append(left)
            if right is not None:
                rights.append(right)
            if ir_left is not None:
                ir_left_values.append(ir_left)
            if ir_right is not None:
                ir_right_values.append(ir_right)
            time.sleep(0.025)

        front = median_or_none(fronts)
        left = median_or_none(lefts)
        right = median_or_none(rights)
        self.maze.record_sensor_snapshot(
            self.cell,
            self.heading_index,
            front,
            left,
            right,
        )
        relative = {
            0: None if front is None else front <= self.config.front_wall_cm,
            # This intentionally matches robomaster_maze: a side at or beyond
            # SIDE_OPEN_ENTER_CM (~18 cm) is an opening.  side_wall_cm (~28 cm)
            # belongs to motion control and must not be used to paint topology.
            -1: None if left is None else left < self.config.side_topology_open_cm,
            1: None if right is None else right < self.config.side_topology_open_cm,
        }
        if any(ir_left_values) or any(ir_right_values):
            relative[0] = True

        absolute: Dict[int, Optional[bool]] = {}
        for offset, blocked in relative.items():
            direction = (self.heading_index + offset) % 4
            if (
                offset != 0
                and blocked is True
                and self.maze.is_traversed_open(self.cell, direction)
            ):
                print(
                    f"[MAP GUARD] ignore side-Sharp wall at {self.cell} "
                    f"toward {HEADINGS[direction]}: chassis already crossed edge"
                )
                blocked = False
            absolute[direction] = blocked
            if blocked is not None:
                # A close FRONT reading is safety-critical and may represent a
                # newly placed obstacle. Side Sharp cannot close an edge that
                # has already been physically traversed.
                self.maze.observe_edge(
                    self.cell,
                    direction,
                    blocked,
                    force=(offset == 0 and blocked is True),
                )
        self.events.put(("map", None))
        print(
            f"[SENSE] cell={self.cell} H={HEADINGS[self.heading_index]} "
            f"F={fmt(front)} L={fmt(left)} R={fmt(right)} "
            f"blocked(F/L/R)={relative[0]}/{relative[-1]}/{relative[1]} "
            f"side_open_at={self.config.side_topology_open_cm:.1f}cm"
        )
        return absolute

    def turn_to(self, target_heading: int) -> bool:
        target_heading %= 4
        if target_heading == self.heading_index:
            return True
        relative = relative_turn(self.heading_index, target_heading)
        self.status(f"TURN {relative} -> {HEADINGS[target_heading]}")
        if not feedback_turn(self.chassis, self.sensors, self.pose, relative):
            return False
        self.heading_index = target_heading
        self.heading.set_heading_index(target_heading)
        align_heading(self.chassis, self.pose, self.heading)
        # feedback_turn reads Sharp continuously.  Those samples describe
        # changing directions during rotation and are invalid for map labels.
        self.sensors.reset_sharp_filters()
        self.walls.reset()
        self.publish_pose()
        return True

    def _manual_end_wall_expected(self, target_cell, travel_direction):
        next_cell = self.maze.neighbour(target_cell, travel_direction)

        if not self.maze.in_bounds(next_cell):
            return True

        edge = (
            target_cell[0],
            target_cell[1],
            travel_direction % 4,
        )
        return edge in self.maze.manual_walls

    def _commit_cell_arrival(
        self,
        previous: Tuple[int, int],
        next_cell: Tuple[int, int],
        direction: int,
        reason: str,
    ) -> bool:
        """Atomically advance the logical cell and protect the crossed edge."""
        self.cell = next_cell
        self.last_arrival_previous = previous
        self.last_arrival_direction = direction % 4
        self.cell_anchor_xy = self.pose.get_xy()
        self.maze.confirm_traversed_open_edge(previous, direction)
        edge_key = tuple(sorted((previous, next_cell)))
        self.edge_traversals[edge_key] += 1
        self.visited[next_cell] += 1
        self.status(f"ARRIVED {next_cell} ({reason})")
        self.publish_pose()
        return True

    def begin_manual_drop_departure(self) -> None:
        self.manual_drop_departure_pending = True
        self.manual_drop_return_cell = None
        self.manual_drop_return_direction = None
        if (
            self.last_arrival_previous is not None
            and self.last_arrival_direction is not None
            and self.cell is not None
            and self.maze.neighbour(
                self.cell, (self.last_arrival_direction + 2) % 4
            ) == self.last_arrival_previous
        ):
            # Leave DROP through the edge that was physically crossed on the
            # way in. This edge has stronger evidence than any off-centre
            # Sharp/ToF snapshot taken after placing the object.
            self.manual_drop_return_cell = self.last_arrival_previous
            self.manual_drop_return_direction = (
                self.last_arrival_direction + 2
            ) % 4
            self.status(
                "DROP DEPARTURE ARMED: return first to "
                f"{self.manual_drop_return_cell} toward "
                f"{HEADINGS[self.manual_drop_return_direction]}"
            )
        else:
            self.status(
                "DROP DEPARTURE ARMED: fixed map is authoritative until next cell"
            )

    def _drop_departure_target_distance(
        self,
        start_xy: Tuple[Optional[float], Optional[float]],
        nominal_m: float,
    ) -> float:
        """Distance from the post-alignment pose to the next logical anchor."""
        anchor = self.cell_anchor_xy
        yaw = self.heading.target_yaw
        if None in (anchor[0], anchor[1], start_xy[0], start_xy[1], yaw):
            return nominal_m
        radians = math.radians(float(yaw))
        target_x = float(anchor[0]) + nominal_m * math.cos(radians)
        target_y = float(anchor[1]) + nominal_m * math.sin(radians)
        remaining = math.hypot(
            target_x - float(start_xy[0]),
            target_y - float(start_xy[1]),
        )
        # Do not accept an accidental near-zero odometry jump, and bound a
        # bad pose so DROP recovery cannot command an unlimited move.
        return clamp(remaining, 0.08, nominal_m + 0.40)

    def drive_one_cell(
        self,
        direction: int,
        manual_departure: bool = False,
    ) -> bool:
        if self.cell is None:
            return False
        if direction != self.heading_index and not self.turn_to(direction):
            return False
        # Re-scan after turning: the requested edge may previously have been
        # behind the robot and therefore invisible to front/side sensors.
        if manual_departure:
            # The robot is deliberately off-centre after placing the object.
            # A 45 cm ToF/18 cm Sharp topology threshold is not meaningful at
            # this pose. Keep hard-stop protection below, but do not label an
            # edge or reject the operator-drawn route from this snapshot.
            self.sensors.reset_sharp_filters()
            time.sleep(0.06)
            turned_snapshot: Dict[int, Optional[bool]] = {}
        else:
            turned_snapshot = self.sense_current_cell(samples=3)
            if turned_snapshot.get(direction) is True:
                self.status(f"WALL AFTER TURN {HEADINGS[direction]}: replanning")
                return False
        start_xy = self.pose.get_xy()
        if None in start_xy:
            self.status("ODOMETRY LOST: cannot move a grid cell")
            return False

        previous = self.cell
        next_cell = self.maze.neighbour(previous, direction)
        if not self.maze.in_bounds(next_cell):
            self.status(
                f"MOVE REJECTED: {HEADINGS[direction]} from {previous} leaves map"
            )
            return False

        nominal_target_m = self.config.cell_size_cm / 100.0
        target_m = nominal_target_m
        if manual_departure:
            target_m = self._drop_departure_target_distance(
                start_xy,
                nominal_target_m,
            )
            self.status(
                f"DROP OFFSET COMPENSATION: move {target_m:.2f}m "
                f"instead of {nominal_target_m:.2f}m"
            )
        tolerance_m = self.config.cell_tolerance_cm / 100.0
        departure_front_cm = self.sensors.get_front_cm()
        self.status(f"MOVE {HEADINGS[direction]} one cell")
        last_print = 0.0
        while not self.stop_event.is_set():
            travelled = travelled_from(start_xy, self.pose)
            if travelled is None:
                hybrid_hard_stop(self.chassis)
                return False
            if travelled >= max(0.01, target_m - tolerance_m):
                hybrid_hard_stop(self.chassis)
                return self._commit_cell_arrival(
                    previous,
                    next_cell,
                    direction,
                    f"odometry {travelled:.2f}/{target_m:.2f} m",
                )

            front_cm = self.sensors.get_front_cm()
            if front_cm is not None and front_cm <= self.config.hard_stop_front_cm:
                hybrid_hard_stop(self.chassis)
                minimum_end_wall_progress = (
                    target_m * self.config.end_wall_arrival_min_ratio
                )
                if (
                    manual_departure
                    and departure_front_cm is not None
                    and self._manual_end_wall_expected(next_cell, direction)
                ):
                    # DROP may leave the chassis closer to the next cell's far
                    # wall than a nominal centre-to-centre 60 cm move. Estimate
                    # the available run from the initial ToF and require most
                    # of that run instead of blindly requiring 70% of a cell.
                    visible_run_m = max(
                        0.0,
                        (departure_front_cm - self.config.hard_stop_front_cm)
                        / 100.0,
                    )
                    minimum_end_wall_progress = min(
                        minimum_end_wall_progress,
                        max(0.08, visible_run_m * 0.70),
                    )
                if (
                    travelled >= minimum_end_wall_progress
                    and self._manual_end_wall_expected(next_cell, direction)
                ):
                    # The chassis is already well inside next_cell and ToF is
                    # seeing the wall drawn at that cell's far edge.  Mark the
                    # far edge, not the edge just crossed; then commit the new
                    # logical cell so the GUI cannot snap back to previous.
                    self.maze.observe_edge(next_cell, direction, True)
                    self.events.put(("map", None))
                    return self._commit_cell_arrival(
                        previous,
                        next_cell,
                        direction,
                        (
                            f"expected end wall {front_cm:.1f} cm, "
                            f"progress {travelled:.2f}/{target_m:.2f} m"
                        ),
                    )

                if manual_departure:
                    # This is the already-traversed entrance edge. The chassis
                    # is off-centre after DROP, so a close reading cannot prove
                    # that a new wall exists between the two cells.
                    self.maze.confirm_traversed_open_edge(previous, direction)
                    self.events.put(("map", None))
                    self.manual_drop_departure_pending = True
                    self.status(
                        f"DROP DEPARTURE BLOCKED at {front_cm:.1f} cm after "
                        f"{travelled:.2f}/{target_m:.2f} m; edge kept OPEN"
                    )
                    return False
                self.maze.observe_edge(previous, direction, True, force=True)
                self.events.put(("map", None))
                self.status(
                    f"UNEXPECTED WALL {front_cm:.1f} cm after "
                    f"{travelled:.2f}/{target_m:.2f} m: replan from {previous}"
                )
                return False

            _, left_cm = self.sensors.read_left()
            _, right_cm = self.sensors.read_right()
            ir_left, ir_right = self.sensors.read_front_corner_ir()
            remaining = max(0.0, target_m - travelled)
            x_cmd = min(
                self.config.forward_speed_mps,
                max(self.config.minimum_speed_mps, remaining * 0.8),
            )
            y_cmd, mode = self.walls.lateral(left_cm, right_cm)
            x_cmd, y_cmd, ir_mode = apply_front_corner_ir_guard(
                x_cmd, y_cmd, ir_left, ir_right
            )
            if ir_mode:
                mode = ir_mode
            x_cmd, y_cmd, z_cmd, mode, _ = self.heading.apply(
                x_cmd, y_cmd, self.pose.get_yaw(), mode
            )
            self.chassis.drive_speed(
                x=x_cmd,
                y=y_cmd,
                z=z_cmd,
                timeout=self.config.drive_timeout_sec,
            )
            self.publish_pose(min(1.0, travelled / target_m), direction)
            now = time.monotonic()
            if now - last_print >= 0.30:
                print(
                    f"[CELL] {travelled:.3f}/{target_m:.3f}m "
                    f"front={fmt(front_cm)} mode={mode} x={x_cmd:.3f} y={y_cmd:+.3f}"
                )
                last_print = now
            time.sleep(self.config.control_period_sec)

        hybrid_hard_stop(self.chassis)
        return False

    def navigate_to(self, goal: Tuple[int, int], label: str) -> bool:
        if self.cell is None:
            return False
        replans = 0
        while self.cell != goal and not self.stop_event.is_set():
            manual_departure = self.manual_drop_departure_pending
            if manual_departure:
                self.status(
                    f"DROP DEPARTURE: plan from fixed map at {self.cell}"
                )
            else:
                self.sense_current_cell()
            planning_maze = self.maze
            if manual_departure:
                # Build a clean planning view so stale red observations from
                # before DROP cannot close the only valid departure edge.
                planning_maze = GridMazeMap(self.maze.rows, self.maze.cols)
                planning_maze.manual_walls = set(self.maze.manual_walls)
            if (
                manual_departure
                and self.manual_drop_return_cell is not None
                and self.manual_drop_return_direction is not None
                and self.maze.neighbour(
                    self.cell, self.manual_drop_return_direction
                ) == self.manual_drop_return_cell
            ):
                # The known-safe first step takes priority over a shorter A*
                # turn from an off-centre DROP pose.
                route = [self.cell, self.manual_drop_return_cell]
            else:
                route = astar_oriented(
                    planning_maze,
                    self.cell,
                    self.heading_index,
                    goal,
                    self.config.turn_cost,
                    False if manual_departure else self.config.sensor_overrides_drawing,
                )
            if route is None or len(route) < 2:
                self.status(f"A* has no route to {label}; switching to exploration")
                return False
            self.maze.planned_path = route
            self.events.put(("map", None))
            direction = direction_between(route[0], route[1])
            edge_is_blocked = (
                (self.cell[0], self.cell[1], direction) in self.maze.manual_walls
                if manual_departure
                else self.maze.has_wall(
                    self.cell,
                    direction,
                    self.config.sensor_overrides_drawing,
                )
            )
            if edge_is_blocked:
                replans += 1
                continue
            if not self.drive_one_cell(direction, manual_departure):
                replans += 1
                if manual_departure:
                    # Never fall through to a normal scan at the same
                    # off-centre pose; that was the source of the false red
                    # wall between DROP and its entrance cell.
                    self.status(
                        "DROP DEPARTURE could not reach the previous cell safely"
                    )
                    return False
                if replans > self.config.max_replans:
                    return False
                continue
            if manual_departure:
                self.manual_drop_departure_pending = False
                self.manual_drop_return_cell = None
                self.manual_drop_return_direction = None
            replans = 0
        return self.cell == goal

    def explore_to_exit(
        self,
        exit_cell: Tuple[int, int],
        on_drop: Optional[Callable[[], None]] = None,
    ) -> bool:
        """Online Trémaux fallback when prior-map A* cannot be trusted."""
        self.status("FALLBACK: online Trémaux exploration")
        dropped = on_drop is None
        for _ in range(self.config.explore_max_steps):
            if self.stop_event.is_set() or self.cell is None:
                return False
            if not dropped and self.cell == self.maze.drop:
                on_drop()
                dropped = True
                if self.manual_drop_departure_pending:
                    # Do not take a Trémaux sensor snapshot while DROP ALIGN
                    # has left the chassis away from the logical cell anchor.
                    # First return through the physically traversed entrance.
                    return self.navigate_to(exit_cell, "EXIT")

            sensed = self.sense_current_cell()
            # When the prior map is absent or wrong, an open edge on the map
            # boundary is a physical exit candidate.  Remember it, continue to
            # DROP first if necessary, then return here and leave the maze.
            for direction, blocked in sensed.items():
                if blocked is not False:
                    continue
                if self.maze.in_bounds(self.maze.neighbour(self.cell, direction)):
                    continue
                self.maze.exit = self.cell
                self.config.exit_heading = HEADINGS[direction]
                exit_cell = self.cell
                self.status(
                    f"PHYSICAL EXIT DISCOVERED at {self.cell} toward {HEADINGS[direction]}"
                )
                self.events.put(("map", None))
                if dropped:
                    return True

            if dropped and self.cell == exit_cell:
                return True

            candidates = []
            for direction in range(4):
                neighbour = self.maze.neighbour(self.cell, direction)
                if not self.maze.in_bounds(neighbour):
                    continue
                if sensed.get(direction) is True:
                    continue
                edge_state = (self.cell[0], self.cell[1], direction)
                if (
                    edge_state in self.maze.observed_edges
                    and edge_state in self.maze.sensor_walls
                ):
                    continue
                edge = tuple(sorted((self.cell, neighbour)))
                traversals = self.edge_traversals[edge]
                if traversals >= self.config.fallback_edge_limit:
                    continue
                score = (
                    0 if self.visited[neighbour] == 0 else 1,
                    traversals,
                    turn_distance(self.heading_index, direction),
                )
                candidates.append((score, direction))
            if not candidates:
                self.status("FALLBACK exhausted all reachable edges")
                return False
            candidates.sort()
            if not self.drive_one_cell(candidates[0][1]):
                continue
        self.status("FALLBACK step limit reached")
        return False

    def drive_outside(self) -> bool:
        if self.cell is None:
            return False
        direction = DIR_FROM_NAME[self.config.exit_heading]
        outside = self.maze.neighbour(self.cell, direction)
        if self.maze.in_bounds(outside):
            self.status("EXIT ERROR: selected exit heading does not point outside map")
            return False
        if not self.turn_to(direction):
            return False
        self.sense_current_cell()
        front_cm = self.sensors.get_front_cm()
        if front_cm is not None and front_cm <= self.config.front_wall_cm:
            self.status(f"EXIT BLOCKED: front wall at {front_cm:.1f} cm")
            return False

        start_xy = self.pose.get_xy()
        target_m = self.config.exit_drive_cm / 100.0
        if None in start_xy:
            return False
        self.status("EXITING MAZE")
        while not self.stop_event.is_set():
            distance = travelled_from(start_xy, self.pose)
            if distance is None:
                break
            if distance >= target_m:
                hybrid_hard_stop(self.chassis)
                self.status("MAZE EXIT COMPLETE")
                return True
            front_cm = self.sensors.get_front_cm()
            if front_cm is not None and front_cm <= self.config.hard_stop_front_cm:
                break
            x = min(self.config.forward_speed_mps, max(self.config.minimum_speed_mps, (target_m - distance) * 0.8))
            x, y, z, _, _ = self.heading.apply(x, 0.0, self.pose.get_yaw(), "EXIT")
            self.chassis.drive_speed(x=x, y=y, z=z, timeout=self.config.drive_timeout_sec)
            time.sleep(self.config.control_period_sec)
        hybrid_hard_stop(self.chassis)
        return False


class TopologicalGuidedNavigator(HybridNavigator):
    """A* supplies expected nodes; sensors decide when each node is reached."""

    def __init__(
        self,
        chassis,
        sensors: SensorManager,
        pose: PoseTracker,
        heading: HeadingManager,
        maze: GridMazeMap,
        config: HybridConfig,
        events: queue.Queue,
        stop_event: threading.Event,
    ):
        super().__init__(chassis, sensors, pose, heading, maze, config, events, stop_event)
        self.graph = TopologicalMazeGraph(maze, config.cell_size_cm)
        self.current_node = self.graph.marker_nodes["start"]
        self.cell = self.graph.nodes[self.current_node].cell
        self.localization_confirmed = True
        self.localization_failure_reason: Optional[str] = None
        self.maze.topology_memory = self.graph.to_dict()

    def _invalidate_localization(
        self,
        reason: str,
        travelled_m: Optional[float],
    ) -> None:
        self.localization_confirmed = False
        self.localization_failure_reason = reason
        distance_text = "unknown distance" if travelled_m is None else f"{travelled_m:.2f}m"
        self.status(
            "LOCALIZATION UNCERTAIN after "
            f"{distance_text}: {reason}. Stop map writes and do not run Tremaux "
            "from the previous node."
        )

    def _try_relocalize_at_scanned_node(
        self,
        step: TopologyRouteStep,
        scan: dict,
        travelled_m: float,
    ) -> bool:
        """Snap only to a unique, exact node signature on the travel ray."""
        arrival_heading = step.arrival_heading
        observed = self._observed_absolute_directions(scan, arrival_heading)
        from_cell = self.graph.nodes[step.from_node].cell
        dr, dc = DIR_DELTA[step.departure_direction]
        candidates = []
        for node_id, node in self.graph.nodes.items():
            if node_id == step.from_node:
                continue
            row_delta = node.cell[0] - from_cell[0]
            col_delta = node.cell[1] - from_cell[1]
            along_cells = row_delta * dr + col_delta * dc
            cross_cells = abs(row_delta * dc - col_delta * dr)
            if along_cells <= 0 or cross_cells != 0:
                continue
            expected = self._expected_node_directions(node_id)
            if expected != observed:
                continue
            estimated_m = along_cells * self.graph.cell_size_m
            error_m = abs(estimated_m - travelled_m)
            candidates.append((error_m, node_id))

        candidates.sort()
        if not candidates:
            return False
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 0.25:
            print(f"[RELOCALIZE] ambiguous exact matches: {candidates[:3]}")
            return False

        error_m, node_id = candidates[0]
        tolerance_m = max(
            0.30,
            travelled_m * (self.config.corridor_max_length_ratio - 1.0),
        )
        if error_m > tolerance_m:
            print(
                f"[RELOCALIZE] reject {node_id}: length error "
                f"{error_m:.2f}m > {tolerance_m:.2f}m"
            )
            return False

        self.current_node = node_id
        self.cell = self.graph.nodes[node_id].cell
        self.heading_index = arrival_heading
        self.heading.set_heading_index(arrival_heading)
        self.localization_confirmed = True
        self.localization_failure_reason = None
        self.maze.record_pose(self.cell, self.heading_index)
        self._record_confirmed_node_scan(node_id, scan, arrival_heading)
        self.publish_pose()
        self.status(
            f"RELOCALIZED at {node_id} {self.cell}; exact sensor signature, "
            f"distance error {error_m:.2f}m"
        )
        return True

    def _observed_absolute_directions(self, scan: dict, arrival_heading: int) -> Set[int]:
        observed = {(arrival_heading + 2) % 4}  # corridor used to arrive
        if scan.get("front_open"):
            observed.add(arrival_heading)
        if scan.get("left_open"):
            observed.add((arrival_heading - 1) % 4)
        if scan.get("right_open"):
            observed.add((arrival_heading + 1) % 4)
        return observed

    def _expected_node_directions(self, node_id: str) -> Set[int]:
        expected = set(self.graph.node_open_directions(node_id))
        # The topology graph contains only cells inside the drawing. At EXIT,
        # however, the physical opening that points outside is part of the
        # node's sensor signature and must be expected too.
        if node_id == self.graph.marker_nodes.get("exit"):
            expected.add(DIR_FROM_NAME[self.config.exit_heading])
        return expected

    def _signature_score(
        self,
        expected_node: str,
        observed: Set[int],
        incoming_direction: int,
    ) -> float:
        expected = self._expected_node_directions(expected_node)
        incoming = (incoming_direction + 2) % 4
        if incoming not in expected or incoming not in observed:
            return 0.0
        union = expected | observed
        if not union:
            return 1.0
        overlap = len(expected & observed) / len(union)
        # Expected target identity is intentionally stronger than odometry.
        return clamp(0.20 + 0.80 * overlap, 0.0, 1.0)

    def _requires_physical_event(self, node_id: str, arrival_heading: int) -> bool:
        directions = self._expected_node_directions(node_id)
        straight = {arrival_heading, (arrival_heading + 2) % 4}
        return directions != straight

    def _centre_and_scan(self, event: JunctionEvent) -> Optional[dict]:
        hybrid_hard_stop(self.chassis)
        self.walls.reset()
        if event.kind == "SIDE_WINDOW":
            backtrack_to_opening_center(
                self.chassis, self.pose, self.heading, event.backtrack_m
            )
        else:
            center_front_blocked(
                self.chassis, self.sensors, self.pose, self.heading
            )
        self.sensors.reset_sharp_filters()
        scan = scan_junction(self.sensors, event)
        if None in (scan.get("front_cm"), scan.get("left_cm"), scan.get("right_cm")):
            return None
        return scan

    def _record_confirmed_node_scan(
        self,
        node_id: str,
        scan: dict,
        arrival_heading: int,
    ) -> None:
        """Attach a centred physical scan to the confirmed logical node."""
        cell = self.graph.nodes[node_id].cell
        observations = {
            arrival_heading: not bool(scan.get("front_open")),
            (arrival_heading - 1) % 4: not bool(scan.get("left_open")),
            (arrival_heading + 1) % 4: not bool(scan.get("right_open")),
            # The edge just traversed physically proves that BACK is open.
            (arrival_heading + 2) % 4: False,
        }
        for direction, blocked in observations.items():
            self.maze.observe_edge(cell, direction, blocked)
        self.events.put(("map", None))

    def _publish_edge_progress(
        self,
        step: TopologyRouteStep,
        travelled_m: float,
    ) -> None:
        edge = self.graph.edges[step.edge_id]
        path = edge.path_from(step.from_node)
        if len(path) < 2:
            return
        ratio = clamp(travelled_m / max(0.05, edge.planning_length_m), 0.0, 1.0)
        scaled = ratio * (len(path) - 1)
        index = min(len(path) - 2, int(scaled))
        local_fraction = scaled - index
        visual_cell = path[index]
        visual_direction = direction_between(path[index], path[index + 1])
        self.maze.robot_heading = self.heading_index
        self.events.put((
            "pose",
            {
                "cell": visual_cell,
                "heading": self.heading_index,
                "fraction": local_fraction,
                "direction": visual_direction,
            },
        ))

    def _confirm_topology_arrival(
        self,
        step: TopologyRouteStep,
        measured_length_m: float,
        reason: str,
    ) -> None:
        edge = self.graph.edges[step.edge_id]
        self.graph.confirm_edge(
            step.edge_id, measured_length_m, self.config.learned_length_alpha
        )
        path = edge.path_from(step.from_node)
        for previous, current in zip(path, path[1:]):
            self.maze.confirm_traversed_open_edge(
                previous,
                direction_between(previous, current),
            )
        for cell in path[1:]:
            self.maze.record_pose(cell, step.arrival_heading)
        self.current_node = step.to_node
        self.cell = self.graph.nodes[step.to_node].cell
        self.heading_index = step.arrival_heading
        self.heading.set_heading_index(self.heading_index)
        self.maze.record_pose(self.cell, self.heading_index)
        self.events.put(("map", None))
        self.publish_pose()
        self.status(
            f"NODE CONFIRMED {step.to_node} at {self.cell} "
            f"via {step.edge_id}, {measured_length_m:.2f}m ({reason})"
        )

    def traverse_topology_edge(self, step: TopologyRouteStep) -> bool:
        if not self.localization_confirmed:
            self.status("Refusing edge traversal: logical node is not confirmed")
            return False
        if step.from_node != self.current_node:
            raise RuntimeError(
                f"Route desync: at {self.current_node}, step starts at {step.from_node}"
            )
        edge = self.graph.edges[step.edge_id]
        edge.attempts += 1
        edge.state = "IN_PROGRESS"
        self.maze.topology_memory = self.graph.to_dict()
        self.status(
            f"EDGE {edge.edge_id}: {step.from_node} -> {step.to_node}, "
            f"expected≈{edge.planning_length_m:.2f}m"
        )

        if not self.turn_to(step.departure_direction):
            self.graph.block_edge(edge.edge_id, "TURN_FAILED")
            return False
        sensors_after_turn = self.sense_current_cell(samples=4)
        if sensors_after_turn.get(step.departure_direction) is True:
            self.graph.block_edge(edge.edge_id, "BLOCKED_AT_DEPARTURE")
            self.status(f"EDGE {edge.edge_id} blocked at departure; A* will replan")
            return False

        start_xy = self.pose.get_xy()
        if None in start_xy:
            self.graph.block_edge(edge.edge_id, "ODOMETRY_UNAVAILABLE")
            return False

        detector = JunctionDetector()
        detector.lock_here(start_xy)
        expected_length = max(0.05, edge.planning_length_m)
        minimum_event_m = max(
            self.config.corridor_min_progress_cm / 100.0,
            # The drawing scale is only a guide. Cap this guard so a short
            # real corridor is not rejected merely because the GUI cells were
            # drawn too large. Its main job is suppressing a re-trigger from
            # the junction we have just left.
            min(expected_length * self.config.corridor_min_length_ratio, 0.35),
        )
        maximum_m = max(
            minimum_event_m + 0.20,
            expected_length * self.config.corridor_max_length_ratio,
        )
        requires_event = self._requires_physical_event(
            step.to_node, step.arrival_heading
        )
        started = time.monotonic()
        last_print = 0.0
        ignored_events = 0

        while not self.stop_event.is_set():
            if time.monotonic() - started > self.config.corridor_timeout_sec:
                hybrid_hard_stop(self.chassis)
                self.graph.block_edge(edge.edge_id, "CORRIDOR_TIMEOUT")
                timeout_distance = travelled_from(start_xy, self.pose)
                if (
                    timeout_distance is None
                    or timeout_distance >= self.config.localization_guard_cm / 100.0
                ):
                    self._invalidate_localization(
                        "CORRIDOR_TIMEOUT", timeout_distance
                    )
                return False

            travelled = travelled_from(start_xy, self.pose)
            if travelled is None:
                hybrid_hard_stop(self.chassis)
                self.graph.block_edge(edge.edge_id, "ODOMETRY_LOST")
                self._invalidate_localization("ODOMETRY_LOST", None)
                return False

            raw_l, left_cm = self.sensors.read_left()
            raw_r, right_cm = self.sensors.read_right()
            front_cm = self.sensors.get_front_cm()
            ir_left, ir_right = self.sensors.read_front_corner_ir()
            event = detector.update(
                front_cm, left_cm, right_cm, self.pose.get_xy()
            )

            if event is not None:
                scan = self._centre_and_scan(event)
                centred_distance = travelled_from(start_xy, self.pose)
                if centred_distance is not None:
                    travelled = centred_distance
                if scan is None:
                    detector.lock_here(self.pose.get_xy())
                    time.sleep(self.config.control_period_sec)
                    continue
                observed = self._observed_absolute_directions(
                    scan, step.arrival_heading
                )
                expected = self._expected_node_directions(step.to_node)
                score = self._signature_score(
                    step.to_node, observed, step.departure_direction
                )
                print(
                    f"[NODE CANDIDATE] expected={step.to_node} "
                    f"expected_dirs={[HEADINGS[d] for d in sorted(expected)]} "
                    f"observed={[HEADINGS[d] for d in sorted(observed)]} "
                    f"score={score:.2f} progress={travelled:.2f}m"
                )

                if travelled < minimum_event_m:
                    ignored_events += 1
                    self.status(
                        f"IGNORE EARLY JUNCTION #{ignored_events}: "
                        f"{travelled:.2f}m < {minimum_event_m:.2f}m"
                    )
                    detector.lock_here(self.pose.get_xy())
                    self.walls.reset()
                    time.sleep(self.config.control_period_sec)
                    continue

                if score >= self.config.junction_signature_score:
                    self._confirm_topology_arrival(
                        step, travelled, f"SIGNATURE {score:.2f}"
                    )
                    self._record_confirmed_node_scan(
                        step.to_node, scan, step.arrival_heading
                    )
                    detector.lock_here(self.pose.get_xy())
                    return True

                # The robot may have reached a different drawn node because
                # the map scale/topology is imperfect. Accept only a unique
                # exact signature along the current travel ray.
                if self._try_relocalize_at_scanned_node(step, scan, travelled):
                    self.graph.block_edge(
                        edge.edge_id,
                        f"RELOCALIZED_TO_{self.current_node}",
                    )
                    detector.lock_here(self.pose.get_xy())
                    return True

                # Extra opening can be a map mismatch.  Continue through it only
                # when the front is physically open; a mismatched hard corner is
                # unsafe and blocks this planned edge.
                if scan.get("front_open") and travelled < maximum_m:
                    self.status(
                        f"UNEXPECTED JUNCTION score={score:.2f}; keep expected "
                        f"target {step.to_node} and continue straight"
                    )
                    detector.lock_here(self.pose.get_xy())
                    self.walls.reset()
                    time.sleep(self.config.control_period_sec)
                    continue

                self.graph.block_edge(
                    edge.edge_id, f"SIGNATURE_MISMATCH_{score:.2f}"
                )
                self.status(
                    f"EDGE {edge.edge_id} mismatch at {travelled:.2f}m; replan"
                )
                self._invalidate_localization(
                    f"SIGNATURE_MISMATCH_{score:.2f}", travelled
                )
                return False

            # START/DROP/EXIT can be forced onto an otherwise straight corridor.
            # Such a point has no detectable junction, so use the learned edge
            # length (or drawing estimate on the first run) only for that case.
            if not requires_event and travelled >= expected_length:
                hybrid_hard_stop(self.chassis)
                self._confirm_topology_arrival(
                    step, travelled, "LEARNED/ESTIMATED STRAIGHT MARKER"
                )
                return True

            if requires_event and travelled >= maximum_m:
                hybrid_hard_stop(self.chassis)
                self.graph.block_edge(edge.edge_id, "EXPECTED_NODE_NOT_FOUND")
                self.status(
                    f"EXPECTED NODE {step.to_node} not found before {maximum_m:.2f}m"
                )
                self._invalidate_localization(
                    "EXPECTED_NODE_NOT_FOUND", travelled
                )
                return False

            if front_cm is not None and front_cm <= self.config.hard_stop_front_cm:
                x_cmd = y_cmd = z_cmd = 0.0
                mode = "FRONT_CONFIRM"
            else:
                x_cmd = self.walls.forward_speed(front_cm)
                y_cmd, mode = self.walls.lateral(left_cm, right_cm)
                if mode.startswith("ESCAPE_"):
                    x_cmd = min(x_cmd, ESCAPE_FORWARD_SPEED)
                elif mode.startswith("AVOID_"):
                    x_cmd = min(x_cmd, SIDE_WARNING_FORWARD_SPEED)
                if mode in ("BOTH_TOO_CLOSE", "NO_SIDE_SENSOR"):
                    x_cmd = 0.0
                x_cmd, y_cmd, ir_mode = apply_front_corner_ir_guard(
                    x_cmd, y_cmd, ir_left, ir_right
                )
                if ir_mode:
                    mode = ir_mode
                x_cmd, y_cmd, z_cmd, mode, _ = self.heading.apply(
                    x_cmd, y_cmd, self.pose.get_yaw(), mode
                )

            self.chassis.drive_speed(
                x=x_cmd,
                y=y_cmd,
                z=z_cmd,
                timeout=self.config.drive_timeout_sec,
            )
            self._publish_edge_progress(step, travelled)
            now = time.monotonic()
            if now - last_print >= 0.30:
                print(
                    f"[TOPO EDGE] {edge.edge_id} {travelled:.2f}m "
                    f"expected≈{expected_length:.2f}m max={maximum_m:.2f}m "
                    f"F={fmt(front_cm)} L={fmt(left_cm)} R={fmt(right_cm)} "
                    f"ADC=({raw_l},{raw_r}) mode={mode}"
                )
                last_print = now
            time.sleep(self.config.control_period_sec)

        hybrid_hard_stop(self.chassis)
        edge.state = "UNSEEN"
        return False

    def navigate_topology_to(self, goal_marker: str, label: str) -> bool:
        goal_node = self.graph.marker_nodes[goal_marker]
        replans = 0
        while self.current_node != goal_node and not self.stop_event.is_set():
            route = self.graph.route(
                self.current_node,
                self.heading_index,
                goal_node,
                self.config.turn_cost,
            )
            if route is None:
                self.status(f"A* has no graph route to {label}")
                return False
            expanded = self.graph.expanded_cells(route)
            self.maze.planned_path = expanded or [self.cell]
            self.events.put(("map", None))
            if not route:
                return True
            if not self.traverse_topology_edge(route[0]):
                if not self.localization_confirmed:
                    return False
                replans += 1
                if replans > self.config.max_replans:
                    return False
                continue
            replans = 0
        return self.current_node == goal_node

    def tremaux_to_marker(
        self,
        goal_marker: str,
        on_drop: Optional[Callable[[], None]] = None,
    ) -> bool:
        """Known-topology fallback: 0-traversal edges before 1; never over limit."""
        goal_node = self.graph.marker_nodes[goal_marker]
        dropped = on_drop is None
        if not self.localization_confirmed:
            self.status(
                "TRÉMAUX NOT STARTED: current physical node is uncertain; "
                "using the previous cell would corrupt the map"
            )
            return False
        self.status("FALLBACK: TOPOLOGICAL TRÉMAUX")
        for _ in range(self.config.explore_max_steps):
            if self.stop_event.is_set():
                return False
            if not dropped and self.current_node == self.graph.marker_nodes["drop"]:
                on_drop()
                dropped = True
            if dropped and self.current_node == goal_node:
                return True

            sensed = self.sense_current_cell(samples=5)
            candidates = []
            for edge_id in self.graph.nodes[self.current_node].edge_ids:
                edge = self.graph.edges[edge_id]
                if edge.state == "BLOCKED":
                    continue
                if edge.traversals >= self.config.fallback_edge_limit:
                    continue
                direction = edge.direction_from(self.current_node)
                if sensed.get(direction) is True:
                    self.graph.block_edge(edge_id, "SENSOR_BLOCKED_AT_NODE")
                    continue
                target = edge.other(self.current_node)
                target_is_goal = target == goal_node
                score = (
                    0 if target_is_goal else 1,
                    0 if edge.traversals == 0 else 1,
                    edge.traversals,
                    turn_distance(self.heading_index, direction),
                )
                candidates.append((score, edge_id, target, direction))

            if not candidates:
                self.status("TRÉMAUX exhausted all graph edges")
                return False
            candidates.sort()
            _, edge_id, target, direction = candidates[0]
            edge = self.graph.edges[edge_id]
            step = TopologyRouteStep(
                edge_id=edge_id,
                from_node=self.current_node,
                to_node=target,
                departure_direction=direction,
                arrival_heading=edge.arrival_heading_from(self.current_node),
            )
            if not self.traverse_topology_edge(step):
                if not self.localization_confirmed:
                    return False
                continue
        return False


def simulate_mission(
    config: HybridConfig,
    maze: GridMazeMap,
    events: queue.Queue,
    stop_event: threading.Event,
) -> None:
    heading = DIR_FROM_NAME[config.start_heading]
    route = mission_route_preview(maze, heading, config.turn_cost, config.cell_size_cm)
    if route is None:
        raise RuntimeError("No A* route for the drawn map")
    maze.planned_path = route
    maze.travel_path.clear()
    for index, cell in enumerate(route):
        if stop_event.is_set():
            return
        if index > 0:
            heading = direction_between(route[index - 1], cell)
        maze.record_pose(cell, heading)
        maze.status = "SIMULATION"
        events.put(("pose", {"cell": cell, "heading": heading, "fraction": 0.0, "direction": heading}))
        time.sleep(0.20)
    maze.status = "SIMULATION COMPLETE"
    json_path, map_path, graph_path = export_run_artifacts(config, maze)
    events.put(
        (
            "done",
            f"Simulation complete | {json_path.name} | "
            f"{map_path.name} | {graph_path.name}",
        )
    )


def run_hybrid_robot(
    config: HybridConfig,
    maze: GridMazeMap,
    events: queue.Queue,
    stop_event: threading.Event,
) -> None:
    if robot is None:
        raise RuntimeError("RoboMaster SDK is not installed. Use Simulation or install robomaster.")
    apply_runtime_config(config)
    ep_robot = robot.Robot()
    chassis = tof_sensor = None
    pose_subscribed = attitude_subscribed = tof_subscribed = False
    navigator = None
    final_message = "MISSION STOPPED"

    def status(message: str) -> None:
        maze.status = message
        print(f"[MISSION] {message}")
        events.put(("status", message))

    try:
        connection_type = sdk_connection_type(config.connection)
        status(f"CONNECTING ROBOMASTER ({config.connection.upper()})")
        ep_robot.initialize(conn_type=connection_type)
        chassis = ep_robot.chassis
        sensor_adapter = ep_robot.sensor_adaptor
        tof_sensor = ep_robot.sensor
        arm = ep_robot.robotic_arm
        gripper = ep_robot.gripper

        pose = PoseTracker()
        sensors = SensorManager(sensor_adapter)
        pickup_tof = HybridToFMonitor()

        def shared_tof_callback(data) -> None:
            sensors.tof_callback(data)
            pickup_tof.callback(data)

        tof_sensor.sub_distance(freq=20, callback=shared_tof_callback)
        tof_subscribed = True
        chassis.sub_position(cs=1, freq=POSE_FREQ_HZ, callback=pose.position_callback)
        pose_subscribed = True
        chassis.sub_attitude(freq=ATTITUDE_FREQ_HZ, callback=pose.attitude_callback)
        attitude_subscribed = True
        wait_for_position(pose)
        start_yaw = wait_for_yaw(pose)

        if config.pickup_enabled:
            if not pickup_tof.wait_first(5.0, stop_event):
                raise RuntimeError("ToF did not produce a callback")
            if not pickup_and_verify(
                chassis, arm, gripper, pose, pickup_tof, config, stop_event, status
            ):
                raise RuntimeError("Pickup failed; maze mission was not started")
        else:
            status("PICKUP SKIPPED")

        # Anchor map localization after pickup movement is complete.
        sensors.reset_filters()
        time.sleep(0.20)
        start_yaw = pose.get_yaw()
        heading = HeadingManager()
        if not heading.initialize_for_heading(start_yaw, DIR_FROM_NAME[config.start_heading]):
            raise RuntimeError("Attitude/yaw is unavailable")
        maze.reset_run_data()
        maze.record_pose(maze.start, DIR_FROM_NAME[config.start_heading])
        navigator = HybridNavigator(
            chassis, sensors, pose, heading, maze, config, events, stop_event
        )
        navigator.publish_pose()

        carrying = bool(config.pickup_enabled)

        def drop_if_found() -> None:
            nonlocal carrying
            if carrying:
                if not align_drop_position(
                    navigator,
                    sensors,
                    pose,
                    heading,
                    config,
                    stop_event,
                    status,
                ):
                    raise RuntimeError(
                        "Could not align DROP to the configured front/side "
                        "wall distances; object was not released"
                    )
                place_object(arm, gripper, pickup_tof, config, stop_event, status)
                sensors.reset_filters()
                navigator.begin_manual_drop_departure()
                carrying = False

        reached_drop = navigator.navigate_to(maze.drop, "DROP")
        if reached_drop:
            drop_if_found()
        else:
            if not navigator.explore_to_exit(
                maze.exit,
                drop_if_found if carrying else None,
            ):
                raise RuntimeError(
                    "Basic A* and cell-level Tremaux could not reach DROP/EXIT"
                )

        if navigator.cell != maze.exit:
            reached_exit = navigator.navigate_to(maze.exit, "EXIT")
            if not reached_exit:
                if navigator.manual_drop_departure_pending:
                    raise RuntimeError(
                        "Post-DROP return to the previously traversed cell was "
                        "blocked; mission stopped without writing a false wall"
                    )
                reached_exit = navigator.explore_to_exit(maze.exit)
            if not reached_exit:
                raise RuntimeError("Could not reach the selected EXIT cell")
        if carrying:
            raise RuntimeError(
                "EXIT was reached before the selected DROP was confirmed; "
                "object remains in the gripper"
            )

        if not navigator.drive_outside():
            raise RuntimeError(
                "Selected EXIT is physically blocked or its heading is wrong. "
                "The robot stopped safely; correct the EXIT marker/heading."
            )
        final_message = "MISSION COMPLETE: pickup, drop, and exit finished"
        status(final_message)

    finally:
        hybrid_hard_stop(chassis)
        try:
            if tof_sensor is not None and tof_subscribed:
                tof_sensor.unsub_distance()
        except Exception:
            pass
        try:
            if chassis is not None and pose_subscribed:
                chassis.unsub_position()
        except Exception:
            pass
        try:
            if chassis is not None and attitude_subscribed:
                chassis.unsub_attitude()
        except Exception:
            pass
        try:
            ep_robot.close()
        except Exception:
            pass
        if maze.travel_path:
            json_path, map_path, graph_path = export_run_artifacts(config, maze)
            events.put(
                (
                    "done",
                    f"{final_message} | {json_path.name} | "
                    f"{map_path.name} | {graph_path.name}",
                )
            )


# ============================================================
# Main
# ============================================================

def legacy_main():
    ep_robot = robot.Robot()
    chassis = None
    tof_sensor = None
    pose_subscribed = False
    attitude_subscribed = False
    tof_subscribed = False

    try:
        print("Connecting RoboMaster...")
        ep_robot.initialize(conn_type="ap")

        chassis = ep_robot.chassis
        sensor_adapter = ep_robot.sensor_adaptor
        tof_sensor = ep_robot.sensor

        pose = PoseTracker()
        sensors = SensorManager(sensor_adapter)
        heading = HeadingManager()
        walls = WallController()
        junctions = JunctionDetector()
        exit_detector = ExitDetector()
        explorer = MazeGraphExplorer()

        tof_subscribed = tof_sensor.sub_distance(freq=20, callback=sensors.tof_callback)
        pose_subscribed = chassis.sub_position(cs=1, freq=POSE_FREQ_HZ, callback=pose.position_callback)
        attitude_subscribed = chassis.sub_attitude(freq=ATTITUDE_FREQ_HZ, callback=pose.attitude_callback)

        start_x, start_y = wait_for_position(pose)
        start_yaw = wait_for_yaw(pose)
        heading.initialize(start_yaw)

        start_node = explorer.initialize_start(start_x, start_y)

        # Initial corridor departure: create a forward frontier at the synthetic start.
        start_forward = explorer.absolute_for_relative("FRONT")
        explorer._edge(start_node, start_forward).observed = True
        initial_plan = Plan("FRONT", start_forward, "INITIAL_FORWARD")
        explorer.commit_departure(initial_plan, start_xy=(start_x, start_y))

        print("============================================================")
        print(" RoboMaster CLEAN Wall-Maze Solver")
        print("============================================================")
        print(f"Version             : {PROGRAM_VERSION}")
        print(f"Forward speed       : {FORWARD_SPEED:.2f} m/s")
        print(f"Side open           : >= {SIDE_OPEN_ENTER_CM:.1f} cm")
        print(f"Front open          : >= {EXPLORATION_FRONT_OPEN_CM:.1f} cm")
        print(f"Node match          : {NODE_MATCH_RADIUS_M:.2f} m")
        print(f"Topology loop match : {LOOP_MATCH_RADIUS_M:.2f} m")
        print(f"Same-node retrigger : <= {SAME_NODE_RETRIGGER_RADIUS_M:.2f} m")
        print("Entry edge guard    : ON (start corridor is not a return route)")
        print(f"Grid resolution     : {GRID_RESOLUTION_M:.2f} m")
        print(f"Exit confirm        : {EXIT_CONFIRM_DISTANCE_M:.2f} m sustained open space")
        print(f"IR front-left ID    : {IR_FRONT_LEFT_ID}")
        print(f"IR front-right ID   : {IR_FRONT_RIGHT_ID}")
        print(f"IR blocked level    : {IR_BLOCKED_LEVEL}")
        print(f"Digital IR mode      : get_io() active-low")
        print(f"Side warning        : < {SIDE_WARNING_CM:.1f} cm")
        print("Confirmed-edge DFS  : ON (count edge only after arrival)")
        print("Post-turn clearance : ON" if ENABLE_POST_TURN_CLEARANCE else "Post-turn clearance : OFF")
        print("============================================================")

        last_print = 0.0

        while True:
            raw_l, left_cm = sensors.read_left()
            raw_r, right_cm = sensors.read_right()
            front_cm = sensors.get_front_cm()
            ir_left, ir_right = sensors.read_front_corner_ir()
            pose_xy = pose.get_xy()

            # ------------------------------------------------
            # 1) Maze exit has priority over creating endless outside nodes.
            # ------------------------------------------------
            if exit_detector.update(
                front_cm,
                left_cm,
                right_cm,
                pose_xy,
                ir_left,
                ir_right,
            ):
                stop_chassis(chassis)
                print()
                print("============================================")
                print(" MAZE EXIT CONFIRMED")
                print("============================================")
                if SAVE_MAZE_MEMORY:
                    explorer.save()
                break

            # ------------------------------------------------
            # 2) Junction detection
            # ------------------------------------------------
            event = junctions.update(front_cm, left_cm, right_cm, pose_xy)

            if event is not None:
                stop_chassis(chassis)
                walls.reset()

                # Establish a repeatable physical node position BEFORE graph registration.
                if event.kind == "SIDE_WINDOW":
                    backtrack_to_opening_center(chassis, pose, heading, event.backtrack_m)
                else:
                    center_front_blocked(chassis, sensors, pose, heading)

                # Stable stopped scan at the canonical junction centre.
                scan = scan_junction(sensors, event)
                if None in (scan["front_cm"], scan["left_cm"], scan["right_cm"]):
                    print("Decision rejected: incomplete sensor snapshot")
                    junctions.lock_here(pose.get_xy())
                    time.sleep(LOOP_DELAY_SEC)
                    continue

                # If a side event resolves into a plain corridor, ignore it.
                if (
                    event.kind == "SIDE_WINDOW"
                    and scan["front_open"]
                    and not scan["left_open"]
                    and not scan["right_open"]
                    and not (event.observed_left or event.observed_right)
                ):
                    print("Decision rejected: plain corridor after centred scan")
                    junctions.lock_here(pose.get_xy())
                    time.sleep(LOOP_DELAY_SEC)
                    continue

                x, y = pose.get_xy()
                if x is None or y is None:
                    print("No odometry at junction; stopping for safety")
                    break

                observed_abs = observed_absolute_openings(explorer, scan)
                node_id, is_new, match_reason = explorer.arrive(
                    float(x), float(y), observed_abs
                )
                explorer.observe_openings(observed_abs)

                print()
                print(
                    f"[{'NEW' if is_new else 'KNOWN'} NODE] {node_id} "
                    f"at ({x:+.3f},{y:+.3f}) match={match_reason}"
                )
                print("Memory:", explorer.describe_node(node_id))
                print(
                    "Frontiers:",
                    " | ".join(f"{n}.{HEADINGS[h]}" for n, h in explorer.all_frontiers()) or "NONE",
                )

                # V3: a same-node retrigger is NOT a successful edge traversal.
                # If the planned direction is still open, this was probably a
                # wide junction mouth: ignore the duplicate event and continue.
                # If it immediately ends in a hard front block after only a
                # short distance, cancel it as a false/blocked exit.
                if match_reason == "SAME_NODE_RETRIGGER":
                    pending_abs = explorer.pending_abs
                    progress = explorer.pending_progress_m(float(x), float(y))
                    pending_still_open = (
                        pending_abs is not None and pending_abs in observed_abs
                    )
                    pending_unknown = False
                    if explorer.pending_from is not None and pending_abs is not None:
                        pending_unknown = (
                            explorer._edge(explorer.pending_from, pending_abs).target is None
                        )
                    hard_front_blocked = (
                        scan["front_cm"] is not None
                        and scan["front_cm"] <= STOP_FRONT_CM
                    )

                    if (
                        pending_unknown
                        and not pending_still_open
                        and hard_front_blocked
                        and progress is not None
                        and progress <= FAILED_EDGE_MAX_PROGRESS_M
                    ):
                        print(
                            ">>> FALSE/SHORT EXIT: cancel pending edge "
                            f"progress={progress:.3f}m front={scan['front_cm']:.1f}cm"
                        )
                        explorer.cancel_pending_as_blocked(
                            "SAME_NODE_HARD_FRONT",
                            float(x),
                            float(y),
                        )
                        junctions.lock_here(pose.get_xy())
                    else:
                        print(
                            ">>> SAME NODE WINDOW: keep pending corridor, "
                            f"progress={0.0 if progress is None else progress:.3f}m"
                        )
                        if SAVE_MAZE_MEMORY:
                            explorer.save()
                        junctions.lock_here(pose.get_xy())
                        time.sleep(LOOP_DELAY_SEC)
                        continue

                if SAVE_MAZE_MEMORY:
                    explorer.save()

                plan = explorer.plan(current_observed_abs=observed_abs)
                if plan is None:
                    stop_chassis(chassis)
                    print()
                    print("============================================")
                    print(" GRAPH EXPLORATION COMPLETE")
                    print(" No unexplored junction exits remain.")
                    print("============================================")
                    if SAVE_MAZE_MEMORY:
                        explorer.save()
                    break

                print(
                    f">>> PLAN {plan.relative} ({HEADINGS[plan.absolute]}) "
                    f"reason={plan.reason}"
                )

                # Physical action first; graph edge is marked only if turn succeeds.
                if not feedback_turn(chassis, sensors, pose, plan.relative):
                    stop_chassis(chassis)
                    print("Turn failed safely; departure NOT committed")
                    break

                explorer.commit_departure(plan, start_xy=(float(x), float(y)))
                heading.set_heading_index(explorer.heading_index)
                align_heading(chassis, pose, heading)

                # Clear the inside corner before resuming normal speed.
                post_turn_clearance(
                    chassis,
                    sensors,
                    pose,
                    heading,
                    plan.relative,
                )

                sensors.reset_filters()
                walls.reset()
                junctions.lock_here(pose.get_xy())
                exit_detector.reset()

                if SAVE_MAZE_MEMORY:
                    explorer.save()

                stop_chassis(chassis)
                time.sleep(AFTER_TURN_DELAY_SEC)
                continue

            # ------------------------------------------------
            # 3) Normal corridor movement
            # ------------------------------------------------
            if front_cm is not None and front_cm <= STOP_FRONT_CM:
                # Stop while JunctionDetector gathers confirmation samples.
                x_cmd = 0.0
                y_cmd = 0.0
                z_cmd = 0.0
                mode = "FRONT_CONFIRM"
            else:
                x_cmd = walls.forward_speed(front_cm)
                y_cmd, mode = walls.lateral(left_cm, right_cm)

                # Side-too-close escape should not keep full forward speed.
                if mode.startswith("ESCAPE_"):
                    x_cmd = min(x_cmd, ESCAPE_FORWARD_SPEED)
                elif mode.startswith("AVOID_"):
                    x_cmd = min(x_cmd, SIDE_WARNING_FORWARD_SPEED)
                if mode in ("BOTH_TOO_CLOSE", "NO_SIDE_SENSOR"):
                    x_cmd = 0.0

                # Front-corner digital IR guard protects chassis corners.
                x_cmd, y_cmd, ir_mode = apply_front_corner_ir_guard(
                    x_cmd, y_cmd, ir_left, ir_right
                )
                if ir_mode:
                    mode = ir_mode

                x_cmd, y_cmd, z_cmd, mode, _ = heading.apply(
                    x_cmd,
                    y_cmd,
                    pose.get_yaw(),
                    mode,
                )

            if ENABLE_MOTION:
                chassis.drive_speed(
                    x=x_cmd,
                    y=y_cmd,
                    z=z_cmd,
                    timeout=DRIVE_TIMEOUT_SEC,
                )

            # ------------------------------------------------
            # Debug print, throttled
            # ------------------------------------------------
            now = time.monotonic()
            if now - last_print >= PRINT_EVERY_SEC:
                px, py = pose.get_xy()
                yaw = pose.get_yaw()
                yaw_err = heading.error(yaw)
                print(
                    f"ToF:{fmt(front_cm)} | "
                    f"L:{fmt(left_cm)} ADC:{raw_l:4d} | "
                    f"R:{fmt(right_cm)} ADC:{raw_r:4d} | "
                    f"IR-L:{ir_left} IR-R:{ir_right} | "
                    f"Pose:({px if px is not None else 0:+.2f},{py if py is not None else 0:+.2f}) | "
                    f"Yaw:{yaw if yaw is not None else 0:+.1f} "
                    f"Err:{yaw_err if yaw_err is not None else 0:+.1f} | "
                    f"H:{HEADINGS[explorer.heading_index]} | "
                    f"{mode:22s} | x={x_cmd:.3f} y={y_cmd:+.3f} z={z_cmd:+.1f}"
                )
                last_print = now

            time.sleep(LOOP_DELAY_SEC)

    except KeyboardInterrupt:
        print("\nSTOP REQUESTED BY USER")

    except Exception as exc:
        print("\nERROR:", exc)
        raise

    finally:
        try:
            stop_chassis(chassis)
        except Exception:
            pass

        try:
            if tof_sensor is not None and tof_subscribed:
                tof_sensor.unsub_distance()
        except Exception:
            pass

        try:
            if chassis is not None and pose_subscribed:
                chassis.unsub_position()
        except Exception:
            pass

        try:
            if chassis is not None and attitude_subscribed:
                chassis.unsub_attitude()
        except Exception:
            pass

        try:
            ep_robot.close()
        except Exception:
            pass

        print("Robot stopped and disconnected.")
