from __future__ import annotations

import math
import statistics
import threading
from collections import deque
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Set, Tuple

from robomaster import robot

from robomaster_mission.mission import (
    HeadingManager,
    PoseTracker,
    SensorManager,
    normalize_angle_deg,
    wait_for_position,
    wait_for_yaw,
)

from .camera_service import CameraService
from .live_survey import LiveSurveyBridge
from .motion_safety_v05 import adjacent_wall_sides
from .wall_clearance_v05 import choose_clearance_plan, clearance_target
from .config import Classwork8Config
from .occupancy_grid import OccupancyGrid
from .reporting import RunRecorder
from .target_detection import (
    TargetDetector,
    TargetRegistry,
    save_topology,
)
from .vision import CorridorVision


def stop_chassis(chassis) -> None:
    """V05 ONLY: stop in individual-wheel mode, never zero chassis speed mode.

    Real-robot isolation tests showed slow physical left rotation after
    chassis.drive_speed(0,0,0) switched SDK status 8 -> 1. In contrast,
    drive_wheels(0,0,0,0) returned True, switched status to 0, and held
    chassis yaw during and after the independent gimbal sweep. Preserve the
    legacy mission stop helper unchanged for other programs.
    """
    if chassis is None:
        return
    # SDK Chassis.drive_speed(..., timeout=...) arms a Timer that later
    # invokes drive_speed(0,0,0). It could otherwise switch us BACK into
    # the drifting chassis speed mode even after a successful wheel stop.
    # SDK Chassis.stop() cancels only that timer (Module.stop is a no-op).
    timer_error = None
    try:
        chassis.stop()
    except Exception as exc:
        timer_error = exc
    # Still issue a physical zero-wheel command if timer cancellation failed.
    ack = chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    if ack is not True:
        # Fail closed rather than silently fall back to the speed-mode
        # zero command that reproduced physical yaw creep on this robot.
        raise RuntimeError(
            "V05_WHEEL_STOP_NOT_ACKNOWLEDGED: drive_wheels(0,0,0,0) "
            "did not confirm the stop; halt the run and inspect the robot"
        )
    if timer_error is not None:
        raise RuntimeError(
            "V05_STOP_TIMER_CANCEL_FAILED: wheel zero was sent, but the "
            "previous drive_speed timer may remain active"
        ) from timer_error


# Logical map directions relative to the chassis heading at mission start.
# Map convention keeps +Y upward/left, while RoboMaster chassis +Y moves right.
DIR_VEC_MAP = {
    0: (1, 0),    # front
    1: (0, -1),   # right
    2: (-1, 0),   # back
    3: (0, 1),    # left
}

# RoboMaster body-frame drive_speed vectors.
DIR_VEC_DRIVE = {
    0: (1.0, 0.0),   # forward
    1: (0.0, 1.0),   # right strafe
    2: (-1.0, 0.0),  # reverse
    3: (0.0, -1.0),  # left strafe
}

# Positive camera correction means "move right relative to the current travel
# direction". Convert that travel-frame vector into chassis x/y.
DIR_RIGHT_VEC_DRIVE = {
    0: (0.0, 1.0),    # facing front
    1: (-1.0, 0.0),   # facing right
    2: (0.0, -1.0),   # facing back
    3: (1.0, 0.0),    # facing left
}

DIR_NAME = {
    0: "FRONT",
    1: "RIGHT",
    2: "BACK",
    3: "LEFT",
}


class ToFOnlySensorManager(SensorManager):
    """Use the legacy ToF filtering without touching Sensor Adapter hardware."""

    def __init__(self):
        super().__init__(None)

    def read_front_corner_ir(self):
        return None, None


class V05PoseTracker(PoseTracker):
    """Include attitude receipt time; non-None yaw can still be stale."""

    def __init__(self):
        super().__init__()
        self._yaw_received_at = None

    def attitude_callback(self, data):
        try:
            if data is None or len(data) < 3 or not math.isfinite(float(data[0])):
                return
        except (TypeError, ValueError, IndexError):
            return
        super().attitude_callback(data)
        with self._lock:
            self._yaw_received_at = time.monotonic()

    def attitude_age_sec(self) -> Optional[float]:
        with self._lock:
            timestamp = self._yaw_received_at
        return None if timestamp is None else max(0.0, time.monotonic() - timestamp)


class GimbalTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self.pitch = None
        self.yaw = None
        self.yaw_ground = None  # Diagnostic only; not a second chassis controller.
        self._angle_history = deque(maxlen=4000)

    def callback(self, data):
        try:
            if data is None or len(data) < 2:
                return
            pitch = float(data[0])
            yaw = float(data[1])
            ground = float(data[3]) if len(data) >= 4 else None
            with self._lock:
                self.pitch = pitch
                self.yaw = yaw
                self.yaw_ground = ground
                self._angle_history.append((time.monotonic(), pitch, yaw))
        except Exception:
            return

    def get_yaw(self) -> Optional[float]:
        with self._lock:
            return self.yaw

    def get_pitch(self) -> Optional[float]:
        with self._lock:
            return self.pitch

    def get_angles(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.pitch, self.yaw

    def get_yaws(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.yaw, self.yaw_ground

    def pitch_samples_since(self, start_monotonic: float) -> List[float]:
        """Measured pitch during a yaw sweep, including transient excursions."""
        with self._lock:
            return [
                float(pitch)
                for timestamp, pitch, _yaw in self._angle_history
                if timestamp >= float(start_monotonic)
            ]


def _sleep_interruptible(seconds: float, stop_event: Optional[threading.Event]) -> bool:
    deadline = time.monotonic() + max(0.0, float(seconds))
    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return False
        time.sleep(min(0.03, max(0.0, deadline - time.monotonic())))
    return True


def _map_xy_from_raw(
    raw_x: float,
    raw_y: float,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
) -> Tuple[float, float]:
    """Rotate DJI power-on odometry into the chassis frame at mission start.

    DJI sub_position(cs=1) keeps the coordinate axes from robot power-on.
    The mission may begin after the robot has already been moved/rotated, so
    subtracting only x/y is not enough.  We rotate by the initial chassis yaw
    so map +X is the robot FRONT at mission start and map +Y is robot LEFT.
    """
    dx = float(raw_x) - float(start_x)
    dy = float(raw_y) - float(start_y)

    theta = math.radians(float(start_yaw_deg))
    c = math.cos(theta)
    sn = math.sin(theta)

    # World(power-on frame) -> body frame at mission start.
    body_x = c * dx + sn * dy
    body_y_right = -sn * dx + c * dy

    # Conventional map frame: +X front, +Y left/up.
    # V02: wheel odometry on the real mecanum platform under-reported travel,
    # so convert raw odometry into calibrated physical maze metres here.
    return body_x * float(scale_x), -body_y_right * float(scale_y)


def _relative_xy(
    pose: PoseTracker,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    config: Optional[Classwork8Config] = None,
) -> Tuple[Optional[float], Optional[float]]:
    x, y = pose.get_xy()
    if x is None or y is None:
        return None, None
    scale_x = 1.0 if config is None else float(config.odom_scale_x)
    scale_y = 1.0 if config is None else float(config.odom_scale_y)
    return _map_xy_from_raw(
        float(x),
        float(y),
        start_x,
        start_y,
        start_yaw_deg,
        scale_x,
        scale_y,
    )


def _direction_angle_rad(direction: int) -> float:
    return {
        0: 0.0,
        1: -math.pi / 2.0,
        2: math.pi,
        3: math.pi / 2.0,
    }[int(direction) % 4]


def _neighbor(node: Tuple[int, int], direction: int) -> Tuple[int, int]:
    dx, dy = DIR_VEC_MAP[int(direction) % 4]
    return node[0] + dx, node[1] + dy


def _set_edge_state(
    edge_states: Dict[Tuple[int, int, int], str],
    cell: Tuple[int, int],
    direction: int,
    state: str,
) -> None:
    """Record a WALL/OPEN edge on both adjacent logical cells."""
    direction %= 4
    state = str(state).upper()
    edge_states[(int(cell[0]), int(cell[1]), direction)] = state
    other = _neighbor(cell, direction)
    edge_states[(int(other[0]), int(other[1]), (direction + 2) % 4)] = state


def _direction_to(
    current: Tuple[int, int],
    target: Tuple[int, int],
) -> Optional[int]:
    delta = (target[0] - current[0], target[1] - current[1])
    for direction, vec in DIR_VEC_MAP.items():
        if vec == delta:
            return direction
    return None


def _inside_working_canvas(
    node: Tuple[int, int],
    config: Classwork8Config,
) -> bool:
    x = node[0] * config.cell_size_m
    y = node[1] * config.cell_size_m
    margin = config.cell_size_m / 2.0
    return (
        abs(x) <= config.map_width_m / 2.0 - margin
        and abs(y) <= config.map_height_m / 2.0 - margin
    )


def _update_tof_ray(
    grid: OccupancyGrid,
    config: Classwork8Config,
    rel_x: float,
    rel_y: float,
    direction: int,
    distance_cm: Optional[float],
) -> None:
    if distance_cm is None or distance_cm < config.mapping_min_cm:
        return

    angle = _direction_angle_rad(direction)
    origin_x = rel_x + math.cos(angle) * config.tof_forward_offset_m
    origin_y = rel_y + math.sin(angle) * config.tof_forward_offset_m
    hit = distance_cm < config.tof_max_mapping_cm - 1.0

    grid.update_ray(
        origin_x,
        origin_y,
        angle,
        float(distance_cm) / 100.0,
        max_range_m=config.tof_max_mapping_cm / 100.0,
        hit=hit,
    )


def _point_gimbal(
    gimbal,
    sensors: ToFOnlySensorManager,
    tracker: GimbalTracker,
    direction: int,
    config: Classwork8Config,
    stop_event: Optional[threading.Event],
    _allow_endpoint_retry: bool = True,
) -> bool:
    """Staged, single-axis mapping scan: level pitch -> yaw only -> level pitch.

    The real stationary hardware test showed up to 24 degrees transient pitch
    error during the old simultaneous pitch/yaw controller even though every
    final endpoint was level.  Never send nonzero pitch and yaw simultaneously.
    Pitch can transiently move DURING the yaw sweep without a ToF reading.
    Report the excursion but only accept a ray after the gimbal has stopped
    and BOTH axes have been brought back to their horizontal scan targets.
    """
    target_yaw = float(config.gimbal_yaw_for_direction(direction))
    target_pitch = float(config.gimbal_scan_pitch_deg)
    def _stopped() -> bool:
        return stop_event is not None and stop_event.is_set()

    def _stop_axes() -> None:
        gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)

    def _level_pitch(stage: str) -> bool:
        """Pitch moves only while yaw_speed is exactly zero."""
        stable = 0
        pitch_deadline = time.monotonic() + float(config.gimbal_turn_timeout_sec)
        while time.monotonic() < pitch_deadline:
            if _stopped():
                _stop_axes()
                return False

            pitch, yaw = tracker.get_angles()
            if pitch is None or yaw is None:
                _stop_axes()
                time.sleep(0.03)
                continue

            error = target_pitch - float(pitch)
            if abs(error) <= float(config.gimbal_pitch_tolerance_deg):
                _stop_axes()
                stable += 1
                if stable >= int(config.gimbal_stable_samples):
                    return True
            else:
                stable = 0
                speed = max(
                    float(config.gimbal_pitch_min_speed_dps),
                    min(
                        float(config.gimbal_pitch_max_speed_dps),
                        abs(error) * float(config.gimbal_pitch_kp),
                    ),
                )
                gimbal.drive_speed(
                    pitch_speed=(
                        math.copysign(speed, error)
                        * float(config.gimbal_pitch_drive_sign)
                    ),
                    yaw_speed=0.0,
                )
            time.sleep(0.03)

        _stop_axes()
        print(
            "[GIMBAL_FAIL] {} pitch timeout: target {:+.1f} current {}.".format(
                stage,
                target_pitch,
                "---" if tracker.get_pitch() is None
                else "{:+.1f}".format(float(tracker.get_pitch())),
            ),
            flush=True,
        )
        return False

    if _stopped():
        _stop_axes()
        return False

    # Correct any existing tilt before beginning the horizontal sweep.
    if not _level_pitch("PRE_YAW"):
        return False

    # Do not use the shortest wrapped angle at +/-180: these are mechanical
    # absolute yaw coordinates. Scan order already avoids direct 180 -> -90.
    yaw_stable = 0
    max_pitch_during_yaw = 0.0
    started_yaw = time.monotonic()
    yaw_deadline = started_yaw + float(config.gimbal_turn_timeout_sec)

    while time.monotonic() < yaw_deadline:
        if _stopped():
            _stop_axes()
            return False

        pitch, yaw = tracker.get_angles()
        if pitch is None or yaw is None:
            _stop_axes()
            time.sleep(0.03)
            continue

        pitch_error = abs(float(pitch) - target_pitch)
        max_pitch_during_yaw = max(max_pitch_during_yaw, pitch_error)

        # This is a TRANSIENT diagnostic, not a mapping failure: the robot
        # is stationary and no ToF ray is sampled during the yaw movement.
        # The final level/pitch checks below still reject an unlevel ray.
        # An earlier field run aborted at 6.1 deg while the endpoint was
        # about to be reached; that premature abort produced targets.json=0.
        yaw_error = target_yaw - float(yaw)
        if abs(yaw_error) <= float(config.gimbal_tolerance_deg):
            _stop_axes()
            yaw_stable += 1
            if yaw_stable >= int(config.gimbal_stable_samples):
                break
        else:
            yaw_stable = 0
            speed = max(
                float(config.gimbal_min_yaw_speed_dps),
                min(
                    float(config.gimbal_yaw_speed_dps),
                    abs(yaw_error) * float(config.gimbal_yaw_kp),
                ),
            )
            gimbal.drive_speed(
                pitch_speed=0.0,
                yaw_speed=math.copysign(speed, yaw_error),
            )

        time.sleep(0.03)
    else:
        _stop_axes()
        print(
            "[GIMBAL_FAIL] Yaw timeout at {}. Measured yaw={}.".format(
                DIR_NAME[int(direction) % 4],
                tracker.get_yaw(),
            ),
            flush=True,
        )
        return False

    _stop_axes()
    if not _sleep_interruptible(0.12, stop_event):
        return False

    # Only after yaw is stopped may pitch be corrected again. This also
    # accounts for any small physical/firmware coupling under the guard.
    if not _level_pitch("POST_YAW"):
        return False

    _stop_axes()
    if not _sleep_interruptible(config.gimbal_settle_sec, stop_event):
        return False

    final_pitch, final_yaw = tracker.get_angles()
    if (
        final_pitch is None
        or final_yaw is None
        or abs(target_pitch - float(final_pitch))
        > float(config.gimbal_pitch_tolerance_deg)
        or abs(target_yaw - float(final_yaw))
        > float(config.gimbal_tolerance_deg)
    ):
        if _allow_endpoint_retry and not _stopped():
            print("[GIMBAL] Final endpoint out of tolerance; one bounded retry.", flush=True)
            return _point_gimbal(
                gimbal, sensors, tracker, direction, config, stop_event,
                _allow_endpoint_retry=False,
            )
        print(
            "[GIMBAL] Final orientation unstable: target pitch={:+.1f}, "
            "yaw={:+.1f}; actual pitch={}, yaw={}.".format(
                target_pitch,
                target_yaw,
                "---" if final_pitch is None else "{:+.1f}".format(float(final_pitch)),
                "---" if final_yaw is None else "{:+.1f}".format(float(final_yaw)),
            ),
            flush=True,
        )
        return False

    samples = tracker.pitch_samples_since(started_yaw)
    if max_pitch_during_yaw > float(config.gimbal_yaw_pitch_guard_deg):
        print(
            "[GIMBAL] TRANSIENT_PITCH_WARNING {}: peak={:.1f} deg while yaw "
            "was turning. Final pitch/yaw are verified level; no ToF was "
            "sampled during the sweep.".format(
                DIR_NAME[int(direction) % 4], max_pitch_during_yaw
            ),
            flush=True,
        )
    print(
        "[GIMBAL] {} yaw-only sweep finished; peak pitch error={:.1f} deg "
        "({} feedback samples).".format(
            DIR_NAME[int(direction) % 4],
            max_pitch_during_yaw,
            len(samples),
        ),
        flush=True,
    )
    sensors.reset_filters()
    return True


def _set_camera_observation_pitch(
    gimbal,
    tracker: GimbalTracker,
    config: Classwork8Config,
    target_pitch: float,
    stop_event: Optional[threading.Event],
) -> bool:
    """Change pitch while chassis is stopped, never commanding yaw at once."""
    desired = max(
        float(config.target_camera_pitch_min_deg),
        min(float(config.target_camera_pitch_max_deg), float(target_pitch)),
    )
    deadline = time.monotonic() + float(config.target_camera_pitch_timeout_sec)
    stable = 0

    try:
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                return False

            pitch, yaw = tracker.get_angles()
            if pitch is None or yaw is None:
                time.sleep(0.025)
                continue

            error = desired - float(pitch)
            if abs(error) <= float(config.target_camera_pitch_tolerance_deg):
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                stable += 1
                if stable >= int(config.gimbal_stable_samples):
                    if not _sleep_interruptible(
                        config.target_camera_settle_sec, stop_event
                    ):
                        return False
                    final = tracker.get_pitch()
                    return (
                        final is not None
                        and abs(desired - float(final))
                        <= float(config.target_camera_pitch_tolerance_deg)
                    )
            else:
                stable = 0
                speed = max(
                    float(config.gimbal_pitch_min_speed_dps),
                    min(
                        float(config.gimbal_pitch_max_speed_dps),
                        abs(error) * float(config.gimbal_pitch_kp),
                    ),
                )
                gimbal.drive_speed(
                    pitch_speed=(
                        math.copysign(speed, error)
                        * float(config.gimbal_pitch_drive_sign)
                    ),
                    yaw_speed=0.0,
                )
            time.sleep(0.03)

        print(
            "[CAMERA] Observation pitch timeout: target={:+.1f}, measured={}".format(
                desired, tracker.get_pitch()
            ),
            flush=True,
        )
        return False
    finally:
        gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)


def _sample_tof(
    sensors: ToFOnlySensorManager,
    config: Classwork8Config,
    stop_event: Optional[threading.Event],
) -> Optional[float]:
    values: List[float] = []
    deadline = time.monotonic() + 1.5

    while len(values) < config.scan_samples and time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return None
        value = sensors.get_front_cm()
        if value is not None:
            values.append(float(value))
        time.sleep(config.scan_sample_interval_sec)

    if not values:
        return None
    return float(statistics.median(values))



def _wait_for_fresh_tof(
    sensors: ToFOnlySensorManager,
    timeout_sec: float,
    stop_event: Optional[threading.Event],
) -> Optional[float]:
    """Wait for a fresh ToF sample after a gimbal/filter reset."""
    deadline = time.monotonic() + max(0.05, float(timeout_sec))
    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return None
        value = sensors.get_front_cm()
        if value is not None:
            return float(value)
        time.sleep(0.02)
    return None


def _fixed_heading_control_v02(
    config: Classwork8Config,
    target_yaw_deg: float,
    current_yaw_deg: Optional[float],
    x_cmd: float,
    y_cmd: float,
    mode: str,
) -> Tuple[float, float, float, str, Optional[float]]:
    """Continuous yaw P steering without translation pause or speed scaling."""
    if not config.heading_hold_enabled or current_yaw_deg is None:
        return x_cmd, y_cmd, 0.0, mode, None
    error = normalize_angle_deg(float(target_yaw_deg) - float(current_yaw_deg))
    if abs(error) <= float(config.heading_deadband_deg):
        return x_cmd, y_cmd, 0.0, mode, error
    max_z = float(config.heading_max_z_dps)
    z_cmd = error * float(config.heading_kp_z) / float(config.heading_drive_sign)
    z_cmd = max(-max_z, min(max_z, z_cmd))
    return x_cmd, y_cmd, z_cmd, mode, error

def _should_reuse_scan(
    current_cell: Tuple[int, int],
    scanned_cells: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    skip_enabled: bool,
    rescan_requested: bool,
) -> bool:
    """Skip only a previously completed 4-way scan with known topology.

    UNKNOWN edges, first visits, or explicit operator rescans require a
    physical scan. This decision does not disable fresh travel-direction ToF.
    """
    return bool(
        skip_enabled
        and current_cell in scanned_cells
        and not rescan_requested
        and all(
            edge_states.get(
                (current_cell[0], current_cell[1], direction)
            ) in ("OPEN", "WALL")
            for direction in range(4)
        )
    )


def _scan_four_directions(
    gimbal,
    pose: PoseTracker,
    sensors: ToFOnlySensorManager,
    gimbal_tracker: GimbalTracker,
    grid: OccupancyGrid,
    recorder: RunRecorder,
    config: Classwork8Config,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    stop_event: Optional[threading.Event],
    publish_state: Callable[..., None],
    current_cell: Tuple[int, int],
    moves: int,
    known_cells: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    traversed_edges: Set[Tuple[Tuple[int, int], Tuple[int, int]]],
    camera_service: Optional[CameraService],
    target_detector: Optional[TargetDetector],
    target_registry: TargetRegistry,
    target_debug_holder: List[object],
    survey_bridge: LiveSurveyBridge,
) -> Optional[Tuple[Dict[int, Optional[float]], Set[int]]]:
    # Sweep in the direction that is closest to the current gimbal endpoint.
    # This avoids a large BACK(+180) -> LEFT(-90) wrap across the +250 deg
    # mechanical limit.  Each sweep segment is then about 90 degrees.
    current_gimbal_yaw = gimbal_tracker.get_yaw()
    if current_gimbal_yaw is not None and float(current_gimbal_yaw) > 45.0:
        order = [2, 1, 0, 3]  # BACK -> RIGHT -> FRONT -> LEFT
    else:
        order = [3, 0, 1, 2]  # LEFT -> FRONT -> RIGHT -> BACK
    ranges: Dict[int, Optional[float]] = {}
    open_dirs: Set[int] = set()

    # Caller sends x=y=z=0 before any stationary scan.
    for direction in order:
        if stop_event is not None and stop_event.is_set():
            return None

        _heading_snapshot(
            "PRE_GIMBAL_{}_{}".format(current_cell, DIR_NAME[direction]),
            pose, gimbal_tracker, float(start_yaw_deg),
        )
        print(
            "[SCAN] Pointing Gimbal {} (target {:+.0f} deg)...".format(
                DIR_NAME[direction],
                config.gimbal_yaw_for_direction(direction),
            ),
            flush=True,
        )

        publish_state(
            status="Scanning {}".format(DIR_NAME[direction]),
            logical_cell=current_cell,
            gimbal_direction=direction,
            tof_cm=sensors.get_front_cm(),
            moves=moves,
            force=True,
        )

        if not _point_gimbal(
            gimbal,
            sensors,
            gimbal_tracker,
            direction,
            config,
            stop_event,
        ):
            print(
                "[SCAN] Gimbal FAILED for {}. measured_yaw={}".format(
                    DIR_NAME[direction],
                    "---" if gimbal_tracker.get_yaw() is None
                    else "{:+.1f}".format(float(gimbal_tracker.get_yaw())),
                ),
                flush=True,
            )
            return None

        _heading_snapshot(
            "POST_GIMBAL_{}_{}".format(current_cell, DIR_NAME[direction]),
            pose, gimbal_tracker, float(start_yaw_deg),
        )
        print(
            "[SCAN] Gimbal {} ready: yaw={:+.1f} pitch={:+.1f} deg.".format(
                DIR_NAME[direction],
                float(gimbal_tracker.get_yaw()),
                float(gimbal_tracker.get_pitch()),
            ),
            flush=True,
        )

        distance_cm = _sample_tof(sensors, config, stop_event)

        # Do not classify an edge using an upward/downward-tilted ToF ray.
        # Check again AFTER the sample interval, not only when yaw settled.
        pitch_after_sample = gimbal_tracker.get_pitch()
        if (
            pitch_after_sample is None
            or abs(
                float(pitch_after_sample) - float(config.gimbal_scan_pitch_deg)
            ) > float(config.gimbal_pitch_tolerance_deg)
        ):
            print(
                "[SCAN] {} pitch drift after ToF sampling (pitch={}); "
                "stop and retry orientation once.".format(
                    DIR_NAME[direction],
                    "---" if pitch_after_sample is None
                    else "{:+.1f}".format(float(pitch_after_sample)),
                ),
                flush=True,
            )
            if not _point_gimbal(
                gimbal, sensors, gimbal_tracker, direction, config, stop_event
            ):
                return None
            distance_cm = _sample_tof(sensors, config, stop_event)
            final_pitch = gimbal_tracker.get_pitch()
            if (
                final_pitch is None
                or abs(
                    float(final_pitch) - float(config.gimbal_scan_pitch_deg)
                ) > float(config.gimbal_pitch_tolerance_deg)
            ):
                print(
                    "[SCAN] Unsafe pitch after retry; refusing ToF/map update.",
                    flush=True,
                )
                return None

        # V02: readings between a definite near wall and the normal OPEN
        # threshold are ambiguous.  A foam edge / floor reflection can create
        # one short median even when the branch is physically open.  Re-sample
        # at the same gimbal angle and keep the larger robust median.  If this
        # turns out to be a false-open remains a mapping risk: BASIC motion does not
        # stop the chassis from a ToF reading.
        if (
            distance_cm is not None
            and float(config.scan_hard_wall_cm) < float(distance_cm)
            < float(config.tof_open_cm)
            and int(config.scan_ambiguous_retries) > 0
        ):
            retry_values = [float(distance_cm)]
            for _ in range(int(config.scan_ambiguous_retries)):
                sensors.reset_filters()
                if not _sleep_interruptible(
                    config.scan_ambiguous_retry_settle_sec,
                    stop_event,
                ):
                    return None
                retry = _sample_tof(sensors, config, stop_event)
                if retry is not None:
                    retry_values.append(float(retry))
            distance_cm = max(retry_values)
            print(
                "[SCAN] {} ambiguous -> retry candidates {} -> {:.1f} cm".format(
                    DIR_NAME[direction],
                    [round(v, 1) for v in retry_values],
                    float(distance_cm),
                ),
                flush=True,
            )

        # One final pitch guard also covers the ambiguous-range retry period.
        # Never commit WALL/OPEN topology from a ray that has tilted away from
        # its horizontal scan plane.
        final_scan_pitch = gimbal_tracker.get_pitch()
        if (
            final_scan_pitch is None
            or abs(
                float(final_scan_pitch) - float(config.gimbal_scan_pitch_deg)
            ) > float(config.gimbal_pitch_tolerance_deg)
        ):
            print(
                "[SCAN] {} pitch changed during sampling/retry; aborting scan "
                "without updating topology.".format(DIR_NAME[direction]),
                flush=True,
            )
            return None

        ranges[direction] = distance_cm
        print(
            "[SCAN] {} ToF = {} cm".format(
                DIR_NAME[direction],
                "---" if distance_cm is None else "{:.1f}".format(distance_cm),
            ),
            flush=True,
        )

        # Camera targets are physically lower than the horizontal ToF ray.
        # Keep the proven mapping range above, then (only while chassis is
        # stopped) tip the camera down to its independently configured angle.
        # Restore horizontal ToF before any next ray or chassis movement.
        near_wall = (
            distance_cm is not None
            and float(distance_cm) < float(config.tof_open_cm)
        )
        survey_this_direction = (
            camera_service is not None
            and camera_service.running
            and target_detector is not None
            and (
                near_wall
                or bool(config.target_survey_open_directions)
            )
        )

        if survey_this_direction:
            # Explicit intentional pause: target observation, NOT motion safety.
            recorder.event(
                time.monotonic(), "TARGET_SCAN_PAUSE",
                "stationary camera target observation",
                logical_node=current_cell, direction=DIR_NAME[direction],
            )
            print(
                "[TARGET_SCAN_PAUSE] stationary survey {}".format(
                    DIR_NAME[direction]
                ), flush=True,
            )
            selected_pitch = float(survey_bridge.get_pitch())
            survey_bridge.set_status(
                "Survey {} at pitch {:+.1f} deg (robot stopped)".format(
                    DIR_NAME[direction], selected_pitch
                )
            )
            publish_state(
                status="Camera surveying {} at {:+.1f} deg".format(
                    DIR_NAME[direction], selected_pitch
                ),
                logical_cell=current_cell,
                gimbal_direction=direction,
                tof_cm=distance_cm,
                moves=moves,
                force=True,
            )

            verified_targets = []
            restore_ok = False
            camera_position_ok = _set_camera_observation_pitch(
                gimbal,
                gimbal_tracker,
                config,
                selected_pitch,
                stop_event,
            )
            try:
                if camera_position_ok:
                    # Do not verify cached frames from the previous horizontal
                    # ToF viewpoint: the low sign may only enter the image
                    # after the new camera pitch has settled.
                    survey_frame_epoch = time.monotonic()
                    verified_targets, target_debug = target_detector.verify_latest(
                        camera_service,
                        not_before=survey_frame_epoch,
                    )
                    target_debug_holder[0] = target_debug

                    measured_pitch = gimbal_tracker.get_pitch()
                    if (
                        measured_pitch is None
                        or abs(float(measured_pitch) - selected_pitch)
                        > float(config.target_camera_pitch_tolerance_deg)
                    ):
                        print(
                            "[TARGET] Discarding observations: camera pitch "
                            "changed while sampling.",
                            flush=True,
                        )
                        verified_targets = []

                    for verified in verified_targets:
                        saved_target = target_registry.add_verified(
                            verified,
                            current_cell,
                            direction,
                            distance_cm,
                            range_confirmed_wall=near_wall,
                            camera_pitch_deg=selected_pitch,
                        )
                        recorder.event(
                            time.monotonic(),
                            "TARGET_OBSERVATION",
                            "{} {} from {} {}".format(
                                verified.detection.color.upper(),
                                verified.detection.shape.upper(),
                                current_cell,
                                DIR_NAME[direction],
                            ),
                            logical_node=current_cell,
                            direction=DIR_NAME[direction],
                            target_id=saved_target["target_id"],
                            target_color=verified.detection.color,
                            target_shape=verified.detection.shape,
                            target_confidence=round(float(verified.confidence), 4),
                            camera_pitch_deg=selected_pitch,
                            range_confirmed_wall=near_wall,
                            tof_cm=distance_cm,
                            localization_status=saved_target["localization_status"],
                            sighting_cell_hint=saved_target.get("sighting_cell_hint"),
                        )
                        print(
                            "[TARGET] {} {} -> {} conf={:.2f} {} from {} {}".format(
                                verified.detection.color.upper(),
                                verified.detection.shape.upper(),
                                saved_target["target_id"],
                                float(verified.confidence),
                                "NEAR_WALL_ESTIMATE" if near_wall
                                else "SIGHTING_ONLY (distance unconfirmed)",
                                current_cell,
                                DIR_NAME[direction],
                            ),
                            flush=True,
                        )
                        if not near_wall:
                            print(
                                "[TARGET] {} is a distant sighting only. "
                                "Candidate cell {} along {}; do NOT use as "
                                "Round-2 target position until close revalidation.".format(
                                    saved_target["target_id"],
                                    saved_target.get("sighting_cell_hint"),
                                    DIR_NAME[direction],
                                ),
                                flush=True,
                            )
                else:
                    print(
                        "[TARGET] Camera pitch not reached at {}; survey skipped.".format(
                            DIR_NAME[direction]
                        ),
                        flush=True,
                    )
            finally:
                # This restore is mandatory. The ToF ray is not safe for
                # navigation or topology if the camera remains angled down.
                restore_ok = _point_gimbal(
                    gimbal,
                    sensors,
                    gimbal_tracker,
                    direction,
                    config,
                    stop_event,
                )
                survey_bridge.set_status(
                    "Live preview; ToF horizontal restored"
                    if restore_ok else "ERROR: cannot restore horizontal ToF"
                )

            if not restore_ok:
                print(
                    "[SCAN] Camera pitch restore FAILED. No further mapping/move.",
                    flush=True,
                )
                return None

            print(
                "[TARGET_SCAN_RESUME] survey completed; continuing scan/navigation.",
                flush=True,
            )
            live_preview_status = survey_bridge.latest_preview()
            recorder.event(
                time.monotonic(),
                "TARGET_SURVEY",
                "{} at {:+.1f} deg: {} verified / {} current candidates, "
                "wall_range_confirmed={}".format(
                    DIR_NAME[direction],
                    selected_pitch,
                    len(verified_targets),
                    live_preview_status["candidate_count"],
                    near_wall,
                ),
                logical_node=current_cell,
                direction=DIR_NAME[direction],
                verified_targets=len(verified_targets),
                live_candidate_count=live_preview_status["candidate_count"],
                camera_pitch_deg=selected_pitch,
                range_confirmed_wall=near_wall,
                tof_cm=distance_cm,
            )
            print(
                "[TARGET_SURVEY] {} pitch={:+.1f} candidates={} verified={} wall={}".format(
                    DIR_NAME[direction],
                    selected_pitch,
                    live_preview_status["candidate_count"],
                    len(verified_targets),
                    near_wall,
                ),
                flush=True,
            )
        elif camera_service is not None and camera_service.running:
            recorder.event(
                time.monotonic(),
                "TARGET_SURVEY_SKIPPED",
                "{}: open direction not enabled for target survey".format(
                    DIR_NAME[direction]
                ),
                logical_node=current_cell,
                direction=DIR_NAME[direction],
                tof_cm=distance_cm,
            )

        rel_x, rel_y = _relative_xy(
            pose, start_x, start_y, start_yaw_deg, config
        )
        yaw = pose.get_yaw()

        if rel_x is not None and rel_y is not None:
            _update_tof_ray(
                grid,
                config,
                rel_x,
                rel_y,
                direction,
                distance_cm,
            )

        edge_key = _canonical_edge(current_cell, direction)

        if edge_key in traversed_edges:
            # Physical traversal is stronger evidence than a later noisy scan.
            open_dirs.add(direction)
            _set_edge_state(edge_states, current_cell, direction, "OPEN")
            known_cells.add(_neighbor(current_cell, direction))
        elif distance_cm is not None:
            if distance_cm >= config.tof_open_cm:
                open_dirs.add(direction)
                _set_edge_state(edge_states, current_cell, direction, "OPEN")
                known_cells.add(_neighbor(current_cell, direction))
            else:
                _set_edge_state(edge_states, current_cell, direction, "WALL")

        recorder.record_sample(
            time.monotonic(),
            rel_x,
            rel_y,
            yaw,
            direction,
            distance_cm,
            None,
            None,
            None,
            None,
            "GIMBAL_SCAN_{}".format(DIR_NAME[direction]),
        )

        publish_state(
            status="Scanned {}".format(DIR_NAME[direction]),
            logical_cell=current_cell,
            gimbal_direction=direction,
            tof_cm=distance_cm,
            moves=moves,
            force=True,
        )

    return ranges, open_dirs



def _heading_error(target: float, actual: float) -> float:
    return normalize_angle_deg(float(target) - float(actual))


def _heading_snapshot(phase: str, pose: PoseTracker, tracker: GimbalTracker,
                      reference: float) -> Optional[float]:
    """Compare chassis attitude with relative/ground gimbal yaw across phases."""
    actual = pose.get_yaw()
    age = pose.attitude_age_sec() if hasattr(pose, "attitude_age_sec") else None
    rel, ground = tracker.get_yaws()
    error = None if actual is None else _heading_error(reference, actual)
    proxy = None if rel is None or ground is None else normalize_angle_deg(ground - rel)
    def fmt(v):
        return "---" if v is None else "{:+.2f}".format(float(v))
    print(
        "[HEADING_TRACE] phase={} chassis={} ref={} diff={} "
        "yaw_age_sec={} gimbal_relative={} gimbal_ground={} "
        "ground_minus_relative={}".format(
            phase, fmt(actual), fmt(reference), fmt(error), fmt(age),
            fmt(rel), fmt(ground), fmt(proxy)
        ), flush=True,
    )
    return actual


def _align_chassis_after_scan(chassis, pose: PoseTracker, config: Classwork8Config,
                              target: float, stop_event: Optional[threading.Event]
                              ) -> Tuple[bool, str]:
    """Single bounded, stationary heading correction before departing a cell."""
    stop_chassis(chassis)
    actual = pose.get_yaw()
    if actual is None:
        return False, "HEADING_FEEDBACK_LOST"
    initial_error = abs(_heading_error(target, actual))
    print("[HEADING_ALIGN] target={:+.2f} actual={:+.2f} diff={:+.2f}".format(
        target, actual, _heading_error(target, actual)), flush=True)
    if config.yaw_isolation_mode or not config.heading_hold_enabled:
        return True, "YAW_ISOLATION" if config.yaw_isolation_mode else "HEADING_HOLD_DISABLED"
    if initial_error <= float(config.heading_align_tolerance_deg):
        return True, "ALREADY_ALIGNED"
    if initial_error > float(config.heading_align_max_error_deg):
        return False, "HEADING_LARGE_DRIFT"
    started = time.monotonic()
    deadline = started + float(config.heading_align_timeout_sec)
    stable = 0
    try:
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                return False, "USER_STOP"
            actual = pose.get_yaw()
            if actual is None:
                return False, "HEADING_FEEDBACK_LOST"
            error = _heading_error(target, actual)
            if abs(error) <= float(config.heading_align_tolerance_deg):
                stop_chassis(chassis)
                stable += 1
                if stable >= 3:
                    print("[HEADING_ALIGN] OK final={:+.2f} error={:+.2f}".format(
                        actual, error), flush=True)
                    return True, "ALIGNED"
            else:
                stable = 0
                # A wrong z-to-attitude sign must not induce runaway rotation.
                if time.monotonic() - started > 0.55 and abs(error) > initial_error + 1.0:
                    print("[HEADING_FAIL] SIGN_MISMATCH: {:.2f} -> {:.2f}; "
                          "inspect heading_drive_sign.".format(
                        initial_error, abs(error)), flush=True)
                    return False, "HEADING_SIGN_MISMATCH"
                z = max(-float(config.heading_align_max_z_dps),
                        min(float(config.heading_align_max_z_dps),
                            error * float(config.heading_kp_z)
                            / float(config.heading_drive_sign)))
                chassis.drive_speed(x=0.0, y=0.0, z=z,
                                    timeout=config.drive_timeout_sec)
            time.sleep(0.05)
        print("[HEADING_FAIL] ALIGN_TIMEOUT actual={} target={}".format(
            pose.get_yaw(), target), flush=True)
        return False, "HEADING_ALIGN_TIMEOUT"
    finally:
        stop_chassis(chassis)


# A live moving yaw fault must abort before the existing 0.65-second
# divergence probe can send progressively larger turning commands.
# Field run 2026-09-28 reached 5.51 deg then 40.87 deg while moving.
V05_MOVING_YAW_ABORT_DEG = 4.0


def _moving_heading_over_limit(
    config: Classwork8Config,
    target_yaw_deg: float,
    actual_yaw_deg: Optional[float],
) -> bool:
    if (
        not config.heading_hold_enabled
        or config.yaw_isolation_mode
        or actual_yaw_deg is None
    ):
        return False
    return abs(_heading_error(target_yaw_deg, actual_yaw_deg)) > V05_MOVING_YAW_ABORT_DEG


def _maintain_wall_clearance_checkpoint(
    chassis, gimbal, pose: PoseTracker, sensors: ToFOnlySensorManager,
    tracker: GimbalTracker, config: Classwork8Config,
    ranges: Dict[int, Optional[float]], raw_start_x: float,
    raw_start_y: float, raw_start_yaw: float,
    stop_event: Optional[threading.Event],
) -> Tuple[bool, Optional[str]]:
    """One small, physically verified, zero-yaw move AWAY from a near wall.

    The initial four-side ToF ranges are fresh from this cell's full scan.
    One gimbal ToF cannot monitor four sides at once: point it at the near
    wall and continuously observe that wall while adjusting. The opposite
    scan range budgets how far we can go; never invent missing clearance.
    Return (physically_moved, fatal_reason). Caller rescans ALL FOUR sides
    after any move before committing current-cell scan results.
    """
    if not config.wall_clearance_enabled or config.yaw_isolation_mode:
        return False, None
    plan = choose_clearance_plan(ranges, config)
    if plan is None:
        print("[CLEARANCE] No safe correction required/possible in this cell.",
              flush=True)
        return False, None
    if stop_event is not None and stop_event.is_set():
        return False, "USER_STOP"
    initial_yaw = pose.get_yaw()
    age = pose.attitude_age_sec()
    if (initial_yaw is None or age is None or age > 0.3 or
            abs(_heading_error(raw_start_yaw, initial_yaw)) > 2.0):
        print("[CLEARANCE] SKIP: heading not aligned/fresh for a body-frame shift.",
              flush=True)
        return False, None

    stop_chassis(chassis)
    if not _point_gimbal(
        gimbal, sensors, tracker, plan.wall_direction, config, stop_event
    ):
        return False, "CLEARANCE_GIMBAL_UNAVAILABLE"
    # A stopped yaw sweep changes which wall the ToF measures. Reset the
    # median filter and require fresh data on the chosen wall before moving.
    sensors.reset_filters()
    fresh = _wait_for_fresh_tof(sensors, 1.0, stop_event)
    if fresh is None:
        print("[CLEARANCE] SKIP: fresh wall-distance feedback unavailable.",
              flush=True)
        return False, None
    target = clearance_target(config, plan.wall_direction)
    tol = float(config.wall_clearance_deadband_cm)
    if fresh >= target - tol:
        print("[CLEARANCE] SKIP: fresh {} range {:.1f} cm already acceptable.".format(
            DIR_NAME[plan.wall_direction], fresh), flush=True)
        return False, None
    if (abs(float(fresh) - plan.measured_cm) > 8.0 or
            float(fresh) < float(config.mapping_min_cm)):
        print("[CLEARANCE] SKIP: scanned and fresh wall ranges disagree.",
              flush=True)
        return False, None
    opposite_budget = (
        plan.opposite_cm
        - clearance_target(config, plan.away_direction) - tol
    )
    limit_cm = min(
        target - float(fresh),
        float(config.wall_clearance_max_step_cm),
        opposite_budget,
    )
    if limit_cm <= tol:
        print("[CLEARANCE] SKIP: opposing wall leaves no safe room.", flush=True)
        return False, None

    xy = pose.get_xy()
    if xy[0] is None or xy[1] is None:
        return False, "CLEARANCE_ODOMETRY_MISSING"
    initial_map = _map_xy_from_raw(
        float(xy[0]), float(xy[1]),
        raw_start_x, raw_start_y, raw_start_yaw,
        config.odom_scale_x, config.odom_scale_y,
    )
    unit_map = DIR_VEC_MAP[plan.away_direction]
    unit_body = DIR_VEC_DRIVE[plan.away_direction]
    speed = float(config.wall_clearance_speed_mps)
    started = time.monotonic()
    timeout = limit_cm / 100.0 / speed + 1.0
    last_progress = 0.0
    print(
        "[CLEARANCE] wall={} range={:.1f} target={:.1f} opposite={:.1f} "
        "away={} limit={:.1f}cm speed={:.3f} z=0".format(
            DIR_NAME[plan.wall_direction], fresh, target, plan.opposite_cm,
            DIR_NAME[plan.away_direction], limit_cm, speed,
        ), flush=True,
    )
    try:
        while time.monotonic() - started <= timeout:
            if stop_event is not None and stop_event.is_set():
                return last_progress >= 0.003, "USER_STOP"
            yaw = pose.get_yaw()
            yaw_age = pose.attitude_age_sec()
            if (yaw is None or yaw_age is None or yaw_age > 0.3 or
                    abs(_heading_error(raw_start_yaw, yaw)) > 2.0):
                return last_progress >= 0.003, "CLEARANCE_HEADING_GUARD"
            observed_pitch, observed_yaw = tracker.get_angles()
            if (observed_pitch is None or observed_yaw is None or
                    abs(float(observed_pitch) - config.gimbal_scan_pitch_deg)
                    > config.gimbal_pitch_tolerance_deg or
                    abs(_heading_error(
                        config.gimbal_yaw_for_direction(plan.wall_direction),
                        observed_yaw,
                    )) > config.gimbal_tolerance_deg):
                return last_progress >= 0.003, "CLEARANCE_GIMBAL_MOVED"
            stamp = sensors.tof_last_update
            now = time.monotonic()
            if stamp is None or now - stamp > 0.35:
                return last_progress >= 0.003, "CLEARANCE_TOF_STALE"
            live = sensors.get_front_cm()
            if live is None or float(live) < float(fresh) - 2.0:
                return last_progress >= 0.003, "CLEARANCE_RANGE_UNSAFE"
            xy = pose.get_xy()
            if xy[0] is None or xy[1] is None:
                return last_progress >= 0.003, "CLEARANCE_ODOMETRY_LOST"
            map_xy = _map_xy_from_raw(
                float(xy[0]), float(xy[1]),
                raw_start_x, raw_start_y, raw_start_yaw,
                config.odom_scale_x, config.odom_scale_y,
            )
            progress = (
                (map_xy[0] - initial_map[0]) * unit_map[0]
                + (map_xy[1] - initial_map[1]) * unit_map[1]
            )
            if progress < -0.005 or progress > limit_cm / 100.0 + 0.012:
                return progress >= 0.003, "CLEARANCE_ODOMETRY_DIRECTION"
            last_progress = max(0.0, progress)
            if live >= target - tol or progress >= limit_cm / 100.0:
                print(
                    "[CLEARANCE] STOP wall={} live={:.1f}cm shifted={:.3f}m "
                    "(bounded checkpoint adjustment)".format(
                        DIR_NAME[plan.wall_direction], live, progress,
                    ), flush=True,
                )
                return progress >= 0.003, None
            chassis.drive_speed(
                x=unit_body[0] * speed,
                y=unit_body[1] * speed,
                z=0.0,
                timeout=config.drive_timeout_sec,
            )
            if not _sleep_interruptible(0.04, stop_event):
                return last_progress >= 0.003, "USER_STOP"
        return last_progress >= 0.003, "CLEARANCE_MOTION_TIMEOUT"
    finally:
        # Cancel any pending SDK auto-zero speed timer before direct wheel stop.
        stop_chassis(chassis)


def _basic_motion_command(
    config: Classwork8Config,
    direction: int,
    target_yaw_deg: float,
    current_yaw_deg: Optional[float],
) -> Tuple[float, float, float, Optional[float]]:
    """Single longitudinal speed owner: no wall/ToF/cross-track speed inputs."""
    requested_speed = float(config.travel_speed_mps)
    ux, uy = DIR_VEC_DRIVE[int(direction) % 4]
    x_cmd = ux * requested_speed
    y_cmd = uy * requested_speed
    x_cmd, y_cmd, z_cmd, _mode, yaw_error = _fixed_heading_control_v02(
        config, target_yaw_deg, current_yaw_deg, x_cmd, y_cmd, "BASIC_MOVE"
    )
    if config.yaw_isolation_mode:
        z_cmd = 0.0  # Enforce at the sole translation-command producer.
    return x_cmd, y_cmd, z_cmd, yaw_error


def _drive_one_cell(
    chassis,
    gimbal,
    pose: PoseTracker,
    heading: HeadingManager,
    sensors: ToFOnlySensorManager,
    gimbal_tracker: GimbalTracker,
    vision: Optional[CorridorVision],
    grid: OccupancyGrid,
    recorder: RunRecorder,
    config: Classwork8Config,
    start_x: float,
    start_y: float,
    start_yaw_deg: float,
    direction: int,
    current_cell: Tuple[int, int],
    target_cell: Tuple[int, int],
    scan_ranges: Optional[Dict[int, Optional[float]]],
    wall_sides: Set[int],
    moves: int,
    stop_event: Optional[threading.Event],
    publish_state: Callable[..., None],
) -> Tuple[bool, str, float]:
    """BASIC movement: requested longitudinal speed plus continuous yaw steering.

    SLAM, camera/target detection, gimbal and ToF observation are retained.
    Historical wall/scan/vision inputs do not modify chassis speed. Only manual
    stop, normal cell completion, and fatal missing feedback stop this move.
    """
    direction %= 4
    _ = (heading, vision, scan_ranges, wall_sides)  # Legacy call compatibility.
    if not _point_gimbal(
        gimbal, sensors, gimbal_tracker, direction, config, stop_event
    ):
        stop_chassis(chassis)
        return False, "GIMBAL_UNAVAILABLE", 0.0

    x0, y0 = pose.get_xy()
    if x0 is None or y0 is None:
        stop_chassis(chassis)
        return False, "ODOMETRY_UNAVAILABLE", 0.0
    start_map_x, start_map_y = _map_xy_from_raw(
        float(x0), float(y0), start_x, start_y, start_yaw_deg,
        config.odom_scale_x, config.odom_scale_y,
    )
    target_map_x = float(target_cell[0]) * config.cell_size_m
    target_map_y = float(target_cell[1]) * config.cell_size_m
    max_abs_cross_track_m = 0.0
    max_abs_heading_error_deg = 0.0
    command_logged = False
    heading_probe = None
    last_heading_log = 0.0

    # No environment-triggered stop, slowdown, auto-recovery or motion watchdog.
    while True:
        if stop_event is not None and stop_event.is_set():
            stop_chassis(chassis)
            return False, "USER_STOP", 0.0

        raw_x, raw_y = pose.get_xy()
        yaw = pose.get_yaw()
        front_cm = sensors.get_front_cm()  # Observation only.
        if raw_x is None or raw_y is None:
            stop_chassis(chassis)
            return False, "ODOMETRY_LOST", 0.0
        if config.heading_hold_enabled and yaw is None:
            stop_chassis(chassis)
            return False, "HEADING_FEEDBACK_LOST", 0.0
        # An old but non-None attitude value is not feedback. Never steer
        # against a frozen yaw sample or draw conclusions from a stale probe.
        yaw_age = pose.attitude_age_sec() if hasattr(pose, "attitude_age_sec") else None
        if (config.heading_hold_enabled or config.yaw_isolation_mode) and (
            yaw_age is None or yaw_age > 1.5
        ):
            stop_chassis(chassis)
            print("[HEADING_FAIL] STALE_ATTITUDE age_sec={}".format(yaw_age), flush=True)
            return False, "HEADING_FEEDBACK_STALE", 0.0

        rel_x, rel_y = _map_xy_from_raw(
            float(raw_x), float(raw_y), start_x, start_y, start_yaw_deg,
            config.odom_scale_x, config.odom_scale_y,
        )
        moved = math.hypot(rel_x - start_map_x, rel_y - start_map_y)
        if direction == 0:
            remaining = target_map_x - rel_x
            cross_track = rel_y - target_map_y
        elif direction == 1:
            remaining = rel_y - target_map_y
            cross_track = rel_x - target_map_x
        elif direction == 2:
            remaining = rel_x - target_map_x
            cross_track = rel_y - target_map_y
        else:
            remaining = target_map_y - rel_y
            cross_track = rel_x - target_map_x
        max_abs_cross_track_m = max(max_abs_cross_track_m, abs(cross_track))
        if yaw is not None:
            max_abs_heading_error_deg = max(
                max_abs_heading_error_deg,
                abs(normalize_angle_deg(float(start_yaw_deg) - float(yaw))),
            )
        # Abort a large yaw departure before producing another nonzero
        # chassis command. The original delayed divergence probe did not
        # fire until the field run had already rotated >40 degrees.
        if _moving_heading_over_limit(config, start_yaw_deg, yaw):
            current_error = _heading_error(start_yaw_deg, yaw)
            stop_chassis(chassis)
            print(
                "[HEADING_FAIL] MOVING_YAW_LIMIT reference={:+.2f} "
                "actual={:+.2f} error={:+.2f} limit={:.1f}; "
                "four-wheel zero stop acknowledged".format(
                    float(start_yaw_deg), float(yaw), float(current_error),
                    V05_MOVING_YAW_ABORT_DEG,
                ), flush=True,
            )
            return False, "MOVING_YAW_LIMIT", moved
        # Validate observation geometry for mapping ONLY. Incorrect gimbal
        # pitch/yaw must not corrupt SLAM, but cannot alter chassis speed.
        sensor_pitch = gimbal_tracker.get_pitch()
        sensor_yaw = gimbal_tracker.get_yaw()
        if (
            sensor_pitch is not None
            and sensor_yaw is not None
            and abs(
                float(sensor_pitch) - float(config.gimbal_scan_pitch_deg)
            ) <= float(config.gimbal_pitch_tolerance_deg)
            and abs(normalize_angle_deg(
                float(sensor_yaw) - float(config.gimbal_yaw_for_direction(direction))
            )) <= float(config.gimbal_tolerance_deg)
        ):
            _update_tof_ray(grid, config, rel_x, rel_y, direction, front_cm)

        # Planned cell completion, not a wall/obstacle safety stop.
        if remaining <= float(config.step_tolerance_m):
            stop_chassis(chassis)
            recorder.record_sample(
                time.monotonic(), rel_x, rel_y, yaw, direction, front_cm,
                None, None, None, None, "CELL_COMPLETE",
            )
            recorder.event(
                time.monotonic(), "CELL_MOTION_QUALITY",
                "cross-track and yaw are observations only",
                logical_node=target_cell,
                peak_cross_track_m=round(max_abs_cross_track_m, 4),
                peak_heading_error_deg=round(max_abs_heading_error_deg, 3),
                midcell_side_checked=False,
            )
            publish_state(
                status="Reached cell {}".format(target_cell),
                logical_cell=target_cell, gimbal_direction=direction,
                tof_cm=front_cm, moves=moves + 1, force=True,
            )
            print(
                "[MOVE] Reached {} progress={:.3f}m cross_track={:+.3f}m".format(
                    target_cell, config.cell_size_m - remaining, cross_track
                ), flush=True,
            )
            return True, "CELL_COMPLETE", moved

        x_cmd, y_cmd, z_cmd, _yaw_error = _basic_motion_command(
            config, direction, start_yaw_deg, yaw
        )
        now = time.monotonic()
        if _yaw_error is not None:
            if now - last_heading_log >= 0.5:
                print("[HEADING_MOVE] yaw={:+.2f} reference={:+.2f} "
                      "diff={:+.2f} z={:+.2f} cross_track={:+.3f}".format(
                    float(yaw), float(start_yaw_deg), float(_yaw_error),
                    z_cmd, cross_track), flush=True)
                last_heading_log = now
            if abs(z_cmd) >= 2.0 and abs(_yaw_error) >= 1.5:
                if heading_probe is None:
                    heading_probe = (now, abs(_yaw_error))
                elif now - heading_probe[0] >= 0.65:
                    if abs(_yaw_error) >= heading_probe[1] + 2.0:
                        stop_chassis(chassis)
                        print("[HEADING_FAIL] DIVERGED {:.2f} -> {:.2f}; "
                              "verify heading_drive_sign.".format(
                            heading_probe[1], abs(_yaw_error)), flush=True)
                        return False, "HEADING_CORRECTION_DIVERGED", moved
                    heading_probe = (now, abs(_yaw_error))
            else:
                heading_probe = None
        if not command_logged:
            ux, uy = DIR_VEC_DRIVE[direction]
            print(
                "[MOTION] requested={:.3f} final={:.3f} direction={} "
                "x={:+.3f} y={:+.3f} yaw_correction={:+.2f}".format(
                    float(config.travel_speed_mps), x_cmd * ux + y_cmd * uy,
                    DIR_NAME[direction], x_cmd, y_cmd, z_cmd,
                ), flush=True,
            )
            command_logged = True

        chassis.drive_speed(
            x=x_cmd, y=y_cmd, z=z_cmd, timeout=config.drive_timeout_sec,
        )
        recorder.record_sample(
            time.monotonic(), rel_x, rel_y, yaw, direction, front_cm,
            None, None, None, None, "BASIC_MOVE_{}".format(DIR_NAME[direction]),
        )
        publish_state(
            status="Moving {} to {}".format(DIR_NAME[direction], target_cell),
            logical_cell=current_cell, gimbal_direction=direction,
            tof_cm=front_cm, moves=moves, force=False,
        )
        time.sleep(config.loop_delay_sec)

def _canonical_edge(
    cell: Tuple[int, int],
    direction: int,
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Return an undirected canonical key for one logical grid edge."""
    other = _neighbor(cell, direction)
    a = (int(cell[0]), int(cell[1]))
    b = (int(other[0]), int(other[1]))
    return (a, b) if a <= b else (b, a)


def _preference_order(last_direction: int) -> List[int]:
    last_direction %= 4
    return [
        last_direction,
        (last_direction - 1) % 4,
        (last_direction + 1) % 4,
        (last_direction + 2) % 4,
    ]


def _preference_rank(direction: int, last_direction: int) -> int:
    order = _preference_order(last_direction)
    try:
        return order.index(int(direction) % 4)
    except ValueError:
        return 99


def _visited_open_neighbors(
    cell: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> List[Tuple[int, Tuple[int, int]]]:
    result: List[Tuple[int, Tuple[int, int]]] = []
    for direction in range(4):
        if edge_states.get((cell[0], cell[1], direction)) != "OPEN":
            continue
        if (cell, direction) in blocked_edges:
            continue
        nxt = _neighbor(cell, direction)
        if nxt not in visited:
            continue
        if not _inside_working_canvas(nxt, config):
            continue
        result.append((direction, nxt))
    return result


def _frontier_options(
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> List[Tuple[Tuple[int, int], int, Tuple[int, int]]]:
    """All known OPEN edges from a visited cell into an unvisited cell."""
    options: List[Tuple[Tuple[int, int], int, Tuple[int, int]]] = []
    for cell in sorted(visited):
        for direction in range(4):
            if edge_states.get((cell[0], cell[1], direction)) != "OPEN":
                continue
            if (cell, direction) in blocked_edges:
                continue
            nxt = _neighbor(cell, direction)
            if nxt in visited:
                continue
            if not _inside_working_canvas(nxt, config):
                continue
            options.append((cell, direction, nxt))
    return options


def _shortest_open_path(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> Optional[List[Tuple[int, int]]]:
    """BFS shortest path through already-visited, confirmed-open cells."""
    if start == goal:
        return [start]

    queue_nodes: List[Tuple[int, int]] = [start]
    head = 0
    previous: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {
        start: None
    }

    while head < len(queue_nodes):
        cell = queue_nodes[head]
        head += 1

        for _direction, nxt in _visited_open_neighbors(
            cell,
            visited,
            edge_states,
            blocked_edges,
            config,
        ):
            if nxt in previous:
                continue
            previous[nxt] = cell
            if nxt == goal:
                path = [goal]
                cursor = goal
                while previous[cursor] is not None:
                    cursor = previous[cursor]  # type: ignore[index]
                    path.append(cursor)
                path.reverse()
                return path
            queue_nodes.append(nxt)

    return None


def _frontier_information_gain(
    frontier_cell: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> int:
    gain = 0
    for direction in range(4):
        if edge_states.get((frontier_cell[0], frontier_cell[1], direction)) != "OPEN":
            continue
        if (frontier_cell, direction) in blocked_edges:
            continue
        nxt = _neighbor(frontier_cell, direction)
        if nxt not in visited and _inside_working_canvas(nxt, config):
            gain += 1
    return gain


def _plan_frontier_move(
    current_cell: Tuple[int, int],
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
    last_move_direction: int,
) -> Optional[dict]:
    """Plan one step using nearest-frontier BFS.

    Priority:
    1) If the current cell has a known-open unvisited neighbour, expand now.
    2) Otherwise BFS through confirmed-open visited cells to the nearest cell
       that still has an open edge into unknown territory.
    3) Ties prefer more information gain, then motion continuity, then the
       frontier farther from the start so equal-cost choices expand outward.
    """
    frontiers = _frontier_options(
        visited,
        edge_states,
        blocked_edges,
        config,
    )
    if not frontiers:
        return None

    preference = _preference_order(last_move_direction)

    local_options = [
        option for option in frontiers if option[0] == current_cell
    ]
    if local_options:
        local_options.sort(
            key=lambda option: (
                _preference_rank(option[1], last_move_direction),
                -(
                    abs(option[2][0])
                    + abs(option[2][1])
                ),
            )
        )
        frontier_cell, direction, target = local_options[0]
        return {
            "move_direction": direction,
            "next_cell": target,
            "is_new": True,
            "frontier_cell": frontier_cell,
            "frontier_target": target,
            "route": [current_cell, target],
            "frontier_count": len(frontiers),
            "mode": "EXPAND_LOCAL_FRONTIER",
        }

    candidates = []
    for frontier_cell, frontier_direction, frontier_target in frontiers:
        path = _shortest_open_path(
            current_cell,
            frontier_cell,
            visited,
            edge_states,
            blocked_edges,
            config,
        )
        if not path or len(path) < 2:
            continue

        next_cell = path[1]
        move_direction = _direction_to(current_cell, next_cell)
        if move_direction is None:
            continue

        gain = _frontier_information_gain(
            frontier_cell,
            visited,
            edge_states,
            blocked_edges,
            config,
        )
        path_steps = len(path) - 1
        continuity_rank = _preference_rank(
            move_direction,
            last_move_direction,
        )
        outward = abs(frontier_target[0]) + abs(frontier_target[1])

        score = (
            path_steps,
            -gain,
            continuity_rank,
            -outward,
            frontier_cell[0],
            frontier_cell[1],
            frontier_direction,
        )
        candidates.append(
            (
                score,
                {
                    "move_direction": move_direction,
                    "next_cell": next_cell,
                    "is_new": False,
                    "frontier_cell": frontier_cell,
                    "frontier_target": frontier_target,
                    "route": path + [frontier_target],
                    "frontier_count": len(frontiers),
                    "mode": "RELOCATE_TO_FRONTIER",
                },
            )
        )

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def _closed_maze_completion_v04(
    visited: Set[Tuple[int, int]],
    edge_states: Dict[Tuple[int, int, int], str],
    blocked_edges: Set[Tuple[Tuple[int, int], int]],
    config: Classwork8Config,
) -> dict:
    """Robust completion test for the closed rectangular classwork arena.

    Frontier-only completion can be held open forever by one ToF miss on a low
    foam outer wall.  This secondary test never needs prior field dimensions:
    it derives the bounding rectangle from actually visited cells, requires the
    rectangle to be completely filled, and then checks wall evidence on all
    four outer sides.

    A ratio is used rather than demanding 100% because one low-wall reflection
    miss is common in the real arena.
    """
    result = {
        "enabled": bool(config.closed_maze_auto_stop),
        "complete": False,
        "filled": False,
        "rows": 0,
        "cols": 0,
        "bbox": None,
        "ratios": {},
        "threshold": float(config.closed_maze_perimeter_wall_ratio),
    }

    if not config.closed_maze_auto_stop or not visited:
        return result

    xs = [int(cell[0]) for cell in visited]
    ys = [int(cell[1]) for cell in visited]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    rows = max_x - min_x + 1
    cols = max_y - min_y + 1

    result["rows"] = rows
    result["cols"] = cols
    result["bbox"] = (min_x, max_x, min_y, max_y)

    if (
        rows < int(config.closed_maze_min_rows)
        or cols < int(config.closed_maze_min_cols)
    ):
        return result

    expected = {
        (x, y)
        for x in range(min_x, max_x + 1)
        for y in range(min_y, max_y + 1)
    }

    filled = expected.issubset(visited)
    result["filled"] = bool(filled)

    if not filled:
        return result

    sides = {
        "FRONT": [((max_x, y), 0) for y in range(min_y, max_y + 1)],
        "BACK": [((min_x, y), 2) for y in range(min_y, max_y + 1)],
        "RIGHT": [((x, min_y), 1) for x in range(min_x, max_x + 1)],
        "LEFT": [((x, max_y), 3) for x in range(min_x, max_x + 1)],
    }

    ratios: Dict[str, float] = {}

    for name, checks in sides.items():
        confirmed = 0

        for cell, direction in checks:
            state = edge_states.get((cell[0], cell[1], direction))
            blocked = (cell, direction) in blocked_edges

            if state == "WALL" or blocked:
                confirmed += 1

        ratios[name] = confirmed / float(max(1, len(checks)))

    result["ratios"] = ratios

    threshold = float(config.closed_maze_perimeter_wall_ratio)

    result["complete"] = all(
        ratio >= threshold
        for ratio in ratios.values()
    )

    return result

def run(
    config: Optional[Classwork8Config] = None,
    publish: Optional[Callable[[dict], None]] = None,
    stop_event: Optional[threading.Event] = None,
    ep_robot=None,
    survey_bridge: Optional[LiveSurveyBridge] = None,
) -> Path:
    config = config or Classwork8Config()
    survey_bridge = survey_bridge or LiveSurveyBridge(config)
    config.validate()
    stop_event = stop_event or threading.Event()

    grid = OccupancyGrid(
        config.map_width_m,
        config.map_height_m,
        config.resolution_m,
        free_delta=config.free_delta,
        occupied_delta=config.occupied_delta,
        min_score=config.min_score,
        max_score=config.max_score,
        free_threshold=config.free_threshold,
        occupied_threshold=config.occupied_threshold,
    )
    recorder = RunRecorder(config)
    recorder.start(time.monotonic())

    robot_was_preconnected = ep_robot is not None
    if ep_robot is None:
        ep_robot = robot.Robot()
    chassis = None
    gimbal = None
    tof_sensor = None
    tof_subscribed = False
    pose_subscribed = False
    attitude_subscribed = False
    gimbal_subscribed = False

    start_pose = (0.0, 0.0, 0.0)
    finish_reason = "UNKNOWN"
    current_cell = (0, 0)
    moves = 0
    current_gimbal_direction = 0
    current_tof = None
    last_publish = [0.0]

    planner_mode = "FRONTIER_BFS"
    planner_frontier_count = 0
    planner_frontier_cell: Optional[Tuple[int, int]] = None
    planner_frontier_target: Optional[Tuple[int, int]] = None
    planner_route: List[Tuple[int, int]] = []

    completion_status = {
        "enabled": bool(config.closed_maze_auto_stop),
        "complete": False,
        "filled": False,
        "rows": 0,
        "cols": 0,
        "bbox": None,
        "ratios": {},
        "threshold": float(config.closed_maze_perimeter_wall_ratio),
    }

    # Unknown-world topological map used by the realtime GUI.
    known_cells: Set[Tuple[int, int]] = {(0, 0)}
    edge_states: Dict[Tuple[int, int, int], str] = {}
    logical_path: List[Tuple[int, int]] = [(0, 0)]

    pose = V05PoseTracker()
    sensors = ToFOnlySensorManager()
    gimbal_tracker = GimbalTracker()

    # V05 uses one shared camera stream for target survey. Corridor steering is
    # intentionally disabled in this ToF+camera baseline; movement remains the
    # proven V04 ToF + odometry controller.
    vision: Optional[CorridorVision] = None
    camera_service: Optional[CameraService] = None
    target_detector: Optional[TargetDetector] = (
        TargetDetector(config) if config.target_detection_enabled else None
    )
    target_registry = TargetRegistry(config)
    target_debug_holder: List[object] = [None]
    start_scan_ranges: Optional[Dict[int, Optional[float]]] = None

    # Keep topology containers alive even if initialization fails so cleanup
    # can still export a useful partial run.
    visited: Set[Tuple[int, int]] = {current_cell}
    blocked_edges: Set[Tuple[Tuple[int, int], int]] = set()
    traversed_edges: Set[
        Tuple[Tuple[int, int], Tuple[int, int]]
    ] = set()
    last_move_direction = 0
    # A cached scan may be reused only for a FULLY scanned visited cell.
    # Never reuse readings as physical side-distance corrections after moving:
    # the cached wall topology is for planning; movement always samples fresh
    # travel-direction ToF before and throughout every cell.
    scanned_cells: Set[Tuple[int, int]] = set()

    raw_start_x = 0.0
    raw_start_y = 0.0
    raw_start_yaw = 0.0

    def publish_state(
        *,
        status: str,
        logical_cell: Tuple[int, int],
        gimbal_direction: int,
        tof_cm: Optional[float],
        moves: int,
        force: bool = False,
        reason: str = "",
        finished: bool = False,
        run_dir: Optional[str] = None,
    ) -> None:
        nonlocal current_gimbal_direction, current_tof

        current_gimbal_direction = int(gimbal_direction) % 4
        current_tof = tof_cm

        if publish is None:
            return

        now = time.monotonic()
        min_period = max(0.05, config.gui_refresh_ms / 1000.0)
        if not force and now - last_publish[0] < min_period:
            return
        last_publish[0] = now

        rel_x, rel_y = _relative_xy(
            pose, raw_start_x, raw_start_y, raw_start_yaw, config
        )

        camera_active = bool(
            camera_service is not None and camera_service.running
        )
        publish({
            "status": status,
            "reason": reason,
            "finished": bool(finished),
            # The runtime logical GUI does not read the 160x160 matrix.
            # Avoid allocating it at every pose refresh; export still writes
            # the full occupancy grid from the mapper itself.
            "resolution_m": grid.resolution_m,
            "cell_size_m": config.cell_size_m,
            "trajectory": recorder.trajectory_xy(),
            "robot_xy": None if rel_x is None or rel_y is None else (rel_x, rel_y),
            "logical_cell": logical_cell,
            "known_cells": sorted(known_cells),
            "wall_edges": sorted(
                key for key, state in edge_states.items() if state == "WALL"
            ),
            "open_edges": sorted(
                key for key, state in edge_states.items() if state == "OPEN"
            ),
            "logical_path": list(logical_path),
            "planner_mode": planner_mode,
            "planner_frontier_count": int(planner_frontier_count),
            "planner_frontier_cell": planner_frontier_cell,
            "planner_frontier_target": planner_frontier_target,
            "planner_route": list(planner_route),
            "completion_enabled": bool(completion_status.get("enabled")),
            "completion_ready": bool(completion_status.get("complete")),
            "completion_filled": bool(completion_status.get("filled")),
            "completion_rows": int(completion_status.get("rows", 0)),
            "completion_cols": int(completion_status.get("cols", 0)),
            "completion_bbox": completion_status.get("bbox"),
            "completion_perimeter_ratios": dict(
                completion_status.get("ratios", {})
            ),
            "completion_threshold": float(
                completion_status.get(
                    "threshold",
                    config.closed_maze_perimeter_wall_ratio,
                )
            ),
            "run_dir": run_dir,
            "gimbal_direction": current_gimbal_direction,
            "gimbal_direction_name": DIR_NAME[current_gimbal_direction],
            "gimbal_yaw_deg": gimbal_tracker.get_yaw(),
            "gimbal_pitch_deg": gimbal_tracker.get_pitch(),
            "gimbal_scan_pitch_target_deg": float(config.gimbal_scan_pitch_deg),
            "gimbal_pitch_tolerance_deg": float(config.gimbal_pitch_tolerance_deg),
            "tof_cm": tof_cm,
            "moves": int(moves),
            "coverage": grid.coverage_percent(),
            "vision_active": camera_active,
            "vision_steering_enabled": False,
            "vision_error": None,
            "vision_confidence": 0.0,
            # The annotated image is polled directly from LiveSurveyBridge by
            # Tk at preview cadence, never copied into every map snapshot.
            "vision_frame": None,
            "target_detection_active": bool(
                camera_active and target_detector is not None
            ),
            "target_count": len(target_registry.targets),
            "target_sighting_count": sum(
                1 for item in target_registry.targets
                if item.get("localization_status") == "SIGHTING_ONLY"
            ),
            "target_position_candidate_count": sum(
                1 for item in target_registry.targets
                if item.get("localization_status") == "NEAR_WALL_ESTIMATE"
            ),
            "targets": target_registry.public_targets(),
        })

    try:
        publish_state(
            status="Connecting to RoboMaster...",
            logical_cell=current_cell,
            gimbal_direction=0,
            tof_cm=None,
            moves=0,
            force=True,
        )

        if not robot_was_preconnected:
            print("Connecting to RoboMaster (mode={})...".format(config.connection), flush=True)
            ok = ep_robot.initialize(conn_type=config.connection)
            print("RoboMaster initialize returned: {!r}".format(ok), flush=True)
            if not ok:
                raise RuntimeError(
                    "RoboMaster connection failed. Check that the PC is connected "
                    "to the RoboMaster Wi-Fi/AP network."
                )
        chassis = ep_robot.chassis
        gimbal = ep_robot.gimbal
        tof_sensor = ep_robot.sensor

        # FREE mode decouples chassis yaw from gimbal yaw. The chassis therefore
        # never performs 90-degree scan turns.
        print("[INIT] Setting robot mode to FREE...", flush=True)
        mode_ok = ep_robot.set_robot_mode(mode=robot.FREE)
        print("[INIT] FREE mode result: {!r}".format(mode_ok), flush=True)
        if not mode_ok:
            raise RuntimeError(
                "FREE_MODE_FAILED: refusing scan; chassis could be coupled to gimbal"
            )
        stop_chassis(chassis)  # Explicitly enter the measured-stable zero-wheel mode.

        # Subscribe BEFORE recentering so we can verify the actual gimbal angle
        # even if the DJI action-completion packet is delayed/lost.
        print("[INIT] Subscribing ToF...", flush=True)
        tof_subscribed = bool(
            tof_sensor.sub_distance(
                freq=20,
                callback=sensors.tof_callback,
            )
        )
        print("[INIT] ToF subscription: {!r}".format(tof_subscribed), flush=True)

        print("[INIT] Subscribing odometry...", flush=True)
        pose_subscribed = bool(
            chassis.sub_position(
                cs=1,
                freq=20,
                callback=pose.position_callback,
            )
        )
        print("[INIT] Position subscription: {!r}".format(pose_subscribed), flush=True)

        print("[INIT] Subscribing attitude...", flush=True)
        attitude_subscribed = bool(
            chassis.sub_attitude(
                freq=20,
                callback=pose.attitude_callback,
            )
        )
        print("[INIT] Attitude subscription: {!r}".format(attitude_subscribed), flush=True)

        print("[INIT] Subscribing gimbal angle...", flush=True)
        gimbal_subscribed = bool(
            gimbal.sub_angle(
                freq=20,
                callback=gimbal_tracker.callback,
            )
        )
        print("[INIT] Gimbal subscription: {!r}".format(gimbal_subscribed), flush=True)

        print("[INIT] Waiting for initial position/yaw...", flush=True)
        raw_start_x, raw_start_y = wait_for_position(pose)
        raw_start_yaw = wait_for_yaw(pose)
        print(
            "[INIT] Pose ready: x={:+.3f} y={:+.3f} yaw={}".format(
                float(raw_start_x),
                float(raw_start_y),
                "---" if raw_start_yaw is None else "{:+.1f}".format(float(raw_start_yaw)),
            ),
            flush=True,
        )

        if raw_start_yaw is None:
            raise RuntimeError("attitude/yaw subscription did not produce data")

        print(
            "[INIT] Local map frame locked to mission-start yaw {:+.1f} deg.".format(
                float(raw_start_yaw)
            ),
            flush=True,
        )

        # Do not use gimbal.recenter().wait_for_completed() here. On this
        # RoboMaster the mechanical action can complete while its action-complete
        # packet is not received, which previously caused an indefinite wait.
        # Use the same closed-loop angle feedback as normal scanning instead.
        print("[INIT] Pointing gimbal to FRONT (0 deg) with angle feedback...", flush=True)
        if not _point_gimbal(
            gimbal,
            sensors,
            gimbal_tracker,
            0,
            config,
            stop_event,
        ):
            measured_yaw = gimbal_tracker.get_yaw()
            raise RuntimeError(
                "could not point gimbal to FRONT; measured yaw={}".format(
                    "---" if measured_yaw is None else "{:+.1f}".format(float(measured_yaw))
                )
            )
        print(
            "[INIT] Gimbal FRONT ready at yaw={:+.1f} deg.".format(
                float(gimbal_tracker.get_yaw())
            ),
            flush=True,
        )

        if config.yaw_isolation_mode:
            config.heading_hold_enabled = False
            print(
                "[YAW_ISOLATION] ACTIVE: every move uses chassis z=0; "
                "post-scan yaw correction is disabled.", flush=True,
            )
            stop_chassis(chassis)
            for count in range(6):
                _heading_snapshot(
                    "STATIONARY_{}".format(count), pose, gimbal_tracker,
                    float(raw_start_yaw),
                )
                if not _sleep_interruptible(0.5, stop_event):
                    break
        heading = HeadingManager()
        if not heading.initialize(raw_start_yaw):
            raise RuntimeError("yaw/attitude unavailable")

        # One shared camera stream. Round 1 uses it for target surveying only;
        # navigation remains ToF + odometry in this baseline.
        camera_ok = False
        if config.target_detection_enabled:
            camera_service = CameraService(
                ep_robot,
                resolution=config.target_camera_resolution,
                start_timeout_sec=config.target_camera_start_timeout_sec,
            )
            camera_ok = camera_service.start()
            if camera_ok:
                survey_bridge.attach_camera(camera_service)
            else:
                survey_bridge.set_status("Camera stream could not start")

        print(
            "[CAMERA] Round-1 target survey: {}".format(
                "ACTIVE" if camera_ok else "DISABLED / UNAVAILABLE"
            ),
            flush=True,
        )

        print("============================================================")
        print(" FINAL ROUND 1 V05 - ToF + CAMERA / MAP + TARGET SURVEY")
        print("============================================================")
        print("Cell size     : {:.2f} m".format(config.cell_size_m))
        print("Move per step : {:.2f} m (1 full cell)".format(config.exploration_step_m))
        print("Chassis yaw   : fixed; NO chassis scan turns")
        print("Scanning      : gimbal-only, 4 directions")
        print("Travel        : mecanum forward/right/back/left")
        print("Planner       : nearest-frontier BFS; no DFS parent-stack backtracking")
        print("Completion    : frontier exhaustion + closed-rectangle wall validation")
        print("Safety        : ToF points along travel direction continuously")
        print(
            "Camera        : {}".format(
                "target detection ACTIVE; steering remains ToF + odometry"
                if camera_service is not None and camera_service.running
                else "target detection unavailable"
            )
        )
        print("============================================================")

        recorder.event(
            time.monotonic(),
            "START",
            "V05 Round 1 ToF + camera map and target survey",
            logical_node=current_cell,
        )

        publish_state(
            status="Ready - scanning start cell",
            logical_cell=current_cell,
            gimbal_direction=0,
            tof_cm=sensors.get_front_cm(),
            moves=moves,
            force=True,
        )

        while moves < config.max_moves:
            if stop_event.is_set():
                finish_reason = "USER_STOP"
                break

            _heading_snapshot("PRE_SCAN_{}".format(current_cell),
                              pose, gimbal_tracker, float(raw_start_yaw))
            cache_valid = _should_reuse_scan(
                current_cell,
                scanned_cells,
                edge_states,
                config.skip_scanned_visited_cells,
                survey_bridge.rescan_requested(),
            )

            if cache_valid:
                # Cache contains only confirmed topology, not a fresh ToF
                # distance. Never feed old side distances to centering.
                ranges = {}
                open_dirs = {
                    direction for direction in range(4)
                    if edge_states.get(
                        (current_cell[0], current_cell[1], direction)
                    ) == "OPEN"
                    and (current_cell, direction) not in blocked_edges
                }
                recorder.event(
                    time.monotonic(),
                    "SCAN_REUSED",
                    "Visited cell: confirmed topology reused; live ToF observation remains enabled",
                    logical_node=current_cell,
                    open_directions=sorted(open_dirs),
                )
                print(
                    "[SCAN_REUSED] {} already scanned; skip 4-way sweep. "
                    "Travel-direction ToF remains observation only.".format(current_cell),
                    flush=True,
                )
                publish_state(
                    status="Visited cell {}: reusing map; no repeat 4-way scan".format(
                        current_cell
                    ),
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                )
            else:
                # Enter the experimentally stable zero-wheel mode before each scan.
                stop_chassis(chassis)
                _heading_snapshot(
                    "SCAN_STOP_SENT_{}".format(current_cell),
                    pose, gimbal_tracker, float(raw_start_yaw),
                )
                scan = _scan_four_directions(
                    gimbal,
                    pose,
                    sensors,
                    gimbal_tracker,
                    grid,
                    recorder,
                    config,
                    float(raw_start_x),
                    float(raw_start_y),
                    float(raw_start_yaw),
                    stop_event,
                    publish_state,
                    current_cell,
                    moves,
                    known_cells,
                    edge_states,
                    traversed_edges,
                    camera_service,
                    target_detector,
                    target_registry,
                    target_debug_holder,
                    survey_bridge,
                )
                if scan is None:
                    finish_reason = (
                        "USER_STOP"
                        if stop_event.is_set()
                        else "GIMBAL_SCAN_FAILED"
                    )
                    break

                ranges, open_dirs = scan
                _heading_snapshot("POST_SCAN_{}".format(current_cell),
                                  pose, gimbal_tracker, float(raw_start_yaw))
                scanned_cells.add(current_cell)

                if start_scan_ranges is None and current_cell == (0, 0) and moves == 0:
                    start_scan_ranges = dict(ranges)

                recorder.event(
                    time.monotonic(),
                    "SCAN",
                    "gimbal-only four-direction ToF scan",
                    logical_node=current_cell,
                    open_directions=sorted(open_dirs),
                    ranges_cm={str(k): v for k, v in sorted(ranges.items())},
                )

            # The operator can adjust the camera observation pitch from Tk
            # while exploration is running. At a safe stationary checkpoint,
            # repeat this cell's scan once instead of driving away from a
            # low target that the previous pitch might have missed.
            if survey_bridge.consume_rescan() and not stop_event.is_set():
                recorder.event(
                    time.monotonic(),
                    "TARGET_RESCAN_REQUESTED",
                    "Operator requested another survey of current cell",
                    logical_node=current_cell,
                    camera_pitch_deg=survey_bridge.get_pitch(),
                )
                publish_state(
                    status="Rescanning current cell with updated camera pitch",
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                )
                continue

            completion_status = _closed_maze_completion_v04(
                visited,
                edge_states,
                blocked_edges,
                config,
            )

            if completion_status["complete"]:
                stop_chassis(chassis)

                ratios_text = ", ".join(
                    "{}={:.0f}%".format(name, value * 100.0)
                    for name, value in sorted(
                        completion_status["ratios"].items()
                    )
                )

                finish_reason = "CLOSED_MAZE_COMPLETE"

                recorder.event(
                    time.monotonic(),
                    "FINISH",
                    (
                        "closed rectangular maze fully visited; "
                        "perimeter wall ratios: {}"
                    ).format(ratios_text),
                    logical_node=current_cell,
                    visited_nodes=len(visited),
                    moves=moves,
                )

                publish_state(
                    status=(
                        "Closed maze complete: {}x{} cells, {}".format(
                            completion_status["rows"],
                            completion_status["cols"],
                            ratios_text,
                        )
                    ),
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                )

                print(
                    "[COMPLETE] Closed maze fully explored: "
                    "{}x{} cells | {}".format(
                        completion_status["rows"],
                        completion_status["cols"],
                        ratios_text,
                    ),
                    flush=True,
                )
                break

            plan = _plan_frontier_move(
                current_cell,
                visited,
                edge_states,
                blocked_edges,
                config,
                last_move_direction,
            )

            if plan is None:
                planner_frontier_count = 0
                planner_frontier_cell = None
                planner_frontier_target = None
                planner_route = []

                finish_reason = "FRONTIER_EXPLORATION_COMPLETE"
                recorder.event(
                    time.monotonic(),
                    "FINISH",
                    finish_reason,
                    visited_nodes=len(visited),
                    moves=moves,
                )
                publish_state(
                    status="No reachable frontier remains",
                    logical_cell=current_cell,
                    gimbal_direction=current_gimbal_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                )
                break

            _heading_snapshot("PRE_DEPARTURE_{}".format(current_cell),
                              pose, gimbal_tracker, float(raw_start_yaw))
            align_ok, align_reason = _align_chassis_after_scan(
                chassis, pose, config, float(raw_start_yaw), stop_event)
            recorder.event(time.monotonic(), "HEADING_ALIGNMENT", align_reason,
                           logical_node=current_cell, yaw=pose.get_yaw(),
                           reference=float(raw_start_yaw))
            if not align_ok:
                finish_reason = align_reason
                break

            move_direction = int(plan["move_direction"])
            next_cell = tuple(plan["next_cell"])
            is_new = bool(plan["is_new"])
            planner_frontier_count = int(plan["frontier_count"])
            planner_frontier_cell = tuple(plan["frontier_cell"])
            planner_frontier_target = tuple(plan["frontier_target"])
            planner_route = [
                tuple(cell) for cell in plan["route"]
            ]

            recorder.event(
                time.monotonic(),
                "FRONTIER_PLAN",
                str(plan["mode"]),
                logical_node=current_cell,
                from_node=current_cell,
                to_node=next_cell,
                direction=DIR_NAME[move_direction],
            )

            publish_state(
                status=(
                    "Exploring new cell {} via {}".format(
                        next_cell,
                        DIR_NAME[move_direction],
                    )
                    if is_new
                    else "Routing to frontier {} via {}".format(
                        planner_frontier_cell,
                        DIR_NAME[move_direction],
                    )
                ),
                logical_cell=current_cell,
                gimbal_direction=move_direction,
                tof_cm=sensors.get_front_cm(),
                moves=moves,
                force=True,
            )

            recorder.event(
                time.monotonic(),
                "EXPLORE" if is_new else "RELOCATE_FRONTIER",
                (
                    "enter unvisited frontier cell"
                    if is_new
                    else "shortest-path relocation to nearest frontier"
                ),
                from_node=current_cell,
                to_node=next_cell,
                direction=DIR_NAME[move_direction],
            )

            ok, reason, moved = _drive_one_cell(
                chassis,
                gimbal,
                pose,
                heading,
                sensors,
                gimbal_tracker,
                vision,
                grid,
                recorder,
                config,
                float(raw_start_x),
                float(raw_start_y),
                float(raw_start_yaw),
                move_direction,
                current_cell,
                next_cell,
                ranges,
                adjacent_wall_sides(
                    move_direction, current_cell, next_cell, edge_states
                ),
                moves,
                stop_event,
                publish_state,
            )

            _heading_snapshot("POST_MOVE_{}".format(next_cell),
                              pose, gimbal_tracker, float(raw_start_yaw))
            if ok:
                previous_cell = current_cell
                current_cell = next_cell

                traversed_edges.add(
                    _canonical_edge(previous_cell, move_direction)
                )
                _set_edge_state(
                    edge_states,
                    previous_cell,
                    move_direction,
                    "OPEN",
                )

                if is_new:
                    visited.add(current_cell)
                known_cells.add(current_cell)
                logical_path.append(current_cell)

                last_move_direction = move_direction
                moves += 1
                continue

            # BASIC: no blocked-edge / replanning recovery.

            finish_reason = (
                "FRONTIER_RELOCATE_{}".format(reason)
                if not is_new
                else reason
            )
            break

        else:
            finish_reason = "MAX_MOVES_REACHED"

        if finish_reason == "UNKNOWN":
            finish_reason = "FRONTIER_EXPLORATION_COMPLETE"

    except KeyboardInterrupt:
        stop_event.set()
        finish_reason = "USER_STOP"

    except Exception as exc:
        finish_reason = "ERROR: {}".format(exc)
        recorder.event(time.monotonic(), "ERROR", str(exc))
        raise

    finally:
        # The preview worker must stop before CameraService releases the
        # shared OpenCV stream; it never issues robot commands.
        survey_bridge.stop()
        if chassis is not None:
            try:
                stop_chassis(chassis)
            except Exception:
                pass

        try:
            if camera_service is not None:
                camera_service.stop()
        except Exception:
            pass

        try:
            if vision is not None:
                vision.stop()
        except Exception:
            pass

        # Never wait on a gimbal action while shutting down. Just stop angular
        # motion; this keeps Ctrl+C / errors from hanging during cleanup.
        try:
            if gimbal is not None:
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
        except Exception:
            pass

        end_pose = None
        try:
            rel_x, rel_y = _relative_xy(
                pose,
                float(raw_start_x),
                float(raw_start_y),
                float(raw_start_yaw),
                config,
            )
            yaw = pose.get_yaw()
            if rel_x is not None and rel_y is not None and yaw is not None:
                end_pose = (
                    rel_x,
                    rel_y,
                    normalize_angle_deg(float(yaw) - float(raw_start_yaw)),
                )
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
            if gimbal is not None and gimbal_subscribed:
                gimbal.unsub_angle()
        except Exception:
            pass

        try:
            ep_robot.close()
        except Exception:
            pass

        print(
            "[MISSION] Finish reason: {}".format(finish_reason),
            flush=True,
        )
        run_dir = recorder.export(
            grid,
            reason=finish_reason,
            start_pose=start_pose,
            end_pose=end_pose,
        )

        try:
            target_registry.save(run_dir)
            save_topology(
                run_dir,
                cell_size_m=config.cell_size_m,
                start_cell=(0, 0),
                final_cell=current_cell,
                visited=visited,
                edge_states=edge_states,
                traversed_edges=traversed_edges,
                start_scan_ranges=start_scan_ranges,
                finish_reason=finish_reason,
            )
            print(
                "[EXPORT] Saved topology.json and targets.json ({} targets).".format(
                    len(target_registry.targets)
                ),
                flush=True,
            )
        except Exception as exc:
            print("[EXPORT] Final metadata export failed: {}".format(exc), flush=True)

        publish_state(
            status="Finished: {}".format(finish_reason),
            logical_cell=current_cell,
            gimbal_direction=current_gimbal_direction,
            tof_cm=current_tof,
            moves=moves,
            force=True,
            reason="{} | saved: {}".format(finish_reason, run_dir),
            finished=True,
            run_dir=str(run_dir),
        )

        print("Classwork 8 results saved to: {}".format(run_dir))

    return run_dir
