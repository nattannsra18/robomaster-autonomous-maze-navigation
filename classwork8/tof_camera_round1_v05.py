from __future__ import annotations

import math
import statistics
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Set, Tuple

from robomaster import robot

from robomaster_mission.mission import (
    HeadingManager,
    PoseTracker,
    SensorManager,
    normalize_angle_deg,
    stop_chassis,
    wait_for_position,
    wait_for_yaw,
)

from .camera_service import CameraService
from .config import Classwork8Config
from .occupancy_grid import OccupancyGrid
from .reporting import RunRecorder
from .target_detection import (
    TargetDetector,
    TargetRegistry,
    save_topology,
)
from .vision import CorridorVision


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


class GimbalTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self.pitch = None
        self.yaw = None

    def callback(self, data):
        try:
            if data is None or len(data) < 2:
                return
            pitch = float(data[0])
            yaw = float(data[1])
            with self._lock:
                self.pitch = pitch
                self.yaw = yaw
        except Exception:
            return

    def get_yaw(self) -> Optional[float]:
        with self._lock:
            return self.yaw


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
) -> bool:
    """Point ToF using closed-loop relative gimbal yaw feedback."""
    if stop_event is not None and stop_event.is_set():
        return False

    target = float(config.gimbal_yaw_for_direction(direction))
    started = time.monotonic()
    stable = 0

    while time.monotonic() - started < config.gimbal_turn_timeout_sec:
        if stop_event is not None and stop_event.is_set():
            gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
            return False

        current = tracker.get_yaw()
        if current is None:
            time.sleep(0.03)
            continue

        # Gimbal yaw is a mechanically limited absolute axis (about -250..250),
        # not an unlimited circular heading.  Do NOT wrap the error here.
        # Example: from BACK +180 deg to LEFT -90 deg, the safe move is -270 deg
        # through 0. Wrapping would incorrectly command +90 deg toward +270,
        # which hits the mechanical yaw limit and makes the scan time out.
        error = target - float(current)
        if abs(error) <= config.gimbal_tolerance_deg:
            stable += 1
            gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
            if stable >= config.gimbal_stable_samples:
                sensors.reset_filters()
                return _sleep_interruptible(config.gimbal_settle_sec, stop_event)
        else:
            stable = 0
            speed = max(
                config.gimbal_min_yaw_speed_dps,
                min(
                    config.gimbal_yaw_speed_dps,
                    abs(error) * config.gimbal_yaw_kp,
                ),
            )
            gimbal.drive_speed(
                pitch_speed=0.0,
                yaw_speed=math.copysign(speed, error),
            )

        time.sleep(0.03)

    gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
    return False


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


def _confirm_front_blocked(
    chassis,
    sensors: ToFOnlySensorManager,
    config: Classwork8Config,
    stop_event: Optional[threading.Event],
) -> Tuple[bool, Optional[float]]:
    """Stop first, then require several fresh low ToF readings.

    A single low reading must never be enough to rewrite maze topology.
    """
    stop_chassis(chassis)
    consecutive = 0
    last_value: Optional[float] = None
    deadline = time.monotonic() + max(
        0.6,
        float(config.front_block_confirm_samples)
        * float(config.front_block_confirm_interval_sec)
        * 4.0,
    )

    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return True, last_value

        value = sensors.get_front_cm()
        if value is None:
            time.sleep(0.02)
            continue

        last_value = float(value)

        if last_value <= float(config.stop_front_cm):
            consecutive += 1
            if consecutive >= int(config.front_block_confirm_samples):
                return True, last_value
        elif last_value >= (
            float(config.stop_front_cm)
            + float(config.front_block_release_margin_cm)
        ):
            return False, last_value
        else:
            consecutive = max(0, consecutive - 1)

        time.sleep(float(config.front_block_confirm_interval_sec))

    return consecutive >= int(config.front_block_confirm_samples), last_value


def _fixed_heading_control_v02(
    config: Classwork8Config,
    target_yaw_deg: float,
    current_yaw_deg: Optional[float],
    x_cmd: float,
    y_cmd: float,
    mode: str,
) -> Tuple[float, float, float, str, Optional[float]]:
    """Keep the chassis at mission-start yaw while translating.

    V01 allowed several degrees of yaw error while still translating.  V02
    corrects earlier and pauses translation when the error becomes large.
    """
    if not config.heading_hold_enabled or current_yaw_deg is None:
        return x_cmd, y_cmd, 0.0, mode, None

    error = normalize_angle_deg(float(target_yaw_deg) - float(current_yaw_deg))
    abs_error = abs(error)

    if abs_error <= float(config.heading_deadband_deg):
        return x_cmd, y_cmd, 0.0, mode, error

    hard_recover = abs_error >= float(config.heading_recover_trigger_deg)
    hold_translation = abs_error >= float(config.heading_recover_release_deg)

    max_z = (
        float(config.heading_recover_max_z_dps)
        if hard_recover
        else float(config.heading_max_z_dps)
    )

    z_cmd = (
        error
        * float(config.heading_kp_z)
        / float(config.heading_drive_sign)
    )
    z_cmd = max(-max_z, min(max_z, z_cmd))

    # Do not translate while the chassis is still visibly angled.  This is
    # intentionally stricter than V01 because mecanum translation with a
    # 2-4 degree yaw error accumulates real lateral displacement.
    if hold_translation:
        return 0.0, 0.0, z_cmd, "HEADING_RECOVER_V02", error

    return x_cmd, y_cmd, z_cmd, mode, error


def _scan_side_guidance_v02(
    direction: int,
    scan_ranges: Optional[Dict[int, Optional[float]]],
    config: Classwork8Config,
) -> Tuple[float, str]:
    """Return a small right-relative lateral correction from the stopped scan."""
    if (
        not config.scan_side_guidance_enabled
        or not scan_ranges
    ):
        return 0.0, "SIDE_GUIDANCE_OFF"

    direction %= 4
    left_dir = (direction - 1) % 4
    right_dir = (direction + 1) % 4

    left_cm = scan_ranges.get(left_dir)
    right_cm = scan_ranges.get(right_dir)

    left_valid = left_cm is not None and float(left_cm) > 0.0
    right_valid = right_cm is not None and float(right_cm) > 0.0

    max_corr = float(config.scan_side_max_correction_mps)

    # Both corridor walls are visible: centre from their distance difference.
    if (
        left_valid
        and right_valid
        and float(left_cm) <= float(config.scan_side_wall_max_cm)
        and float(right_cm) <= float(config.scan_side_wall_max_cm)
    ):
        # If the left wall is closer, move right (positive correction).
        correction = (
            float(right_cm) - float(left_cm)
        ) * float(config.scan_side_kp_mps_per_cm)
        correction = max(-max_corr, min(max_corr, correction))
        return correction, "CENTER_BETWEEN_SCAN_WALLS"

    # One wall is dangerously close: bias away from it.
    if left_valid and float(left_cm) <= float(config.scan_side_danger_cm):
        return +max_corr, "AVOID_SCAN_LEFT"

    if right_valid and float(right_cm) <= float(config.scan_side_danger_cm):
        return -max_corr, "AVOID_SCAN_RIGHT"

    return 0.0, "NO_SCAN_SIDE_GUIDANCE"


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

    stop_chassis_fn_called = False

    for direction in order:
        if stop_event is not None and stop_event.is_set():
            return None

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

        print(
            "[SCAN] Gimbal {} ready at {:+.1f} deg.".format(
                DIR_NAME[direction],
                float(gimbal_tracker.get_yaw()),
            ),
            flush=True,
        )

        distance_cm = _sample_tof(sensors, config, stop_event)

        # V02: readings between a definite near wall and the normal OPEN
        # threshold are ambiguous.  A foam edge / floor reflection can create
        # one short median even when the branch is physically open.  Re-sample
        # at the same gimbal angle and keep the larger robust median.  If this
        # turns out to be a false-open, _drive_one_cell still has a continuous
        # ToF stop guard before any topology is committed by movement.
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

        ranges[direction] = distance_cm
        print(
            "[SCAN] {} ToF = {} cm".format(
                DIR_NAME[direction],
                "---" if distance_cm is None else "{:.1f}".format(distance_cm),
            ),
            flush=True,
        )

        # V05 Round 1: while the gimbal is already stationary for the ToF scan,
        # use the same camera direction to survey colored shape targets.  This
        # adds no extra gimbal sweep and keeps one shared camera stream.
        if (
            camera_service is not None
            and camera_service.running
            and target_detector is not None
        ):
            verified_targets, target_debug = target_detector.verify_latest(
                camera_service
            )
            if target_debug is not None:
                target_debug_holder[0] = target_debug

            for verified in verified_targets:
                saved_target = target_registry.add_verified(
                    verified,
                    current_cell,
                    direction,
                    distance_cm,
                )
                recorder.event(
                    time.monotonic(),
                    "TARGET_OBSERVATION",
                    "{} {} at {} looking {}".format(
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
                    tof_cm=distance_cm,
                )
                print(
                    "[TARGET] {} {} -> {} conf={:.2f} from {} {}".format(
                        verified.detection.color.upper(),
                        verified.detection.shape.upper(),
                        saved_target["target_id"],
                        float(verified.confidence),
                        current_cell,
                        DIR_NAME[direction],
                    ),
                    flush=True,
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
    moves: int,
    stop_event: Optional[threading.Event],
    publish_state: Callable[..., None],
) -> Tuple[bool, str, float]:
    direction %= 4

    # ToF always faces the actual translation direction.
    if not _point_gimbal(
        gimbal,
        sensors,
        gimbal_tracker,
        direction,
        config,
        stop_event,
    ):
        return False, "USER_STOP", 0.0

    initial_front_cm = _wait_for_move_tof_v03(
        sensors,
        config,
        stop_event,
    )
    if initial_front_cm is None:
        return False, "TOF_STALE", 0.0

    x0, y0 = pose.get_xy()
    if x0 is None or y0 is None:
        return False, "ODOMETRY_UNAVAILABLE", 0.0

    start_map_x, start_map_y = _map_xy_from_raw(
        float(x0),
        float(y0),
        start_x,
        start_y,
        start_yaw_deg,
        config.odom_scale_x,
        config.odom_scale_y,
    )

    # Each logical node has a fixed metric centre.  Aim for that centre instead
    # of merely travelling 0.60 m from wherever the previous move happened to
    # finish.  This prevents mecanum slip from accumulating cell after cell.
    target_map_x = float(target_cell[0]) * config.cell_size_m
    target_map_y = float(target_cell[1]) * config.cell_size_m

    deadline = time.monotonic() + max(
        7.0,
        (config.exploration_step_m / config.travel_speed_mps) * 3.5,
    )

    drive_x_unit, drive_y_unit = DIR_VEC_DRIVE[direction]

    scan_side_correction, scan_side_mode = _scan_side_guidance_v02(
        direction,
        scan_ranges,
        config,
    )
    if abs(scan_side_correction) > 1e-9:
        print(
            "[MOVE] V02 {} side correction {:+.3f} m/s".format(
                scan_side_mode,
                scan_side_correction,
            ),
            flush=True,
        )

    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            stop_chassis(chassis)
            return False, "USER_STOP", 0.0

        raw_x, raw_y = pose.get_xy()
        yaw = pose.get_yaw()
        front_cm = sensors.get_front_cm()

        if raw_x is None or raw_y is None:
            stop_chassis(chassis)
            return False, "ODOMETRY_LOST", 0.0

        rel_x, rel_y = _map_xy_from_raw(
            float(raw_x),
            float(raw_y),
            start_x,
            start_y,
            start_yaw_deg,
            config.odom_scale_x,
            config.odom_scale_y,
        )

        delta_map_x = rel_x - start_map_x
        delta_map_y = rel_y - start_map_y
        moved = math.hypot(delta_map_x, delta_map_y)

        if direction == 0:      # FRONT
            remaining = target_map_x - rel_x
            cross_track = rel_y - target_map_y
        elif direction == 1:    # RIGHT
            remaining = rel_y - target_map_y
            cross_track = rel_x - target_map_x
        elif direction == 2:    # BACK
            remaining = rel_x - target_map_x
            cross_track = rel_y - target_map_y
        else:                   # LEFT
            remaining = target_map_y - rel_y
            cross_track = rel_x - target_map_x

        progress = config.cell_size_m - max(0.0, remaining)

        if rel_x is not None and rel_y is not None:
            _update_tof_ray(
                grid,
                config,
                rel_x,
                rel_y,
                direction,
                front_cm,
            )

        if (
            remaining <= config.step_tolerance_m
            and abs(cross_track) <= config.cell_center_tolerance_m
        ):
            stop_chassis(chassis)
            recorder.record_sample(
                time.monotonic(),
                rel_x,
                rel_y,
                yaw,
                direction,
                front_cm,
                None,
                None,
                None,
                None,
                "CELL_COMPLETE",
            )
            publish_state(
                status="Reached cell {}".format(target_cell),
                logical_cell=target_cell,
                gimbal_direction=direction,
                tof_cm=front_cm,
                moves=moves + 1,
                force=True,
            )
            print(
                "[MOVE] Reached {}: progress={:.3f} m cross_track={:+.3f} m".format(
                    target_cell,
                    config.cell_size_m - remaining,
                    cross_track,
                ),
                flush=True,
            )
            return True, "CELL_COMPLETE", moved

        if front_cm is None:
            stop_chassis(chassis)
            front_cm = _wait_for_move_tof_v03(
                sensors,
                config,
                stop_event,
            )
            if front_cm is None:
                return False, "TOF_STALE", moved

        if front_cm <= config.stop_front_cm:
            blocked, confirmed_cm = _confirm_front_blocked(
                chassis,
                sensors,
                config,
                stop_event,
            )
            if blocked:
                # If most of the calibrated cell was already traversed, the
                # physical robot is effectively in the target cell.  Keeping
                # current_cell at the parent would corrupt subsequent scans.
                along_progress = max(0.0, config.cell_size_m - max(0.0, remaining))
                if along_progress >= (
                    config.cell_size_m
                    * config.blocked_near_target_accept_ratio
                ):
                    publish_state(
                        status="Reached cell {} near front wall".format(target_cell),
                        logical_cell=target_cell,
                        gimbal_direction=direction,
                        tof_cm=confirmed_cm,
                        moves=moves + 1,
                        force=True,
                    )
                    print(
                        "[MOVE] Accepted {} near wall at progress {:.3f} m, ToF {:.1f} cm".format(
                            target_cell,
                            along_progress,
                            float(confirmed_cm or front_cm),
                        ),
                        flush=True,
                    )
                    return True, "CELL_COMPLETE_NEAR_WALL", moved
                return False, "FRONT_BLOCKED", moved

            # Transient low ToF reading cleared while stationary.
            front_cm = confirmed_cm if confirmed_cm is not None else front_cm

        speed = config.travel_speed_mps
        if front_cm < config.slow_front_cm:
            span = max(1.0, config.slow_front_cm - config.stop_front_cm)
            ratio = (front_cm - config.stop_front_cm) / span
            ratio = max(0.0, min(1.0, ratio))
            speed = max(0.04, config.travel_speed_mps * ratio)

        # Once the longitudinal 60 cm target is reached, do not keep pushing
        # forward merely because the robot is a few centimetres off the cell
        # centre-line.  Finish with a small perpendicular correction instead.
        along_speed = 0.0 if remaining <= config.step_tolerance_m else speed

        x_cmd = drive_x_unit * along_speed
        y_cmd = drive_y_unit * along_speed

        # Keep the robot near the centre-line of the current 60 cm cell.
        correction = max(
            -config.cross_track_max_mps,
            min(
                config.cross_track_max_mps,
                config.cross_track_kp * cross_track,
            ),
        )
        if direction in (0, 2):
            # map +Y is LEFT, chassis +Y is RIGHT.
            y_cmd += correction
        else:
            # Positive cross-track x means we drifted forward; correct backward.
            x_cmd -= correction

        # V02: use the four-way stopped scan as a small physical corridor
        # centering bias. This is independent of wheel odometry, which can slip
        # on mecanum motion.
        if abs(scan_side_correction) > 1e-9:
            # The stopped scan is a snapshot, not a continuous side sensor.
            # Apply its bias strongly only at the beginning of the cell and
            # fade it out after ~35 cm so it cannot push across the corridor.
            fade_distance = max(0.10, min(config.cell_size_m, 0.35))
            fade = max(0.0, min(1.0, 1.0 - progress / fade_distance))
            applied_scan_side = scan_side_correction * fade
            right_x_unit, right_y_unit = DIR_RIGHT_VEC_DRIVE[direction]
            x_cmd += right_x_unit * applied_scan_side
            y_cmd += right_y_unit * applied_scan_side

        # Camera assistance is deliberately secondary to odometry. It only
        # contributes when both corridor boundaries are visible with enough
        # confidence. Positive correction means "move right" in camera/travel
        # coordinates, then gets rotated into chassis x/y here.
        vision_correction = 0.0
        vision_error = 0.0
        vision_confidence = 0.0
        if vision is not None and vision.running:
            (
                proposed_vision_correction,
                vision_error,
                vision_confidence,
            ) = vision.correction_mps()
            if config.vision_steering_enabled:
                vision_correction = proposed_vision_correction
                right_x_unit, right_y_unit = DIR_RIGHT_VEC_DRIVE[direction]
                x_cmd += right_x_unit * vision_correction
                y_cmd += right_y_unit * vision_correction

        z_cmd = 0.0
        mode = "MOVE_{}".format(DIR_NAME[direction])

        if config.heading_hold_enabled:
            x_cmd, y_cmd, z_cmd, mode, _ = _fixed_heading_control_v02(
                config,
                start_yaw_deg,
                yaw,
                x_cmd,
                y_cmd,
                mode,
            )

        # Keep combined odometry+vision lateral correction bounded.
        component_limit = max(
            config.travel_speed_mps,
            config.travel_speed_mps
            + config.cross_track_max_mps
            + config.vision_max_correction_mps,
        )
        x_cmd = max(-component_limit, min(component_limit, x_cmd))
        y_cmd = max(-component_limit, min(component_limit, y_cmd))

        chassis.drive_speed(
            x=x_cmd,
            y=y_cmd,
            z=z_cmd,
            timeout=config.drive_timeout_sec,
        )

        recorder.record_sample(
            time.monotonic(),
            rel_x,
            rel_y,
            yaw,
            direction,
            front_cm,
            None,
            None,
            None,
            None,
            mode,
            vision_error=vision_error,
            vision_confidence=vision_confidence,
            vision_correction_mps=vision_correction,
        )

        publish_state(
            status="Moving {} to {}".format(DIR_NAME[direction], target_cell),
            logical_cell=current_cell,
            gimbal_direction=direction,
            tof_cm=front_cm,
            moves=moves,
            force=False,
        )

        time.sleep(config.loop_delay_sec)

    stop_chassis(chassis)
    return False, "CELL_TIMEOUT", math.hypot(
        float(pose.get_xy()[0] or x0) - float(x0),
        float(pose.get_xy()[1] or y0) - float(y0),
    )



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


def _wait_for_move_tof_v03(
    sensors: ToFOnlySensorManager,
    config: Classwork8Config,
    stop_event: Optional[threading.Event],
) -> Optional[float]:
    """Retry a fresh ToF sample instead of ending a mission on one stale gap."""
    attempts = max(1, int(config.tof_recovery_retries) + 1)

    for attempt in range(attempts):
        value = _wait_for_fresh_tof(
            sensors,
            config.tof_recovery_wait_sec,
            stop_event,
        )
        if value is not None:
            return value

        if stop_event is not None and stop_event.is_set():
            return None

        if attempt + 1 < attempts:
            sensors.reset_filters()
            _sleep_interruptible(0.15, stop_event)

    return None


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
) -> Path:
    config = config or Classwork8Config()
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

    pose = PoseTracker()
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
        target_preview = target_debug_holder[0]
        if target_preview is None and camera_active:
            try:
                target_preview = camera_service.latest(max_age_sec=1.0)
            except Exception:
                target_preview = None

        publish({
            "status": status,
            "reason": reason,
            "finished": bool(finished),
            "matrix": grid.matrix(),
            "rows": grid.rows,
            "cols": grid.cols,
            "origin_x_m": grid.origin_x_m,
            "origin_y_m": grid.origin_y_m,
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
            "tof_cm": tof_cm,
            "moves": int(moves),
            "coverage": grid.coverage_percent(),
            "vision_active": camera_active,
            "vision_steering_enabled": False,
            "vision_error": None,
            "vision_confidence": 0.0,
            "vision_frame": target_preview,
            "target_detection_active": bool(
                camera_active and target_detector is not None
            ),
            "target_count": len(target_registry.targets),
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
            )
            if scan is None:
                finish_reason = (
                    "USER_STOP"
                    if stop_event.is_set()
                    else "GIMBAL_SCAN_FAILED"
                )
                break

            ranges, open_dirs = scan

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
                moves,
                stop_event,
                publish_state,
            )

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

            if reason == "FRONT_BLOCKED":
                blocked_edges.add((current_cell, move_direction))

                edge_key = _canonical_edge(
                    current_cell,
                    move_direction,
                )
                if edge_key not in traversed_edges:
                    _set_edge_state(
                        edge_states,
                        current_cell,
                        move_direction,
                        "WALL",
                    )

                recorder.event(
                    time.monotonic(),
                    "BLOCKED_EDGE",
                    (
                        "confirmed ToF block while expanding frontier"
                        if is_new
                        else "confirmed ToF block while relocating to frontier"
                    ),
                    logical_node=current_cell,
                    direction=DIR_NAME[move_direction],
                    travelled_m=round(moved, 3),
                )
                publish_state(
                    status=(
                        "Blocked {} - replanning frontier route".format(
                            DIR_NAME[move_direction]
                        )
                    ),
                    logical_cell=current_cell,
                    gimbal_direction=move_direction,
                    tof_cm=sensors.get_front_cm(),
                    moves=moves,
                    force=True,
                )
                continue

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
            status="Finished",
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
