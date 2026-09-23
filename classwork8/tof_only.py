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

from .config import Classwork8Config
from .occupancy_grid import OccupancyGrid
from .reporting import RunRecorder


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


def _relative_xy(
    pose: PoseTracker,
    start_x: float,
    start_y: float,
) -> Tuple[Optional[float], Optional[float]]:
    """Convert RoboMaster body-start coordinates to a conventional map frame."""
    x, y = pose.get_xy()
    if x is None or y is None:
        return None, None
    # RoboMaster +Y is right. Map +Y is left/up.
    return float(x) - start_x, -(float(y) - start_y)


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

        error = normalize_angle_deg(target - float(current))
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
    stop_event: Optional[threading.Event],
    publish_state: Callable[..., None],
    current_cell: Tuple[int, int],
    moves: int,
) -> Optional[Tuple[Dict[int, Optional[float]], Set[int]]]:
    # Sweep monotonically from left -> front -> right -> back.
    # The chassis itself does not rotate.
    order = [3, 0, 1, 2]
    ranges: Dict[int, Optional[float]] = {}
    open_dirs: Set[int] = set()

    stop_chassis_fn_called = False

    for direction in order:
        if stop_event is not None and stop_event.is_set():
            return None

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
            return None

        distance_cm = _sample_tof(sensors, config, stop_event)
        ranges[direction] = distance_cm

        rel_x, rel_y = _relative_xy(pose, start_x, start_y)
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

        if distance_cm is not None and distance_cm >= config.tof_open_cm:
            open_dirs.add(direction)

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
    grid: OccupancyGrid,
    recorder: RunRecorder,
    config: Classwork8Config,
    start_x: float,
    start_y: float,
    direction: int,
    current_cell: Tuple[int, int],
    target_cell: Tuple[int, int],
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

    x0, y0 = pose.get_xy()
    if x0 is None or y0 is None:
        return False, "ODOMETRY_UNAVAILABLE", 0.0

    deadline = time.monotonic() + max(
        7.0,
        (config.exploration_step_m / config.travel_speed_mps) * 3.5,
    )

    drive_x_unit, drive_y_unit = DIR_VEC_DRIVE[direction]

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

        moved = math.hypot(float(raw_x) - float(x0), float(raw_y) - float(y0))
        rel_x, rel_y = _relative_xy(pose, start_x, start_y)

        if rel_x is not None and rel_y is not None:
            _update_tof_ray(
                grid,
                config,
                rel_x,
                rel_y,
                direction,
                front_cm,
            )

        if moved >= config.exploration_step_m - config.step_tolerance_m:
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
            return True, "CELL_COMPLETE", moved

        if front_cm is None:
            stop_chassis(chassis)
            return False, "TOF_STALE", moved

        if front_cm <= config.stop_front_cm:
            stop_chassis(chassis)
            return False, "FRONT_BLOCKED", moved

        speed = config.travel_speed_mps
        if front_cm < config.slow_front_cm:
            span = max(1.0, config.slow_front_cm - config.stop_front_cm)
            ratio = (front_cm - config.stop_front_cm) / span
            ratio = max(0.0, min(1.0, ratio))
            speed = max(0.04, config.travel_speed_mps * ratio)

        x_cmd = drive_x_unit * speed
        y_cmd = drive_y_unit * speed
        z_cmd = 0.0
        mode = "MOVE_{}".format(DIR_NAME[direction])

        if config.heading_hold_enabled:
            x_cmd, y_cmd, z_cmd, mode, _ = heading.apply(
                x_cmd,
                y_cmd,
                yaw,
                mode,
            )

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

    pose = PoseTracker()
    sensors = ToFOnlySensorManager()
    gimbal_tracker = GimbalTracker()

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

        rel_x, rel_y = _relative_xy(pose, raw_start_x, raw_start_y)
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
            "gimbal_direction": current_gimbal_direction,
            "gimbal_direction_name": DIR_NAME[current_gimbal_direction],
            "gimbal_yaw_deg": gimbal_tracker.get_yaw(),
            "tof_cm": tof_cm,
            "moves": int(moves),
            "coverage": grid.coverage_percent(),
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

        # Never block forever on an SDK action. If the action completion packet
        # is lost, validate the real relative gimbal yaw from sub_angle().
        print("[INIT] Recentering gimbal (5 s timeout)...", flush=True)
        recenter_action = gimbal.recenter(
            pitch_speed=config.gimbal_yaw_speed_dps,
            yaw_speed=config.gimbal_yaw_speed_dps,
        )
        recenter_ok = recenter_action.wait_for_completed(timeout=5.0)

        if not recenter_ok:
            print(
                "[INIT] Recenter action timed out; checking measured gimbal yaw...",
                flush=True,
            )
            deadline = time.monotonic() + 2.0
            measured_yaw = gimbal_tracker.get_yaw()
            while measured_yaw is None and time.monotonic() < deadline:
                time.sleep(0.05)
                measured_yaw = gimbal_tracker.get_yaw()

            if measured_yaw is None or abs(normalize_angle_deg(float(measured_yaw))) > 5.0:
                raise RuntimeError(
                    "gimbal recenter failed/timed out and measured yaw is not near 0 deg"
                )

            print(
                "[INIT] Measured gimbal yaw is {:+.1f} deg; continuing.".format(
                    float(measured_yaw)
                ),
                flush=True,
            )
        else:
            print("[INIT] Gimbal recenter complete.", flush=True)

        heading = HeadingManager()
        if not heading.initialize(raw_start_yaw):
            raise RuntimeError("yaw/attitude unavailable")

        print("============================================================")
        print(" Classwork 8 - ToF ONLY / 60 cm CELL / REALTIME GUI")
        print("============================================================")
        print("Cell size     : {:.2f} m".format(config.cell_size_m))
        print("Move per step : {:.2f} m (1 full cell)".format(config.exploration_step_m))
        print("Chassis yaw   : fixed; NO chassis scan turns")
        print("Scanning      : gimbal-only, 4 directions")
        print("Travel        : mecanum forward/right/back/left")
        print("Safety        : ToF points along travel direction continuously")
        print("============================================================")

        recorder.event(
            time.monotonic(),
            "START",
            "ToF-only 60 cm cell exploration with gimbal scanning",
            logical_node=current_cell,
        )

        visited: Set[Tuple[int, int]] = {current_cell}
        parent_stack: List[Tuple[int, int]] = []
        blocked_edges: Set[Tuple[Tuple[int, int], int]] = set()
        last_move_direction = 0

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
                stop_event,
                publish_state,
                current_cell,
                moves,
            )
            if scan is None:
                finish_reason = "USER_STOP" if stop_event.is_set() else "GIMBAL_SCAN_FAILED"
                break

            ranges, open_dirs = scan
            recorder.event(
                time.monotonic(),
                "SCAN",
                "gimbal-only four-direction ToF scan",
                logical_node=current_cell,
                open_directions=sorted(open_dirs),
                ranges_cm={str(k): v for k, v in sorted(ranges.items())},
            )

            # Prefer continuing in the previous travel direction, then left,
            # right, and back relative to it.
            preference = [
                last_move_direction,
                (last_move_direction - 1) % 4,
                (last_move_direction + 1) % 4,
                (last_move_direction + 2) % 4,
            ]

            chosen = None
            for direction in preference:
                if direction not in open_dirs:
                    continue
                if (current_cell, direction) in blocked_edges:
                    continue
                nxt = _neighbor(current_cell, direction)
                if not _inside_working_canvas(nxt, config):
                    continue
                if nxt not in visited:
                    chosen = direction
                    break

            if chosen is not None:
                nxt = _neighbor(current_cell, chosen)
                recorder.event(
                    time.monotonic(),
                    "EXPLORE",
                    "move one full 60 cm cell without rotating chassis",
                    from_node=current_cell,
                    to_node=nxt,
                    direction=DIR_NAME[chosen],
                )

                ok, reason, moved = _drive_one_cell(
                    chassis,
                    gimbal,
                    pose,
                    heading,
                    sensors,
                    gimbal_tracker,
                    grid,
                    recorder,
                    config,
                    float(raw_start_x),
                    float(raw_start_y),
                    chosen,
                    current_cell,
                    nxt,
                    moves,
                    stop_event,
                    publish_state,
                )

                if ok:
                    parent_stack.append(current_cell)
                    current_cell = nxt
                    visited.add(current_cell)
                    last_move_direction = chosen
                    moves += 1
                    continue

                if reason == "FRONT_BLOCKED":
                    blocked_edges.add((current_cell, chosen))
                    recorder.event(
                        time.monotonic(),
                        "BLOCKED_EDGE",
                        "ToF stopped attempted cell traversal",
                        logical_node=current_cell,
                        direction=DIR_NAME[chosen],
                        travelled_m=round(moved, 3),
                    )
                    publish_state(
                        status="Blocked {} - choosing another path".format(DIR_NAME[chosen]),
                        logical_cell=current_cell,
                        gimbal_direction=chosen,
                        tof_cm=sensors.get_front_cm(),
                        moves=moves,
                        force=True,
                    )
                    continue

                finish_reason = reason
                break

            if not parent_stack:
                finish_reason = "DFS_EXPLORATION_COMPLETE"
                recorder.event(
                    time.monotonic(),
                    "FINISH",
                    finish_reason,
                    visited_nodes=len(visited),
                    moves=moves,
                )
                break

            parent = parent_stack.pop()
            back_direction = _direction_to(current_cell, parent)
            if back_direction is None:
                finish_reason = "DFS_STACK_ERROR"
                break

            recorder.event(
                time.monotonic(),
                "BACKTRACK",
                "return one 60 cm cell to DFS parent",
                from_node=current_cell,
                to_node=parent,
                direction=DIR_NAME[back_direction],
            )

            ok, reason, moved = _drive_one_cell(
                chassis,
                gimbal,
                pose,
                heading,
                sensors,
                gimbal_tracker,
                grid,
                recorder,
                config,
                float(raw_start_x),
                float(raw_start_y),
                back_direction,
                current_cell,
                parent,
                moves,
                stop_event,
                publish_state,
            )
            if not ok:
                finish_reason = "BACKTRACK_{}".format(reason)
                break

            current_cell = parent
            last_move_direction = back_direction
            moves += 1

        else:
            finish_reason = "MAX_MOVES_REACHED"

        if finish_reason == "UNKNOWN":
            finish_reason = "DFS_EXPLORATION_COMPLETE"

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
            if gimbal is not None:
                gimbal.recenter(
                    pitch_speed=config.gimbal_yaw_speed_dps,
                    yaw_speed=config.gimbal_yaw_speed_dps,
                ).wait_for_completed()
        except Exception:
            pass

        end_pose = None
        try:
            rel_x, rel_y = _relative_xy(pose, float(raw_start_x), float(raw_start_y))
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

        publish_state(
            status="Finished",
            logical_cell=current_cell,
            gimbal_direction=current_gimbal_direction,
            tof_cm=current_tof,
            moves=moves,
            force=True,
            reason="{} | saved: {}".format(finish_reason, run_dir),
            finished=True,
        )

        print("Classwork 8 results saved to: {}".format(run_dir))

    return run_dir
