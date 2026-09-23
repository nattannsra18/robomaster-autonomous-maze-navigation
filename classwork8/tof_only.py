from __future__ import annotations

import math
import statistics
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

from robomaster import robot

from robomaster_mission.mission import (
    HeadingManager,
    PoseTracker,
    SensorManager,
    feedback_turn,
    normalize_angle_deg,
    stop_chassis,
    wait_for_position,
    wait_for_yaw,
)

from .config import Classwork8Config
from .occupancy_grid import OccupancyGrid
from .reporting import RunRecorder


# Logical cardinal directions relative to the robot's starting pose.
# 0 = forward/+X, 1 = right/-Y, 2 = back/-X, 3 = left/+Y.
DIR_VEC = {
    0: (1, 0),
    1: (0, -1),
    2: (-1, 0),
    3: (0, 1),
}
DIR_NAME = {
    0: "FRONT/N",
    1: "RIGHT/E",
    2: "BACK/S",
    3: "LEFT/W",
}


def _direction_angle_rad(direction: int) -> float:
    return {
        0: 0.0,
        1: -math.pi / 2.0,
        2: math.pi,
        3: math.pi / 2.0,
    }[direction % 4]


def _relative_xy(
    pose: PoseTracker,
    start_x: float,
    start_y: float,
) -> Tuple[Optional[float], Optional[float]]:
    x, y = pose.get_xy()
    if x is None or y is None:
        return None, None
    return float(x) - start_x, float(y) - start_y


class ToFOnlySensorManager(SensorManager):
    """Sensor manager that intentionally disables IR/Sharp access."""

    def __init__(self):
        super().__init__(None)

    def read_front_corner_ir(self):
        return None, None


def _sample_tof(
    sensors: SensorManager,
    config: Classwork8Config,
) -> Optional[float]:
    values: List[float] = []
    deadline = time.monotonic() + 1.2
    while len(values) < config.scan_samples and time.monotonic() < deadline:
        value = sensors.get_front_cm()
        if value is not None:
            values.append(float(value))
        time.sleep(config.scan_sample_interval_sec)
    if not values:
        return None
    return float(statistics.median(values))


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
        distance_cm / 100.0,
        max_range_m=config.tof_max_mapping_cm / 100.0,
        hit=hit,
    )


def _turn_to_direction(
    chassis,
    pose: PoseTracker,
    heading: HeadingManager,
    sensors: SensorManager,
    target_direction: int,
    config: Classwork8Config,
) -> bool:
    target_direction %= 4
    current_direction = heading.heading_index % 4
    diff = (target_direction - current_direction) % 4
    relative = {
        0: "FRONT",
        1: "RIGHT",
        2: "BACK",
        3: "LEFT",
    }[diff]

    if relative != "FRONT":
        if not feedback_turn(chassis, sensors, pose, relative):
            return False

    heading.set_heading_index(target_direction)
    time.sleep(config.scan_settle_sec)
    return True


def _scan_four_directions(
    chassis,
    pose: PoseTracker,
    heading: HeadingManager,
    sensors: SensorManager,
    grid: OccupancyGrid,
    recorder: RunRecorder,
    config: Classwork8Config,
    start_x: float,
    start_y: float,
) -> Optional[Tuple[Dict[int, Optional[float]], Set[int], int]]:
    arrival_direction = heading.heading_index % 4
    order = [
        arrival_direction,
        (arrival_direction - 1) % 4,  # left
        (arrival_direction + 1) % 4,  # right
        (arrival_direction + 2) % 4,  # back
    ]

    ranges: Dict[int, Optional[float]] = {}
    open_dirs: Set[int] = set()

    for direction in order:
        if not _turn_to_direction(
            chassis, pose, heading, sensors, direction, config
        ):
            return None

        # Discard ToF readings that may still belong to the previous heading.
        sensors.reset_filters()
        time.sleep(config.scan_settle_sec)
        distance_cm = _sample_tof(sensors, config)
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
            "TOF_SCAN",
        )

    return ranges, open_dirs, arrival_direction


def _move_one_step(
    chassis,
    pose: PoseTracker,
    heading: HeadingManager,
    sensors: SensorManager,
    grid: OccupancyGrid,
    recorder: RunRecorder,
    config: Classwork8Config,
    start_x: float,
    start_y: float,
    direction: int,
) -> Tuple[bool, str]:
    if not _turn_to_direction(
        chassis, pose, heading, sensors, direction, config
    ):
        return False, "TURN_FAILED"

    x0, y0 = pose.get_xy()
    if x0 is None or y0 is None:
        return False, "ODOMETRY_UNAVAILABLE"

    deadline = time.monotonic() + max(
        4.0,
        (config.exploration_step_m / config.forward_speed_mps) * 4.0,
    )

    while time.monotonic() < deadline:
        x, y = pose.get_xy()
        yaw = pose.get_yaw()
        front_cm = sensors.get_front_cm()

        if x is None or y is None:
            stop_chassis(chassis)
            return False, "ODOMETRY_LOST"
        if front_cm is None:
            stop_chassis(chassis)
            return False, "TOF_STALE"

        rel_x = float(x) - start_x
        rel_y = float(y) - start_y
        _update_tof_ray(
            grid,
            config,
            rel_x,
            rel_y,
            direction,
            front_cm,
        )

        moved = math.hypot(float(x) - float(x0), float(y) - float(y0))
        if moved >= config.exploration_step_m:
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
                "STEP_COMPLETE",
            )
            return True, "STEP_COMPLETE"

        if front_cm <= config.stop_front_cm:
            stop_chassis(chassis)
            return False, "FRONT_BLOCKED"

        speed = config.forward_speed_mps
        if front_cm < config.slow_front_cm:
            span = max(
                1.0,
                config.slow_front_cm - config.stop_front_cm,
            )
            ratio = (
                (front_cm - config.stop_front_cm) / span
            )
            ratio = max(0.0, min(1.0, ratio))
            speed = max(0.05, config.forward_speed_mps * ratio)

        x_cmd, y_cmd, z_cmd, mode, _ = heading.apply(
            speed,
            0.0,
            yaw,
            "TOF_ONLY_FORWARD",
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

        time.sleep(config.loop_delay_sec)

    stop_chassis(chassis)
    return False, "STEP_TIMEOUT"


def _neighbor(node: Tuple[int, int], direction: int) -> Tuple[int, int]:
    dx, dy = DIR_VEC[direction % 4]
    return node[0] + dx, node[1] + dy


def _direction_to(
    current: Tuple[int, int],
    target: Tuple[int, int],
) -> Optional[int]:
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    for direction, vec in DIR_VEC.items():
        if vec == (dx, dy):
            return direction
    return None


def _inside_working_canvas(
    node: Tuple[int, int],
    config: Classwork8Config,
) -> bool:
    x = node[0] * config.exploration_step_m
    y = node[1] * config.exploration_step_m
    margin = max(0.25, config.exploration_step_m)
    return (
        abs(x) <= config.map_width_m / 2.0 - margin
        and abs(y) <= config.map_height_m / 2.0 - margin
    )


def run(config: Optional[Classwork8Config] = None) -> Path:
    config = config or Classwork8Config()
    config.validate()

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

    ep_robot = robot.Robot()
    chassis = None
    tof_sensor = None
    tof_subscribed = False
    pose_subscribed = False
    attitude_subscribed = False
    start_pose = (0.0, 0.0, 0.0)
    finish_reason = "UNKNOWN"

    try:
        print("Connecting RoboMaster for ToF-only Classwork 8...")
        ep_robot.initialize(conn_type=config.connection)

        chassis = ep_robot.chassis
        tof_sensor = ep_robot.sensor
        gimbal = ep_robot.gimbal

        ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
        gimbal.recenter(
            pitch_speed=100,
            yaw_speed=100,
        ).wait_for_completed()

        pose = PoseTracker()
        sensors = ToFOnlySensorManager()
        heading = HeadingManager()

        tof_subscribed = bool(
            tof_sensor.sub_distance(
                freq=20,
                callback=sensors.tof_callback,
            )
        )
        pose_subscribed = bool(
            chassis.sub_position(
                cs=1,
                freq=20,
                callback=pose.position_callback,
            )
        )
        attitude_subscribed = bool(
            chassis.sub_attitude(
                freq=20,
                callback=pose.attitude_callback,
            )
        )

        raw_start_x, raw_start_y = wait_for_position(pose)
        raw_start_yaw = wait_for_yaw(pose)
        if not heading.initialize(raw_start_yaw):
            raise RuntimeError("yaw/attitude unavailable")

        print("============================================================")
        print(" Classwork 8 - ToF ONLY Unknown World Exploration")
        print("============================================================")
        print("Sensor      : front ToF on recentered gimbal")
        print("Localization: RoboMaster odometry + yaw")
        print(
            "Method      : stop -> rotate chassis -> scan 4 directions "
            "-> DFS/backtrack"
        )
        print(
            "Step        : {:.2f} m | Open threshold: {:.1f} cm | Speed: {:.2f} m/s".format(
                config.exploration_step_m,
                config.tof_open_cm,
                config.forward_speed_mps,
            )
        )
        print("Press Ctrl+C at any time to stop and save results.")
        print("============================================================")

        current = (0, 0)
        visited: Set[Tuple[int, int]] = {current}
        parent_stack: List[Tuple[int, int]] = []
        moves = 0

        recorder.event(
            time.monotonic(),
            "START",
            "ToF-only exploration started",
            logical_node=current,
        )

        while moves < config.max_moves:
            scan = _scan_four_directions(
                chassis,
                pose,
                heading,
                sensors,
                grid,
                recorder,
                config,
                float(raw_start_x),
                float(raw_start_y),
            )
            if scan is None:
                finish_reason = "SCAN_TURN_FAILED"
                recorder.event(
                    time.monotonic(), "ERROR", finish_reason
                )
                break

            ranges, open_dirs, arrival_direction = scan
            recorder.event(
                time.monotonic(),
                "SCAN",
                "four-direction ToF scan",
                logical_node=current,
                open_directions=sorted(open_dirs),
                ranges_cm={
                    str(k): v for k, v in sorted(ranges.items())
                },
            )

            preference = [
                arrival_direction,
                (arrival_direction - 1) % 4,
                (arrival_direction + 1) % 4,
                (arrival_direction + 2) % 4,
            ]

            chosen = None
            for direction in preference:
                if direction not in open_dirs:
                    continue
                nxt = _neighbor(current, direction)
                if not _inside_working_canvas(nxt, config):
                    continue
                if nxt not in visited:
                    chosen = direction
                    break

            if chosen is not None:
                nxt = _neighbor(current, chosen)
                recorder.event(
                    time.monotonic(),
                    "EXPLORE",
                    "move to unvisited logical node",
                    from_node=current,
                    to_node=nxt,
                    direction=DIR_NAME[chosen],
                )
                ok, reason = _move_one_step(
                    chassis,
                    pose,
                    heading,
                    sensors,
                    grid,
                    recorder,
                    config,
                    float(raw_start_x),
                    float(raw_start_y),
                    chosen,
                )
                if not ok:
                    finish_reason = "UNEXPECTED_MOVE_FAILURE_" + reason
                    recorder.event(
                        time.monotonic(),
                        "ERROR",
                        finish_reason,
                        from_node=current,
                        attempted_node=nxt,
                    )
                    break

                parent_stack.append(current)
                current = nxt
                visited.add(current)
                moves += 1
                continue

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
            back_direction = _direction_to(current, parent)
            if back_direction is None:
                finish_reason = "DFS_STACK_ERROR"
                recorder.event(
                    time.monotonic(), "ERROR", finish_reason
                )
                break

            recorder.event(
                time.monotonic(),
                "BACKTRACK",
                "return to parent logical node",
                from_node=current,
                to_node=parent,
                direction=DIR_NAME[back_direction],
            )
            ok, reason = _move_one_step(
                chassis,
                pose,
                heading,
                sensors,
                grid,
                recorder,
                config,
                float(raw_start_x),
                float(raw_start_y),
                back_direction,
            )
            if not ok:
                finish_reason = "BACKTRACK_FAILURE_" + reason
                recorder.event(
                    time.monotonic(), "ERROR", finish_reason
                )
                break

            current = parent
            moves += 1

        else:
            finish_reason = "MAX_MOVES_REACHED"
            recorder.event(
                time.monotonic(),
                "FINISH",
                finish_reason,
                visited_nodes=len(visited),
                moves=moves,
            )

    except KeyboardInterrupt:
        finish_reason = "USER_STOP"
        recorder.event(
            time.monotonic(), "FINISH", finish_reason
        )

    except Exception as exc:
        finish_reason = "ERROR: {}".format(exc)
        recorder.event(
            time.monotonic(), "ERROR", str(exc)
        )
        raise

    finally:
        if chassis is not None:
            try:
                stop_chassis(chassis)
            except Exception:
                pass

        end_pose = None
        try:
            if "pose" in locals():
                x, y = pose.get_xy()
                yaw = pose.get_yaw()
                if x is not None and y is not None and yaw is not None:
                    end_pose = (
                        float(x) - float(raw_start_x),
                        float(y) - float(raw_start_y),
                        normalize_angle_deg(
                            float(yaw) - float(raw_start_yaw)
                        ),
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
            ep_robot.close()
        except Exception:
            pass

        run_dir = recorder.export(
            grid,
            reason=finish_reason,
            start_pose=start_pose,
            end_pose=end_pose,
        )
        print("Classwork 8 results saved to: {}".format(run_dir))

    return run_dir
