from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional, Tuple

try:
    from robomaster import robot
except ModuleNotFoundError:
    robot = None

from robomaster_mission.mission import (
    AFTER_TURN_DELAY_SEC,
    ENABLE_MOTION,
    ESCAPE_FORWARD_SPEED,
    HEADINGS,
    LOOP_DELAY_SEC,
    PRINT_EVERY_SEC,
    SIDE_WARNING_FORWARD_SPEED,
    STOP_FRONT_CM,
    ExitDetector,
    HeadingManager,
    JunctionDetector,
    MazeGraphExplorer,
    Plan,
    PoseTracker,
    WallController,
    align_heading,
    apply_front_corner_ir_guard,
    backtrack_to_opening_center,
    center_front_blocked,
    feedback_turn,
    fmt,
    observed_absolute_openings,
    post_turn_clearance,
    scan_junction,
    stop_chassis,
    wait_for_position,
    wait_for_yaw,
)

from .config import Classwork8Config
from .occupancy_grid import OccupancyGrid
from .reporting import RunRecorder
from .sensors import Classwork8SensorManager


def _sensor_origin(
    x_m: float,
    y_m: float,
    forward_angle_rad: float,
    forward_offset_m: float,
    lateral_offset_m: float,
) -> Tuple[float, float]:
    fx, fy = math.cos(forward_angle_rad), math.sin(forward_angle_rad)
    lx, ly = -fy, fx
    return (
        x_m + fx * forward_offset_m + lx * lateral_offset_m,
        y_m + fy * forward_offset_m + ly * lateral_offset_m,
    )


def _heading_forward_angle(heading_index: int) -> float:
    return {
        0: 0.0,
        1: -math.pi / 2.0,
        2: math.pi,
        3: math.pi / 2.0,
    }[int(heading_index) % 4]


def _update_map(
    grid: OccupancyGrid,
    config: Classwork8Config,
    x_m: float,
    y_m: float,
    heading_index: int,
    front_cm: Optional[float],
    left_cm: Optional[float],
    right_cm: Optional[float],
) -> None:
    forward = _heading_forward_angle(heading_index)

    if front_cm is not None and front_cm >= config.mapping_min_cm:
        origin = _sensor_origin(
            x_m, y_m, forward, config.tof_forward_offset_m, 0.0
        )
        hit = front_cm < config.tof_max_mapping_cm - 1.0
        grid.update_ray(
            *origin,
            forward,
            front_cm / 100.0,
            max_range_m=config.tof_max_mapping_cm / 100.0,
            hit=hit,
        )

    if left_cm is not None and left_cm >= config.mapping_min_cm:
        angle = forward + math.pi / 2.0
        origin = _sensor_origin(
            x_m, y_m, forward, 0.0, config.sharp_lateral_offset_m
        )
        hit = left_cm < config.sharp_max_mapping_cm - 0.5
        grid.update_ray(
            *origin,
            angle,
            left_cm / 100.0,
            max_range_m=config.sharp_max_mapping_cm / 100.0,
            hit=hit,
        )

    if right_cm is not None and right_cm >= config.mapping_min_cm:
        angle = forward - math.pi / 2.0
        origin = _sensor_origin(
            x_m, y_m, forward, 0.0, -config.sharp_lateral_offset_m
        )
        hit = right_cm < config.sharp_max_mapping_cm - 0.5
        grid.update_ray(
            *origin,
            angle,
            right_cm / 100.0,
            max_range_m=config.sharp_max_mapping_cm / 100.0,
            hit=hit,
        )


def _pose_tuple(pose: PoseTracker) -> Optional[Tuple[float, float, float]]:
    x, y = pose.get_xy()
    yaw = pose.get_yaw()
    if x is None or y is None or yaw is None:
        return None
    return float(x), float(y), float(yaw)


def run(config: Optional[Classwork8Config] = None) -> Path:
    config = config or Classwork8Config()
    config.validate()
    if robot is None:
        raise RuntimeError("RoboMaster SDK is not installed")

    ep_robot = robot.Robot()
    chassis = None
    tof_sensor = None
    pose_subscribed = False
    attitude_subscribed = False
    tof_subscribed = False

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
    start_pose = None
    finish_reason = "UNKNOWN"

    try:
        print("Connecting RoboMaster for Classwork 8...")
        ep_robot.initialize(conn_type=config.connection)
        chassis = ep_robot.chassis
        sensor_adapter = ep_robot.sensor_adaptor
        tof_sensor = ep_robot.sensor

        try:
            ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
            ep_robot.gimbal.recenter(
                pitch_speed=100, yaw_speed=100
            ).wait_for_completed()
        except Exception as exc:
            print(f"WARNING: gimbal setup failed: {exc}")

        pose = PoseTracker()
        sensors = Classwork8SensorManager(sensor_adapter, config)
        heading = HeadingManager()
        walls = WallController()
        junctions = JunctionDetector()
        exit_detector = ExitDetector()
        explorer = MazeGraphExplorer()

        tof_subscribed = bool(
            tof_sensor.sub_distance(freq=20, callback=sensors.tof_callback)
        )
        pose_subscribed = bool(
            chassis.sub_position(cs=1, freq=20, callback=pose.position_callback)
        )
        attitude_subscribed = bool(
            chassis.sub_attitude(freq=20, callback=pose.attitude_callback)
        )

        start_x, start_y = wait_for_position(pose)
        start_yaw = wait_for_yaw(pose)
        if not heading.initialize(start_yaw):
            raise RuntimeError("Yaw/attitude is unavailable")
        start_pose = (float(start_x), float(start_y), float(start_yaw))

        start_node = explorer.initialize_start(start_x, start_y)
        start_forward = explorer.absolute_for_relative("FRONT")
        explorer._edge(start_node, start_forward).observed = True
        initial_plan = Plan("FRONT", start_forward, "INITIAL_FORWARD")
        explorer.commit_departure(
            initial_plan, start_xy=(start_x, start_y)
        )
        recorder.event(
            time.monotonic(),
            "START",
            "unknown-world exploration started",
            node=start_node,
        )

        print("============================================================")
        print(" Classwork 8 - Unknown World Mapping + Exploration")
        print("============================================================")
        print("LEFT hub 2 : IR port 1, Sharp port 2")
        print("RIGHT hub 1: IR port 1, Sharp port 2")
        print("Front ToF  : gimbal, CHASSIS_LEAD + recentered")
        print(
            f"Map: {config.map_width_m:.1f} x "
            f"{config.map_height_m:.1f} m @ "
            f"{config.resolution_m:.2f} m/cell"
        )
        print("============================================================")

        last_print = 0.0
        while True:
            now = time.monotonic()
            raw_l, left_cm = sensors.read_left()
            raw_r, right_cm = sensors.read_right()
            front_cm = sensors.get_front_cm()
            ir_left, ir_right = sensors.read_front_corner_ir()
            x, y = pose.get_xy()
            yaw = pose.get_yaw()
            yaw_error = heading.error(yaw)

            if (
                x is not None
                and y is not None
                and (
                    yaw_error is None
                    or abs(yaw_error)
                    <= config.mapping_heading_tolerance_deg
                )
            ):
                _update_map(
                    grid,
                    config,
                    float(x),
                    float(y),
                    explorer.heading_index,
                    front_cm,
                    left_cm,
                    right_cm,
                )

            if (
                config.stop_on_open_exit
                and exit_detector.update(
                    front_cm,
                    left_cm,
                    right_cm,
                    (x, y),
                    ir_left,
                    ir_right,
                )
            ):
                stop_chassis(chassis)
                finish_reason = "MAZE_EXIT_CONFIRMED"
                recorder.event(now, "FINISH", finish_reason)
                break

            event = junctions.update(
                front_cm, left_cm, right_cm, (x, y)
            )
            if event is not None:
                stop_chassis(chassis)
                walls.reset()

                if event.kind == "SIDE_WINDOW":
                    backtrack_to_opening_center(
                        chassis, pose, heading, event.backtrack_m
                    )
                else:
                    center_front_blocked(
                        chassis, sensors, pose, heading
                    )

                scan = scan_junction(sensors, event)
                if None in (
                    scan["front_cm"],
                    scan["left_cm"],
                    scan["right_cm"],
                ):
                    recorder.event(
                        now,
                        "SCAN_REJECTED",
                        "incomplete sensor snapshot",
                    )
                    junctions.lock_here(pose.get_xy())
                    time.sleep(LOOP_DELAY_SEC)
                    continue

                xj, yj = pose.get_xy()
                if xj is None or yj is None:
                    finish_reason = "ODOMETRY_LOST_AT_JUNCTION"
                    recorder.event(now, "ERROR", finish_reason)
                    break

                observed_abs = observed_absolute_openings(
                    explorer, scan
                )
                node_id, is_new, match_reason = explorer.arrive(
                    float(xj), float(yj), observed_abs
                )
                explorer.observe_openings(observed_abs)
                recorder.event(
                    now,
                    "NODE",
                    "new junction" if is_new else "known junction",
                    node=node_id,
                    match=match_reason,
                    x_m=float(xj),
                    y_m=float(yj),
                    openings=sorted(observed_abs),
                )

                if match_reason == "SAME_NODE_RETRIGGER":
                    pending_abs = explorer.pending_abs
                    progress = explorer.pending_progress_m(
                        float(xj), float(yj)
                    )
                    pending_still_open = (
                        pending_abs is not None
                        and pending_abs in observed_abs
                    )
                    pending_unknown = False
                    if (
                        explorer.pending_from is not None
                        and pending_abs is not None
                    ):
                        pending_unknown = (
                            explorer._edge(
                                explorer.pending_from, pending_abs
                            ).target
                            is None
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
                        and progress <= 0.30
                    ):
                        explorer.cancel_pending_as_blocked(
                            "SAME_NODE_HARD_FRONT",
                            float(xj),
                            float(yj),
                        )
                        recorder.event(
                            now,
                            "EDGE_BLOCKED",
                            "false/short exit cancelled",
                            progress_m=progress,
                        )
                        junctions.lock_here(pose.get_xy())
                    else:
                        recorder.event(
                            now,
                            "NODE_RETRIGGER",
                            "same wide junction; continue corridor",
                        )
                        junctions.lock_here(pose.get_xy())
                        time.sleep(LOOP_DELAY_SEC)
                        continue

                plan = explorer.plan(
                    current_observed_abs=observed_abs
                )
                if plan is None:
                    stop_chassis(chassis)
                    finish_reason = "GRAPH_EXPLORATION_COMPLETE"
                    recorder.event(now, "FINISH", finish_reason)
                    break

                recorder.event(
                    now,
                    "PLAN",
                    plan.reason,
                    relative=plan.relative,
                    absolute=plan.absolute,
                    absolute_name=HEADINGS[plan.absolute],
                )

                if not feedback_turn(
                    chassis, sensors, pose, plan.relative
                ):
                    finish_reason = "TURN_FAILED"
                    recorder.event(
                        now,
                        "ERROR",
                        finish_reason,
                        relative=plan.relative,
                    )
                    break

                explorer.commit_departure(
                    plan, start_xy=(float(xj), float(yj))
                )
                heading.set_heading_index(explorer.heading_index)
                align_heading(chassis, pose, heading)
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
                stop_chassis(chassis)
                time.sleep(AFTER_TURN_DELAY_SEC)
                continue

            if front_cm is not None and front_cm <= STOP_FRONT_CM:
                x_cmd = 0.0
                y_cmd = 0.0
                z_cmd = 0.0
                mode = "FRONT_CONFIRM"
            else:
                x_cmd = min(
                    walls.forward_speed(front_cm),
                    config.forward_speed_mps,
                )
                y_cmd, mode = walls.lateral(left_cm, right_cm)

                if mode.startswith("ESCAPE_"):
                    x_cmd = min(x_cmd, ESCAPE_FORWARD_SPEED)
                elif mode.startswith("AVOID_"):
                    x_cmd = min(
                        x_cmd, SIDE_WARNING_FORWARD_SPEED
                    )
                if mode in ("BOTH_TOO_CLOSE", "NO_SIDE_SENSOR"):
                    x_cmd = 0.0

                x_cmd, y_cmd, ir_mode = (
                    apply_front_corner_ir_guard(
                        x_cmd,
                        y_cmd,
                        ir_left,
                        ir_right,
                    )
                )
                if ir_mode:
                    mode = ir_mode

                x_cmd, y_cmd, z_cmd, mode, _ = heading.apply(
                    x_cmd, y_cmd, yaw, mode
                )

            if ENABLE_MOTION:
                chassis.drive_speed(
                    x=x_cmd,
                    y=y_cmd,
                    z=z_cmd,
                    timeout=config.drive_timeout_sec,
                )

            recorder.record_sample(
                now,
                None if x is None else float(x),
                None if y is None else float(y),
                None if yaw is None else float(yaw),
                explorer.heading_index,
                front_cm,
                left_cm,
                right_cm,
                ir_left,
                ir_right,
                mode,
            )

            if now - last_print >= PRINT_EVERY_SEC:
                print(
                    f"ToF:{fmt(front_cm)} | "
                    f"L:{fmt(left_cm)} ADC:{raw_l:4d} | "
                    f"R:{fmt(right_cm)} ADC:{raw_r:4d} | "
                    f"IR-L:{ir_left} IR-R:{ir_right} | "
                    f"Pose:("
                    f"{0.0 if x is None else x:+.2f},"
                    f"{0.0 if y is None else y:+.2f}) | "
                    f"H:{HEADINGS[explorer.heading_index]} | "
                    f"{mode:22s} | "
                    f"Coverage:{grid.coverage_percent():.1f}%"
                )
                last_print = now

            time.sleep(config.loop_delay_sec)

    except KeyboardInterrupt:
        finish_reason = "USER_STOP"
        recorder.event(
            time.monotonic(), "FINISH", finish_reason
        )

    except Exception as exc:
        finish_reason = f"ERROR: {exc}"
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
                end_pose = _pose_tuple(pose)
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
        print(f"Classwork 8 results saved to: {run_dir}")

    return run_dir


def main() -> None:
    run()


if __name__ == "__main__":
    main()
