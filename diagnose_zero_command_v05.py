"""Isolate physical left yaw after starting RoboMaster SDK (no gimbal motion).

Does setting FREE mode alone cause creep? Does a single zero-speed command
cause it? Does SDK timeout=0.2 cause it? Does repeating the zero command
cause it? Only commands x=y=z=0, never commands nonzero wheel motion.
Run with robot on a clear supervised floor and a visual nose reference.
"""
import argparse
import math
import sys
import threading
import time

from diagnose_chassis_drift_v05 import Telemetry, _fmt
from robomaster import robot


def angle_change(initial, current):
    """Shortest signed chassis yaw change in degrees."""
    return (float(current) - float(initial) + 180.0) % 360.0 - 180.0


def sample_stage(label, seconds, telemetry, t0, initial_yaw, stop, repeat_zero=None):
    print("[ZERO_STAGE_START] {} duration={:.1f}s".format(label, seconds), flush=True)
    began = time.monotonic()
    baseline = None
    next_zero = began
    while not stop.is_set() and time.monotonic() - began < seconds:
        now = time.monotonic()
        if repeat_zero is not None and now >= next_zero:
            ack = repeat_zero()
            print("[ZERO_REPEAT] stage={} result={!r}".format(label, ack), flush=True)
            next_zero = now + 0.20
            if ack is False:
                print("[ABORT] SDK explicitly returned False for zero command.", flush=True)
                stop.set()
                break
        v = telemetry.snapshot()
        yaw = v["yaw"]
        if yaw is not None and baseline is None:
            baseline = yaw
        phase_delta = None if yaw is None or baseline is None else angle_change(baseline, yaw)
        overall_delta = None if yaw is None else angle_change(initial_yaw, yaw)
        print(
            "[ZERO_TRACE] t={:.2f} stage={} yaw={} phase_delta={} "
            "overall_delta={} yaw_age={} gyro_z={} esc_rpm={} "
            "gimbal_relative={} mode={}".format(
                now - t0, label, _fmt(yaw), _fmt(phase_delta),
                _fmt(overall_delta), _fmt(v["age"]),
                _fmt(v["gyro_z"]), repr(v["esc"]),
                _fmt(v["gimbal_relative"]), repr(v["mode"])
            ),
            flush=True,
        )
        if overall_delta is not None and abs(overall_delta) > 2.0:
            print(
                "[ABORT] Chassis yaw changed >2 deg since baseline; "
                "stop trial and inspect physical wheel/nose motion.", flush=True,
            )
            stop.set()
            break
        stop.wait(0.25)
    print("[ZERO_STAGE_END] {}".format(label), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conn", choices=("ap", "sta", "rndis"), default="ap")
    args = parser.parse_args()
    ep = robot.Robot()
    chassis = None
    telemetry = Telemetry()
    stop = threading.Event()
    subs = []
    t0 = time.monotonic()
    try:
        print(
            "[ZERO_TEST] No gimbal motion or nonzero chassis commands; "
            "keep hands at emergency stop.", flush=True,
        )
        ack = ep.initialize(conn_type=args.conn)
        print("[CONNECT] initialize={!r}".format(ack), flush=True)
        if not ack:
            return
        chassis = ep.chassis
        for name, subscribe, unsubscribe in (
            ("attitude", lambda: chassis.sub_attitude(
                freq=20, callback=telemetry.on_attitude), chassis.unsub_attitude),
            ("esc", lambda: chassis.sub_esc(
                freq=20, callback=telemetry.on_esc), chassis.unsub_esc),
            ("imu", lambda: chassis.sub_imu(
                freq=20, callback=telemetry.on_imu), chassis.unsub_imu),
            ("mode", lambda: chassis.sub_mode(
                freq=5, callback=telemetry.on_mode), chassis.unsub_mode),
        ):
            try:
                ok = subscribe()
                print("[SUB] {}={!r}".format(name, ok), flush=True)
                if ok:
                    subs.append((name, unsubscribe))
            except Exception as exc:
                print("[SUB_WARN] {} {}".format(name, exc), flush=True)
        deadline = time.monotonic() + 2.0
        while telemetry.snapshot()["yaw"] is None and time.monotonic() < deadline:
            stop.wait(0.05)
        initial = telemetry.snapshot()["yaw"]
        if initial is None or not math.isfinite(initial):
            print("[ABORT] No valid chassis yaw feedback.", flush=True)
            return
        print("[ZERO_TEST] baseline_yaw={:+.2f}".format(initial), flush=True)
        # DO NOT issue any chassis speed command during these first two stages.
        sample_stage("CONNECTED_NO_CHASSIS_COMMAND", 4.0,
                     telemetry, t0, initial, stop)
        if stop.is_set():
            return
        ok = ep.set_robot_mode(mode=robot.FREE)
        print("[MODE] set_robot_mode(FREE)={!r}".format(ok), flush=True)
        if not ok:
            return
        print("[MODE] get_robot_mode()={!r}".format(ep.get_robot_mode()), flush=True)
        sample_stage("FREE_NO_CHASSIS_COMMAND", 5.0,
                     telemetry, t0, initial, stop)
        if stop.is_set():
            return
        ok = chassis.drive_speed(x=0.0, y=0.0, z=0.0)
        print("[ZERO_SEND] no_timeout result={!r}".format(ok), flush=True)
        if ok is False:
            return
        sample_stage("FREE_SINGLE_ZERO_NO_TIMEOUT", 5.0,
                     telemetry, t0, initial, stop)
        if stop.is_set():
            return
        ok = chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
        print("[ZERO_SEND] timeout_0.2 result={!r}".format(ok), flush=True)
        if ok is False:
            return
        sample_stage("FREE_SINGLE_ZERO_WITH_TIMEOUT", 5.0,
                     telemetry, t0, initial, stop)
        if stop.is_set():
            return
        sample_stage(
            "FREE_PERIODIC_ZERO_WITH_TIMEOUT", 5.0,
            telemetry, t0, initial, stop,
            repeat_zero=lambda: chassis.drive_speed(
                x=0.0, y=0.0, z=0.0, timeout=0.2
            ),
        )
    except KeyboardInterrupt:
        print("[ABORT] Operator keyboard interrupt.", flush=True)
        stop.set()
    finally:
        if chassis is not None:
            try:
                chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
                print("[ZERO_TEST] Final chassis zero sent.", flush=True)
            except Exception as exc:
                print("[STOP_WARN] {}".format(exc), flush=True)
        for name, unsub in reversed(subs):
            try:
                unsub()
            except Exception as exc:
                print("[UNSUB_WARN] {} {}".format(name, exc), flush=True)
        try:
            ep.close()
        except Exception as exc:
            print("[CLOSE_WARN] {}".format(exc), flush=True)
        print("[ZERO_TEST] Complete; include physical nose/wheel observations.", flush=True)


if __name__ == "__main__":
    main()
