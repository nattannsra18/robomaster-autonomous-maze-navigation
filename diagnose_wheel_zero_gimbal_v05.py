"""Test bounded Gimbal yaw with direct wheel-zero hold: NEVER chassis.drive_speed.

Previous real-robot tests: SDK speed-mode zero gave left physical creep and
wheel-mode zero held chassis yaw at 74.01-74.02 deg. This probe tests whether
a short gimbal sweep also preserves wheel-zero stationarity.

No translation, chassis turn, or nonzero wheel RPM is commanded. Use an open
floor and an operator ready to stop the robot if physical wheels turn.
"""
import argparse
import logging
import math
import threading
import time

from diagnose_zero_command_v05 import (
    Telemetry, UnregisteredTelemetryNoiseFilter, _fmt, angle_change,
)
from robomaster import logger as sdk_logger, robot


def _wheel_zero(chassis):
    return chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)


def _observe(name, duration, telem, chassis_baseline, started, stop):
    print("[WZG_STAGE_START] {}".format(name), flush=True)
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and not stop.is_set():
        v = telem.snapshot()
        body = v["yaw"]
        change = None if body is None else angle_change(chassis_baseline, body)
        print(
            "[WZG_TRACE] t={:.2f} stage={} chassis_yaw={} chassis_delta={} "
            "yaw_age={} gimbal_rel={} gimbal_ground={} esc_rpm={} "
            "sdk_mode={}".format(
                time.monotonic() - started, name, _fmt(body), _fmt(change),
                _fmt(v["age"]), _fmt(v["gimbal_relative"]),
                _fmt(v["gimbal_ground"]), repr(v["esc"]), repr(v["mode"])
            ), flush=True,
        )
        if change is None or v["age"] is None or v["age"] > 0.5:
            print("[WZG_ABORT] Chassis yaw feedback missing/stale.", flush=True)
            stop.set()
            break
        if abs(change) > 1.0:
            print("[WZG_ABORT] Chassis yaw moved >1 degree.", flush=True)
            stop.set()
            break
        stop.wait(0.25)
    print("[WZG_STAGE_END] {}".format(name), flush=True)


def _sweep(gimbal, telem, chassis_baseline, started, stop):
    snap = telem.snapshot()
    initial = snap["gimbal_relative"]
    if initial is None or not math.isfinite(initial):
        print("[WZG_ABORT] No valid gimbal relative yaw.", flush=True)
        return False
    speed = +30.0 if initial < -5.0 else -30.0 if initial > 5.0 else +30.0
    sign = math.copysign(1.0, speed)
    started_move = time.monotonic()
    sent = 0
    last_print = -1.0
    print(
        "[WZG_SWEEP] initial={:+.2f} speed={:+.1f} dps limit=12deg "
        "timeout=2sec commands=30ms".format(initial, speed), flush=True,
    )
    try:
        while not stop.is_set() and time.monotonic() - started_move < 2.0:
            now = time.monotonic()
            v = telem.snapshot()
            rel = v["gimbal_relative"]
            body = v["yaw"]
            stamp = telem.stamps.get("gimbal_relative")
            if (rel is None or body is None or stamp is None or
                    now - stamp > 0.5 or v["age"] is None or v["age"] > 0.5):
                print("[WZG_ABORT] Stale or missing live feedback.", flush=True)
                stop.set()
                return False
            inward = (rel - initial) * sign
            body_delta = angle_change(chassis_baseline, body)
            if now - last_print >= 0.20:
                print(
                    "[WZG_SWEEP_TRACE] t={:.2f} gimbal_rel={:+.2f} "
                    "inward={:+.2f} chassis_delta={:+.2f} "
                    "esc={} sdk_mode={} commands={}".format(
                        now - started_move, rel, inward, body_delta,
                        repr(v["esc"]), repr(v["mode"]), sent
                    ), flush=True,
                )
                last_print = now
            if abs(body_delta) > 1.0:
                print("[WZG_ABORT] Physical chassis yaw threshold reached.", flush=True)
                stop.set()
                return False
            if inward >= 12.0:
                print("[WZG_SWEEP_OK] Gimbal verified moving inward.", flush=True)
                return True
            if inward < -1.0 or (now - started_move > 0.8 and inward < 1.0):
                print("[WZG_ABORT] Gimbal absent/opposite feedback.", flush=True)
                stop.set()
                return False
            result = gimbal.drive_speed(pitch_speed=0.0, yaw_speed=speed)
            sent += 1
            if result is False:
                print("[WZG_ABORT] SDK explicitly returned False for gimbal.", flush=True)
                stop.set()
                return False
            # SDK asynchronous gimbal speed command may return None normally.
            stop.wait(0.03)
        print("[WZG_ABORT] Gimbal sweep timed out.", flush=True)
        stop.set()
        return False
    finally:
        gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conn", choices=("ap", "sta", "rndis"), default="ap")
    args = parser.parse_args()
    noise = UnregisteredTelemetryNoiseFilter()
    sdk_logger.setLevel(logging.WARNING)
    sdk_logger.addFilter(noise)
    ep = robot.Robot()
    chassis = None
    gimbal = None
    telem = Telemetry()
    stop = threading.Event()
    subscriptions = []
    started = time.monotonic()
    try:
        print("[WZG] Only wheel-zero (all 4 wheels) and small Gimbal yaw.", flush=True)
        ok = ep.initialize(conn_type=args.conn)
        print("[CONNECT] initialize={!r}".format(ok), flush=True)
        if not ok:
            return
        chassis, gimbal = ep.chassis, ep.gimbal
        for name, sub, unsub in (
            ("attitude", lambda: chassis.sub_attitude(
                freq=20, callback=telem.on_attitude), chassis.unsub_attitude),
            ("esc", lambda: chassis.sub_esc(
                freq=20, callback=telem.on_esc), chassis.unsub_esc),
            ("imu", lambda: chassis.sub_imu(
                freq=20, callback=telem.on_imu), chassis.unsub_imu),
            ("mode", lambda: chassis.sub_mode(
                freq=5, callback=telem.on_mode), chassis.unsub_mode),
            ("gimbal", lambda: gimbal.sub_angle(
                freq=20, callback=telem.on_gimbal), gimbal.unsub_angle),
        ):
            try:
                result = sub()
                print("[SUB] {}={!r}".format(name, result), flush=True)
                if result:
                    subscriptions.append((name, unsub))
            except Exception as exc:
                print("[SUB_WARN] {}: {}".format(name, exc), flush=True)
        deadline = time.monotonic() + 2.0
        while telem.snapshot()["yaw"] is None and time.monotonic() < deadline:
            stop.wait(0.05)
        baseline = telem.snapshot()["yaw"]
        if baseline is None or not math.isfinite(baseline):
            print("[WZG_ABORT] No chassis baseline.", flush=True)
            return
        print("[BASELINE] chassis={:+.2f}".format(baseline), flush=True)
        _observe("CONNECTED_NO_DRIVE", 3.0, telem, baseline, started, stop)
        if stop.is_set():
            return
        ok = ep.set_robot_mode(mode=robot.FREE)
        print("[FREE] result={!r} readback={!r}".format(
            ok, ep.get_robot_mode()), flush=True)
        if not ok:
            return
        _observe("FREE_NO_DRIVE", 3.0, telem, baseline, started, stop)
        if stop.is_set():
            return
        ok = _wheel_zero(chassis)
        print("[WHEEL_ZERO] result={!r}".format(ok), flush=True)
        if ok is not True:
            print("[WZG_ABORT] Zero wheel command not ACKed.", flush=True)
            return
        _observe("WHEEL_ZERO_BEFORE_GIMBAL", 2.0, telem, baseline, started, stop)
        if stop.is_set():
            return
        resume = gimbal.resume()
        print("[GIMBAL] resume={!r}".format(resume), flush=True)
        if resume is False:
            print("[WZG_ABORT] Resume rejected.", flush=True)
            return
        stop.wait(0.30)
        if not _sweep(gimbal, telem, baseline, started, stop):
            return
        _observe("WHEEL_ZERO_AFTER_GIMBAL_STOP", 6.0,
                 telem, baseline, started, stop)
    except KeyboardInterrupt:
        stop.set()
        print("[WZG_ABORT] Operator interruption.", flush=True)
    finally:
        if gimbal is not None:
            try:
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
            except Exception as exc:
                print("[STOP_WARN] gimbal {}".format(exc), flush=True)
        if chassis is not None:
            try:
                print("[FINAL_WHEEL_ZERO] result={!r}".format(_wheel_zero(chassis)),
                      flush=True)
            except Exception as exc:
                print("[STOP_WARN] chassis {}".format(exc), flush=True)
        for name, unsub in reversed(subscriptions):
            try:
                unsub()
            except Exception as exc:
                print("[UNSUB_WARN] {} {}".format(name, exc), flush=True)
        try:
            ep.close()
        except Exception as exc:
            print("[CLOSE_WARN] {}".format(exc), flush=True)
        sdk_logger.removeFilter(noise)
        print("[SDK_LOG] Suppressed {} unrelated 0x24/0x21 warnings.".format(
            noise.suppressed), flush=True)
        print("[WZG] Complete. Note physical wheel/nose motion.", flush=True)


if __name__ == "__main__":
    main()
