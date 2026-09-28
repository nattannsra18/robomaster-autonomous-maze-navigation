"""Stationary comparison: one ZERO wheel-RPM command, never drive_speed.

Previous logs: Robot FREE alone held yaw, but drive_speed(0,0,0) changed
Chassis SDK mode 8 -> 1 and was followed by physical left creep. The
RoboMaster SDK declares its speed command PUSH/no-ACK, so returned False
is a wrapper artifact rather than evidence of failed transport. This
experiment uses a DIFFERENT zero-motion primitive (drive_wheels 0,0,0,0).
No gimbal motion, no translation or turning request. A physical observation
is required; feedback alone cannot establish wheel movement.

Place RoboMaster on open floor with nose reference; keep emergency stop
available. Do not run maze control simultaneously.
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


def observe(label, seconds, telem, baseline, t0, stop):
    print("[WHEEL_ZERO_STAGE_START] {}".format(label), flush=True)
    start = time.monotonic()
    while not stop.is_set() and time.monotonic() - start < seconds:
        v = telem.snapshot()
        yaw = v["yaw"]
        delta = None if yaw is None else angle_change(baseline, yaw)
        print(
            "[WHEEL_ZERO_TRACE] t={:.2f} stage={} yaw={} overall_delta={} "
            "yaw_age={} gyro_z={} esc_rpm={} chassis_sdk_mode={}".format(
                time.monotonic() - t0, label, _fmt(yaw), _fmt(delta),
                _fmt(v["age"]), _fmt(v["gyro_z"]), repr(v["esc"]),
                repr(v["mode"]),
            ), flush=True,
        )
        if delta is None or v["age"] is None or v["age"] > 0.5:
            print("[ABORT] Missing or stale chassis attitude.", flush=True)
            stop.set()
            break
        if abs(delta) > 1.0:
            print(
                "[ABORT] Chassis yaw changed more than 1 degree. "
                "Stop the physical test; note nose/wheel motion.",
                flush=True,
            )
            stop.set()
            break
        stop.wait(0.25)
    print("[WHEEL_ZERO_STAGE_END] {}".format(label), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--conn", choices=("ap", "sta", "rndis"), default="ap")
    args = ap.parse_args()
    noise = UnregisteredTelemetryNoiseFilter()
    sdk_logger.setLevel(logging.WARNING)
    sdk_logger.addFilter(noise)
    ep = robot.Robot()
    chassis = None
    subscribed = []
    telem = Telemetry()
    stop = threading.Event()
    t0 = time.monotonic()
    wheel_zero_sent = False
    try:
        print(
            "[WHEEL_ZERO] No gimbal command. The ONLY chassis primitive is "
            "drive_wheels(w1=0,w2=0,w3=0,w4=0).", flush=True,
        )
        ok = ep.initialize(conn_type=args.conn)
        print("[CONNECT] initialize={!r}".format(ok), flush=True)
        if not ok:
            return
        chassis = ep.chassis
        for name, sub, unsub in (
            ("attitude", lambda: chassis.sub_attitude(
                freq=20, callback=telem.on_attitude), chassis.unsub_attitude),
            ("esc", lambda: chassis.sub_esc(
                freq=20, callback=telem.on_esc), chassis.unsub_esc),
            ("imu", lambda: chassis.sub_imu(
                freq=20, callback=telem.on_imu), chassis.unsub_imu),
            ("mode", lambda: chassis.sub_mode(
                freq=5, callback=telem.on_mode), chassis.unsub_mode),
        ):
            try:
                result = sub()
                print("[SUB] {}={!r}".format(name, result), flush=True)
                if result:
                    subscribed.append((name, unsub))
            except Exception as exc:
                print("[SUB_WARN] {}: {}".format(name, exc), flush=True)
        until = time.monotonic() + 2.0
        while telem.snapshot()["yaw"] is None and time.monotonic() < until:
            stop.wait(0.05)
        baseline = telem.snapshot()["yaw"]
        if baseline is None or not math.isfinite(baseline):
            print("[ABORT] Cannot acquire initial chassis yaw.", flush=True)
            return
        print("[BASELINE] yaw={:+.2f}".format(baseline), flush=True)
        observe("CONNECTED_NO_DRIVE", 3.0, telem, baseline, t0, stop)
        if stop.is_set():
            return
        ok = ep.set_robot_mode(mode=robot.FREE)
        print("[FREE] set_robot_mode={!r} readback={!r}".format(
            ok, ep.get_robot_mode()), flush=True)
        if not ok:
            return
        observe("FREE_NO_DRIVE", 5.0, telem, baseline, t0, stop)
        if stop.is_set():
            return
        print("[WHEEL_ZERO_SEND] Four individual wheel targets = 0 rpm.",
              flush=True)
        wheel_zero_sent = True
        result = chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        print("[WHEEL_ZERO_RESULT] {!r}".format(result), flush=True)
        # Wheel-speed proto is an ACK-requesting type, unlike drive_speed
        # PUSH. A False ACK is reported but actual motion is still observed.
        if result is False:
            print("[WHEEL_ZERO_ACK_WARN] Inspect SDK warnings; measuring "
                  "physical result regardless.", flush=True)
        observe("FREE_AFTER_SINGLE_WHEEL_ZERO", 8.0,
                telem, baseline, t0, stop)
    except KeyboardInterrupt:
        stop.set()
        print("[ABORT] Operator interrupted.", flush=True)
    finally:
        if chassis is not None and wheel_zero_sent:
            try:
                result = chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                print("[FINAL_WHEEL_ZERO] result={!r}".format(result),
                      flush=True)
            except Exception as exc:
                print("[STOP_WARN] {}".format(exc), flush=True)
        for name, unsub in reversed(subscribed):
            try:
                unsub()
            except Exception as exc:
                print("[UNSUB_WARN] {} {}".format(name, exc), flush=True)
        try:
            ep.close()
        except Exception as exc:
            print("[CLOSE_WARN] {}".format(exc), flush=True)
        sdk_logger.removeFilter(noise)
        print("[SDK_LOG] Suppressed {} repeated incoming 0x24/0x21 warnings".format(
            noise.suppressed), flush=True)
        print("[WHEEL_ZERO] Complete. Report physical nose and wheel movement.",
              flush=True)


if __name__ == "__main__":
    main()
