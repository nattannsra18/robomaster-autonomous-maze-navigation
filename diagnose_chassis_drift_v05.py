"""Stationary V05 yaw-drift experiment: NO translational or chassis-turn commands.

RoboMaster EP on clear, supervised floor. Run without other controller apps.
The only chassis drive_speed call in this file commands x=y=z=0. Gimbal
yaw motion is bounded to 12 degrees, low speed and optional. Ctrl+C stops both modules.

Stages separate connection alone, FREE-mode transition, SDK input overlay,
and the effect of short gimbal yaw sweeps. ESC wheel RPM, chassis yaw,
gyro_z, gimbal relative/ground yaw and reported mode are sampled together.
"""

import argparse
import math
import sys
import threading
import time
import types


def _prepare_optional_media_codec():
    try:
        __import__("libmedia_codec")
    except ModuleNotFoundError:
        codec = types.ModuleType("libmedia_codec")

        class H264Decoder:
            def decode(self, _data):
                return []

        class OpusDecoder:
            def decode(self, _data):
                return None

        codec.H264Decoder = H264Decoder
        codec.OpusDecoder = OpusDecoder
        sys.modules["libmedia_codec"] = codec


_prepare_optional_media_codec()
from robomaster import robot


class Telemetry:
    def __init__(self):
        self.lock = threading.RLock()
        self.yaw = None
        self.gyro_z = None
        self.esc = None
        self.gimbal_relative = None
        self.gimbal_ground = None
        self.mode = None
        self.stamps = {}

    def _update(self, field, value):
        with self.lock:
            setattr(self, field, value)
            self.stamps[field] = time.monotonic()

    def on_attitude(self, data):
        if data is not None and len(data) >= 1:
            self._update("yaw", float(data[0]))

    def on_imu(self, data):
        if data is not None and len(data) >= 6:
            self._update("gyro_z", float(data[5]))

    def on_esc(self, data):
        if data is not None and len(data) >= 1:
            speeds = data[0]
            if isinstance(speeds, (list, tuple)) and len(speeds) == 4:
                self._update("esc", tuple(round(float(v), 1) for v in speeds))

    def on_gimbal(self, data):
        if data is not None and len(data) >= 2:
            with self.lock:
                self.gimbal_relative = float(data[1])
                self.gimbal_ground = (
                    float(data[3]) if len(data) >= 4 else None
                )
                now = time.monotonic()
                self.stamps["gimbal_relative"] = now
                self.stamps["gimbal_ground"] = now

    def on_mode(self, data):
        self._update("mode", data)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            age = (
                None if "yaw" not in self.stamps
                else now - self.stamps["yaw"]
            )
            return {
                "yaw": self.yaw,
                "age": age,
                "gyro_z": self.gyro_z,
                "esc": self.esc,
                "gimbal_relative": self.gimbal_relative,
                "gimbal_ground": self.gimbal_ground,
                "mode": self.mode,
            }


def _fmt(value):
    if value is None:
        return "---"
    if isinstance(value, (int, float)):
        return "{:+.2f}".format(value)
    return repr(value)


def _stop_both(chassis, gimbal):
    if gimbal is not None:
        try:
            gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
        except Exception as exc:
            print("[STOP_ERROR] gimbal: {}".format(exc), flush=True)
    if chassis is not None:
        try:
            chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
        except Exception as exc:
            print("[STOP_ERROR] chassis: {}".format(exc), flush=True)


def _stage(name, duration, telem, started, stop, abort_on_drift=False):
    print("[STAGE_START] {} ({}s)".format(name, duration), flush=True)
    first = None
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and not stop.is_set():
        v = telem.snapshot()
        if first is None and v["yaw"] is not None:
            first = v["yaw"]
        yaw_delta = (
            None if first is None or v["yaw"] is None
            else v["yaw"] - first
        )
        esc = v["esc"]
        esc_text = (
            "---" if esc is None else
            "/".join("{:+.1f}".format(w) for w in esc)
        )
        print(
            "[DRIFT_DIAG] t={:.2f} stage={} yaw={} delta={} yaw_age={} "
            "gyro_z={} esc_rpm={} gimbal_rel={} gimbal_ground={} mode={}".format(
                time.monotonic() - started, name, _fmt(v["yaw"]),
                _fmt(yaw_delta), _fmt(v["age"]), _fmt(v["gyro_z"]),
                esc_text, _fmt(v["gimbal_relative"]),
                _fmt(v["gimbal_ground"]), repr(v["mode"])
            ), flush=True,
        )
        if abort_on_drift and yaw_delta is not None and abs(yaw_delta) > 2.0:
            print(
                "[ABORT] Unexpected chassis yaw change >2 degrees during {}. "
                "Do not continue into the gimbal test.".format(name),
                flush=True,
            )
            stop.set()
            break
        stop.wait(0.25)
    print("[STAGE_END] {}".format(name), flush=True)


def _gimbal_closed_loop_probe(chassis, gimbal, telem, started, stop):
    """Repeat SDK yaw commands as the WORKING mapper does (about 30 ms).

    No nonzero chassis commands. A pulse is only a successful test when fresh
    angle feedback shows at least 3 degrees of actual inward movement.
    Stop at 12 degrees travel or after 2 seconds; a lack of response within
    0.8 seconds fails. Abort on >1 degree chassis yaw rotation.
    """
    before = telem.snapshot()
    initial = before["gimbal_relative"]
    chassis_initial = before["yaw"]
    if (initial is None or chassis_initial is None or
            not math.isfinite(initial) or not math.isfinite(chassis_initial)):
        print("[GIMBAL_FAIL] Missing initial yaw feedback.", flush=True)
        return False
    stamp = telem.stamps.get("gimbal_relative")
    if stamp is None or time.monotonic() - stamp > 0.5:
        print("[GIMBAL_FAIL] Initial gimbal feedback is stale.", flush=True)
        return False

    speed = +30.0 if initial < -5.0 else -30.0 if initial > 5.0 else +30.0
    sign = math.copysign(1.0, speed)
    began = time.monotonic()
    last_log = -1.0
    sends = 0
    reached = False
    print(
        "[GIMBAL_PROBE] CLOSED_LOOP initial={:+.2f} direction={} "
        "speed={:+.1f} deg/s, command every 0.03s; chassis x=y=z=0".format(
            initial, "RIGHT" if speed > 0.0 else "LEFT", speed
        ), flush=True,
    )
    try:
        while not stop.is_set() and time.monotonic() - began < 2.0:
            now = time.monotonic()
            feedback = telem.snapshot()
            actual = feedback["gimbal_relative"]
            body = feedback["yaw"]
            age = telem.stamps.get("gimbal_relative")
            if (actual is None or age is None or
                    now - age > 0.5 or not math.isfinite(actual)):
                print("[GIMBAL_FAIL] Gimbal feedback missing/stale.", flush=True)
                return False
            delta = (actual - initial) * sign
            body_delta = (
                None if body is None else
                (body - chassis_initial + 180.0) % 360.0 - 180.0
            )
            if now - last_log >= 0.20:
                print(
                    "[GIMBAL_CONTROL] t={:.2f} rel={:+.2f} inward={:+.2f} "
                    "body_delta={} esc_rpm={} sends={}".format(
                        now - began, actual, delta, _fmt(body_delta),
                        feedback["esc"], sends
                    ), flush=True,
                )
                last_log = now
            if body_delta is None or abs(body_delta) > 1.0:
                print(
                    "[ABORT] Chassis yaw changed more than 1 degree "
                    "during stationary gimbal probe.", flush=True,
                )
                stop.set()
                return False
            if delta >= 12.0:
                reached = True
                break
            if delta < -1.0:
                print("[GIMBAL_FAIL] Moving outward/opposite feedback.", flush=True)
                return False
            if now - began > 0.8 and delta < 1.0:
                print(
                    "[GIMBAL_FAIL] No yaw response despite repeated commands.",
                    flush=True,
                )
                return False
            result = gimbal.drive_speed(pitch_speed=0.0, yaw_speed=speed)
            sends += 1
            if result is False:
                print("[GIMBAL_FAIL] SDK explicitly rejected speed command.", flush=True)
                return False
            # None is the official SDK fire-and-forget return, not rejection.
            stop.wait(0.03)
    finally:
        gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
        chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
    final = telem.snapshot()["gimbal_relative"]
    inward = None if final is None else (final - initial) * sign
    print(
        "[GIMBAL_PROBE] sent={} initial={:+.2f} final={} inward={} "
        "reached_12deg={}".format(
            sends, initial, _fmt(final), _fmt(inward), reached
        ), flush=True,
    )
    success = inward is not None and inward >= 3.0 and not stop.is_set()
    if not success:
        print("[GIMBAL_FAIL] Measured inward travel <3 degrees.", flush=True)
    return success


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conn", default="ap", choices=("ap", "sta", "rndis"))
    parser.add_argument(
        "--with-gimbal", action="store_true",
        help="test a supervised bounded inward Gimbal yaw command repeated every 30ms with real angle feedback",
    )
    args = parser.parse_args()
    ep = robot.Robot()
    chassis = None
    gimbal = None
    telem = Telemetry()
    stop = threading.Event()
    subscriptions = []
    started = time.monotonic()
    try:
        print("[DIAG] Connecting: no chassis movement will be requested.", flush=True)
        ok = ep.initialize(conn_type=args.conn)
        print("[DIAG] initialize={!r}".format(ok), flush=True)
        if not ok:
            raise RuntimeError("Robot connection failed")
        chassis, gimbal = ep.chassis, ep.gimbal

        for name, subscribe, unsubscribe in (
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
                subscribed = bool(subscribe())
                print("[SUB] {}={!r}".format(name, subscribed), flush=True)
                if subscribed:
                    subscriptions.append((name, unsubscribe))
            except Exception as exc:
                print("[SUB] {} unavailable: {}".format(name, exc), flush=True)

        # No zero-speed command yet: identify side effects of SDK connection.
        _stage("CONNECTED_NO_COMMAND", 4.0, telem, started, stop, True)
        if stop.is_set():
            return
        mode_ok = ep.set_robot_mode(mode=robot.FREE)
        print("[MODE] set FREE={!r}".format(mode_ok), flush=True)
        if not mode_ok:
            raise RuntimeError("FREE mode did not acknowledge")
        _stop_both(chassis, gimbal)
        _stage("FREE_WITH_ZERO_CHASSIS", 4.0, telem, started, stop, True)
        if stop.is_set():
            return
        fusion_ok = chassis.stick_overlay(0)
        print("[OVERLAY] stick_overlay(0)={!r}".format(fusion_ok), flush=True)
        _stop_both(chassis, gimbal)
        _stage("OVERLAY_OFF_WITH_ZERO_CHASSIS", 4.0, telem, started, stop, True)
        if stop.is_set() or not args.with_gimbal:
            return

        # The main mapper repeatedly drives yaw every 30 ms using live
        # gimbal feedback. Use that same streaming control pattern here.
        # Earlier probes sent just ONE command and then slept; that did not
        # reproduce the working mapping controller.
        _stop_both(chassis, gimbal)
        gimbal_ready = gimbal.resume()
        print("[GIMBAL_PROBE] resume={!r}".format(gimbal_ready), flush=True)
        if gimbal_ready is False:
            print("[GIMBAL_FAIL] resume returned False.", flush=True)
            return
        if not _gimbal_closed_loop_probe(chassis, gimbal, telem, started, stop):
            return
        _stage("GIMBAL_AFTER_STOP", 3.0, telem, started, stop, True)
    except KeyboardInterrupt:
        print("[DIAG] Manual stop.", flush=True)
        stop.set()
    finally:
        _stop_both(chassis, gimbal)
        for name, unsubscribe in reversed(subscriptions):
            try:
                unsubscribe()
            except Exception as exc:
                print("[UNSUB] {}: {}".format(name, exc), flush=True)
        try:
            ep.close()
        except Exception as exc:
            print("[CLOSE] {}".format(exc), flush=True)
        print("[DIAG] Finished. Record physical nose/wheel motion per stage.",
              flush=True)


if __name__ == "__main__":
    main()
