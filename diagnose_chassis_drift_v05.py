"""Stationary V05 yaw-drift experiment: NO translational or chassis-turn commands.

RoboMaster EP on clear, supervised floor. Run without other controller apps.
The only chassis drive_speed call in this file commands x=y=z=0. Gimbal
sweeps are deliberately small, slow and optional. Ctrl+C stops both modules.

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conn", default="ap", choices=("ap", "sta", "rndis"))
    parser.add_argument(
        "--with-gimbal", action="store_true",
        help="resume gimbal and test ONE supervised 1.2-second 30 deg/s inward yaw pulse; abort if feedback does not move",
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

        # Previous probe started around -269 deg: a LEFT pulse was directed
        # farther into the extreme. First resume and choose only an INWARD
        # pulse. Do not use action.wait_for_completed() on this robot.
        _stop_both(chassis, gimbal)
        gimbal_ready = gimbal.resume()
        print("[GIMBAL_PROBE] resume={!r}".format(gimbal_ready), flush=True)
        if not gimbal_ready:
            print("[GIMBAL_FAIL] resume command rejected; stopping.", flush=True)
            return
        time.sleep(0.30)
        before = telem.snapshot()
        initial = before["gimbal_relative"]
        if initial is None or not math.isfinite(initial):
            print("[GIMBAL_FAIL] no valid yaw feedback; stopping.", flush=True)
            return
        age = telem.stamps.get("gimbal_relative")
        if age is None or time.monotonic() - age > 0.6:
            print("[GIMBAL_FAIL] stale gimbal feedback; stopping.", flush=True)
            return
        # Away from mechanical extremes. -269 -> positive (RIGHT).
        speed = +30.0 if initial < -5.0 else -30.0 if initial > 5.0 else +30.0
        print(
            "[GIMBAL_PROBE] initial={:+.2f} direction={} command={:+.1f} deg/s "
            "duration=1.2s; no chassis movement commanded.".format(
                initial, "RIGHT" if speed > 0 else "LEFT", speed
            ),
            flush=True,
        )
        command_ok = gimbal.drive_speed(pitch_speed=0.0, yaw_speed=speed)
        print("[GIMBAL_PROBE] drive_speed result={!r}".format(command_ok), flush=True)
        if not command_ok:
            print("[GIMBAL_FAIL] drive_speed rejected; stopping.", flush=True)
            return
        try:
            _stage("GIMBAL_INWARD_PULSE", 1.2, telem, started, stop, True)
        finally:
            gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
            chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
        time.sleep(0.20)
        final = telem.snapshot()["gimbal_relative"]
        delta = None if final is None else float(final) - float(initial)
        print(
            "[GIMBAL_PROBE] final={} moved_deg={}".format(
                _fmt(final), _fmt(delta)
            ), flush=True,
        )
        if delta is None or delta * speed <= 3.0:
            print(
                "[GIMBAL_FAIL] no verified inward gimbal motion (>3 deg). "
                "Do not interpret the pulse as a successful gimbal test; "
                "inspect gimbal state, obstruction and angle feedback.",
                flush=True,
            )
            return
        if stop.is_set():
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
