"""Stationary gimbal level diagnostic for Final Round 1 V05.

No chassis travel, no target aiming, no blaster control. The test turns the
gimbal in the mapping scan sequence and prints both SDK angle axes.
"""

import argparse
import sys
import time
import types


def _prepare_optional_media_codec():
    try:
        __import__("libmedia_codec")
        return
    except ModuleNotFoundError:
        pass

    module = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    module.H264Decoder = H264Decoder
    module.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = module


_prepare_optional_media_codec()

from robomaster import robot

from classwork8.config import Classwork8Config
from classwork8.tof_camera_round1_v05 import (
    DIR_NAME,
    GimbalTracker,
    _point_gimbal,
)


class DummyToFFilter:
    def reset_filters(self):
        pass


def main():
    parser = argparse.ArgumentParser(
        description="Stationary four-way gimbal pitch/yaw verification"
    )
    parser.add_argument(
        "--pitch-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
        help="sign relating SDK pitch_speed to pitch feedback",
    )
    parser.add_argument(
        "--pitch-target",
        type=float,
        default=0.0,
        help="desired relative pitch in degrees (default horizontal 0)",
    )
    args = parser.parse_args()

    config = Classwork8Config()
    config.gimbal_scan_pitch_deg = float(args.pitch_target)
    config.gimbal_pitch_drive_sign = float(args.pitch_sign)
    config.validate()

    ep_robot = None
    chassis = None
    gimbal = None
    subscribed = False
    tracker = GimbalTracker()
    sensors = DummyToFFilter()

    try:
        ep_robot = robot.Robot()
        ok = ep_robot.initialize(conn_type=config.connection)
        if not ok:
            raise RuntimeError("RoboMaster connection failed")

        chassis = ep_robot.chassis
        gimbal = ep_robot.gimbal

        chassis.drive_speed(x=0.0, y=0.0, z=0.0)
        ep_robot.set_robot_mode(mode=robot.FREE)

        subscribed = bool(gimbal.sub_angle(freq=20, callback=tracker.callback))
        if not subscribed:
            raise RuntimeError("Gimbal angle subscription failed")

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            pitch, yaw = tracker.get_angles()
            if pitch is not None and yaw is not None:
                break
            time.sleep(0.03)
        else:
            raise RuntimeError("No gimbal pitch/yaw feedback")

        print("STATIONARY GIMBAL TEST — chassis remains stopped")
        print("Pitch target: {:+.1f} deg | drive sign: {:+.0f}".format(
            config.gimbal_scan_pitch_deg,
            config.gimbal_pitch_drive_sign,
        ))

        # Safe adjacent-yaw scan sequence; avoid direct +180 to -90 wrapping.
        for direction in (0, 1, 2, 1, 0, 3, 0):
            print("[TEST] Pointing {}...".format(DIR_NAME[direction]), flush=True)
            sweep_started = time.monotonic()
            reached = _point_gimbal(
                gimbal, sensors, tracker, direction, config, None
            )
            pitch_samples = tracker.pitch_samples_since(sweep_started)
            if pitch_samples:
                low, high = min(pitch_samples), max(pitch_samples)
                peak = max(
                    abs(value - config.gimbal_scan_pitch_deg)
                    for value in pitch_samples
                )
                print(
                    "[SWEEP] {} pitch min={:+.1f} max={:+.1f} peak_error={:.1f} deg "
                    "({} fresh angle samples)".format(
                        DIR_NAME[direction],
                        low,
                        high,
                        peak,
                        len(pitch_samples),
                    ),
                    flush=True,
                )
            if not reached:
                print(
                    "[TEST] YAW-ONLY FAILED at {}; chassis remains stopped. "
                    "The SWEEP above shows whether pitch drift persisted "
                    "without simultaneous pitch commands.".format(
                        DIR_NAME[direction]
                    ),
                    flush=True,
                )
                raise RuntimeError(
                    "Gimbal yaw-only/level sequence failed at {}".format(
                        DIR_NAME[direction]
                    )
                )
            pitch, yaw = tracker.get_angles()
            print(
                "[TEST] {} yaw={:+.1f} pitch={:+.1f}".format
                    DIR_NAME[direction],
                    float(yaw),
                    float(pitch),
                ),
                flush=True,
            )
            time.sleep(0.30)
            pitch_after, yaw_after = tracker.get_angles()
            print(
                "[HOLD] {} yaw={:+.1f} pitch={:+.1f}".format(
                    DIR_NAME[direction],
                    float(yaw_after),
                    float(pitch_after),
                ),
                flush=True,
            )
            if abs(float(pitch_after) - config.gimbal_scan_pitch_deg) > (
                config.gimbal_pitch_tolerance_deg + 0.25
            ):
                raise RuntimeError(
                    "Pitch not level after stabilization at {}: {:+.1f} deg".format(
                        DIR_NAME[direction], float(pitch_after)
                    )
                )
            if pitch_samples and peak > config.gimbal_pitch_unsafe_deg:
                print(
                    "[WARN] Large TRANSIENT pitch during {}. Check the gimbal "
                    "mount and record a side-view video before maze driving.".format(
                        DIR_NAME[direction]
                    ),
                    flush=True,
                )

        print("[TEST] Four-way pitch/yaw hold completed.")

    finally:
        if chassis is not None:
            try:
                chassis.drive_speed(x=0.0, y=0.0, z=0.0)
            except Exception:
                pass
        if gimbal is not None:
            try:
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
            except Exception:
                pass
            if subscribed:
                try:
                    gimbal.unsub_angle()
                except Exception:
                    pass
        if ep_robot is not None:
            try:
                ep_robot.close()
            except Exception:
                pass
        print("[CLEANUP] Robot safely stopped.")


if __name__ == "__main__":
    main()
