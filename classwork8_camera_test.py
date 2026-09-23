"""Stationary camera/vision test for Classwork 8.

The chassis never moves. The script points the gimbal forward, enables the raw
RoboMaster H.264 stream, decodes it with OpenCV/FFmpeg, and displays the
corridor-detection overlay.

Press Q or ESC to stop.
"""

import sys
import time
import types


def _prepare_optional_media_codec():
    try:
        __import__("libmedia_codec")
        return
    except ModuleNotFoundError:
        pass

    media_codec = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    media_codec.H264Decoder = H264Decoder
    media_codec.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media_codec


_prepare_optional_media_codec()

import cv2
from robomaster import robot

from classwork8.config import Classwork8Config
from classwork8.tof_only import (
    GimbalTracker,
    ToFOnlySensorManager,
    _point_gimbal,
)
from classwork8.vision import CorridorVision


def main():
    config = Classwork8Config()
    ep_robot = robot.Robot()
    gimbal = None
    gimbal_subscribed = False
    vision = None

    try:
        print("Connecting to RoboMaster...", flush=True)
        ok = ep_robot.initialize(conn_type=config.connection)
        print("RoboMaster initialize returned: {!r}".format(ok), flush=True)
        if not ok:
            return 2

        ep_robot.set_robot_mode(mode=robot.FREE)
        gimbal = ep_robot.gimbal

        tracker = GimbalTracker()
        sensors = ToFOnlySensorManager()

        gimbal_subscribed = bool(
            gimbal.sub_angle(
                freq=20,
                callback=tracker.callback,
            )
        )
        print("Gimbal subscription: {!r}".format(gimbal_subscribed), flush=True)

        deadline = time.monotonic() + 2.0
        while tracker.get_yaw() is None and time.monotonic() < deadline:
            time.sleep(0.05)

        print("Pointing gimbal FRONT...", flush=True)
        if not _point_gimbal(
            gimbal,
            sensors,
            tracker,
            0,
            config,
            None,
        ):
            print("Could not point gimbal to FRONT.", flush=True)
            return 3

        vision = CorridorVision(ep_robot, config)
        if not vision.start():
            print(
                "Camera stream/decoder test failed. Navigation can still run "
                "with --no-vision.",
                flush=True,
            )
            return 4

        print()
        print("============================================================")
        print(" CAMERA CORRIDOR TEST - CHASSIS WILL NOT MOVE")
        print(" Blue/orange lines = detected side-boundary candidates")
        print(" Green vertical line = estimated corridor centre")
        print(" Q / ESC = stop")
        print("============================================================")

        last_print = 0.0
        while True:
            estimate = vision.latest(max_age_sec=1.0)
            frame = vision.latest_debug_frame()

            now = time.monotonic()
            if estimate is not None and now - last_print >= 0.5:
                correction, error, confidence = vision.correction_mps()
                print(
                    "error={:+.3f} confidence={:.2f} correction={:+.3f} m/s".format(
                        error,
                        confidence,
                        correction,
                    ),
                    flush=True,
                )
                last_print = now

            if frame is not None:
                cv2.imshow("Classwork 8 - Camera Corridor Test", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            else:
                time.sleep(0.02)

        return 0

    except KeyboardInterrupt:
        print("\nStopped by user.", flush=True)
        return 130

    finally:
        try:
            if vision is not None:
                vision.stop()
        except Exception:
            pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            if gimbal is not None:
                gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
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


if __name__ == "__main__":
    raise SystemExit(main())
