"""Stationary gimbal/ToF direction check for Classwork 8.

The chassis never moves. Visually confirm:
  FRONT -> gimbal faces front
  RIGHT -> gimbal faces robot right
  BACK  -> gimbal faces rear
  LEFT  -> gimbal faces robot left
"""

import sys
import time
import types
import threading


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

from robomaster import robot

from classwork8.config import Classwork8Config
from classwork8.tof_only import (
    DIR_NAME,
    GimbalTracker,
    ToFOnlySensorManager,
    _point_gimbal,
    _sample_tof,
)


def main():
    config = Classwork8Config()
    ep_robot = robot.Robot()
    gimbal = None
    tof_sensor = None
    tof_subscribed = False
    gimbal_subscribed = False
    stop_event = threading.Event()

    try:
        print("Connecting to RoboMaster...")
        ep_robot.initialize(conn_type=config.connection)
        ep_robot.set_robot_mode(mode=robot.FREE)

        gimbal = ep_robot.gimbal
        tof_sensor = ep_robot.sensor
        sensors = ToFOnlySensorManager()
        tracker = GimbalTracker()

        gimbal.recenter(
            pitch_speed=config.gimbal_yaw_speed_dps,
            yaw_speed=config.gimbal_yaw_speed_dps,
        ).wait_for_completed()

        tof_subscribed = bool(
            tof_sensor.sub_distance(freq=20, callback=sensors.tof_callback)
        )
        gimbal_subscribed = bool(
            gimbal.sub_angle(freq=20, callback=tracker.callback)
        )

        print()
        print("============================================================")
        print(" GIMBAL / ToF DIRECTION TEST - CHASSIS WILL NOT MOVE")
        print("============================================================")
        print("Watch the physical gimbal direction for each label.")
        print("Press Ctrl+C to stop.")
        print("============================================================")

        time.sleep(0.5)

        for direction in (0, 1, 2, 3, 0):
            name = DIR_NAME[direction]
            print("\nPointing {}...".format(name))
            ok = _point_gimbal(
                gimbal,
                sensors,
                tracker,
                direction,
                config,
                stop_event,
            )
            if not ok:
                print("FAILED to reach {} target.".format(name))
                break

            distance = _sample_tof(sensors, config, stop_event)
            print(
                "{}: gimbal_yaw={:+.1f} deg, ToF={} cm".format(
                    name,
                    tracker.get_yaw() if tracker.get_yaw() is not None else 999.0,
                    "---" if distance is None else "{:.1f}".format(distance),
                )
            )
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        try:
            if gimbal is not None:
                gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                gimbal.recenter(
                    pitch_speed=config.gimbal_yaw_speed_dps,
                    yaw_speed=config.gimbal_yaw_speed_dps,
                ).wait_for_completed()
        except Exception:
            pass
        try:
            if tof_sensor is not None and tof_subscribed:
                tof_sensor.unsub_distance()
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

        print("RoboMaster connection closed.")


if __name__ == "__main__":
    main()
