"""Stationary hardware test for Classwork 8.

No chassis drive commands are sent. The script only:
- connects to the RoboMaster
- sets CHASSIS_LEAD and recenters the gimbal
- subscribes ToF / odometry / attitude
- reads left/right Sharp and front-corner IR
- prints live values until Ctrl+C
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

from robomaster import robot

from classwork8.config import Classwork8Config
from classwork8.sensors import Classwork8SensorManager
from robomaster_mission.mission import PoseTracker


def fmt_cm(value):
    return "---" if value is None else f"{value:6.1f} cm"


def fmt_bool(value):
    if value is None:
        return "WAIT"
    return "BLOCKED" if value else "CLEAR"


def main():
    config = Classwork8Config()
    ep_robot = robot.Robot()

    chassis = None
    tof_sensor = None
    tof_subscribed = False
    pose_subscribed = False
    attitude_subscribed = False

    try:
        print("Connecting to RoboMaster...")
        ep_robot.initialize(conn_type=config.connection)

        chassis = ep_robot.chassis
        gimbal = ep_robot.gimbal
        tof_sensor = ep_robot.sensor
        sensor_adapter = ep_robot.sensor_adaptor

        print("Setting CHASSIS_LEAD and recentering gimbal...")
        ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
        gimbal.recenter(
            pitch_speed=100,
            yaw_speed=100,
        ).wait_for_completed()

        pose = PoseTracker()
        sensors = Classwork8SensorManager(sensor_adapter, config)

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

        print()
        print("============================================================")
        print(" Classwork 8 - SENSOR TEST (NO CHASSIS MOTION)")
        print("============================================================")
        print("LEFT  Hub 2: IR port 1, Sharp port 2")
        print("RIGHT Hub 1: IR port 1, Sharp port 2")
        print("Front ToF  : Gimbal")
        print("Press Ctrl+C to stop.")
        print("============================================================")
        print()

        while True:
            raw_l, left_cm = sensors.read_left()
            raw_r, right_cm = sensors.read_right()
            front_cm = sensors.get_front_cm()
            ir_left, ir_right = sensors.read_front_corner_ir()

            x, y = pose.get_xy()
            yaw = pose.get_yaw()

            x_text = "---" if x is None else f"{x:+.3f}"
            y_text = "---" if y is None else f"{y:+.3f}"
            yaw_text = "---" if yaw is None else f"{yaw:+.1f}"

            print(
                f"ToF {fmt_cm(front_cm)} | "
                f"Sharp-L {fmt_cm(left_cm)} ADC={raw_l:4d} | "
                f"Sharp-R {fmt_cm(right_cm)} ADC={raw_r:4d} | "
                f"IR-L {fmt_bool(ir_left):7s} | "
                f"IR-R {fmt_bool(ir_right):7s} | "
                f"x={x_text} y={y_text} yaw={yaw_text}"
            )

            time.sleep(0.20)

    except KeyboardInterrupt:
        print("\nSensor test stopped by user.")

    finally:
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

        print("RoboMaster connection closed.")


if __name__ == "__main__":
    main()
