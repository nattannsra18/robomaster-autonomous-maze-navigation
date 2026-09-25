"""Classwork 8 V03 entry point.

V03 changes:
- pre-mission GUI configuration (no source editing required)
- nearest-frontier BFS instead of DFS parent-stack backtracking
- planner-aware realtime map GUI
- safer 60 cm defaults for physical-cell calibration
"""

import argparse
import sys
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
from classwork8.tof_only_v03 import run


def _v03_defaults(config: Classwork8Config) -> None:
    # The V02 1.25 scale made the controller reach its logical 60 cm target
    # before the physical robot had travelled a full cell in the latest field
    # test. Start V03 from neutral odometry scale and tune through the GUI.
    config.cell_size_m = 0.60
    config.exploration_step_m = 0.60
    config.step_tolerance_m = 0.005
    config.odom_scale_x = 1.00
    config.odom_scale_y = 1.00

    config.travel_speed_mps = 0.10
    config.tof_recovery_wait_sec = 1.20
    config.tof_recovery_retries = 2

    # Camera can be enabled from the configuration GUI. Steering remains off
    # until the corridor detector is calibrated on the real maze.
    config.vision_enabled = False
    config.vision_steering_enabled = False


def main():
    parser = argparse.ArgumentParser(
        description="Classwork 8 V03 nearest-frontier SLAM exploration"
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="run terminal-only using V03 defaults",
    )
    parser.add_argument(
        "--no-vision",
        action="store_true",
        help="force camera processing off",
    )
    args = parser.parse_args()

    config = Classwork8Config()
    _v03_defaults(config)

    if args.no_gui:
        if args.no_vision:
            config.vision_enabled = False
        config.validate()
        run(config=config)
        return

    from classwork8.config_gui import configure_before_run

    if not configure_before_run(config):
        print("Mission cancelled before connection.")
        return

    if args.no_vision:
        config.vision_enabled = False
        config.vision_steering_enabled = False

    config.validate()

    print("Connecting to RoboMaster after configuration...")
    ep_robot = robot.Robot()

    try:
        ok = ep_robot.initialize(conn_type=config.connection)
        print("RoboMaster initialize returned: {!r}".format(ok))
        if not ok:
            raise RuntimeError(
                "RoboMaster connection failed. Check the robot Wi-Fi/AP connection."
            )
    except Exception:
        try:
            ep_robot.close()
        except Exception:
            pass
        raise

    from classwork8.gui_v03 import run_with_gui

    run_with_gui(
        run,
        config,
        ep_robot,
    )


if __name__ == "__main__":
    main()
