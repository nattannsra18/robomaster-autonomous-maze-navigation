"""Final Assignment - Round 1 baseline (ToF + camera only).

Round 1 responsibilities:
- explore unknown maze with nearest-frontier BFS
- build occupancy + logical topology map
- detect colored shape targets during the same gimbal scan
- record navigation-ready target observations
- stop when exploration completes
- export map, topology.json, targets.json and GUI map

This baseline intentionally does not aim or fire the blaster.
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
from classwork8.tof_camera_round1_v05 import run


def _defaults(config: Classwork8Config) -> None:
    # Keep the proven V04 movement setup from the successful field run.
    config.cell_size_m = 0.60
    config.exploration_step_m = 0.60
    config.step_tolerance_m = 0.005
    config.odom_scale_x = 1.00
    config.odom_scale_y = 1.00
    config.travel_speed_mps = 0.10

    config.tof_recovery_wait_sec = 1.20
    config.tof_recovery_retries = 2

    config.closed_maze_auto_stop = True
    config.closed_maze_perimeter_wall_ratio = 0.70
    config.gui_auto_save_map = True

    # Final Round 1 camera target survey.
    config.target_detection_enabled = True
    config.target_camera_resolution = "360p"
    config.target_min_confidence = 0.58
    config.target_save_confidence = 0.70
    config.target_sample_frames = 5
    config.target_verify_frames = 3

    # Do not run the older corridor-steering camera pipeline in this baseline.
    config.vision_enabled = False
    config.vision_steering_enabled = False


def main():
    parser = argparse.ArgumentParser(
        description="Final Assignment Round 1 - ToF + camera map and target survey"
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="run terminal-only using current defaults",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="disable target camera and run ToF mapping only",
    )
    args = parser.parse_args()

    config = Classwork8Config()
    _defaults(config)

    if args.no_camera:
        config.target_detection_enabled = False

    if args.no_gui:
        config.validate()
        run(config=config)
        return

    from classwork8.config_gui_v05 import configure_before_run

    if not configure_before_run(config):
        print("Mission cancelled before connection.")
        return

    if args.no_camera:
        config.target_detection_enabled = False

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

    from classwork8.gui_v05 import run_with_gui

    run_with_gui(
        run,
        config,
        ep_robot,
    )


if __name__ == "__main__":
    main()
