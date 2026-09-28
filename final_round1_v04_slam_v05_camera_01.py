"""Final Round 1: EXACT V04 navigation/SLAM + V05 image processing only.

Navigation source: classwork8_slam_04.py -> tof_only_v04.py on
classwork8-slam-exploration. No import of tof_camera_round1_v05.py.
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
from classwork8.tof_only_v04_camera import run


def _defaults(config):
    # Same V04 mission and movement defaults as classwork8_slam_04.py.
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
    # V04 motion must NEVER consume the other camera corridor-steering stream.
    config.vision_enabled = False
    config.vision_steering_enabled = False
    config.target_detection_enabled = True
    config.target_camera_resolution = "360p"
    config.target_min_confidence = 0.50
    config.target_save_confidence = 0.60
    config.target_sample_frames = 10
    config.target_verify_frames = 3
    config.target_hold_max_sec = 3.0
    config.target_hold_max_windows = 3
    config.target_camera_multi_angle_enabled = True
    # These V05-only flags are explicitly OFF; the V04 controller never reads them.
    config.side_start_auto_recovery_enabled = False
    config.wall_follow_recovery_enabled = False
    config.supervised_hold_on_safety_dead_end = False
    config.side_start_heading_recovery_enabled = False
    # Restore V04 gimbal yaw tuning, rather than taking V05 global config values.
    config.gimbal_yaw_speed_dps = 90.0
    config.gimbal_min_yaw_speed_dps = 12.0
    config.gimbal_yaw_kp = 2.0


def main():
    parser = argparse.ArgumentParser(
        description="V04 movement and SLAM + V05 camera target detection, without V05 recovery")
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--no-camera", action="store_true")
    args = parser.parse_args()
    config = Classwork8Config()
    _defaults(config)
    if args.no_camera:
        config.target_detection_enabled = False
    if args.no_gui:
        config.validate()
        run(config=config)
        return
    # The V04 pre-flight UI controls ONLY V04 movement/map parameters; it does
    # not include the V05 "Reset defaults" button that enables V05 recovery.
    from classwork8.config_gui_v04 import configure_before_run
    if not configure_before_run(config):
        print("Mission cancelled before connection.")
        return
    config.vision_enabled = False
    config.vision_steering_enabled = False
    if args.no_camera:
        config.target_detection_enabled = False
    config.validate()
    ep_robot = robot.Robot()
    try:
        ok = ep_robot.initialize(conn_type=config.connection)
        print("RoboMaster initialize returned: {!r}".format(ok))
        if not ok:
            raise RuntimeError("RoboMaster connection failed.")
    except Exception:
        try:
            ep_robot.close()
        except Exception:
            pass
        raise
    # Presentation only: V05 live-preview GUI. It does NOT supply motion logic.
    from classwork8.gui_v05 import run_with_gui
    run_with_gui(run, config, ep_robot)


if __name__ == "__main__":
    main()
