"""Independent Classwork 8 V05 SLAM-only entry point.

Run:
    python -u classwork8_slam_05.py
    python -u classwork8_slam_05.py --no-gui

Explores an unknown maze with gimbal-only four-way ToF scans, odometry,
nearest-frontier BFS and occupancy/topology map export. No target camera,
color/shape recognition, Ground Truth input, or blaster actions.

Uses the *current V05 movement and recovery backend* without changing either
classwork8_slam_04.py or final_round1_tof_camera_01.py. The shared V05 export
may still include an empty targets.json for compatibility.
"""

import argparse
import sys
import types


def _prepare_optional_media_codec():
    # Match the existing project launchers: mapping does not use video, but
    # some RoboMaster SDK builds eagerly import this optional codec package.
    try:
        __import__("libmedia_codec")
        return
    except ModuleNotFoundError:
        pass

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

from classwork8.config import Classwork8Config
from classwork8.tof_camera_round1_v05 import run as run_v05


def apply_slam_defaults(config: Classwork8Config) -> None:
    """Keep V05 physical-cell movement, with an isolated mapping output folder."""
    config.cell_size_m = 0.60
    config.exploration_step_m = 0.60
    config.step_tolerance_m = 0.005
    config.odom_scale_x = 1.00
    config.odom_scale_y = 1.00
    config.travel_speed_mps = 0.20

    config.midcell_side_check_enabled = False
    config.side_start_auto_recovery_enabled = True
    config.supervised_hold_on_safety_dead_end = True
    config.wall_follow_recovery_enabled = True
    config.tof_recovery_wait_sec = 1.20
    config.tof_recovery_retries = 2

    config.closed_maze_auto_stop = True
    config.closed_maze_perimeter_wall_ratio = 0.70
    config.gui_auto_save_map = True
    config.output_dir = "classwork8_output_slam"

    # Take current V05 values (20 cm escape preflight, 13 cm moving ToF stop,
    # gimbal tuning) from Classwork8Config rather than duplicating them here.
    force_slam_only(config)


def force_slam_only(config: Classwork8Config) -> None:
    """Hard-disable the camera path, even after GUI Reset Defaults."""
    config.target_detection_enabled = False
    config.target_survey_open_directions = False
    config.vision_enabled = False
    config.vision_steering_enabled = False


def run_slam_only(**kwargs):
    """Guard both GUI and terminal entry points against camera re-enabling."""
    config = kwargs["config"]
    force_slam_only(config)
    config.validate()
    return run_v05(**kwargs)


def main():
    parser = argparse.ArgumentParser(
        description="Classwork 8 V05: autonomous ToF SLAM only, no Ground Truth"
    )
    parser.add_argument(
        "--no-gui", action="store_true",
        help="run mapping without the configuration/realtime map windows",
    )
    parser.add_argument(
        "--output-dir", default=None,
        help="optional directory for map/log exports (default classwork8_output_slam)",
    )
    args = parser.parse_args()

    config = Classwork8Config()
    apply_slam_defaults(config)
    if args.output_dir:
        config.output_dir = args.output_dir

    if args.no_gui:
        print("[SLAM ONLY] ToF + odometry + BFS; target camera OFF; no Ground Truth.", flush=True)
        run_slam_only(config=config)
        return

    from classwork8.config_gui_v05 import configure_before_run

    if not configure_before_run(config):
        print("SLAM mission cancelled before connection.", flush=True)
        return

    # The shared V05 configuration screen includes target detection controls;
    # these controls do not apply to this dedicated SLAM-only launcher.
    force_slam_only(config)
    if args.output_dir:
        config.output_dir = args.output_dir
    config.validate()
    print("[SLAM ONLY] Camera/targets disabled; using the V05 realtime map.", flush=True)
    print("Connecting to RoboMaster after configuration...", flush=True)

    ep_robot = robot.Robot()
    try:
        ok = ep_robot.initialize(conn_type=config.connection)
        print("RoboMaster initialize returned: {!r}".format(ok), flush=True)
        if not ok:
            raise RuntimeError("RoboMaster connection failed; check robot Wi-Fi/AP.")
    except Exception:
        try:
            ep_robot.close()
        except Exception:
            pass
        raise

    from classwork8.gui_v05 import run_with_gui
    run_with_gui(run_slam_only, config, ep_robot)


if __name__ == "__main__":
    main()
