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
    # Keep geometry and odometry calibration; use Classwork8Config travel speed.
    config.cell_size_m = 0.60
    config.exploration_step_m = 0.60
    config.step_tolerance_m = 0.005
    config.odom_scale_x = 1.00
    config.odom_scale_y = 1.00
    # No speed override here: the configured value is the direct chassis request.

    config.tof_recovery_wait_sec = 1.20
    config.tof_recovery_retries = 2

    config.closed_maze_auto_stop = True
    config.closed_maze_perimeter_wall_ratio = 0.70
    config.gui_auto_save_map = True

    # Final Round 1 camera target survey.
    config.target_detection_enabled = True
    config.target_camera_resolution = "360p"
    config.target_min_confidence = 0.50
    config.target_save_confidence = 0.60
    config.target_sample_frames = 6
    config.target_verify_frames = 4

    # Do not run the older corridor-steering camera pipeline in this baseline.
    config.vision_enabled = False
    config.vision_steering_enabled = False


def _apply_cli_overrides(config, args) -> None:
    """Reapply diagnostic limits AFTER GUI so the 1-cell trial is bounded."""
    if args.travel_speed is not None:
        config.travel_speed_mps = args.travel_speed
    if args.no_camera:
        config.target_detection_enabled = False
    if args.yaw_isolation:
        config.yaw_isolation_mode = True
        config.heading_hold_enabled = False
    if args.max_moves is not None:
        config.max_moves = args.max_moves
    if args.max_yaw_correction is not None:
        config.heading_max_z_dps = args.max_yaw_correction
        config.heading_align_max_z_dps = min(
            float(config.heading_align_max_z_dps),
            float(args.max_yaw_correction),
        )


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
    parser.add_argument(
        "--travel-speed",
        type=float,
        default=None,
        metavar="MPS",
        help="direct longitudinal chassis speed in m/s (overrides config default)",
    )
    parser.add_argument(
        "--yaw-isolation",
        action="store_true",
        help="diagnostic: force chassis z=0 during every move and disable all post-scan yaw alignment; log chassis/gimbal yaw separately",
    )
    parser.add_argument(
        "--max-moves",
        type=int,
        default=None,
        metavar="N",
        help="hard mission cap (e.g. 1 for a one-cell test), also after GUI",
    )
    parser.add_argument(
        "--max-yaw-correction",
        type=float,
        default=None,
        metavar="DPS",
        help="cap moving and post-scan chassis yaw commands, also after GUI",
    )
    args = parser.parse_args()
    if args.max_moves is not None and args.max_moves < 1:
        parser.error("--max-moves must be at least 1")
    if args.max_yaw_correction is not None and not (
        0.0 < args.max_yaw_correction <= 30.0
    ):
        parser.error("--max-yaw-correction must be >0 and <=30 deg/s")

    config = Classwork8Config()
    _defaults(config)
    _apply_cli_overrides(config, args)

    if args.no_gui:
        config.validate()
        print(
            "[DIAG_LIMITS] max_moves={} heading_max_z={} "
            "align_max_z={} speed={} yaw_isolation={}".format(
                config.max_moves, config.heading_max_z_dps,
                config.heading_align_max_z_dps,
                config.travel_speed_mps, config.yaw_isolation_mode
            ), flush=True,
        )
        run(config=config)
        return

    from classwork8.config_gui_v05 import configure_before_run

    if not configure_before_run(config):
        print("Mission cancelled before connection.")
        return

    _apply_cli_overrides(config, args)
    config.validate()
    print(
        "[DIAG_LIMITS] max_moves={} heading_max_z={} "
        "align_max_z={} speed={} yaw_isolation={}".format(
            config.max_moves, config.heading_max_z_dps,
            config.heading_align_max_z_dps,
            config.travel_speed_mps, config.yaw_isolation_mode
        ), flush=True,
    )

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
