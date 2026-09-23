"""Standalone entry point for Classwork 8.

Default: realtime Tkinter GUI.
Optional: --no-gui for terminal-only execution.
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
from classwork8.tof_only import run


def main():
    parser = argparse.ArgumentParser(description="Classwork 8 ToF exploration")
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="run in terminal without the realtime Tkinter map",
    )
    args = parser.parse_args()

    config = Classwork8Config()

    if args.no_gui:
        run(config=config)
        return

    # Connect in the main thread before Tkinter starts. This matches the
    # working non-GUI connection flow and avoids SDK/network initialization
    # from a GUI worker thread.
    print("Connecting to RoboMaster before opening GUI...")
    ep_robot = robot.Robot()
    try:
        ep_robot.initialize(conn_type=config.connection)
    except Exception:
        try:
            ep_robot.close()
        except Exception:
            pass
        raise

    from classwork8.gui import run_with_gui
    run_with_gui(run, config, ep_robot)


if __name__ == "__main__":
    main()
