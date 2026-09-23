"""Standalone entry point for Classwork 8.

Classwork 8 does not use the RoboMaster camera or audio stream. The upstream
DJI SDK imports its optional native media codec while importing robot.py,
though, so provide a minimal in-memory compatibility module when that codec is
not installed. Chassis, gimbal, ToF, sensor adaptor, odometry and attitude still
use the real RoboMaster SDK.
"""

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

from classwork8.run import main


if __name__ == "__main__":
    main()
