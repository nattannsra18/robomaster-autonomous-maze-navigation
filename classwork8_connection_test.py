"""Minimal RoboMaster AP connection diagnostic for Classwork 8."""

import socket
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


def main():
    print("=== RoboMaster AP connection diagnostic ===", flush=True)
    print("Computer hostname : {}".format(socket.gethostname()), flush=True)

    try:
        host_ip = socket.gethostbyname(socket.gethostname())
    except Exception as exc:
        host_ip = "ERROR: {}".format(exc)
    print("Hostname IPv4     : {}".format(host_ip), flush=True)
    print("Expected robot IP : 192.168.2.1", flush=True)
    print("Connection mode   : ap", flush=True)
    print("", flush=True)

    ep_robot = robot.Robot()

    try:
        print("[1/3] Calling ep_robot.initialize(conn_type='ap') ...", flush=True)
        ok = ep_robot.initialize(conn_type="ap")
        print("[2/3] initialize returned: {!r}".format(ok), flush=True)

        if not ok:
            print("FAILED: SDK did not establish a RoboMaster connection.", flush=True)
            return 2

        print("[3/3] RoboMaster connection OK.", flush=True)
        return 0

    except KeyboardInterrupt:
        print("\nCancelled by user.", flush=True)
        return 130

    except Exception as exc:
        print("\nConnection exception: {!r}".format(exc), flush=True)
        return 1

    finally:
        try:
            ep_robot.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
