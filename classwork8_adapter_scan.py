"""Scan RoboMaster Sensor Adapter IDs without moving the chassis.

This uses DJI's sub_adapter() DDS stream instead of synchronous get_adc/get_io,
so a wrong adapter ID does not cause repeated 3-second command timeouts.
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


_lock = threading.Lock()
_latest = None


def adapter_callback(sub_info):
    global _latest
    io_data, ad_data = sub_info
    with _lock:
        _latest = (list(io_data), list(ad_data))


def main():
    ep_robot = robot.Robot()
    sensor_adapter = None
    subscribed = False

    try:
        print("Connecting to RoboMaster...")
        ep_robot.initialize(conn_type="ap")
        sensor_adapter = ep_robot.sensor_adaptor

        print("Subscribing to Sensor Adapter DDS stream...")
        subscribed = bool(
            sensor_adapter.sub_adapter(
                freq=5,
                callback=adapter_callback,
            )
        )

        print()
        print("==============================================================")
        print(" Classwork 8 - SENSOR ADAPTER ID SCAN (NO CHASSIS MOTION)")
        print("==============================================================")
        print("Each adapter has Port 1 and Port 2.")
        print("Move a wall near each Sharp and cover each IR sensor.")
        print("Watch which Adapter ID / Port values change.")
        print("Press Ctrl+C to stop.")
        print("==============================================================")
        print()

        while True:
            with _lock:
                snapshot = None if _latest is None else (
                    list(_latest[0]),
                    list(_latest[1]),
                )

            if snapshot is None:
                print("Waiting for adapter DDS data...")
                time.sleep(1.0)
                continue

            io_data, ad_data = snapshot
            print("--------------- Sensor Adapter snapshot ---------------")
            # DJI AdapterSubject exposes 12 entries = 6 adapters x 2 ports.
            for adapter_id in range(1, 7):
                p1 = (adapter_id - 1) * 2
                p2 = p1 + 1
                io1 = io_data[p1] if p1 < len(io_data) else None
                io2 = io_data[p2] if p2 < len(io_data) else None
                ad1 = ad_data[p1] if p1 < len(ad_data) else None
                ad2 = ad_data[p2] if p2 < len(ad_data) else None
                print(
                    "ID {0}: "
                    "P1 IO={1!s:>4} ADC={2!s:>5} | "
                    "P2 IO={3!s:>4} ADC={4!s:>5}".format(
                        adapter_id, io1, ad1, io2, ad2
                    )
                )
            print()
            time.sleep(0.8)

    except KeyboardInterrupt:
        print("\nAdapter scan stopped by user.")

    finally:
        try:
            if sensor_adapter is not None and subscribed:
                sensor_adapter.unsub_adapter()
        except Exception:
            pass

        try:
            ep_robot.close()
        except Exception:
            pass

        print("RoboMaster connection closed.")


if __name__ == "__main__":
    main()
