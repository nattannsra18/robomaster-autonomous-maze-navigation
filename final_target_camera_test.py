"""Stationary Final Assignment target-camera diagnostic.

Connects to RoboMaster and opens the shared camera stream, but never commands
the chassis and never fires the blaster.

Keys:
- S: save raw + annotated frames
- Q / ESC: quit
"""

import sys
import time
import types
from datetime import datetime
from pathlib import Path

import cv2


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

from classwork8.camera_service import CameraService
from classwork8.config import Classwork8Config
from classwork8.target_detection import TargetDetector


def main():
    config = Classwork8Config()
    config.target_detection_enabled = True
    config.target_camera_resolution = "360p"

    ep_robot = robot.Robot()
    camera = None

    try:
        print("Connecting to RoboMaster...")
        ok = ep_robot.initialize(conn_type=config.connection)
        print("RoboMaster initialize returned: {!r}".format(ok))
        if not ok:
            raise RuntimeError("Could not connect to RoboMaster")

        # This test never commands chassis motion.
        try:
            ep_robot.chassis.drive_speed(x=0.0, y=0.0, z=0.0)
        except Exception:
            pass

        camera = CameraService(
            ep_robot,
            resolution=config.target_camera_resolution,
            start_timeout_sec=config.target_camera_start_timeout_sec,
        )
        if not camera.start():
            raise RuntimeError("Could not start camera stream")

        detector = TargetDetector(config)

        print()
        print("Final Target Camera Test")
        print("S = save raw + annotated frame")
        print("Q / ESC = quit")

        while True:
            frame = camera.latest(max_age_sec=1.0)
            if frame is None:
                time.sleep(0.01)
                continue

            detections, debug = detector.detect(frame)

            y = 24
            cv2.putText(
                debug,
                "DETECTIONS: {}".format(len(detections)),
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            y += 24

            for item in detections[:8]:
                text = "{} {} conf={:.2f} hsv=({:.0f},{:.0f},{:.0f})".format(
                    item.color.upper(),
                    item.shape.upper(),
                    item.confidence,
                    item.median_hsv[0],
                    item.median_hsv[1],
                    item.median_hsv[2],
                )
                cv2.putText(
                    debug,
                    text,
                    (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                y += 20

            cv2.imshow("Final Target Camera Test", debug)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord("s"):
                folder = Path(config.output_dir) / "target_camera_samples"
                folder.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                raw_path = folder / "target_raw_{}.png".format(stamp)
                debug_path = folder / "target_debug_{}.png".format(stamp)
                cv2.imwrite(str(raw_path), frame)
                cv2.imwrite(str(debug_path), debug)
                print("Saved: {}".format(raw_path))
                print("Saved: {}".format(debug_path))

    finally:
        try:
            ep_robot.chassis.drive_speed(x=0.0, y=0.0, z=0.0)
        except Exception:
            pass

        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            ep_robot.close()
        except Exception:
            pass

        print("Camera test finished safely.")


if __name__ == "__main__":
    main()
