"""Offline tests for live camera survey and safe stop-only pitch logic."""

import sys
import time
import types
import unittest

import cv2
import numpy as np


if "libmedia_codec" not in sys.modules:
    media = types.ModuleType("libmedia_codec")

    class H264Decoder:
        def decode(self, _data):
            return []

    class OpusDecoder:
        def decode(self, _data):
            return None

    media.H264Decoder = H264Decoder
    media.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = media


from classwork8.config import Classwork8Config
from classwork8.live_survey import LiveSurveyBridge
from classwork8.tof_camera_round1_v05 import (
    GimbalTracker,
    _set_camera_observation_pitch,
)


class LiveSurveyTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.config.target_preview_fps = 12.0

    def test_runtime_pitch_request_clamped_and_rescan_consumed_once(self):
        bridge = LiveSurveyBridge(self.config)
        self.assertEqual(bridge.set_pitch(-200), -20.0)
        self.assertEqual(bridge.set_pitch(200), 10.0)
        self.assertEqual(bridge.get_pitch(), 10.0)
        bridge.request_rescan()
        self.assertTrue(bridge.consume_rescan())
        self.assertFalse(bridge.consume_rescan())

    def test_independent_preview_annotates_without_robot(self):
        frame = np.full((360, 640, 3), 110, dtype=np.uint8)
        cv2.rectangle(frame, (270, 220), (312, 262), (0, 190, 0), -1)

        class FakeCamera:
            running = True

            def latest_with_timestamp(self, max_age_sec=0.6):
                return frame.copy(), time.monotonic()

        bridge = LiveSurveyBridge(self.config)
        bridge.attach_camera(FakeCamera())
        try:
            deadline = time.monotonic() + 3.0
            preview = bridge.latest_preview()
            while preview["frame"] is None and time.monotonic() < deadline:
                time.sleep(0.03)
                preview = bridge.latest_preview()
            self.assertIsNotNone(preview["frame"])
            self.assertGreaterEqual(preview["candidate_count"], 1)
        finally:
            bridge.stop()

    def test_camera_pitch_motion_never_commands_yaw(self):
        tracker = GimbalTracker()
        tracker.pitch = 0.0
        tracker.yaw = 90.0

        class FakeGimbal:
            def __init__(self):
                self.commands = []

            def drive_speed(self, pitch_speed=0.0, yaw_speed=0.0):
                self.commands.append((float(pitch_speed), float(yaw_speed)))
                with tracker._lock:
                    tracker.pitch += float(pitch_speed) * 0.03
                return True

        gimbal = FakeGimbal()
        self.config.target_camera_pitch_timeout_sec = 5.0
        self.config.target_camera_settle_sec = 0.01
        reached = _set_camera_observation_pitch(
            gimbal, tracker, self.config, -10.0, None
        )
        self.assertTrue(reached)
        self.assertTrue(any(pitch < 0.0 for pitch, _yaw in gimbal.commands))
        self.assertTrue(all(abs(yaw) == 0.0 for _pitch, yaw in gimbal.commands))
        self.assertLessEqual(abs(tracker.pitch + 10.0), 1.5)


if __name__ == "__main__":
    unittest.main()
