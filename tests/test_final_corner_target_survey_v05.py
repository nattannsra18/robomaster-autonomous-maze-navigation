"""Offline regressions for camera-only side views of corner/wall signs.

These tests do not drive a real RoboMaster and do not assert field coverage.
The four cardinal ToF mapping directions must stay unchanged.
"""
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

import cv2
import numpy as np
from unittest.mock import patch

if "libmedia_codec" not in sys.modules:
    shim = types.ModuleType("libmedia_codec")
    class H264Decoder:
        def decode(self, _data): return []
    class OpusDecoder:
        def decode(self, _data): return None
    shim.H264Decoder = H264Decoder
    shim.OpusDecoder = OpusDecoder
    sys.modules["libmedia_codec"] = shim

from classwork8.config import Classwork8Config
from classwork8.target_detection import TargetDetector, TargetRegistry
from classwork8.tof_camera_round1_v05 import (
    GimbalTracker, _camera_side_view_yaws,
    _set_camera_observation_yaw, _survey_side_camera_views,
)


class FakeGimbal:
    def __init__(self, tracker):
        self.tracker = tracker
        self.commands = []
    def drive_speed(self, pitch_speed=0.0, yaw_speed=0.0):
        self.commands.append((float(pitch_speed), float(yaw_speed)))
        with self.tracker._lock:
            if self.tracker.pitch is not None:
                self.tracker.pitch += float(pitch_speed) * 0.03
            if self.tracker.yaw is not None:
                self.tracker.yaw += float(yaw_speed) * 0.03
        return True


class FakeChassis:
    def drive_speed(self, **kwargs): return True


class FakeSensors:
    def reset_filters(self): pass


class FakeBridge:
    def set_status(self, status): self.status = status


class FakeRecorder:
    def __init__(self): self.events = []
    def event(self, *args, **kwargs): self.events.append((args, kwargs))


class SideSurveyTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Classwork8Config()
        self.cfg.gimbal_stable_samples = 2
        self.cfg.gimbal_settle_sec = 0.01
        self.cfg.gimbal_turn_timeout_sec = 5.0
        self.tracker = GimbalTracker()
        self.tracker.pitch = 0.0
        self.tracker.yaw = 0.0
        self.gimbal = FakeGimbal(self.tracker)

    def test_offsets_are_bounded_and_camera_only(self):
        self.assertEqual(_camera_side_view_yaws(0, self.cfg), [-20.0, 20.0])
        self.assertEqual(_camera_side_view_yaws(180, self.cfg), [160.0, 200.0])
        self.assertEqual(_camera_side_view_yaws(-90, self.cfg), [-110.0, -70.0])
        self.cfg.target_camera_multi_angle_enabled = False
        self.assertEqual(_camera_side_view_yaws(180, self.cfg), [])

    def test_config_rejects_unbounded_angles_and_capture(self):
        self.cfg.target_camera_side_yaw_offset_deg = 30
        with self.assertRaises(ValueError): self.cfg.validate()
        self.cfg.target_camera_side_yaw_offset_deg = 20
        self.cfg.target_camera_side_sample_frames = 2
        with self.assertRaises(ValueError): self.cfg.validate()

    def test_yaw_only_controller_keeps_pitch_level(self):
        self.assertTrue(_set_camera_observation_yaw(
            self.gimbal, self.tracker, self.cfg, 20.0, None))
        self.assertTrue(any(abs(y)>0 for _, y in self.gimbal.commands))
        self.assertTrue(all(p == 0 for p, _ in self.gimbal.commands))
        self.assertLessEqual(abs(self.tracker.yaw - 20), self.cfg.gimbal_tolerance_deg)

    def test_refuses_turn_when_pitch_not_horizontal(self):
        self.tracker.pitch = -20.0
        self.assertFalse(_set_camera_observation_yaw(
            self.gimbal, self.tracker, self.cfg, 20.0, None))
        self.assertTrue(all(y == 0 for _, y in self.gimbal.commands))

    def test_two_off_center_signs_are_detected_without_centering(self):
        cfg = Classwork8Config()
        cfg.target_min_contour_area_px = 80.0
        cfg.target_min_confidence = 0.40
        frame = np.full((360, 640, 3), 125, dtype=np.uint8)
        # Both shapes remain wholly inside the ROI/frame, but neither is
        # centered. A truly cropped sign is intentionally rejected.
        cv2.rectangle(frame, (19, 218), (62, 260), (0, 0, 210), -1)
        cv2.rectangle(frame, (562, 218), (606, 262), (210, 0, 0), -1)
        detected, _debug = TargetDetector(cfg).detect(frame)
        self.assertTrue(any(x.color == "red" and x.shape == "square" for x in detected))
        self.assertTrue(any(x.color == "blue" and x.shape == "square" for x in detected))

    def test_registry_keeps_side_views_separate_from_unique_targets(self):
        registry = TargetRegistry(self.cfg)
        sign = types.SimpleNamespace(color="red", shape="square",
                                     centroid=(300, 220), bbox=(280, 200, 40, 40))
        left = registry.add_side_view_sighting(
            sign, (1, 1), 0, camera_yaw_deg=-20.0,
            camera_pitch_deg=-20.0, side_yaw_offset_deg=-20.0,
            confidence=0.86, verified_frames=3, verified=True)
        same = registry.add_side_view_sighting(
            sign, (1, 1), 0, camera_yaw_deg=-20.3,
            camera_pitch_deg=-20.0, side_yaw_offset_deg=-20.0,
            confidence=0.87, verified_frames=4, verified=True)
        other = registry.add_side_view_sighting(
            sign, (1, 1), 0, camera_yaw_deg=20.0,
            camera_pitch_deg=-20.0, side_yaw_offset_deg=20.0,
            confidence=0.84, verified_frames=3, verified=True)
        self.assertEqual(left["sighting_id"], same["sighting_id"])
        self.assertNotEqual(left["sighting_id"], other["sighting_id"])
        self.assertEqual(len(registry.targets), 0)
        self.assertIsNone(left["estimated_target_xy_m"])
        self.assertFalse(left["round2_position_ready"])
        with tempfile.TemporaryDirectory() as folder:
            registry.save(Path(folder))
            data = json.loads((Path(folder)/"targets.json").read_text())
            self.assertEqual(data["target_count"], 0)
            self.assertEqual(data["side_view_sighting_count"], 2)
            self.assertEqual(data["side_view_verified_count"], 2)

    def test_side_scan_restores_cardinal_heading_and_never_assigns_xy(self):
        registry = TargetRegistry(self.cfg)
        recorder = FakeRecorder()
        bridge = FakeBridge()
        sign = types.SimpleNamespace(color="green", shape="square",
                                     centroid=(300, 230), bbox=(285, 215, 30, 30))
        verified = types.SimpleNamespace(detection=sign, confidence=0.91,
                                         verified_frames=3)
        def yaw_move(_gimbal, tracker, _config, angle, _stop):
            tracker.yaw = angle
            return True
        def pitch_move(_gimbal, tracker, _config, angle, _stop):
            tracker.pitch = angle
            return True
        def restore(_gimbal, _sensors, tracker, direction, config, _stop):
            tracker.pitch = config.gimbal_scan_pitch_deg
            tracker.yaw = config.gimbal_yaw_for_direction(direction)
            return True
        with patch("classwork8.tof_camera_round1_v05.stop_chassis"), \
             patch("classwork8.tof_camera_round1_v05._set_camera_observation_yaw", side_effect=yaw_move), \
             patch("classwork8.tof_camera_round1_v05._set_camera_observation_pitch", side_effect=pitch_move), \
             patch("classwork8.tof_camera_round1_v05._point_gimbal", side_effect=restore) as restore_mock, \
             patch("classwork8.tof_camera_round1_v05.survey_targets_with_hold",
                   return_value=([verified], [], None, 1)):
            self.assertTrue(_survey_side_camera_views(
                FakeChassis(), self.gimbal, FakeSensors(), self.tracker,
                recorder, self.cfg, None, (0, 0), 0, -20.0,
                types.SimpleNamespace(running=True), object(), registry, [None], bridge))
        self.assertEqual(restore_mock.call_count, 2)
        self.assertEqual(self.tracker.pitch, self.cfg.gimbal_scan_pitch_deg)
        self.assertEqual(self.tracker.yaw, 0.0)
        self.assertEqual(len(registry.side_view_sightings), 2)
        self.assertEqual(len(registry.targets), 0)
        self.assertTrue(all(e["localization_status"] == "SIGHTING_ONLY"
                            for e in registry.side_view_sightings))
        self.assertEqual(len(recorder.events), 2)

    def test_restore_failure_returns_false_and_does_not_try_next_view(self):
        registry = TargetRegistry(self.cfg)
        with patch("classwork8.tof_camera_round1_v05.stop_chassis"), \
             patch("classwork8.tof_camera_round1_v05._set_camera_observation_yaw", return_value=False), \
             patch("classwork8.tof_camera_round1_v05._point_gimbal", return_value=False) as restore:
            ok = _survey_side_camera_views(
                FakeChassis(), self.gimbal, FakeSensors(), self.tracker,
                FakeRecorder(), self.cfg, None, (0, 0), 0, -20.0,
                types.SimpleNamespace(running=True), object(), registry, [None], FakeBridge())
        self.assertFalse(ok)
        self.assertEqual(restore.call_count, 1)


if __name__ == "__main__":
    unittest.main()
