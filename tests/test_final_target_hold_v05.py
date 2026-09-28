"""Offline checks for bounded stationary camera hold and provisional sightings."""

import json
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from classwork8.config import Classwork8Config
from classwork8.target_detection import (
    TargetDetector, TargetRegistry, VerifiedTarget, survey_targets_with_hold,
)
from classwork8.target_map_visual import target_plot_geometry


def frame_with_signs(red=True, green=True):
    frame = np.full((360, 640, 3), 120, dtype=np.uint8)
    if red:
        cv2.rectangle(frame, (145, 160), (220, 235), (0, 0, 220), -1)
    if green:
        cv2.rectangle(frame, (360, 155), (430, 225), (0, 195, 0), -1)
    return frame


class DistinctFrameCamera:
    def __init__(self, frames):
        self.frames = frames
        self.index = 0
        self.origin = time.monotonic()

    def latest_with_timestamp(self, max_age_sec=0.6):
        if self.index >= len(self.frames):
            return None
        frame = self.frames[self.index]
        self.index += 1
        # Every frame is independent, and each later window has a larger time.
        return frame, self.origin + self.index * 0.001


class HoldSurveyTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Classwork8Config()
        self.cfg.target_min_contour_area_px = 80.0
        self.cfg.target_sample_frames = 4
        self.cfg.target_verify_frames = 3
        self.cfg.target_frame_interval_sec = 0.005
        self.cfg.target_hold_max_sec = 3.0
        self.cfg.target_hold_max_windows = 3
        self.cfg.target_pending_min_frames = 2
        self.detector = TargetDetector(self.cfg)

    def test_waits_for_second_sign_before_turning_away(self):
        both = frame_with_signs()
        red_only = frame_with_signs(green=False)
        camera = DistinctFrameCamera(
            [both, both, red_only, red_only] + [both] * 4
        )
        verified, pending, debug, windows = survey_targets_with_hold(
            self.detector, camera, self.cfg,
            not_before=camera.origin,
        )
        found = {(t.detection.color, t.detection.shape) for t in verified}
        self.assertTrue(any(color == "red" for color, _ in found))
        self.assertTrue(any(color == "green" for color, _ in found))
        self.assertGreaterEqual(windows, 2)
        self.assertEqual(pending, [])
        self.assertIsNotNone(debug)

    def test_nonverified_two_frame_sign_is_saved_only_as_pending(self):
        red = frame_with_signs(green=False)
        blank = frame_with_signs(red=False, green=False)
        camera = DistinctFrameCamera([red, red, blank, blank] + [blank] * 4)
        verified, pending, _debug, windows = survey_targets_with_hold(
            self.detector, camera, self.cfg,
            not_before=camera.origin,
        )
        self.assertEqual(verified, [])
        self.assertGreaterEqual(windows, 2)
        self.assertTrue(any(x["detection"].color == "red" and x["frames"] == 2
                            for x in pending))
        registry = TargetRegistry(self.cfg)
        entry = registry.add_pending(pending[0], (0, 0), 0, 85.0)
        self.assertEqual(entry["status"], "PENDING_RECHECK")
        self.assertEqual(entry["color"], "red")
        self.assertIsNone(entry["estimated_target_xy_m"])
        self.assertFalse(entry["round2_position_ready"])
        hint, source, sighting_only = target_plot_geometry(entry, 0.6)
        self.assertTrue(sighting_only)
        self.assertIsNotNone(hint)
        self.assertIsNotNone(source)
        with tempfile.TemporaryDirectory() as folder:
            registry.save(Path(folder))
            result = json.loads((Path(folder) / "targets.json").read_text())
            self.assertEqual(result["target_count"], 0)
            self.assertEqual(result["pending_count"], 1)
            self.assertEqual(result["pending_targets"][0]["shape"], entry["shape"])

    def test_confirmed_observation_replaces_same_view_pending(self):
        red = frame_with_signs(green=False)
        sign = next(d for d in self.detector.detect(red)[0] if d.color == "red")
        registry = TargetRegistry(self.cfg)
        registry.add_pending({
            "detection": sign, "frames": 2,
            "confidence_sum": 2 * sign.confidence,
        }, (1, 1), 0, 25.0)
        registry.add_verified(VerifiedTarget(sign, 3, sign.confidence),
                              (1, 1), 0, 25.0)
        self.assertEqual(len(registry.targets), 1)
        self.assertEqual(len(registry.pending_targets), 0)

    def test_frozen_frame_never_becomes_confirmed(self):
        red = frame_with_signs(green=False)
        class FrozenCamera:
            def latest_with_timestamp(self, max_age_sec=0.6):
                return red, 123.0
        self.cfg.target_hold_max_windows = 1
        self.cfg.target_hold_max_sec = 0.4
        verified, pending, _debug, _windows = survey_targets_with_hold(
            self.detector, FrozenCamera(), self.cfg, not_before=122.0,
        )
        self.assertEqual(verified, [])
        self.assertEqual(pending, [])


if __name__ == "__main__":
    unittest.main()
