import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from classwork8.config import Classwork8Config
from classwork8.target_detection import (
    TargetDetector,
    TargetRegistry,
    VerifiedTarget,
)


class FinalTargetDetectionTests(unittest.TestCase):
    def setUp(self):
        self.config = Classwork8Config()
        self.config.target_min_confidence = 0.45
        self.config.target_save_confidence = 0.55
        self.config.target_min_contour_area_px = 80.0
        self.detector = TargetDetector(self.config)

    def test_green_square_detected_under_dark_and_bright_backgrounds(self):
        for background in (30, 180):
            frame = np.full((360, 640, 3), background, dtype=np.uint8)
            cv2.rectangle(frame, (250, 110), (390, 250), (0, 190, 0), -1)

            detections, _debug = self.detector.detect(frame)

            matches = [
                item for item in detections
                if item.color == "green" and item.shape == "square"
            ]
            self.assertTrue(matches)

    def test_red_circle_detected(self):
        frame = np.full((360, 640, 3), 120, dtype=np.uint8)
        cv2.circle(frame, (320, 180), 70, (0, 0, 220), -1)

        detections, _debug = self.detector.detect(frame)

        matches = [
            item for item in detections
            if item.color == "red" and item.shape == "circle"
        ]
        self.assertTrue(matches)

    def test_registry_merges_repeat_observations(self):
        frame = np.full((360, 640, 3), 100, dtype=np.uint8)
        cv2.rectangle(frame, (250, 110), (390, 250), (0, 190, 0), -1)

        detections, _debug = self.detector.detect(frame)
        detection = next(
            item for item in detections
            if item.color == "green" and item.shape == "square"
        )

        verified = VerifiedTarget(
            detection=detection,
            verified_frames=4,
            confidence=max(0.80, detection.confidence),
        )

        registry = TargetRegistry(self.config)
        first = registry.add_verified(
            verified,
            approach_cell=(2, -1),
            direction=1,
            tof_cm=30.0,
        )
        second = registry.add_verified(
            verified,
            approach_cell=(2, -1),
            direction=1,
            tof_cm=31.0,
        )

        self.assertEqual(first["target_id"], second["target_id"])
        self.assertEqual(len(registry.targets), 1)
        self.assertEqual(registry.targets[0]["observations"], 2)

        with tempfile.TemporaryDirectory() as folder:
            registry.save(Path(folder))
            self.assertTrue(Path(folder, "targets.json").exists())


if __name__ == "__main__":
    unittest.main()
