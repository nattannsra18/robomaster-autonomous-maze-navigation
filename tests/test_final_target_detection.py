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

    def test_floor_reflection_is_outside_configured_target_roi(self):
        # Reproduce the user's real sample geometry without committing their
        # private camera photo into the repository.
        frame = np.full((360, 640, 3), 130, dtype=np.uint8)
        cv2.rectangle(frame, (274, 230), (316, 271), (0, 190, 0), -1)
        cv2.rectangle(frame, (277, 313), (309, 328), (0, 220, 220), -1)

        detections, _debug = self.detector.detect(frame)

        self.assertTrue(any(
            item.color == "green" and item.shape == "square"
            for item in detections
        ))
        self.assertFalse(any(item.centroid[1] >= 300 for item in detections))

    def test_wide_green_sign_is_rectangle_not_square(self):
        frame = np.full((360, 640, 3), 130, dtype=np.uint8)
        # Similar proportions to the user's rightmost green sign: 48 x 36.
        cv2.rectangle(frame, (565, 232), (612, 267), (0, 190, 0), -1)

        detections, _debug = self.detector.detect(frame)

        self.assertTrue(any(
            item.color == "green" and item.shape == "rectangle"
            for item in detections
        ))
        self.assertFalse(any(
            item.color == "green" and item.shape == "square"
            for item in detections
        ))

    def test_two_same_color_same_shape_on_one_wall_keep_separate_ids(self):
        frame = np.full((360, 640, 3), 130, dtype=np.uint8)
        cv2.rectangle(frame, (230, 227), (271, 267), (0, 0, 210), -1)
        cv2.rectangle(frame, (405, 230), (443, 269), (0, 0, 210), -1)

        detections, _debug = self.detector.detect(frame)
        squares = [
            item for item in detections
            if item.color == "red" and item.shape == "square"
        ]
        self.assertEqual(len(squares), 2)

        registry = TargetRegistry(self.config)
        for item in squares:
            registry.add_verified(
                VerifiedTarget(
                    detection=item,
                    verified_frames=4,
                    confidence=max(0.80, item.confidence),
                ),
                approach_cell=(2, -1),
                direction=0,
                tof_cm=29.0,
            )

        self.assertEqual(len(registry.targets), 2)
        self.assertNotEqual(
            registry.targets[0]["target_id"],
            registry.targets[1]["target_id"],
        )

    def test_cached_same_frame_does_not_count_as_temporal_verification(self):
        frame = np.full((360, 640, 3), 130, dtype=np.uint8)
        cv2.rectangle(frame, (270, 220), (311, 262), (0, 190, 0), -1)

        class FrozenCamera:
            def latest_with_timestamp(self, max_age_sec=0.6):
                return frame, 123.0

        self.config.target_sample_frames = 4
        self.config.target_verify_frames = 3
        self.config.target_frame_interval_sec = 0.005

        verified, _debug = self.detector.verify_latest(FrozenCamera())
        self.assertEqual(verified, [])

    def test_ground_level_sign_roi_can_be_adjusted_live(self):
        # A sign near the image bottom was invisible in the 0.82 wall ROI.
        frame = np.full((360, 640, 3), 110, dtype=np.uint8)
        cv2.rectangle(frame, (250, 300), (303, 326), (0, 180, 0), -1)

        self.config.target_roi_bottom_ratio = 0.82
        narrow, _ = self.detector.detect(frame)
        self.assertFalse(any(item.color == "green" for item in narrow))

        self.config.target_roi_bottom_ratio = 0.94
        extended, debug = self.detector.detect(frame)
        self.assertTrue(any(
            item.color == "green" and item.shape == "rectangle"
            for item in extended
        ))
        self.assertEqual(debug.shape, frame.shape)

    def test_verification_skips_old_frames_from_previous_camera_pitch(self):
        frame = np.full((360, 640, 3), 110, dtype=np.uint8)
        cv2.rectangle(frame, (250, 205), (301, 254), (0, 180, 0), -1)
        self.config.target_sample_frames = 4
        self.config.target_verify_frames = 3
        self.config.target_frame_interval_sec = 0.005
        self.config.target_min_confidence = 0.40
        self.config.target_save_confidence = 0.55

        class TimedCamera:
            def __init__(self):
                self.items = [
                    (frame, 100.0), (frame, 101.0),
                    (frame, 102.0), (frame, 103.0),
                    (frame, 104.0),
                ]
                self.index = 0

            def latest_with_timestamp(self, max_age_sec=0.6):
                if self.index >= len(self.items):
                    return None
                result = self.items[self.index]
                self.index += 1
                return result

        # Ignore the cached frame at timestamp 100 (before pitch settled).
        verified, _ = self.detector.verify_latest(
            TimedCamera(), not_before=100.0
        )
        self.assertTrue(any(
            item.detection.color == "green"
            for item in verified
        ))

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
