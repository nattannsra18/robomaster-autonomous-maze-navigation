import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from classwork8.config import Classwork8Config
from classwork8.target_detection import (
    COLOR_RANGES,
    TargetDetector,
    TargetRegistry,
    VerifiedTarget,
)
from classwork8.target_map_visual import target_plot_geometry


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

    def test_yellow_orange_candidate_hue_bands_do_not_overlap(self):
        yellow_low = COLOR_RANGES["yellow"][0][0][0]
        orange_high = COLOR_RANGES["orange"][0][1][0]
        self.assertGreater(yellow_low, orange_high)

    def test_warm_yellow_is_not_saved_as_orange(self):
        # Synthetic, controlled illumination: warm yellow ~H22 and clear
        # yellow ~H29 must retain YELLOW after CLAHE and contour scoring.
        for hue in (22, 29):
            frame = np.full((360, 640, 3), 120, dtype=np.uint8)
            hsv = np.uint8([[[hue, 220, 240]]])
            bgr = tuple(int(v) for v in cv2.cvtColor(
                hsv, cv2.COLOR_HSV2BGR
            )[0, 0])
            cv2.rectangle(frame, (250, 145), (380, 215), bgr, -1)
            detections, _debug = self.detector.detect(frame)
            hits = [
                item for item in detections
                if 250 <= item.centroid[0] <= 380
                and 145 <= item.centroid[1] <= 215
            ]
            self.assertTrue(hits, "No warm-yellow detection at H={}".format(hue))
            self.assertTrue(
                all(item.color == "yellow" for item in hits),
                "Warm yellow classified as {}".format([item.color for item in hits]),
            )

    def test_true_orange_remains_orange(self):
        frame = np.full((360, 640, 3), 120, dtype=np.uint8)
        hsv = np.uint8([[[13, 225, 240]]])
        bgr = tuple(int(v) for v in cv2.cvtColor(
            hsv, cv2.COLOR_HSV2BGR
        )[0, 0])
        cv2.rectangle(frame, (250, 145), (380, 215), bgr, -1)
        detections, _debug = self.detector.detect(frame)
        hits = [
            item for item in detections
            if 250 <= item.centroid[0] <= 380
            and 145 <= item.centroid[1] <= 215
        ]
        self.assertTrue(hits)
        self.assertTrue(all(item.color == "orange" for item in hits))

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
        # Wall-target profile: retain the narrower 0.82 ROI for the earlier
        # white-foam-wall sample. Ground-sign mode intentionally uses 0.94.
        self.config.target_roi_bottom_ratio = 0.82
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

    def test_verified_target_survives_last_frame_glare(self):
        # A floor sign visible for four distinct frames must remain valid
        # even if the last camera frames are lost under changing lighting.
        clear = np.full((360, 640, 3), 110, dtype=np.uint8)
        cv2.rectangle(clear, (250, 208), (300, 257), (0, 180, 0), -1)
        glare = np.full((360, 640, 3), 110, dtype=np.uint8)

        class SequenceCamera:
            def __init__(self):
                self.frames = [clear] * 4 + [glare] * 4
                self.index = 0

            def latest_with_timestamp(self, max_age_sec=0.6):
                if self.index >= len(self.frames):
                    return None
                result = (
                    self.frames[self.index],
                    float(100 + self.index),
                )
                self.index += 1
                return result

        self.config.target_sample_frames = 8
        self.config.target_verify_frames = 4
        self.config.target_frame_interval_sec = 0.005
        verified, _debug = self.detector.verify_latest(SequenceCamera())
        self.assertTrue(any(
            item.detection.color == "green"
            and item.detection.shape == "square"
            and item.verified_frames >= 4
            for item in verified
        ))

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

    def test_far_sign_is_a_sighting_not_an_observer_cell_target(self):
        frame = np.full((360, 640, 3), 110, dtype=np.uint8)
        cv2.rectangle(frame, (275, 214), (315, 254), (0, 0, 215), -1)
        detections, _debug = self.detector.detect(frame)
        sign = next(item for item in detections
                    if item.color == "red" and item.shape == "square")
        verified = VerifiedTarget(
            detection=sign, verified_frames=4,
            confidence=max(0.80, sign.confidence),
        )
        registry = TargetRegistry(self.config)
        far = registry.add_verified(
            verified, (0, 0), 3, 136.0,
            range_confirmed_wall=False, camera_pitch_deg=-10.0,
        )

        self.assertEqual(far["status"], "SIGHTING_ONLY")
        self.assertIsNone(far["estimated_target_xy_m"])
        self.assertEqual(far["approach_cells"], [])
        self.assertEqual(far["observation_cells"], [[0, 0]])
        self.assertEqual(far["sighting_cell_hint"], [0, 2])
        self.assertFalse(far["round2_position_ready"])
        hint, source, sighting = target_plot_geometry(
            far, self.config.cell_size_m
        )
        self.assertTrue(sighting)
        self.assertEqual(source, (0.0, 0.0))
        # Unlocalized signs stay within their observer cell in the GUI,
        # even when the raw ToF hint beyond a low foam wall is much farther.
        self.assertLess(hint[1], self.config.cell_size_m / 2.0)
        self.assertGreater(hint[1], 0.0)

        with tempfile.TemporaryDirectory() as folder:
            registry.save(Path(folder))
            import json
            saved = json.loads(Path(folder, "targets.json").read_text(
                encoding="utf-8"
            ))
            self.assertIsNone(
                saved["targets"][0]["estimated_target_xy_m"]
            )

    def test_same_view_close_observation_upgrades_sighting_tentatively(self):
        frame = np.full((360, 640, 3), 110, dtype=np.uint8)
        cv2.rectangle(frame, (275, 214), (315, 254), (0, 0, 215), -1)
        detections, _debug = self.detector.detect(frame)
        sign = next(item for item in detections
                    if item.color == "red" and item.shape == "square")
        verified = VerifiedTarget(
            detection=sign, verified_frames=4,
            confidence=max(0.80, sign.confidence),
        )
        registry = TargetRegistry(self.config)
        far = registry.add_verified(
            verified, (0, 0), 3, 135.0,
            range_confirmed_wall=False,
        )
        near = registry.add_verified(
            verified, (0, 0), 3, 29.0,
            range_confirmed_wall=True,
        )
        self.assertEqual(far["target_id"], near["target_id"])
        self.assertEqual(len(registry.targets), 1)
        self.assertEqual(near["status"], "POSITION_CANDIDATE")
        self.assertEqual(near["approach_cells"], [[0, 0]])
        self.assertIsNotNone(near["estimated_target_xy_m"])
        self.assertFalse(near["round2_position_ready"])

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
