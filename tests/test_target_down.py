import unittest

import cv2
import numpy as np

from final_assignment.target_down import (
    STATUS_DOWN_CONFIRMED,
    STATUS_STILL_UP,
    STATUS_UNCERTAIN,
    TargetDownConfig,
    TargetDownVerifier,
)
from final_assignment.target_detector import Detection


def make_detection(
    x=60,
    y=40,
    w=40,
    h=40,
    color="green",
    shape="square",
):
    contour = np.array(
        [
            [[x, y]],
            [[x + w, y]],
            [[x + w, y + h]],
            [[x, y + h]],
        ],
        dtype=np.int32,
    )

    return Detection(
        color=color,
        shape=shape,
        contour=contour,
        bbox=(x, y, w, h),
        centroid=(x + w // 2, y + h // 2),
        area=float(w * h),
        perimeter=float(2 * (w + h)),
        vertices=4,
        aspect_ratio=float(w) / float(h),
        circularity=0.78,
        rectangularity=1.0,
        color_fill_ratio=1.0,
    )


def scene(
    with_target=True,
    brightness=100,
):
    frame = np.full(
        (120, 180, 3),
        brightness,
        dtype=np.uint8,
    )

    mask = np.zeros(
        (120, 180),
        dtype=np.uint8,
    )

    if with_target:
        cv2.rectangle(
            frame,
            (60, 40),
            (100, 80),
            (0, 180, 0),
            -1,
        )
        cv2.rectangle(
            mask,
            (60, 40),
            (100, 80),
            255,
            -1,
        )

    return frame, mask


class TargetDownVerifierTests(
    unittest.TestCase
):
    def setUp(self):
        self.config = TargetDownConfig(
            settle_sec=0.0,
            timeout_sec=2.0,
            down_confirm_frames=3,
            still_up_confirm_frames=3,
            patch_change_min=0.05,
        )

    def test_unchanged_target_reports_still_up(self):
        verifier = TargetDownVerifier(
            self.config
        )

        frame, mask = scene(True)
        detection = make_detection()

        verifier.capture_baseline(
            frame,
            mask,
            detection,
        )

        verifier.begin_verification(
            now=10.0
        )

        result = None

        for index in range(3):
            result = verifier.update(
                frame,
                mask,
                [detection],
                now=10.1 + index * 0.1,
            )

        self.assertEqual(
            result.status,
            STATUS_STILL_UP,
        )

    def test_removed_target_and_scene_change_reports_down(self):
        verifier = TargetDownVerifier(
            self.config
        )

        frame, mask = scene(True)
        detection = make_detection()

        verifier.capture_baseline(
            frame,
            mask,
            detection,
        )

        verifier.begin_verification(
            now=20.0
        )

        after, after_mask = scene(False)

        cv2.line(
            after,
            (55, 85),
            (110, 85),
            (40, 40, 40),
            4,
        )

        result = None

        for index in range(3):
            result = verifier.update(
                after,
                after_mask,
                [],
                now=20.1 + index * 0.1,
            )

        self.assertEqual(
            result.status,
            STATUS_DOWN_CONFIRMED,
        )

    def test_mask_loss_without_visual_change_is_not_down(self):
        verifier = TargetDownVerifier(
            self.config
        )

        frame, mask = scene(True)
        detection = make_detection()

        verifier.capture_baseline(
            frame,
            mask,
            detection,
        )

        verifier.begin_verification(
            now=30.0
        )

        lost_mask = np.zeros_like(mask)

        result = None

        for index in range(8):
            result = verifier.update(
                frame,
                lost_mask,
                [],
                now=30.1 + index * 0.3,
            )

        self.assertEqual(
            result.status,
            STATUS_UNCERTAIN,
        )


if __name__ == "__main__":
    unittest.main()
