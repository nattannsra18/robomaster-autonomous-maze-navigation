"""Regression tests for OpenCV HoughLinesP array layouts.

No RoboMaster connection, camera or robot motion is required.
"""

import unittest
from unittest.mock import patch

import numpy as np

from classwork8.config import Classwork8Config
from classwork8.vision import CorridorVision


class HoughLineLayoutTests(unittest.TestCase):
    def setUp(self):
        self.vision = CorridorVision.__new__(CorridorVision)
        self.vision.config = Classwork8Config()
        self.vision._last_error = 0.0
        self.frame = np.zeros((360, 640, 3), dtype=np.uint8)

    def _analyse_lines(self, lines):
        with patch("classwork8.vision.cv2.HoughLinesP", return_value=lines):
            return self.vision._analyse(self.frame)

    def test_opencv_n_1_4_shape(self):
        lines = np.array([
            [[90, 15, 140, 170]],
            [[550, 15, 500, 170]],
        ], dtype=np.int32)
        estimate = self._analyse_lines(lines)
        self.assertIsNotNone(estimate.left_x)
        self.assertIsNotNone(estimate.right_x)
        self.assertEqual(estimate.frame_width, 640)

    def test_opencv_n_4_shape(self):
        lines = np.array([
            [90, 15, 140, 170],
            [550, 15, 500, 170],
        ], dtype=np.int32)
        estimate = self._analyse_lines(lines)
        self.assertIsNotNone(estimate.left_x)
        self.assertIsNotNone(estimate.right_x)
        self.assertEqual(estimate.frame_height, 360)

    def test_no_lines_does_not_raise(self):
        estimate = self._analyse_lines(None)
        self.assertEqual(estimate.confidence, 0.0)
        self.assertIsNone(estimate.corridor_center_x)


if __name__ == "__main__":
    unittest.main()
