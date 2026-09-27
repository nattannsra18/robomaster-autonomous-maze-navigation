"""Offline scan-cache regression for Final Round 1 V05.

No robot, chassis command, or camera connection is required.
"""

import sys
import types
import unittest


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


from classwork8.tof_camera_round1_v05 import _should_reuse_scan


class ScanReuseTests(unittest.TestCase):
    def setUp(self):
        self.cell = (2, -1)
        self.edges = {
            (2, -1, 0): "WALL",
            (2, -1, 1): "OPEN",
            (2, -1, 2): "OPEN",
            (2, -1, 3): "WALL",
        }

    def test_first_visit_never_skips_scan(self):
        self.assertFalse(_should_reuse_scan(
            self.cell, set(), self.edges, True, False
        ))

    def test_completed_visited_cell_can_reuse_scan(self):
        self.assertTrue(_should_reuse_scan(
            self.cell, {self.cell}, self.edges, True, False
        ))

    def test_operator_can_disable_reuse(self):
        self.assertFalse(_should_reuse_scan(
            self.cell, {self.cell}, self.edges, False, False
        ))

    def test_rescan_request_is_always_honored(self):
        self.assertFalse(_should_reuse_scan(
            self.cell, {self.cell}, self.edges, True, True
        ))

    def test_missing_or_unknown_edge_requires_fresh_scan(self):
        edges = dict(self.edges)
        edges.pop((2, -1, 1))
        self.assertFalse(_should_reuse_scan(
            self.cell, {self.cell}, edges, True, False
        ))
        edges[(2, -1, 1)] = "UNKNOWN"
        self.assertFalse(_should_reuse_scan(
            self.cell, {self.cell}, edges, True, False
        ))


if __name__ == "__main__":
    unittest.main()
