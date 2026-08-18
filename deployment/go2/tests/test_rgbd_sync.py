import threading
import unittest
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navdp_ros_node import NavDPGo2Adapter


class _Stamp:
    def __init__(self, seconds: float):
        self.sec = int(seconds)
        self.nanosec = int(round((seconds - self.sec) * 1e9))


class _Header:
    def __init__(self, seconds: float):
        self.stamp = _Stamp(seconds)


class _Image:
    def __init__(self, array: np.ndarray, seconds: float, encoding: str):
        self.array = array
        self.header = _Header(seconds)
        self.encoding = encoding


class _Bridge:
    @staticmethod
    def imgmsg_to_cv2(message, desired_encoding):
        return message.array


def make_adapter():
    adapter = object.__new__(NavDPGo2Adapter)
    adapter.max_rgb_depth_skew_s = 0.10
    adapter.depth_scale_m = 0.001
    adapter._bridge = _Bridge()
    adapter._lock = threading.RLock()
    adapter._rgb = None
    adapter._depth_m = None
    adapter._rgbd_monotonic = 0.0
    adapter._rgb_depth_skew_s = None
    adapter.warnings = []
    adapter._warn_throttled = lambda key, message: adapter.warnings.append(
        (key, message)
    )
    return adapter


class RgbdSyncTests(unittest.TestCase):
    def test_valid_pair_updates_rgb_and_metric_depth_together(self):
        adapter = make_adapter()
        rgb = np.full((2, 3, 3), 17, dtype=np.uint8)
        depth = np.full((2, 3), 1250, dtype=np.uint16)

        adapter._on_rgbd(
            _Image(rgb, 10.00, "rgb8"),
            _Image(depth, 10.03, "16UC1"),
        )

        np.testing.assert_array_equal(adapter._rgb, rgb)
        np.testing.assert_allclose(adapter._depth_m, 1.25)
        self.assertGreater(adapter._rgbd_monotonic, 0.0)
        self.assertAlmostEqual(adapter._rgb_depth_skew_s, 0.03, places=6)
        self.assertEqual(adapter.warnings, [])

    def test_excessive_skew_does_not_replace_previous_pair(self):
        adapter = make_adapter()
        previous_rgb = np.ones((2, 3, 3), dtype=np.uint8)
        previous_depth = np.ones((2, 3), dtype=np.float32)
        adapter._rgb = previous_rgb
        adapter._depth_m = previous_depth
        adapter._rgbd_monotonic = 123.0

        adapter._on_rgbd(
            _Image(np.zeros_like(previous_rgb), 10.0, "rgb8"),
            _Image(np.zeros((2, 3), dtype=np.uint16), 10.2, "16UC1"),
        )

        self.assertIs(adapter._rgb, previous_rgb)
        self.assertIs(adapter._depth_m, previous_depth)
        self.assertEqual(adapter._rgbd_monotonic, 123.0)
        self.assertEqual(adapter.warnings[0][0], "rgbd_pair_skew")

    def test_shape_mismatch_does_not_publish_partial_pair(self):
        adapter = make_adapter()

        adapter._on_rgbd(
            _Image(np.zeros((2, 3, 3), dtype=np.uint8), 10.0, "rgb8"),
            _Image(np.zeros((3, 2), dtype=np.uint16), 10.0, "16UC1"),
        )

        self.assertIsNone(adapter._rgb)
        self.assertIsNone(adapter._depth_m)
        self.assertEqual(adapter._rgbd_monotonic, 0.0)
        self.assertEqual(adapter.warnings[0][0], "rgbd_shape")


if __name__ == "__main__":
    unittest.main()
