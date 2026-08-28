import pathlib
import sys
import threading
import unittest

import cv2
import numpy as np


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from navdp_ros_node import NavDPGo2Adapter  # noqa: E402
from rgb_goal_arrival import RgbGoalArrivalVerifier  # noqa: E402


def textured_image(seed: int) -> np.ndarray:
    random = np.random.default_rng(seed)
    image = random.integers(0, 256, (240, 320, 3), dtype=np.uint8)
    image = cv2.GaussianBlur(image, (3, 3), 0)
    for _ in range(30):
        center = (
            int(random.integers(10, image.shape[1] - 10)),
            int(random.integers(10, image.shape[0] - 10)),
        )
        color = tuple(int(value) for value in random.integers(0, 256, 3))
        cv2.circle(image, center, int(random.integers(3, 10)), color, -1)
    return image


class RgbGoalArrivalVerifierTests(unittest.TestCase):
    def setUp(self):
        self.target = textured_image(31)

    def test_requires_three_consecutive_rgb_matches(self):
        verifier = RgbGoalArrivalVerifier(
            self.target,
            image_width=320,
            min_good_matches=30,
            min_inliers=25,
            required_consecutive_matches=3,
        )
        first = verifier.evaluate(self.target)
        second = verifier.evaluate(self.target)
        third = verifier.evaluate(self.target)

        self.assertTrue(first.matched)
        self.assertFalse(first.confirmed)
        self.assertFalse(second.confirmed)
        self.assertTrue(third.confirmed)
        self.assertEqual(third.consecutive_matches, 3)

    def test_unrelated_view_resets_confirmation(self):
        verifier = RgbGoalArrivalVerifier(
            self.target,
            image_width=320,
            min_good_matches=30,
            min_inliers=25,
            required_consecutive_matches=2,
        )
        verifier.evaluate(self.target)
        unrelated = verifier.evaluate(textured_image(44))

        self.assertFalse(unrelated.matched)
        self.assertFalse(unrelated.confirmed)
        self.assertEqual(unrelated.consecutive_matches, 0)

    def test_too_distant_view_is_rejected_by_scale(self):
        height, width = self.target.shape[:2]
        transform = cv2.getRotationMatrix2D(
            ((width - 1) / 2.0, (height - 1) / 2.0), 0.0, 0.65
        )
        distant = cv2.warpAffine(
            self.target,
            transform,
            (width, height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        verifier = RgbGoalArrivalVerifier(
            self.target,
            image_width=320,
            min_good_matches=30,
            min_inliers=25,
        )
        result = verifier.evaluate(distant)

        self.assertFalse(result.matched)
        self.assertEqual(result.reason, "scale_mismatch")


class _Logger:
    def warning(self, _message):
        return None


class AdapterArrivalLatchTests(unittest.TestCase):
    def test_arrival_disables_motion_and_freezes_novel_memory(self):
        adapter = object.__new__(NavDPGo2Adapter)
        adapter._lock = threading.RLock()
        adapter._arrival_latched = False
        adapter._estop = False
        adapter._enabled = True
        adapter.two_phase_episode = True
        adapter._phase = "memory_recording"
        adapter.pause_memory_recording = False
        adapter._frames_recorded = 127
        published = []
        adapter._publish_zero = lambda reason: published.append(("zero", reason))
        adapter._publish_receipt = (
            lambda event, receipt: published.append((event, receipt))
        )
        adapter.get_logger = lambda: _Logger()

        message = type("BoolMessage", (), {"data": True})()
        adapter._on_arrival(message)

        self.assertTrue(adapter._arrival_latched)
        self.assertTrue(adapter._estop)
        self.assertFalse(adapter._enabled)
        self.assertTrue(adapter.pause_memory_recording)
        self.assertEqual(published[0], ("zero", "rgb_imagegoal_arrival"))
        self.assertEqual(published[1][0], "rgb_imagegoal_arrival")
        self.assertEqual(published[1][1]["frames_recorded"], 127)


if __name__ == "__main__":
    unittest.main()
