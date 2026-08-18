import pathlib
import sys
import unittest

import cv2
import numpy as np


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from visual_goal_verifier import VisualGoalVerifier  # noqa: E402


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


class VisualGoalVerifierTests(unittest.TestCase):
    def setUp(self):
        self.target = textured_image(7)
        self.depth = np.full(self.target.shape[:2], 2.0, dtype=np.float32)

    def test_requires_consecutive_geometric_matches(self):
        verifier = VisualGoalVerifier(
            self.target,
            self.depth,
            image_width=320,
            required_consecutive_matches=3,
        )
        first = verifier.evaluate(self.target, self.depth)
        second = verifier.evaluate(self.target, self.depth)
        third = verifier.evaluate(self.target, self.depth)
        self.assertTrue(first.matched)
        self.assertFalse(first.confirmed)
        self.assertFalse(second.confirmed)
        self.assertTrue(third.confirmed)
        self.assertTrue(third.goal_object_confirmed)
        self.assertGreaterEqual(third.inliers, 20)
        self.assertLess(third.median_depth_error_m, 0.01)

    def test_unrelated_view_resets_confirmation(self):
        verifier = VisualGoalVerifier(
            self.target,
            self.depth,
            image_width=320,
            required_consecutive_matches=2,
        )
        verifier.evaluate(self.target, self.depth)
        unrelated = verifier.evaluate(textured_image(19), self.depth)
        self.assertFalse(unrelated.matched)
        self.assertFalse(unrelated.confirmed)
        self.assertFalse(unrelated.goal_object_matched)
        self.assertFalse(unrelated.goal_object_confirmed)
        self.assertEqual(unrelated.consecutive_matches, 0)
        self.assertEqual(unrelated.consecutive_goal_object_matches, 0)

    def test_closer_matching_object_is_not_exact_view(self):
        height, width = self.target.shape[:2]
        transform = cv2.getRotationMatrix2D(
            ((width - 1) / 2.0, (height - 1) / 2.0), 0.0, 1.6
        )
        closer = cv2.warpAffine(
            self.target,
            transform,
            (width, height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REFLECT,
        )
        closer_depth = np.full_like(self.depth, 1.2)
        verifier = VisualGoalVerifier(
            self.target,
            self.depth,
            image_width=320,
            required_consecutive_matches=2,
            goal_object_required_consecutive_matches=2,
        )
        first = verifier.evaluate(closer, closer_depth)
        second = verifier.evaluate(closer, closer_depth)
        self.assertFalse(second.confirmed)
        self.assertTrue(first.goal_object_matched)
        self.assertTrue(second.goal_object_confirmed)
        self.assertAlmostEqual(second.median_depth_delta_m, -0.8, places=2)
        self.assertLess(second.depth_delta_mad_m, 0.01)

    def test_matching_view_farther_than_reference_is_not_object_arrival(self):
        verifier = VisualGoalVerifier(self.target, self.depth, image_width=320)
        result = verifier.evaluate(
            self.target,
            np.full_like(self.depth, 2.6),
        )
        self.assertFalse(result.goal_object_matched)
        self.assertEqual(
            result.goal_object_reason, "goal_object_farther_than_reference"
        )

    def test_depth_mismatch_rejects_same_rgb(self):
        verifier = VisualGoalVerifier(
            self.target,
            self.depth,
            image_width=320,
            max_median_depth_error_m=0.25,
        )
        result = verifier.evaluate(
            self.target,
            np.full_like(self.depth, 3.0),
        )
        self.assertFalse(result.matched)
        self.assertEqual(result.reason, "depth_mismatch")

    def test_reference_requires_textured_valid_depth(self):
        blank = np.zeros_like(self.target)
        with self.assertRaises(ValueError):
            VisualGoalVerifier(blank, self.depth, image_width=320)
        with self.assertRaises(ValueError):
            VisualGoalVerifier(self.target, np.zeros_like(self.depth), image_width=320)


if __name__ == "__main__":
    unittest.main()
