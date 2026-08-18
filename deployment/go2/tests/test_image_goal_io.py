import pathlib
import sys
import tempfile
import unittest

import numpy as np


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from image_goal_io import (  # noqa: E402
    depth_array_to_meters,
    load_depth_image,
    load_rgb_image,
    save_depth_image,
    save_rgb_image,
)


class ImageGoalIoTests(unittest.TestCase):
    def test_png_round_trip_preserves_rgb(self):
        image = np.zeros((7, 11, 3), dtype=np.uint8)
        image[..., 0] = 231
        image[..., 1] = 92
        image[..., 2] = 17
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "goal.png"
            saved = save_rgb_image(path, image)
            self.assertEqual(saved, path.resolve())
            np.testing.assert_array_equal(load_rgb_image(path), image)

    def test_missing_goal_is_rejected(self):
        with self.assertRaises(FileNotFoundError):
            load_rgb_image("/definitely/missing/navdp-image-goal.png")

    def test_depth_png_round_trip_uses_millimetres(self):
        depth = np.array([[0.0, 0.25, 1.234], [2.0, 4.5, np.nan]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "goal_depth.png"
            saved = save_depth_image(path, depth)
            self.assertEqual(saved, path.resolve())
            np.testing.assert_allclose(
                load_depth_image(path),
                np.nan_to_num(depth),
                atol=0.001,
            )

    def test_integer_depth_array_applies_sensor_scale(self):
        raw = np.array([[0, 250, 1234]], dtype=np.uint16)
        np.testing.assert_allclose(
            depth_array_to_meters(raw, 0.001),
            np.array([[0.0, 0.25, 1.234]], dtype=np.float32),
        )


if __name__ == "__main__":
    unittest.main()
