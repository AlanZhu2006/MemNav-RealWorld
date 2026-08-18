import pathlib
import sys
import unittest

import numpy as np


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from navdp_client import NavDPClient  # noqa: E402


class _Response:
    @staticmethod
    def raise_for_status():
        return None

    @staticmethod
    def json():
        return {
            "trajectory": [[[0.1, 0.0, 0.0]]],
            "all_trajectory": [[[[0.1, 0.0, 0.0]]]],
            "all_values": [[1.0]],
        }


class _Session:
    def __init__(self):
        self.url = ""
        self.files = None
        self.timeout = None

    def post(self, url, files, timeout):
        self.url = url
        self.files = files
        self.timeout = timeout
        return _Response()


class NavDPClientImageGoalTests(unittest.TestCase):
    def test_imagegoal_request_contains_goal_rgb_and_depth(self):
        client = NavDPClient("http://127.0.0.1:8888", 2.0, 30.0)
        session = _Session()
        client.session = session
        goal = np.full((8, 12, 3), 50, dtype=np.uint8)
        rgb = np.full((8, 12, 3), 100, dtype=np.uint8)
        depth = np.full((8, 12), 1.5, dtype=np.float32)

        trajectory, candidates, values = client.imagegoal_step(goal, rgb, depth)

        self.assertEqual(session.url, "http://127.0.0.1:8888/imagegoal_step")
        self.assertEqual(set(session.files), {"image", "goal", "depth"})
        self.assertEqual(session.files["goal"][2], "image/jpeg")
        self.assertEqual(session.files["depth"][2], "image/png")
        self.assertEqual(session.timeout, (2.0, 30.0))
        self.assertEqual(trajectory.shape, (1, 1, 3))
        self.assertEqual(candidates.shape, (1, 1, 1, 3))
        self.assertEqual(values.shape, (1, 1))


if __name__ == "__main__":
    unittest.main()
