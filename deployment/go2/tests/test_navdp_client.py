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


class _PhaseResponse:
    def __init__(self, payload, status_code=200):
        self.payload = payload
        self.status_code = status_code
        self.text = str(payload)

    def json(self):
        return self.payload


class _PhaseSession:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def post(self, url, files=None, timeout=None):
        self.calls.append((url, files, timeout))
        return self.responses.pop(0)


class NavDPClientPhaseProtocolTests(unittest.TestCase):
    def _client(self, responses):
        client = NavDPClient("http://127.0.0.1:18889", 2.0, 30.0)
        session = _PhaseSession(responses)
        client.session = session
        return client, session

    def test_memory_step_posts_rgb_only(self):
        client, session = self._client(
            [_PhaseResponse({"phase": "memory_recording", "frame_idx": 0,
                             "frames_recorded": 1})]
        )
        rgb = np.full((8, 12, 3), 90, dtype=np.uint8)
        receipt = client.memory_step(rgb)
        self.assertEqual(receipt["frames_recorded"], 1)
        url, files, _ = session.calls[0]
        self.assertEqual(url, "http://127.0.0.1:18889/memory_step")
        self.assertEqual(set(files), {"image"})

    def test_goal_candidate_posts_rgb_only(self):
        client, session = self._client(
            [_PhaseResponse({"candidate_id": 0, "captured_after_frame": 5,
                             "appended_to_memory": False})]
        )
        receipt = client.goal_candidate(np.full((8, 12, 3), 90, dtype=np.uint8))
        self.assertFalse(receipt["appended_to_memory"])
        self.assertEqual(session.calls[0][0],
                         "http://127.0.0.1:18889/goal_candidate")

    def test_begin_revisit_returns_warmup_receipt(self):
        client, session = self._client(
            [_PhaseResponse({"phase": "revisit_query", "frames_recorded": 42,
                             "navdp_warmup_frames": 6})]
        )
        receipt = client.begin_revisit()
        self.assertEqual(receipt["phase"], "revisit_query")
        self.assertEqual(session.calls[0][0],
                         "http://127.0.0.1:18889/begin_revisit")
        self.assertIsNone(session.calls[0][1])

    def test_contract_rejection_surfaces_hub_error(self):
        client, _ = self._client([_PhaseResponse(
            {"error": "goal queries are forbidden during memory recording"},
            status_code=400,
        )])
        with self.assertRaises(RuntimeError) as context:
            client.begin_revisit()
        self.assertIn("forbidden during memory recording",
                      str(context.exception))
