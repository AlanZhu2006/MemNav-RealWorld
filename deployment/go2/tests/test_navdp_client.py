import pathlib
import sys
import unittest
import base64
import hashlib
import json

import cv2
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
        self.data = None
        self.timeout = None

    def post(self, url, files, timeout, data=None):
        self.url = url
        self.files = files
        self.data = data
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

    def test_imagegoal_preserves_audit_receipt_and_goal_ack(self):
        client = NavDPClient("http://127.0.0.1:8888", 2.0, 30.0)
        session = _Session()
        client.session = session
        image = np.full((8, 12, 3), 100, dtype=np.uint8)

        client.imagegoal_step(
            image, image, np.ones((8, 12), dtype=np.float32),
            installed_goal_sha256="abc123",
        )

        self.assertEqual(session.data, {"installed_goal_sha256": "abc123"})
        self.assertEqual(client.last_plan_receipt, {})


if __name__ == "__main__":
    unittest.main()


class _PhaseResponse:
    def __init__(self, payload, status_code=200):
        self.payload = payload
        self.status_code = status_code
        self.text = str(payload)

    def json(self):
        return self.payload

    def raise_for_status(self):
        if self.status_code >= 400:
            raise RuntimeError(str(self.payload))


class _PhaseSession:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []
        self.data = []

    def post(self, url, files=None, timeout=None, data=None, json=None):
        self.calls.append((url, files, timeout))
        self.data.append(data)
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

    def test_memory_step_binds_source_rgbd_stamps_without_copying_depth(self):
        client, session = self._client(
            [_PhaseResponse({"phase": "memory_recording", "frame_idx": 0,
                             "frames_recorded": 1})]
        )
        rgb = np.full((8, 12, 3), 90, dtype=np.uint8)
        source = {
            "schema": "memnav_rgbd_source_observation_v1",
            "rgb_stamp_ns": 123,
            "depth_stamp_ns": 124,
            "pair_received_ros_ns": 130,
        }

        client.memory_step(rgb, source_observation=source)

        self.assertEqual(json.loads(session.data[0]["source_observation"]), source)
        self.assertEqual(set(session.calls[0][1]), {"image"})

    def test_novel_imagegoal_step_uses_atomic_recording_endpoint(self):
        client, session = self._client([_PhaseResponse({
            "trajectory": [[[0.1, 0.0, 0.0]]],
            "all_trajectory": [[[[0.1, 0.0, 0.0]]]],
            "all_values": [[1.0]],
            "phase": "memory_recording",
            "frames_recorded": 1,
            "novel_recording": True,
        })])
        rgb = np.full((8, 12, 3), 90, dtype=np.uint8)
        depth = np.ones((8, 12), dtype=np.float32)

        trajectory, candidates, values = client.novel_imagegoal_step(
            rgb, rgb, depth
        )

        url, files, _ = session.calls[0]
        self.assertEqual(
            url, "http://127.0.0.1:18889/novel_imagegoal_step"
        )
        self.assertEqual(set(files), {"image", "goal", "depth"})
        self.assertEqual(trajectory.shape, (1, 1, 3))
        self.assertEqual(candidates.shape, (1, 1, 1, 3))
        self.assertEqual(values.shape, (1, 1))
        self.assertTrue(client.last_plan_receipt["novel_recording"])

    def test_goal_candidate_posts_rgb_only(self):
        client, session = self._client(
            [_PhaseResponse({"candidate_id": 0, "captured_after_frame": 5,
                             "appended_to_memory": False})]
        )
        receipt = client.goal_candidate(np.full((8, 12, 3), 90, dtype=np.uint8))
        self.assertFalse(receipt["appended_to_memory"])
        self.assertEqual(session.calls[0][0],
                         "http://127.0.0.1:18889/goal_candidate")
        self.assertEqual(session.data[0], {"validate_support": "0"})

    def test_auto_goal_candidate_requests_read_only_support_validation(self):
        client, session = self._client(
            [_PhaseResponse({"candidate_id": 0, "registered": False})]
        )
        receipt = client.goal_candidate(
            np.full((8, 12, 3), 90, dtype=np.uint8),
            validate_support=True,
        )
        self.assertFalse(receipt["registered"])
        self.assertEqual(session.data[0], {"validate_support": "1"})

    def test_goal_candidate_carries_depth_for_evaluator_only(self):
        client, session = self._client(
            [_PhaseResponse({"candidate_id": 0, "registered": True})]
        )
        rgb = np.full((8, 12, 3), 90, dtype=np.uint8)
        depth = np.full((8, 12), 1.25, dtype=np.float32)
        client.goal_candidate(rgb, evaluation_depth_m=depth)
        _, files, _ = session.calls[0]
        self.assertEqual(set(files), {"image", "evaluation_depth"})
        self.assertEqual(files["evaluation_depth"][2], "image/png")
        self.assertEqual(
            session.data[0],
            {
                "validate_support": "0",
                "evaluation_depth_scale_m": "0.001",
            },
        )
        encoded_depth = files["evaluation_depth"][1]
        decoded_depth = cv2.imdecode(
            np.frombuffer(encoded_depth, dtype=np.uint8), cv2.IMREAD_UNCHANGED
        )
        self.assertEqual(int(decoded_depth[0, 0]), 1250)

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

    def test_prepare_revisit_verifies_and_decodes_selected_goal(self):
        rgb = np.full((8, 12, 3), [20, 80, 140], dtype=np.uint8)
        jpeg = NavDPClient._encode_rgb(rgb)
        digest = hashlib.sha256(jpeg).hexdigest()
        client, session = self._client([_PhaseResponse({
            "phase": "revisit_query",
            "selected_goal": {"candidate_id": 2, "sha256": digest},
            "goal_image_jpeg_base64": base64.b64encode(jpeg).decode("ascii"),
        })])

        receipt, decoded = client.prepare_revisit()

        self.assertEqual(receipt["selected_goal"]["candidate_id"], 2)
        self.assertNotIn("goal_image_jpeg_base64", receipt)
        self.assertEqual(decoded.shape, rgb.shape)
        self.assertEqual(session.calls[0][0],
                         "http://127.0.0.1:18889/prepare_revisit")

    def test_loaded_dataset_prepare_sends_query_start_and_keeps_eval_depth(self):
        rgb = np.full((8, 12, 3), [20, 80, 140], dtype=np.uint8)
        query = np.full((8, 12, 3), [90, 40, 10], dtype=np.uint8)
        jpeg = NavDPClient._encode_rgb(rgb)
        depth_png = NavDPClient._encode_depth(
            np.full((8, 12), 1.0, dtype=np.float32)
        )
        digest = hashlib.sha256(jpeg).hexdigest()
        client, session = self._client([_PhaseResponse({
            "phase": "revisit_query",
            "selected_goal": {"candidate_id": 0, "sha256": digest},
            "goal_image_jpeg_base64": base64.b64encode(jpeg).decode("ascii"),
            "goal_evaluation_depth_png_base64": (
                base64.b64encode(depth_png).decode("ascii")
            ),
            "goal_evaluation_depth_scale_m": 1.0e-3,
        })])

        receipt, _ = client.prepare_revisit(query_start_rgb=query)

        _, files, _ = session.calls[0]
        self.assertEqual(set(files), {"query_start"})
        self.assertEqual(client.last_goal_jpeg, jpeg)
        self.assertEqual(client.last_goal_evaluation_depth_png, depth_png)
        self.assertEqual(client.last_goal_evaluation_depth_scale_m, 1.0e-3)
        self.assertNotIn("goal_evaluation_depth_png_base64", receipt)

    def test_prepare_external_revisit_uploads_and_verifies_frozen_goal(self):
        rgb = np.full((8, 12, 3), [30, 90, 150], dtype=np.uint8)
        jpeg = NavDPClient._encode_rgb(rgb)
        digest = hashlib.sha256(jpeg).hexdigest()
        client, session = self._client([_PhaseResponse({
            "phase": "revisit_query",
            "goal_selection_contract": "operator_frozen_external_v1",
            "selected_goal": {
                "candidate_id": None,
                "sha256": digest,
                "goal_source": "operator_frozen_external",
            },
            "goal_image_jpeg_base64": base64.b64encode(jpeg).decode("ascii"),
        })])

        receipt, decoded = client.prepare_revisit_goal(rgb)

        url, files, _ = session.calls[0]
        self.assertEqual(
            url, "http://127.0.0.1:18889/prepare_revisit_goal"
        )
        self.assertEqual(set(files), {"goal"})
        self.assertEqual(receipt["selected_goal"]["sha256"], digest)
        self.assertNotIn("goal_image_jpeg_base64", receipt)
        self.assertEqual(decoded.shape, rgb.shape)

    def test_contract_rejection_surfaces_hub_error(self):
        client, _ = self._client([_PhaseResponse(
            {"error": "goal queries are forbidden during memory recording"},
            status_code=400,
        )])
        with self.assertRaises(RuntimeError) as context:
            client.begin_revisit()
        self.assertIn("forbidden during memory recording",
                      str(context.exception))

    def test_reset_accepts_matching_cec_terminal_contract(self):
        client, _ = self._client([_PhaseResponse({
            "algo": "cec_hybrid_navdp",
            "protocol_version": 3,
            "terminal_handoff_schema": (
                "cec_direct_bearing_handoff_v2_20260824"
            ),
        })])
        algorithm = client.reset(np.eye(3, dtype=np.float32))
        self.assertEqual(algorithm, "cec_hybrid_navdp")

    def test_reset_rejects_stale_terminal_contract(self):
        client, _ = self._client([_PhaseResponse({
            "algo": "cec_hybrid_navdp",
            "protocol_version": 3,
            "terminal_handoff_schema": "cec_local_pose_handoff_v1_20260824",
        })])
        with self.assertRaisesRegex(RuntimeError, "runtime contract mismatch"):
            client.reset(np.eye(3, dtype=np.float32))
