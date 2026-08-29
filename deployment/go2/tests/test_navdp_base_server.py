import io
import json
import sys
import types
import unittest
from unittest import mock

import numpy as np
from PIL import Image

policy_agent_stub = types.ModuleType("policy_agent")
policy_agent_stub.NavDP_Agent = object
sys.modules.setdefault("policy_agent", policy_agent_stub)

from deployment.go2 import navdp_base_server


class FakeNavigator:
    batch_size = 1

    def __init__(self):
        self.point_goal = None
        self.image_goal_shape = None

    def step_point_image_goal(self, point_goal, image_goal, rgb, depth):
        self.point_goal = point_goal
        self.image_goal_shape = image_goal.shape
        trajectory = np.zeros((1, 24, 3), dtype=np.float32)
        candidates = np.zeros((1, 2, 24, 3), dtype=np.float32)
        values = np.zeros((1, 2), dtype=np.float32)
        return trajectory, candidates, values, None


class ResetRouteTests(unittest.TestCase):
    def tearDown(self):
        navdp_base_server.navigator = None

    def test_reset_uses_original_navdp_constructor_contract(self):
        constructed = {}

        class FakeAgent:
            def __init__(self, intrinsic, **kwargs):
                constructed["intrinsic"] = intrinsic
                constructed["kwargs"] = kwargs

            def reset(self, batch_size, stop_threshold):
                constructed["reset"] = (batch_size, stop_threshold)

        navdp_base_server.navigator = None
        navdp_base_server.checkpoint_path = "/tmp/navdp.ckpt"
        with mock.patch.object(navdp_base_server, "NavDP_Agent", FakeAgent):
            response = navdp_base_server.app.test_client().post(
                "/navigator_reset",
                json={
                    "intrinsic": [[1.0, 0.0, 2.0], [0.0, 1.0, 3.0], [0.0, 0.0, 1.0]],
                    "batch_size": 1,
                    "stop_threshold": -2.0,
                },
            )

        self.assertEqual(response.status_code, 200)
        self.assertNotIn("enable_visualization", constructed["kwargs"])
        self.assertEqual(constructed["reset"], (1, -2.0))


class MixedImagePointGoalRouteTests(unittest.TestCase):
    def setUp(self):
        self.navigator = FakeNavigator()
        navdp_base_server.navigator = self.navigator
        self.client = navdp_base_server.app.test_client()

    def tearDown(self):
        navdp_base_server.navigator = None

    def test_mixed_route_decodes_bearing_and_returns_trajectory(self):
        image_buffer = io.BytesIO()
        Image.new("RGB", (8, 6), color=(20, 40, 60)).save(
            image_buffer, format="PNG"
        )
        image_buffer.seek(0)
        rgb = np.zeros((1, 6, 8, 3), dtype=np.uint8)
        depth = np.ones((1, 6, 8, 1), dtype=np.float32)

        with mock.patch.object(
            navdp_base_server, "_decode_rgb_depth", return_value=(rgb, depth)
        ):
            response = self.client.post(
                "/navdp_step_ip_mixgoal",
                data={
                    "goal_data": json.dumps(
                        {"goal_x": [1.5], "goal_y": [-2.0]}
                    ),
                    "image_goal": (image_buffer, "goal.png"),
                },
                content_type="multipart/form-data",
            )

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["trajectory"][0][0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(
            self.navigator.point_goal, [[1.5, -2.0, 0.0]]
        )
        self.assertEqual(self.navigator.image_goal_shape, (1, 6, 8, 3))


if __name__ == "__main__":
    unittest.main()
