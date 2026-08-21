import copy
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from deployment.gpu.monocular_depth_runtime import (
    ACTIVE_FROM_FRAME_INDEX,
    bind_monocular_depth_transaction,
    build_monocular_depth_payload,
    compute_first40_scale_receipt,
    decode_monocular_depth_payload,
    image_sha256,
    validate_monocular_depth_transaction,
)


class _FakeLingBot:
    def compute_metric_scale(
        self, paths, poses, camera_height_m, n_frames, return_debug
    ):
        assert len(paths) == 40
        assert poses.shape == (40, 9)
        assert n_frames == 40
        assert return_debug is True
        return 2.0, {
            "h_est": 0.25,
            "n_points": 100,
            "n_frames": 40,
            "n_valid": 30,
            "h_iqr": 0.025,
        }


class MonocularDepthRuntimeTest(unittest.TestCase):
    def test_first40_receipt_uses_exact_prefix(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for index in range(40):
                Image.new("RGB", (2, 2), color=(index, 0, 0)).save(
                    root / f"{index}.jpg"
                )
            receipt = compute_first40_scale_receipt(
                _FakeLingBot(), root, np.zeros((41, 9)), 0.5
            )
        self.assertTrue(receipt["scale_valid"])
        self.assertEqual(receipt["scale_hat"], 2.0)
        self.assertEqual(receipt["scale_prefix_last_frame"], 39)
        self.assertEqual(receipt["active_from_frame_index"], 40)
        self.assertFalse(receipt["whole_episode_ground_cache_consumed"])

    def test_bootstrap_is_exact_zero_even_with_relative_depth(self):
        current = b"jpeg-current"
        payload = build_monocular_depth_payload(
            relative_depth=np.ones((3, 4), np.float32),
            depth_shape=(3, 4),
            image_sha256_value=image_sha256(current),
            frame_index=ACTIVE_FROM_FRAME_INDEX - 1,
            scale_receipt=None,
        )
        depth, metadata = decode_monocular_depth_payload(
            payload, expected_image_sha256=image_sha256(current)
        )
        self.assertTrue(np.array_equal(depth, np.zeros((3, 4))))
        self.assertEqual(metadata["scale_state"], "bootstrap_zero_depth")
        self.assertFalse(metadata["metric_depth_sensor_consumed"])

    def test_active_raw_depth_is_scaled_and_hash_bound(self):
        current = b"jpeg-current"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for index in range(40):
                Image.new("RGB", (2, 2), color=(index, 0, 0)).save(
                    root / f"{index}.jpg"
                )
            receipt = compute_first40_scale_receipt(
                _FakeLingBot(), root, np.zeros((40, 9)), 0.5
            )
        payload = build_monocular_depth_payload(
            relative_depth=np.full((3, 4), 0.75, np.float32),
            depth_shape=(3, 4),
            image_sha256_value=image_sha256(current),
            frame_index=ACTIVE_FROM_FRAME_INDEX,
            scale_receipt=receipt,
        )
        depth, metadata = decode_monocular_depth_payload(
            payload, expected_image_sha256=image_sha256(current)
        )
        self.assertTrue(np.allclose(depth, 1.5, atol=1e-4))
        self.assertEqual(metadata["scale_state"], "raw_lingbot_metric_depth")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            decode_monocular_depth_payload(
                payload, expected_image_sha256="0" * 64
            )

        corrupt = copy.deepcopy(payload)
        corrupt["scale_receipt"]["scale_hat"] = 3.0
        with self.assertRaisesRegex(ValueError, "receipt checksum"):
            decode_monocular_depth_payload(
                corrupt, expected_image_sha256=image_sha256(current)
            )

    def test_transaction_binds_exact_image_frame_and_depth(self):
        current = b"transaction-current-jpeg"
        digest = image_sha256(current)
        payload = build_monocular_depth_payload(
            relative_depth=None,
            depth_shape=(3, 4),
            image_sha256_value=digest,
            frame_index=ACTIVE_FROM_FRAME_INDEX - 1,
            scale_receipt=None,
        )
        bound = bind_monocular_depth_transaction(payload)
        token = bound["monocular_depth_transaction_token"]
        validate_monocular_depth_transaction(
            bound,
            expected_token=token,
            expected_image_sha256=digest,
            expected_frame_index=ACTIVE_FROM_FRAME_INDEX - 1,
        )

        wrong_frame = copy.deepcopy(bound)
        wrong_frame["frame_index"] += 1
        with self.assertRaisesRegex(ValueError, "payload changed"):
            validate_monocular_depth_transaction(
                wrong_frame,
                expected_token=token,
                expected_image_sha256=digest,
            )
        with self.assertRaisesRegex(ValueError, "image mismatch"):
            validate_monocular_depth_transaction(
                bound,
                expected_token=token,
                expected_image_sha256="0" * 64,
            )


if __name__ == "__main__":
    unittest.main()
