#!/usr/bin/env python3
"""Fail-closed runtime contract for the MDTEC raw LingBot-depth interface.

The module contains no navigation policy.  It turns one frozen LingBot depth
prediction plus one immutable first-40 RGB scale receipt into the metric-depth
array consumed by the unchanged NavDP RGB-D encoder.  Before the scale receipt
becomes causally available, or when scale recovery fails, the only permitted
output is explicit zero depth.

The wire payload binds the depth PNG to the exact current JPEG by SHA-256.  A
NavDP server therefore cannot accidentally consume a stale sidecar frame, and
it can audit that simulator metric depth was not used.
"""

from __future__ import annotations

import base64
import hashlib
import io
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping

import numpy as np
from PIL import Image


SCALE_SCHEMA = "mdtec_first40_scale_receipt_v1_20260819"
DEPTH_SCHEMA = "mdtec_monocular_depth_observation_v1_20260819"
DEPTH_TRANSACTION_SCHEMA = "mdtec_monocular_depth_transaction_v1_20260821"
SCALE_CONTRACT = "causal_first_prefix_rgb_only_v1"
DEPTH_CONTRACT = "raw_lingbot_depth_first40_v1"
PREFIX_FRAMES = 40
ACTIVE_FROM_FRAME_INDEX = 40
# Pinned to the LingBot implementation used by formal Gate C.  Keeping the
# scalar here makes the runtime receipt testable without importing the heavy
# InternNav package; the actual scale value is still produced by LingBot's
# frozen ``compute_metric_scale`` implementation.
GROUND_BIAS_CORRECTION = 1.15
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def canonical_sha256(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        dict(value), sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def image_sha256(image_bytes: bytes) -> str:
    if not image_bytes:
        raise ValueError("current JPEG is empty")
    return hashlib.sha256(image_bytes).hexdigest()


def monocular_depth_transaction_token(payload: Mapping[str, Any]) -> str:
    """Bind one materialized depth payload to its exact RGB stream position."""

    image_digest = str(payload.get("image_sha256", ""))
    depth_digest = str(payload.get("depth_png_sha256", ""))
    scale_digest = payload.get("scale_receipt_sha256")
    if not _SHA256.fullmatch(image_digest):
        raise ValueError("transaction image SHA-256 is invalid")
    if not _SHA256.fullmatch(depth_digest):
        raise ValueError("transaction depth SHA-256 is invalid")
    if scale_digest is not None and not _SHA256.fullmatch(str(scale_digest)):
        raise ValueError("transaction scale SHA-256 is invalid")
    frame_index = int(payload.get("frame_index", -1))
    if frame_index < 0:
        raise ValueError("transaction frame index is invalid")
    return canonical_sha256({
        "schema": DEPTH_TRANSACTION_SCHEMA,
        "depth_schema": payload.get("schema"),
        "depth_contract": payload.get("depth_contract"),
        "frame_index": frame_index,
        "image_sha256": image_digest,
        "depth_png_sha256": depth_digest,
        "scale_receipt_sha256": scale_digest,
    })


def bind_monocular_depth_transaction(
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    """Return a copy of ``payload`` carrying its immutable transaction token."""

    bound = dict(payload)
    bound["monocular_depth_transaction_schema"] = DEPTH_TRANSACTION_SCHEMA
    bound["monocular_depth_transaction_token"] = (
        monocular_depth_transaction_token(payload)
    )
    return bound


def validate_monocular_depth_transaction(
    payload: Mapping[str, Any],
    *,
    expected_token: str,
    expected_image_sha256: str,
    expected_frame_index: int | None = None,
) -> None:
    """Fail closed unless a cached payload matches the caller's exact append."""

    if not _SHA256.fullmatch(str(expected_token)):
        raise ValueError("expected transaction token is invalid")
    if payload.get("monocular_depth_transaction_schema") != (
        DEPTH_TRANSACTION_SCHEMA
    ):
        raise ValueError("unexpected monocular depth transaction schema")
    if payload.get("monocular_depth_transaction_token") != expected_token:
        raise ValueError("monocular depth transaction token mismatch")
    if monocular_depth_transaction_token(payload) != expected_token:
        raise ValueError("monocular depth transaction payload changed")
    if payload.get("image_sha256") != expected_image_sha256:
        raise ValueError("monocular depth transaction image mismatch")
    if expected_frame_index is not None and int(payload.get(
        "frame_index", -1
    )) != int(expected_frame_index):
        raise ValueError("monocular depth transaction frame mismatch")


def _base_scale_receipt(camera_height_m: float) -> dict[str, Any]:
    camera_height_m = float(camera_height_m)
    if not math.isfinite(camera_height_m) or camera_height_m <= 0.0:
        raise ValueError("camera height must be finite and positive")
    return {
        "schema": SCALE_SCHEMA,
        "scale_evidence_contract": SCALE_CONTRACT,
        "scale_prefix_frames": PREFIX_FRAMES,
        "scale_prefix_first_frame": 0,
        "scale_prefix_last_frame": PREFIX_FRAMES - 1,
        "frozen_after_observation_count": PREFIX_FRAMES,
        "active_from_frame_index": ACTIVE_FROM_FRAME_INDEX,
        "whole_episode_ground_cache_consumed": False,
        "camera_height_m": camera_height_m,
    }


def failed_first40_scale_receipt(
    camera_height_m: float, error: str
) -> dict[str, Any]:
    receipt = _base_scale_receipt(camera_height_m)
    receipt.update(
        {
            "ground_h_est_raw": None,
            "scale_valid": False,
            "scale_hat": None,
            "valid_frame_ratio": 0.0,
            "relative_floor_iqr": 0.0,
            "scale_clamped": False,
            "freeze_error": str(error),
        }
    )
    validate_first40_scale_receipt(receipt)
    return receipt


def compute_first40_scale_receipt(
    lingbot,
    rgb_dir: str | Path,
    cam_pose_enc: np.ndarray,
    camera_height_m: float,
) -> dict[str, Any]:
    """Replay exactly RGB frames 0..39 and freeze one metric-scale receipt."""

    rgb_dir = Path(rgb_dir)
    poses = np.asarray(cam_pose_enc)
    if poses.ndim != 2 or poses.shape[1] != 9:
        raise ValueError(f"cam_pose_enc must have shape [T,9], got {poses.shape}")
    if len(poses) < PREFIX_FRAMES:
        raise ValueError(
            f"only {len(poses)} poses for the {PREFIX_FRAMES}-frame prefix"
        )
    paths = [rgb_dir / f"{index}.jpg" for index in range(PREFIX_FRAMES)]
    missing = next((path for path in paths if not path.is_file()), None)
    if missing is not None:
        raise FileNotFoundError(f"first-40 RGB prefix is incomplete: {missing}")

    scale, debug = lingbot.compute_metric_scale(
        [str(path) for path in paths],
        poses[:PREFIX_FRAMES].copy(),
        camera_height_m=float(camera_height_m),
        n_frames=PREFIX_FRAMES,
        return_debug=True,
    )
    debug = dict(debug)
    h_est = debug.get("h_est")
    valid = (
        scale is not None
        and math.isfinite(float(scale))
        and float(scale) > 0.0
        and h_est is not None
        and math.isfinite(float(h_est))
        and float(h_est) > 0.0
    )
    receipt = _base_scale_receipt(camera_height_m)
    if valid:
        raw_scale = (
            GROUND_BIAS_CORRECTION
            * float(camera_height_m)
            / float(h_est)
        )
        n_frames = max(float(debug.get("n_frames", 0)), 1.0)
        n_valid = float(debug.get("n_valid", 0))
        h_iqr = debug.get("h_iqr")
        relative_iqr = (
            0.0 if h_iqr is None else float(h_iqr) / float(h_est)
        )
        receipt.update(
            {
                "ground_h_est_raw": float(h_est),
                "scale_valid": True,
                "scale_hat": float(scale),
                "valid_frame_ratio": n_valid / n_frames,
                "relative_floor_iqr": relative_iqr,
                "scale_clamped": not math.isclose(
                    float(scale), raw_scale, rel_tol=1e-6, abs_tol=1e-9
                ),
                "freeze_error": None,
            }
        )
    else:
        receipt.update(
            {
                "ground_h_est_raw": (
                    None if h_est is None else float(h_est)
                ),
                "scale_valid": False,
                "scale_hat": None,
                "valid_frame_ratio": 0.0,
                "relative_floor_iqr": 0.0,
                "scale_clamped": False,
                "freeze_error": None,
            }
        )
    validate_first40_scale_receipt(receipt)
    return receipt


def validate_first40_scale_receipt(receipt: Mapping[str, Any]) -> None:
    if receipt.get("schema") != SCALE_SCHEMA:
        raise ValueError("unexpected first-40 scale schema")
    if receipt.get("scale_evidence_contract") != SCALE_CONTRACT:
        raise ValueError("unexpected scale evidence contract")
    exact = {
        "scale_prefix_frames": PREFIX_FRAMES,
        "scale_prefix_first_frame": 0,
        "scale_prefix_last_frame": PREFIX_FRAMES - 1,
        "frozen_after_observation_count": PREFIX_FRAMES,
        "active_from_frame_index": ACTIVE_FROM_FRAME_INDEX,
    }
    for key, expected in exact.items():
        if int(receipt.get(key, -1)) != expected:
            raise ValueError(f"{key} drifted from {expected}")
    if receipt.get("whole_episode_ground_cache_consumed") is not False:
        raise ValueError("whole-episode scale evidence is forbidden")
    height = float(receipt.get("camera_height_m", float("nan")))
    if not math.isfinite(height) or height <= 0.0:
        raise ValueError("invalid camera height in scale receipt")
    valid = receipt.get("scale_valid") is True
    scale = receipt.get("scale_hat")
    if valid:
        if scale is None or not math.isfinite(float(scale)) or float(scale) <= 0.0:
            raise ValueError("valid scale receipt lacks a positive scale")
    elif scale is not None:
        raise ValueError("invalid scale receipt retained scale_hat")
    for key in ("valid_frame_ratio", "relative_floor_iqr"):
        value = float(receipt.get(key, float("nan")))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"invalid {key}")


def encode_depth_png(metric_depth_m: np.ndarray) -> bytes:
    depth = np.asarray(metric_depth_m, dtype=np.float32)
    if depth.ndim != 2 or not np.isfinite(depth).all():
        raise ValueError("metric depth must be a finite [H,W] array")
    if np.any(depth < 0.0):
        raise ValueError("metric depth must be non-negative")
    encoded = np.clip(depth * 10000.0, 0.0, 65535.0).astype(np.uint16)
    buffer = io.BytesIO()
    Image.fromarray(encoded).save(buffer, format="PNG")
    return buffer.getvalue()


def decode_depth_png(depth_png: bytes) -> np.ndarray:
    if not depth_png:
        raise ValueError("depth PNG is empty")
    depth = np.asarray(
        Image.open(io.BytesIO(depth_png)).convert("I"), dtype=np.float32
    ) / 10000.0
    if depth.ndim != 2 or not np.isfinite(depth).all() or np.any(depth < 0.0):
        raise ValueError("decoded depth PNG is invalid")
    return depth


def build_monocular_depth_payload(
    *,
    relative_depth: np.ndarray | None,
    depth_shape: tuple[int, int],
    image_sha256_value: str,
    frame_index: int,
    scale_receipt: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Create one current-frame depth payload under the frozen bootstrap rule."""

    frame_index = int(frame_index)
    if frame_index < 0:
        raise ValueError("frame index must be non-negative")
    if not _SHA256.fullmatch(str(image_sha256_value)):
        raise ValueError("current image SHA-256 is invalid")
    height, width = (int(depth_shape[0]), int(depth_shape[1]))
    if height < 1 or width < 1:
        raise ValueError("image shape must be positive")

    scale_active = frame_index >= ACTIVE_FROM_FRAME_INDEX
    receipt_hash = None
    if scale_receipt is not None:
        validate_first40_scale_receipt(scale_receipt)
        receipt_hash = canonical_sha256(scale_receipt)

    if not scale_active:
        depth = np.zeros((height, width), dtype=np.float32)
        state = "bootstrap_zero_depth"
        scale_valid = False
    elif scale_receipt is None:
        raise RuntimeError("first-40 scale receipt is missing after activation")
    elif scale_receipt["scale_valid"] is not True:
        depth = np.zeros((height, width), dtype=np.float32)
        state = "frozen_scale_invalid_zero_depth"
        scale_valid = False
    else:
        relative = np.asarray(relative_depth, dtype=np.float32)
        if relative.shape != (height, width):
            raise ValueError(
                f"relative depth shape {relative.shape} != {(height, width)}"
            )
        if not np.isfinite(relative).all() or np.any(relative < 0.0):
            raise ValueError("relative LingBot depth is invalid")
        depth = relative * float(scale_receipt["scale_hat"])
        state = "raw_lingbot_metric_depth"
        scale_valid = True

    png = encode_depth_png(depth)
    nonzero = depth[depth > 0.0]
    metadata = {
        "schema": DEPTH_SCHEMA,
        "depth_contract": DEPTH_CONTRACT,
        "frame_index": frame_index,
        "image_sha256": str(image_sha256_value),
        "depth_shape": [height, width],
        "scale_state": state,
        "scale_active": bool(scale_active),
        "scale_valid": bool(scale_valid),
        "scale_receipt_sha256": receipt_hash,
        "scale_receipt": None if scale_receipt is None else dict(scale_receipt),
        "metric_depth_sensor_consumed": False,
        "relative_depth_model": "frozen_lingbot_map",
        "depth_nonzero_fraction": float(np.count_nonzero(depth) / depth.size),
        "depth_nonzero_median_m": (
            None if nonzero.size == 0 else float(np.median(nonzero))
        ),
        "depth_png_sha256": hashlib.sha256(png).hexdigest(),
    }
    return {
        **metadata,
        "depth_png_base64": base64.b64encode(png).decode("ascii"),
    }


def decode_monocular_depth_payload(
    payload: Mapping[str, Any], *, expected_image_sha256: str
) -> tuple[np.ndarray, dict[str, Any]]:
    """Validate a sidecar response and return NavDP metric-depth metres."""

    if payload.get("schema") != DEPTH_SCHEMA:
        raise ValueError("unexpected monocular depth schema")
    if payload.get("depth_contract") != DEPTH_CONTRACT:
        raise ValueError("unexpected monocular depth contract")
    if payload.get("metric_depth_sensor_consumed") is not False:
        raise ValueError("sidecar payload does not prove sensor-free depth")
    if payload.get("image_sha256") != expected_image_sha256:
        raise ValueError("sidecar current-image hash mismatch")
    raw = payload.get("depth_png_base64")
    if not isinstance(raw, str):
        raise ValueError("sidecar payload lacks depth PNG")
    try:
        png = base64.b64decode(raw, validate=True)
    except Exception as error:
        raise ValueError("invalid base64 depth PNG") from error
    if hashlib.sha256(png).hexdigest() != payload.get("depth_png_sha256"):
        raise ValueError("sidecar depth PNG checksum mismatch")
    depth = decode_depth_png(png)
    if list(depth.shape) != list(payload.get("depth_shape", [])):
        raise ValueError("sidecar depth shape mismatch")
    metadata = dict(payload)
    metadata.pop("depth_png_base64", None)
    receipt = metadata.get("scale_receipt")
    if receipt is not None:
        validate_first40_scale_receipt(receipt)
        if canonical_sha256(receipt) != metadata.get("scale_receipt_sha256"):
            raise ValueError("sidecar scale receipt checksum mismatch")
    return depth, metadata
