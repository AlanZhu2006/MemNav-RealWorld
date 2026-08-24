#!/usr/bin/env python3
"""HTTP client matching the NavDP and X-NavDP benchmark wire format."""

from __future__ import annotations

import base64
import hashlib
import io
import json
import math
from typing import Any, Optional

import cv2
import numpy as np
import requests

from terminal_motion_override import (
    EXPECTED_HANDOFF_SCHEMA as EXPECTED_TERMINAL_HANDOFF_SCHEMA,
)

EXPECTED_CEC_PROTOCOL_VERSION = 3
EVALUATION_DEPTH_PNG_SCALE_M = 1.0e-4


class NavDPClient:
    def __init__(self, server_url: str, connect_timeout_s: float, request_timeout_s: float):
        self.server_url = server_url.rstrip("/")
        self.timeout = (float(connect_timeout_s), float(request_timeout_s))
        self.session = requests.Session()
        self.last_plan_receipt: dict[str, Any] = {}
        self.last_phase_receipt: dict[str, Any] = {}
        self.last_goal_jpeg: bytes | None = None
        self.last_goal_evaluation_depth_png: bytes | None = None
        self.last_goal_evaluation_depth_scale_m: float | None = None

    @staticmethod
    def _encode_rgb(rgb: np.ndarray) -> bytes:
        image = np.asarray(rgb, dtype=np.uint8)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"RGB image must have shape (H, W, 3), got {image.shape}")
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            raise RuntimeError("failed to encode RGB image")
        return encoded.tobytes()

    @staticmethod
    def _encode_depth(depth_m: np.ndarray) -> bytes:
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim == 3:
            depth = depth[..., 0]
        if depth.ndim != 2:
            raise ValueError(f"depth image must have shape (H, W), got {depth.shape}")
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        encoded_depth = np.clip(
            depth / EVALUATION_DEPTH_PNG_SCALE_M, 0.0, 65535.0
        ).astype(np.uint16)
        ok, encoded = cv2.imencode(".png", encoded_depth)
        if not ok:
            raise RuntimeError("failed to encode depth image")
        return encoded.tobytes()

    def _post_phase_endpoint(self, route: str, files: Optional[dict] = None) -> dict:
        """POST a protocol-v3 phase endpoint, surfacing hub contract errors."""
        response = self.session.post(
            f"{self.server_url}{route}",
            files=files,
            timeout=self.timeout,
        )
        if response.status_code >= 400:
            try:
                detail = str(response.json().get("error", ""))
            except Exception:
                detail = response.text[:200]
            raise RuntimeError(
                f"{route} rejected by hub ({response.status_code}): {detail}"
            )
        return response.json()

    def memory_step(self, rgb: np.ndarray) -> dict:
        """Protocol v3: record-only causal RGB append (memory_recording phase)."""
        return self._post_phase_endpoint(
            "/memory_step",
            files={"image": ("image.jpg", self._encode_rgb(rgb), "image/jpeg")},
        )

    def novel_imagegoal_step(
        self,
        goal_rgb: np.ndarray,
        rgb: np.ndarray,
        depth_m: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Record one causal frame while native NavDP executes a Novel goal."""
        files = {
            "image": ("image.jpg", self._encode_rgb(rgb), "image/jpeg"),
            "goal": ("goal.jpg", self._encode_rgb(goal_rgb), "image/jpeg"),
            # Wire compatibility only; the hub never forwards metric depth.
            "depth": ("depth.png", self._encode_depth(depth_m), "image/png"),
        }
        response = self.session.post(
            f"{self.server_url}/novel_imagegoal_step",
            files=files,
            timeout=self.timeout,
        )
        if response.status_code >= 400:
            try:
                detail = str(response.json().get("error", ""))
            except Exception:
                detail = response.text[:200]
            raise RuntimeError(
                "/novel_imagegoal_step rejected by hub "
                f"({response.status_code}): {detail}"
            )
        result = response.json()
        self.last_plan_receipt = {
            key: value for key, value in result.items()
            if key not in {"trajectory", "all_trajectory", "all_values"}
        }
        return (
            np.asarray(result["trajectory"], dtype=np.float32),
            np.asarray(result["all_trajectory"], dtype=np.float32),
            np.asarray(result["all_values"], dtype=np.float32),
        )

    def goal_candidate(
        self,
        rgb: np.ndarray,
        *,
        validate_support: bool = False,
        evaluation_depth_m: np.ndarray | None = None,
    ) -> dict:
        """Protocol v3: register a goal-candidate photo excluded from memory."""
        files = {"image": ("image.jpg", self._encode_rgb(rgb), "image/jpeg")}
        if evaluation_depth_m is not None:
            files["evaluation_depth"] = (
                "evaluation_depth.png",
                self._encode_depth(evaluation_depth_m),
                "image/png",
            )
        data = {"validate_support": "1" if validate_support else "0"}
        if evaluation_depth_m is not None:
            data["evaluation_depth_scale_m"] = str(
                EVALUATION_DEPTH_PNG_SCALE_M
            )
        response = self.session.post(
            f"{self.server_url}/goal_candidate",
            files=files,
            data=data,
            timeout=self.timeout,
        )
        if response.status_code >= 400:
            try:
                detail = str(response.json().get("error", ""))
            except Exception:
                detail = response.text[:200]
            raise RuntimeError(
                f"/goal_candidate rejected by hub ({response.status_code}): {detail}"
            )
        return response.json()

    def begin_revisit(self, query_start_rgb: np.ndarray | None = None) -> dict:
        """Protocol v3: switch to revisit_query; hub warms NavDP and verifies."""
        files = None
        if query_start_rgb is not None:
            files = {"query_start": (
                "query_start.jpg", self._encode_rgb(query_start_rgb), "image/jpeg"
            )}
        receipt = self._post_phase_endpoint("/begin_revisit", files=files)
        self.last_phase_receipt = dict(receipt)
        return receipt

    def prepare_revisit(
        self, query_start_rgb: np.ndarray | None = None
    ) -> tuple[dict, np.ndarray]:
        """Atomically score/select a candidate, switch phase and install it.

        The hub owns the exact candidate JPEG.  The decoded RGB is returned for
        local display; subsequent control requests acknowledge the selected
        SHA-256 while the hub continues to use its committed bytes.
        """
        files = None
        if query_start_rgb is not None:
            files = {"query_start": (
                "query_start.jpg", self._encode_rgb(query_start_rgb), "image/jpeg"
            )}
        receipt = self._post_phase_endpoint("/prepare_revisit", files=files)
        encoded = receipt.get("goal_image_jpeg_base64")
        selected = receipt.get("selected_goal")
        if not isinstance(encoded, str) or not isinstance(selected, dict):
            raise RuntimeError("prepare_revisit omitted the selected goal payload")
        try:
            jpeg = base64.b64decode(encoded, validate=True)
        except Exception as error:
            raise RuntimeError(f"invalid selected goal encoding: {error}") from error
        expected = str(selected.get("sha256", ""))
        try:
            int(selected["candidate_id"])
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError("selected goal omitted a valid candidate id") from error
        actual = hashlib.sha256(jpeg).hexdigest()
        if not expected or actual != expected:
            raise RuntimeError("selected goal SHA-256 mismatch")
        self.last_goal_jpeg = jpeg
        depth_encoded = receipt.get("goal_evaluation_depth_png_base64")
        self.last_goal_evaluation_depth_png = (
            None
            if depth_encoded is None
            else base64.b64decode(str(depth_encoded), validate=True)
        )
        depth_scale = receipt.get("goal_evaluation_depth_scale_m")
        self.last_goal_evaluation_depth_scale_m = (
            None if depth_scale is None else float(depth_scale)
        )
        if self.last_goal_evaluation_depth_png is not None and (
            self.last_goal_evaluation_depth_scale_m is None
            or not math.isfinite(self.last_goal_evaluation_depth_scale_m)
            or self.last_goal_evaluation_depth_scale_m <= 0.0
        ):
            raise RuntimeError(
                "selected evaluator depth has no valid metre scale"
            )
        decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            raise RuntimeError("selected goal JPEG is not decodable")
        rgb = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
        status_receipt = {
            key: value for key, value in receipt.items()
            if key not in {
                "goal_image_jpeg_base64",
                "goal_evaluation_depth_png_base64",
            }
        }
        self.last_phase_receipt = status_receipt
        return status_receipt, rgb

    def prepare_revisit_goal(
        self,
        goal_rgb: np.ndarray,
        query_start_rgb: np.ndarray | None = None,
    ) -> tuple[dict, np.ndarray]:
        """Install one pre-episode frozen Revisit goal and switch phase."""
        files = {
            "goal": (
                "goal.jpg",
                self._encode_rgb(goal_rgb),
                "image/jpeg",
            )
        }
        if query_start_rgb is not None:
            files["query_start"] = (
                "query_start.jpg",
                self._encode_rgb(query_start_rgb),
                "image/jpeg",
            )
        response = self.session.post(
            f"{self.server_url}/prepare_revisit_goal",
            files=files,
            timeout=self.timeout,
        )
        if response.status_code >= 400:
            try:
                detail = str(response.json().get("error", ""))
            except Exception:
                detail = response.text[:200]
            raise RuntimeError(
                "/prepare_revisit_goal rejected by hub "
                f"({response.status_code}): {detail}"
            )
        receipt = response.json()
        encoded = receipt.get("goal_image_jpeg_base64")
        selected = receipt.get("selected_goal")
        if not isinstance(encoded, str) or not isinstance(selected, dict):
            raise RuntimeError(
                "prepare_revisit_goal omitted the committed goal payload"
            )
        try:
            jpeg = base64.b64decode(encoded, validate=True)
        except Exception as error:
            raise RuntimeError(f"invalid committed goal encoding: {error}") from error
        expected = str(selected.get("sha256", ""))
        actual = hashlib.sha256(jpeg).hexdigest()
        if not expected or actual != expected:
            raise RuntimeError(
                "committed external goal SHA-256 does not match its receipt"
            )
        self.last_goal_jpeg = jpeg
        decoded = cv2.imdecode(
            np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if decoded is None:
            raise RuntimeError("committed external goal JPEG is not decodable")
        rgb = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
        status_receipt = {
            key: value for key, value in receipt.items()
            if key != "goal_image_jpeg_base64"
        }
        self.last_goal_evaluation_depth_png = None
        self.last_goal_evaluation_depth_scale_m = None
        self.last_phase_receipt = status_receipt
        return status_receipt, rgb

    def dataset_status(self) -> dict:
        response = self.session.get(
            f"{self.server_url}/dataset/status", timeout=self.timeout
        )
        response.raise_for_status()
        return response.json()

    def start_dataset(
        self, dataset_id: str, *, metadata: dict[str, Any] | None = None
    ) -> dict:
        response = self.session.post(
            f"{self.server_url}/dataset/start",
            json={"dataset_id": dataset_id, "metadata": dict(metadata or {})},
            timeout=self.timeout,
        )
        response.raise_for_status()
        return response.json()

    def seal_dataset(self) -> dict:
        response = self.session.post(
            f"{self.server_url}/dataset/seal", json={}, timeout=self.timeout
        )
        response.raise_for_status()
        return response.json()

    def load_dataset(self, dataset_id: str) -> dict:
        timeout = (self.timeout[0], max(3600.0, self.timeout[1]))
        response = self.session.post(
            f"{self.server_url}/dataset/load",
            json={"dataset_id": dataset_id},
            timeout=timeout,
        )
        response.raise_for_status()
        return response.json()

    def health(self) -> dict:
        response = self.session.get(f"{self.server_url}/healthz", timeout=self.timeout)
        response.raise_for_status()
        return response.json()

    def reset(self, intrinsic: np.ndarray, stop_threshold: float = -2.0) -> str:
        payload = {
            "intrinsic": np.asarray(intrinsic, dtype=float).tolist(),
            "stop_threshold": float(stop_threshold),
            "batch_size": 1,
            "sample_indices": [0],
            "scene_name": "go2_real",
        }
        response = self.session.post(
            f"{self.server_url}/navigator_reset", json=payload, timeout=self.timeout
        )
        response.raise_for_status()
        receipt = response.json()
        algorithm = str(receipt.get("algo", "unknown"))
        if algorithm == "cec_hybrid_navdp":
            if (
                receipt.get("protocol_version")
                != EXPECTED_CEC_PROTOCOL_VERSION
                or receipt.get("terminal_handoff_schema")
                != EXPECTED_TERMINAL_HANDOFF_SCHEMA
            ):
                raise RuntimeError(
                    "CEC hub/Jetson runtime contract mismatch: "
                    f"protocol={receipt.get('protocol_version')!r}, "
                    "terminal_handoff_schema="
                    f"{receipt.get('terminal_handoff_schema')!r}"
                )
        return algorithm

    def pointgoal_step(
        self,
        goal_xy: np.ndarray,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        robot_position: Optional[np.ndarray] = None,
        robot_quaternion_xyzw: Optional[np.ndarray] = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        goal = np.asarray(goal_xy, dtype=np.float32).reshape(2)
        files = {
            "image": ("image.jpg", self._encode_rgb(rgb), "image/jpeg"),
            "depth": ("depth.png", self._encode_depth(depth_m), "image/png"),
        }
        data = {
            "goal_data": json.dumps({"goal_x": [float(goal[0])], "goal_y": [float(goal[1])]})
        }
        state = {}
        if robot_position is not None:
            state["robot_pos"] = [np.asarray(robot_position, dtype=float).reshape(3).tolist()]
        if robot_quaternion_xyzw is not None:
            state["robot_quat"] = [
                np.asarray(robot_quaternion_xyzw, dtype=float).reshape(4).tolist()
            ]
        if state:
            data["state_data"] = json.dumps(state)

        response = self.session.post(
            f"{self.server_url}/pointgoal_step",
            files=files,
            data=data,
            timeout=self.timeout,
        )
        response.raise_for_status()
        result = response.json()
        return (
            np.asarray(result["trajectory"], dtype=np.float32),
            np.asarray(result["all_trajectory"], dtype=np.float32),
            np.asarray(result["all_values"], dtype=np.float32),
        )

    def nogoal_step(
        self, rgb: np.ndarray, depth_m: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        files = {
            "image": ("image.jpg", self._encode_rgb(rgb), "image/jpeg"),
            "depth": ("depth.png", self._encode_depth(depth_m), "image/png"),
        }
        response = self.session.post(
            f"{self.server_url}/nogoal_step", files=files, timeout=self.timeout
        )
        response.raise_for_status()
        result = response.json()
        return (
            np.asarray(result["trajectory"], dtype=np.float32),
            np.asarray(result["all_trajectory"], dtype=np.float32),
            np.asarray(result["all_values"], dtype=np.float32),
        )

    def imagegoal_step(
        self,
        goal_rgb: np.ndarray,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        installed_goal_sha256: Optional[str] = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        files = {
            "image": ("image.jpg", self._encode_rgb(rgb), "image/jpeg"),
            "goal": ("goal.jpg", self._encode_rgb(goal_rgb), "image/jpeg"),
            "depth": ("depth.png", self._encode_depth(depth_m), "image/png"),
        }
        request_args: dict[str, Any] = {
            "files": files,
            "timeout": self.timeout,
        }
        if installed_goal_sha256:
            request_args["data"] = {
                "installed_goal_sha256": str(installed_goal_sha256)
            }
        response = self.session.post(
            f"{self.server_url}/imagegoal_step", **request_args
        )
        response.raise_for_status()
        result = response.json()
        self.last_plan_receipt = {
            key: value for key, value in result.items()
            if key not in {"trajectory", "all_trajectory", "all_values"}
        }
        return (
            np.asarray(result["trajectory"], dtype=np.float32),
            np.asarray(result["all_trajectory"], dtype=np.float32),
            np.asarray(result["all_values"], dtype=np.float32),
        )
