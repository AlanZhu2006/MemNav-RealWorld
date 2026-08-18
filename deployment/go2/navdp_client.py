#!/usr/bin/env python3
"""HTTP client matching the NavDP and X-NavDP benchmark wire format."""

from __future__ import annotations

import io
import json
from typing import Optional

import cv2
import numpy as np
import requests


class NavDPClient:
    def __init__(self, server_url: str, connect_timeout_s: float, request_timeout_s: float):
        self.server_url = server_url.rstrip("/")
        self.timeout = (float(connect_timeout_s), float(request_timeout_s))
        self.session = requests.Session()

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
        encoded_depth = np.clip(depth * 10000.0, 0.0, 65535.0).astype(np.uint16)
        ok, encoded = cv2.imencode(".png", encoded_depth)
        if not ok:
            raise RuntimeError("failed to encode depth image")
        return encoded.tobytes()

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
        return str(response.json().get("algo", "unknown"))

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
        self, goal_rgb: np.ndarray, rgb: np.ndarray, depth_m: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        files = {
            "image": ("image.jpg", self._encode_rgb(rgb), "image/jpeg"),
            "goal": ("goal.jpg", self._encode_rgb(goal_rgb), "image/jpeg"),
            "depth": ("depth.png", self._encode_depth(depth_m), "image/png"),
        }
        response = self.session.post(
            f"{self.server_url}/imagegoal_step",
            files=files,
            timeout=self.timeout,
        )
        response.raise_for_status()
        result = response.json()
        return (
            np.asarray(result["trajectory"], dtype=np.float32),
            np.asarray(result["all_trajectory"], dtype=np.float32),
            np.asarray(result["all_values"], dtype=np.float32),
        )
