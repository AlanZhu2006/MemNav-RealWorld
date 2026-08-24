#!/usr/bin/env python3
"""Capture Go2 goal ground truth and score NavDP ImageGoal episodes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Optional

import numpy as np

from image_goal_io import (
    depth_array_to_meters,
    load_depth_image,
    load_rgb_image,
)
from visual_goal_verifier import VisualGoalVerifier, VisualMatchResult


GO2_DIR = Path(__file__).resolve().parent
DEFAULT_IMAGE_GOAL = GO2_DIR / "goals" / "image_goal.png"
DEFAULT_IMAGE_GOAL_DEPTH = GO2_DIR / "goals" / "image_goal_depth.png"
DEFAULT_TARGET_POSE = GO2_DIR / "goals" / "image_goal_pose.json"


@dataclass(frozen=True)
class StateSample:
    monotonic_s: float
    position_xyz: np.ndarray
    velocity_xyz: np.ndarray
    yaw_rad: float


class EpisodeTracker:
    def __init__(
        self,
        target_xy: np.ndarray,
        target_yaw_rad: float,
        success_distance_m: float = 0.85,
        success_speed_mps: float = 0.30,
        success_hold_s: float = 0.50,
        max_position_jump_m: float = 0.50,
    ) -> None:
        self.target_xy = np.asarray(target_xy, dtype=np.float64).reshape(2)
        self.target_yaw_rad = float(target_yaw_rad)
        self.success_distance_m = float(success_distance_m)
        self.success_speed_mps = float(success_speed_mps)
        self.success_hold_s = float(success_hold_s)
        self.max_position_jump_m = float(max_position_jump_m)
        self.started_s: Optional[float] = None
        self.start_xy: Optional[np.ndarray] = None
        self.last_sample: Optional[StateSample] = None
        self.path_length_m = 0.0
        self.min_distance_m = math.inf
        self.within_since_s: Optional[float] = None
        self.success = False
        self.invalid_reason = ""

    @staticmethod
    def angular_error_rad(first: float, second: float) -> float:
        return math.atan2(math.sin(first - second), math.cos(first - second))

    def update(self, sample: StateSample) -> str:
        xy = np.asarray(sample.position_xyz[:2], dtype=np.float64)
        if not np.isfinite(xy).all() or not np.isfinite(sample.velocity_xyz).all():
            self.invalid_reason = "nonfinite_state"
            return "invalid"
        if self.started_s is None:
            self.started_s = sample.monotonic_s
            self.start_xy = xy.copy()
        if self.last_sample is not None:
            step = float(np.linalg.norm(xy - self.last_sample.position_xyz[:2]))
            if step > self.max_position_jump_m:
                self.invalid_reason = "position_jump"
                return "invalid"
            self.path_length_m += step
        self.last_sample = sample

        distance = float(np.linalg.norm(xy - self.target_xy))
        speed = float(np.abs(sample.velocity_xyz).sum())
        self.min_distance_m = min(self.min_distance_m, distance)
        if distance <= self.success_distance_m and speed <= self.success_speed_mps:
            if self.within_since_s is None:
                self.within_since_s = sample.monotonic_s
            if sample.monotonic_s - self.within_since_s >= self.success_hold_s:
                self.success = True
                return "success"
        else:
            self.within_since_s = None
        return "running"

    def metrics(self, now_s: Optional[float] = None) -> dict:
        if self.last_sample is None or self.start_xy is None or self.started_s is None:
            return {
                "success": False,
                "invalid_reason": self.invalid_reason,
                "sample_count_ready": False,
            }
        sample = self.last_sample
        current_xy = np.asarray(sample.position_xyz[:2], dtype=np.float64)
        initial_distance = float(np.linalg.norm(self.start_xy - self.target_xy))
        current_distance = float(np.linalg.norm(current_xy - self.target_xy))
        denominator = max(self.path_length_m, initial_distance, 1e-6)
        elapsed = max(0.0, (sample.monotonic_s if now_s is None else now_s) - self.started_s)
        return {
            "success": self.success,
            "invalid_reason": self.invalid_reason,
            "sample_count_ready": True,
            "elapsed_s": round(elapsed, 3),
            "initial_distance_m": round(initial_distance, 3),
            "current_distance_m": round(current_distance, 3),
            "min_distance_m": round(self.min_distance_m, 3),
            "path_length_m": round(self.path_length_m, 3),
            "spl": round(initial_distance / denominator, 4) if self.success else 0.0,
            "current_speed_mps": round(float(np.abs(sample.velocity_xyz).sum()), 3),
            "current_planar_speed_mps": round(
                float(np.linalg.norm(sample.velocity_xyz[:2])), 3
            ),
            "final_yaw_error_rad": round(
                abs(self.angular_error_rad(sample.yaw_rad, self.target_yaw_rad)), 3
            ),
            "start_xy": np.round(self.start_xy, 4).tolist(),
            "current_xy": np.round(current_xy, 4).tolist(),
            "target_xy": np.round(self.target_xy, 4).tolist(),
        }


class PolicyStopTracker:
    def __init__(
        self,
        hold_s: float = 1.50,
        max_linear_mps: float = 0.03,
        max_angular_rps: float = 0.05,
        max_path_radius_m: float = 0.20,
        status_timeout_s: float = 1.25,
        path_timeout_s: float = 2.50,
    ) -> None:
        self.hold_s = float(hold_s)
        self.max_linear_mps = float(max_linear_mps)
        self.max_angular_rps = float(max_angular_rps)
        self.max_path_radius_m = float(max_path_radius_m)
        self.status_timeout_s = float(status_timeout_s)
        self.path_timeout_s = float(path_timeout_s)
        self.status_monotonic_s = 0.0
        self.path_monotonic_s = 0.0
        self.status_ready = False
        self.status_reason = "waiting_for_navdp_status"
        self.path_radius_m = math.inf
        self.command_linear_mps = math.inf
        self.command_angular_rps = math.inf
        self.zero_since_s: Optional[float] = None
        self.confirmed = False
        self.reason = "waiting_for_navdp_status"

    def _refresh(self, now_s: float) -> None:
        if self.status_monotonic_s <= 0.0:
            ready = False
            reason = "waiting_for_navdp_status"
        elif now_s - self.status_monotonic_s > self.status_timeout_s:
            ready = False
            reason = "navdp_status_stale"
        elif not self.status_ready:
            ready = False
            reason = self.status_reason
        elif self.path_monotonic_s <= 0.0:
            ready = False
            reason = "waiting_for_navdp_path"
        elif now_s - self.path_monotonic_s > self.path_timeout_s:
            ready = False
            reason = "navdp_path_stale"
        elif self.path_radius_m > self.max_path_radius_m:
            ready = False
            reason = "navdp_path_nonzero"
        else:
            ready = True
            reason = "holding_zero_policy_output"

        if not ready:
            self.zero_since_s = None
            self.confirmed = False
            self.reason = reason
            return
        if self.zero_since_s is None:
            self.zero_since_s = now_s
        self.confirmed = now_s - self.zero_since_s >= self.hold_s
        self.reason = "zero_policy_output_confirmed" if self.confirmed else reason

    def update_status(self, payload: dict, now_s: float) -> None:
        self.status_monotonic_s = float(now_s)
        try:
            self.command_linear_mps = abs(float(payload.get("cmd_vx", math.inf)))
            self.command_angular_rps = abs(float(payload.get("cmd_wz", math.inf)))
        except (TypeError, ValueError):
            self.command_linear_mps = math.inf
            self.command_angular_rps = math.inf

        checks = (
            (payload.get("backend") == "navdp", "wrong_navdp_backend"),
            (payload.get("mode") == "imagegoal", "wrong_navdp_mode"),
            (bool(payload.get("enabled")), "navdp_disabled"),
            (not bool(payload.get("estop")), "navdp_estop_asserted"),
            (bool(payload.get("server_initialized")), "navdp_server_uninitialized"),
            (not str(payload.get("last_error", "")), "navdp_inference_error"),
            (
                str(payload.get("stop_reason", "")) in {"clear", "depth_slow"},
                "navdp_motion_blocked",
            ),
            (
                self.command_linear_mps <= self.max_linear_mps,
                "navdp_linear_command_nonzero",
            ),
            (
                self.command_angular_rps <= self.max_angular_rps,
                "navdp_angular_command_nonzero",
            ),
        )
        self.status_ready = True
        self.status_reason = "navdp_command_zero"
        for passed, failure_reason in checks:
            if not passed:
                self.status_ready = False
                self.status_reason = failure_reason
                break
        self._refresh(now_s)

    def update_path(self, path_xy: np.ndarray, now_s: float) -> None:
        points = np.asarray(path_xy, dtype=np.float64)
        self.path_monotonic_s = float(now_s)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            self.path_radius_m = math.inf
        elif len(points) == 0:
            self.path_radius_m = math.inf
        else:
            self.path_radius_m = float(np.linalg.norm(points, axis=1).max())
        self._refresh(now_s)

    def snapshot(self, now_s: float) -> dict:
        self._refresh(now_s)
        zero_hold_s = (
            0.0 if self.zero_since_s is None else max(0.0, now_s - self.zero_since_s)
        )
        return {
            "confirmed": self.confirmed,
            "reason": self.reason,
            "zero_hold_s": round(zero_hold_s, 3),
            "required_hold_s": self.hold_s,
            "command_linear_mps": (
                round(self.command_linear_mps, 3)
                if math.isfinite(self.command_linear_mps)
                else None
            ),
            "command_angular_rps": (
                round(self.command_angular_rps, 3)
                if math.isfinite(self.command_angular_rps)
                else None
            ),
            "path_radius_m": (
                round(self.path_radius_m, 3)
                if math.isfinite(self.path_radius_m)
                else None
            ),
            "status_age_s": (
                round(max(0.0, now_s - self.status_monotonic_s), 3)
                if self.status_monotonic_s > 0.0
                else None
            ),
            "path_age_s": (
                round(max(0.0, now_s - self.path_monotonic_s), 3)
                if self.path_monotonic_s > 0.0
                else None
            ),
        }


def import_unitree_sdk(sdk_path: str):
    path = Path(sdk_path).expanduser()
    if path.exists():
        sys.path.insert(0, str(path))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    return ChannelFactoryInitialize, ChannelSubscriber, SportModeState_


def state_from_message(message) -> StateSample:
    return StateSample(
        monotonic_s=time.monotonic(),
        position_xyz=np.asarray(message.position, dtype=np.float64),
        velocity_xyz=np.asarray(message.velocity, dtype=np.float64),
        yaw_rad=float(message.imu_state.rpy[2]),
    )


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: str | Path, payload: dict) -> Path:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    return destination


def capture_target(args) -> int:
    image_path = Path(args.image_goal).expanduser().resolve()
    depth_path = Path(args.image_goal_depth).expanduser().resolve()
    if not image_path.is_file():
        raise FileNotFoundError(f"image goal missing: {image_path}")
    if not depth_path.is_file():
        raise FileNotFoundError(f"aligned image-goal depth missing: {depth_path}")
    visual_reference = VisualGoalVerifier(
        load_rgb_image(image_path),
        load_depth_image(depth_path),
        image_width=args.visual_image_width,
    )
    factory, subscriber_type, message_type = import_unitree_sdk(args.sdk_path)
    factory(0, args.net_if)
    samples: list[StateSample] = []
    condition = threading.Condition()

    def on_state(message) -> None:
        sample = state_from_message(message)
        if not np.isfinite(sample.position_xyz).all():
            return
        with condition:
            if len(samples) < args.samples:
                samples.append(sample)
                condition.notify_all()

    subscriber = subscriber_type(args.state_topic, message_type)
    subscriber.Init(on_state, 10)
    deadline = time.monotonic() + args.timeout_s
    with condition:
        while len(samples) < args.samples:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            condition.wait(timeout=min(remaining, 0.2))
    if not samples:
        raise RuntimeError(f"no Go2 state received from {args.state_topic}")

    positions = np.stack([sample.position_xyz for sample in samples])
    velocities = np.stack([sample.velocity_xyz for sample in samples])
    yaws = np.unwrap(np.asarray([sample.yaw_rad for sample in samples]))
    planar_speeds = np.linalg.norm(velocities[:, :2], axis=1)
    position_std_m = float(np.linalg.norm(np.std(positions[:, :2], axis=0)))
    speed_percentile_mps = float(np.percentile(planar_speeds, 90.0))
    if speed_percentile_mps > args.max_speed_mps:
        raise RuntimeError(
            f"Go2 is moving: p90 planar speed {speed_percentile_mps:.3f}m/s"
        )
    if position_std_m > args.max_position_std_m:
        raise RuntimeError(
            f"Go2 pose is unstable: planar position std {position_std_m:.3f}m"
        )
    payload = {
        "schema": "navdp-imagegoal-target-v2",
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "state_topic": args.state_topic,
        "sample_count": len(samples),
        "position_xyz": np.median(positions, axis=0).round(6).tolist(),
        "yaw_rad": round(float(np.median(yaws)), 6),
        "velocity_xyz": np.median(velocities, axis=0).round(6).tolist(),
        "planar_speed_p90_mps": round(speed_percentile_mps, 6),
        "planar_position_std_m": round(position_std_m, 6),
        "image_goal_path": str(image_path),
        "image_goal_sha256": file_sha256(image_path),
        "image_goal_depth_path": str(depth_path),
        "image_goal_depth_sha256": file_sha256(depth_path),
        "visual_reference_width": int(visual_reference.target_rgb.shape[1]),
        "visual_reference_height": int(visual_reference.target_rgb.shape[0]),
        "visual_reference_features": len(visual_reference.target_keypoints),
        "visual_reference_feature_coverage": round(
            visual_reference.target_feature_coverage, 4
        ),
        "visual_reference_depth_features": (
            visual_reference.valid_target_depth_features
        ),
    }
    output = write_json(args.output, payload)
    print(f"Saved ImageGoal target pose: {output}")
    print(json.dumps(payload, ensure_ascii=False))
    return 0


def safety_abort_reason(
    metrics: dict,
    *,
    max_path_length_m: float = 0.0,
    max_target_distance_regression_m: float = 0.0,
) -> str:
    """Return a fail-closed reason when an episode exceeds frozen bounds."""
    path_length = metrics.get("path_length_m")
    if (
        max_path_length_m > 0.0
        and isinstance(path_length, (int, float))
        and math.isfinite(float(path_length))
        and float(path_length) > max_path_length_m
    ):
        return "path_length_limit"
    initial_distance = metrics.get("initial_distance_m")
    current_distance = metrics.get("current_distance_m")
    if (
        max_target_distance_regression_m > 0.0
        and isinstance(initial_distance, (int, float))
        and isinstance(current_distance, (int, float))
        and math.isfinite(float(initial_distance))
        and math.isfinite(float(current_distance))
        and float(current_distance)
        > float(initial_distance) + max_target_distance_regression_m
    ):
        return "target_distance_regression"
    return ""


class EvaluationNode:
    def __init__(
        self,
        args,
        target: dict,
        state_subscriber_type,
        state_message_type,
        visual_verifier: Optional[VisualGoalVerifier],
    ):
        import message_filters
        import rclpy
        from cv_bridge import CvBridge, CvBridgeError
        from nav_msgs.msg import Path as RosPath
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool, String

        class NodeImpl(Node):
            pass

        self.rclpy = rclpy
        self.bool_type = Bool
        self.string_type = String
        self.cv_bridge_error_type = CvBridgeError
        self.node = NodeImpl("navdp_imagegoal_evaluator")
        self.bridge = CvBridge()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.node.create_publisher(String, args.status_topic, qos)
        self.estop_pub = self.node.create_publisher(Bool, args.estop_topic, 10)
        self.visual_debug_pub = self.node.create_publisher(
            Image, args.visual_debug_topic, qos
        )
        self.args = args
        self.target = target
        self.visual_verifier = visual_verifier
        self.visual_result: Optional[VisualMatchResult] = None
        self.visual_error = ""
        self.policy_stop_tracker = PolicyStopTracker(
            hold_s=args.policy_stop_hold_s,
            max_linear_mps=args.policy_stop_max_linear_mps,
            max_angular_rps=args.policy_stop_max_angular_rps,
            max_path_radius_m=args.policy_stop_max_path_radius_m,
            status_timeout_s=args.policy_status_timeout_s,
            path_timeout_s=args.policy_path_timeout_s,
        )
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_depth_m: Optional[np.ndarray] = None
        self.latest_rgbd_monotonic = 0.0
        self.latest_rgbd_sequence = 0
        self.processed_rgbd_sequence = 0
        self.last_visual_evaluation_s = 0.0
        self.tracker = EpisodeTracker(
            np.asarray(target["position_xyz"][:2], dtype=np.float64),
            float(target["yaw_rad"]),
            args.success_distance_m,
            args.success_speed_mps,
            args.success_hold_s,
            args.max_position_jump_m,
        )
        self.latest_sample: Optional[StateSample] = None
        self.latest_sequence = 0
        self.processed_sequence = 0
        self.lock = threading.RLock()
        self.started_s = time.monotonic()
        self.done = False
        self.final_payload: Optional[dict] = None
        self.subscriber = state_subscriber_type(args.state_topic, state_message_type)
        self.subscriber.Init(self._on_state, 10)
        self.navdp_status_subscriber = self.node.create_subscription(
            String, args.navdp_status_topic, self._on_navdp_status, qos
        )
        self.navdp_path_subscriber = self.node.create_subscription(
            RosPath, args.navdp_path_topic, self._on_navdp_path, qos
        )
        self.rgb_subscriber = message_filters.Subscriber(
            self.node,
            Image,
            args.rgb_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscriber = message_filters.Subscriber(
            self.node,
            Image,
            args.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.rgbd_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber],
            queue_size=args.rgbd_sync_queue_size,
            slop=args.max_rgb_depth_skew_s,
        )
        self.rgbd_synchronizer.registerCallback(self._on_rgbd)
        self.timer = self.node.create_timer(1.0 / args.rate_hz, self._tick)

    def _on_state(self, message) -> None:
        with self.lock:
            self.latest_sample = state_from_message(message)
            self.latest_sequence += 1

    def _on_navdp_status(self, message) -> None:
        now_s = time.monotonic()
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("status payload is not an object")
        except (json.JSONDecodeError, TypeError, ValueError):
            payload = {}
        with self.lock:
            self.policy_stop_tracker.update_status(payload, now_s)

    def _on_navdp_path(self, message) -> None:
        path_xy = np.asarray(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in message.poses
            ],
            dtype=np.float64,
        ).reshape(-1, 2)
        with self.lock:
            self.policy_stop_tracker.update_path(path_xy, time.monotonic())

    def _on_rgbd(self, rgb_message, depth_message) -> None:
        try:
            rgb = np.asarray(
                self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="rgb8"),
                dtype=np.uint8,
            )
            depth_raw = np.asarray(
                self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            )
            depth_m = depth_array_to_meters(depth_raw, self.args.depth_scale_m)
        except self.cv_bridge_error_type as exc:
            with self.lock:
                self.visual_error = f"rgbd_conversion_failed:{exc}"
            return
        if rgb.shape[:2] != depth_m.shape:
            with self.lock:
                self.visual_error = (
                    f"rgbd_shape_mismatch:{rgb.shape[:2]}!={depth_m.shape}"
                )
            return
        with self.lock:
            self.latest_rgb = rgb.copy()
            self.latest_depth_m = depth_m.copy()
            self.latest_rgbd_monotonic = time.monotonic()
            self.latest_rgbd_sequence += 1
            self.visual_error = ""

    def _publish(self, payload: dict) -> None:
        message = self.string_type()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(message)

    def _visual_payload(self, now_s: Optional[float] = None) -> dict:
        now = time.monotonic() if now_s is None else now_s
        payload = (
            self.visual_result.to_dict()
            if self.visual_result is not None
            else {
                "matched": False,
                "confirmed": False,
                "reason": "waiting_for_rgbd",
                "consecutive_matches": 0,
                "goal_object_matched": False,
                "goal_object_confirmed": False,
                "goal_object_reason": "waiting_for_rgbd",
                "consecutive_goal_object_matches": 0,
            }
        )
        payload["error"] = self.visual_error
        payload["rgbd_age_s"] = (
            round(max(0.0, now - self.latest_rgbd_monotonic), 3)
            if self.latest_rgbd_monotonic > 0.0
            else None
        )
        return payload
    def _policy_stop_payload(self, now_s: Optional[float] = None) -> dict:
        now = time.monotonic() if now_s is None else now_s
        with self.lock:
            return self.policy_stop_tracker.snapshot(now)

    def _pose_metrics_valid(self, metrics: dict) -> bool:
        initial_distance = metrics.get("initial_distance_m")
        return bool(
            metrics.get("sample_count_ready")
            and not metrics.get("invalid_reason")
            and initial_distance is not None
            and self.args.min_start_distance_m
            <= float(initial_distance)
            <= self.args.max_start_distance_m
        )

    def _evaluate_visual(self, now_s: float) -> None:
        if self.visual_verifier is None:
            return
        minimum_interval = 1.0 / self.args.visual_rate_hz
        with self.lock:
            sequence = self.latest_rgbd_sequence
            rgb = None if self.latest_rgb is None else self.latest_rgb.copy()
            depth_m = (
                None if self.latest_depth_m is None else self.latest_depth_m.copy()
            )
        if (
            sequence == self.processed_rgbd_sequence
            or rgb is None
            or depth_m is None
            or now_s - self.last_visual_evaluation_s < minimum_interval
        ):
            return
        self.processed_rgbd_sequence = sequence
        self.last_visual_evaluation_s = now_s
        try:
            result = self.visual_verifier.evaluate(rgb, depth_m)
        except Exception as exc:
            with self.lock:
                self.visual_error = f"visual_evaluation_failed:{exc}"
            return
        with self.lock:
            self.visual_result = result
            self.visual_error = ""
        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                self.visual_verifier.last_debug_rgb, encoding="rgb8"
            )
            debug_message.header.stamp = self.node.get_clock().now().to_msg()
            self.visual_debug_pub.publish(debug_message)
        except Exception as exc:
            with self.lock:
                self.visual_error = f"visual_debug_publish_failed:{exc}"

    def _assert_estop(self) -> None:
        if not self.args.auto_estop:
            return
        message = self.bool_type()
        message.data = True
        for _ in range(3):
            self.estop_pub.publish(message)
            time.sleep(0.05)

    def _arrival_signals(self, sample: Optional[StateSample], now_s: float) -> dict:
        policy_stop = self._policy_stop_payload(now_s)
        view_confirmed = bool(
            self.visual_result is not None and self.visual_result.confirmed
        )
        goal_object_recognized = bool(
            self.visual_result is not None
            and self.visual_result.goal_object_confirmed
        )
        speed_mps = (
            float(np.abs(sample.velocity_xyz).sum()) if sample is not None else math.inf
        )
        exact_view_success = view_confirmed and speed_mps <= self.args.success_speed_mps
        goal_object_success = (
            goal_object_recognized
            and bool(policy_stop["confirmed"])
            and speed_mps <= self.args.goal_object_success_speed_mps
        )
        return {
            "exact_view_success": exact_view_success,
            "goal_object_recognized": goal_object_recognized,
            "goal_object_success": goal_object_success,
            "goal_object_speed_ready": (
                speed_mps <= self.args.goal_object_success_speed_mps
            ),
            "policy_stop": policy_stop,
        }

    def _finish(self, termination: str) -> None:
        if self.done:
            return
        now_s = time.monotonic()
        metrics = self.tracker.metrics()
        raw_pose_success = bool(metrics.pop("success", False))
        auxiliary_pose_valid = self._pose_metrics_valid(metrics)
        pose_success = raw_pose_success and auxiliary_pose_valid
        success = termination == "success"
        signals = self._arrival_signals(self.latest_sample, now_s)
        if signals["exact_view_success"]:
            achieved_level = "exact_view"
        elif signals["goal_object_success"]:
            achieved_level = "goal_object"
        elif pose_success:
            achieved_level = "pose_only"
        else:
            achieved_level = "none"
        if success and self.tracker.start_xy is not None:
            initial_distance = float(
                np.linalg.norm(self.tracker.start_xy - self.tracker.target_xy)
            )
            metrics["spl"] = round(
                initial_distance
                / max(self.tracker.path_length_m, initial_distance, 1e-6),
                4,
            )
        payload = {
            "schema": "navdp-imagegoal-episode-v3",
            "episode": self.args.episode,
            "arrival_mode": self.args.arrival_mode,
            "termination": termination,
            "success": success,
            "achieved_level": achieved_level,
            "goal_object_success": signals["goal_object_success"],
            "goal_object_recognized": signals["goal_object_recognized"],
            "exact_view_success": signals["exact_view_success"],
            "pose_success": pose_success,
            "raw_pose_success": raw_pose_success,
            "auxiliary_pose_valid": auxiliary_pose_valid,
            "completed_utc": datetime.now(timezone.utc).isoformat(),
            "target_pose_file": str(Path(self.args.target_pose).expanduser().resolve()),
            "image_goal_path": self.target.get("image_goal_path"),
            "image_goal_sha256": self.target.get("image_goal_sha256"),
            "image_goal_depth_path": self.target.get("image_goal_depth_path"),
            "image_goal_depth_sha256": self.target.get("image_goal_depth_sha256"),
            "success_distance_m": self.args.success_distance_m,
            "success_speed_mps": self.args.success_speed_mps,
            "success_hold_s": self.args.success_hold_s,
            "goal_object_success_speed_mps": (
                self.args.goal_object_success_speed_mps
            ),
            "visual_thresholds": (
                self.visual_verifier.settings()
                if self.visual_verifier is not None
                else None
            ),
            "visual": self._visual_payload(),
            "policy_stop": signals["policy_stop"],
            **metrics,
        }
        self.final_payload = payload
        output = self.args.output
        if not output:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output = GO2_DIR / "goals" / "results" / f"{stamp}_{self.args.episode}.json"
        destination = Path(output).expanduser().resolve()
        payload["result_path"] = str(destination)
        write_json(destination, payload)
        self._publish(payload)
        self._assert_estop()
        self.done = True
        print(json.dumps(payload, ensure_ascii=False, indent=2))

    def _tick(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        if now - self.started_s > self.args.timeout_s:
            self._finish("timeout")
            return
        with self.lock:
            sample = self.latest_sample
            sequence = self.latest_sequence
        if sample is None:
            if now - self.started_s > self.args.state_timeout_s:
                self.tracker.invalid_reason = "state_timeout"
                self._finish("invalid_state")
            return
        if now - sample.monotonic_s > self.args.state_timeout_s:
            self.tracker.invalid_reason = "state_stale"
            self._finish("invalid_state")
            return
        self._evaluate_visual(now)
        visual_required = self.args.arrival_mode in {
            "object",
            "visual",
            "visual_pose",
        }
        if visual_required:
            if self.latest_rgbd_monotonic <= 0.0:
                if now - self.started_s > self.args.visual_startup_timeout_s:
                    self.visual_error = "rgbd_timeout"
                    self._finish("invalid_visual")
                    return
            elif now - self.latest_rgbd_monotonic > self.args.visual_timeout_s:
                self.visual_error = "rgbd_stale"
                self._finish("invalid_visual")
                return
        pose_result = "running"
        if sequence != self.processed_sequence:
            self.processed_sequence = sequence
            pose_result = self.tracker.update(sample)
            if self.tracker.start_xy is not None:
                initial_distance = float(
                    np.linalg.norm(self.tracker.start_xy - self.tracker.target_xy)
                )
                if (
                    self.args.arrival_mode in {"pose", "visual_pose"}
                    and not self.args.min_start_distance_m
                    <= initial_distance
                    <= self.args.max_start_distance_m
                ):
                    self.tracker.invalid_reason = "start_distance_out_of_range"
                    self._finish("invalid_start")
                    return
            if pose_result == "invalid" and self.args.arrival_mode in {
                "pose",
                "visual_pose",
            }:
                self._finish("invalid_state")
                return
        pose_confirmed = self.tracker.success
        signals = self._arrival_signals(sample, now)
        if self.args.arrival_mode == "object":
            arrived = signals["goal_object_success"]
        elif self.args.arrival_mode == "pose":
            arrived = pose_confirmed
        elif self.args.arrival_mode == "visual_pose":
            arrived = pose_confirmed and signals["exact_view_success"]
        else:
            arrived = signals["exact_view_success"]
        if arrived:
            self._finish("success")
            return
        metrics = self.tracker.metrics(now)
        abort_reason = safety_abort_reason(
            metrics,
            max_path_length_m=max(0.0, self.args.max_path_length_m),
            max_target_distance_regression_m=max(
                0.0, self.args.max_target_distance_regression_m
            ),
        )
        if abort_reason:
            self.tracker.invalid_reason = abort_reason
            self._finish(f"safety_abort_{abort_reason}")
            return
        raw_pose_success = bool(metrics.pop("success", False))
        metrics["raw_pose_success"] = raw_pose_success
        metrics["auxiliary_pose_valid"] = self._pose_metrics_valid(metrics)
        metrics["pose_success"] = (
            raw_pose_success and metrics["auxiliary_pose_valid"]
        )
        payload = {
            "episode": self.args.episode,
            "arrival_mode": self.args.arrival_mode,
            "state": "running",
            "visual": self._visual_payload(now),
            "goal_object_success": signals["goal_object_success"],
            "exact_view_success": signals["exact_view_success"],
            "policy_stop": signals["policy_stop"],
            **metrics,
        }
        self._publish(payload)

    def stop(self) -> None:
        if not self.done:
            self._finish("operator_stop")
        self.node.destroy_node()


def run_episode(args, ros_args) -> int:
    target_path = Path(args.target_pose).expanduser().resolve()
    target = json.loads(target_path.read_text(encoding="utf-8"))
    if target.get("schema") not in {
        "navdp-imagegoal-target-v1",
        "navdp-imagegoal-target-v2",
    }:
        raise ValueError(f"unsupported target pose schema in {target_path}")
    image_path = Path(target["image_goal_path"])
    if not image_path.is_file() or file_sha256(image_path) != target["image_goal_sha256"]:
        raise ValueError("saved target pose does not match the current image goal")
    visual_verifier = None
    depth_path_value = target.get("image_goal_depth_path")
    if depth_path_value:
        depth_path = Path(depth_path_value)
        if (
            not depth_path.is_file()
            or file_sha256(depth_path) != target.get("image_goal_depth_sha256")
        ):
            raise ValueError("saved target pose does not match the aligned goal depth")
        visual_verifier = VisualGoalVerifier(
            load_rgb_image(image_path),
            load_depth_image(depth_path),
            image_width=args.visual_image_width,
            ratio_test=args.visual_ratio_test,
            min_good_matches=args.visual_min_good_matches,
            min_inliers=args.visual_min_inliers,
            min_inlier_ratio=args.visual_min_inlier_ratio,
            min_coverage=args.visual_min_coverage,
            max_center_offset_norm=args.visual_max_center_offset_norm,
            min_image_scale=args.visual_min_image_scale,
            max_image_scale=args.visual_max_image_scale,
            max_rotation_deg=args.visual_max_rotation_deg,
            max_reprojection_error_px=args.visual_max_reprojection_error_px,
            min_depth_pairs=args.visual_min_depth_pairs,
            max_median_depth_error_m=args.visual_max_depth_error_m,
            required_consecutive_matches=args.visual_required_consecutive,
            goal_object_min_good_matches=(
                args.goal_object_min_good_matches
            ),
            goal_object_min_inliers=args.goal_object_min_inliers,
            goal_object_min_inlier_ratio=args.goal_object_min_inlier_ratio,
            goal_object_min_coverage=args.goal_object_min_coverage,
            goal_object_max_center_offset_norm=(
                args.goal_object_max_center_offset_norm
            ),
            goal_object_min_image_scale=args.goal_object_min_image_scale,
            goal_object_max_image_scale=args.goal_object_max_image_scale,
            goal_object_max_rotation_deg=args.goal_object_max_rotation_deg,
            goal_object_max_reprojection_error_px=(
                args.goal_object_max_reprojection_error_px
            ),
            goal_object_min_depth_pairs=args.goal_object_min_depth_pairs,
            goal_object_min_depth_delta_m=args.goal_object_min_depth_delta_m,
            goal_object_max_depth_delta_m=args.goal_object_max_depth_delta_m,
            goal_object_max_depth_delta_mad_m=(
                args.goal_object_max_depth_delta_mad_m
            ),
            goal_object_required_consecutive_matches=(
                args.goal_object_required_consecutive
            ),
        )
    if args.arrival_mode in {"object", "visual", "visual_pose"} and visual_verifier is None:
        raise ValueError(
            "visual arrival requires a v2 target reference with aligned goal depth; "
            "recapture it with capture_imagegoal_reference.sh"
        )

    factory, subscriber_type, message_type = import_unitree_sdk(args.sdk_path)
    factory(0, args.net_if)
    import rclpy

    rclpy.init(args=ros_args)
    evaluator = EvaluationNode(
        args,
        target,
        subscriber_type,
        message_type,
        visual_verifier,
    )
    try:
        while rclpy.ok() and not evaluator.done:
            rclpy.spin_once(evaluator.node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        evaluator.stop()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if evaluator.final_payload and evaluator.final_payload.get("success") else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="NavDP ImageGoal Go2 experiment")
    parser.add_argument("--net-if", default=os.environ.get("UNITREE_NET_IF", "eth0"))
    parser.add_argument(
        "--sdk-path",
        default=os.environ.get(
            "UNITREE_SDK2PY_PATH", "/home/nvidia/unitree_ws/src/unitree_sdk2_python"
        ),
    )
    parser.add_argument("--state-topic", default="rt/sportmodestate")
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture = subparsers.add_parser("capture", help="record target ground-truth pose")
    capture.add_argument("--image-goal", default=str(DEFAULT_IMAGE_GOAL))
    capture.add_argument(
        "--image-goal-depth", default=str(DEFAULT_IMAGE_GOAL_DEPTH)
    )
    capture.add_argument("--output", default=str(DEFAULT_TARGET_POSE))
    capture.add_argument("--samples", type=int, default=30)
    capture.add_argument("--timeout-s", type=float, default=5.0)
    capture.add_argument("--max-speed-mps", type=float, default=0.05)
    capture.add_argument("--max-position-std-m", type=float, default=0.01)
    capture.add_argument("--visual-image-width", type=int, default=480)

    run = subparsers.add_parser("run", help="score one ImageGoal episode")
    run.add_argument("--target-pose", default=str(DEFAULT_TARGET_POSE))
    run.add_argument("--episode", choices=("first", "revisit"), required=True)
    run.add_argument(
        "--arrival-mode",
        choices=("object", "visual", "visual_pose", "pose"),
        default="object",
    )
    run.add_argument("--output", default="")
    run.add_argument("--status-topic", default="/navdp/imagegoal_evaluation")
    run.add_argument("--navdp-status-topic", default="/navdp/status")
    run.add_argument("--navdp-path-topic", default="/navdp/trajectory")
    run.add_argument("--estop-topic", default="/navdp/estop")
    run.add_argument(
        "--visual-debug-topic", default="/navdp/imagegoal_match_debug"
    )
    run.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    run.add_argument(
        "--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw"
    )
    run.add_argument("--auto-estop", action="store_true")
    run.add_argument("--success-distance-m", type=float, default=0.85)
    run.add_argument("--success-speed-mps", type=float, default=0.30)
    run.add_argument("--success-hold-s", type=float, default=0.50)
    run.add_argument("--goal-object-success-speed-mps", type=float, default=0.10)
    run.add_argument("--policy-stop-hold-s", type=float, default=1.50)
    run.add_argument("--policy-stop-max-linear-mps", type=float, default=0.03)
    run.add_argument("--policy-stop-max-angular-rps", type=float, default=0.05)
    run.add_argument("--policy-stop-max-path-radius-m", type=float, default=0.20)
    run.add_argument("--policy-status-timeout-s", type=float, default=1.25)
    run.add_argument("--policy-path-timeout-s", type=float, default=2.50)
    run.add_argument("--min-start-distance-m", type=float, default=1.20)
    run.add_argument("--max-start-distance-m", type=float, default=8.00)
    run.add_argument("--max-position-jump-m", type=float, default=0.50)
    run.add_argument(
        "--max-path-length-m",
        type=float,
        default=0.0,
        help="fail-closed path-length limit; zero disables it",
    )
    run.add_argument(
        "--max-target-distance-regression-m",
        type=float,
        default=0.0,
        help="abort if target distance exceeds its initial value by this margin",
    )
    run.add_argument("--state-timeout-s", type=float, default=0.50)
    run.add_argument("--visual-startup-timeout-s", type=float, default=5.00)
    run.add_argument("--visual-timeout-s", type=float, default=1.00)
    run.add_argument("--timeout-s", type=float, default=60.0)
    run.add_argument("--rate-hz", type=float, default=20.0)
    run.add_argument("--visual-rate-hz", type=float, default=4.0)
    run.add_argument("--depth-scale-m", type=float, default=0.001)
    run.add_argument("--rgbd-sync-queue-size", type=int, default=15)
    run.add_argument("--max-rgb-depth-skew-s", type=float, default=0.10)
    run.add_argument("--visual-image-width", type=int, default=480)
    run.add_argument("--visual-ratio-test", type=float, default=0.72)
    run.add_argument("--visual-min-good-matches", type=int, default=30)
    run.add_argument("--visual-min-inliers", type=int, default=20)
    run.add_argument("--visual-min-inlier-ratio", type=float, default=0.45)
    run.add_argument("--visual-min-coverage", type=float, default=0.08)
    run.add_argument("--visual-max-center-offset-norm", type=float, default=0.18)
    run.add_argument("--visual-min-image-scale", type=float, default=0.70)
    run.add_argument("--visual-max-image-scale", type=float, default=1.45)
    run.add_argument("--visual-max-rotation-deg", type=float, default=25.0)
    run.add_argument(
        "--visual-max-reprojection-error-px", type=float, default=3.0
    )
    run.add_argument("--visual-min-depth-pairs", type=int, default=12)
    run.add_argument("--visual-max-depth-error-m", type=float, default=0.40)
    run.add_argument("--visual-required-consecutive", type=int, default=3)
    run.add_argument("--goal-object-min-good-matches", type=int, default=30)
    run.add_argument("--goal-object-min-inliers", type=int, default=20)
    run.add_argument("--goal-object-min-inlier-ratio", type=float, default=0.45)
    run.add_argument("--goal-object-min-coverage", type=float, default=0.02)
    run.add_argument(
        "--goal-object-max-center-offset-norm", type=float, default=0.45
    )
    run.add_argument("--goal-object-min-image-scale", type=float, default=0.55)
    run.add_argument("--goal-object-max-image-scale", type=float, default=2.25)
    run.add_argument("--goal-object-max-rotation-deg", type=float, default=30.0)
    run.add_argument(
        "--goal-object-max-reprojection-error-px", type=float, default=3.0
    )
    run.add_argument("--goal-object-min-depth-pairs", type=int, default=12)
    run.add_argument("--goal-object-min-depth-delta-m", type=float, default=-1.25)
    run.add_argument("--goal-object-max-depth-delta-m", type=float, default=0.25)
    run.add_argument(
        "--goal-object-max-depth-delta-mad-m", type=float, default=0.20
    )
    run.add_argument("--goal-object-required-consecutive", type=int, default=3)
    return parser


def main() -> None:
    parser = build_parser()
    args, ros_args = parser.parse_known_args()
    if args.command == "capture":
        raise SystemExit(capture_target(args))
    raise SystemExit(run_episode(args, ros_args))


if __name__ == "__main__":
    main()
