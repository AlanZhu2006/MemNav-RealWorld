#!/usr/bin/env python3
"""Observation-only live matcher for a frozen external Revisit goal.

This node is intentionally separate from the RGB arrival authority.  It runs
while the autonomous adapter is disabled so an operator can inspect whether a
manual Survey view matches the frozen M point.  It never publishes arrival,
estop, enable or velocity commands.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from image_goal_io import load_rgb_image
from rgb_goal_arrival import RgbArrivalResult, RgbGoalArrivalVerifier


SCHEMA = "memnav_revisit_goal_monitor_v1"


def render_match_debug(
    comparison_rgb: np.ndarray,
    result: RgbArrivalResult,
    *,
    point_label: str,
) -> np.ndarray:
    """Add a legible MATCH/NO MATCH receipt above the compact live overlay."""

    image = np.asarray(comparison_rgb, dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("comparison image must be HxWx3 RGB")
    banner_height = 58
    matched = bool(result.matched)
    color = (25, 150, 62) if matched else (190, 56, 48)
    canvas = np.zeros((image.shape[0] + banner_height, image.shape[1], 3), np.uint8)
    canvas[:banner_height] = color
    canvas[banner_height:] = image
    state = "MATCH" if matched else "NO MATCH"
    title = f"{point_label} REVISIT  {state}"
    detail = (
        f"reason={result.reason}  good={result.good_matches}  "
        f"inliers={result.inliers}  scale={result.image_scale:.3f}"
    )
    cv2.putText(
        canvas,
        title,
        (16, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        detail,
        (16, 49),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return canvas


class RevisitGoalMonitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("memnav_revisit_goal_monitor")
        self.args = args
        self.bridge = CvBridge()
        self.goal_path = Path(args.goal).expanduser().resolve()
        self.goal_sha256 = hashlib.sha256(self.goal_path.read_bytes()).hexdigest()
        self.verifier = RgbGoalArrivalVerifier(
            load_rgb_image(self.goal_path),
            min_image_scale=args.min_image_scale,
            max_image_scale=args.max_image_scale,
            required_consecutive_matches=1,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.debug_pub = self.create_publisher(Image, args.debug_topic, state_qos)
        self.status_pub = self.create_publisher(String, args.status_topic, state_qos)
        self.create_subscription(
            Image, args.rgb_topic, self._on_rgb, qos_profile_sensor_data
        )
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_sequence = 0
        self.processed_sequence = 0
        self.last_result: Optional[RgbArrivalResult] = None
        self.last_error = ""
        self.last_evaluated_monotonic = 0.0
        self.create_timer(1.0 / args.rate_hz, self._tick)
        self.get_logger().info(
            "Observation-only Revisit monitor: point=%s goal_sha256=%s; "
            "no motion or arrival authority"
            % (args.point_label, self.goal_sha256)
        )

    def _on_rgb(self, message: Image) -> None:
        try:
            self.latest_rgb = np.asarray(
                self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8"),
                dtype=np.uint8,
            ).copy()
            self.latest_sequence += 1
        except CvBridgeError as exc:
            self.last_error = f"rgb_decode_failed:{exc}"

    def _publish_status(self) -> None:
        age_s = (
            None
            if self.last_evaluated_monotonic <= 0.0
            else round(time.monotonic() - self.last_evaluated_monotonic, 3)
        )
        payload = {
            "schema": SCHEMA,
            "authority": "observation_only",
            "point_label": self.args.point_label,
            "goal_path": str(self.goal_path),
            "goal_sha256": self.goal_sha256,
            "latest_rgb_ready": self.latest_rgb is not None,
            "evaluation_age_s": age_s,
            "error": self.last_error,
            "result": (
                None if self.last_result is None else self.last_result.to_dict()
            ),
        }
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )
        self.status_pub.publish(message)

    def _tick(self) -> None:
        if (
            self.latest_rgb is None
            or self.latest_sequence == self.processed_sequence
        ):
            self._publish_status()
            return
        self.processed_sequence = self.latest_sequence
        try:
            result = self.verifier.evaluate(self.latest_rgb)
            self.last_result = result
            self.last_error = ""
            self.last_evaluated_monotonic = time.monotonic()
            debug_rgb = render_match_debug(
                self.verifier.last_debug_rgb,
                result,
                point_label=self.args.point_label,
            )
            message = self.bridge.cv2_to_imgmsg(debug_rgb, encoding="rgb8")
            message.header.stamp = self.get_clock().now().to_msg()
            self.debug_pub.publish(message)
        except Exception as exc:
            self.last_error = f"match_failed:{type(exc).__name__}:{exc}"
        self._publish_status()


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", required=True)
    parser.add_argument("--point-label", default="M")
    parser.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--debug-topic", default="/navdp/rgb_arrival_debug")
    parser.add_argument("--status-topic", default="/navdp/rgb_arrival_status")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--min-image-scale", type=float, default=0.60)
    parser.add_argument("--max-image-scale", type=float, default=1.45)
    args = parser.parse_args(argv)
    if not Path(args.goal).expanduser().is_file():
        parser.error("--goal must name an existing image")
    if not args.point_label.strip():
        parser.error("--point-label must not be empty")
    if not 0.1 <= args.rate_hz <= 15.0:
        parser.error("--rate-hz must be in [0.1, 15]")
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = RevisitGoalMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
