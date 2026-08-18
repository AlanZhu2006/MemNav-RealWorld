#!/usr/bin/env python3
"""Capture a synchronized RGB-D reference for NavDP ImageGoal evaluation."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from image_goal_io import depth_array_to_meters, save_depth_image, save_rgb_image


class ImageGoalCapture(Node):
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        samples: int,
        depth_scale_m: float,
        max_rgb_depth_skew_s: float,
    ) -> None:
        super().__init__("navdp_image_goal_capture")
        self.bridge = CvBridge()
        self.samples = max(1, int(samples))
        self.depth_scale_m = float(depth_scale_m)
        self.received = 0
        self.best_rgb: np.ndarray | None = None
        self.best_depth_m: np.ndarray | None = None
        self.best_sharpness = -1.0
        self.best_skew_s = 0.0
        self.rgb_subscriber = message_filters.Subscriber(
            self, Image, rgb_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_subscriber = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber],
            queue_size=15,
            slop=float(max_rgb_depth_skew_s),
        )
        self.synchronizer.registerCallback(self._on_rgbd)

    @property
    def complete(self) -> bool:
        return self.received >= self.samples

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

    def _on_rgbd(self, rgb_message: Image, depth_message: Image) -> None:
        if self.complete:
            return
        try:
            rgb = np.asarray(
                self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="rgb8"),
                dtype=np.uint8,
            )
            depth_raw = np.asarray(
                self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            )
        except CvBridgeError as exc:
            self.get_logger().warning(f"RGB-D conversion failed: {exc}")
            return
        depth_m = depth_array_to_meters(depth_raw, self.depth_scale_m)
        if rgb.shape[:2] != depth_m.shape:
            self.get_logger().warning(
                f"RGB/depth shape mismatch: {rgb.shape[:2]} vs {depth_m.shape}"
            )
            return
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        skew_s = abs(
            self._stamp_seconds(rgb_message) - self._stamp_seconds(depth_message)
        )
        self.received += 1
        if sharpness > self.best_sharpness:
            self.best_rgb = rgb.copy()
            self.best_depth_m = depth_m.copy()
            self.best_sharpness = sharpness
            self.best_skew_s = skew_s


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture a NavDP image goal")
    parser.add_argument(
        "--rgb-topic", "--topic", dest="rgb_topic",
        default="/camera/camera/color/image_raw",
    )
    parser.add_argument(
        "--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw"
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--depth-output", default="")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--depth-scale-m", type=float, default=0.001)
    parser.add_argument("--max-rgb-depth-skew-s", type=float, default=0.10)
    args = parser.parse_args()
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    if args.depth_scale_m <= 0.0:
        parser.error("--depth-scale-m must be positive")

    rclpy.init()
    node = ImageGoalCapture(
        args.rgb_topic,
        args.depth_topic,
        args.samples,
        args.depth_scale_m,
        args.max_rgb_depth_skew_s,
    )
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and not node.complete and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.best_rgb is None or node.best_depth_m is None:
            raise RuntimeError(
                f"no synchronized RGB-D pair received from {args.rgb_topic} and "
                f"{args.depth_topic}"
            )
        output = save_rgb_image(args.output, node.best_rgb)
        depth_output = (
            Path(args.depth_output).expanduser()
            if args.depth_output
            else output.with_name(f"{output.stem}_depth.png")
        )
        depth_output = save_depth_image(depth_output, node.best_depth_m)
        valid_depth = node.best_depth_m[
            np.isfinite(node.best_depth_m) & (node.best_depth_m > 0.0)
        ]
        metadata = {
            "captured_utc": datetime.now(timezone.utc).isoformat(),
            "rgb_topic": args.rgb_topic,
            "depth_topic": args.depth_topic,
            "samples": node.received,
            "selected_sharpness": round(node.best_sharpness, 3),
            "selected_rgb_depth_skew_s": round(node.best_skew_s, 6),
            "height": int(node.best_rgb.shape[0]),
            "width": int(node.best_rgb.shape[1]),
            "depth_path": str(depth_output),
            "depth_scale_m": 0.001,
            "valid_depth_fraction": round(
                float(valid_depth.size / node.best_depth_m.size), 4
            ),
        }
        Path(f"{output}.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"Saved image goal: {output}")
        print(f"Saved aligned goal depth: {depth_output}")
        print(json.dumps(metadata, ensure_ascii=False))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
