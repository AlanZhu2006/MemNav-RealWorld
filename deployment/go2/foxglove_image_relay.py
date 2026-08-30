#!/usr/bin/env python3
"""Publish bandwidth-bounded RGB and colorized-depth previews for Foxglove.

The relay is deliberately observation-only.  NavDP and safety consumers keep
subscribing to the original RealSense topics; only the Foxglove layout uses
these lossy JPEG previews.
"""

from __future__ import annotations

import argparse
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def _rows(message: Any, bytes_per_pixel: int) -> np.ndarray:
    expected_step = int(message.width) * bytes_per_pixel
    step = int(message.step)
    if step < expected_step:
        raise ValueError(f"image step {step} is smaller than {expected_step}")
    expected_size = int(message.height) * step
    data = memoryview(message.data)
    if data.nbytes < expected_size:
        raise ValueError(f"image data has {data.nbytes} bytes; expected {expected_size}")
    return np.frombuffer(data[:expected_size], dtype=np.uint8).reshape(
        int(message.height), step
    )


def rgb_message_to_bgr(message: Any) -> np.ndarray:
    """Decode common ROS RGB image encodings into a contiguous BGR array."""

    encoding = str(message.encoding).lower()
    if encoding in {"rgb8", "bgr8", "8uc3"}:
        channels = 3
    elif encoding in {"rgba8", "bgra8", "8uc4"}:
        channels = 4
    else:
        raise ValueError(f"unsupported RGB encoding: {message.encoding}")
    packed = _rows(message, channels)[:, : int(message.width) * channels]
    image = np.ascontiguousarray(
        packed.reshape(int(message.height), int(message.width), channels)
    )
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding in {"rgba8", "8uc4"}:
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def depth_message_to_u16(message: Any) -> np.ndarray:
    """Decode a 16-bit ROS depth image, including row padding and endianness."""

    encoding = str(message.encoding).lower()
    if encoding not in {"16uc1", "mono16"}:
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    rows = _rows(message, 2)
    packed = np.ascontiguousarray(rows[:, : int(message.width) * 2])
    byte_order = ">u2" if bool(message.is_bigendian) else "<u2"
    depth = packed.view(np.dtype(byte_order)).reshape(
        int(message.height), int(message.width)
    )
    return np.ascontiguousarray(depth.astype(np.uint16, copy=False))


def resize_rgb_preview(image: np.ndarray, width: int, height: int) -> np.ndarray:
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def colorize_depth_preview(
    depth_mm: np.ndarray,
    width: int,
    height: int,
    minimum_mm: float,
    maximum_mm: float,
) -> np.ndarray:
    """Resize depth with nearest-neighbour sampling and apply Turbo colors."""

    resized = cv2.resize(depth_mm, (width, height), interpolation=cv2.INTER_NEAREST)
    invalid = resized == 0
    clipped = np.clip(resized.astype(np.float32), minimum_mm, maximum_mm)
    scaled = ((clipped - minimum_mm) * (255.0 / (maximum_mm - minimum_mm))).astype(
        np.uint8
    )
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    color[invalid] = 0
    return color


def encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    ok, encoded = cv2.imencode(
        ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        raise RuntimeError("OpenCV failed to encode JPEG preview")
    return encoded.tobytes()


class FoxgloveImageRelay(Node):
    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__("navdp_foxglove_image_relay")
        self.options = options
        self._rgb_period = 1.0 / options.rgb_fps
        self._depth_period = 1.0 / options.depth_fps
        self._goal_period = 1.0 / options.goal_fps
        self._arrival_period = 1.0 / options.arrival_fps
        self._next_rgb = 0.0
        self._next_depth = 0.0
        self._next_goal = 0.0
        self._next_arrival = 0.0
        self._last_error_log = 0.0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._rgb_publisher = self.create_publisher(
            CompressedImage, options.rgb_output, qos
        )
        self._depth_publisher = self.create_publisher(
            CompressedImage, options.depth_output, qos
        )
        self._goal_publisher = self.create_publisher(
            CompressedImage, options.goal_output, qos
        )
        self._arrival_publisher = self.create_publisher(
            CompressedImage, options.arrival_output, qos
        )
        rgb_callbacks = MutuallyExclusiveCallbackGroup()
        depth_callbacks = MutuallyExclusiveCallbackGroup()
        goal_callbacks = MutuallyExclusiveCallbackGroup()
        arrival_callbacks = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image,
            options.rgb_input,
            self._on_rgb,
            qos,
            callback_group=rgb_callbacks,
        )
        self.create_subscription(
            Image,
            options.depth_input,
            self._on_depth,
            qos,
            callback_group=depth_callbacks,
        )
        self.create_subscription(
            Image,
            options.goal_input,
            self._on_goal,
            qos,
            callback_group=goal_callbacks,
        )
        self.create_subscription(
            Image,
            options.arrival_input,
            self._on_arrival,
            qos,
            callback_group=arrival_callbacks,
        )
        self.get_logger().info(
            "Foxglove previews: RGB %dx%d@%.1f Hz q=%d -> %s; "
            "depth %dx%d@%.1f Hz q=%d range=%.0f..%.0f mm -> %s"
            % (
                options.width,
                options.height,
                options.rgb_fps,
                options.rgb_jpeg_quality,
                options.rgb_output,
                options.width,
                options.height,
                options.depth_fps,
                options.depth_jpeg_quality,
                options.depth_min_mm,
                options.depth_max_mm,
                options.depth_output,
            )
        )
        self.get_logger().info(
            "Goal %.1f Hz q=%d -> %s; arrival %.1f Hz q=%d -> %s"
            % (
                options.goal_fps,
                options.goal_jpeg_quality,
                options.goal_output,
                options.arrival_fps,
                options.arrival_jpeg_quality,
                options.arrival_output,
            )
        )

    def _report_error(self, stream: str, error: Exception) -> None:
        now = time.monotonic()
        if now - self._last_error_log >= 5.0:
            self.get_logger().error(f"Cannot publish {stream} preview: {error}")
            self._last_error_log = now

    @staticmethod
    def _compressed(message: Image, data: bytes) -> CompressedImage:
        preview = CompressedImage()
        preview.header = message.header
        preview.format = "jpeg"
        preview.data = data
        return preview

    def _due(self, stream: str, now: float, period: float) -> bool:
        attribute = f"_next_{stream}"
        deadline = getattr(self, attribute)
        if deadline and now < deadline:
            return False
        if not deadline or now - deadline > period:
            setattr(self, attribute, now + period)
        else:
            # Advance from the previous deadline instead of resetting from the
            # current frame.  This avoids 30 Hz input quantizing a 15 Hz target
            # down to 10 Hz when the second source frame arrives just early.
            setattr(self, attribute, deadline + period)
        return True

    def _on_rgb(self, message: Image) -> None:
        now = time.monotonic()
        if not self._due("rgb", now, self._rgb_period):
            return
        try:
            image = rgb_message_to_bgr(message)
            image = resize_rgb_preview(
                image, self.options.width, self.options.height
            )
            data = encode_jpeg(image, self.options.rgb_jpeg_quality)
            self._rgb_publisher.publish(self._compressed(message, data))
        except (ValueError, RuntimeError, cv2.error) as error:
            self._report_error("RGB", error)

    def _publish_color_image(
        self,
        message: Image,
        stream: str,
        period: float,
        quality: int,
        publisher: Any,
    ) -> None:
        now = time.monotonic()
        if not self._due(stream, now, period):
            return
        try:
            image = rgb_message_to_bgr(message)
            image = resize_rgb_preview(
                image, self.options.width, self.options.height
            )
            publisher.publish(self._compressed(message, encode_jpeg(image, quality)))
        except (ValueError, RuntimeError, cv2.error) as error:
            self._report_error(stream, error)

    def _on_goal(self, message: Image) -> None:
        self._publish_color_image(
            message,
            "goal",
            self._goal_period,
            self.options.goal_jpeg_quality,
            self._goal_publisher,
        )

    def _on_arrival(self, message: Image) -> None:
        self._publish_color_image(
            message,
            "arrival",
            self._arrival_period,
            self.options.arrival_jpeg_quality,
            self._arrival_publisher,
        )

    def _on_depth(self, message: Image) -> None:
        now = time.monotonic()
        if not self._due("depth", now, self._depth_period):
            return
        try:
            depth = depth_message_to_u16(message)
            image = colorize_depth_preview(
                depth,
                self.options.width,
                self.options.height,
                self.options.depth_min_mm,
                self.options.depth_max_mm,
            )
            data = encode_jpeg(image, self.options.depth_jpeg_quality)
            self._depth_publisher.publish(self._compressed(message, data))
        except (ValueError, RuntimeError, cv2.error) as error:
            self._report_error("depth", error)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb-input", required=True)
    parser.add_argument("--depth-input", required=True)
    parser.add_argument("--rgb-output", required=True)
    parser.add_argument("--depth-output", required=True)
    parser.add_argument("--goal-input", required=True)
    parser.add_argument("--arrival-input", required=True)
    parser.add_argument("--goal-output", required=True)
    parser.add_argument("--arrival-output", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--rgb-fps", type=float, required=True)
    parser.add_argument("--depth-fps", type=float, required=True)
    parser.add_argument("--goal-fps", type=float, required=True)
    parser.add_argument("--arrival-fps", type=float, required=True)
    parser.add_argument("--rgb-jpeg-quality", type=int, required=True)
    parser.add_argument("--depth-jpeg-quality", type=int, required=True)
    parser.add_argument("--goal-jpeg-quality", type=int, required=True)
    parser.add_argument("--arrival-jpeg-quality", type=int, required=True)
    parser.add_argument("--depth-min-mm", type=float, required=True)
    parser.add_argument("--depth-max-mm", type=float, required=True)
    options = parser.parse_args()
    if options.width <= 0 or options.height <= 0:
        parser.error("preview width and height must be positive")
    if any(
        value <= 0
        for value in (
            options.rgb_fps,
            options.depth_fps,
            options.goal_fps,
            options.arrival_fps,
        )
    ):
        parser.error("preview frame rates must be positive")
    for label in (
        "rgb_jpeg_quality",
        "depth_jpeg_quality",
        "goal_jpeg_quality",
        "arrival_jpeg_quality",
    ):
        if not 1 <= getattr(options, label) <= 100:
            parser.error(f"{label.replace('_', '-')} must be in [1, 100]")
    if options.depth_min_mm < 0 or options.depth_max_mm <= options.depth_min_mm:
        parser.error("depth range must satisfy 0 <= min < max")
    return options


def main() -> None:
    options = parse_args()
    rclpy.init(args=[])
    node = FoxgloveImageRelay(options)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
