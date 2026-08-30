#!/usr/bin/env python3
"""Publish bandwidth-bounded RGB and colorized-depth previews for Foxglove.

The relay is deliberately observation-only.  NavDP and safety consumers keep
subscribing to the original RealSense topics; only the Foxglove layout uses
these lossy JPEG previews.
"""

from __future__ import annotations

import argparse
import json
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
from std_msgs.msg import String


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


def prepare_color_preview(
    message: Any, width: int, height: int, *, resize: bool
) -> np.ndarray:
    """Decode a color image and optionally resize it for display.

    Arrival debug images already contain two camera views side by side.  They
    must keep their native aspect ratio; resizing them to the single-camera
    preview dimensions makes both halves unnaturally narrow.
    """

    image = rgb_message_to_bgr(message)
    if resize:
        return resize_rgb_preview(image, width, height)
    return image


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


_CARD_BACKGROUND = (30, 33, 39)
_CARD_SURFACE = (42, 46, 54)
_TEXT_PRIMARY = (238, 240, 244)
_TEXT_SECONDARY = (163, 171, 184)
_GOOD = (101, 210, 143)
_WARNING = (78, 190, 245)
_DANGER = (90, 92, 245)
_NEUTRAL = (128, 137, 150)


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if np.isfinite(result) else None


def _metric_color(
    value: float | None,
    *,
    good_max: float | None = None,
    danger_max: float | None = None,
    danger_min: float | None = None,
    good_min: float | None = None,
) -> tuple[int, int, int]:
    if value is None:
        return _NEUTRAL
    if good_max is not None and value <= good_max:
        return _GOOD
    if danger_max is not None and value > danger_max:
        return _DANGER
    if danger_min is not None and value <= danger_min:
        return _DANGER
    if good_min is not None and value >= good_min:
        return _GOOD
    return _WARNING


def _fit_text(value: Any, limit: int) -> str:
    text = "-" if value in (None, "") else str(value)
    if len(text) <= limit:
        return text
    return text[: max(1, limit - 3)] + "..."


def render_status_card(
    payload: dict[str, Any], width: int, height: int
) -> np.ndarray:
    """Render the high-value navigation state as a compact Foxglove image."""

    if width < 480 or height < 220:
        raise ValueError("status card must be at least 480x220")
    canvas = np.full((height, width, 3), _CARD_BACKGROUND, dtype=np.uint8)
    pad = max(12, round(width * 0.02))

    enabled = bool(payload.get("enabled"))
    estop = bool(payload.get("estop"))
    arrived = bool(payload.get("arrival_latched"))
    if arrived:
        state, state_color = "ARRIVED / LOCKED", _GOOD
    elif estop:
        state, state_color = "E-STOP / LOCKED", _DANGER
    elif enabled:
        state, state_color = "NAVIGATING", _GOOD
    else:
        state, state_color = "READY / DISABLED", _WARNING

    cv2.putText(
        canvas,
        "NAVDP OPERATOR STATUS",
        (pad, 29),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.53,
        _TEXT_SECONDARY,
        1,
        cv2.LINE_AA,
    )
    (state_width, _), _ = cv2.getTextSize(
        state, cv2.FONT_HERSHEY_SIMPLEX, 0.68, 2
    )
    badge_left = max(pad + 220, width - pad - state_width - 22)
    cv2.rectangle(canvas, (badge_left, 10), (width - pad, 43), state_color, -1)
    cv2.putText(
        canvas,
        state,
        (badge_left + 11, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (18, 20, 24),
        2,
        cv2.LINE_AA,
    )

    rgbd_age = _number(payload.get("rgbd_age_s"))
    plan_age = _number(payload.get("plan_age_s"))
    clearance = _number(payload.get("clearance_m"))
    cmd_vx = _number(payload.get("cmd_vx"))
    cmd_wz = _number(payload.get("cmd_wz"))
    metrics = (
        (
            "RGB-D AGE",
            "n/a" if rgbd_age is None else f"{rgbd_age:.2f} s",
            _metric_color(rgbd_age, good_max=0.25, danger_max=0.75),
        ),
        (
            "PLAN AGE",
            "n/a" if plan_age is None else f"{plan_age:.2f} s",
            _metric_color(plan_age, good_max=1.5, danger_max=2.5),
        ),
        (
            "CLEARANCE",
            "n/a" if clearance is None else f"{clearance:.2f} m",
            _metric_color(clearance, danger_min=0.45, good_min=0.80),
        ),
        (
            "COMMAND",
            (
                "n/a"
                if cmd_vx is None or cmd_wz is None
                else f"{cmd_vx:+.2f} / {cmd_wz:+.2f}"
            ),
            _GOOD if enabled and not estop else _NEUTRAL,
        ),
    )
    gap = 8
    card_top, card_bottom = 57, min(height - 82, 164)
    card_width = (width - 2 * pad - 3 * gap) // 4
    for index, (label, value, color) in enumerate(metrics):
        left = pad + index * (card_width + gap)
        right = width - pad if index == 3 else left + card_width
        cv2.rectangle(canvas, (left, card_top), (right, card_bottom), _CARD_SURFACE, -1)
        cv2.rectangle(canvas, (left, card_top), (left + 5, card_bottom), color, -1)
        cv2.putText(
            canvas,
            label,
            (left + 14, card_top + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            _TEXT_SECONDARY,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            value,
            (left + 14, card_top + 61),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.63,
            _TEXT_PRIMARY,
            2,
            cv2.LINE_AA,
        )

    footer_top = card_bottom + 25
    phase = _fit_text(payload.get("phase"), 18)
    stop_reason = _fit_text(payload.get("stop_reason"), 28)
    goal = "LOADED" if payload.get("image_goal_loaded") else "MISSING"
    arrival = "YES" if arrived else "NO"
    cv2.putText(
        canvas,
        f"PHASE  {phase}     GOAL  {goal}     ARRIVAL  {arrival}     STOP  {stop_reason}",
        (pad, footer_top),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.47,
        _TEXT_PRIMARY,
        1,
        cv2.LINE_AA,
    )
    error = _fit_text(payload.get("last_error"), 84)
    error_color = _DANGER if error != "-" else _TEXT_SECONDARY
    cv2.putText(
        canvas,
        f"ERROR  {error}",
        (pad, min(height - 18, footer_top + 34)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        error_color,
        1,
        cv2.LINE_AA,
    )
    return canvas


class FoxgloveImageRelay(Node):
    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__("navdp_foxglove_image_relay")
        self.options = options
        self._rgb_period = 1.0 / options.rgb_fps
        self._depth_period = 1.0 / options.depth_fps
        self._goal_period = 1.0 / options.goal_fps
        self._arrival_period = 1.0 / options.arrival_fps
        self._status_period = 1.0 / options.status_fps
        self._next_rgb = 0.0
        self._next_depth = 0.0
        self._next_goal = 0.0
        self._next_arrival = 0.0
        self._next_status = 0.0
        self._last_error_log = 0.0
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._rgb_publisher = self.create_publisher(
            CompressedImage, options.rgb_output, sensor_qos
        )
        self._depth_publisher = self.create_publisher(
            CompressedImage, options.depth_output, sensor_qos
        )
        self._goal_publisher = self.create_publisher(
            CompressedImage, options.goal_output, state_qos
        )
        self._arrival_publisher = self.create_publisher(
            CompressedImage, options.arrival_output, state_qos
        )
        self._status_publisher = self.create_publisher(
            CompressedImage, options.status_output, state_qos
        )
        rgb_callbacks = MutuallyExclusiveCallbackGroup()
        depth_callbacks = MutuallyExclusiveCallbackGroup()
        goal_callbacks = MutuallyExclusiveCallbackGroup()
        arrival_callbacks = MutuallyExclusiveCallbackGroup()
        status_callbacks = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image,
            options.rgb_input,
            self._on_rgb,
            sensor_qos,
            callback_group=rgb_callbacks,
        )
        self.create_subscription(
            Image,
            options.depth_input,
            self._on_depth,
            sensor_qos,
            callback_group=depth_callbacks,
        )
        self.create_subscription(
            Image,
            options.goal_input,
            self._on_goal,
            state_qos,
            callback_group=goal_callbacks,
        )
        self.create_subscription(
            Image,
            options.arrival_input,
            self._on_arrival,
            state_qos,
            callback_group=arrival_callbacks,
        )
        self.create_subscription(
            String,
            options.status_input,
            self._on_status,
            state_qos,
            callback_group=status_callbacks,
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
            "Goal %.1f Hz q=%d -> %s; arrival %.1f Hz q=%d native_aspect=%s -> %s; "
            "status %dx%d@%.1f Hz q=%d -> %s"
            % (
                options.goal_fps,
                options.goal_jpeg_quality,
                options.goal_output,
                options.arrival_fps,
                options.arrival_jpeg_quality,
                options.arrival_preserve_resolution,
                options.arrival_output,
                options.status_width,
                options.status_height,
                options.status_fps,
                options.status_jpeg_quality,
                options.status_output,
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
        *,
        resize: bool,
    ) -> None:
        now = time.monotonic()
        if not self._due(stream, now, period):
            return
        try:
            image = prepare_color_preview(
                message,
                self.options.width,
                self.options.height,
                resize=resize,
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
            resize=True,
        )

    def _on_arrival(self, message: Image) -> None:
        self._publish_color_image(
            message,
            "arrival",
            self._arrival_period,
            self.options.arrival_jpeg_quality,
            self._arrival_publisher,
            resize=not self.options.arrival_preserve_resolution,
        )

    def _on_status(self, message: String) -> None:
        now = time.monotonic()
        if not self._due("status", now, self._status_period):
            return
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("status payload is not a JSON object")
            image = render_status_card(
                payload, self.options.status_width, self.options.status_height
            )
            preview = CompressedImage()
            preview.header.stamp = self.get_clock().now().to_msg()
            preview.header.frame_id = "navdp_operator_status"
            preview.format = "jpeg"
            preview.data = encode_jpeg(image, self.options.status_jpeg_quality)
            self._status_publisher.publish(preview)
        except (json.JSONDecodeError, ValueError, RuntimeError, cv2.error) as error:
            self._report_error("status", error)

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
    parser.add_argument("--status-input", required=True)
    parser.add_argument("--goal-output", required=True)
    parser.add_argument("--arrival-output", required=True)
    parser.add_argument("--status-output", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--rgb-fps", type=float, required=True)
    parser.add_argument("--depth-fps", type=float, required=True)
    parser.add_argument("--goal-fps", type=float, required=True)
    parser.add_argument("--arrival-fps", type=float, required=True)
    parser.add_argument("--status-width", type=int, required=True)
    parser.add_argument("--status-height", type=int, required=True)
    parser.add_argument("--status-fps", type=float, required=True)
    parser.add_argument("--rgb-jpeg-quality", type=int, required=True)
    parser.add_argument("--depth-jpeg-quality", type=int, required=True)
    parser.add_argument("--goal-jpeg-quality", type=int, required=True)
    parser.add_argument("--arrival-jpeg-quality", type=int, required=True)
    parser.add_argument("--status-jpeg-quality", type=int, required=True)
    parser.add_argument("--arrival-preserve-resolution", action="store_true")
    parser.add_argument("--depth-min-mm", type=float, required=True)
    parser.add_argument("--depth-max-mm", type=float, required=True)
    options = parser.parse_args()
    if (
        options.width <= 0
        or options.height <= 0
        or options.status_width < 480
        or options.status_height < 220
    ):
        parser.error("preview width and height must be positive")
    if any(
        value <= 0
        for value in (
            options.rgb_fps,
            options.depth_fps,
            options.goal_fps,
            options.arrival_fps,
            options.status_fps,
        )
    ):
        parser.error("preview frame rates must be positive")
    for label in (
        "rgb_jpeg_quality",
        "depth_jpeg_quality",
        "goal_jpeg_quality",
        "arrival_jpeg_quality",
        "status_jpeg_quality",
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
