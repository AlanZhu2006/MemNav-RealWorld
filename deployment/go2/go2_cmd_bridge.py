#!/usr/bin/env python3
"""Bridge NavDP ROS Twist commands to Unitree Go2 SportClient.Move()."""

from __future__ import annotations

import argparse
import math
import os
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from geometry_msgs.msg import Twist, Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

from go2_battery_monitor import battery_message, sample_from_low_state


@dataclass
class TimedTwist:
    stamp_sec: float
    msg: Twist


def clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


class Go2CmdBridge(Node):
    """Forward fresh bounded commands while yielding to the hand controller."""

    def __init__(self, sport_client) -> None:
        super().__init__("navdp_go2_cmd_bridge")
        self.sport_client = sport_client

        self.declare_parameter("cmd_vel_topic", "/navdp/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("timeout_sec", 0.35)
        self.declare_parameter("max_vx", 0.30)
        self.declare_parameter("max_vy", 0.0)
        self.declare_parameter("max_wz", 0.60)
        self.declare_parameter("x_sign", 1.0)
        self.declare_parameter("y_sign", 1.0)
        self.declare_parameter("wz_sign", 1.0)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("deadband_v", 0.01)
        self.declare_parameter("deadband_w", 0.02)
        # Forward the controller's output without an extra velocity floor.
        self.declare_parameter("min_cmd_v", 0.0)
        self.declare_parameter("min_cmd_w", 0.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("send_zero_when_idle", False)
        self.declare_parameter("stop_once_on_release", True)
        self.declare_parameter("remote_priority", True)
        self.declare_parameter("remote_topic", "rt/lowstate")
        self.declare_parameter("remote_deadband", 0.12)
        self.declare_parameter("remote_hold_sec", 0.8)
        self.declare_parameter("log_commands", True)
        self.declare_parameter("log_interval_sec", 0.5)
        self.declare_parameter("battery_topic", "/navdp/go2/battery")
        self.declare_parameter("battery_publish_rate_hz", 2.0)
        self.declare_parameter("battery_sample_rate_hz", 5.0)
        self.declare_parameter("battery_offline_timeout_s", 2.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)
        self.x_sign = float(self.get_parameter("x_sign").value)
        self.y_sign = float(self.get_parameter("y_sign").value)
        self.wz_sign = float(self.get_parameter("wz_sign").value)
        self.swap_xy = bool(self.get_parameter("swap_xy").value)
        self.deadband_v = float(self.get_parameter("deadband_v").value)
        self.deadband_w = float(self.get_parameter("deadband_w").value)
        self.min_cmd_v = abs(float(self.get_parameter("min_cmd_v").value))
        self.min_cmd_w = abs(float(self.get_parameter("min_cmd_w").value))
        self.enabled = bool(self.get_parameter("enabled").value)
        self.send_zero_when_idle = bool(self.get_parameter("send_zero_when_idle").value)
        self.stop_once_on_release = bool(self.get_parameter("stop_once_on_release").value)
        self.remote_priority = bool(self.get_parameter("remote_priority").value)
        self.remote_topic = str(self.get_parameter("remote_topic").value)
        self.remote_deadband = float(self.get_parameter("remote_deadband").value)
        self.remote_hold_sec = float(self.get_parameter("remote_hold_sec").value)
        self.log_commands = bool(self.get_parameter("log_commands").value)
        self.log_interval_sec = float(self.get_parameter("log_interval_sec").value)
        battery_publish_rate_hz = max(
            0.2, float(self.get_parameter("battery_publish_rate_hz").value)
        )
        battery_sample_rate_hz = max(
            battery_publish_rate_hz,
            float(self.get_parameter("battery_sample_rate_hz").value),
        )
        self.battery_offline_timeout_s = max(
            0.2, float(self.get_parameter("battery_offline_timeout_s").value)
        )
        self._battery_sample_period_s = 1.0 / battery_sample_rate_hz
        self._next_battery_sample_monotonic = 0.0
        self._battery_lock = threading.Lock()
        self._battery_sample = None
        self._heading_publisher = self.create_publisher(
            Vector3Stamped, "/navdp/go2/body_heading", 1
        )
        self._next_heading_sample_monotonic = 0.0
        battery_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._battery_publisher = self.create_publisher(
            BatteryState,
            str(self.get_parameter("battery_topic").value),
            battery_qos,
        )

        self.latest_cmd: Optional[TimedTwist] = None
        self.last_sent = (None, None, None)
        self.command_active = False
        self.latest_remote_stamp = 0.0
        self.remote_subscriber = None
        self.remote_takeover_logged = False
        self.last_command_log_stamp = 0.0

        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 1)
        self.create_timer(1.0 / max(1.0, self.rate_hz), self.publish_to_go2)
        self.create_timer(1.0 / battery_publish_rate_hz, self.publish_battery)
        self.setup_remote_priority()

        self.get_logger().info(
            "Go2 bridge ready: "
            f"{self.cmd_vel_topic} -> SportClient.Move(), "
            f"limits=({self.max_vx:.2f}, {self.max_vy:.2f}, {self.max_wz:.2f}), "
            f"timeout={self.timeout_sec:.2f}s, remote_priority={self.remote_priority}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = TimedTwist(self.now_sec(), msg)

    def setup_remote_priority(self) -> None:
        if not self.remote_priority:
            return
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

            self.remote_subscriber = ChannelSubscriber(self.remote_topic, LowState_)
            self.remote_subscriber.Init(self.on_low_state, 10)
            self.get_logger().info(
                f"Hand-controller priority: topic={self.remote_topic}, "
                f"deadband={self.remote_deadband:.2f}, hold={self.remote_hold_sec:.2f}s"
            )
        except Exception as exc:
            self.get_logger().warning(f"Hand-controller priority unavailable: {exc}")

    def on_low_state(self, msg) -> None:
        now = time.monotonic()
        if now >= self._next_heading_sample_monotonic:
            self._next_heading_sample_monotonic = now + 0.02
            try:
                yaw = float(msg.imu_state.rpy[2])
                if math.isfinite(yaw):
                    heading = Vector3Stamped()
                    heading.header.stamp = self.get_clock().now().to_msg()
                    heading.header.frame_id = "go2_imu_world"
                    heading.vector.z = yaw
                    self._heading_publisher.publish(heading)
            except (AttributeError, IndexError, TypeError, ValueError):
                pass
        if now >= self._next_battery_sample_monotonic:
            self._next_battery_sample_monotonic = (
                now + self._battery_sample_period_s
            )
            sample = sample_from_low_state(msg, received_monotonic=now)
            with self._battery_lock:
                self._battery_sample = sample
        try:
            data = bytes(msg.wireless_remote)
            if len(data) < 24:
                return
            axes = (
                struct.unpack("<f", data[4:8])[0],
                struct.unpack("<f", data[8:12])[0],
                struct.unpack("<f", data[12:16])[0],
                struct.unpack("<f", data[20:24])[0],
            )
            if max(abs(axis) for axis in axes) > self.remote_deadband:
                self.latest_remote_stamp = time.monotonic()
        except Exception:
            return

    def publish_battery(self) -> None:
        with self._battery_lock:
            sample = self._battery_sample
        message = battery_message(
            sample,
            now_monotonic=time.monotonic(),
            offline_timeout_s=self.battery_offline_timeout_s,
        )
        message.header.stamp = self.get_clock().now().to_msg()
        self._battery_publisher.publish(message)

    def remote_is_active(self) -> bool:
        return self.remote_priority and (
            time.monotonic() - self.latest_remote_stamp
        ) <= self.remote_hold_sec

    @staticmethod
    def apply_floor(value: float, floor: float) -> float:
        if value == 0.0 or floor <= 0.0 or abs(value) >= floor:
            return value
        return floor if value > 0.0 else -floor

    def command_from_latest(self) -> Optional[tuple[float, float, float]]:
        if self.latest_cmd is None:
            return None
        if self.now_sec() - self.latest_cmd.stamp_sec > self.timeout_sec:
            return None

        msg = self.latest_cmd.msg
        raw_x = float(msg.linear.x)
        raw_y = float(msg.linear.y)
        if self.swap_xy:
            raw_x, raw_y = raw_y, raw_x

        vx = clamp(self.x_sign * raw_x, self.max_vx)
        vy = clamp(self.y_sign * raw_y, self.max_vy)
        wz = clamp(self.wz_sign * float(msg.angular.z), self.max_wz)
        if abs(vx) < self.deadband_v:
            vx = 0.0
        if abs(vy) < self.deadband_v:
            vy = 0.0
        if abs(wz) < self.deadband_w:
            wz = 0.0
        return (
            self.apply_floor(vx, self.min_cmd_v),
            self.apply_floor(vy, self.min_cmd_v),
            self.apply_floor(wz, self.min_cmd_w),
        )

    @staticmethod
    def is_zero_command(vx: float, vy: float, wz: float) -> bool:
        return vx == 0.0 and vy == 0.0 and wz == 0.0

    def release_control(self, reason: str, *, normal_pause: bool = False) -> None:
        if not self.command_active and not self.send_zero_when_idle:
            return
        try:
            self.sport_client.Move(0.0, 0.0, 0.0)
            if self.stop_once_on_release and not normal_pause:
                self.sport_client.StopMove()
            self.last_sent = (0.0, 0.0, 0.0)
            if self.command_active:
                self.get_logger().info(
                    f"Released Go2 control after {reason}; hand controller can take over"
                )
            self.command_active = False
        except Exception as exc:
            self.get_logger().error(f"SportClient stop failed: {exc}")

    def publish_to_go2(self) -> None:
        enabled = bool(self.get_parameter("enabled").value)
        if enabled != self.enabled:
            self.enabled = enabled
            self.get_logger().info(f"Go2 bridge {'enabled' if enabled else 'disabled'}")
        if not self.enabled:
            self.release_control("bridge disabled")
            return

        if self.remote_is_active():
            if not self.remote_takeover_logged:
                self.get_logger().info("Hand controller active; holding NavDP commands")
                self.remote_takeover_logged = True
            self.release_control("hand controller activity")
            return
        self.remote_takeover_logged = False

        command = self.command_from_latest()
        if command is None:
            self.release_control("cmd_vel timeout")
            return
        vx, vy, wz = command
        if self.is_zero_command(vx, vy, wz) and not self.send_zero_when_idle:
            self.release_control("zero cmd_vel", normal_pause=True)
            return

        try:
            self.sport_client.Move(vx, vy, wz)
            now = time.monotonic()
            if self.log_commands and now - self.last_command_log_stamp >= self.log_interval_sec:
                self.get_logger().info(f"Move(vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f})")
                self.last_command_log_stamp = now
            self.last_sent = (vx, vy, wz)
            self.command_active = not self.is_zero_command(vx, vy, wz)
        except Exception as exc:
            self.get_logger().error(f"SportClient.Move failed: {exc}")

    def stop_robot(self) -> None:
        try:
            self.sport_client.Move(0.0, 0.0, 0.0)
            self.sport_client.StopMove()
        except Exception as exc:
            self.get_logger().warning(f"Failed to stop Go2 cleanly: {exc}")


def import_unitree_sdk(sdk_path: str):
    if sdk_path:
        path = Path(sdk_path).expanduser()
        if path.exists():
            sys.path.insert(0, str(path))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    return ChannelFactoryInitialize, SportClient


def main() -> None:
    parser = argparse.ArgumentParser(description="NavDP Twist to Unitree Go2 bridge")
    parser.add_argument("--net-if", default=os.environ.get("UNITREE_NET_IF", "eth0"))
    parser.add_argument(
        "--sdk-path",
        default=os.environ.get(
            "UNITREE_SDK2PY_PATH",
            "/home/unitree/.local/share/memnav/unitree_ws/src/unitree_sdk2_python",
        ),
    )
    args, ros_args = parser.parse_known_args()

    channel_factory_initialize, sport_client_type = import_unitree_sdk(args.sdk_path)
    channel_factory_initialize(0, args.net_if)
    sport_client = sport_client_type()
    # Safety-release RPCs must not freeze the ROS executor for ten seconds.
    # Ordinary plan pauses use the nonblocking Move(0, 0, 0) API only.
    sport_client.SetTimeout(0.20)
    sport_client.Init()

    rclpy.init(args=ros_args)
    node = Go2CmdBridge(sport_client)

    def handle_signal(_signum, _frame) -> None:
        node.stop_robot()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
