#!/usr/bin/env python3
"""Publish observation-only Unitree Go2 battery telemetry to ROS 2."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState


@dataclass(frozen=True)
class Go2BatterySample:
    received_monotonic: float
    soc_pct: Optional[float]
    voltage_v: Optional[float]
    current_a: Optional[float]
    cell_voltage_v: tuple[float, ...]


def _finite(value) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def sample_from_low_state(
    message, *, received_monotonic: Optional[float] = None
) -> Go2BatterySample:
    """Extract only documented battery fields from one Unitree LowState."""

    bms = getattr(message, "bms_state", None)
    soc = _finite(getattr(bms, "soc", None))
    if soc is None or not 0.0 <= soc <= 100.0:
        soc = None
    voltage = _finite(getattr(message, "power_v", None))
    if voltage is not None and voltage <= 0.0:
        voltage = None
    current = _finite(getattr(message, "power_a", None))
    cells = []
    for raw in getattr(bms, "cell_vol", ()):
        millivolts = _finite(raw)
        if millivolts is not None and millivolts > 0.0:
            cells.append(millivolts * 0.001)
    return Go2BatterySample(
        received_monotonic=(
            time.monotonic()
            if received_monotonic is None
            else float(received_monotonic)
        ),
        soc_pct=soc,
        voltage_v=voltage,
        current_a=current,
        cell_voltage_v=tuple(cells),
    )


def battery_message(
    sample: Optional[Go2BatterySample],
    *,
    now_monotonic: float,
    offline_timeout_s: float,
) -> BatteryState:
    """Build a live BatteryState or an explicit non-stale OFFLINE state."""

    message = BatteryState()
    message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
    message.location = "unitree_go2"
    message.design_capacity = math.nan
    message.capacity = math.nan
    message.charge = math.nan
    online = sample is not None and (
        now_monotonic - sample.received_monotonic
    ) <= offline_timeout_s
    message.present = bool(online)
    if not online:
        message.voltage = math.nan
        message.current = math.nan
        message.percentage = math.nan
        message.cell_voltage = []
        return message
    assert sample is not None
    message.voltage = (
        math.nan if sample.voltage_v is None else sample.voltage_v
    )
    message.current = (
        math.nan if sample.current_a is None else sample.current_a
    )
    message.percentage = (
        math.nan if sample.soc_pct is None else sample.soc_pct * 0.01
    )
    message.cell_voltage = list(sample.cell_voltage_v)
    return message


class Go2BatteryMonitor(Node):
    def __init__(self) -> None:
        super().__init__("navdp_go2_battery_monitor")
        self.declare_parameter("battery_topic", "/navdp/go2/battery")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("offline_timeout_s", 2.0)
        self.battery_topic = str(self.get_parameter("battery_topic").value)
        publish_rate_hz = max(
            0.2, float(self.get_parameter("publish_rate_hz").value)
        )
        self.offline_timeout_s = max(
            0.2, float(self.get_parameter("offline_timeout_s").value)
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            BatteryState, self.battery_topic, qos
        )
        self._lock = threading.Lock()
        self._sample: Optional[Go2BatterySample] = None
        self._online = False
        self._dds_subscriber = None
        self.create_timer(1.0 / publish_rate_hz, self._publish)

    def attach_dds_subscriber(self, subscriber) -> None:
        self._dds_subscriber = subscriber

    def on_low_state(self, message) -> None:
        sample = sample_from_low_state(message)
        with self._lock:
            self._sample = sample

    def _publish(self) -> None:
        with self._lock:
            sample = self._sample
        message = battery_message(
            sample,
            now_monotonic=time.monotonic(),
            offline_timeout_s=self.offline_timeout_s,
        )
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)
        if message.present != self._online:
            self._online = bool(message.present)
            log = (
                self.get_logger().info
                if self._online
                else self.get_logger().warning
            )
            log(f"Go2 battery telemetry {'ONLINE' if self._online else 'OFFLINE'}")


def import_unitree_sdk(sdk_path: str):
    if sdk_path:
        path = Path(sdk_path).expanduser()
        if path.exists():
            sys.path.insert(0, str(path))
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

    return ChannelFactoryInitialize, ChannelSubscriber, LowState_


def network_link_ready(
    net_if: str, *, sys_class_net: Path = Path("/sys/class/net")
) -> bool:
    """Return true only when the configured wired interface has carrier."""

    interface = sys_class_net / net_if
    if not interface.exists():
        return False
    carrier = interface / "carrier"
    try:
        return carrier.read_text(encoding="utf-8").strip() == "1"
    except OSError:
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--net-if", default=os.environ.get("UNITREE_NET_IF", "eth0"))
    parser.add_argument(
        "--sdk-path",
        default=os.environ.get(
            "UNITREE_SDK2PY_PATH",
            "/home/unitree/.local/share/memnav/unitree_ws/src/unitree_sdk2_python",
        ),
    )
    parser.add_argument("--dds-topic", default="rt/lowstate")
    args, ros_args = parser.parse_known_args()

    channel_init, subscriber_type, low_state_type = import_unitree_sdk(args.sdk_path)
    rclpy.init(args=ros_args)
    node = Go2BatteryMonitor()
    connector_stop = threading.Event()

    def connect_dds() -> None:
        last_warning = 0.0
        while rclpy.ok() and not connector_stop.is_set():
            if not network_link_ready(args.net_if):
                now = time.monotonic()
                if now - last_warning >= 15.0:
                    node.get_logger().warning(
                        f"Unitree network link {args.net_if} is offline; "
                        "publishing BATTERY OFFLINE and waiting"
                    )
                    last_warning = now
                connector_stop.wait(2.0)
                continue
            try:
                channel_init(0, args.net_if)
                subscriber = subscriber_type(args.dds_topic, low_state_type)
                subscriber.Init(node.on_low_state, 10)
                node.attach_dds_subscriber(subscriber)
                node.get_logger().info(
                    f"Unitree battery DDS connected: {args.dds_topic} on {args.net_if}"
                )
                return
            except Exception as error:
                now = time.monotonic()
                if now - last_warning >= 15.0:
                    node.get_logger().warning(
                        "Unitree battery DDS unavailable; publishing BATTERY "
                        f"OFFLINE and retrying: {type(error).__name__}: {error}"
                    )
                    last_warning = now
                connector_stop.wait(2.0)

    connector = threading.Thread(
        target=connect_dds,
        name="go2-battery-dds-connect",
        daemon=True,
    )
    connector.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        connector_stop.set()
        connector.join(timeout=2.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
