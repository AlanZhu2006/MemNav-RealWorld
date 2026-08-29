#!/usr/bin/env python3
"""Persist NavDP status, CEC and arrival receipts as JSON Lines."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any, TextIO


TOPICS = {
    "/navdp/status": "status.jsonl",
    "/navdp/cec_receipt": "cec_receipt.jsonl",
    "/navdp/rgb_arrival_status": "rgb_arrival_status.jsonl",
    "/navdp/gt/status": "odin_gt_status.jsonl",
    "/navdp/experiment_event": "experiment_event.jsonl",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class ReceiptLogWriter:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir.resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._handles: dict[str, TextIO] = {}

    def append(self, topic: str, data: str) -> dict[str, Any]:
        if topic not in TOPICS:
            raise ValueError(f"unsupported receipt topic: {topic}")
        try:
            payload: Any = json.loads(data)
        except json.JSONDecodeError:
            payload = {"raw": data}
        row = {
            "received_utc": utc_now(),
            "received_monotonic_ns": time.monotonic_ns(),
            "topic": topic,
            "payload": payload,
        }
        handle = self._handles.get(topic)
        if handle is None:
            handle = (self.output_dir / TOPICS[topic]).open(
                "a", encoding="utf-8", buffering=1
            )
            self._handles[topic] = handle
        handle.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
        handle.flush()
        return row

    def close(self) -> None:
        for handle in self._handles.values():
            handle.close()
        self._handles.clear()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    writer = ReceiptLogWriter(args.output_dir)

    class ReceiptLoggerNode(Node):
        def __init__(self) -> None:
            super().__init__("navdp_experiment_receipt_logger")
            qos = QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            # Node.subscriptions is a read-only rclpy property.  Keep our own
            # references so subscriptions are not garbage-collected without
            # trying to overwrite that property.
            self._receipt_subscriptions = []
            for topic in TOPICS:
                self._receipt_subscriptions.append(
                    self.create_subscription(
                        String,
                        topic,
                        lambda message, source=topic: writer.append(source, message.data),
                        qos,
                    )
                )

    rclpy.init()
    node = ReceiptLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        writer.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
