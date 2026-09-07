#!/usr/bin/env python3
"""Request operator_stop and confirm fresh locked status with one DDS node."""

import argparse
import json
import math
import os
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-s", type=float, default=8.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or not 0 < args.timeout_s <= 30:
        parser.error("timeout must be in (0, 30] seconds")

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init(args=[])
    node = rclpy.create_node(f"memnav_lock_waiter_{os.getpid()}",
                            enable_rosout=False, start_parameter_services=False)
    started = time.monotonic()
    deadline = started + args.timeout_s
    latest = None
    received_at = 0.0
    acknowledged_at = None
    future = None

    def on_status(message):
        nonlocal latest, received_at
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            latest = payload
            received_at = time.monotonic()

    # VOLATILE rejects a retained pre-stop status. Require a new sample after
    # the stop response as well; a successful service call alone is not proof.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(String, "/navdp/status", on_status, qos)
    client = node.create_client(Trigger, "/navdp_go2_adapter/operator_stop")
    issue = "operator_stop service unavailable"
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if future is None and client.service_is_ready():
                future = client.call_async(Trigger.Request())
                issue = "waiting for operator_stop response"
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0., deadline-time.monotonic())))
            if future is not None and future.done() and acknowledged_at is None:
                response = future.result()
                if response is None or not response.success:
                    raise RuntimeError("operator_stop was rejected")
                acknowledged_at = time.monotonic()
                issue = "waiting for fresh disabled/estop/zero status"
            if acknowledged_at is None or received_at <= acknowledged_at or latest is None:
                continue
            velocities = (latest.get("cmd_vx"), latest.get("cmd_wz"))
            zero = all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       and math.isfinite(v) and abs(v) < 1e-3 for v in velocities)
            if latest.get("enabled") is False and latest.get("estop") is True and zero:
                print(json.dumps({"motion_locked": True,
                                  "elapsed_s": round(time.monotonic()-started, 3),
                                  "enabled": False, "estop": True,
                                  "cmd_vx": velocities[0], "cmd_wz": velocities[1]}))
                return 0
        raise RuntimeError(issue)
    except Exception as error:
        print(f"Motion lock not confirmed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
