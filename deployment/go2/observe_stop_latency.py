#!/usr/bin/env python3
"""Observe real Go2 STOP delivery on an isolated ROS domain, with motion disabled.

Uses the production operator service and Go2 bridge, not synthetic callbacks.
Only zero Move / StopMove may reach the SDK. Does not start Survey or navigation.
The result measures unloaded software delivery, not physical braking distance.
"""

import argparse
import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from go2_cmd_bridge import Go2CmdBridge, import_unitree_sdk
from revisit_operator_service import RevisitOperatorService


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    system = json.loads((repo / "deployment/config/system.json").read_text())
    unitree = system["sites"]["jetson"]["unitree"]
    initialize, client_type = import_unitree_sdk(unitree["sdk_python_path"])
    initialize(0, unitree["network_interface"])
    client = client_type()
    client.SetTimeout(0.20)
    client.Init()
    rclpy.init(domain_id=73, signal_handler_options=SignalHandlerOptions.NO,
               args=["--ros-args", "-p", "enabled:=false", "-p", "max_vx:=0.0",
                     "-p", "max_vy:=0.0", "-p", "max_wz:=0.0"])
    bridge = Go2CmdBridge(client)
    operator = RevisitOperatorService(
        repo_root=repo, state_path=output / "unused-state.json",
        episodes_root=output / "episodes", capture_root=output / "capture",
        capture_session_prefix="stop-observation", rgb_topic="/unused/rgb",
        depth_topic="/unused/depth", timeout_s=30., robot_ip="192.168.123.161")
    observer = rclpy.create_node("memnav_stop_latency_observer")
    readings = []
    observer.create_subscription(
        String, "/navdp/go2/motion_stop",
        lambda msg: readings.append((time.monotonic(), json.loads(msg.data))),
        qos_profile_sensor_data)
    stop = observer.create_client(Trigger, "/memnav_operator/operator_stop")
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (bridge, operator.node, observer):
        executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        if not stop.wait_for_service(timeout_sec=8.):
            raise RuntimeError("isolated operator service was not discovered")
        deadline = time.monotonic() + 8.
        while operator.fast_stop_pub.get_subscription_count() < 1:
            if time.monotonic() > deadline:
                raise RuntimeError("isolated stop lane was not discovered")
            time.sleep(0.02)
        started = time.monotonic()
        future = stop.call_async(Trigger.Request())
        while time.monotonic() - started < 5.:
            if readings and future.done():
                break
            time.sleep(0.005)
        valid = [(at, p) for at, p in readings if p.get("zero_command_sent") is True]
        if not valid or not future.done() or not future.result().success:
            raise RuntimeError("STOP did not receive a successful zero-command receipt")
        result = {"service_to_zero_receipt_s": valid[0][0] - started,
                  "receipt": valid[0][1], "motion_enabled": bridge.enabled,
                  "velocity_limits": [bridge.max_vx, bridge.max_vy, bridge.max_wz],
                  "ros_domain": 73, "robot_motion_test": False,
                  "full_navigation_load_test": False}
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
    finally:
        executor.shutdown(timeout_sec=2.)
        worker.join(timeout=2.)
        operator.close()
        bridge.stop_robot()
        for name in ("remote_subscriber", "position_subscriber"):
            subscriber = getattr(bridge, name, None)
            if subscriber is not None:
                subscriber.Close()
        bridge.destroy_node()
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
