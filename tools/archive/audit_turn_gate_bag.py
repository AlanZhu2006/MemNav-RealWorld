#!/usr/bin/env python3
"""Offline audit of NavDP-to-Go2 turn gating from a ROS 2 bag.

This reader never republishes bag topics.  It compares the controller used in
an experiment with the validated TinyNav real-robot contract: 0.30 m/s forward
limit and an 8-degree heading deadband before the Go2 0.20 rad/s yaw floor.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from deployment.go2.trajectory_control import (
    ControllerConfig,
    trajectory_to_command,
)


def _sign_flips(values: list[float], epsilon: float = 1e-9) -> int:
    signs = [1 if value > 0.0 else -1 for value in values if abs(value) > epsilon]
    return sum(first != second for first, second in zip(signs, signs[1:]))


def _bridge_yaw(value: float, deadband: float = 0.02, floor: float = 0.20) -> float:
    if abs(value) < deadband:
        return 0.0
    if abs(value) < floor:
        return math.copysign(floor, value)
    return float(value)


def audit_bag(bag_path: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {
        item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
    }
    relevant = {"/navdp/trajectory", "/navdp/cmd_vel", "/navdp/status"}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(relevant)))

    actual_wz: list[float] = []
    all_actual_wz: list[float] = []
    headings: list[float] = []
    old_wz: list[float] = []
    corrected_wz: list[float] = []
    corrected_vx: list[float] = []
    active = False

    old_config = ControllerConfig(
        max_linear_mps=0.15,
        max_angular_rps=0.35,
        heading_deadband_rad=0.0,
    )
    corrected_config = ControllerConfig(
        max_linear_mps=0.30,
        max_angular_rps=0.55,
        heading_deadband_rad=math.radians(8.0),
    )

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, topic_types[topic])
        if topic == "/navdp/status":
            try:
                status = json.loads(message.data)
                active = bool(status.get("enabled")) and not bool(status.get("estop"))
            except (TypeError, ValueError, json.JSONDecodeError):
                active = False
            continue
        if topic == "/navdp/cmd_vel":
            value = float(message.angular.z)
            all_actual_wz.append(value)
            if active:
                actual_wz.append(value)
            continue
        if not active:
            continue
        trajectory = np.asarray(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in message.poses
            ],
            dtype=np.float64,
        )
        if trajectory.ndim != 2 or trajectory.shape[0] == 0:
            continue
        old = trajectory_to_command(trajectory, old_config)
        corrected = trajectory_to_command(trajectory, corrected_config)
        headings.append(math.atan2(corrected.target_y, corrected.target_x))
        old_wz.append(old.angular_z)
        corrected_wz.append(corrected.angular_z)
        corrected_vx.append(corrected.linear_x)

    actual_nonzero = [value for value in actual_wz if abs(value) >= 0.02]
    actual_below_floor = [value for value in actual_nonzero if abs(value) < 0.20]
    corrected_bridge_wz = [_bridge_yaw(value) for value in corrected_wz]
    deadband = math.radians(8.0)
    return {
        "bag": str(bag_path),
        "contract": {
            "max_linear_mps": 0.30,
            "heading_deadband_deg": 8.0,
            "go2_min_cmd_v_mps": 0.10,
            "go2_min_cmd_w_radps": 0.20,
        },
        "recorded_cmd": {
            "all_samples": len(all_actual_wz),
            "active_samples": len(actual_wz),
            "samples": len(actual_wz),
            "nonzero_turn_samples": len(actual_nonzero),
            "below_go2_yaw_floor_samples": len(actual_below_floor),
            "below_go2_yaw_floor_fraction": (
                len(actual_below_floor) / len(actual_nonzero) if actual_nonzero else 0.0
            ),
            "raw_sign_flips": _sign_flips(actual_wz, epsilon=0.02),
            "hard_floor_sign_flips": _sign_flips(
                [_bridge_yaw(value) for value in actual_wz]
            ),
        },
        "path_updates": {
            "count": len(headings),
            "within_8deg": sum(abs(value) < deadband for value in headings),
            "old_nonzero_turns": sum(abs(value) > 0.0 for value in old_wz),
            "corrected_nonzero_turns": sum(abs(value) > 0.0 for value in corrected_wz),
            "old_sign_flips": _sign_flips(old_wz),
            "corrected_sign_flips": _sign_flips(corrected_wz),
            "corrected_bridge_sign_flips": _sign_flips(corrected_bridge_wz),
            "corrected_median_vx_mps": (
                float(np.median(corrected_vx)) if corrected_vx else 0.0
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit_bag(args.bag), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
