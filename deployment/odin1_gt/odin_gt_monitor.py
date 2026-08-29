#!/usr/bin/env python3
"""Capture Odin goal anchors and monitor independent formal-run GT evidence."""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import time
from typing import Any, Optional

import numpy as np
import yaml

from odin_gt_core import (
    ArrivalGate,
    PathAccumulator,
    Pose2D,
    RelocalizationGate,
    compose_pose,
    inverse_pose,
    quaternion_to_yaw,
    sha256_file,
    wrap_angle,
)


GOAL_DRAFT_SCHEMA = "memnav-odin1-goal-anchor-draft-v1"
GOAL_SCHEMA = "memnav-odin1-goal-anchor-v1"
STATUS_SCHEMA = "memnav-odin1-gt-status-v1"
RESULT_SCHEMA = "memnav-odin1-gt-result-v1"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def pose_payload(pose: Pose2D) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "yaw_rad": pose.yaw}


def pose_from_payload(payload: dict[str, Any]) -> Pose2D:
    return Pose2D(
        x=float(payload["x"]),
        y=float(payload["y"]),
        yaw=float(payload["yaw_rad"]),
    )


def ros_stamp_s(message: Any) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def message_pose(message: Any) -> Pose2D:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return Pose2D(
        x=float(position.x),
        y=float(position.y),
        yaw=quaternion_to_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ),
    )


def transform_pose(transform: Any) -> Pose2D:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return Pose2D(
        x=float(translation.x),
        y=float(translation.y),
        yaw=quaternion_to_yaw(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        ),
    )


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def file_receipt(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve()
    if not path.is_file() or path.stat().st_size <= 0:
        raise ValueError(f"required artifact is missing or empty: {path}")
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def verified_receipt_path(receipt: dict[str, Any], label: str) -> Path:
    path = Path(receipt.get("path", "")).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"sealed {label} is missing: {path}")
    if path.stat().st_size != int(receipt.get("bytes", -1)):
        raise ValueError(f"sealed {label} size changed")
    if sha256_file(path) != receipt.get("sha256"):
        raise ValueError(f"sealed {label} SHA changed")
    return path


def driver_profile_semantics(payload: dict[str, Any]) -> dict[str, Any]:
    return {
        "profile": payload.get("profile"),
        "repository": payload.get("repository"),
        "commit": payload.get("commit"),
        "tag": payload.get("tag"),
        "firmware_contract": payload.get("firmware_contract"),
        "native_mode1": payload.get("native_mode1"),
        "patches": [
            {"name": item.get("name"), "sha256": item.get("sha256")}
            for item in payload.get("patches", [])
        ],
        "modified_files": payload.get("modified_files"),
    }


def circular_mean(values: list[float]) -> float:
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


def capture_goal(args: argparse.Namespace) -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    goal_rgb = file_receipt(args.goal_rgb)
    goal_depth = file_receipt(args.goal_depth) if args.goal_depth else None
    samples: deque[tuple[float, Pose2D, float]] = deque(maxlen=args.samples)

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("memnav_odin1_goal_anchor_capture")
            self.frame_error = ""
            self.create_subscription(
                Odometry,
                args.odometry_topic,
                self.on_odometry,
                qos_profile_sensor_data,
            )

        def on_odometry(self, message: Odometry) -> None:
            parent = message.header.frame_id.lstrip("/")
            if parent != args.map_frame.lstrip("/"):
                self.frame_error = f"unexpected_odometry_frame:{parent}"
                return
            linear = message.twist.twist.linear
            speed_mps = math.sqrt(
                float(linear.x) ** 2 + float(linear.y) ** 2 + float(linear.z) ** 2
            )
            if speed_mps > args.maximum_speed_mps:
                samples.clear()
                return
            samples.append((ros_stamp_s(message), message_pose(message), speed_mps))

    rclpy.init()
    node = CaptureNode()
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and len(samples) < args.samples:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.frame_error:
                raise RuntimeError(node.frame_error)
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"only {len(samples)}/{args.samples} stationary Odin samples arrived"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    poses = [sample[1] for sample in samples]
    goal_pose = Pose2D(
        x=float(np.median([pose.x for pose in poses])),
        y=float(np.median([pose.y for pose in poses])),
        yaw=circular_mean([pose.yaw for pose in poses]),
    )
    payload = {
        "schema": GOAL_DRAFT_SCHEMA,
        "created_utc": utc_now(),
        "mapping_session_id": args.mapping_session_id,
        "mapping_mode_contract": "vendor_mode_1_map_frame_equals_startup_odom",
        "map_frame": args.map_frame,
        "base_frame": args.base_frame,
        "goal_pose_map": pose_payload(goal_pose),
        "stationary_capture": {
            "samples": len(samples),
            "first_stamp_s": samples[0][0],
            "last_stamp_s": samples[-1][0],
            "maximum_speed_mps": args.maximum_speed_mps,
        },
        "d435i_goal_rgb": goal_rgb,
        "d435i_goal_depth": goal_depth,
        "policy_input": False,
        "motion_authority": False,
        "classification": "independent_reference_anchor_draft",
    }
    atomic_write_json(args.output, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


def seal_goal(args: argparse.Namespace) -> int:
    draft = load_json(args.draft)
    if draft.get("schema") != GOAL_DRAFT_SCHEMA:
        raise ValueError("goal draft has an unsupported schema")
    map_receipt = file_receipt(args.map_file)
    occupancy_yaml = file_receipt(args.occupancy_yaml)
    occupancy_payload = yaml.safe_load(
        Path(occupancy_yaml["path"]).read_text(encoding="utf-8")
    )
    occupancy_image_path = Path(occupancy_payload["image"])
    if not occupancy_image_path.is_absolute():
        occupancy_image_path = Path(occupancy_yaml["path"]).parent / occupancy_image_path
    occupancy_image = file_receipt(occupancy_image_path)
    occupancy_receipt_path = Path(occupancy_yaml["path"]).with_suffix(
        ".receipt.json"
    )
    occupancy_receipt = load_json(occupancy_receipt_path)
    if occupancy_receipt.get("schema") != "memnav-odin1-occupancy-v1":
        raise ValueError("occupancy receipt has an unsupported schema")
    if occupancy_receipt.get("session_id") != draft.get("mapping_session_id"):
        raise ValueError("goal anchor and occupancy map use different mapping sessions")
    scene_contract_path = args.scene_contract.expanduser().resolve()
    scene_contract = load_json(scene_contract_path)
    if scene_contract.get("schema") != "memnav-odin1-scene-contract-v1":
        raise ValueError("scene contract has an unsupported schema")
    if scene_contract.get("mapping_session_id") != draft.get("mapping_session_id"):
        raise ValueError("goal anchor and scene contract use different mapping sessions")
    if scene_contract.get("mount", {}).get("validated") is not True:
        raise ValueError("the Odin-to-Go2 mount receipt is not independently validated")
    if (
        occupancy_receipt.get("grid", {}).get("yaml_sha256")
        != occupancy_yaml["sha256"]
        or occupancy_receipt.get("grid", {}).get("pgm_sha256")
        != occupancy_image["sha256"]
    ):
        raise ValueError("occupancy files do not match their builder receipt")
    rgb = draft["d435i_goal_rgb"]
    if sha256_file(Path(rgb["path"])) != rgb["sha256"]:
        raise ValueError("D435i goal RGB changed after anchor capture")
    depth = draft.get("d435i_goal_depth")
    if depth and sha256_file(Path(depth["path"])) != depth["sha256"]:
        raise ValueError("D435i goal depth changed after anchor capture")
    payload = {
        **draft,
        "schema": GOAL_SCHEMA,
        "sealed_utc": utc_now(),
        "classification": "independent_reference_anchor_hash_sealed",
        "odin_map": map_receipt,
        "occupancy_yaml": occupancy_yaml,
        "occupancy_image": occupancy_image,
        "occupancy_receipt": file_receipt(occupancy_receipt_path),
        "scene_contract": file_receipt(scene_contract_path),
        "sensor_serial": scene_contract["sensor_serial"],
        "calibration_sha256": scene_contract["calibration"]["sha256"],
        "mount_receipt_sha256": scene_contract["mount"]["sha256"],
    }
    atomic_write_json(args.output, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


class FormalMonitor:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self.args = args
        self.goal_receipt_path = args.goal_receipt.expanduser().resolve()
        self.goal_receipt = load_json(self.goal_receipt_path)
        if self.goal_receipt.get("schema") != GOAL_SCHEMA:
            raise ValueError("formal GT requires a sealed Odin goal receipt")
        scene_contract_receipt = self.goal_receipt.get("scene_contract", {})
        scene_contract_path = verified_receipt_path(
            scene_contract_receipt, "Odin scene contract"
        )
        self.scene_contract = load_json(scene_contract_path)
        if self.scene_contract.get("schema") != "memnav-odin1-scene-contract-v1":
            raise ValueError("formal GT requires a sealed Odin scene contract")
        if self.scene_contract.get("mount", {}).get("validated") is not True:
            raise ValueError("formal GT requires a validated Odin-to-Go2 mount")
        verified_receipt_path(
            self.scene_contract.get("calibration", {}), "Odin calibration"
        )
        verified_receipt_path(
            self.scene_contract.get("mount", {}), "Odin-to-Go2 mount receipt"
        )
        sealed_driver_path = verified_receipt_path(
            self.scene_contract.get("driver_profile", {}),
            "Odin mapping driver profile",
        )
        sealed_driver = load_json(sealed_driver_path)
        current_driver_path = args.driver_profile_receipt.expanduser().resolve()
        current_driver = load_json(current_driver_path)
        if current_driver.get("schema") != "memnav-odin1-driver-profile-v1":
            raise ValueError("formal GT requires a valid current driver profile")
        if driver_profile_semantics(current_driver) != driver_profile_semantics(
            sealed_driver
        ):
            raise ValueError(
                "formal Odin driver profile differs from the mapping profile"
            )
        self.current_driver_receipt = file_receipt(current_driver_path)
        self.map_path = args.map_file.expanduser().resolve()
        actual_map_sha = sha256_file(self.map_path)
        expected_map_sha = self.goal_receipt["odin_map"]["sha256"]
        if actual_map_sha != expected_map_sha:
            raise ValueError(
                f"Odin map SHA mismatch: expected {expected_map_sha}, got {actual_map_sha}"
            )
        rgb_receipt = self.goal_receipt["d435i_goal_rgb"]
        if sha256_file(Path(rgb_receipt["path"])) != rgb_receipt["sha256"]:
            raise ValueError("D435i goal RGB no longer matches the sealed goal receipt")
        self.target_pose = pose_from_payload(self.goal_receipt["goal_pose_map"])
        self.output_dir = args.output_dir.expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.status_log = (self.output_dir / "status.jsonl").open(
            "a", encoding="utf-8", buffering=1
        )
        self.result_path = self.output_dir / "result.json"
        self.relocalization = RelocalizationGate(
            hold_s=args.relocalization_hold_s,
            minimum_samples=args.relocalization_minimum_samples,
            max_translation_change_m=args.maximum_tf_translation_change_m,
            max_rotation_change_rad=math.radians(args.maximum_tf_rotation_change_deg),
        )
        self.path = PathAccumulator(
            max_step_m=args.maximum_odometry_step_m,
            max_inferred_speed_mps=args.maximum_inferred_speed_mps,
        )
        self.arrival = ArrivalGate(
            distance_m=args.arrival_distance_m,
            speed_mps=args.arrival_speed_mps,
            hold_s=args.arrival_hold_s,
        )
        self.latest_odom_pose: Optional[Pose2D] = None
        self.latest_odom_stamp_s: Optional[float] = None
        self.latest_odom_monotonic_s = 0.0
        self.latest_speed_mps = math.inf
        self.latest_arrival_monotonic_s = 0.0
        self.latest_arrival_payload: dict[str, Any] = {}
        self.episode_started = False
        self.episode_started_utc: Optional[str] = None
        self.start_pose_map: Optional[Pose2D] = None
        self.final_result: Optional[dict[str, Any]] = None
        self.node = Node("memnav_odin1_gt_monitor")
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.node.create_publisher(String, args.status_topic, qos)
        self.string_type = String
        self.node.create_subscription(
            Odometry,
            args.odometry_topic,
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            TFMessage, "/tf", self.on_tf, qos_profile_sensor_data
        )
        self.node.create_subscription(
            TFMessage, "/tf_static", self.on_tf, qos_profile_sensor_data
        )
        self.node.create_subscription(
            String, args.arrival_status_topic, self.on_arrival_status, qos
        )
        self.node.create_timer(1.0 / args.publish_rate_hz, self.tick)

    def on_odometry(self, message: Any) -> None:
        parent = message.header.frame_id.lstrip("/")
        if parent != self.args.odom_frame.lstrip("/"):
            self.path.invalid_reason = f"unexpected_odometry_frame:{parent}"
            return
        pose = message_pose(message)
        linear = message.twist.twist.linear
        speed_mps = math.sqrt(
            float(linear.x) ** 2 + float(linear.y) ** 2 + float(linear.z) ** 2
        )
        stamp_s = ros_stamp_s(message)
        self.latest_odom_pose = pose
        self.latest_odom_stamp_s = stamp_s
        self.latest_odom_monotonic_s = time.monotonic()
        self.latest_speed_mps = speed_mps
        if self.episode_started:
            self.path.update(stamp_s, pose)

    def on_tf(self, message: Any) -> None:
        now_s = time.monotonic()
        map_frame = self.args.map_frame.lstrip("/")
        odom_frame = self.args.odom_frame.lstrip("/")
        for stamped in message.transforms:
            parent = stamped.header.frame_id.lstrip("/")
            child = stamped.child_frame_id.lstrip("/")
            if parent == map_frame and child == odom_frame:
                self.relocalization.update(now_s, transform_pose(stamped))
            elif parent == odom_frame and child == map_frame:
                self.relocalization.update(
                    now_s, inverse_pose(transform_pose(stamped))
                )

    def on_arrival_status(self, message: Any) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.latest_arrival_payload = payload
            self.latest_arrival_monotonic_s = time.monotonic()

    def rgb_arrival_confirmed(self, now_s: float) -> bool:
        return bool(
            now_s - self.latest_arrival_monotonic_s
            <= self.args.arrival_status_timeout_s
            and self.latest_arrival_payload.get("schema")
            == "navdp_rgb_arrival_v1"
            and self.latest_arrival_payload.get("arrival_latched") is True
        )

    def current_map_pose(self) -> Optional[Pose2D]:
        if self.latest_odom_pose is None or self.relocalization.latest is None:
            return None
        return compose_pose(self.relocalization.latest, self.latest_odom_pose)

    def reference_ready(self, now_s: float) -> bool:
        return bool(
            self.relocalization.ready(now_s)
            and not self.relocalization.invalid_reason
            and self.latest_odom_pose is not None
            and now_s - self.latest_odom_monotonic_s <= self.args.odometry_timeout_s
            and not self.path.invalid_reason
        )

    def payload(self, now_s: float) -> dict[str, Any]:
        ready = self.reference_ready(now_s)
        current_pose = self.current_map_pose()
        distance_m = (
            math.inf if current_pose is None else current_pose.distance(self.target_pose)
        )
        yaw_error_rad = (
            math.inf if current_pose is None else current_pose.yaw_error(self.target_pose)
        )
        rgb_arrival_confirmed = self.rgb_arrival_confirmed(now_s)
        if ready and not self.episode_started:
            self.episode_started = True
            self.episode_started_utc = utc_now()
            self.start_pose_map = current_pose
            if self.latest_odom_pose is not None and self.latest_odom_stamp_s is not None:
                self.path.update(self.latest_odom_stamp_s, self.latest_odom_pose)
        success = self.arrival.update(
            now_s=now_s,
            metric_distance_m=distance_m,
            planar_speed_mps=self.latest_speed_mps,
            rgb_arrival_confirmed=rgb_arrival_confirmed,
            reference_ready=ready and self.episode_started,
        )
        return {
            "schema": STATUS_SCHEMA,
            "run_id": self.args.run_id,
            "utc": utc_now(),
            "classification": "independent_reference_slam_not_metrological_ground_truth",
            "reference_ready": ready,
            "episode_started": self.episode_started,
            "episode_started_utc": self.episode_started_utc,
            "start_pose_map": (
                None if self.start_pose_map is None else pose_payload(self.start_pose_map)
            ),
            "success": success,
            "relocalization_evidence": {
                "authority": "vendor_map_to_odom_tf",
                "fallback_slam_assumed_while_tf_absent": True,
                **self.relocalization.status(now_s),
            },
            "odometry": {
                "fresh": now_s - self.latest_odom_monotonic_s
                <= self.args.odometry_timeout_s,
                "age_s": (
                    None
                    if self.latest_odom_monotonic_s <= 0.0
                    else round(now_s - self.latest_odom_monotonic_s, 3)
                ),
                "planar_speed_mps": self.latest_speed_mps,
                **self.path.status(),
            },
            "goal": {
                "receipt_path": str(self.goal_receipt_path),
                "receipt_sha256": sha256_file(self.goal_receipt_path),
                "goal_rgb_sha256": self.goal_receipt["d435i_goal_rgb"]["sha256"],
                "target_pose_map": pose_payload(self.target_pose),
            },
            "map": {
                "path": str(self.map_path),
                "sha256": self.goal_receipt["odin_map"]["sha256"],
            },
            "sensor_contract": {
                "scene_contract_sha256": self.goal_receipt["scene_contract"]["sha256"],
                "sensor_serial": self.goal_receipt["sensor_serial"],
                "calibration_sha256": self.goal_receipt["calibration_sha256"],
                "mount_receipt_sha256": self.goal_receipt["mount_receipt_sha256"],
                "current_driver_profile_sha256": self.current_driver_receipt[
                    "sha256"
                ],
            },
            "current_pose_map": (
                None if current_pose is None else pose_payload(current_pose)
            ),
            "distance_to_goal_m": (
                None if not math.isfinite(distance_m) else round(distance_m, 4)
            ),
            "yaw_error_rad": (
                None if not math.isfinite(yaw_error_rad) else round(yaw_error_rad, 4)
            ),
            "rgb_arrival": {
                "topic": self.args.arrival_status_topic,
                "fresh": now_s - self.latest_arrival_monotonic_s
                <= self.args.arrival_status_timeout_s,
                "latched": rgb_arrival_confirmed,
                "source_payload": self.latest_arrival_payload,
            },
            "arrival": self.arrival.status(now_s),
            "policy_input": False,
            "motion_authority": False,
            "estop_authority": False,
        }

    def finish(self, termination: str, payload: Optional[dict[str, Any]] = None) -> None:
        if self.final_result is not None:
            return
        status = payload or self.payload(time.monotonic())
        result = {
            **status,
            "schema": RESULT_SCHEMA,
            "completed_utc": utc_now(),
            "termination": termination,
            "success": bool(status["success"] and termination == "arrival_confirmed"),
            "actual_path_m": status["odometry"]["path_length_m"],
            "shortest_path_m": None,
            "spl": None,
            "metric_note": "Run score is incomplete until score_odin_gt.py binds frozen A* L_i.",
        }
        atomic_write_json(self.result_path, result)
        self.final_result = result

    def tick(self) -> None:
        now_s = time.monotonic()
        payload = self.payload(now_s)
        message = self.string_type()
        message.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.status_pub.publish(message)
        self.status_log.write(message.data + "\n")
        if payload["success"]:
            self.finish("arrival_confirmed", payload)

    def close(self) -> None:
        self.finish("operator_stop")
        self.status_log.close()
        self.node.destroy_node()


def run_formal(args: argparse.Namespace) -> int:
    import rclpy

    rclpy.init()
    monitor = FormalMonitor(args)
    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(monitor.node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        monitor.close()
        rclpy.shutdown()
    print(json.dumps(monitor.final_result, indent=2, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture = subparsers.add_parser("capture-goal")
    capture.add_argument("--mapping-session-id", required=True)
    capture.add_argument("--goal-rgb", type=Path, required=True)
    capture.add_argument("--goal-depth", type=Path)
    capture.add_argument("--output", type=Path, required=True)
    capture.add_argument("--odometry-topic", default="/odin1/odometry")
    capture.add_argument("--map-frame", default="odom")
    capture.add_argument("--base-frame", default="odin1_base_link")
    capture.add_argument("--samples", type=int, default=20)
    capture.add_argument("--maximum-speed-mps", type=float, default=0.03)
    capture.add_argument("--timeout-s", type=float, default=15.0)

    seal = subparsers.add_parser("seal-goal")
    seal.add_argument("--draft", type=Path, required=True)
    seal.add_argument("--map-file", type=Path, required=True)
    seal.add_argument("--occupancy-yaml", type=Path, required=True)
    seal.add_argument("--scene-contract", type=Path, required=True)
    seal.add_argument("--output", type=Path, required=True)

    run = subparsers.add_parser("run")
    run.add_argument("--run-id", required=True)
    run.add_argument("--goal-receipt", type=Path, required=True)
    run.add_argument("--map-file", type=Path, required=True)
    run.add_argument("--driver-profile-receipt", type=Path, required=True)
    run.add_argument("--output-dir", type=Path, required=True)
    run.add_argument("--odometry-topic", default="/odin1/odometry")
    run.add_argument(
        "--arrival-status-topic", default="/navdp/rgb_arrival_status"
    )
    run.add_argument("--status-topic", default="/navdp/gt/status")
    run.add_argument("--map-frame", default="map")
    run.add_argument("--odom-frame", default="odom")
    run.add_argument("--publish-rate-hz", type=float, default=5.0)
    run.add_argument("--odometry-timeout-s", type=float, default=0.50)
    run.add_argument("--arrival-status-timeout-s", type=float, default=1.0)
    run.add_argument("--relocalization-hold-s", type=float, default=2.0)
    run.add_argument("--relocalization-minimum-samples", type=int, default=5)
    run.add_argument("--maximum-tf-translation-change-m", type=float, default=0.15)
    run.add_argument("--maximum-tf-rotation-change-deg", type=float, default=5.0)
    run.add_argument("--maximum-odometry-step-m", type=float, default=0.50)
    run.add_argument("--maximum-inferred-speed-mps", type=float, default=2.0)
    run.add_argument("--arrival-distance-m", type=float, default=0.85)
    run.add_argument("--arrival-speed-mps", type=float, default=0.10)
    run.add_argument("--arrival-hold-s", type=float, default=1.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.command == "capture-goal":
        return capture_goal(args)
    if args.command == "seal-goal":
        return seal_goal(args)
    if args.command == "run":
        return run_formal(args)
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
