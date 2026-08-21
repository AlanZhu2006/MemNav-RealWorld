#!/usr/bin/env python3
"""ROS 2 adapter from aligned RGB-D NavDP trajectories to safe Go2 Twist commands."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import message_filters
import numpy as np

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from debug_visualization import ranked_candidates, score_rgb
from image_goal_io import load_rgb_image
from navdp_client import NavDPClient
from trajectory_control import (
    ControllerConfig,
    DepthSafetyConfig,
    VelocityCommand,
    apply_depth_safety,
    front_clearance,
    slew_limit,
    trajectory_to_command,
)


class NavDPGo2Adapter(Node):
    """Run NavDP asynchronously and publish fail-closed velocity commands."""

    def __init__(self) -> None:
        super().__init__("navdp_go2_adapter")
        self._declare_parameters()
        self._load_parameters()

        self._bridge = CvBridge()
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._inference_event = threading.Event()
        self._inference_busy = False

        self._rgb: Optional[np.ndarray] = None
        self._depth_m: Optional[np.ndarray] = None
        self._intrinsic: Optional[np.ndarray] = None
        self._rgbd_monotonic = 0.0
        self._rgb_depth_skew_s: Optional[float] = None
        self._goal_xy: Optional[np.ndarray] = None
        self._goal_monotonic = 0.0
        self._image_goal: Optional[np.ndarray] = None
        if self.mode == "imagegoal":
            self._image_goal = load_rgb_image(self.image_goal_path)

        self._enabled = bool(self.get_parameter("enable_on_start").value)
        self._estop = False
        self._server_initialized = False
        self._reset_requested = True
        self._trajectory: Optional[np.ndarray] = None
        self._candidate_trajectories = np.empty((0, 0, 2), dtype=np.float32)
        self._candidate_values = np.empty((0,), dtype=np.float32)
        self._target_command = VelocityCommand()
        self._last_command = VelocityCommand()
        self._plan_monotonic = 0.0
        self._last_inference_s = 0.0
        self._last_error = ""
        self._stop_reason = "disabled" if not self._enabled else "waiting_for_plan"
        self._last_warn: dict[str, float] = {}
        # Protocol-v3 two-phase episode state (hub is authoritative; these
        # mirror its receipts for status reporting and local gating only).
        self._phase: Optional[str] = None
        self._frames_recorded = 0
        self._goal_candidates_captured = 0
        self._client_lock = threading.Lock()

        self._client = NavDPClient(
            self.server_url,
            self.connect_timeout_s,
            self.request_timeout_s,
        )

        command_qos = QoSProfile(depth=10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, command_qos)
        self._path_pub = self.create_publisher(Path, self.path_topic, state_qos)
        self._status_pub = self.create_publisher(String, self.status_topic, state_qos)
        self._debug_markers_pub = None
        self._image_goal_pub = None
        if self.debug_visualization:
            self._debug_markers_pub = self.create_publisher(
                MarkerArray, self.debug_markers_topic, state_qos
            )
        if self.mode == "imagegoal":
            self._image_goal_pub = self.create_publisher(
                Image, self.image_goal_debug_topic, state_qos
            )

        self._rgb_sub = message_filters.Subscriber(
            self, Image, self.rgb_topic, qos_profile_sensor_data
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile_sensor_data
        )
        self._rgbd_sync = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub],
            queue_size=self.rgbd_sync_queue_size,
            slop=self.max_rgb_depth_skew_s,
        )
        self._rgbd_sync.registerCallback(self._on_rgbd)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, qos_profile_sensor_data
        )
        if self.mode in {"pointgoal", "startgoal"}:
            self.create_subscription(
                PointStamped, self.goal_topic, self._on_goal, command_qos
            )
        self.create_subscription(Bool, self.enable_topic, self._on_enable, command_qos)
        self.create_subscription(Bool, self.estop_topic, self._on_estop, command_qos)

        self.create_service(SetBool, "~/set_enabled", self._set_enabled_service)
        self.create_service(Trigger, "~/reset_policy", self._reset_policy_service)
        if self.two_phase_episode:
            self.create_service(
                Trigger, "~/capture_goal_candidate",
                self._capture_goal_candidate_service,
            )
            self.create_service(
                Trigger, "~/begin_revisit", self._begin_revisit_service
            )

        self.create_timer(1.0 / self.planning_rate_hz, self._request_inference)
        self.create_timer(1.0 / self.control_rate_hz, self._control_tick)
        self.create_timer(0.5, self._publish_status)

        self._worker = threading.Thread(
            target=self._inference_worker,
            name="navdp-inference",
            daemon=True,
        )
        self._worker.start()

        self.get_logger().info(
            "NavDP Go2 adapter ready: "
            f"backend={self.backend}, mode={self.mode}, server={self.server_url}, "
            f"cmd={self.cmd_vel_topic}, enabled={self._enabled}, odometry=disabled, "
            f"rgbd_sync=approximate(queue={self.rgbd_sync_queue_size}, "
            f"slop={self.max_rgb_depth_skew_s:.3f}s)"
        )
        if self.mode == "pointgoal":
            self.get_logger().warning(
                f"pointgoal requires a CURRENT {self.base_frame} goal on {self.goal_topic}; "
                "publish it continuously because no VIO/odometry is used"
            )
        elif self.mode == "startgoal":
            self.get_logger().info(
                f"startgoal latches one initial {self.base_frame} goal; no VIO/odometry is used"
            )
        elif self.mode == "imagegoal":
            self.get_logger().warning(
                f"imagegoal loaded {self.image_goal_path}; policy has no internal arrival signal, "
                "use the isolated ImageGoal evaluator for ground-truth termination"
            )

    def _declare_parameters(self) -> None:
        defaults = {
            "backend": "x_navdp",
            "mode": "startgoal",
            "server_url": "http://127.0.0.1:8888",
            "rgb_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "goal_topic": "/navdp/relative_goal",
            "image_goal_path": "",
            "image_goal_debug_topic": "/navdp/image_goal",
            "cmd_vel_topic": "/navdp/cmd_vel",
            "path_topic": "/navdp/trajectory",
            "status_topic": "/navdp/status",
            "debug_markers_topic": "/navdp/debug/markers",
            "enable_topic": "/navdp/enabled",
            "estop_topic": "/navdp/estop",
            "base_frame": "base_link",
            "debug_visualization": True,
            "debug_max_candidates": 6,
            "enable_on_start": False,
            "plan_while_disabled": True,
            "two_phase_episode": False,
            "planning_rate_hz": 2.0,
            "control_rate_hz": 20.0,
            "connect_timeout_s": 3.0,
            "request_timeout_s": 180.0,
            "sensor_timeout_s": 0.60,
            "goal_timeout_s": 0.75,
            "trajectory_timeout_s": 1.25,
            "max_rgb_depth_skew_s": 0.10,
            "rgbd_sync_queue_size": 15,
            "depth_scale_m": 0.001,
            "goal_arrival_m": 0.60,
            "lookahead_m": 0.60,
            "max_linear_mps": 0.30,
            "max_angular_rps": 0.60,
            "rotate_in_place_angle_rad": 0.70,
            "rotate_gain": 1.50,
            "slow_path_length_m": 1.00,
            "allow_reverse": False,
            "reverse_lateral_angle_rad": 0.55,
            "max_linear_accel_mps2": 0.50,
            "max_angular_accel_rps2": 1.20,
            "depth_hard_stop_m": 0.45,
            "depth_slow_distance_m": 0.80,
            "depth_percentile": 10.0,
            "depth_roi_left": 0.35,
            "depth_roi_right": 0.65,
            "depth_roi_top": 0.30,
            "depth_roi_bottom": 0.70,
            "depth_min_valid_fraction": 0.03,
            "depth_max_valid_m": 5.0,
            "depth_fail_closed": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.backend not in {"x_navdp", "navdp"}:
            raise ValueError("backend must be 'x_navdp' or 'navdp'")
        if self.mode not in {"pointgoal", "startgoal", "imagegoal", "nogoal"}:
            raise ValueError("mode must be pointgoal, startgoal, imagegoal, or nogoal")
        if self.backend == "x_navdp" and self.mode in {"imagegoal", "nogoal"}:
            raise ValueError("X-NavDP server supports pointgoal/startgoal only")
        self.two_phase_episode = bool(
            self.get_parameter("two_phase_episode").value
        )
        if self.two_phase_episode and self.mode != "imagegoal":
            raise ValueError(
                "two_phase_episode (protocol-v3 hub) requires mode=imagegoal"
            )

        for name in (
            "server_url",
            "rgb_topic",
            "depth_topic",
            "camera_info_topic",
            "goal_topic",
            "image_goal_path",
            "image_goal_debug_topic",
            "cmd_vel_topic",
            "path_topic",
            "status_topic",
            "debug_markers_topic",
            "enable_topic",
            "estop_topic",
            "base_frame",
        ):
            setattr(self, name, str(self.get_parameter(name).value))

        for name in (
            "planning_rate_hz",
            "control_rate_hz",
            "connect_timeout_s",
            "request_timeout_s",
            "sensor_timeout_s",
            "goal_timeout_s",
            "trajectory_timeout_s",
            "max_rgb_depth_skew_s",
            "depth_scale_m",
            "goal_arrival_m",
            "max_linear_accel_mps2",
            "max_angular_accel_rps2",
        ):
            setattr(self, name, float(self.get_parameter(name).value))
        self.plan_while_disabled = bool(self.get_parameter("plan_while_disabled").value)
        self.debug_visualization = bool(
            self.get_parameter("debug_visualization").value
        )
        self.debug_max_candidates = max(
            0, int(self.get_parameter("debug_max_candidates").value)
        )
        self.rgbd_sync_queue_size = int(
            self.get_parameter("rgbd_sync_queue_size").value
        )

        if self.planning_rate_hz <= 0.0 or self.control_rate_hz <= 0.0:
            raise ValueError("planning_rate_hz and control_rate_hz must be positive")
        if self.sensor_timeout_s <= 0.0 or self.max_rgb_depth_skew_s <= 0.0:
            raise ValueError("sensor_timeout_s and max_rgb_depth_skew_s must be positive")
        if self.rgbd_sync_queue_size <= 0:
            raise ValueError("rgbd_sync_queue_size must be positive")

        self.controller_config = ControllerConfig(
            lookahead_m=float(self.get_parameter("lookahead_m").value),
            max_linear_mps=float(self.get_parameter("max_linear_mps").value),
            max_angular_rps=float(self.get_parameter("max_angular_rps").value),
            rotate_in_place_angle_rad=float(
                self.get_parameter("rotate_in_place_angle_rad").value
            ),
            rotate_gain=float(self.get_parameter("rotate_gain").value),
            slow_path_length_m=float(self.get_parameter("slow_path_length_m").value),
            allow_reverse=bool(self.get_parameter("allow_reverse").value),
            reverse_lateral_angle_rad=float(
                self.get_parameter("reverse_lateral_angle_rad").value
            ),
        )
        self.depth_safety_config = DepthSafetyConfig(
            hard_stop_m=float(self.get_parameter("depth_hard_stop_m").value),
            slow_distance_m=float(self.get_parameter("depth_slow_distance_m").value),
            percentile=float(self.get_parameter("depth_percentile").value),
            roi_left=float(self.get_parameter("depth_roi_left").value),
            roi_right=float(self.get_parameter("depth_roi_right").value),
            roi_top=float(self.get_parameter("depth_roi_top").value),
            roi_bottom=float(self.get_parameter("depth_roi_bottom").value),
            min_valid_fraction=float(
                self.get_parameter("depth_min_valid_fraction").value
            ),
            max_valid_depth_m=float(self.get_parameter("depth_max_valid_m").value),
            fail_closed=bool(self.get_parameter("depth_fail_closed").value),
            protect_reverse=True,
            protect_rotation=True,
        )

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image) -> None:
        rgb_stamp_s = self._stamp_to_seconds(rgb_msg.header.stamp)
        depth_stamp_s = self._stamp_to_seconds(depth_msg.header.stamp)
        skew_s = abs(rgb_stamp_s - depth_stamp_s)
        if skew_s > self.max_rgb_depth_skew_s:
            self._warn_throttled(
                "rgbd_pair_skew",
                f"Rejected RGB-D pair with {skew_s:.3f}s timestamp skew",
            )
            return
        try:
            image = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            raw_depth = self._bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except CvBridgeError as exc:
            self._warn_throttled("rgbd_convert", f"RGB-D conversion failed: {exc}")
            return

        rgb = np.asarray(image, dtype=np.uint8)
        depth = np.asarray(raw_depth)
        if depth.ndim == 3:
            depth = depth[..., 0]
        encoding = depth_msg.encoding.upper()
        if encoding in {"16UC1", "MONO16", "16SC1"} or np.issubdtype(depth.dtype, np.integer):
            depth_m = depth.astype(np.float32) * self.depth_scale_m
        elif encoding in {"32FC1", "64FC1"} or np.issubdtype(depth.dtype, np.floating):
            depth_m = depth.astype(np.float32)
        else:
            self._warn_throttled(
                "depth_encoding", f"Unsupported depth encoding {depth_msg.encoding!r}"
            )
            return
        depth_m[~np.isfinite(depth_m)] = 0.0
        depth_m[depth_m < 0.0] = 0.0
        if rgb.shape[:2] != depth_m.shape[:2]:
            self._warn_throttled(
                "rgbd_shape",
                f"Rejected RGB-D pair with shapes {rgb.shape[:2]} and {depth_m.shape[:2]}",
            )
            return

        with self._lock:
            self._rgb = rgb.copy()
            self._depth_m = depth_m.copy()
            self._rgbd_monotonic = time.monotonic()
            self._rgb_depth_skew_s = skew_s

    def _on_camera_info(self, msg: CameraInfo) -> None:
        intrinsic = np.asarray(msg.k, dtype=np.float32).reshape(3, 3)
        if not np.isfinite(intrinsic).all() or intrinsic[0, 0] <= 0.0 or intrinsic[1, 1] <= 0.0:
            self._warn_throttled("camera_info", "Rejected invalid camera intrinsics")
            return
        with self._lock:
            if self._intrinsic is None or not np.allclose(self._intrinsic, intrinsic):
                self._intrinsic = intrinsic
                self._reset_requested = True

    def _on_goal(self, msg: PointStamped) -> None:
        if self.mode not in {"pointgoal", "startgoal"}:
            return
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            self._warn_throttled(
                "goal_frame",
                f"Rejected goal in {msg.header.frame_id!r}; expected {self.base_frame!r}",
            )
            return
        goal = np.array([msg.point.x, msg.point.y], dtype=np.float32)
        if not np.isfinite(goal).all():
            self._warn_throttled("goal_value", "Rejected non-finite relative goal")
            return
        with self._lock:
            had_goal = self._goal_xy is not None
            self._goal_xy = goal
            self._goal_monotonic = time.monotonic()
            if self.mode == "startgoal" or not had_goal:
                self._trajectory = None
                self._plan_monotonic = 0.0
            needs_immediate_plan = self._trajectory is None
        if needs_immediate_plan:
            self._inference_event.set()
        if self.mode == "startgoal" or not had_goal:
            self.get_logger().info(
                f"Accepted {self.mode} goal in {self.base_frame}: "
                f"x={goal[0]:.2f}, y={goal[1]:.2f}"
            )

    def _on_enable(self, msg: Bool) -> None:
        self._set_enabled(bool(msg.data), "enable topic")

    def _on_estop(self, msg: Bool) -> None:
        with self._lock:
            self._estop = bool(msg.data)
        if msg.data:
            self._publish_zero("estop")
            self.get_logger().warning("NavDP emergency stop asserted")
        else:
            self.get_logger().info("NavDP emergency stop released; enable state unchanged")

    def _set_enabled_service(self, request, response):
        self._set_enabled(bool(request.data), "set_enabled service")
        response.success = True
        response.message = f"NavDP motion {'enabled' if request.data else 'disabled'}"
        return response

    def _reset_policy_service(self, _request, response):
        with self._lock:
            self._reset_requested = True
            self._server_initialized = False
            self._trajectory = None
            self._candidate_trajectories = np.empty((0, 0, 2), dtype=np.float32)
            self._candidate_values = np.empty((0,), dtype=np.float32)
            self._plan_monotonic = 0.0
            self._last_error = ""
        self._publish_zero("policy_reset")
        self._inference_event.set()
        response.success = True
        response.message = "NavDP reset queued"
        return response

    def _capture_goal_candidate_service(self, _request, response):
        with self._lock:
            rgb = None if self._rgb is None else self._rgb.copy()
            initialized = self._server_initialized
            phase = self._phase
        if not initialized or rgb is None:
            response.success = False
            response.message = "server not initialized or no RGB frame yet"
            return response
        if phase != "memory_recording":
            response.success = False
            response.message = (
                f"goal candidates require the memory_recording phase, not {phase}"
            )
            return response
        try:
            with self._client_lock:
                receipt = self._client.goal_candidate(rgb)
        except Exception as exc:
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            return response
        with self._lock:
            self._goal_candidates_captured += 1
        self.get_logger().info(f"goal candidate captured: {receipt}")
        response.success = True
        response.message = json.dumps(receipt, ensure_ascii=False)
        return response

    def _begin_revisit_service(self, _request, response):
        with self._lock:
            initialized = self._server_initialized
            phase = self._phase
            busy = self._inference_busy
        if not initialized:
            response.success = False
            response.message = "server not initialized"
            return response
        if phase != "memory_recording":
            response.success = False
            response.message = f"begin_revisit requires memory_recording, not {phase}"
            return response
        if busy:
            response.success = False
            response.message = "inference busy; retry when the recording step settles"
            return response
        try:
            with self._client_lock:
                receipt = self._client.begin_revisit()
        except Exception as exc:
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            return response
        with self._lock:
            self._phase = "revisit_query"
        self.get_logger().info(f"revisit phase started: {receipt}")
        response.success = True
        response.message = json.dumps(receipt, ensure_ascii=False)
        return response

    def _set_enabled(self, enabled: bool, source: str) -> None:
        with self._lock:
            changed = enabled != self._enabled
            self._enabled = enabled
        if not enabled:
            self._publish_zero("disabled")
        elif self._estop:
            self.get_logger().warning("Motion enable accepted, but estop is still asserted")
        if changed:
            self.get_logger().info(
                f"NavDP motion {'enabled' if enabled else 'disabled'} by {source}"
            )

    def _request_inference(self) -> None:
        with self._lock:
            should_plan = self.plan_while_disabled or self._enabled
        if should_plan:
            self._inference_event.set()

    def _snapshot_inference_input(self):
        now = time.monotonic()
        with self._lock:
            if self._rgb is None or self._depth_m is None or self._intrinsic is None:
                return None, "waiting_for_rgbd_or_camera_info"
            if now - self._rgbd_monotonic > self.sensor_timeout_s:
                return None, "rgbd_stale"
            if self.mode in {"pointgoal", "startgoal"}:
                if self._goal_xy is None:
                    return None, "waiting_for_goal"
                if self.mode == "pointgoal" and now - self._goal_monotonic > self.goal_timeout_s:
                    return None, "goal_stale"
                goal_condition = self._goal_xy.copy()
            elif self.mode == "imagegoal":
                if self.two_phase_episode and self._phase in (None, "memory_recording"):
                    goal_condition = None
                elif self._image_goal is None:
                    return None, "waiting_for_image_goal"
                else:
                    goal_condition = self._image_goal.copy()
            else:
                goal_condition = None
            return (
                self._rgb.copy(),
                self._depth_m.copy(),
                self._intrinsic.copy(),
                goal_condition,
                self._reset_requested,
            ), "ready"

    def _inference_worker(self) -> None:
        while not self._stop_event.is_set():
            self._inference_event.wait(timeout=0.2)
            self._inference_event.clear()
            if self._stop_event.is_set():
                break

            snapshot, reason = self._snapshot_inference_input()
            if snapshot is None:
                with self._lock:
                    self._stop_reason = reason
                continue

            rgb, depth_m, intrinsic, goal_condition, reset_requested = snapshot
            with self._lock:
                if self._inference_busy:
                    continue
                self._inference_busy = True
            started = time.monotonic()
            try:
                if reset_requested or not self._server_initialized:
                    with self._client_lock:
                        algorithm = self._client.reset(intrinsic)
                    with self._lock:
                        self._server_initialized = True
                        self._reset_requested = False
                        self._frames_recorded = 0
                        self._goal_candidates_captured = 0
                        self._phase = (
                            "memory_recording"
                            if self.two_phase_episode
                            else "revisit_query"
                        )
                    self.get_logger().info(f"Policy server initialized: {algorithm}")

                with self._lock:
                    recording = (
                        self.two_phase_episode
                        and self.mode == "imagegoal"
                        and self._phase == "memory_recording"
                    )
                if recording:
                    # Protocol v3: record-only append.  No goal is sent, no
                    # trajectory is produced; the hub rejects goal queries in
                    # this phase, so do not attempt planning at all.
                    with self._client_lock:
                        receipt = self._client.memory_step(rgb)
                    finished = time.monotonic()
                    with self._lock:
                        self._frames_recorded = int(
                            receipt.get("frames_recorded", self._frames_recorded + 1)
                        )
                        self._last_inference_s = finished - started
                        self._last_error = ""
                        self._stop_reason = "memory_recording"
                    continue

                if self.mode == "nogoal":
                    with self._client_lock:
                        trajectory, all_trajectories, all_values = self._client.nogoal_step(
                            rgb, depth_m
                        )
                elif self.mode == "imagegoal":
                    with self._client_lock:
                        trajectory, all_trajectories, all_values = self._client.imagegoal_step(
                            goal_condition, rgb, depth_m
                        )
                else:
                    with self._client_lock:
                        trajectory, all_trajectories, all_values = self._client.pointgoal_step(
                            goal_condition, rgb, depth_m
                        )
                path = self._normalize_trajectory(trajectory)
                candidates, candidate_values = ranked_candidates(
                    all_trajectories, all_values, self.debug_max_candidates
                )
                target = trajectory_to_command(path, self.controller_config)
                finished = time.monotonic()
                with self._lock:
                    self._trajectory = path
                    self._candidate_trajectories = candidates
                    self._candidate_values = candidate_values
                    self._target_command = target
                    self._plan_monotonic = finished
                    self._last_inference_s = finished - started
                    self._last_error = ""
                    self._stop_reason = "ready"
                if not self._stop_event.is_set() and rclpy.ok():
                    self._publish_path(path)
                    self._publish_debug_markers()
            except Exception as exc:
                with self._lock:
                    self._last_error = f"{type(exc).__name__}: {exc}"
                    self._stop_reason = "inference_error"
                if not self._stop_event.is_set() and rclpy.ok():
                    self._warn_throttled(
                        "inference", f"NavDP inference failed: {exc}", period_s=2.0
                    )
            finally:
                with self._lock:
                    self._inference_busy = False

    @staticmethod
    def _normalize_trajectory(trajectory: np.ndarray) -> np.ndarray:
        path = np.asarray(trajectory, dtype=np.float32)
        while path.ndim > 2 and path.shape[0] == 1:
            path = path[0]
        if path.ndim != 2 or path.shape[0] < 1 or path.shape[1] < 2:
            raise ValueError(f"invalid trajectory shape {path.shape}")
        if not np.isfinite(path[:, :2]).all():
            raise ValueError("trajectory contains non-finite values")
        return path[:, :3].copy()

    def _motion_block_reason(self, now: float) -> Optional[str]:
        if not self._enabled:
            return "disabled"
        if self._estop:
            return "estop"
        if self._rgb is None or self._depth_m is None:
            return "waiting_for_rgbd_or_camera_info"
        if now - self._rgbd_monotonic > self.sensor_timeout_s:
            return "rgbd_stale"
        if self.mode in {"pointgoal", "startgoal"} and self._goal_xy is None:
            return "waiting_for_goal"
        if self.mode == "imagegoal" and self._image_goal is None:
            return "waiting_for_image_goal"
        if (
            self.mode == "pointgoal"
            and now - self._goal_monotonic > self.goal_timeout_s
        ):
            return "goal_stale"
        if self._trajectory is None or self._plan_monotonic <= 0.0:
            return "waiting_for_plan"
        if now - self._plan_monotonic > self.trajectory_timeout_s:
            return "trajectory_stale"
        if self._last_error:
            return "inference_error"
        if (
            self.mode == "pointgoal"
            and self._goal_xy is not None
            and float(np.linalg.norm(self._goal_xy)) <= self.goal_arrival_m
        ):
            return "goal_reached"
        return None

    def _control_tick(self) -> None:
        now = time.monotonic()
        dt = 1.0 / self.control_rate_hz
        with self._lock:
            reason = self._motion_block_reason(now)
            depth = None if self._depth_m is None else self._depth_m.copy()
            target = self._target_command

        if reason is not None:
            self._publish_zero(reason)
            return

        safety = apply_depth_safety(target, depth, self.depth_safety_config)
        if safety.reason in {"obstacle_stop", "depth_unavailable_stop"}:
            self._publish_zero(safety.reason)
            return

        command = slew_limit(
            self._last_command,
            safety.command,
            dt,
            self.max_linear_accel_mps2,
            self.max_angular_accel_rps2,
        )
        self._publish_command(command, safety.reason)

    def _publish_command(self, command: VelocityCommand, reason: str) -> None:
        msg = Twist()
        msg.linear.x = float(command.linear_x)
        msg.angular.z = float(command.angular_z)
        self._cmd_pub.publish(msg)
        with self._lock:
            self._last_command = command
            self._stop_reason = reason

    def _publish_zero(self, reason: str) -> None:
        msg = Twist()
        self._cmd_pub.publish(msg)
        with self._lock:
            self._last_command = VelocityCommand()
            self._stop_reason = reason

    def _publish_path(self, trajectory: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        xy = trajectory[:, :2]
        for index, point in enumerate(trajectory):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.03
            previous = xy[max(0, index - 1)]
            following = xy[min(xy.shape[0] - 1, index + 1)]
            delta = following - previous
            yaw = math.atan2(float(delta[1]), float(delta[0]))
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def _publish_image_goal(self) -> None:
        if self._image_goal_pub is None or self._image_goal is None:
            return
        message = self._bridge.cv2_to_imgmsg(self._image_goal, encoding="rgb8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        self._image_goal_pub.publish(message)

    @staticmethod
    def _line_marker(
        header,
        namespace: str,
        marker_id: int,
        trajectory: np.ndarray,
        width: float,
        color: tuple[float, float, float, float],
        height: float,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points.append(Point(x=0.0, y=0.0, z=height))
        for waypoint in trajectory:
            marker.points.append(
                Point(x=float(waypoint[0]), y=float(waypoint[1]), z=height)
            )
        return marker

    @staticmethod
    def _text_marker(
        header,
        namespace: str,
        marker_id: int,
        text: str,
        x: float,
        y: float,
        z: float,
        height: float,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.z = height
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = text
        return marker

    def _publish_debug_markers(self) -> None:
        if self._debug_markers_pub is None:
            return
        with self._lock:
            trajectory = None if self._trajectory is None else self._trajectory.copy()
            candidates = self._candidate_trajectories.copy()
            values = self._candidate_values.copy()
            goal = None if self._goal_xy is None else self._goal_xy.copy()
            target = self._target_command
            command = self._last_command
            enabled = self._enabled
            estop = self._estop
            stop_reason = self._stop_reason
            inference_s = self._last_inference_s
            depth = None if self._depth_m is None else self._depth_m

        stamp = self.get_clock().now().to_msg()
        header = PoseStamped().header
        header.stamp = stamp
        header.frame_id = self.base_frame
        markers = []

        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.append(clear)

        footprint = Marker()
        footprint.header = header
        footprint.ns = "robot"
        footprint.id = 0
        footprint.type = Marker.CUBE
        footprint.action = Marker.ADD
        footprint.pose.position.x = 0.0
        footprint.pose.position.y = 0.0
        footprint.pose.position.z = 0.025
        footprint.pose.orientation.w = 1.0
        footprint.scale.x = 0.55
        footprint.scale.y = 0.32
        footprint.scale.z = 0.05
        footprint.color.r = 0.55
        footprint.color.g = 0.58
        footprint.color.b = 0.62
        footprint.color.a = 0.75
        markers.append(footprint)

        if candidates.shape[0] > 0:
            finite_values = values[np.isfinite(values)]
            minimum = float(finite_values.min()) if finite_values.size else 0.0
            maximum = float(finite_values.max()) if finite_values.size else 0.0
            for index, (candidate, value) in enumerate(zip(candidates, values)):
                red, green, blue = score_rgb(float(value), minimum, maximum)
                markers.append(
                    self._line_marker(
                        header,
                        "candidates",
                        index,
                        candidate,
                        0.018,
                        (red, green, blue, 0.55),
                        0.015,
                    )
                )
                endpoint = candidate[-1]
                markers.append(
                    self._text_marker(
                        header,
                        "candidate_scores",
                        index,
                        f"Q {float(value):.2f}",
                        float(endpoint[0]),
                        float(endpoint[1]),
                        0.10,
                        0.09,
                        (red, green, blue, 0.9),
                    )
                )

        if trajectory is not None:
            markers.append(
                self._line_marker(
                    header,
                    "selected",
                    0,
                    trajectory,
                    0.055,
                    (0.1, 1.0, 0.25, 1.0),
                    0.045,
                )
            )

        if goal is not None:
            goal_marker = Marker()
            goal_marker.header = header
            goal_marker.ns = "goal"
            goal_marker.id = 0
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = float(goal[0])
            goal_marker.pose.position.y = float(goal[1])
            goal_marker.pose.position.z = 0.10
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.20
            goal_marker.scale.y = 0.20
            goal_marker.scale.z = 0.20
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.1
            goal_marker.color.b = 0.2
            goal_marker.color.a = 0.95
            markers.append(goal_marker)
            markers.append(
                self._text_marker(
                    header,
                    "goal_label",
                    0,
                    f"goal ({goal[0]:.1f}, {goal[1]:.1f})",
                    float(goal[0]),
                    float(goal[1]),
                    0.28,
                    0.11,
                    (1.0, 0.35, 0.35, 1.0),
                )
            )

        lookahead = Marker()
        lookahead.header = header
        lookahead.ns = "lookahead"
        lookahead.id = 0
        lookahead.type = Marker.SPHERE
        lookahead.action = Marker.ADD
        lookahead.pose.position.x = target.target_x
        lookahead.pose.position.y = target.target_y
        lookahead.pose.position.z = 0.09
        lookahead.pose.orientation.w = 1.0
        lookahead.scale.x = 0.13
        lookahead.scale.y = 0.13
        lookahead.scale.z = 0.13
        lookahead.color.r = 0.0
        lookahead.color.g = 0.9
        lookahead.color.b = 1.0
        lookahead.color.a = 1.0
        markers.append(lookahead)

        clearance = (
            None if depth is None else front_clearance(depth, self.depth_safety_config)
        )
        state = "ESTOP" if estop else ("ENABLED" if enabled else "DISABLED")
        state_color = (
            (1.0, 0.1, 0.1, 1.0)
            if estop or stop_reason == "inference_error"
            else ((0.2, 1.0, 0.2, 1.0) if enabled else (1.0, 0.8, 0.1, 1.0))
        )
        clearance_text = "n/a" if clearance is None else f"{clearance:.2f}m"
        markers.append(
            self._text_marker(
                header,
                "status",
                0,
                (
                    f"{state} | {stop_reason}\n"
                    f"cmd {command.linear_x:+.2f} m/s  {command.angular_z:+.2f} rad/s\n"
                    f"depth {clearance_text} | infer {inference_s:.2f}s"
                ),
                -0.25,
                -0.65,
                0.38,
                0.12,
                state_color,
            )
        )

        message = MarkerArray()
        message.markers = markers
        self._debug_markers_pub.publish(message)

    @staticmethod
    def _age(now: float, stamp: float) -> Optional[float]:
        return None if stamp <= 0.0 else round(max(0.0, now - stamp), 3)

    def _publish_status(self) -> None:
        now = time.monotonic()
        with self._lock:
            clearance = (
                None
                if self._depth_m is None
                else front_clearance(self._depth_m, self.depth_safety_config)
            )
            payload = {
                "backend": self.backend,
                "mode": self.mode,
                "odometry": False,
                "enabled": self._enabled,
                "estop": self._estop,
                "server_initialized": self._server_initialized,
                "inference_busy": self._inference_busy,
                "rgb_age_s": self._age(now, self._rgbd_monotonic),
                "depth_age_s": self._age(now, self._rgbd_monotonic),
                "rgbd_age_s": self._age(now, self._rgbd_monotonic),
                "rgb_depth_skew_s": (
                    None
                    if self._rgb_depth_skew_s is None
                    else round(self._rgb_depth_skew_s, 4)
                ),
                "goal_age_s": self._age(now, self._goal_monotonic),
                "image_goal_loaded": self._image_goal is not None,
                "plan_age_s": self._age(now, self._plan_monotonic),
                "last_inference_s": round(self._last_inference_s, 3),
                "candidate_count": int(self._candidate_trajectories.shape[0]),
                "clearance_m": None if clearance is None else round(clearance, 3),
                "stop_reason": self._stop_reason,
                "last_error": self._last_error,
                "phase": self._phase,
                "frames_recorded": self._frames_recorded,
                "goal_candidates_captured": self._goal_candidates_captured,
                "cmd_vx": round(self._last_command.linear_x, 3),
                "cmd_wz": round(self._last_command.angular_z, 3),
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(msg)
        self._publish_debug_markers()
        self._publish_image_goal()

    def _warn_throttled(self, key: str, message: str, period_s: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= period_s:
            self._last_warn[key] = now
            self.get_logger().warning(message)

    def stop(self) -> None:
        self._stop_event.set()
        self._inference_event.set()
        if rclpy.ok():
            try:
                self._publish_zero("shutdown")
            except RuntimeError:
                pass
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavDPGo2Adapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
