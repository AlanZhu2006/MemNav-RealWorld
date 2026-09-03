#!/usr/bin/env python3
"""Fail-closed RealSense recovery service for the operator dashboard."""

from __future__ import annotations

import argparse
from pathlib import Path
import shlex
import subprocess
import threading
import time
from typing import Callable

from geometry_msgs.msg import Twist
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def _camera_launch_command(
    camera_script: Path,
    resolved_config: Path,
    camera_log: Path,
) -> str:
    """Return the shell command tmux should own for the RGB-D window."""

    return (
        f"exec {shlex.quote(str(camera_script))} "
        f"--config {shlex.quote(str(resolved_config))} "
        f">>{shlex.quote(str(camera_log))} 2>&1"
    )


def restart_tmux_camera(
    session: str,
    camera_script: Path,
    resolved_config: Path,
    camera_log: Path,
    *,
    runner: Callable = subprocess.run,
) -> None:
    """Replace or recreate only the session's ``rgbd`` window."""

    if not camera_script.is_file():
        raise RuntimeError(f"camera launcher missing: {camera_script}")
    if not resolved_config.is_file():
        raise RuntimeError(f"resolved config missing: {resolved_config}")
    camera_log.parent.mkdir(parents=True, exist_ok=True)
    launch_command = _camera_launch_command(
        camera_script, resolved_config, camera_log
    )
    windows = runner(
        ["tmux", "list-windows", "-t", session, "-F", "#{window_name}"],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )
    if "rgbd" in windows.stdout.splitlines():
        runner(
            [
                "tmux",
                "respawn-window",
                "-k",
                "-t",
                f"{session}:rgbd",
                launch_command,
            ],
            check=True,
            timeout=5,
        )
        return
    runner(
        [
            "tmux",
            "new-window",
            "-d",
            "-t",
            f"{session}:",
            "-n",
            "rgbd",
            launch_command,
        ],
        check=True,
        timeout=5,
    )


def restart_systemd_camera(
    unit: str,
    *,
    runner: Callable = subprocess.run,
) -> None:
    """Restart the always-on observer camera owned by user systemd."""

    if unit != "memnav-observer-camera.service":
        raise RuntimeError(f"unsupported camera systemd unit: {unit}")
    runner(
        ["systemctl", "--user", "restart", unit],
        check=True,
        timeout=30,
    )


class CameraRecoveryService(Node):
    """Restart RealSense while keeping every motion gate fail-closed."""

    def __init__(
        self,
        *,
        session: str,
        camera_script: Path,
        resolved_config: Path,
        camera_log: Path,
        camera_systemd_unit: str | None = None,
        rgb_topic: str,
        depth_topic: str,
        enable_topic: str,
        estop_topic: str,
        cmd_vel_topic: str,
        recovery_timeout_s: float = 45.0,
        minimum_frames: int = 10,
        verification_grace_s: float = 1.0,
    ) -> None:
        super().__init__("navdp_camera_recovery")
        self._session = session
        self._camera_script = camera_script
        self._resolved_config = resolved_config
        self._camera_log = camera_log
        self._camera_systemd_unit = camera_systemd_unit
        self._recovery_timeout_s = max(5.0, float(recovery_timeout_s))
        self._minimum_frames = max(1, int(minimum_frames))
        self._verification_grace_s = max(0.0, float(verification_grace_s))
        self._restart_lock = threading.Lock()
        self._frame_condition = threading.Condition()
        self._verification_after = float("inf")
        self._rgb_frames = 0
        self._depth_frames = 0

        callback_group = ReentrantCallbackGroup()
        self._enable_pub = self.create_publisher(Bool, enable_topic, 10)
        self._estop_pub = self.create_publisher(Bool, estop_topic, 10)
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(
            Image,
            rgb_topic,
            self._on_rgb,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            Image,
            depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            "~/restart",
            self._restart_service,
            callback_group=callback_group,
        )
        self.get_logger().info(
            "Camera recovery ready: restart is fail-closed and never resumes motion"
        )

    def _on_rgb(self, _message: Image) -> None:
        self._record_frame("rgb")

    def _on_depth(self, _message: Image) -> None:
        self._record_frame("depth")

    def _record_frame(self, stream: str) -> None:
        now = time.monotonic()
        with self._frame_condition:
            if now < self._verification_after:
                return
            if stream == "rgb":
                self._rgb_frames += 1
            else:
                self._depth_frames += 1
            self._frame_condition.notify_all()

    def _latch_motion_stop(self) -> None:
        disabled = Bool()
        disabled.data = False
        estop = Bool()
        estop.data = True
        zero = Twist()
        for _ in range(3):
            self._enable_pub.publish(disabled)
            self._estop_pub.publish(estop)
            self._cmd_pub.publish(zero)
            time.sleep(0.02)

    def _restart_camera(self) -> None:
        if self._camera_systemd_unit:
            restart_systemd_camera(self._camera_systemd_unit)
            return
        restart_tmux_camera(
            self._session,
            self._camera_script,
            self._resolved_config,
            self._camera_log,
        )

    def _restart_service(self, _request, response):
        if not self._restart_lock.acquire(blocking=False):
            response.success = False
            response.message = "Camera recovery is already in progress"
            return response
        try:
            self._latch_motion_stop()
            try:
                self._restart_camera()
            except Exception as exc:
                response.success = False
                response.message = (
                    f"Camera restart failed: {exc}; motion remains disabled "
                    "and estop asserted"
                )
                self.get_logger().error(response.message)
                return response

            # Ignore frames queued by the old publisher. The RealSense launch
            # includes a hardware reset and normally needs about 12 seconds.
            with self._frame_condition:
                self._verification_after = (
                    time.monotonic() + self._verification_grace_s
                )
                self._rgb_frames = 0
                self._depth_frames = 0
                deadline = time.monotonic() + self._recovery_timeout_s
                while (
                    self._rgb_frames < self._minimum_frames
                    or self._depth_frames < self._minimum_frames
                ):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self._frame_condition.wait(timeout=remaining)
                rgb_frames = self._rgb_frames
                depth_frames = self._depth_frames
                self._verification_after = float("inf")

            self._latch_motion_stop()
            recovered = (
                rgb_frames >= self._minimum_frames
                and depth_frames >= self._minimum_frames
            )
            response.success = recovered
            if recovered:
                response.message = (
                    "Camera recovery succeeded: fresh RGB and aligned-depth "
                    f"frames verified ({rgb_frames}/{depth_frames}); motion "
                    "remains disabled and estop asserted"
                )
                self.get_logger().warning(response.message)
            else:
                response.message = (
                    "Camera recovery timed out: fresh RGB/aligned-depth frames "
                    f"{rgb_frames}/{depth_frames}; motion remains disabled and "
                    "estop asserted"
                )
                self.get_logger().error(response.message)
            return response
        finally:
            self._restart_lock.release()


def _parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True)
    parser.add_argument("--camera-script", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--camera-log", type=Path, required=True)
    parser.add_argument("--camera-systemd-unit")
    parser.add_argument("--rgb-topic", required=True)
    parser.add_argument("--depth-topic", required=True)
    parser.add_argument("--enable-topic", default="/navdp/enabled")
    parser.add_argument("--estop-topic", default="/navdp/estop")
    parser.add_argument("--cmd-vel-topic", default="/navdp/cmd_vel")
    parser.add_argument("--recovery-timeout-s", type=float, default=45.0)
    parser.add_argument("--minimum-frames", type=int, default=10)
    return parser.parse_known_args()


def main() -> None:
    args, ros_args = _parse_args()
    rclpy.init(args=ros_args)
    node = CameraRecoveryService(
        session=args.session,
        camera_script=args.camera_script.resolve(),
        resolved_config=args.config.resolve(),
        camera_log=args.camera_log.resolve(),
        camera_systemd_unit=args.camera_systemd_unit,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        enable_topic=args.enable_topic,
        estop_topic=args.estop_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        recovery_timeout_s=args.recovery_timeout_s,
        minimum_frames=args.minimum_frames,
    )
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
