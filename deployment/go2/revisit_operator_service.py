#!/usr/bin/env python3
"""Persistent, fixed-contract Foxglove orchestration for engineering Revisit.

The node itself never grants motion authority. A confirmed ``start_revisit``
request runs the existing locked Revisit preparation, then delegates the only
arming step to ``navigation_run_agent.py`` and its fail-closed preflight. The
companion stop service cancels that transaction and continuously reasserts the
software lock while cleanup completes.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
from typing import Any, Optional


ACTIVE_STATE_SCHEMA = "memnav_revisit_debug_state_v1"
STATUS_SCHEMA = "memnav_revisit_operator_status_v1"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class ContractError(RuntimeError):
    """The frozen Revisit state is not safe to execute."""


@dataclass(frozen=True)
class StartContract:
    dataset_id: str
    goal_path: Path
    goal_sha256: str
    experiment_path: Path
    mode: str
    seal_receipt_path: Path
    dataset_manifest_sha256: str


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise ContractError(f"{label} is missing: {path}") from None
    except json.JSONDecodeError as exc:
        raise ContractError(f"{label} is invalid JSON: {exc}") from None
    if not isinstance(value, dict):
        raise ContractError(f"{label} must be a JSON object")
    return value


def _required_string(value: dict[str, Any], key: str, label: str) -> str:
    item = value.get(key)
    if not isinstance(item, str) or not item:
        raise ContractError(f"{label} requires non-empty {key}")
    return item


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_start_contract(repo_root: Path, state_path: Path) -> StartContract:
    """Validate the fixed, argument-free contract used by the start service."""
    state = _load_object(state_path, "active Revisit state")
    if state.get("schema") != ACTIVE_STATE_SCHEMA:
        raise ContractError("active Revisit state has an unsupported schema")

    dataset_id = _required_string(state, "dataset_id", "active Revisit state")
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}", dataset_id) is None:
        raise ContractError("active Revisit dataset_id is invalid")
    mode = _required_string(state, "mode", "active Revisit state")
    if mode not in {"prepared", "recording", "sealed"}:
        raise ContractError(f"Revisit requires a stopped Survey, not mode={mode}")

    goal_path = Path(_required_string(state, "goal_path", "active Revisit state"))
    experiment_path = Path(
        _required_string(state, "experiment_path", "active Revisit state")
    )
    goal_sha256 = _required_string(
        state, "goal_sha256", "active Revisit state"
    )
    if SHA256_RE.fullmatch(goal_sha256) is None:
        raise ContractError("active Revisit goal_sha256 is invalid")
    if not goal_path.is_file():
        raise ContractError(f"frozen Revisit goal is missing: {goal_path}")
    if _sha256(goal_path) != goal_sha256:
        raise ContractError("frozen Revisit goal SHA-256 changed")
    if not experiment_path.is_file():
        raise ContractError(f"Revisit experiment is missing: {experiment_path}")

    seal_receipt_path = (
        repo_root
        / "runtime/go2/two_pass_revisit"
        / dataset_id
        / "survey_seal.json"
    )
    receipt = _load_object(seal_receipt_path, "Survey stop receipt")
    if receipt.get("dataset_id") != dataset_id:
        raise ContractError("Survey stop receipt belongs to a different dataset")
    if receipt.get("recording_active") is not False:
        raise ContractError("Survey stop receipt still reports active recording")
    if receipt.get("motion_enabled") is not False or receipt.get("estop") is not True:
        raise ContractError("Survey stop receipt does not prove disabled + estop")
    if receipt.get("evaluation_depth_consumed_by_policy") is not False:
        raise ContractError("Survey stop receipt violates RGB-only evaluation")
    if int(receipt.get("goal_memory_exact_sha_overlap", -1)) != 0:
        raise ContractError("Survey stop receipt reports goal/memory SHA overlap")
    manifest_sha256 = str(receipt.get("manifest_sha256") or "")
    if SHA256_RE.fullmatch(manifest_sha256) is None:
        raise ContractError("Survey stop receipt has an invalid manifest SHA-256")
    state_manifest = state.get("dataset_manifest_sha256")
    if mode == "sealed" and state_manifest != manifest_sha256:
        raise ContractError("active state and Survey stop receipt manifest differ")

    return StartContract(
        dataset_id=dataset_id,
        goal_path=goal_path,
        goal_sha256=goal_sha256,
        experiment_path=experiment_path,
        mode=mode,
        seal_receipt_path=seal_receipt_path,
        dataset_manifest_sha256=manifest_sha256,
    )


def prepare_command(repo_root: Path, run_id: str) -> list[str]:
    return [
        "bash",
        str(repo_root / "deployment/go2/offboard/revisit_debug.sh"),
        "revisit-prepare",
        "--run-id",
        run_id,
    ]


def navigation_command(
    repo_root: Path, formal_config: Path, timeout_s: float
) -> list[str]:
    return [
        "bash",
        str(repo_root / "deployment/go2/scripts/run_navigation.sh"),
        "--config",
        str(formal_config),
        "--timeout-s",
        f"{timeout_s:g}",
    ]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def validate_realsense_link(camera_summary: str, usb_tree: str) -> None:
    if re.search(r"Intel RealSense D435I?", camera_summary, re.IGNORECASE) is None:
        raise ContractError("RealSense D435i was not enumerated")
    video_speeds = [
        int(speed)
        for speed in re.findall(
            r"Class=Video[^\n]*?,\s*([0-9]+)M", usb_tree, re.IGNORECASE
        )
    ]
    if not video_speeds or max(video_speeds) < 5000:
        raise ContractError("RealSense video interfaces are not on USB SuperSpeed")


class RevisitOperatorService:
    def __init__(
        self,
        *,
        repo_root: Path,
        state_path: Path,
        timeout_s: float,
        robot_ip: str,
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger

        class NodeImpl(Node):
            pass

        self.rclpy = rclpy
        self.Bool = Bool
        self.String = String
        self.Trigger = Trigger
        self.repo_root = repo_root.resolve()
        self.state_path = state_path.resolve()
        self.timeout_s = float(timeout_s)
        self.robot_ip = robot_ip
        self.node = NodeImpl("memnav_revisit_operator")
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        command_qos = QoSProfile(depth=10)
        self.status_pub = self.node.create_publisher(
            String, "/navdp/operator/revisit_workflow", state_qos
        )
        self.enabled_pub = self.node.create_publisher(
            Bool, "/navdp/enabled", command_qos
        )
        self.estop_pub = self.node.create_publisher(
            Bool, "/navdp/estop", command_qos
        )
        self.adapter_stop = self.node.create_client(
            Trigger, "/navdp_go2_adapter/operator_stop"
        )
        self.node.create_service(
            Trigger, "/memnav_operator/start_revisit", self._start_revisit
        )
        self.node.create_service(
            Trigger, "/memnav_operator/operator_stop", self._operator_stop
        )
        self.node.create_timer(0.5, self._tick)

        self._mutex = threading.RLock()
        self._worker: Optional[threading.Thread] = None
        self._process: Optional[subprocess.Popen[bytes]] = None
        self._cancel = threading.Event()
        self._lock_until = 0.0
        self._status: dict[str, Any] = {
            "schema": STATUS_SCHEMA,
            "state": "idle",
            "detail": "Ready for a stopped Survey",
            "active": False,
            "updated_utc": utc_now(),
        }
        self._tick()

    def _set_status(self, state: str, detail: str, **fields: Any) -> None:
        with self._mutex:
            self._status = {
                "schema": STATUS_SCHEMA,
                "state": state,
                "detail": detail,
                "active": state in {"preflight", "preparing", "running", "stopping"},
                "updated_utc": utc_now(),
                **fields,
            }
        self._publish_status()

    def _publish_status(self) -> None:
        with self._mutex:
            payload = dict(self._status)
        message = self.String()
        message.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.status_pub.publish(message)

    def _assert_motion_lock(self) -> None:
        disabled = self.Bool()
        disabled.data = False
        estop = self.Bool()
        estop.data = True
        self.enabled_pub.publish(disabled)
        self.estop_pub.publish(estop)

    def _request_adapter_stop(self) -> None:
        self._assert_motion_lock()
        if self.adapter_stop.service_is_ready():
            self.adapter_stop.call_async(self.Trigger.Request())

    def _tick(self) -> None:
        if time.monotonic() < self._lock_until:
            self._assert_motion_lock()
        self._publish_status()

    def _start_revisit(self, _request: Any, response: Any) -> Any:
        with self._mutex:
            if self._worker is not None and self._worker.is_alive():
                response.success = False
                response.message = "A Revisit transaction is already active"
                return response
        try:
            contract = validate_start_contract(self.repo_root, self.state_path)
        except (ContractError, OSError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            self._set_status("blocked", str(exc))
            return response

        self._cancel.clear()
        self._lock_until = time.monotonic() + 2.0
        self._request_adapter_stop()
        worker = threading.Thread(
            target=self._run_transaction,
            args=(contract,),
            name="memnav-revisit-transaction",
            daemon=True,
        )
        with self._mutex:
            self._worker = worker
        worker.start()
        response.success = True
        response.message = "Revisit accepted; locked stack preparation started"
        return response

    def _operator_stop(self, _request: Any, response: Any) -> Any:
        self._cancel.set()
        self._lock_until = time.monotonic() + 30.0
        self._request_adapter_stop()
        with self._mutex:
            active = self._worker is not None and self._worker.is_alive()
        self._set_status(
            "stopping" if active else "stopped",
            "Cancel requested; disabled + estop asserted",
        )
        response.success = True
        response.message = "Revisit cancelled and motion lock asserted"
        return response

    def _hardware_preflight(self) -> None:
        camera = subprocess.run(
            ["rs-enumerate-devices", "-s"],
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
        camera_text = camera.stdout + camera.stderr
        if camera.returncode != 0:
            raise ContractError("RealSense D435i was not enumerated")
        usb = subprocess.run(
            ["lsusb", "-t"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
        if usb.returncode != 0:
            raise ContractError("USB topology could not be inspected")
        validate_realsense_link(camera_text, usb.stdout + usb.stderr)
        robot = subprocess.run(
            ["ping", "-c", "1", "-W", "2", self.robot_ip],
            check=False,
            capture_output=True,
            timeout=5,
        )
        if robot.returncode != 0:
            raise ContractError(f"Go2 is unreachable at {self.robot_ip}")

    def _run_command(self, argv: list[str], log: Any) -> int:
        log.write(("\n$ " + " ".join(argv) + "\n").encode("utf-8"))
        log.flush()
        process = subprocess.Popen(
            argv,
            cwd=self.repo_root,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        with self._mutex:
            self._process = process
        signalled_at: Optional[float] = None
        terminated_at: Optional[float] = None
        while process.poll() is None:
            if self._cancel.is_set():
                now = time.monotonic()
                try:
                    if signalled_at is None:
                        os.killpg(process.pid, signal.SIGINT)
                        signalled_at = now
                    elif terminated_at is None and now - signalled_at >= 5.0:
                        os.killpg(process.pid, signal.SIGTERM)
                        terminated_at = now
                    elif terminated_at is not None and now - terminated_at >= 5.0:
                        os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            time.sleep(0.2)
        with self._mutex:
            if self._process is process:
                self._process = None
        return int(process.returncode or 0)

    def _cleanup_stack(self, log: Any) -> None:
        self._lock_until = time.monotonic() + 30.0
        self._request_adapter_stop()
        try:
            subprocess.run(
                [
                    "bash",
                    str(self.repo_root / "deployment/go2/offboard/revisit_debug.sh"),
                    "stop",
                ],
                cwd=self.repo_root,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=180,
                check=False,
            )
        except subprocess.TimeoutExpired:
            log.write(b"\nRevisit cleanup timed out; motion lock remains asserted.\n")
            log.flush()

    def _run_transaction(self, contract: StartContract) -> None:
        log_dir = self.repo_root / "runtime/go2/revisit_operator"
        log_dir.mkdir(parents=True, exist_ok=True)
        run_id = f"{contract.dataset_id}_cec_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}"
        log_path = log_dir / f"{run_id}.log"
        try:
            with log_path.open("ab", buffering=0) as log:
                self._set_status(
                    "preflight",
                    "Checking D435i SuperSpeed and Go2 link",
                    dataset_id=contract.dataset_id,
                    run_id=run_id,
                    log_path=str(log_path),
                )
                self._hardware_preflight()
                if self._cancel.is_set():
                    self._cleanup_stack(log)
                    self._set_status("cancelled", "Cancelled before stack preparation")
                    return

                self._set_status(
                    "preparing",
                    "Restarting stack and replaying sealed Survey",
                    dataset_id=contract.dataset_id,
                    run_id=run_id,
                    log_path=str(log_path),
                )
                code = self._run_command(prepare_command(self.repo_root, run_id), log)
                if code != 0 or self._cancel.is_set():
                    self._cleanup_stack(log)
                    state = "cancelled" if self._cancel.is_set() else "failed"
                    self._set_status(state, f"Revisit preparation exited with code {code}")
                    return

                active = _load_object(self.state_path, "active Revisit state")
                if active.get("mode") != "formal_ready" or active.get("run_id") != run_id:
                    raise ContractError("Revisit preparation did not commit formal_ready state")
                formal_config = (
                    self.repo_root
                    / "runtime/go2/two_pass_revisit"
                    / contract.dataset_id
                    / run_id
                    / "formal_config.json"
                )
                if not formal_config.is_file():
                    raise ContractError(f"formal config is missing: {formal_config}")
                if self._cancel.is_set():
                    self._cleanup_stack(log)
                    self._set_status("cancelled", "Cancelled before motion preflight")
                    return

                self._lock_until = 0.0
                self._set_status(
                    "running",
                    "Stack ready; supervised Revisit preflight/navigation active",
                    dataset_id=contract.dataset_id,
                    run_id=run_id,
                    log_path=str(log_path),
                )
                code = self._run_command(
                    navigation_command(self.repo_root, formal_config, self.timeout_s), log
                )
                if self._cancel.is_set():
                    self._cleanup_stack(log)
                    self._set_status("cancelled", "Stopped by operator")
                elif code == 0:
                    self._set_status("complete", "Revisit arrived and motion is locked")
                else:
                    self._lock_until = time.monotonic() + 10.0
                    self._request_adapter_stop()
                    self._set_status("failed", f"Navigation exited with code {code}")
        except Exception as exc:
            self._lock_until = time.monotonic() + 30.0
            self._request_adapter_stop()
            self._set_status("failed", f"{type(exc).__name__}: {exc}")
        finally:
            with self._mutex:
                self._process = None

    def close(self) -> None:
        self._cancel.set()
        self._lock_until = time.monotonic() + 2.0
        self._request_adapter_stop()
        with self._mutex:
            process = self._process
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        self.node.destroy_node()


def main() -> int:
    import argparse
    import rclpy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=300.0)
    parser.add_argument("--robot-ip", default="192.168.123.161")
    args = parser.parse_args()
    if not 0 < args.timeout_s <= 900:
        parser.error("--timeout-s must be in (0, 900]")

    rclpy.init()
    service = RevisitOperatorService(
        repo_root=args.repo_root,
        state_path=args.state,
        timeout_s=args.timeout_s,
        robot_ip=args.robot_ip,
    )
    try:
        rclpy.spin(service.node)
    except KeyboardInterrupt:
        pass
    finally:
        service.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
