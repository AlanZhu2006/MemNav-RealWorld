"""Fresh-plan validation and event-driven stop-plan-act execution."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

from trajectory_control import VelocityCommand


@dataclass(frozen=True)
class LatencyMotionGuardConfig:
    enabled: bool = True
    max_plan_input_age_s: float = 1.50


@dataclass(frozen=True)
class LatencyMotionGuardResult:
    command: VelocityCommand
    reason: str
    plan_input_age_s: float | None
    raw_linear_mps: float
    raw_angular_rps: float

    def audit_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["command"] = asdict(self.command)
        return payload


def _stopped(command: VelocityCommand) -> VelocityCommand:
    return VelocityCommand(
        target_x=command.target_x,
        target_y=command.target_y,
        path_length=command.path_length,
        reverse=command.reverse,
    )


class LatencyMotionGuard:
    """Reject trajectories inferred from an excessively old observation.

    Steering magnitude is bounded by the controller and StopPlanActGate's
    integrated action budget. It is intentionally not filtered across plans:
    every plan is produced from a new observation captured while stationary,
    so a two-plan reversal confirmation would only insert a dead cycle.
    """

    def __init__(
        self,
        config: LatencyMotionGuardConfig = LatencyMotionGuardConfig(),
    ) -> None:
        self.config = config

    def reset(self) -> None:
        """Retained for callers that reset all navigation guards."""

    def apply(
        self,
        command: VelocityCommand,
        *,
        plan_input_age_s: float | None,
    ) -> LatencyMotionGuardResult:
        raw_linear = float(command.linear_x)
        raw_angular = float(command.angular_z)
        if not self.config.enabled:
            return LatencyMotionGuardResult(
                command, "disabled", plan_input_age_s, raw_linear, raw_angular
            )

        try:
            age = float(plan_input_age_s)
        except (TypeError, ValueError, OverflowError):
            age = math.nan
        if not math.isfinite(age) or age < 0.0:
            return LatencyMotionGuardResult(
                _stopped(command),
                "invalid_plan_input_age_hold",
                None,
                raw_linear,
                raw_angular,
            )
        if age > self.config.max_plan_input_age_s:
            return LatencyMotionGuardResult(
                _stopped(command),
                "plan_input_too_old_hold",
                age,
                raw_linear,
                raw_angular,
            )
        return LatencyMotionGuardResult(
            command, "pass", age, raw_linear, raw_angular
        )


@dataclass(frozen=True)
class StopPlanActConfig:
    enabled: bool = True
    max_execution_s: float = 0.80
    max_translation_m: float = 0.10
    max_heading_rad: float = math.radians(10.0)
    settle_before_sense_s: float = 0.15


class StopPlanActGate:
    """Execute one measured command pulse, stop, then admit a new frame.

    The action clock starts at the first command actually published, not when
    inference finishes. Translation and heading budgets integrate published
    velocity, with a wall-clock maximum as a final bound. After zero is
    published, a plan is admitted only from an RGB-D pair whose capture
    timestamp is newer than that stop; callback arrival time is insufficient
    because it can describe a queued pre-stop frame.
    """

    def __init__(self, config: StopPlanActConfig = StopPlanActConfig()) -> None:
        values = (
            config.max_execution_s,
            config.max_translation_m,
            config.max_heading_rad,
            config.settle_before_sense_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("stop-plan-act limits must be finite")
        if config.max_execution_s <= 0.0:
            raise ValueError("maximum execution time must be positive")
        if config.max_translation_m <= 0.0:
            raise ValueError("translation budget must be positive")
        if config.max_heading_rad <= 0.0:
            raise ValueError("heading budget must be positive")
        if config.settle_before_sense_s < 0.0:
            raise ValueError("settle time must be nonnegative")
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._phase = "need_plan"
        self._plan_completed_s = 0.0
        self._action_started_s: float | None = None
        self._last_sample_s: float | None = None
        self._last_linear_mps = 0.0
        self._last_angular_rps = 0.0
        self._translation_m = 0.0
        self._heading_rad = 0.0
        self._stopped_s: float | None = None
        self._stopped_ros_ns: int | None = None
        self._sense_after_ros_ns: int | None = None
        self._completion_reason = ""

    def install_plan(self, plan_completed_s: float) -> None:
        completed = float(plan_completed_s)
        if not math.isfinite(completed) or completed <= 0.0:
            raise ValueError("plan completion time must be finite and positive")
        self._phase = "ready_to_execute"
        self._plan_completed_s = completed
        self._action_started_s = None
        self._last_sample_s = None
        self._last_linear_mps = 0.0
        self._last_angular_rps = 0.0
        self._translation_m = 0.0
        self._heading_rad = 0.0
        self._stopped_s = None
        self._stopped_ros_ns = None
        self._sense_after_ros_ns = None
        self._completion_reason = ""

    @staticmethod
    def _has_motion(command: VelocityCommand) -> bool:
        return (
            abs(float(command.linear_x)) > 1e-6
            or abs(float(command.angular_z)) > 1e-6
        )

    def _integrate_until(self, now_s: float) -> None:
        if self._phase != "execute" or self._last_sample_s is None:
            return
        now = max(float(now_s), self._last_sample_s)
        dt = now - self._last_sample_s
        self._translation_m += abs(self._last_linear_mps) * dt
        self._heading_rad += abs(self._last_angular_rps) * dt
        self._last_sample_s = now

    def _update_completion(self, now_s: float) -> None:
        if self._phase != "execute" or self._action_started_s is None:
            return
        self._integrate_until(now_s)
        elapsed = max(0.0, float(now_s) - self._action_started_s)
        if self._translation_m >= self.config.max_translation_m:
            self._completion_reason = "translation_budget"
        elif self._heading_rad >= self.config.max_heading_rad:
            self._completion_reason = "heading_budget"
        elif elapsed >= self.config.max_execution_s:
            self._completion_reason = "execution_timeout"
        else:
            return
        self._phase = "stop_pending"
        self._last_linear_mps = 0.0
        self._last_angular_rps = 0.0

    def note_command_published(
        self, now_s: float, command: VelocityCommand
    ) -> str:
        if not self.config.enabled:
            return "continuous"
        now = float(now_s)
        if self._phase == "ready_to_execute":
            if not self._has_motion(command):
                self._phase = "stop_pending"
                self._completion_reason = "zero_command"
                return self._phase
            self._phase = "execute"
            self._action_started_s = now
            self._last_sample_s = now
        elif self._phase == "execute":
            self._integrate_until(now)
        else:
            return self._phase
        self._last_linear_mps = float(command.linear_x)
        self._last_angular_rps = float(command.angular_z)
        self._update_completion(now)
        return self._phase

    def note_action_stopped(self, now_s: float, stopped_ros_ns: int | None) -> None:
        if not self.config.enabled:
            return
        if self._phase not in {"ready_to_execute", "execute", "stop_pending"}:
            return
        self._integrate_until(now_s)
        self._phase = "settling"
        self._stopped_s = float(now_s)
        self._stopped_ros_ns = (
            int(stopped_ros_ns)
            if isinstance(stopped_ros_ns, int) and stopped_ros_ns > 0
            else None
        )
        self._sense_after_ros_ns = (
            None
            if self._stopped_ros_ns is None
            else self._stopped_ros_ns
            + int(math.ceil(self.config.settle_before_sense_s * 1e9))
        )
        self._last_linear_mps = 0.0
        self._last_angular_rps = 0.0

    def phase(self, *, now_s: float, latest_rgbd_source_ns: int | None) -> str:
        if not self.config.enabled:
            return "continuous"
        self._update_completion(now_s)
        if self._phase == "settling":
            assert self._stopped_s is not None
            if float(now_s) - self._stopped_s < self.config.settle_before_sense_s:
                return "settling"
            self._phase = "waiting_for_post_stop_rgbd"
        if self._phase == "waiting_for_post_stop_rgbd":
            source_ns = (
                int(latest_rgbd_source_ns)
                if isinstance(latest_rgbd_source_ns, int)
                else 0
            )
            if (
                self._sense_after_ros_ns is not None
                and source_ns > self._sense_after_ros_ns
            ):
                self._phase = "ready_to_plan"
        return self._phase

    @staticmethod
    def motion_allowed(phase: str) -> bool:
        return phase in {"continuous", "ready_to_execute", "execute"}

    @staticmethod
    def planning_allowed(phase: str) -> bool:
        return phase in {"need_plan", "ready_to_plan"}

    def audit_dict(
        self, *, now_s: float, latest_rgbd_source_ns: int | None
    ) -> dict[str, Any]:
        phase = self.phase(
            now_s=now_s,
            latest_rgbd_source_ns=latest_rgbd_source_ns,
        )
        elapsed_s = (
            None
            if self._action_started_s is None
            else max(0.0, float(now_s) - self._action_started_s)
        )
        return {
            "phase": phase,
            "completion_reason": self._completion_reason or None,
            "action_elapsed_s": elapsed_s,
            "integrated_translation_m": self._translation_m,
            "integrated_heading_rad": self._heading_rad,
            "stopped_ros_ns": self._stopped_ros_ns,
            "sense_after_ros_ns": self._sense_after_ros_ns,
        }


__all__ = [
    "LatencyMotionGuard",
    "LatencyMotionGuardConfig",
    "LatencyMotionGuardResult",
    "StopPlanActConfig",
    "StopPlanActGate",
]
