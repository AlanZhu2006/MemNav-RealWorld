"""Conservative steering for a slow vision-policy control loop.

The policy trajectory is expressed in the camera frame captured before the
remote planning request.  When that request takes close to a second, applying
a large angular command until the next result arrives can rotate the robot far
past the heading observed by the policy.  The next result then asks for the
opposite turn and produces the familiar left/right hunting pattern.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

from trajectory_control import VelocityCommand


@dataclass(frozen=True)
class LatencyMotionGuardConfig:
    enabled: bool = True
    # Do not let one planning interval rotate the body by more than 10 deg.
    max_open_loop_heading_rad: float = math.radians(10.0)
    max_plan_input_age_s: float = 1.50
    # With a slow plan, perform material steering corrections in place unless
    # a certified Go2 turn explicitly requires bounded forward creep.
    turn_in_place_after_s: float = 0.45
    turning_translation_cutoff_rps: float = 0.12
    # One opposite result only stops the previous turn.  A second consecutive
    # result must agree before the opposite turn is allowed.
    reversal_deadband_rps: float = 0.08
    reversal_confirmation_plans: int = 2


@dataclass(frozen=True)
class LatencyMotionGuardResult:
    command: VelocityCommand
    reason: str
    plan_input_age_s: float | None
    angular_limit_rps: float | None
    raw_angular_rps: float
    accepted_turn_sign: int
    pending_turn_sign: int
    pending_turn_plans: int

    def audit_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["command"] = asdict(self.command)
        return payload


def _with_motion(
    command: VelocityCommand, *, linear_x: float, angular_z: float
) -> VelocityCommand:
    return VelocityCommand(
        linear_x=float(linear_x),
        angular_z=float(angular_z),
        target_x=command.target_x,
        target_y=command.target_y,
        path_length=command.path_length,
        reverse=command.reverse,
    )


def _turn_sign(value: float, deadband: float) -> int:
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0


class LatencyMotionGuard:
    """Stateful plan-boundary guard; call exactly once per completed plan."""

    def __init__(
        self,
        config: LatencyMotionGuardConfig = LatencyMotionGuardConfig(),
    ) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._accepted_turn_sign = 0
        self._pending_turn_sign = 0
        self._pending_turn_plans = 0

    def _result(
        self,
        command: VelocityCommand,
        reason: str,
        age: float | None,
        limit: float | None,
        raw_angular: float,
    ) -> LatencyMotionGuardResult:
        return LatencyMotionGuardResult(
            command=command,
            reason=reason,
            plan_input_age_s=age,
            angular_limit_rps=limit,
            raw_angular_rps=raw_angular,
            accepted_turn_sign=self._accepted_turn_sign,
            pending_turn_sign=self._pending_turn_sign,
            pending_turn_plans=self._pending_turn_plans,
        )

    def apply(
        self,
        command: VelocityCommand,
        *,
        plan_input_age_s: float | None,
        max_angular_rps: float,
        preserve_turn_creep: bool = False,
    ) -> LatencyMotionGuardResult:
        raw_angular = float(command.angular_z)
        if not self.config.enabled:
            return self._result(
                command, "disabled", plan_input_age_s, max_angular_rps, raw_angular
            )

        try:
            age = float(plan_input_age_s)
        except (TypeError, ValueError, OverflowError):
            age = math.nan
        if not math.isfinite(age) or age < 0.0:
            stopped = _with_motion(command, linear_x=0.0, angular_z=0.0)
            return self._result(
                stopped, "invalid_plan_input_age_hold", None, None, raw_angular
            )
        if age > self.config.max_plan_input_age_s:
            stopped = _with_motion(command, linear_x=0.0, angular_z=0.0)
            return self._result(
                stopped, "plan_input_too_old_hold", age, 0.0, raw_angular
            )

        controller_limit = abs(float(max_angular_rps))
        latency_limit = self.config.max_open_loop_heading_rad / max(age, 1e-3)
        angular_limit = min(controller_limit, latency_limit)

        deadband = max(0.0, self.config.reversal_deadband_rps)
        requested_sign = _turn_sign(raw_angular, deadband)
        if requested_sign != 0 and self._accepted_turn_sign == 0:
            self._accepted_turn_sign = requested_sign
        elif requested_sign != 0 and requested_sign != self._accepted_turn_sign:
            if requested_sign == self._pending_turn_sign:
                self._pending_turn_plans += 1
            else:
                self._pending_turn_sign = requested_sign
                self._pending_turn_plans = 1
            required = max(1, int(self.config.reversal_confirmation_plans))
            if self._pending_turn_plans < required:
                stopped = _with_motion(command, linear_x=0.0, angular_z=0.0)
                return self._result(
                    stopped,
                    "turn_reversal_confirmation_hold",
                    age,
                    angular_limit,
                    raw_angular,
                )
            self._accepted_turn_sign = requested_sign
            self._pending_turn_sign = 0
            self._pending_turn_plans = 0
        elif requested_sign == self._accepted_turn_sign:
            self._pending_turn_sign = 0
            self._pending_turn_plans = 0

        angular = max(-angular_limit, min(angular_limit, raw_angular))
        linear = float(command.linear_x)
        reason = "pass"
        if not math.isclose(angular, raw_angular, abs_tol=1e-9):
            reason = "latency_limited_turn"
        if (
            age >= self.config.turn_in_place_after_s
            and abs(raw_angular) >= self.config.turning_translation_cutoff_rps
            and abs(linear) > 0.0
            and not preserve_turn_creep
        ):
            linear = 0.0
            reason = "stale_plan_turn_in_place"
        guarded = _with_motion(command, linear_x=linear, angular_z=angular)
        return self._result(guarded, reason, age, angular_limit, raw_angular)


__all__ = [
    "LatencyMotionGuard",
    "LatencyMotionGuardConfig",
    "LatencyMotionGuardResult",
]
