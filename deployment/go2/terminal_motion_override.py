"""Robot-side execution boundary for certified terminal handoff receipts."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping

from trajectory_control import VelocityCommand


EXPECTED_HANDOFF_SCHEMA = "cec_direct_bearing_handoff_v2_20260824"
EXPECTED_POINT_TOKEN_SUPPORT_DEG = 60.0
POINT_TOKEN_HANDOFF_MARGIN_DEG = 5.0
# This Go2 has repeatedly failed to enter locomotion for a pure-yaw Move.
# Start certified turns with one short, depth-protected forward pulse at the
# smallest known effective speed, then retain that proven locomotion floor.
CERTIFIED_TURN_CREEP_MPS = 0.10
CERTIFIED_TURN_MAINTENANCE_CREEP_MPS = 0.08
CERTIFIED_TURN_REASONS = frozenset(
    {"certified_atomic_turn", "certified_long_range_atomic_turn"}
)


@dataclass(frozen=True)
class TerminalMotionOverride:
    applied: bool
    command: VelocityCommand | None
    assert_estop: bool
    reason: str

    def audit_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        if self.command is not None:
            payload["command"] = asdict(self.command)
        return payload


@dataclass(frozen=True)
class CertifiedTurnBootstrapResult:
    command: VelocityCommand
    phase: str
    elapsed_s: float | None

    def audit_dict(self) -> dict[str, Any]:
        return {
            "command": asdict(self.command),
            "phase": self.phase,
            "elapsed_s": self.elapsed_s,
        }


class CertifiedTurnBootstrap:
    """Use a gait-start pulse, bounded creep, and an active-time watchdog."""

    def __init__(self, duration_s: float = 0.60, max_duration_s: float = 20.0) -> None:
        if not math.isfinite(duration_s) or duration_s < 0.0:
            raise ValueError(
                "certified turn bootstrap duration must be finite and nonnegative"
            )
        if not math.isfinite(max_duration_s) or max_duration_s <= 0.0:
            raise ValueError(
                "certified turn maximum duration must be finite and positive"
            )
        if max_duration_s < duration_s:
            raise ValueError(
                "certified turn maximum duration must cover the bootstrap"
            )
        self.duration_s = float(duration_s)
        self.max_duration_s = float(max_duration_s)
        self._key: tuple[str, int] | None = None
        self._active_elapsed_s = 0.0

    def reset(self) -> None:
        self._key = None
        self._active_elapsed_s = 0.0

    def _select_turn(self, reason: str, angular_z: float) -> bool:
        if reason not in CERTIFIED_TURN_REASONS or abs(angular_z) <= 0.0:
            self.reset()
            return False
        turn_sign = 1 if angular_z > 0.0 else -1
        key = (reason, turn_sign)
        if key != self._key:
            self._key = key
            self._active_elapsed_s = 0.0
        return True

    def apply(
        self,
        command: VelocityCommand,
        *,
        reason: str,
        motion_allowed: bool,
        now_s: float,
    ) -> CertifiedTurnBootstrapResult:
        if not self._select_turn(reason, command.angular_z):
            return CertifiedTurnBootstrapResult(command, "inactive", None)
        elapsed_s = self._active_elapsed_s
        if not motion_allowed:
            phase = "inactive" if elapsed_s <= 0.0 else "paused"
            return CertifiedTurnBootstrapResult(command, phase, elapsed_s)
        if elapsed_s >= self.max_duration_s:
            return CertifiedTurnBootstrapResult(
                VelocityCommand(), "turn_timeout", elapsed_s
            )
        if elapsed_s < self.duration_s:
            return CertifiedTurnBootstrapResult(command, "gait_bootstrap", elapsed_s)

        maintenance = VelocityCommand(
            linear_x=CERTIFIED_TURN_MAINTENANCE_CREEP_MPS,
            angular_z=command.angular_z,
            target_x=command.target_x,
            target_y=command.target_y,
            path_length=command.path_length,
            reverse=command.reverse,
        )
        return CertifiedTurnBootstrapResult(
            maintenance, "maintenance_creep", elapsed_s
        )

    def record_execution(
        self,
        command: VelocityCommand,
        *,
        reason: str,
        dt_s: float,
    ) -> None:
        """Count only a certified turn command that was actually published."""

        if reason not in CERTIFIED_TURN_REASONS:
            return
        if abs(float(command.angular_z)) <= 1e-6:
            return
        turn_sign = 1 if command.angular_z > 0.0 else -1
        if self._key != (reason, turn_sign):
            return
        dt = float(dt_s)
        if math.isfinite(dt) and dt > 0.0:
            self._active_elapsed_s += dt


def _finite(value: object) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


def _finite_direction(value: object) -> tuple[float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    x = _finite(value[0])
    y = _finite(value[1])
    if x is None or y is None:
        return None
    norm = math.hypot(x, y)
    if norm <= 1e-8:
        return None
    return x / norm, y / norm


def terminal_motion_override(
    receipt: Mapping[str, object] | None,
    *,
    rotate_gain: float,
    max_angular_rps: float,
) -> TerminalMotionOverride:
    """Translate a server proof disposition into one bounded Go2 command.

    The hub cannot actuate the robot.  This boundary recognizes only the
    versioned handoff receipt and requires a certified direct-local proof;
    that proof may be reached from native Novel control or long-range CEC.
    Turns use Go2's positive-angular-z-is-left convention.  Any malformed
    disposition after local handoff fails closed to a zero command.
    """

    if not isinstance(receipt, Mapping):
        return TerminalMotionOverride(False, None, False, "no_terminal_receipt")
    if (
        receipt.get("terminal_handoff_schema") != EXPECTED_HANDOFF_SCHEMA
        or (
            receipt.get("terminal_proof_active") is not True
            and receipt.get("cec_takeover") is not True
        )
    ):
        return TerminalMotionOverride(False, None, False, "terminal_receipt_ineligible")

    disposition = receipt.get("terminal_handoff_disposition")
    latched = receipt.get("terminal_local_latched") is True
    if disposition == "long_range" and not latched:
        # Frozen NavDP clips a negative forward PointGoal component to zero
        # and its measured point-token transfer support is only +/-60 degrees.
        # A certified long-range bearing outside that support must therefore
        # turn at the actuator boundary before the mixed policy can consume it.
        certificate = receipt.get("cec_certificate")
        support_deg = _finite(receipt.get("terminal_point_token_support_deg"))
        direction = _finite_direction(receipt.get("memory_bearing_unit"))
        if (
            not isinstance(certificate, Mapping)
            or certificate.get("accepted") is not True
            or support_deg is None
            or not math.isclose(
                support_deg, EXPECTED_POINT_TOKEN_SUPPORT_DEG, abs_tol=1e-6
            )
            or direction is None
        ):
            return TerminalMotionOverride(
                True,
                VelocityCommand(),
                False,
                "invalid_long_range_turn_receipt",
            )
        bearing_rad = math.atan2(direction[1], direction[0])
        handoff_deg = support_deg - POINT_TOKEN_HANDOFF_MARGIN_DEG
        if abs(math.degrees(bearing_rad)) > handoff_deg:
            gain = _finite(rotate_gain)
            limit = _finite(max_angular_rps)
            if gain is None or limit is None or gain <= 0.0 or limit <= 0.0:
                return TerminalMotionOverride(
                    True,
                    VelocityCommand(),
                    False,
                    "invalid_long_range_turn_limits",
                )
            angular = max(-limit, min(limit, gain * bearing_rad))
            return TerminalMotionOverride(
                True,
                VelocityCommand(
                    linear_x=CERTIFIED_TURN_CREEP_MPS,
                    angular_z=angular,
                ),
                False,
                "certified_long_range_atomic_turn",
            )
        return TerminalMotionOverride(
            False, None, False, "long_range_inside_point_token_support"
        )
    if disposition in (None, "bearing_local") and not latched:
        return TerminalMotionOverride(False, None, False, "long_range_controller")
    if disposition == "bearing_local":
        return TerminalMotionOverride(False, None, False, "bearing_local_controller")

    if disposition == "atomic_turn":
        error = _finite(receipt.get("terminal_turn_error_left_rad"))
        gain = _finite(rotate_gain)
        limit = _finite(max_angular_rps)
        if error is None or gain is None or limit is None or gain <= 0.0 or limit <= 0.0:
            return TerminalMotionOverride(
                True, VelocityCommand(), False, "invalid_atomic_turn_receipt"
            )
        angular = max(-limit, min(limit, gain * error))
        return TerminalMotionOverride(
            True,
            VelocityCommand(
                linear_x=CERTIFIED_TURN_CREEP_MPS,
                angular_z=angular,
            ),
            False,
            "certified_atomic_turn",
        )

    if disposition == "stop" and receipt.get("terminal_stop_authorized") is True:
        return TerminalMotionOverride(
            True, VelocityCommand(), True, "certified_terminal_stop"
        )
    if disposition == "hold" or latched:
        return TerminalMotionOverride(
            True, VelocityCommand(), False, "certified_terminal_hold"
        )
    return TerminalMotionOverride(False, None, False, "long_range_controller")


__all__ = [
    "CERTIFIED_TURN_REASONS",
    "CERTIFIED_TURN_CREEP_MPS",
    "CERTIFIED_TURN_MAINTENANCE_CREEP_MPS",
    "CertifiedTurnBootstrap",
    "CertifiedTurnBootstrapResult",
    "EXPECTED_POINT_TOKEN_SUPPORT_DEG",
    "EXPECTED_HANDOFF_SCHEMA",
    "POINT_TOKEN_HANDOFF_MARGIN_DEG",
    "TerminalMotionOverride",
    "terminal_motion_override",
]
