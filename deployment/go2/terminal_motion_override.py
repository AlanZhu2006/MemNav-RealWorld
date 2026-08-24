"""Robot-side execution boundary for certified terminal handoff receipts."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping

from trajectory_control import VelocityCommand


EXPECTED_HANDOFF_SCHEMA = "cec_direct_bearing_handoff_v2_20260824"


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


def _finite(value: object) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


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
    if disposition in (None, "long_range", "bearing_local") and not latched:
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
            VelocityCommand(angular_z=angular),
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
    "EXPECTED_HANDOFF_SCHEMA",
    "TerminalMotionOverride",
    "terminal_motion_override",
]
