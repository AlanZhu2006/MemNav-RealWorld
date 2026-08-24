"""Role-free handoff from navigation to certified direct bearing control.

Long-range CEC can address a Revisit goal through causal history, while native
NavDP can approach a Novel goal directly.  Once the current and goal views are
geometrically covisible, the same two-view proof supplies a much more local
relative direction.  It does *not* certify monocular metric scale or arrival.

This module therefore grants only the authority supported by the evidence:

* a certified direction may request one bounded atomic turn;
* a direction within NavDP's measured point-token support is projected onto
  the already validated 2.5 m scale-free residual;
* proof loss returns to the preceding route (native or long-range CEC);
* STOP remains fail-closed until an independent visual-convergence proof is
  implemented and validated.

No semantic Novel/Revisit label is consumed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


SCHEMA_VERSION = "cec_direct_bearing_handoff_v2_20260824"

# Frozen from the measured NavDP point-token transfer function: injected
# bearings remain faithful through +/-60 degrees, while rearward targets can
# collapse.  Outside this support the actuator layer turns atomically first.
NAVDP_POINT_TOKEN_MAX_ABS_BEARING_DEG = 60.0

# The deployed CEC projection already validated this residual length.  The
# magnitude of a monocular PnP translation is deliberately ignored here.
CERTIFIED_BEARING_RESIDUAL_M = 2.5


@dataclass(frozen=True)
class LocalPoseHandoffDecision:
    disposition: str
    direct_proof_active: bool
    local_latched: bool
    reason: str
    controller_pointgoal_m: tuple[float, float] | None
    turn_error_left_rad: float | None
    predicted_distance_m: float | None
    predicted_bearing_deg: float | None
    terminal_yaw_right_deg: float | None
    stop_streak: int
    stop_authorized: bool

    def audit_dict(self) -> dict[str, Any]:
        return {
            "terminal_handoff_schema": SCHEMA_VERSION,
            "terminal_handoff_disposition": self.disposition,
            "terminal_local_latched": self.local_latched,
            "terminal_handoff_reason": self.reason,
            "terminal_controller_pointgoal_m": (
                list(self.controller_pointgoal_m)
                if self.controller_pointgoal_m is not None
                else None
            ),
            "terminal_turn_error_left_rad": self.turn_error_left_rad,
            # Retained as a diagnostic only.  It has no control or STOP
            # authority under this schema.
            "terminal_predicted_distance_m": self.predicted_distance_m,
            "terminal_predicted_distance_control_authority": False,
            "terminal_predicted_bearing_deg": self.predicted_bearing_deg,
            "terminal_yaw_right_deg": self.terminal_yaw_right_deg,
            "terminal_stop_streak": self.stop_streak,
            "terminal_stop_authorized": self.stop_authorized,
            "terminal_proof_active": self.direct_proof_active,
            "terminal_point_token_support_deg": (
                NAVDP_POINT_TOKEN_MAX_ABS_BEARING_DEG
            ),
            "terminal_bearing_residual_m": CERTIFIED_BEARING_RESIDUAL_M,
            "terminal_metric_scale_control_authority": False,
            "terminal_stop_authority": (
                "none_until_independent_visual_convergence"
            ),
        }


def _finite_number(value: object) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


def _finite_pair(value: object) -> tuple[float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    x = _finite_number(value[0])
    y = _finite_number(value[1])
    if x is None or y is None or math.hypot(x, y) <= 1e-8:
        return None
    return x, y


def _certified_direction(
    evidence: Mapping[str, object] | None,
) -> tuple[tuple[float, float], float | None, float | None] | None:
    """Return direction plus diagnostic distance/yaw from a valid proof.

    New servers expose the raw scale-free vector explicitly.  The metric
    vector is accepted only as a backward-compatible source of direction; its
    magnitude is never sent to the controller and never authorizes STOP.
    """

    if not isinstance(evidence, Mapping):
        return None
    if evidence.get("certificate_accepted") is not True:
        return None
    point = _finite_pair(evidence.get("predicted_scale_free_relative_xy"))
    if point is None:
        point = _finite_pair(evidence.get("predicted_relative_xy_m"))
    if point is None:
        return None
    distance = _finite_number(evidence.get("predicted_distance_m"))
    yaw_right = _finite_number(evidence.get("terminal_yaw_right_deg"))
    return point, distance, yaw_right


def decide_local_pose_handoff(
    *,
    long_range_available: bool,
    evidence: Mapping[str, object] | None,
    local_latched: bool = False,
    stop_streak: int = 0,
) -> LocalPoseHandoffDecision:
    """Choose native, long-range, scale-free bearing, or atomic turn.

    ``local_latched`` and ``stop_streak`` remain in the call signature so a
    rolling deployment can consume v1 router state safely.  V2 deliberately
    clears both because direct PnP has no independently validated arrival
    authority.
    """

    if type(long_range_available) is not bool or type(local_latched) is not bool:
        raise TypeError("route and latch states must be bool")
    if type(stop_streak) is not int or stop_streak < 0:
        raise ValueError("stop_streak must be a non-negative integer")

    def result(
        disposition: str,
        proof: bool,
        reason: str,
        *,
        pointgoal: tuple[float, float] | None = None,
        turn: float | None = None,
        distance: float | None = None,
        bearing: float | None = None,
        yaw_right: float | None = None,
    ) -> LocalPoseHandoffDecision:
        return LocalPoseHandoffDecision(
            disposition=disposition,
            direct_proof_active=proof,
            local_latched=False,
            reason=reason,
            controller_pointgoal_m=pointgoal,
            turn_error_left_rad=turn,
            predicted_distance_m=distance,
            predicted_bearing_deg=bearing,
            terminal_yaw_right_deg=yaw_right,
            stop_streak=0,
            stop_authorized=False,
        )

    direct = _certified_direction(evidence)
    if direct is None:
        if long_range_available:
            return result(
                "long_range", False, "direct_bearing_certificate_unavailable"
            )
        return result(
            "native", False, "direct_bearing_certificate_unavailable"
        )

    point, diagnostic_distance, yaw_right = direct
    norm = math.hypot(point[0], point[1])
    unit = point[0] / norm, point[1] / norm
    bearing_rad = math.atan2(unit[1], unit[0])
    bearing_deg = math.degrees(bearing_rad)

    if abs(bearing_deg) > NAVDP_POINT_TOKEN_MAX_ABS_BEARING_DEG:
        return result(
            "atomic_turn",
            True,
            "direct_bearing_outside_point_token_support",
            turn=bearing_rad,
            distance=diagnostic_distance,
            bearing=bearing_deg,
            yaw_right=yaw_right,
        )

    return result(
        "bearing_local",
        True,
        "direct_scale_free_bearing_certified",
        pointgoal=(
            CERTIFIED_BEARING_RESIDUAL_M * unit[0],
            CERTIFIED_BEARING_RESIDUAL_M * unit[1],
        ),
        distance=diagnostic_distance,
        bearing=bearing_deg,
        yaw_right=yaw_right,
    )


__all__ = [
    "CERTIFIED_BEARING_RESIDUAL_M",
    "LocalPoseHandoffDecision",
    "NAVDP_POINT_TOKEN_MAX_ABS_BEARING_DEG",
    "SCHEMA_VERSION",
    "decide_local_pose_handoff",
]
