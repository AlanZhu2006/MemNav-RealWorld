"""Role-free handoff from navigation to certified direct bearing control.

Long-range CEC can address a Revisit goal through causal history, while native
NavDP can approach a Novel goal directly.  Once the current and goal views are
geometrically covisible, the same two-view proof supplies a much more local
relative direction.  A separately bound first-40 camera-height receipt may
certify its metric scale for bounded translation, but never arrival.

This module therefore grants only the authority supported by the evidence:

* a certified direction may request one bounded atomic turn;
* a direction within NavDP's measured point-token support uses the exact
  first-40 camera-height metric receipt when that receipt is bound to the
  current RGB/depth transaction;
* missing or inconsistent metric evidence falls back to the already validated
  2.5 m scale-free residual;
* proof loss returns to the preceding route (native or long-range CEC);
* STOP remains fail-closed until an independent visual-convergence proof is
  implemented and validated.

No semantic Novel/Revisit label is consumed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


SCHEMA_VERSION = "cec_direct_metric_handoff_v3_20260831"

# Frozen from the measured NavDP point-token transfer function: injected
# bearings remain faithful through +/-60 degrees, while rearward targets can
# collapse.  Outside this support the actuator layer turns atomically first.
NAVDP_POINT_TOKEN_MAX_ABS_BEARING_DEG = 60.0

# The deployed CEC projection already validated this fallback residual length.
# It is used whenever the separately bound metric-scale contract is absent.
CERTIFIED_BEARING_RESIDUAL_M = 2.5
METRIC_POINTGOAL_STEP_CAP_M = 0.8


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
    controller_distance_m: float | None
    metric_scale_control_authority: bool
    metric_scale_receipt_sha256: str | None

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
            # authority unless the validated first-40 receipt fields below
            # explicitly grant bounded PointGoal authority.
            "terminal_predicted_distance_m": self.predicted_distance_m,
            "terminal_predicted_distance_control_authority": (
                self.metric_scale_control_authority),
            "terminal_predicted_bearing_deg": self.predicted_bearing_deg,
            "terminal_yaw_right_deg": self.terminal_yaw_right_deg,
            "terminal_stop_streak": self.stop_streak,
            "terminal_stop_authorized": self.stop_authorized,
            "terminal_proof_active": self.direct_proof_active,
            "terminal_point_token_support_deg": (
                NAVDP_POINT_TOKEN_MAX_ABS_BEARING_DEG
            ),
            "terminal_bearing_residual_m": (
                None
                if self.metric_scale_control_authority
                else CERTIFIED_BEARING_RESIDUAL_M
            ),
            "terminal_controller_pointgoal_distance_m": (
                self.controller_distance_m),
            "terminal_metric_pointgoal_step_cap_m": (
                METRIC_POINTGOAL_STEP_CAP_M),
            "terminal_metric_scale_control_authority": (
                self.metric_scale_control_authority),
            "terminal_metric_scale_receipt_sha256": (
                self.metric_scale_receipt_sha256),
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
) -> tuple[
    tuple[float, float],
    float | None,
    float | None,
    tuple[float, float] | None,
    str | None,
] | None:
    """Return direction plus diagnostic distance/yaw from a valid proof.

    New servers expose the raw scale-free vector explicitly.  The metric
    vector is accepted for bounded controller distance only when its immutable
    first-40 camera-height receipt and its alignment with the scale-free vector
    validate.  Metric distance still never authorizes STOP.
    """

    if not isinstance(evidence, Mapping):
        return None
    if evidence.get("certificate_accepted") is not True:
        return None
    scale_free = _finite_pair(
        evidence.get("predicted_scale_free_relative_xy")
    )
    point = scale_free
    if point is None:
        point = _finite_pair(evidence.get("predicted_relative_xy_m"))
    if point is None:
        return None
    distance = _finite_number(evidence.get("predicted_distance_m"))
    yaw_right = _finite_number(evidence.get("terminal_yaw_right_deg"))
    metric_point = None
    scale_receipt_sha256 = None
    scale = evidence.get("metric_scale")
    candidate_metric = _finite_pair(evidence.get("predicted_relative_xy_m"))
    if (
        scale_free is not None
        and candidate_metric is not None
        and evidence.get("metric_scale_available") is True
        and evidence.get("metric_scale_policy") == "mdtec_first40"
        and evidence.get("metric_scale_transaction_bound") is True
        and isinstance(scale, Mapping)
        and scale.get("available") is True
        and scale.get("reason") == "mdtec_first40_causal_scale_available"
        and scale.get("scale_evidence_contract")
        == "causal_first_prefix_rgb_only_v1"
        and type(scale.get("frame_count")) is int
        and scale.get("frame_count") == 40
    ):
        scale_value = _finite_number(scale.get("metric_scale_m_per_raw"))
        digest = str(scale.get("scale_receipt_sha256", ""))
        metric_norm = math.hypot(*candidate_metric)
        reported_distance = _finite_number(
            evidence.get("predicted_distance_m")
        )
        scale_norm = math.hypot(*scale_free)
        aligned = (
            scale_free[0] * candidate_metric[0]
            + scale_free[1] * candidate_metric[1]
        ) / (scale_norm * metric_norm)
        distance_tolerance = max(1e-4, 0.01 * metric_norm)
        scale_tolerance = max(1e-5, 0.01 * scale_value) if (
            scale_value is not None
        ) else 0.0
        if (
            scale_value is not None
            and scale_value > 0.0
            and len(digest) == 64
            and all(character in "0123456789abcdef" for character in digest)
            and reported_distance is not None
            and abs(reported_distance - metric_norm) <= distance_tolerance
            and abs(metric_norm / scale_norm - scale_value)
            <= scale_tolerance
            and aligned >= 0.999
        ):
            metric_point = candidate_metric
            scale_receipt_sha256 = digest
    return (
        point,
        distance,
        yaw_right,
        metric_point,
        scale_receipt_sha256,
    )


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
        controller_distance: float | None = None,
        metric_scale_authority: bool = False,
        metric_scale_receipt_sha256: str | None = None,
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
            controller_distance_m=controller_distance,
            metric_scale_control_authority=metric_scale_authority,
            metric_scale_receipt_sha256=metric_scale_receipt_sha256,
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

    (
        point,
        diagnostic_distance,
        yaw_right,
        metric_point,
        scale_receipt_sha256,
    ) = direct
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

    if metric_point is not None:
        metric_norm = math.hypot(*metric_point)
        metric_unit = (
            metric_point[0] / metric_norm,
            metric_point[1] / metric_norm,
        )
        controller_distance = min(
            metric_norm, METRIC_POINTGOAL_STEP_CAP_M
        )
        return result(
            "bearing_local",
            True,
            "direct_camera_height_metric_bounded_step",
            pointgoal=(
                metric_unit[0] * controller_distance,
                metric_unit[1] * controller_distance,
            ),
            distance=diagnostic_distance,
            bearing=bearing_deg,
            yaw_right=yaw_right,
            controller_distance=controller_distance,
            metric_scale_authority=True,
            metric_scale_receipt_sha256=scale_receipt_sha256,
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
        controller_distance=CERTIFIED_BEARING_RESIDUAL_M,
    )


__all__ = [
    "CERTIFIED_BEARING_RESIDUAL_M",
    "LocalPoseHandoffDecision",
    "NAVDP_POINT_TOKEN_MAX_ABS_BEARING_DEG",
    "METRIC_POINTGOAL_STEP_CAP_M",
    "SCHEMA_VERSION",
    "decide_local_pose_handoff",
]
