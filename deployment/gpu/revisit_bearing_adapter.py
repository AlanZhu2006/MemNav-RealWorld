"""Fail-closed controller interface for verified Revisit memory evidence.

The memory system is allowed to decide *whether* a revisit match is verified
and to estimate its camera-relative direction.  An uncalibrated translation
is never exposed to the canonical policy: ``verified_bearing_v1`` projects it
onto one frozen radius.  The separate ``verified_metric_step_v1`` path accepts
only a metre vector whose first-40 camera-height receipt has already been
validated and bound by the hub, then limits one receding-horizon request.

This module is intentionally independent of Habitat, Torch, Flask, and NavDP.
It is the auditable boundary between a direction source and a controller.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence


REVISIT_ADAPTER_SCHEMA_VERSION = 3
REVISIT_ADAPTER_MODES = (
    "legacy_metric",
    "navdp_front_support_v1",
    "raw_fixed_bearing_v1",
    "verified_bearing_v1",
    "verified_metric_step_v1",
)
FIXED_BEARING_MODES = frozenset({
    "raw_fixed_bearing_v1",
    "verified_bearing_v1",
})
POINTGOAL_UNITS = (
    "metric_m",
    "lingbot_raw_direction_only",
    "pi3x_current_camera_direction_only",
)

# B0 froze this value before its paired rollout: the episode-balanced median
# first-active memory radius was 2.513 m and was rounded to 2.5 m.  It is a
# semantic constant, not an evaluation-time hyperparameter.
VERIFIED_BEARING_RADIUS_M = 2.5
# A valid first-40 camera-height receipt may recover metric translation, but
# one monocular estimate must never ask NavDP to consume the whole remaining
# route in one local token.  Receding-horizon control re-estimates after every
# observation and caps only the per-plan PointGoal radius.
VERIFIED_METRIC_STEP_CAP_M = 0.8
ZERO_BEARING_EPS = 1e-12


@dataclass(frozen=True)
class RevisitAdapterDecision:
    """One source-to-controller decision with complete audit provenance."""

    mode: str
    source: str
    takeover: bool
    reason: str
    controller_contract: str
    raw_pointgoal: tuple[float, float] | None
    raw_pointgoal_units: str
    unit_bearing: tuple[float, float] | None
    controller_pointgoal: tuple[float, float] | None
    raw_pointgoal_norm: float | None
    raw_distance_m: float | None
    controller_distance_m: float | None
    metric_scale_control_authority: bool = False
    controller_step_cap_m: float | None = None

    def audit_dict(self) -> dict[str, Any]:
        """Return stable JSON-compatible fields for every planning step."""

        return {
            "revisit_adapter_schema_version": (
                REVISIT_ADAPTER_SCHEMA_VERSION),
            "revisit_adapter_mode": self.mode,
            "revisit_adapter_source": self.source,
            "revisit_adapter_takeover": self.takeover,
            "revisit_adapter_reason": self.reason,
            "revisit_adapter_controller_contract": (
                self.controller_contract),
            "memory_unbounded_pointgoal": (
                list(self.raw_pointgoal)
                if self.raw_pointgoal is not None else None),
            "memory_unbounded_pointgoal_units": self.raw_pointgoal_units,
            "memory_unbounded_pointgoal_norm": self.raw_pointgoal_norm,
            "memory_bearing_unit": (
                list(self.unit_bearing)
                if self.unit_bearing is not None else None),
            "memory_controller_pointgoal": (
                list(self.controller_pointgoal)
                if self.controller_pointgoal is not None else None),
            "memory_unbounded_pointgoal_distance_m": self.raw_distance_m,
            "memory_controller_pointgoal_distance_m": (
                self.controller_distance_m),
            "memory_pointgoal_fixed_radius_m": (
                VERIFIED_BEARING_RADIUS_M
                if self.mode in FIXED_BEARING_MODES else None),
            "memory_metric_scale_control_authority": (
                self.metric_scale_control_authority),
            "memory_controller_pointgoal_step_cap_m": (
                self.controller_step_cap_m),
        }


def _finite_pointgoal(
    pointgoal: Sequence[float] | None,
) -> tuple[float, float] | None:
    if pointgoal is None:
        return None
    try:
        if len(pointgoal) != 2:
            return None
        if isinstance(pointgoal[0], bool) or isinstance(pointgoal[1], bool):
            return None
        point = (float(pointgoal[0]), float(pointgoal[1]))
    except (IndexError, KeyError, TypeError, ValueError, OverflowError):
        return None
    return point if all(math.isfinite(value) for value in point) else None


def adapt_revisit_pointgoal(
    *,
    mode: str,
    router_active: bool,
    pointgoal: Sequence[float] | None,
    source: str = "geometry_memory",
    pointgoal_units: str = "metric_m",
) -> RevisitAdapterDecision:
    """Convert verified memory evidence into a controller request.

    ``legacy_metric`` exactly preserves the old metric PointGoal interface and
    exists only for paired attribution/backward compatibility.

    ``navdp_front_support_v1`` preserves the legacy metric vector only when
    its forward component is inside frozen NavDP's PointGoal preprocessing
    support.  NavDP clips a negative forward component to zero, so sending a
    behind-agent target would silently destroy source information; that case
    fails closed to native ImageGoal control.

    ``verified_bearing_v1`` is the canonical source-agnostic interface: a
    verified non-zero vector contributes only its unit bearing, projected onto
    the frozen 2.5 m local radius.  Missing, malformed, inactive, and zero
    evidence abstains to native ImageGoal control.

    A certified relocalizer may pass ``lingbot_raw_direction_only`` vectors to
    ``verified_bearing_v1``.  Their arbitrary norm is audited but never called
    metres and never reaches the controller; only the normalized direction
    survives the fixed-radius projection.

    ``verified_metric_step_v1`` accepts only a metre vector backed by the
    separately validated first-40 camera-height receipt.  It preserves the
    recovered distance when nearby and caps a single controller request at
    0.8 m when far away.  The next RGB observation produces a fresh vector;
    this adapter never grants metric STOP authority.

    ``raw_fixed_bearing_v1`` is an ablation, not a verified method.  It applies
    the identical fixed-radius projection to an always-on raw metric proposal,
    so experiments can isolate controller input scale from certificate and
    abstention effects.
    """

    if mode not in REVISIT_ADAPTER_MODES:
        raise ValueError(f"unsupported revisit adapter mode {mode!r}")
    if not isinstance(source, str) or not source:
        raise ValueError("source must be a non-empty string")
    if pointgoal_units not in POINTGOAL_UNITS:
        raise ValueError(f"unsupported PointGoal units {pointgoal_units!r}")

    raw = _finite_pointgoal(pointgoal)
    raw_norm = math.hypot(*raw) if raw is not None else None
    raw_distance = raw_norm if pointgoal_units == "metric_m" else None
    unit = None
    if raw_norm is not None and raw_norm > ZERO_BEARING_EPS:
        unit = (raw[0] / raw_norm, raw[1] / raw_norm)

    if not router_active:
        return RevisitAdapterDecision(
            mode=mode,
            source=source,
            takeover=False,
            reason="router_inactive",
            controller_contract="native_imagegoal",
            raw_pointgoal=raw,
            raw_pointgoal_units=pointgoal_units,
            unit_bearing=unit,
            controller_pointgoal=None,
            raw_pointgoal_norm=raw_norm,
            raw_distance_m=raw_distance,
            controller_distance_m=None,
        )
    if raw is None:
        return RevisitAdapterDecision(
            mode=mode,
            source=source,
            takeover=False,
            reason=("missing_pointgoal" if pointgoal is None
                    else "invalid_pointgoal"),
            controller_contract="native_imagegoal",
            raw_pointgoal=None,
            raw_pointgoal_units=pointgoal_units,
            unit_bearing=None,
            controller_pointgoal=None,
            raw_pointgoal_norm=None,
            raw_distance_m=None,
            controller_distance_m=None,
        )

    if mode == "legacy_metric":
        if pointgoal_units != "metric_m":
            return RevisitAdapterDecision(
                mode=mode, source=source, takeover=False,
                reason="metric_units_required",
                controller_contract="native_imagegoal",
                raw_pointgoal=raw,
                raw_pointgoal_units=pointgoal_units,
                unit_bearing=unit,
                controller_pointgoal=None,
                raw_pointgoal_norm=raw_norm,
                raw_distance_m=None,
                controller_distance_m=None,
            )
        return RevisitAdapterDecision(
            mode=mode,
            source=source,
            takeover=True,
            reason="legacy_metric_pointgoal",
            controller_contract="configured_revisit_controller",
            raw_pointgoal=raw,
            raw_pointgoal_units=pointgoal_units,
            unit_bearing=unit,
            controller_pointgoal=raw,
            raw_pointgoal_norm=raw_norm,
            raw_distance_m=raw_distance,
            controller_distance_m=raw_distance,
        )

    if mode == "navdp_front_support_v1":
        if pointgoal_units != "metric_m":
            return RevisitAdapterDecision(
                mode=mode, source=source, takeover=False,
                reason="metric_units_required",
                controller_contract="native_imagegoal",
                raw_pointgoal=raw,
                raw_pointgoal_units=pointgoal_units,
                unit_bearing=unit,
                controller_pointgoal=None,
                raw_pointgoal_norm=raw_norm,
                raw_distance_m=None,
                controller_distance_m=None,
            )
        if unit is None:
            return RevisitAdapterDecision(
                mode=mode,
                source=source,
                takeover=False,
                reason="zero_pointgoal",
                controller_contract="native_imagegoal",
                raw_pointgoal=raw,
                raw_pointgoal_units=pointgoal_units,
                unit_bearing=None,
                controller_pointgoal=None,
                raw_pointgoal_norm=raw_norm,
                raw_distance_m=raw_distance,
                controller_distance_m=None,
            )
        if raw[0] < 0.0:
            return RevisitAdapterDecision(
                mode=mode,
                source=source,
                takeover=False,
                reason="pointgoal_behind_navdp_support",
                controller_contract="native_imagegoal",
                raw_pointgoal=raw,
                raw_pointgoal_units=pointgoal_units,
                unit_bearing=unit,
                controller_pointgoal=None,
                raw_pointgoal_norm=raw_norm,
                raw_distance_m=raw_distance,
                controller_distance_m=None,
            )
        return RevisitAdapterDecision(
            mode=mode,
            source=source,
            takeover=True,
            reason="pointgoal_inside_navdp_support",
            controller_contract="mixed_imagegoal_pointgoal",
            raw_pointgoal=raw,
            raw_pointgoal_units=pointgoal_units,
            unit_bearing=unit,
            controller_pointgoal=raw,
            raw_pointgoal_norm=raw_norm,
            raw_distance_m=raw_distance,
            controller_distance_m=raw_distance,
        )

    if mode == "verified_metric_step_v1":
        if pointgoal_units != "metric_m":
            return RevisitAdapterDecision(
                mode=mode, source=source, takeover=False,
                reason="metric_units_required",
                controller_contract="native_imagegoal",
                raw_pointgoal=raw,
                raw_pointgoal_units=pointgoal_units,
                unit_bearing=unit,
                controller_pointgoal=None,
                raw_pointgoal_norm=raw_norm,
                raw_distance_m=None,
                controller_distance_m=None,
            )
        if unit is None or raw_distance is None:
            return RevisitAdapterDecision(
                mode=mode, source=source, takeover=False,
                reason="zero_metric_pointgoal",
                controller_contract="native_imagegoal",
                raw_pointgoal=raw,
                raw_pointgoal_units=pointgoal_units,
                unit_bearing=None,
                controller_pointgoal=None,
                raw_pointgoal_norm=raw_norm,
                raw_distance_m=raw_distance,
                controller_distance_m=None,
            )
        controller_distance = min(
            raw_distance, VERIFIED_METRIC_STEP_CAP_M
        )
        return RevisitAdapterDecision(
            mode=mode,
            source=source,
            takeover=True,
            reason="verified_camera_height_metric_bounded_step",
            controller_contract="mixed_imagegoal_pointgoal",
            raw_pointgoal=raw,
            raw_pointgoal_units=pointgoal_units,
            unit_bearing=unit,
            controller_pointgoal=(
                unit[0] * controller_distance,
                unit[1] * controller_distance,
            ),
            raw_pointgoal_norm=raw_norm,
            raw_distance_m=raw_distance,
            controller_distance_m=controller_distance,
            metric_scale_control_authority=True,
            controller_step_cap_m=VERIFIED_METRIC_STEP_CAP_M,
        )

    if mode == "raw_fixed_bearing_v1" and pointgoal_units != "metric_m":
        return RevisitAdapterDecision(
            mode=mode, source=source, takeover=False,
            reason="metric_units_required",
            controller_contract="native_imagegoal",
            raw_pointgoal=raw,
            raw_pointgoal_units=pointgoal_units,
            unit_bearing=unit,
            controller_pointgoal=None,
            raw_pointgoal_norm=raw_norm,
            raw_distance_m=None,
            controller_distance_m=None,
        )

    if unit is None:
        return RevisitAdapterDecision(
            mode=mode,
            source=source,
            takeover=False,
            reason="zero_bearing",
            controller_contract="native_imagegoal",
            raw_pointgoal=raw,
            raw_pointgoal_units=pointgoal_units,
            unit_bearing=None,
            controller_pointgoal=None,
            raw_pointgoal_norm=raw_norm,
            raw_distance_m=raw_distance,
            controller_distance_m=None,
        )

    controller_pointgoal = (
        unit[0] * VERIFIED_BEARING_RADIUS_M,
        unit[1] * VERIFIED_BEARING_RADIUS_M,
    )
    if mode == "raw_fixed_bearing_v1":
        reason = "raw_uncertified_fixed_bearing"
    elif pointgoal_units in (
            "lingbot_raw_direction_only",
            "pi3x_current_camera_direction_only"):
        reason = "verified_scale_free_bearing"
    else:
        reason = "verified_geometry_bearing"
    return RevisitAdapterDecision(
        mode=mode,
        source=source,
        takeover=True,
        reason=reason,
        controller_contract="mixed_imagegoal_pointgoal",
        raw_pointgoal=raw,
        raw_pointgoal_units=pointgoal_units,
        unit_bearing=unit,
        controller_pointgoal=controller_pointgoal,
        raw_pointgoal_norm=raw_norm,
        raw_distance_m=raw_distance,
        controller_distance_m=VERIFIED_BEARING_RADIUS_M,
    )


def validate_revisit_adapter_configuration(
    *,
    mode: str,
    server_backend: str,
    revisit_controller: str,
    router_is_automatic_geometry: bool,
    router_is_certified_relocalization: bool = False,
) -> None:
    """Reject configurations that would corrupt the canonical method claim."""

    if mode not in REVISIT_ADAPTER_MODES:
        raise ValueError(f"unsupported revisit adapter mode {mode!r}")
    if mode == "legacy_metric":
        return
    if server_backend != "hybrid_pose" or revisit_controller != "navdp_mixed":
        raise ValueError(
            f"{mode} requires hybrid_pose with the existing navdp_mixed "
            "controller")
    if mode in ("navdp_front_support_v1", "raw_fixed_bearing_v1"):
        return
    if not router_is_automatic_geometry:
        raise ValueError(
            f"{mode} requires an automatic, geometry-verified "
            "router; the phase-oracle route is not deployable")


__all__ = [
    "REVISIT_ADAPTER_MODES",
    "REVISIT_ADAPTER_SCHEMA_VERSION",
    "POINTGOAL_UNITS",
    "RevisitAdapterDecision",
    "VERIFIED_BEARING_RADIUS_M",
    "VERIFIED_METRIC_STEP_CAP_M",
    "adapt_revisit_pointgoal",
    "validate_revisit_adapter_configuration",
]
