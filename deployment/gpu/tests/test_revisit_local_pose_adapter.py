import math

import pytest

from deployment.gpu.revisit_local_pose_adapter import (
    CERTIFIED_BEARING_RESIDUAL_M,
    METRIC_POINTGOAL_STEP_CAP_M,
    decide_local_pose_handoff,
)


def evidence(x, y, *, metric_distance=0.01, yaw_right=0.0):
    return {
        "certificate_accepted": True,
        "scale_free_direction_available": True,
        "predicted_scale_free_relative_xy": [x, y],
        # Deliberately inconsistent with the direction magnitude.  This value
        # is diagnostic only and must never gain control or STOP authority.
        "metric_scale_available": True,
        "predicted_relative_xy_m": [metric_distance, 0.0],
        "predicted_distance_m": metric_distance,
        "terminal_yaw_right_deg": yaw_right,
    }


def metric_evidence(x, y, *, metric_scale=1.0, yaw_right=0.0):
    metric_x = metric_scale * x
    metric_y = metric_scale * y
    distance = math.hypot(metric_x, metric_y)
    return {
        "certificate_accepted": True,
        "scale_free_direction_available": True,
        "predicted_scale_free_relative_xy": [x, y],
        "metric_scale_available": True,
        "metric_scale_policy": "mdtec_first40",
        "metric_scale_transaction_bound": True,
        "metric_scale": {
            "available": True,
            "reason": "mdtec_first40_causal_scale_available",
            "frame_count": 40,
            "metric_scale_m_per_raw": metric_scale,
            "scale_receipt_sha256": "a" * 64,
            "scale_evidence_contract": "causal_first_prefix_rgb_only_v1",
        },
        "predicted_relative_xy_m": [metric_x, metric_y],
        "predicted_distance_m": distance,
        "terminal_yaw_right_deg": yaw_right,
    }


def test_long_range_remains_active_until_direct_bearing_is_certified():
    decision = decide_local_pose_handoff(
        long_range_available=True,
        evidence={"certificate_accepted": False},
    )
    assert decision.disposition == "long_range"
    assert decision.local_latched is False
    assert decision.direct_proof_active is False


def test_rear_goal_requests_atomic_turn_in_go2_sign_convention():
    decision = decide_local_pose_handoff(
        long_range_available=True,
        evidence=evidence(-0.695, -0.024),
    )
    assert decision.disposition == "atomic_turn"
    assert decision.local_latched is False
    assert decision.direct_proof_active is True
    assert decision.predicted_bearing_deg == pytest.approx(-178.02, abs=0.05)
    assert decision.turn_error_left_rad < 0.0
    assert decision.controller_pointgoal_m is None


def test_supported_direction_is_projected_to_frozen_scale_free_residual():
    decision = decide_local_pose_handoff(
        long_range_available=True,
        evidence=evidence(0.60, 0.20, metric_distance=99.0),
    )
    assert decision.disposition == "bearing_local"
    assert math.hypot(*decision.controller_pointgoal_m) == pytest.approx(
        CERTIFIED_BEARING_RESIDUAL_M
    )
    assert decision.controller_pointgoal_m[1] / decision.controller_pointgoal_m[0] == (
        pytest.approx(1.0 / 3.0)
    )
    assert decision.predicted_distance_m == 99.0
    assert decision.stop_authorized is False


def test_valid_first40_metric_distance_is_capped_per_replan():
    decision = decide_local_pose_handoff(
        long_range_available=True,
        evidence=metric_evidence(3.0, 4.0, metric_scale=0.5),
    )

    assert decision.disposition == "bearing_local"
    assert decision.reason == "direct_camera_height_metric_bounded_step"
    assert decision.metric_scale_control_authority is True
    assert math.hypot(*decision.controller_pointgoal_m) == pytest.approx(
        METRIC_POINTGOAL_STEP_CAP_M
    )
    assert decision.controller_pointgoal_m == pytest.approx((0.48, 0.64))
    audit = decision.audit_dict()
    assert audit["terminal_metric_scale_control_authority"] is True
    assert audit["terminal_predicted_distance_control_authority"] is True
    assert audit["terminal_metric_scale_receipt_sha256"] == "a" * 64
    assert audit["terminal_stop_authorized"] is False


def test_valid_near_metric_distance_is_not_inflated_to_step_cap():
    decision = decide_local_pose_handoff(
        long_range_available=False,
        evidence=metric_evidence(0.3, 0.4, metric_scale=0.5),
    )

    assert decision.controller_pointgoal_m == pytest.approx((0.15, 0.20))
    assert decision.controller_distance_m == pytest.approx(0.25)
    assert decision.stop_authorized is False


def test_unbound_metric_receipt_falls_back_to_scale_free_residual():
    unbound = metric_evidence(0.3, 0.4, metric_scale=0.5)
    unbound["metric_scale_transaction_bound"] = False

    decision = decide_local_pose_handoff(
        long_range_available=True,
        evidence=unbound,
    )

    assert decision.metric_scale_control_authority is False
    assert decision.reason == "direct_scale_free_bearing_certified"
    assert math.hypot(*decision.controller_pointgoal_m) == pytest.approx(
        CERTIFIED_BEARING_RESIDUAL_M
    )


def test_proof_loss_returns_to_preceding_route_instead_of_holding():
    decision = decide_local_pose_handoff(
        long_range_available=True,
        evidence={"certificate_accepted": False},
        local_latched=True,
        stop_streak=2,
    )
    assert decision.disposition == "long_range"
    assert decision.local_latched is False
    assert decision.stop_streak == 0


def test_tiny_untrusted_metric_distance_never_authorizes_stop():
    decision = decide_local_pose_handoff(
        long_range_available=False,
        evidence=evidence(1.0, 0.0, metric_distance=0.001, yaw_right=0.0),
        local_latched=True,
        stop_streak=100,
    )
    assert decision.disposition == "bearing_local"
    assert decision.stop_authorized is False
    assert decision.stop_streak == 0
    audit = decision.audit_dict()
    assert audit["terminal_metric_scale_control_authority"] is False
    assert audit["terminal_predicted_distance_control_authority"] is False
    assert audit["terminal_stop_authority"] == (
        "none_until_independent_visual_convergence"
    )


def test_malformed_direction_never_takes_over():
    malformed = evidence(0.4, 0.1)
    malformed["predicted_scale_free_relative_xy"] = [float("nan"), 0.0]
    malformed["predicted_relative_xy_m"] = None
    decision = decide_local_pose_handoff(
        long_range_available=True, evidence=malformed
    )
    assert decision.disposition == "long_range"


def test_legacy_metric_vector_is_used_only_for_its_direction():
    decision = decide_local_pose_handoff(
        long_range_available=False,
        evidence={
            "certificate_accepted": True,
            "metric_scale_available": True,
            "predicted_relative_xy_m": [0.03, -0.04],
            "predicted_distance_m": 0.05,
        },
    )
    assert decision.disposition == "bearing_local"
    assert decision.controller_pointgoal_m == pytest.approx((1.5, -2.0))
    assert decision.stop_authorized is False


def test_direct_bearing_can_take_over_without_long_range_or_role_label():
    decision = decide_local_pose_handoff(
        long_range_available=False,
        evidence=evidence(0.45, -0.10),
    )
    assert decision.disposition == "bearing_local"
    assert decision.local_latched is False
    assert decision.audit_dict()["terminal_proof_active"] is True


def test_no_direct_or_long_range_proof_preserves_native_route():
    decision = decide_local_pose_handoff(
        long_range_available=False,
        evidence={"certificate_accepted": False},
    )
    assert decision.disposition == "native"
    assert decision.local_latched is False
    assert decision.audit_dict()["terminal_proof_active"] is False
