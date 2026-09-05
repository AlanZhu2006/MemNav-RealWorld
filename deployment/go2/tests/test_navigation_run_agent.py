from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation_run_agent import (  # noqa: E402
    assess_motion,
    NavigationRunAgent,
    assess_path,
    live_fault,
    locked_preflight_issue,
    preserved_revisit_issue,
)
from terminal_motion_override import terminal_motion_override


def turn_status(bearing=(-1.0, 0.05), **updates):
    status = ready_status()
    status["heading_turn"] = {"feedback_age_s": 0.02}
    status["heading_reference_available"] = True
    receipt = {
        "terminal_handoff_schema": "cec_direct_bearing_handoff_v2_20260824",
        "terminal_handoff_disposition": "long_range",
        "terminal_local_latched": False,
        "cec_takeover": True,
        "cec_certificate": {"accepted": True},
        "terminal_point_token_support_deg": 60.0,
        "memory_bearing_unit": list(bearing),
    }
    receipt.update(updates)
    status["terminal_motion_override"] = terminal_motion_override(
        receipt, rotate_gain=1.5, max_angular_rps=0.55,
    ).audit_dict()
    return status


@pytest.mark.parametrize("bearing,sign", [((-1, 0.05), 1), ((-1, -0.05), -1)])
def test_rear_turn_can_start_on_zero_path_and_hands_back_to_forward_path(bearing, sign):
    status = turn_status(bearing)
    limits = dict(max_linear_mps=0.30, max_angular_rps=0.55)
    assert locked_preflight_issue(status, min_clearance_m=0.8) == ""
    motion = assess_motion(np.zeros((24, 2)), status, **limits)
    assert motion.predicted_vx == 0.0
    assert motion.predicted_wz == pytest.approx(sign * 0.55)
    assert motion.motion_source == "rear_goal_heading_turn"
    status.update(
        enabled=True,
        estop=False,
        cmd_vx=motion.predicted_vx,
        cmd_wz=motion.predicted_wz,
    )
    assert live_fault(status, **limits) == ""
    forward = turn_status((1, 0.1))
    motion = assess_motion(np.array([[0, 0], [1, 0]]), forward, **limits)
    assert motion.predicted_vx > 0
    assert motion.motion_source == "trajectory"
    with pytest.raises(ValueError, match="too short"):
        assess_motion(np.zeros((24, 2)), forward, **limits)


def test_direct_turn_accepts_zero_translation():
    status = turn_status(
        terminal_handoff_disposition="atomic_turn",
        terminal_proof_active=True, terminal_turn_error_left_rad=-3.1,
    )
    motion = assess_motion(
        np.zeros((24, 2)), status, max_linear_mps=0.3, max_angular_rps=0.55,
    )
    assert motion.predicted_vx == 0.0
    assert motion.predicted_wz == pytest.approx(-0.55)


def test_heading_feedback_required_to_arm_and_during_continuous_turn():
    status = turn_status()
    status["heading_turn"]["feedback_age_s"] = 1.0
    with pytest.raises(ValueError, match="fresh body heading"):
        assess_motion(np.zeros((24, 2)), status, max_linear_mps=0.3, max_angular_rps=0.55)
    status["plan_age_s"] = 12.0
    status["heading_turn"] = {"active": True, "feedback_age_s": 0.02}
    assert live_fault(status, max_linear_mps=0.3, max_angular_rps=0.55) == ""
    status["heading_turn"]["feedback_age_s"] = 0.5
    assert live_fault(status, max_linear_mps=0.3, max_angular_rps=0.55) == "heading_feedback_stale"
    status["heading_turn"] = {"active": False, "phase": "complete", "completed_age_s": 0.5}
    assert live_fault(status, max_linear_mps=0.3, max_angular_rps=0.55) == ""
    status["heading_turn"]["completed_age_s"] = 6.0
    assert live_fault(status, max_linear_mps=0.3, max_angular_rps=0.55) == "trajectory_stale"


@pytest.mark.parametrize("field,value", [
    ("angular_z", 0), ("angular_z", 0.56), ("angular_z", float("nan")),
    ("angular_z", float("inf")), ("angular_z", True),
    ("linear_x", 0.01), ("linear_x", -0.01), ("linear_x", 0.09),
    ("linear_x", 0.11), ("linear_x", 0.31),
    ("reverse", True),
])
def test_turn_rejects_malformed_or_unsafe_commands(field, value):
    status = turn_status()
    status["terminal_motion_override"]["command"][field] = value
    with pytest.raises(ValueError):
        assess_motion(np.zeros((24, 2)), status, max_linear_mps=0.3, max_angular_rps=0.55)


@pytest.mark.parametrize("updates", [
    {"cec_certificate": {"accepted": False}},
    {"memory_bearing_unit": [0, 0]},
    {"terminal_handoff_disposition": "hold", "terminal_local_latched": True},
    {"terminal_handoff_disposition": "stop", "terminal_stop_authorized": True},
])
def test_rejected_or_holding_override_cannot_arm_even_with_valid_path(updates):
    with pytest.raises(ValueError):
        assess_motion(
            np.array([[0, 0], [1, 0]]), turn_status(**updates),
            max_linear_mps=0.3, max_angular_rps=0.55,
        )


def test_turn_does_not_bypass_sensor_plan_or_clearance_gates():
    for field, value in [("rgbd_age_s", 1), ("plan_age_s", 2), ("clearance_m", 0.4)]:
        status = turn_status()
        status[field] = value
        assert locked_preflight_issue(status, min_clearance_m=0.8)


@pytest.mark.parametrize("plan_time,accepted", [
    (None, False), (99.0, False), (100.0, False),
    (float("nan"), False), (103.0, False), (101.0, True),
])
def test_readiness_requires_new_adapter_plan_not_cached_turn(monkeypatch, plan_time, accepted):
    import navigation_run_agent as module
    monkeypatch.setattr(module.time, "monotonic", lambda: 102.0)
    agent = object.__new__(NavigationRunAgent)
    agent.status = turn_status()
    agent.status.update(phase="revisit_query", plan_monotonic_s=plan_time)
    agent.status_received_at = 102.0
    agent.reset_started_at = 100.0
    agent.path_after_reset = np.zeros((24, 2))
    agent.arrival_status = {"arrival_latched": False}
    agent.args = SimpleNamespace(
        preserve_policy_state=False, arrival_phases={"revisit_query"},
        min_clearance_m=0.8, ready_timeout_s=25,
    )
    agent.node = SimpleNamespace(get_subscriptions_info_by_topic=lambda _: [
        SimpleNamespace(node_name="navdp_go2_cmd_bridge"),
    ])
    agent._spin_until = lambda predicate, timeout: predicate()
    ready, issue = agent._wait_for_reset_ready()
    assert ready is accepted
    if not accepted:
        assert issue == "waiting_for_post_reset_plan_status"


def test_run_transaction_accepts_certified_turn_before_mock_arm(monkeypatch):
    """Exercise the actual run orchestration without ROS or motion services."""
    import navigation_run_agent as module
    agent = object.__new__(NavigationRunAgent)
    agent.args = SimpleNamespace(
        preserve_policy_state=False, max_linear_mps=0.3, max_angular_rps=0.55,
        min_clearance_m=0.8, arrival_goal="unused", min_image_scale=0.6,
        max_image_scale=1.45,
    )
    agent.status = turn_status()
    agent.rgb = np.zeros((4, 4, 3), dtype=np.uint8)
    agent.arrival_status = {}
    agent.Trigger = SimpleNamespace(Request=lambda: None)
    agent.reset_client = object()
    events = []
    agent._log = lambda phase, message: events.append(phase)
    agent._operator_stop = lambda reason: events.append("stop")
    agent._spin_until = lambda predicate, timeout: predicate()
    agent._call = lambda *args, **kwargs: SimpleNamespace(message="reset")
    def ready():
        agent.path_after_reset = np.zeros((24, 2))
        return True, ""
    agent._wait_for_reset_ready = ready
    def arm():
        events.append("mock_arm")
        return "enabled"
    agent._arm = arm
    agent._monitor = lambda: (3, "stopped_externally")
    monkeypatch.setattr(module, "load_rgb_image", lambda _: agent.rgb)
    monkeypatch.setattr(module, "RgbGoalArrivalVerifier", lambda *a, **kw: SimpleNamespace(
        evaluate=lambda _: SimpleNamespace(
            matched=False, reason="not_arrived", good_matches=0,
            inliers=0, image_scale=1.0,
        ),
    ))
    assert agent.run() == 3
    assert "mock_arm" in events
    assert events.index("PLAN") < events.index("mock_arm")


def ready_status():
    return {
        "enabled": False,
        "estop": True,
        "server_initialized": True,
        "image_goal_loaded": True,
        "arrival_latched": False,
        "last_error": "",
        "rgbd_age_s": 0.03,
        "plan_age_s": 0.20,
        "clearance_m": 1.10,
        "stop_reason": "disabled",
        "cmd_vx": 0.0,
        "cmd_wz": 0.0,
    }


def test_locked_preflight_accepts_only_a_fresh_safe_locked_plan():
    assert locked_preflight_issue(ready_status(), min_clearance_m=0.8) == ""

    cases = {
        "enabled": "adapter_is_enabled",
        "estop": "estop_is_not_asserted",
        "server_initialized": "policy_not_initialized",
        "image_goal_loaded": "image_goal_not_loaded",
        "arrival_latched": "arrival_latch_not_reset",
    }
    for field, expected in cases.items():
        status = ready_status()
        status[field] = not status[field]
        assert (
            locked_preflight_issue(status, min_clearance_m=0.8) == expected
        )

    status = ready_status()
    status["clearance_m"] = 0.79
    assert locked_preflight_issue(status, min_clearance_m=0.8).startswith(
        "clearance_below_"
    )


def test_selected_path_is_summarized_with_the_real_controller():
    path = np.asarray([[0.1, 0.0], [0.6, 0.0], [1.2, -0.1]])
    assessment = assess_path(
        path, max_linear_mps=0.30, max_angular_rps=0.55
    )
    assert assessment.poses == 3
    assert assessment.path_length_m > 1.0
    assert assessment.predicted_vx == pytest.approx(0.30)
    assert assessment.predicted_wz == pytest.approx(0.0)
    assert assessment.reverse is False


def test_selected_path_rejects_invalid_or_empty_geometry():
    with pytest.raises(ValueError, match="invalid shape"):
        assess_path(
            np.empty((0, 2)), max_linear_mps=0.30, max_angular_rps=0.55
        )
    with pytest.raises(ValueError, match="non-finite"):
        assess_path(
            np.asarray([[0.0, 0.0], [np.nan, 1.0]]),
            max_linear_mps=0.30,
            max_angular_rps=0.55,
        )


def test_live_monitor_faults_are_fail_closed():
    status = ready_status()
    status.update(enabled=True, estop=False, stop_reason="clear", cmd_vx=0.2)
    assert (
        live_fault(status, max_linear_mps=0.30, max_angular_rps=0.55) == ""
    )

    # A sub-second Jetson scheduling hiccup is tolerated after arming.
    status["rgbd_age_s"] = 0.80
    assert live_fault(
        status, max_linear_mps=0.30, max_angular_rps=0.55
    ) == ""
    status["rgbd_age_s"] = 1.01
    assert live_fault(
        status, max_linear_mps=0.30, max_angular_rps=0.55
    ) == "rgbd_stale"
    status["rgbd_age_s"] = 0.02
    status["stop_reason"] = "obstacle_stop"
    assert live_fault(
        status, max_linear_mps=0.30, max_angular_rps=0.55
    ) == "obstacle_stop"
    status["stop_reason"] = "clear"
    status["cmd_wz"] = 0.57
    assert live_fault(
        status, max_linear_mps=0.30, max_angular_rps=0.55
    ) == "angular_command_limit_violation"


def test_rgbd_runtime_grace_does_not_relax_locked_preflight():
    status = ready_status()
    status["rgbd_age_s"] = 0.80
    assert (
        locked_preflight_issue(status, min_clearance_m=0.8)
        == "rgbd_not_fresh"
    )


def test_live_plan_freshness_limit_is_five_seconds_but_preflight_stays_strict():
    status = ready_status()
    status["plan_age_s"] = 4.90
    assert (
        live_fault(status, max_linear_mps=0.30, max_angular_rps=0.55) == ""
    )

    status["plan_age_s"] = 5.01
    assert (
        live_fault(status, max_linear_mps=0.30, max_angular_rps=0.55)
        == "trajectory_stale"
    )

    status["plan_age_s"] = 1.51
    assert (
        locked_preflight_issue(status, min_clearance_m=0.8)
        == "trajectory_not_fresh"
    )

def test_preserved_revisit_requires_exact_phase_dataset_and_goal():
    status = ready_status()
    status.update(
        phase="revisit_query",
        active_goal_sha256="b" * 64,
        begin_revisit_receipt={
            "loaded_dataset_id": "route_01",
            "loaded_dataset_manifest_sha256": "a" * 64,
            "selected_goal": {"sha256": "b" * 64},
        },
    )
    expected = {
        "expected_dataset_id": "route_01",
        "expected_dataset_sha256": "a" * 64,
        "expected_goal_sha256": "b" * 64,
    }
    assert preserved_revisit_issue(status, **expected) == ""

    changes = {
        "phase": "revisit_phase_not_active",
        "active_goal_sha256": "revisit_goal_changed",
    }
    for key, issue in changes.items():
        changed = dict(status)
        changed[key] = "wrong"
        assert preserved_revisit_issue(changed, **expected) == issue

    changed = dict(status)
    changed["begin_revisit_receipt"] = {
        **status["begin_revisit_receipt"],
        "loaded_dataset_id": "wrong",
    }
    assert preserved_revisit_issue(changed, **expected) == "revisit_dataset_changed"


def test_fullmono_runner_preserves_prepared_revisit_instead_of_resetting_it():
    script = (
        Path(__file__).resolve().parents[1] / "scripts/run_navigation.sh"
    ).read_text(encoding="utf-8")
    assert "--preserve-policy-state" in script
    assert "formal.expected_dataset_sha256" in script
    assert 'sha256sum "$CFG_SELECTED_GOAL_IMAGE"' in script
