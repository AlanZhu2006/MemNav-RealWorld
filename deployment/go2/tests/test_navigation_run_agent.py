from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation_run_agent import (  # noqa: E402
    assess_path,
    live_fault,
    locked_preflight_issue,
    preserved_revisit_issue,
)


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

    status["rgbd_age_s"] = 0.80
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
