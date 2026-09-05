import math
import pathlib
import sys

import pytest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from latency_motion_guard import (  # noqa: E402
    LatencyMotionGuard,
    LatencyMotionGuardConfig,
    StopPlanActConfig,
    StopPlanActGate,
)
from trajectory_control import VelocityCommand  # noqa: E402


def command(vx=0.30, wz=0.55):
    return VelocityCommand(
        linear_x=vx,
        angular_z=wz,
        target_x=0.6,
        target_y=0.2,
        path_length=1.2,
    )


def test_fresh_plan_preserves_controller_command_without_duplicate_limiting():
    guard = LatencyMotionGuard()
    original = command()

    result = guard.apply(original, plan_input_age_s=0.90)

    assert result.command == original
    assert result.reason == "pass"


def test_fresh_stationary_plan_can_reverse_turn_immediately():
    guard = LatencyMotionGuard()
    first = guard.apply(command(wz=0.40), plan_input_age_s=0.80)
    reversal = guard.apply(command(wz=-0.40), plan_input_age_s=0.80)

    assert first.command.angular_z > 0.0
    assert reversal.command.angular_z < 0.0
    assert reversal.reason == "pass"


def test_excessively_old_or_invalid_plan_holds_motion():
    guard = LatencyMotionGuard()
    old = guard.apply(command(), plan_input_age_s=1.51)
    missing = guard.apply(command(), plan_input_age_s=None)

    assert old.reason == "plan_input_too_old_hold"
    assert old.command == VelocityCommand(target_x=0.6, target_y=0.2, path_length=1.2)
    assert missing.reason == "invalid_plan_input_age_hold"
    assert missing.command.linear_x == 0.0
    assert missing.command.angular_z == 0.0


def test_guard_can_be_disabled_without_changing_command():
    guard = LatencyMotionGuard(LatencyMotionGuardConfig(enabled=False))
    original = command()
    result = guard.apply(original, plan_input_age_s=None)

    assert result.command == original
    assert result.reason == "disabled"


def test_action_clock_starts_at_first_published_command():
    gate = StopPlanActGate()
    gate.install_plan(10.0)

    assert gate.phase(now_s=10.70, latest_rgbd_source_ns=100) == "ready_to_execute"
    gate.note_command_published(10.70, command(vx=0.10, wz=0.0))
    assert gate.phase(now_s=10.71, latest_rgbd_source_ns=100) == "execute"


def test_translation_budget_stops_action_before_wall_timeout():
    gate = StopPlanActGate(
        StopPlanActConfig(
            max_execution_s=2.0,
            max_translation_m=0.10,
            max_heading_rad=math.radians(30.0),
        )
    )
    gate.install_plan(1.0)
    gate.note_command_published(2.0, command(vx=0.20, wz=0.0))

    assert gate.phase(now_s=2.49, latest_rgbd_source_ns=100) == "execute"
    assert gate.phase(now_s=2.51, latest_rgbd_source_ns=100) == "stop_pending"


def test_heading_budget_stops_action_before_wall_timeout():
    gate = StopPlanActGate(
        StopPlanActConfig(
            max_execution_s=2.0,
            max_translation_m=1.0,
            max_heading_rad=0.10,
        )
    )
    gate.install_plan(1.0)
    gate.note_command_published(2.0, command(vx=0.0, wz=0.20))

    assert gate.phase(now_s=2.49, latest_rgbd_source_ns=100) == "execute"
    assert gate.phase(now_s=2.51, latest_rgbd_source_ns=100) == "stop_pending"


def test_wall_timeout_bounds_a_command_below_motion_budgets():
    gate = StopPlanActGate(
        StopPlanActConfig(
            max_execution_s=0.80,
            max_translation_m=1.0,
            max_heading_rad=1.0,
        )
    )
    gate.install_plan(1.0)
    gate.note_command_published(2.0, command(vx=0.01, wz=0.01))

    assert gate.phase(now_s=2.79, latest_rgbd_source_ns=100) == "execute"
    assert gate.phase(now_s=2.81, latest_rgbd_source_ns=100) == "stop_pending"


def test_stop_settle_requires_frame_captured_after_actual_zero():
    gate = StopPlanActGate(
        StopPlanActConfig(settle_before_sense_s=0.15)
    )
    gate.install_plan(1.0)
    gate.note_command_published(2.0, command(vx=0.10, wz=0.0))
    gate.note_action_stopped(2.60, stopped_ros_ns=1_000_000_000)

    assert (
        gate.phase(now_s=2.70, latest_rgbd_source_ns=1_200_000_000)
        == "settling"
    )
    assert (
        gate.phase(now_s=2.80, latest_rgbd_source_ns=1_100_000_000)
        == "waiting_for_post_stop_rgbd"
    )
    assert (
        gate.phase(now_s=2.80, latest_rgbd_source_ns=1_200_000_000)
        == "ready_to_plan"
    )


def test_callback_arrival_cannot_make_a_pre_stop_frame_eligible():
    gate = StopPlanActGate(StopPlanActConfig(settle_before_sense_s=0.0))
    gate.install_plan(1.0)
    gate.note_command_published(2.0, command(vx=0.10, wz=0.0))
    gate.note_action_stopped(2.10, stopped_ros_ns=5_000)

    # The caller may receive this pair much later, but only its source stamp
    # is admitted at the control boundary.
    assert (
        gate.phase(now_s=20.0, latest_rgbd_source_ns=4_999)
        == "waiting_for_post_stop_rgbd"
    )


def test_zero_plan_skips_empty_execution_slot():
    gate = StopPlanActGate()
    gate.install_plan(1.0)

    assert gate.note_command_published(2.0, VelocityCommand()) == "stop_pending"


def test_planning_and_motion_phases_are_disjoint():
    gate = StopPlanActGate()
    for phase in ("need_plan", "ready_to_plan"):
        assert gate.planning_allowed(phase) is True
        assert gate.motion_allowed(phase) is False


@pytest.mark.parametrize(
    "config",
    [
        StopPlanActConfig(max_execution_s=0.0),
        StopPlanActConfig(max_translation_m=0.0),
        StopPlanActConfig(max_heading_rad=0.0),
        StopPlanActConfig(settle_before_sense_s=-0.1),
    ],
)
def test_stop_plan_act_rejects_invalid_limits(config):
    with pytest.raises(ValueError):
        StopPlanActGate(config)
