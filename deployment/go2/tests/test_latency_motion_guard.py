import math
import pathlib
import sys

import pytest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from latency_motion_guard import (  # noqa: E402
    LatencyMotionGuard,
    LatencyMotionGuardConfig,
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


def test_slow_plan_limits_rotation_and_keeps_minimal_go2_turn_creep():
    guard = LatencyMotionGuard()
    result = guard.apply(command(), plan_input_age_s=0.90, max_angular_rps=0.55)

    assert result.command.linear_x == pytest.approx(0.08)
    assert result.command.angular_z == pytest.approx(math.radians(10.0) / 0.90)
    assert result.reason == "stale_plan_turn_creep"
    assert result.command.target_x == 0.6


def test_small_heading_request_can_keep_driving():
    guard = LatencyMotionGuard()
    result = guard.apply(
        command(wz=0.06), plan_input_age_s=0.90, max_angular_rps=0.55
    )

    assert result.command.linear_x == pytest.approx(0.30)
    assert result.command.angular_z == pytest.approx(0.06)
    assert result.reason == "pass"


def test_slow_reverse_turn_creep_preserves_direction_and_cap():
    guard = LatencyMotionGuard()
    result = guard.apply(
        command(vx=-0.20, wz=0.30),
        plan_input_age_s=0.90,
        max_angular_rps=0.55,
    )

    assert result.command.linear_x == pytest.approx(-0.08)
    assert result.reason == "stale_plan_turn_creep"


def test_slow_certified_turn_preserves_required_go2_creep():
    guard = LatencyMotionGuard()
    result = guard.apply(
        command(vx=0.10, wz=0.55),
        plan_input_age_s=0.90,
        max_angular_rps=0.55,
        preserve_turn_creep=True,
    )

    assert result.command.linear_x == pytest.approx(0.10)
    assert result.command.angular_z == pytest.approx(math.radians(10.0) / 0.90)
    assert result.reason == "latency_limited_turn"


def test_single_opposite_plan_stops_instead_of_reversing_turn():
    guard = LatencyMotionGuard()
    first = guard.apply(command(wz=0.40), plan_input_age_s=0.80, max_angular_rps=0.55)
    reversal = guard.apply(
        command(wz=-0.40), plan_input_age_s=0.80, max_angular_rps=0.55
    )
    confirmed = guard.apply(
        command(wz=-0.35), plan_input_age_s=0.80, max_angular_rps=0.55
    )

    assert first.command.angular_z > 0.0
    assert reversal.reason == "turn_reversal_confirmation_hold"
    assert reversal.command.linear_x == 0.0
    assert reversal.command.angular_z == 0.0
    assert confirmed.command.angular_z < 0.0


def test_one_frame_flip_flop_never_executes_the_opposite_turn():
    guard = LatencyMotionGuard()
    positive = guard.apply(
        command(wz=0.40), plan_input_age_s=0.80, max_angular_rps=0.55
    )
    negative = guard.apply(
        command(wz=-0.40), plan_input_age_s=0.80, max_angular_rps=0.55
    )
    positive_again = guard.apply(
        command(wz=0.40), plan_input_age_s=0.80, max_angular_rps=0.55
    )

    assert positive.command.angular_z > 0.0
    assert negative.command.angular_z == 0.0
    assert positive_again.command.angular_z > 0.0


def test_excessively_old_or_invalid_plan_holds_motion():
    guard = LatencyMotionGuard()
    old = guard.apply(command(), plan_input_age_s=1.51, max_angular_rps=0.55)
    missing = guard.apply(command(), plan_input_age_s=None, max_angular_rps=0.55)

    assert old.reason == "plan_input_too_old_hold"
    assert old.command == VelocityCommand(target_x=0.6, target_y=0.2, path_length=1.2)
    assert missing.reason == "invalid_plan_input_age_hold"
    assert missing.command.linear_x == 0.0
    assert missing.command.angular_z == 0.0


def test_guard_can_be_disabled_without_changing_command():
    guard = LatencyMotionGuard(LatencyMotionGuardConfig(enabled=False))
    original = command()
    result = guard.apply(original, plan_input_age_s=None, max_angular_rps=0.55)

    assert result.command == original
    assert result.reason == "disabled"
