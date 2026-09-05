import pytest

from terminal_motion_override import (
    terminal_motion_override,
)
from trajectory_control import VelocityCommand


def receipt(disposition, **updates):
    value = {
        "terminal_handoff_schema": "cec_direct_bearing_handoff_v2_20260824",
        "cec_takeover": True,
        "terminal_handoff_disposition": disposition,
        "terminal_local_latched": disposition in {"hold", "stop"},
        "terminal_stop_authorized": False,
        "terminal_proof_active": disposition not in {"long_range", "native"},
    }
    value.update(updates)
    return value


def certified_long_range(bearing):
    return receipt(
        "long_range",
        cec_certificate={"accepted": True},
        memory_bearing_unit=bearing,
        terminal_point_token_support_deg=60.0,
    )


def test_front_long_range_and_bearing_local_leave_navdp_command_untouched():
    results = (
        terminal_motion_override(
            certified_long_range([1.0, 0.2]),
            rotate_gain=1.5,
            max_angular_rps=0.35,
        ),
        terminal_motion_override(
            receipt("bearing_local"), rotate_gain=1.5, max_angular_rps=0.35
        ),
    )
    for result in results:
        assert result.applied is False
        assert result.command is None


def test_certified_rear_long_range_requests_bounded_atomic_turn():
    result = terminal_motion_override(
        certified_long_range([-0.96, -0.28]),
        rotate_gain=1.5,
        max_angular_rps=0.35,
    )
    assert result.applied is True
    assert result.command.linear_x == 0.0
    assert result.command.angular_z == pytest.approx(-0.35)
    assert result.assert_estop is False
    assert result.reason == "rear_goal_heading_turn"


def test_malformed_long_range_turn_receipt_fails_closed():
    result = terminal_motion_override(
        receipt("long_range", cec_certificate={"accepted": True}),
        rotate_gain=1.5,
        max_angular_rps=0.35,
    )
    assert result.applied is True
    assert result.command.linear_x == 0.0
    assert result.command.angular_z == 0.0
    assert result.reason == "invalid_long_range_turn_receipt"


def test_rear_target_turn_is_bounded_without_translation():
    result = terminal_motion_override(
        receipt("atomic_turn", terminal_turn_error_left_rad=-3.10),
        rotate_gain=1.5,
        max_angular_rps=0.35,
    )
    assert result.applied is True
    assert result.command.linear_x == 0.0
    assert result.command.angular_z == pytest.approx(-0.35)
    assert result.assert_estop is False


def test_direct_novel_proof_is_eligible_without_cec_takeover():
    result = terminal_motion_override(
        receipt(
            "atomic_turn",
            cec_takeover=False,
            terminal_proof_active=True,
            terminal_turn_error_left_rad=1.0,
        ),
        rotate_gain=1.5,
        max_angular_rps=0.35,
    )
    assert result.applied is True
    assert result.command.linear_x == 0.0
    assert result.command.angular_z == pytest.approx(0.35)


def test_local_proof_loss_holds_without_asserting_success():
    result = terminal_motion_override(
        receipt("hold"), rotate_gain=1.5, max_angular_rps=0.35
    )
    assert result.applied is True
    assert result.command.linear_x == 0.0
    assert result.command.angular_z == 0.0
    assert result.assert_estop is False


def test_only_authorized_stop_asserts_estop():
    pending = terminal_motion_override(
        receipt("hold", terminal_stop_authorized=False),
        rotate_gain=1.5,
        max_angular_rps=0.35,
    )
    stopped = terminal_motion_override(
        receipt("stop", terminal_stop_authorized=True),
        rotate_gain=1.5,
        max_angular_rps=0.35,
    )
    assert pending.assert_estop is False
    assert stopped.assert_estop is True


def test_malformed_turn_fails_closed_to_zero():
    result = terminal_motion_override(
        receipt("atomic_turn", terminal_turn_error_left_rad="bad"),
        rotate_gain=1.5,
        max_angular_rps=0.35,
    )
    assert result.applied is True
    assert result.command.linear_x == 0.0
    assert result.command.angular_z == 0.0
