import pytest

from terminal_motion_override import terminal_motion_override


def receipt(disposition, **updates):
    value = {
        "terminal_handoff_schema": "cec_direct_metric_handoff_v3_20260831",
        "cec_takeover": True,
        "terminal_handoff_disposition": disposition,
        "terminal_local_latched": disposition in {"hold", "stop"},
        "terminal_stop_authorized": False,
        "terminal_proof_active": disposition not in {"long_range", "native"},
    }
    value.update(updates)
    return value


def test_long_range_and_bearing_local_leave_navdp_command_untouched():
    for disposition in ("long_range", "bearing_local"):
        result = terminal_motion_override(
            receipt(disposition), rotate_gain=1.5, max_angular_rps=0.35
        )
        assert result.applied is False
        assert result.command is None


def test_rear_target_turn_is_bounded_and_keeps_translation_zero():
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
