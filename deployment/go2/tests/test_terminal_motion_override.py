import pytest

from terminal_motion_override import (
    CERTIFIED_TURN_CREEP_MPS,
    CERTIFIED_TURN_MAINTENANCE_CREEP_MPS,
    CertifiedTurnBootstrap,
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
    assert result.command.linear_x == CERTIFIED_TURN_CREEP_MPS
    assert result.command.angular_z == pytest.approx(-0.35)
    assert result.assert_estop is False
    assert result.reason == "certified_long_range_atomic_turn"


def test_certified_turn_creep_is_one_short_gait_bootstrap_pulse():
    gate = CertifiedTurnBootstrap(duration_s=0.60)
    command = VelocityCommand(
        linear_x=CERTIFIED_TURN_CREEP_MPS,
        angular_z=0.20,
    )

    locked = gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=False,
        now_s=5.0,
    )
    assert locked.phase == "inactive"
    assert locked.command.linear_x == CERTIFIED_TURN_CREEP_MPS

    started = gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=True,
        now_s=10.0,
    )
    assert started.phase == "gait_bootstrap"
    assert started.command.linear_x == CERTIFIED_TURN_CREEP_MPS

    gate.record_execution(
        started.command,
        reason="certified_long_range_atomic_turn",
        dt_s=0.61,
    )

    maintenance = gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=True,
        now_s=10.61,
    )
    assert maintenance.phase == "maintenance_creep"
    assert maintenance.command.linear_x == CERTIFIED_TURN_MAINTENANCE_CREEP_MPS
    assert maintenance.command.angular_z == 0.20


def test_certified_turn_bootstrap_rearms_only_for_a_new_turn():
    gate = CertifiedTurnBootstrap(duration_s=0.10)
    left = VelocityCommand(linear_x=CERTIFIED_TURN_CREEP_MPS, angular_z=0.20)
    right = VelocityCommand(linear_x=CERTIFIED_TURN_CREEP_MPS, angular_z=-0.20)
    gate.apply(
        left,
        reason="certified_atomic_turn",
        motion_allowed=True,
        now_s=1.0,
    )
    gate.record_execution(
        left,
        reason="certified_atomic_turn",
        dt_s=0.20,
    )
    expired = gate.apply(
        left,
        reason="certified_atomic_turn",
        motion_allowed=True,
        now_s=1.2,
    )
    assert expired.phase == "maintenance_creep"
    assert expired.command.linear_x == CERTIFIED_TURN_MAINTENANCE_CREEP_MPS

    reversed_turn = gate.apply(
        right,
        reason="certified_atomic_turn",
        motion_allowed=True,
        now_s=1.3,
    )
    assert reversed_turn.phase == "gait_bootstrap"
    assert reversed_turn.command.linear_x == CERTIFIED_TURN_CREEP_MPS


def test_certified_turn_bootstrap_fails_closed_after_max_duration():
    gate = CertifiedTurnBootstrap(duration_s=0.60, max_duration_s=20.0)
    command = VelocityCommand(
        linear_x=CERTIFIED_TURN_CREEP_MPS,
        angular_z=0.20,
    )
    gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=True,
        now_s=10.0,
    )
    gate.record_execution(
        command,
        reason="certified_long_range_atomic_turn",
        dt_s=20.0,
    )

    expired = gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=True,
        now_s=30.0,
    )

    assert expired.phase == "turn_timeout"
    assert expired.elapsed_s == pytest.approx(20.0)
    assert expired.command.linear_x == 0.0
    assert expired.command.angular_z == 0.0


def test_certified_turn_watchdog_excludes_stationary_plan_waits():
    gate = CertifiedTurnBootstrap(duration_s=0.60)
    command = VelocityCommand(
        linear_x=CERTIFIED_TURN_CREEP_MPS,
        angular_z=0.20,
    )
    gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=True,
        now_s=10.0,
    )

    paused = gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=False,
        now_s=40.0,
    )
    assert paused.phase == "inactive"

    gate.record_execution(
        command,
        reason="certified_long_range_atomic_turn",
        dt_s=19.9,
    )
    active = gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=True,
        now_s=70.0,
    )
    gate.record_execution(
        command,
        reason="certified_long_range_atomic_turn",
        dt_s=0.1,
    )
    expired = gate.apply(
        command,
        reason="certified_long_range_atomic_turn",
        motion_allowed=True,
        now_s=100.0,
    )

    assert active.phase == "maintenance_creep"
    assert expired.phase == "turn_timeout"
    assert expired.elapsed_s == pytest.approx(20.0)


@pytest.mark.parametrize("duration", [-0.1, float("nan"), float("inf")])
def test_certified_turn_bootstrap_rejects_invalid_duration(duration):
    with pytest.raises(ValueError):
        CertifiedTurnBootstrap(duration_s=duration)


@pytest.mark.parametrize("duration", [-0.1, 0.0, float("nan"), float("inf")])
def test_certified_turn_bootstrap_rejects_invalid_max_duration(duration):
    with pytest.raises(ValueError):
        CertifiedTurnBootstrap(max_duration_s=duration)


def test_certified_turn_bootstrap_rejects_max_shorter_than_bootstrap():
    with pytest.raises(ValueError):
        CertifiedTurnBootstrap(duration_s=0.60, max_duration_s=0.59)


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


def test_rear_target_turn_is_bounded_and_keeps_forward_creep():
    result = terminal_motion_override(
        receipt("atomic_turn", terminal_turn_error_left_rad=-3.10),
        rotate_gain=1.5,
        max_angular_rps=0.35,
    )
    assert result.applied is True
    assert result.command.linear_x == CERTIFIED_TURN_CREEP_MPS
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
    assert result.command.linear_x == CERTIFIED_TURN_CREEP_MPS
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
