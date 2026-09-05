import math

import pytest

from heading_turn import HeadingTurn, wrap


def ns(t):
    return int(t * 1e9)


def test_turn_uses_image_time_heading_and_wraps_across_pi():
    turn = HeadingTurn()
    turn.observe(ns(1), math.radians(170))
    turn.observe(ns(1.5), math.radians(175))
    assert turn.start(math.radians(90), ns(1), ns(1.5), 1.5)
    assert turn.target_yaw == pytest.approx(math.radians(-100))
    command = turn.step(ns(1.5), 1.5, 1.5, 0.55)
    assert command.linear_x == 0
    assert command.angular_z == 0.55
    # Simulated measured rotation, not integration of commanded velocity.
    for i in range(1, 17):
        t = 1.5 + i * 0.1
        yaw = wrap(math.radians(175 + 5 * i))
        turn.observe(ns(t), yaw)
        command = turn.step(ns(t), t, 1.5, 0.55)
    assert turn.phase == "complete"
    assert not turn.active
    assert command.angular_z == command.linear_x == 0


def test_fresh_feedback_keeps_turn_continuous_beyond_old_action_budget():
    turn = HeadingTurn()
    turn.observe(ns(1), 0)
    assert turn.start(-2, ns(1), ns(1), 1)
    for i in range(1, 81):
        t = 1 + i * 0.1
        turn.observe(ns(t), -i * 0.01)
        command = turn.step(ns(t), t, 1.5, 0.55)
        assert command.angular_z < 0
        assert command.linear_x == 0
        assert turn.active


def test_no_body_rotation_does_not_complete_from_published_commands():
    turn = HeadingTurn()
    turn.observe(ns(1), 0)
    assert turn.start(2, ns(1), ns(1), 1)
    for i in range(1, 202):
        t = 1 + i * 0.1
        turn.observe(ns(t), 0)
        turn.step(ns(t), t, 1.5, 0.55)
    assert turn.phase == "heading_turn_timeout"
    assert turn.error_rad == pytest.approx(2)


@pytest.mark.parametrize("image,now", [(0, 1), (0.5, 1), (1, 2), (1, 0.5)])
def test_missing_misaligned_stale_future_heading_cannot_start(image, now):
    turn = HeadingTurn()
    turn.observe(ns(1), 0)
    assert not turn.start(2, ns(image), ns(now), now)


def test_feedback_dropout_and_jump_stop_without_rearming():
    for jump in (False, True):
        turn = HeadingTurn()
        turn.observe(ns(1), 0)
        assert turn.start(2, ns(1), ns(1), 1)
        t = 1.5
        if jump:
            t = 1.1
            turn.observe(ns(t), 2)
        command = turn.step(ns(t), t, 1.5, 0.55)
        assert not turn.active
        assert command.angular_z == 0
        assert turn.phase == ("heading_feedback_discontinuity" if jump else "heading_feedback_stale")
        turn.observe(ns(2), 0)
        assert turn.step(ns(2), 2, 1.5, 0.55).angular_z == 0


def test_reset_discards_target_but_keeps_observation_history():
    turn = HeadingTurn()
    turn.observe(ns(1), 0)
    turn.start(2, ns(1), ns(1), 1)
    turn.reset()
    assert not turn.active
    assert turn.target_yaw is None
    assert turn.reference(ns(1)) == 0
