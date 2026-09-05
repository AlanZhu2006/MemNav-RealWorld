import threading
import time
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from latency_motion_guard import StopPlanActConfig, StopPlanActGate  # noqa: E402
from navdp_ros_node import NavDPGo2Adapter  # noqa: E402
from heading_turn import HeadingTurn
from trajectory_control import VelocityCommand  # noqa: E402


def adapter_at(*, now: float) -> NavDPGo2Adapter:
    adapter = object.__new__(NavDPGo2Adapter)
    adapter._lock = threading.RLock()
    adapter._heading_turn = HeadingTurn()
    adapter._enabled = True
    adapter._estop = False
    adapter._rgb = object()
    adapter._depth_m = object()
    adapter._image_goal = object()
    adapter._trajectory = object()
    adapter._plan_monotonic = now - 1.0
    adapter._rgbd_monotonic = now
    adapter._rgbd_source_stamp_ns = 2_000
    adapter._rgbd_source_age_at_receive_s = None
    adapter.sensor_timeout_s = 1.0
    adapter.trajectory_timeout_s = 5.0
    adapter._last_error = ""
    adapter.stop_plan_act_config = StopPlanActConfig()
    adapter._stop_plan_act_gate = StopPlanActGate(adapter.stop_plan_act_config)
    adapter._stop_plan_act_gate.install_plan(adapter._plan_monotonic)
    return adapter


def test_adapter_action_clock_uses_first_command_not_plan_completion():
    now = time.monotonic()
    adapter = adapter_at(now=now)

    # The plan is already one second old, but no command has yet been emitted.
    assert adapter._motion_block_reason(now) is None


def test_continuous_heading_turn_bypasses_visual_pulse_but_obeys_estop_and_rgbd():
    now = time.monotonic()
    adapter = adapter_at(now=now)
    adapter._plan_monotonic = now - 10
    adapter._heading_turn.active = True
    assert adapter._motion_block_reason(now) is None
    adapter._estop = True
    assert adapter._motion_block_reason(now) == "estop"
    adapter._estop = False
    adapter._rgbd_monotonic = now - 2
    assert adapter._motion_block_reason(now) == "rgbd_stale"


def test_heading_control_runs_continuously_then_consumes_plan(monkeypatch):
    import numpy as np
    from trajectory_control import ControllerConfig, DepthSafetyConfig
    import navdp_ros_node as module

    clock = [10.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    adapter = adapter_at(now=clock[0])
    adapter._ros_now_ns = lambda: int(clock[0] * 1e9)
    adapter._turn_image_ns = int(clock[0] * 1e9)
    adapter._heading_turn.observe(adapter._turn_image_ns, 0)
    adapter._terminal_motion_receipt = {"bearing_rad": 1.0}
    adapter._latency_motion_receipt = {"reason": "pass"}
    adapter._depth_m = np.ones((10, 10), dtype=np.float32) * 3
    adapter._target_command = VelocityCommand(angular_z=0.55)
    adapter._last_command = VelocityCommand()
    adapter.controller_config = ControllerConfig(max_angular_rps=0.55)
    adapter.depth_safety_config = DepthSafetyConfig()
    adapter.control_rate_hz = 20
    adapter.max_linear_accel_mps2 = 0.45
    adapter.max_angular_accel_rps2 = 1.0
    sent = []

    def publish(command, reason):
        sent.append(command)
        adapter._last_command = command
        adapter._stop_reason = reason

    adapter._publish_command = publish
    adapter._publish_zero = lambda reason: publish(VelocityCommand(), reason)
    adapter._control_tick()
    assert adapter._heading_turn.active
    for i in range(1, 21):
        clock[0] = 10 + i * 0.1
        adapter._rgbd_monotonic = clock[0]
        adapter._heading_turn.observe(adapter._ros_now_ns(), i * 0.05)
        adapter._control_tick()
        if not adapter._heading_turn.active:
            break
    assert all(command.angular_z > 0 for command in sent[:-1])
    assert all(command.linear_x == 0 for command in sent)
    assert sent[-1].angular_z == 0
    assert adapter._heading_turn.phase == "complete"
    assert adapter._terminal_motion_receipt == {}
    assert adapter._stop_plan_act_phase_locked(clock[0]) == "settling"


def test_adapter_stops_after_finite_published_command_window():
    now = time.monotonic()
    executing = adapter_at(now=now)
    executing._stop_plan_act_gate.note_command_published(
        now - 0.79, VelocityCommand(linear_x=0.01)
    )
    expired = adapter_at(now=now)
    expired._stop_plan_act_gate.note_command_published(
        now - 0.81, VelocityCommand(linear_x=0.01)
    )

    assert executing._motion_block_reason(now) is None
    assert expired._motion_block_reason(now) == "action_complete"


def test_adapter_never_requests_next_plan_during_command_execution():
    now = time.monotonic()
    adapter = adapter_at(now=now)
    adapter._stop_plan_act_gate.note_command_published(
        now - 0.20, VelocityCommand(linear_x=0.10)
    )
    adapter._inference_busy = False
    adapter._inference_event = threading.Event()
    adapter.plan_while_disabled = True

    adapter._request_inference()

    assert adapter._inference_event.is_set() is False


def test_adapter_requests_plan_only_after_stop_settle_and_new_source_frame():
    now = time.monotonic()
    adapter = adapter_at(now=now)
    adapter._stop_plan_act_gate.note_command_published(
        now - 0.40, VelocityCommand(linear_x=0.10)
    )
    adapter._stop_plan_act_gate.note_action_stopped(
        now - 0.20, stopped_ros_ns=1_000_000_000
    )
    adapter._rgbd_source_stamp_ns = 1_200_000_000
    adapter._inference_busy = False
    adapter._inference_event = threading.Event()
    adapter.plan_while_disabled = True

    adapter._request_inference()

    assert adapter._inference_event.is_set() is True


def test_adapter_rejects_pre_stop_frame_even_if_callback_is_recent():
    now = time.monotonic()
    adapter = adapter_at(now=now)
    adapter._stop_plan_act_gate.note_command_published(
        now - 0.40, VelocityCommand(linear_x=0.10)
    )
    adapter._stop_plan_act_gate.note_action_stopped(
        now - 0.20, stopped_ros_ns=1_000_000_000
    )
    adapter._rgbd_monotonic = now
    adapter._rgbd_source_stamp_ns = 1_100_000_000
    adapter._inference_busy = False
    adapter._inference_event = threading.Event()
    adapter.plan_while_disabled = True

    adapter._request_inference()

    assert adapter._inference_event.is_set() is False
