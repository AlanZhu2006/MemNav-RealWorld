import threading
import time
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from latency_motion_guard import StopPlanActConfig, StopPlanActGate  # noqa: E402
from navdp_ros_node import NavDPGo2Adapter  # noqa: E402
from trajectory_control import VelocityCommand  # noqa: E402


def adapter_at(*, now: float) -> NavDPGo2Adapter:
    adapter = object.__new__(NavDPGo2Adapter)
    adapter._lock = threading.RLock()
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
