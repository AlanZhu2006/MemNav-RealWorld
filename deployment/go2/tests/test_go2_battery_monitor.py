import math
from pathlib import Path
from types import SimpleNamespace
import sys
import threading

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import go2_battery_monitor as battery_monitor_module  # noqa: E402
from go2_battery_monitor import (  # noqa: E402
    battery_message,
    Go2BatteryMonitor,
    network_link_ready,
    sample_from_low_state,
)
from go2_cmd_bridge import Go2CmdBridge  # noqa: E402


def _low_state(*, soc=73, voltage=28.7, current=-1.4):
    return SimpleNamespace(
        power_v=voltage,
        power_a=current,
        bms_state=SimpleNamespace(
            soc=soc,
            cell_vol=[4010, 4005, 0, 3998],
        ),
    )


def test_live_low_state_becomes_standard_ros_battery_state():
    sample = sample_from_low_state(_low_state(), received_monotonic=10.0)
    message = battery_message(
        sample, now_monotonic=11.0, offline_timeout_s=2.0
    )

    assert message.present is True
    assert message.percentage == 0.73
    assert message.voltage == 28.7
    assert message.current == -1.4
    assert list(message.cell_voltage) == pytest.approx([4.01, 4.005, 3.998])
    assert message.location == "unitree_go2"


def test_stale_or_missing_low_state_is_explicitly_offline_without_stale_values():
    sample = sample_from_low_state(_low_state(), received_monotonic=10.0)
    stale = battery_message(
        sample, now_monotonic=12.1, offline_timeout_s=2.0
    )
    missing = battery_message(
        None, now_monotonic=12.1, offline_timeout_s=2.0
    )

    for message in (stale, missing):
        assert message.present is False
        assert math.isnan(message.percentage)
        assert math.isnan(message.voltage)
        assert math.isnan(message.current)
        assert list(message.cell_voltage) == []


def test_invalid_unitree_soc_is_not_presented_as_a_percentage():
    sample = sample_from_low_state(
        _low_state(soc=255), received_monotonic=10.0
    )
    message = battery_message(
        sample, now_monotonic=10.1, offline_timeout_s=2.0
    )

    assert message.present is True
    assert math.isnan(message.percentage)


def test_high_rate_lowstate_is_sampled_without_losing_battery_freshness(monkeypatch):
    monitor = object.__new__(Go2BatteryMonitor)
    monitor._sample_period_s = 0.2
    monitor._next_sample_monotonic = 0.0
    monitor._lock = threading.Lock()
    monitor._sample = None
    now = [10.0]
    monkeypatch.setattr(battery_monitor_module.time, "monotonic", lambda: now[0])

    monitor.on_low_state(_low_state(soc=73))
    assert monitor._sample.soc_pct == 73
    assert monitor._sample.received_monotonic == 10.0

    now[0] = 10.1
    monitor.on_low_state(_low_state(soc=20))
    assert monitor._sample.soc_pct == 73

    now[0] = 10.21
    monitor.on_low_state(_low_state(soc=72))
    assert monitor._sample.soc_pct == 72
    assert monitor._sample.received_monotonic == 10.21


def test_command_bridge_reuses_its_lowstate_subscription_for_battery(monkeypatch):
    bridge = object.__new__(Go2CmdBridge)
    bridge._battery_sample_period_s = 0.2
    bridge._next_battery_sample_monotonic = 0.0
    bridge._battery_lock = threading.Lock()
    bridge._battery_sample = None
    bridge.remote_deadband = 0.12
    bridge.latest_remote_stamp = 0.0
    monkeypatch.setattr(battery_monitor_module.time, "monotonic", lambda: 10.0)

    bridge.on_low_state(_low_state(soc=68))

    assert bridge._battery_sample.soc_pct == 68
    assert bridge._battery_sample.received_monotonic == 10.0


def test_zero_linear_floor_does_not_amplify_small_commands():
    assert Go2CmdBridge.apply_floor(0.03, 0.0) == pytest.approx(0.03)
    assert Go2CmdBridge.apply_floor(0.10, 0.0) == pytest.approx(0.10)


def test_network_link_requires_interface_carrier(tmp_path):
    interface = tmp_path / "eth0"
    interface.mkdir()
    (interface / "carrier").write_text("0\n", encoding="utf-8")
    assert network_link_ready("eth0", sys_class_net=tmp_path) is False

    (interface / "carrier").write_text("1\n", encoding="utf-8")
    assert network_link_ready("eth0", sys_class_net=tmp_path) is True
    assert network_link_ready("missing", sys_class_net=tmp_path) is False
