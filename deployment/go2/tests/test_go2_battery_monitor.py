import math
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_battery_monitor import (  # noqa: E402
    battery_message,
    network_link_ready,
    sample_from_low_state,
)


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


def test_network_link_requires_interface_carrier(tmp_path):
    interface = tmp_path / "eth0"
    interface.mkdir()
    (interface / "carrier").write_text("0\n", encoding="utf-8")
    assert network_link_ready("eth0", sys_class_net=tmp_path) is False

    (interface / "carrier").write_text("1\n", encoding="utf-8")
    assert network_link_ready("eth0", sys_class_net=tmp_path) is True
    assert network_link_ready("missing", sys_class_net=tmp_path) is False
