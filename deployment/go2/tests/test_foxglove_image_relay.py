from pathlib import Path
from types import SimpleNamespace
import sys

import cv2
import numpy as np
import pytest


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "deployment/go2"))

from foxglove_image_relay import (  # noqa: E402
    battery_payload_from_message,
    build_operator_diagnostics,
    colorize_depth_preview,
    depth_message_to_u16,
    derive_operator_state,
    encode_jpeg,
    prepare_color_preview,
    render_status_card,
    resize_rgb_preview,
    rgb_message_to_bgr,
)
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from sensor_msgs.msg import BatteryState  # noqa: E402


def _message(
    array: np.ndarray,
    encoding: str,
    *,
    step: int | None = None,
    is_bigendian: bool = False,
) -> SimpleNamespace:
    height, width = array.shape[:2]
    return SimpleNamespace(
        width=width,
        height=height,
        encoding=encoding,
        step=step if step is not None else array.strides[0],
        is_bigendian=is_bigendian,
        data=array.tobytes(),
    )


def test_rgb_preview_converts_resizes_and_encodes_jpeg():
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    bgr = rgb_message_to_bgr(_message(rgb, "rgb8"))
    assert np.array_equal(bgr[0, 0], [0, 0, 255])

    preview = resize_rgb_preview(bgr, 640, 360)
    decoded = cv2.imdecode(
        np.frombuffer(encode_jpeg(preview, 75), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert decoded.shape == (360, 640, 3)
    assert int(decoded[180, 320, 2]) > 240


def test_rgb_decoder_respects_padded_rows():
    rows = np.zeros((2, 8), dtype=np.uint8)
    rows[:, :6] = np.array([255, 0, 0, 0, 255, 0], dtype=np.uint8)
    message = SimpleNamespace(
        width=2,
        height=2,
        encoding="rgb8",
        step=8,
        is_bigendian=False,
        data=rows.tobytes(),
    )
    bgr = rgb_message_to_bgr(message)
    assert bgr.shape == (2, 2, 3)
    assert np.array_equal(bgr[0, 0], [0, 0, 255])
    assert np.array_equal(bgr[0, 1], [0, 255, 0])


def test_arrival_preview_can_preserve_compact_native_aspect():
    arrival = np.zeros((272, 480, 3), dtype=np.uint8)
    arrival[:, :240, 0] = 255
    message = _message(arrival, "rgb8")

    native = prepare_color_preview(message, 640, 360, resize=False)
    resized = prepare_color_preview(message, 640, 360, resize=True)

    assert native.shape == (272, 480, 3)
    assert resized.shape == (360, 640, 3)
    assert np.array_equal(native[100, 100], [0, 0, 255])


def test_operator_status_card_is_readable_jpeg_at_configured_size():
    payload = {
        "enabled": False,
        "estop": True,
        "arrival_latched": False,
        "rgbd_age_s": 0.08,
        "plan_age_s": 0.31,
        "clearance_m": 1.42,
        "cmd_vx": 0.0,
        "cmd_wz": 0.0,
        "image_goal_loaded": True,
        "phase": "revisit_query",
        "stop_reason": "estop",
        "last_error": "",
    }
    card = render_status_card(payload, 720, 272)
    decoded = cv2.imdecode(
        np.frombuffer(encode_jpeg(card, 80), dtype=np.uint8), cv2.IMREAD_COLOR
    )

    assert card.shape == (272, 720, 3)
    assert decoded.shape == (272, 720, 3)
    assert np.unique(card.reshape(-1, 3), axis=0).shape[0] > 10


def test_survey_status_card_derives_active_and_paused_from_legacy_status():
    payload = {
        "enabled": False,
        "estop": True,
        "rgbd_age_s": 0.08,
        "phase": "memory_recording",
        "pause_memory_recording": True,
        "frames_recorded": 40,
        "goal_candidates_captured": 0,
        "last_receipt_event": "survey_start",
        "stop_reason": "memory_recording",
    }

    paused = render_status_card(payload, 720, 272)
    active = render_status_card(
        {**payload, "pause_memory_recording": False, "frames_recorded": 41},
        720,
        272,
    )

    assert paused.shape == (272, 720, 3)
    assert active.shape == paused.shape
    assert not np.array_equal(paused, active)
    assert np.unique(paused.reshape(-1, 3), axis=0).shape[0] > 10


def test_battery_state_is_rendered_live_and_offline_without_stale_soc():
    message = BatteryState()
    message.present = True
    message.percentage = 0.73
    message.voltage = 28.7
    message.current = -1.4
    message.cell_voltage = [4.01, 4.00]
    live = battery_payload_from_message(message)
    assert live["online"] is True
    assert live["soc_pct"] == pytest.approx(73.0)
    assert live["voltage_v"] == pytest.approx(28.7)
    assert live["current_a"] == pytest.approx(-1.4)
    assert live["cell_min_v"] == pytest.approx(4.0)
    assert live["cell_max_v"] == pytest.approx(4.01)

    payload = {
        "enabled": False,
        "estop": True,
        "phase": "revisit_query",
        "go2_battery": live,
    }
    live_card = render_status_card(payload, 720, 272)
    offline_card = render_status_card(
        {**payload, "go2_battery": {"online": False}}, 720, 272
    )
    assert not np.array_equal(live_card, offline_card)

    message.present = False
    offline = battery_payload_from_message(message)
    assert offline["online"] is False
    assert offline["soc_pct"] is None


@pytest.mark.parametrize(
    ("payload", "expected"),
    [
        (
            {
                "phase": "memory_recording",
                "survey_state": "ACTIVE",
                "enabled": False,
                "estop": True,
            },
            {
                "mode": "SURVEY",
                "activity": "SURVEY_ACTIVE",
                "safety": "LOCKED",
                "go2": "OFFLINE",
            },
        ),
        (
            {
                "phase": "memory_recording",
                "survey_state": "PAUSED",
                "enabled": False,
                "estop": True,
                "go2_battery": {"online": True},
            },
            {
                "mode": "SURVEY",
                "activity": "SURVEY_PAUSED",
                "safety": "LOCKED",
                "go2": "ONLINE",
            },
        ),
        (
            {
                "phase": "revisit_query",
                "survey_state": "INACTIVE",
                "enabled": True,
                "estop": False,
            },
            {
                "mode": "REVISIT",
                "activity": "REVISITING",
                "safety": "ENABLED",
                "go2": "OFFLINE",
            },
        ),
        (
            {
                "phase": "revisit_query",
                "server_initialized": True,
                "enabled": False,
                "estop": True,
            },
            {
                "mode": "REVISIT",
                "activity": "REVISIT_READY",
                "safety": "LOCKED",
                "go2": "OFFLINE",
            },
        ),
        (
            {
                "server_initialized": True,
                "enabled": False,
                "estop": True,
            },
            {
                "mode": "IDLE",
                "activity": "READY",
                "safety": "LOCKED",
                "go2": "OFFLINE",
            },
        ),
    ],
)
def test_operator_state_keeps_workflow_safety_and_connection_independent(
    payload, expected
):
    assert derive_operator_state(payload) == expected


def test_operator_diagnostics_expose_raw_phase_and_health_details():
    payload = {
        "phase": "revisit_query",
        "survey_state": "INACTIVE",
        "server_initialized": True,
        "enabled": True,
        "estop": False,
        "rgbd_age_s": 0.08,
        "rgb_depth_skew_s": 0.01,
        "plan_age_s": 0.3,
        "candidate_count": 5,
        "go2_battery": {"online": False},
    }
    diagnostics = build_operator_diagnostics(payload)
    by_name = {status.name: status for status in diagnostics.status}

    assert set(by_name) == {
        "MemNav/Workflow",
        "MemNav/RGB-D",
        "MemNav/Policy",
        "MemNav/Go2",
    }
    workflow = by_name["MemNav/Workflow"]
    workflow_values = {value.key: value.value for value in workflow.values}
    assert workflow.level == DiagnosticStatus.OK
    assert workflow.message == "REVISITING"
    assert workflow_values["mode"] == "REVISIT"
    assert workflow_values["raw_phase"] == "revisit_query"
    assert by_name["MemNav/RGB-D"].level == DiagnosticStatus.OK
    assert by_name["MemNav/Go2"].level == DiagnosticStatus.WARN


def test_operator_fault_has_error_diagnostic_without_hiding_mode():
    payload = {
        "phase": "revisit_query",
        "server_initialized": True,
        "enabled": False,
        "estop": True,
        "last_error": "policy timeout",
    }
    state = derive_operator_state(payload)
    diagnostics = build_operator_diagnostics(payload)
    workflow = next(
        status for status in diagnostics.status if status.name == "MemNav/Workflow"
    )

    assert state["mode"] == "REVISIT"
    assert state["activity"] == "FAULT"
    assert workflow.level == DiagnosticStatus.ERROR
    assert workflow.message == "policy timeout"


def test_depth_preview_preserves_invalid_mask_and_colorizes_range():
    depth = np.array([[0, 200], [1000, 4000]], dtype=np.uint16)
    decoded = depth_message_to_u16(_message(depth, "16UC1"))
    assert np.array_equal(decoded, depth)

    preview = colorize_depth_preview(decoded, 2, 2, 200, 4000)
    assert preview.shape == (2, 2, 3)
    assert np.array_equal(preview[0, 0], [0, 0, 0])
    assert np.any(preview[0, 1] != 0)
    assert np.any(preview[1, 1] != preview[0, 1])


@pytest.mark.parametrize(
    ("encoding", "decoder"),
    [("mono8", rgb_message_to_bgr), ("32FC1", depth_message_to_u16)],
)
def test_unsupported_encodings_fail_closed(encoding, decoder):
    array = np.zeros((2, 2), dtype=np.uint8)
    with pytest.raises(ValueError, match="unsupported"):
        decoder(_message(array, encoding))
