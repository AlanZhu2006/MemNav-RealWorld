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
    build_arrival_diagnostics,
    build_observer_payload,
    build_operator_diagnostics,
    colorize_depth_preview,
    depth_message_to_u16,
    derive_arrival_state,
    derive_operator_state,
    encode_jpeg,
    image_stamp_seconds,
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
                "workflow": "SURVEY_RECORDING",
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
                "workflow": "SURVEY_PAUSED",
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
                "workflow": "REVISIT_ACTIVE",
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
                "workflow": "REVISIT_READY",
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
                "workflow": "IDLE_READY",
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


def test_operator_diagnostics_are_fixed_glanceable_six_row_summary():
    payload = {
        "phase": "revisit_query",
        "survey_state": "INACTIVE",
        "server_initialized": True,
        "enabled": True,
        "estop": False,
        "rgbd_age_s": 0.08,
        "rgb_depth_skew_s": 0.01,
        "plan_age_s": 0.3,
        "clearance_m": 1.42,
        "depth_hard_stop_m": 0.45,
        "candidate_count": 5,
        "go2_battery": {
            "online": True,
            "soc_pct": 73.0,
            "voltage_v": 28.7,
        },
    }
    diagnostics = build_operator_diagnostics(payload)
    by_name = {status.name: status for status in diagnostics.status}

    expected_names = [
        "MemNav/Overall",
        "MemNav/Mode",
        "MemNav/Front depth",
        "MemNav/Battery",
        "MemNav/Image refresh",
        "MemNav/Policy refresh",
    ]
    assert list(by_name) == expected_names
    assert all(not status.values for status in diagnostics.status)
    assert by_name["MemNav/Overall"].message == "OK · MOTION ON"
    assert by_name["MemNav/Mode"].message == "REVISIT · RUNNING"
    assert by_name["MemNav/Front depth"].message == "1.42 m · CLEAR"
    assert by_name["MemNav/Battery"].message == "73%"
    assert by_name["MemNav/Image refresh"].message == "FRESH · 0.08 s"
    assert by_name["MemNav/Policy refresh"].message == "FRESH · 0.30 s"


def test_camera_only_observer_reports_ready_without_claiming_policy_online():
    payload = build_observer_payload(
        now=12.0,
        last_rgb_received=11.96,
        last_depth_received=11.94,
        last_rgb_stamp_s=101.000,
        last_depth_stamp_s=100.985,
        clearance_m=1.24,
        battery={"online": True, "soc_pct": 68.0},
    )
    diagnostics = build_operator_diagnostics(payload)
    by_name = {status.name: status for status in diagnostics.status}

    assert payload["enabled"] is False
    assert payload["estop"] is True
    assert payload["rgbd_age_s"] == pytest.approx(0.06)
    assert payload["rgb_depth_skew_s"] == pytest.approx(0.015)
    assert by_name["MemNav/Overall"].message == "OK · LOCKED"
    assert by_name["MemNav/Mode"].message == "READY · CAMERA ONLY"
    assert by_name["MemNav/Front depth"].message == "1.24 m · CLEAR"
    assert by_name["MemNav/Policy refresh"].message == "OFF · NOT STARTED"
    assert by_name["MemNav/Policy refresh"].level == DiagnosticStatus.STALE


def test_observer_hides_old_clearance_when_depth_stream_is_stale():
    payload = build_observer_payload(
        now=12.0,
        last_rgb_received=11.95,
        last_depth_received=10.0,
        last_rgb_stamp_s=None,
        last_depth_stamp_s=None,
        clearance_m=1.24,
        battery={"online": False},
    )

    assert payload["rgbd_age_s"] == pytest.approx(2.0)
    assert payload["clearance_m"] is None


def test_image_stamp_ignores_zero_sentinel_and_converts_nanoseconds():
    zero = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
    )
    stamped = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=4, nanosec=250_000_000))
    )

    assert image_stamp_seconds(zero) is None
    assert image_stamp_seconds(stamped) == pytest.approx(4.25)


@pytest.mark.parametrize(
    ("payload", "expected_message"),
    [
        ({"server_initialized": False}, "OFFLINE"),
        (
            {"server_initialized": True, "enabled": False, "estop": True},
            "READY",
        ),
        (
            {
                "phase": "memory_recording",
                "survey_state": "ACTIVE",
                "server_initialized": True,
                "enabled": False,
                "estop": True,
            },
            "SURVEY · RECORDING",
        ),
        (
            {
                "phase": "revisit_query",
                "server_initialized": True,
                "enabled": True,
                "estop": False,
            },
            "REVISIT · RUNNING",
        ),
        (
            {
                "server_initialized": True,
                "enabled": True,
                "estop": False,
            },
            "NAVIGATION · RUNNING",
        ),
        (
            {
                "phase": "revisit_query",
                "server_initialized": True,
                "arrival_latched": True,
                "enabled": False,
                "estop": True,
            },
            "ARRIVED",
        ),
    ],
)
def test_operator_mode_is_one_readable_stage(payload, expected_message):
    diagnostics = build_operator_diagnostics(payload)
    mode = next(
        status for status in diagnostics.status if status.name == "MemNav/Mode"
    )

    assert mode.message == expected_message


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
    overall = next(
        status for status in diagnostics.status if status.name == "MemNav/Overall"
    )

    assert state["mode"] == "REVISIT"
    assert state["activity"] == "FAULT"
    assert overall.level == DiagnosticStatus.ERROR
    assert overall.message == "FAULT"


@pytest.mark.parametrize(
    ("clearance_m", "expected_level", "expected_message"),
    [
        (None, DiagnosticStatus.STALE, "OFFLINE"),
        (0.40, DiagnosticStatus.ERROR, "0.40 m · STOP"),
        (0.60, DiagnosticStatus.OK, "0.60 m · CLEAR"),
        (0.90, DiagnosticStatus.OK, "0.90 m · CLEAR"),
    ],
)
def test_clearance_diagnostic_explains_safety_thresholds(
    clearance_m, expected_level, expected_message
):
    diagnostics = build_operator_diagnostics(
        {
            "server_initialized": True,
            "enabled": False,
            "estop": True,
            "clearance_m": clearance_m,
            "depth_hard_stop_m": 0.45,
            "go2_battery": {"online": False},
        }
    )
    clearance = next(
        status
        for status in diagnostics.status
        if status.name == "MemNav/Front depth"
    )

    assert clearance.level == expected_level
    assert clearance.message == expected_message


@pytest.mark.parametrize(
    ("payload", "expected"),
    [
        ({"latest_rgb_ready": True, "armed": False, "result": None}, "STANDBY"),
        ({"latest_rgb_ready": True, "armed": True, "result": None}, "CHECKING"),
        (
            {
                "latest_rgb_ready": True,
                "armed": True,
                "result": {"matched": False},
            },
            "NO_MATCH",
        ),
        (
            {
                "latest_rgb_ready": True,
                "armed": True,
                "result": {"matched": True, "confirmed": False},
            },
            "MATCHING",
        ),
        (
            {
                "latest_rgb_ready": True,
                "armed": False,
                "result": {"matched": True},
            },
            "MATCH",
        ),
        (
            {
                "latest_rgb_ready": True,
                "arrival_latched": True,
                "result": {"matched": True, "confirmed": True},
            },
            "ARRIVED",
        ),
        ({"latest_rgb_ready": False}, "NO_RGB"),
        ({"latest_rgb_ready": True, "error": "decode failed"}, "ERROR"),
    ],
)
def test_arrival_state_reduces_json_to_one_operator_verdict(payload, expected):
    assert derive_arrival_state(payload) == expected


def test_arrival_diagnostics_keep_match_evidence_without_marking_no_match_faulty():
    payload = {
        "schema": "navdp_rgb_arrival_v1",
        "armed": True,
        "arrival_latched": False,
        "phase": "revisit_query",
        "latest_rgb_ready": True,
        "error": "",
        "result": {
            "matched": False,
            "confirmed": False,
            "reason": "low_inlier_ratio",
            "good_matches": 48,
            "inliers": 19,
            "inlier_ratio": 0.396,
            "image_scale": 0.82,
        },
    }
    diagnostics = build_arrival_diagnostics(payload)
    status = diagnostics.status[0]
    values = {value.key: value.value for value in status.values}

    assert status.name == "MemNav/Arrival"
    assert status.hardware_id == "rgb_arrival"
    assert status.level == DiagnosticStatus.OK
    assert status.message == "No goal match"
    assert values["reason"] == "low_inlier_ratio"
    assert values["inliers"] == "19"
    assert values["inlier_ratio"] == "0.396"


def test_operator_diagnostics_keep_arrival_detail_out_of_compact_summary():
    operator = {
        "phase": "revisit_query",
        "server_initialized": True,
        "enabled": False,
        "estop": True,
    }
    arrival = {
        "latest_rgb_ready": True,
        "armed": False,
        "result": None,
    }
    diagnostics = build_operator_diagnostics(
        operator, arrival_payload=arrival
    )

    assert len(diagnostics.status) == 6
    assert all(status.name != "MemNav/Arrival" for status in diagnostics.status)


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
