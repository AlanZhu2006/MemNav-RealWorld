#!/usr/bin/env python3
"""Publish bandwidth-bounded previews and native operator state for Foxglove.

The relay is deliberately observation-only.  NavDP and safety consumers keep
subscribing to the original RealSense topics; only the Foxglove layout uses
the lossy JPEG previews and read-only derived status topics.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CompressedImage, Image
from std_msgs.msg import String


def _rows(message: Any, bytes_per_pixel: int) -> np.ndarray:
    expected_step = int(message.width) * bytes_per_pixel
    step = int(message.step)
    if step < expected_step:
        raise ValueError(f"image step {step} is smaller than {expected_step}")
    expected_size = int(message.height) * step
    data = memoryview(message.data)
    if data.nbytes < expected_size:
        raise ValueError(f"image data has {data.nbytes} bytes; expected {expected_size}")
    return np.frombuffer(data[:expected_size], dtype=np.uint8).reshape(
        int(message.height), step
    )


def rgb_message_to_bgr(message: Any) -> np.ndarray:
    """Decode common ROS RGB image encodings into a contiguous BGR array."""

    encoding = str(message.encoding).lower()
    if encoding in {"rgb8", "bgr8", "8uc3"}:
        channels = 3
    elif encoding in {"rgba8", "bgra8", "8uc4"}:
        channels = 4
    else:
        raise ValueError(f"unsupported RGB encoding: {message.encoding}")
    packed = _rows(message, channels)[:, : int(message.width) * channels]
    image = np.ascontiguousarray(
        packed.reshape(int(message.height), int(message.width), channels)
    )
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding in {"rgba8", "8uc4"}:
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def depth_message_to_u16(message: Any) -> np.ndarray:
    """Decode a 16-bit ROS depth image, including row padding and endianness."""

    encoding = str(message.encoding).lower()
    if encoding not in {"16uc1", "mono16"}:
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    rows = _rows(message, 2)
    packed = np.ascontiguousarray(rows[:, : int(message.width) * 2])
    byte_order = ">u2" if bool(message.is_bigendian) else "<u2"
    depth = packed.view(np.dtype(byte_order)).reshape(
        int(message.height), int(message.width)
    )
    return np.ascontiguousarray(depth.astype(np.uint16, copy=False))


def resize_rgb_preview(image: np.ndarray, width: int, height: int) -> np.ndarray:
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def prepare_color_preview(
    message: Any, width: int, height: int, *, resize: bool
) -> np.ndarray:
    """Decode a color image and optionally resize it for display.

    Arrival debug images contain geometry overlays aligned to their source
    pixels.  Keep their native aspect ratio instead of forcing the camera
    preview dimensions and distorting that evidence.
    """

    image = rgb_message_to_bgr(message)
    if resize:
        return resize_rgb_preview(image, width, height)
    return image


def colorize_depth_preview(
    depth_mm: np.ndarray,
    width: int,
    height: int,
    minimum_mm: float,
    maximum_mm: float,
) -> np.ndarray:
    """Resize depth with nearest-neighbour sampling and apply Turbo colors."""

    resized = cv2.resize(depth_mm, (width, height), interpolation=cv2.INTER_NEAREST)
    invalid = resized == 0
    clipped = np.clip(resized.astype(np.float32), minimum_mm, maximum_mm)
    scaled = ((clipped - minimum_mm) * (255.0 / (maximum_mm - minimum_mm))).astype(
        np.uint8
    )
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    color[invalid] = 0
    return color


def encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    ok, encoded = cv2.imencode(
        ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        raise RuntimeError("OpenCV failed to encode JPEG preview")
    return encoded.tobytes()


_CARD_BACKGROUND = (30, 33, 39)
_CARD_SURFACE = (42, 46, 54)
_TEXT_PRIMARY = (238, 240, 244)
_TEXT_SECONDARY = (163, 171, 184)
_GOOD = (101, 210, 143)
_WARNING = (78, 190, 245)
_DANGER = (90, 92, 245)
_NEUTRAL = (128, 137, 150)


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if np.isfinite(result) else None


def _metric_color(
    value: float | None,
    *,
    good_max: float | None = None,
    danger_max: float | None = None,
    danger_min: float | None = None,
    good_min: float | None = None,
) -> tuple[int, int, int]:
    if value is None:
        return _NEUTRAL
    if good_max is not None and value <= good_max:
        return _GOOD
    if danger_max is not None and value > danger_max:
        return _DANGER
    if danger_min is not None and value <= danger_min:
        return _DANGER
    if good_min is not None and value >= good_min:
        return _GOOD
    return _WARNING


def _fit_text(value: Any, limit: int) -> str:
    text = "-" if value in (None, "") else str(value)
    if len(text) <= limit:
        return text
    return text[: max(1, limit - 3)] + "..."


def battery_payload_from_message(message: BatteryState) -> dict[str, Any]:
    """Convert ROS BatteryState into status-card values without stale fallback."""

    present = bool(message.present)
    percentage = _number(message.percentage)
    voltage = _number(message.voltage)
    current = _number(message.current)
    if percentage is not None and not 0.0 <= percentage <= 1.0:
        percentage = None
    cells = [
        float(value)
        for value in message.cell_voltage
        if math.isfinite(float(value)) and float(value) > 0.0
    ]
    return {
        "online": present,
        "soc_pct": None if not present or percentage is None else percentage * 100.0,
        "voltage_v": None if not present else voltage,
        "current_a": None if not present else current,
        "cell_min_v": None if not present or not cells else min(cells),
        "cell_max_v": None if not present or not cells else max(cells),
    }


def derive_operator_state(payload: dict[str, Any]) -> dict[str, str]:
    """Normalize protocol details into independent operator-facing states.

    Workflow, activity, safety and robot connectivity are deliberately kept
    separate. For example, an offline Go2 must not hide that the policy is in
    the Revisit phase.
    """

    phase = str(payload.get("phase") or "").strip()
    survey_state = str(payload.get("survey_state") or "").strip().upper()
    if not survey_state:
        if payload.get("stop_reason") == "survey_sealed":
            survey_state = "SEALED"
        elif phase == "memory_recording":
            survey_state = (
                "PAUSED" if payload.get("pause_memory_recording") else "ACTIVE"
            )
        else:
            survey_state = "INACTIVE"

    if phase == "memory_recording" or survey_state in {
        "ACTIVE",
        "PAUSED",
        "SEALED",
    }:
        mode = "SURVEY"
    elif phase == "revisit_query":
        mode = "REVISIT"
    elif not payload.get("server_initialized"):
        mode = "STARTING"
    else:
        mode = "IDLE"

    enabled = bool(payload.get("enabled"))
    estop = bool(payload.get("estop"))
    if enabled and not estop:
        safety = "ENABLED"
    elif not enabled and estop:
        safety = "LOCKED"
    else:
        safety = "INCONSISTENT"

    last_error = str(payload.get("last_error") or "").strip()
    if last_error:
        activity = "FAULT"
    elif payload.get("arrival_latched"):
        activity = "ARRIVED"
    elif mode == "SURVEY":
        activity = f"SURVEY_{survey_state}"
    elif mode == "REVISIT":
        activity = "REVISITING" if safety == "ENABLED" else "REVISIT_READY"
    elif mode == "STARTING":
        activity = "STARTING"
    elif safety == "ENABLED":
        activity = "NAVIGATING"
    else:
        activity = "READY"

    workflow_step = {
        "SURVEY_ACTIVE": "RECORDING",
        "SURVEY_PAUSED": "PAUSED",
        "SURVEY_SEALED": "SEALED",
        "REVISITING": "ACTIVE",
        "REVISIT_READY": "READY",
        "NAVIGATING": "ACTIVE",
        "ARRIVED": "ARRIVED",
        "FAULT": "FAULT",
        "READY": "READY",
        "STARTING": "STARTING",
    }.get(activity, activity.removeprefix("SURVEY_"))
    if mode == "STARTING":
        workflow = "STARTING"
    elif mode == "IDLE" and workflow_step == "ACTIVE":
        workflow = "NAVIGATION_ACTIVE"
    else:
        workflow = f"{mode}_{workflow_step}"

    battery = payload.get("go2_battery")
    if not isinstance(battery, dict):
        battery = {}
    go2 = "ONLINE" if battery.get("online") is True else "OFFLINE"
    return {
        "workflow": workflow,
        "mode": mode,
        "activity": activity,
        "safety": safety,
        "go2": go2,
    }


def derive_arrival_state(payload: dict[str, Any]) -> str:
    """Reduce either arrival-status schema to one operator verdict."""

    error = str(payload.get("error") or "").strip()
    result = payload.get("result")
    if not isinstance(result, dict):
        result = {}
    if error:
        return "ERROR"
    if payload.get("arrival_latched") or result.get("confirmed"):
        return "ARRIVED"
    if not payload.get("latest_rgb_ready"):
        return "NO_RGB"
    if payload.get("armed"):
        if not result:
            return "CHECKING"
        return "MATCHING" if result.get("matched") else "NO_MATCH"
    if result:
        return "MATCH" if result.get("matched") else "NO_MATCH"
    return "STANDBY"


def _diagnostic_value(value: Any) -> str:
    if value is None or value == "":
        return "-"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def _diagnostic_status(
    *,
    name: str,
    hardware_id: str,
    level: bytes,
    message: str,
    values: dict[str, Any],
) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.name = name
    status.hardware_id = hardware_id
    status.level = level
    status.message = message
    status.values = [
        KeyValue(key=key, value=_diagnostic_value(value))
        for key, value in values.items()
    ]
    return status


def _operator_workflow_message(state: dict[str, str], last_error: str) -> str:
    if last_error:
        return f"Fault · {last_error}"
    activity = {
        "SURVEY_ACTIVE": "Survey recording",
        "SURVEY_PAUSED": "Survey paused",
        "SURVEY_SEALED": "Survey sealed",
        "REVISITING": "Revisit navigating",
        "REVISIT_READY": "Revisit ready",
        "NAVIGATING": "Navigating",
        "ARRIVED": "Arrived",
        "FAULT": "Fault",
        "READY": "Ready",
        "STARTING": "Starting",
    }.get(state["activity"], state["activity"].replace("_", " ").title())
    safety = {
        "LOCKED": "motion locked",
        "ENABLED": "motion on",
        "INCONSISTENT": "check safety state",
    }.get(state["safety"], state["safety"].replace("_", " ").lower())
    return f"{activity} · {safety}"


def _arrival_summary_message(payload: dict[str, Any], state: str) -> str:
    error = str(payload.get("error") or "").strip()
    if error:
        return f"Fault · {error}"
    return {
        "STANDBY": "Waiting for goal check",
        "CHECKING": "Checking goal match",
        "NO_MATCH": "No goal match",
        "MATCHING": "Possible match · confirming",
        "MATCH": "Goal matched",
        "ARRIVED": "Arrival confirmed",
        "NO_RGB": "Camera image unavailable",
        "ERROR": "Goal check fault",
    }.get(state, state.replace("_", " ").title())


def build_arrival_diagnostic(payload: dict[str, Any]) -> DiagnosticStatus:
    """Convert JSON arrival evidence into one standard ROS diagnostic."""

    state = derive_arrival_state(payload)
    result = payload.get("result")
    if not isinstance(result, dict):
        result = {}
    if state == "ERROR":
        level = DiagnosticStatus.ERROR
    elif state == "NO_RGB":
        level = DiagnosticStatus.STALE
    else:
        # NO_MATCH is expected evidence while the robot is en route, not a
        # component-health warning.
        level = DiagnosticStatus.OK
    return _diagnostic_status(
        name="MemNav/Arrival",
        hardware_id="rgb_arrival",
        level=level,
        message=_arrival_summary_message(payload, state),
        values={
            "state": state,
            "schema": payload.get("schema"),
            "armed": payload.get("armed"),
            "arrival_latched": payload.get("arrival_latched"),
            "phase": payload.get("phase"),
            "latest_rgb_ready": payload.get("latest_rgb_ready"),
            "evaluation_age_s": payload.get("evaluation_age_s"),
            "reason": result.get("reason"),
            "matched": result.get("matched"),
            "confirmed": result.get("confirmed"),
            "consecutive_matches": result.get("consecutive_matches"),
            "good_matches": result.get("good_matches"),
            "inliers": result.get("inliers"),
            "inlier_ratio": result.get("inlier_ratio"),
            "target_coverage": result.get("target_coverage"),
            "current_coverage": result.get("current_coverage"),
            "center_offset_norm": result.get("center_offset_norm"),
            "image_scale": result.get("image_scale"),
            "rotation_deg": result.get("rotation_deg"),
            "reprojection_error_px": result.get("reprojection_error_px"),
        },
    )


def build_arrival_diagnostics(
    payload: dict[str, Any], *, stamp: Any | None = None
) -> DiagnosticArray:
    diagnostic_array = DiagnosticArray()
    if stamp is not None:
        diagnostic_array.header.stamp = stamp
    diagnostic_array.status.append(build_arrival_diagnostic(payload))
    return diagnostic_array


def build_operator_diagnostics(
    payload: dict[str, Any],
    *,
    arrival_payload: dict[str, Any] | None = None,
    stamp: Any | None = None,
) -> DiagnosticArray:
    """Build standard ROS diagnostics consumed by Foxglove's native panel."""

    state = derive_operator_state(payload)
    diagnostic_array = DiagnosticArray()
    if stamp is not None:
        diagnostic_array.header.stamp = stamp

    last_error = str(payload.get("last_error") or "").strip()
    workflow_level = DiagnosticStatus.ERROR if last_error else DiagnosticStatus.OK
    if state["mode"] == "STARTING" and not last_error:
        workflow_level = DiagnosticStatus.WARN
    diagnostic_array.status.append(
        _diagnostic_status(
            name="MemNav/Workflow",
            hardware_id="navdp",
            level=workflow_level,
            message=_operator_workflow_message(state, last_error),
            values={
                "mode": state["mode"],
                "activity": state["activity"],
                "workflow": state["workflow"],
                "safety": state["safety"],
                "enabled": payload.get("enabled"),
                "estop": payload.get("estop"),
                "raw_phase": payload.get("phase"),
                "survey_state": payload.get("survey_state"),
                "frames_recorded": payload.get("frames_recorded"),
                "goal_candidates": payload.get("goal_candidates_captured"),
                "active_goal_id": payload.get("active_goal_id"),
                "stop_reason": payload.get("stop_reason"),
            },
        )
    )

    rgbd_age = _number(payload.get("rgbd_age_s"))
    rgbd_skew = _number(payload.get("rgb_depth_skew_s"))
    if rgbd_age is None:
        sensor_level, sensor_message = DiagnosticStatus.STALE, "No RGB-D sample"
    elif rgbd_age > 0.75:
        sensor_level = DiagnosticStatus.ERROR
        sensor_message = f"Stale · {rgbd_age:.2f} s old"
    elif rgbd_age > 0.25 or (rgbd_skew is not None and rgbd_skew > 0.12):
        sensor_level = DiagnosticStatus.WARN
        sensor_message = f"Delayed · {rgbd_age:.2f} s old"
    else:
        sensor_level = DiagnosticStatus.OK
        sensor_message = f"Fresh · {rgbd_age:.2f} s old"
    diagnostic_array.status.append(
        _diagnostic_status(
            name="MemNav/RGB-D",
            hardware_id="realsense_d435i",
            level=sensor_level,
            message=sensor_message,
            values={
                "age_s": rgbd_age,
                "rgb_depth_skew_s": rgbd_skew,
                "clearance_m": payload.get("clearance_m"),
            },
        )
    )

    clearance = _number(payload.get("clearance_m"))
    hard_stop_m = _number(payload.get("depth_hard_stop_m"))
    slow_distance_m = _number(payload.get("depth_slow_distance_m"))
    hard_stop_m = 0.45 if hard_stop_m is None else hard_stop_m
    slow_distance_m = 0.80 if slow_distance_m is None else slow_distance_m
    if clearance is None:
        clearance_level = DiagnosticStatus.STALE
        clearance_message = "Unavailable · motion stops if needed"
    elif clearance <= hard_stop_m:
        clearance_level = DiagnosticStatus.ERROR
        clearance_message = f"{clearance:.2f} m · STOP zone"
    elif clearance < slow_distance_m:
        clearance_level = DiagnosticStatus.WARN
        clearance_message = f"{clearance:.2f} m · SLOW zone"
    else:
        clearance_level = DiagnosticStatus.OK
        clearance_message = f"{clearance:.2f} m · clear"
    diagnostic_array.status.append(
        _diagnostic_status(
            name="MemNav/Clearance",
            hardware_id="realsense_d435i",
            level=clearance_level,
            message=clearance_message,
            values={
                "clearance_m": clearance,
                "hard_stop_m": hard_stop_m,
                "slow_distance_m": slow_distance_m,
                "meaning": "front depth safety region",
            },
        )
    )

    initialized = bool(payload.get("server_initialized"))
    inference_busy = bool(payload.get("inference_busy"))
    plan_age = _number(payload.get("plan_age_s"))
    if last_error:
        policy_level, policy_message = DiagnosticStatus.ERROR, last_error
    elif not initialized:
        policy_level, policy_message = DiagnosticStatus.WARN, "Starting"
    elif inference_busy:
        policy_level, policy_message = DiagnosticStatus.OK, "Inference busy"
    else:
        policy_level, policy_message = DiagnosticStatus.OK, "Ready"
    if plan_age is not None:
        policy_message += f" · plan {plan_age:.2f} s old"
    diagnostic_array.status.append(
        _diagnostic_status(
            name="MemNav/Policy",
            hardware_id="jetson",
            level=policy_level,
            message=policy_message,
            values={
                "initialized": initialized,
                "inference_busy": inference_busy,
                "last_inference_s": payload.get("last_inference_s"),
                "plan_age_s": payload.get("plan_age_s"),
                "candidate_count": payload.get("candidate_count"),
                "backend": payload.get("backend"),
            },
        )
    )

    battery = payload.get("go2_battery")
    if not isinstance(battery, dict):
        battery = {}
    battery_soc = _number(battery.get("soc_pct"))
    if state["go2"] == "OFFLINE":
        go2_level = DiagnosticStatus.WARN
        go2_message = "Offline · battery unavailable"
    elif battery_soc is None:
        go2_level = DiagnosticStatus.WARN
        go2_message = "Online · battery unavailable"
    elif battery_soc is not None and battery_soc < 15.0:
        go2_level = DiagnosticStatus.WARN
        go2_message = f"Battery low · {battery_soc:.0f}%"
    else:
        go2_level = DiagnosticStatus.OK
        go2_message = f"Online · battery {battery_soc:.0f}%"
    diagnostic_array.status.append(
        _diagnostic_status(
            name="MemNav/Go2",
            hardware_id="unitree_go2",
            level=go2_level,
            message=go2_message,
            values={
                "connection": state["go2"],
                "safety": state["safety"],
                "soc_pct": battery_soc,
                "voltage_v": battery.get("voltage_v"),
                "current_a": battery.get("current_a"),
                "enabled": payload.get("enabled"),
                "estop": payload.get("estop"),
            },
        )
    )
    if arrival_payload is not None:
        diagnostic_array.status.append(build_arrival_diagnostic(arrival_payload))
    return diagnostic_array


def render_status_card(
    payload: dict[str, Any], width: int, height: int
) -> np.ndarray:
    """Render a terse, glanceable operator card for the compact dashboard."""

    if width < 480 or height < 220:
        raise ValueError("status card must be at least 480x220")
    canvas = np.full((height, width, 3), _CARD_BACKGROUND, dtype=np.uint8)
    pad = max(12, round(width * 0.02))

    enabled = bool(payload.get("enabled"))
    estop = bool(payload.get("estop"))
    arrived = bool(payload.get("arrival_latched"))
    phase_value = str(payload.get("phase") or "")
    survey_state = str(payload.get("survey_state") or "").upper()
    if not survey_state:
        if payload.get("stop_reason") == "survey_sealed":
            survey_state = "SEALED"
        elif phase_value == "memory_recording":
            survey_state = (
                "PAUSED" if payload.get("pause_memory_recording") else "ACTIVE"
            )
        else:
            survey_state = "INACTIVE"
    survey_visible = survey_state != "INACTIVE" or phase_value == "memory_recording"
    frames_recorded = int(payload.get("frames_recorded") or 0)
    survey_last_success = payload.get("survey_last_success")
    battery = payload.get("go2_battery")
    if not isinstance(battery, dict):
        battery = {}
    battery_online = battery.get("online") is True
    battery_soc = _number(battery.get("soc_pct"))
    battery_voltage = _number(battery.get("voltage_v"))
    if battery_online:
        battery_text = "--%" if battery_soc is None else f"{battery_soc:.0f}%"
        if battery_voltage is not None:
            battery_text += f"  {battery_voltage:.1f} V"
        if battery_soc is None:
            battery_color = _WARNING
        elif battery_soc < 20.0:
            battery_color = _DANGER
        elif battery_soc < 40.0:
            battery_color = _WARNING
        else:
            battery_color = _GOOD
    else:
        battery_text = "GO2 OFFLINE"
        battery_color = _DANGER
    if survey_visible:
        state = f"SURVEY {survey_state}"
        state_color = {
            "ACTIVE": _GOOD,
            "PAUSED": _WARNING,
            "SEALED": _GOOD,
        }.get(survey_state, _NEUTRAL)
        if survey_last_success is False:
            state_color = _DANGER
    elif arrived:
        state, state_color = "ARRIVED | LOCKED", _GOOD
    elif estop:
        state, state_color = "E-STOP | LOCKED", _DANGER
    elif enabled:
        state, state_color = "NAVIGATING", _GOOD
    else:
        state, state_color = "READY | DISABLED", _WARNING

    (state_width, _), _ = cv2.getTextSize(
        state, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2
    )
    badge_right = pad + state_width + 24
    cv2.rectangle(canvas, (pad, 10), (badge_right, 45), state_color, -1)
    cv2.putText(
        canvas,
        state,
        (pad + 12, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (18, 20, 24),
        2,
        cv2.LINE_AA,
    )
    (battery_width, _), _ = cv2.getTextSize(
        battery_text, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1
    )
    cv2.putText(
        canvas,
        battery_text,
        (width - pad - battery_width, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        battery_color,
        1,
        cv2.LINE_AA,
    )

    rgbd_age = _number(payload.get("rgbd_age_s"))
    plan_age = _number(payload.get("plan_age_s"))
    clearance = _number(payload.get("clearance_m"))
    cmd_vx = _number(payload.get("cmd_vx"))
    cmd_wz = _number(payload.get("cmd_wz"))
    if survey_visible:
        last_action = str(
            payload.get("survey_last_action")
            or payload.get("last_receipt_event")
            or "-"
        ).replace("survey_", "").replace("_", " ").upper()
        if survey_last_success is True:
            last_action = f"{last_action} OK"
        elif survey_last_success is False:
            last_action = f"{last_action} FAILED"
        metrics = (
            (
                "FRAMES",
                str(frames_recorded),
                _GOOD if frames_recorded > 0 else _WARNING,
            ),
            (
                "GOALS",
                str(int(payload.get("goal_candidates_captured") or 0)),
                _NEUTRAL,
            ),
            (
                "LAST",
                _fit_text(last_action, 15),
                (
                    _DANGER
                    if survey_last_success is False
                    else _GOOD if survey_last_success is True else _NEUTRAL
                ),
            ),
            ("MOTION", "LOCKED", _GOOD if estop and not enabled else _DANGER),
        )
    else:
        metrics = (
            (
                "RGB-D",
                "n/a" if rgbd_age is None else f"{rgbd_age:.2f} s",
                _metric_color(rgbd_age, good_max=0.25, danger_max=0.75),
            ),
            (
                "PLAN",
                "n/a" if plan_age is None else f"{plan_age:.2f} s",
                _metric_color(plan_age, good_max=1.5, danger_max=2.5),
            ),
            (
                "CLEAR",
                "n/a" if clearance is None else f"{clearance:.2f} m",
                _metric_color(clearance, danger_min=0.45, good_min=0.80),
            ),
            (
                "CMD  V / W",
                (
                    "n/a"
                    if cmd_vx is None or cmd_wz is None
                    else f"{cmd_vx:+.2f} / {cmd_wz:+.2f}"
                ),
                _GOOD if enabled and not estop else _NEUTRAL,
            ),
        )
    gap = 8
    card_top, card_bottom = 57, 137
    card_width = (width - 2 * pad - 3 * gap) // 4
    for index, (label, value, color) in enumerate(metrics):
        left = pad + index * (card_width + gap)
        right = width - pad if index == 3 else left + card_width
        cv2.rectangle(canvas, (left, card_top), (right, card_bottom), _CARD_SURFACE, -1)
        cv2.rectangle(canvas, (left, card_top), (left + 5, card_bottom), color, -1)
        cv2.putText(
            canvas,
            label,
            (left + 13, card_top + 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            _TEXT_SECONDARY,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            value,
            (left + 13, card_top + 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.63,
            _TEXT_PRIMARY,
            2,
            cv2.LINE_AA,
        )

    footer_top = card_bottom + 26
    if survey_visible:
        dataset = _fit_text(payload.get("survey_dataset_id"), 44)
        footer = f"{dataset}  |  {_fit_text(payload.get('last_receipt_event'), 22)}"
    else:
        phase = _fit_text(payload.get("phase"), 18)
        stop_reason = _fit_text(payload.get("stop_reason"), 28)
        goal = "GOAL OK" if payload.get("image_goal_loaded") else "GOAL MISSING"
        arrival = "ARRIVED" if arrived else "EN ROUTE"
        footer = f"{phase}  |  {goal}  |  {arrival}"
        if stop_reason != "-":
            footer += f"  |  {stop_reason}"
    cv2.putText(
        canvas,
        footer,
        (pad, footer_top),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        _TEXT_PRIMARY,
        1,
        cv2.LINE_AA,
    )
    if survey_visible:
        survey_message = payload.get("survey_last_message")
        if not survey_message and survey_state == "PAUSED":
            survey_message = "Paused | START SURVEY to resume"
        feedback = _fit_text(survey_message, 88)
        feedback_color = (
            _DANGER
            if survey_last_success is False
            else _GOOD if survey_state == "ACTIVE" else _WARNING
        )
    else:
        feedback = _fit_text(payload.get("last_error"), 84)
        feedback_color = _DANGER if feedback != "-" else _TEXT_SECONDARY
    if feedback != "-":
        cv2.putText(
            canvas,
            feedback,
            (pad, min(height - 13, footer_top + 31)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            feedback_color,
            1,
            cv2.LINE_AA,
        )
    return canvas


class FoxgloveImageRelay(Node):
    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__("navdp_foxglove_image_relay")
        self.options = options
        self._rgb_period = 1.0 / options.rgb_fps
        self._depth_period = 1.0 / options.depth_fps
        self._goal_period = 1.0 / options.goal_fps
        self._arrival_period = 1.0 / options.arrival_fps
        self._status_period = 1.0 / options.status_fps
        self._next_rgb = 0.0
        self._next_depth = 0.0
        self._next_goal = 0.0
        self._next_arrival = 0.0
        self._next_status = 0.0
        self._last_error_log = 0.0
        self._battery_payload: dict[str, Any] = {"online": False}
        self._last_status_payload: dict[str, Any] | None = None
        self._last_arrival_status_payload: dict[str, Any] | None = None
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._rgb_publisher = self.create_publisher(
            CompressedImage, options.rgb_output, sensor_qos
        )
        self._depth_publisher = self.create_publisher(
            CompressedImage, options.depth_output, sensor_qos
        )
        self._goal_publisher = self.create_publisher(
            CompressedImage, options.goal_output, state_qos
        )
        self._arrival_publisher = self.create_publisher(
            CompressedImage, options.arrival_output, state_qos
        )
        self._status_publisher = self.create_publisher(
            CompressedImage, options.status_output, state_qos
        )
        self._operator_publishers = {
            "workflow": self.create_publisher(
                String, options.operator_workflow_output, state_qos
            ),
            "mode": self.create_publisher(
                String, options.operator_mode_output, state_qos
            ),
            "activity": self.create_publisher(
                String, options.operator_activity_output, state_qos
            ),
            "safety": self.create_publisher(
                String, options.operator_safety_output, state_qos
            ),
            "go2": self.create_publisher(
                String, options.operator_go2_output, state_qos
            ),
        }
        self._arrival_state_publisher = self.create_publisher(
            String, options.operator_arrival_output, state_qos
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, options.operator_diagnostics_output, state_qos
        )
        self._arrival_diagnostics_publisher = self.create_publisher(
            DiagnosticArray, options.operator_arrival_diagnostics_output, state_qos
        )
        rgb_callbacks = MutuallyExclusiveCallbackGroup()
        depth_callbacks = MutuallyExclusiveCallbackGroup()
        goal_callbacks = MutuallyExclusiveCallbackGroup()
        arrival_callbacks = MutuallyExclusiveCallbackGroup()
        arrival_status_callbacks = MutuallyExclusiveCallbackGroup()
        status_callbacks = MutuallyExclusiveCallbackGroup()
        battery_callbacks = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image,
            options.rgb_input,
            self._on_rgb,
            sensor_qos,
            callback_group=rgb_callbacks,
        )
        self.create_subscription(
            Image,
            options.depth_input,
            self._on_depth,
            sensor_qos,
            callback_group=depth_callbacks,
        )
        self.create_subscription(
            Image,
            options.goal_input,
            self._on_goal,
            state_qos,
            callback_group=goal_callbacks,
        )
        self.create_subscription(
            Image,
            options.arrival_input,
            self._on_arrival,
            state_qos,
            callback_group=arrival_callbacks,
        )
        self.create_subscription(
            String,
            options.arrival_status_input,
            self._on_arrival_status,
            state_qos,
            callback_group=arrival_status_callbacks,
        )
        self.create_subscription(
            String,
            options.status_input,
            self._on_status,
            state_qos,
            callback_group=status_callbacks,
        )
        self.create_subscription(
            BatteryState,
            options.battery_input,
            self._on_battery,
            state_qos,
            callback_group=battery_callbacks,
        )
        self.get_logger().info(
            "Foxglove previews: RGB %dx%d@%.1f Hz q=%d -> %s; "
            "depth %dx%d@%.1f Hz q=%d range=%.0f..%.0f mm -> %s"
            % (
                options.width,
                options.height,
                options.rgb_fps,
                options.rgb_jpeg_quality,
                options.rgb_output,
                options.width,
                options.height,
                options.depth_fps,
                options.depth_jpeg_quality,
                options.depth_min_mm,
                options.depth_max_mm,
                options.depth_output,
            )
        )
        self.get_logger().info(
            "Goal %.1f Hz q=%d -> %s; arrival %.1f Hz q=%d native_aspect=%s -> %s; "
            "status %dx%d@%.1f Hz q=%d -> %s"
            % (
                options.goal_fps,
                options.goal_jpeg_quality,
                options.goal_output,
                options.arrival_fps,
                options.arrival_jpeg_quality,
                options.arrival_preserve_resolution,
                options.arrival_output,
                options.status_width,
                options.status_height,
                options.status_fps,
                options.status_jpeg_quality,
                options.status_output,
            )
        )
        self.get_logger().info(
            "Native operator state -> %s, %s, %s, %s, %s, %s; "
            "diagnostics -> %s, %s"
            % (
                options.operator_workflow_output,
                options.operator_mode_output,
                options.operator_activity_output,
                options.operator_safety_output,
                options.operator_go2_output,
                options.operator_arrival_output,
                options.operator_diagnostics_output,
                options.operator_arrival_diagnostics_output,
            )
        )

    def _report_error(self, stream: str, error: Exception) -> None:
        now = time.monotonic()
        if now - self._last_error_log >= 5.0:
            self.get_logger().error(f"Cannot process {stream}: {error}")
            self._last_error_log = now

    @staticmethod
    def _compressed(message: Image, data: bytes) -> CompressedImage:
        preview = CompressedImage()
        preview.header = message.header
        preview.format = "jpeg"
        preview.data = data
        return preview

    def _due(self, stream: str, now: float, period: float) -> bool:
        attribute = f"_next_{stream}"
        deadline = getattr(self, attribute)
        if deadline and now < deadline:
            return False
        if not deadline or now - deadline > period:
            setattr(self, attribute, now + period)
        else:
            # Advance from the previous deadline instead of resetting from the
            # current frame.  This avoids 30 Hz input quantizing a 15 Hz target
            # down to 10 Hz when the second source frame arrives just early.
            setattr(self, attribute, deadline + period)
        return True

    def _on_rgb(self, message: Image) -> None:
        now = time.monotonic()
        if not self._due("rgb", now, self._rgb_period):
            return
        try:
            image = rgb_message_to_bgr(message)
            image = resize_rgb_preview(
                image, self.options.width, self.options.height
            )
            data = encode_jpeg(image, self.options.rgb_jpeg_quality)
            self._rgb_publisher.publish(self._compressed(message, data))
        except (ValueError, RuntimeError, cv2.error) as error:
            self._report_error("RGB", error)

    def _publish_color_image(
        self,
        message: Image,
        stream: str,
        period: float,
        quality: int,
        publisher: Any,
        *,
        resize: bool,
    ) -> None:
        now = time.monotonic()
        if not self._due(stream, now, period):
            return
        try:
            image = prepare_color_preview(
                message,
                self.options.width,
                self.options.height,
                resize=resize,
            )
            publisher.publish(self._compressed(message, encode_jpeg(image, quality)))
        except (ValueError, RuntimeError, cv2.error) as error:
            self._report_error(stream, error)

    def _on_goal(self, message: Image) -> None:
        self._publish_color_image(
            message,
            "goal",
            self._goal_period,
            self.options.goal_jpeg_quality,
            self._goal_publisher,
            resize=True,
        )

    def _on_arrival(self, message: Image) -> None:
        self._publish_color_image(
            message,
            "arrival",
            self._arrival_period,
            self.options.arrival_jpeg_quality,
            self._arrival_publisher,
            resize=not self.options.arrival_preserve_resolution,
        )

    def _on_battery(self, message: BatteryState) -> None:
        self._battery_payload = battery_payload_from_message(message)
        if self._last_status_payload is not None:
            payload = dict(self._last_status_payload)
            payload["go2_battery"] = dict(self._battery_payload)
            self._publish_native_status(payload)

    def _publish_native_status(self, payload: dict[str, Any]) -> None:
        state = derive_operator_state(payload)
        for key, publisher in self._operator_publishers.items():
            message = String()
            message.data = state[key]
            publisher.publish(message)
        stamp = self.get_clock().now().to_msg()
        self._diagnostics_publisher.publish(
            build_operator_diagnostics(
                payload,
                arrival_payload=self._last_arrival_status_payload,
                stamp=stamp,
            )
        )

    def _on_arrival_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("arrival status payload is not a JSON object")
            self._last_arrival_status_payload = dict(payload)
            state_message = String()
            state_message.data = derive_arrival_state(payload)
            self._arrival_state_publisher.publish(state_message)
            stamp = self.get_clock().now().to_msg()
            self._arrival_diagnostics_publisher.publish(
                build_arrival_diagnostics(payload, stamp=stamp)
            )
            if self._last_status_payload is not None:
                status_payload = dict(self._last_status_payload)
                status_payload["go2_battery"] = dict(self._battery_payload)
                self._diagnostics_publisher.publish(
                    build_operator_diagnostics(
                        status_payload,
                        arrival_payload=payload,
                        stamp=stamp,
                    )
                )
        except (json.JSONDecodeError, ValueError) as error:
            self._report_error("arrival status", error)

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("status payload is not a JSON object")
            self._last_status_payload = dict(payload)
            payload["go2_battery"] = dict(self._battery_payload)
            self._publish_native_status(payload)
            now = time.monotonic()
            if not self._due("status", now, self._status_period):
                return
            image = render_status_card(
                payload, self.options.status_width, self.options.status_height
            )
            preview = CompressedImage()
            preview.header.stamp = self.get_clock().now().to_msg()
            preview.header.frame_id = "navdp_operator_status"
            preview.format = "jpeg"
            preview.data = encode_jpeg(image, self.options.status_jpeg_quality)
            self._status_publisher.publish(preview)
        except (json.JSONDecodeError, ValueError, RuntimeError, cv2.error) as error:
            self._report_error("status", error)

    def _on_depth(self, message: Image) -> None:
        now = time.monotonic()
        if not self._due("depth", now, self._depth_period):
            return
        try:
            depth = depth_message_to_u16(message)
            image = colorize_depth_preview(
                depth,
                self.options.width,
                self.options.height,
                self.options.depth_min_mm,
                self.options.depth_max_mm,
            )
            data = encode_jpeg(image, self.options.depth_jpeg_quality)
            self._depth_publisher.publish(self._compressed(message, data))
        except (ValueError, RuntimeError, cv2.error) as error:
            self._report_error("depth", error)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb-input", required=True)
    parser.add_argument("--depth-input", required=True)
    parser.add_argument("--rgb-output", required=True)
    parser.add_argument("--depth-output", required=True)
    parser.add_argument("--goal-input", required=True)
    parser.add_argument("--arrival-input", required=True)
    parser.add_argument(
        "--arrival-status-input", default="/navdp/rgb_arrival_status"
    )
    parser.add_argument("--status-input", required=True)
    parser.add_argument("--battery-input", required=True)
    parser.add_argument("--goal-output", required=True)
    parser.add_argument("--arrival-output", required=True)
    parser.add_argument("--status-output", required=True)
    parser.add_argument("--operator-mode-output", default="/navdp/operator/mode")
    parser.add_argument(
        "--operator-workflow-output", default="/navdp/operator/workflow"
    )
    parser.add_argument(
        "--operator-activity-output", default="/navdp/operator/activity"
    )
    parser.add_argument(
        "--operator-safety-output", default="/navdp/operator/safety"
    )
    parser.add_argument("--operator-go2-output", default="/navdp/operator/go2")
    parser.add_argument(
        "--operator-arrival-output", default="/navdp/operator/arrival"
    )
    parser.add_argument(
        "--operator-diagnostics-output", default="/navdp/operator/diagnostics"
    )
    parser.add_argument(
        "--operator-arrival-diagnostics-output",
        default="/navdp/operator/arrival_diagnostics",
    )
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--rgb-fps", type=float, required=True)
    parser.add_argument("--depth-fps", type=float, required=True)
    parser.add_argument("--goal-fps", type=float, required=True)
    parser.add_argument("--arrival-fps", type=float, required=True)
    parser.add_argument("--status-width", type=int, required=True)
    parser.add_argument("--status-height", type=int, required=True)
    parser.add_argument("--status-fps", type=float, required=True)
    parser.add_argument("--rgb-jpeg-quality", type=int, required=True)
    parser.add_argument("--depth-jpeg-quality", type=int, required=True)
    parser.add_argument("--goal-jpeg-quality", type=int, required=True)
    parser.add_argument("--arrival-jpeg-quality", type=int, required=True)
    parser.add_argument("--status-jpeg-quality", type=int, required=True)
    parser.add_argument("--arrival-preserve-resolution", action="store_true")
    parser.add_argument("--depth-min-mm", type=float, required=True)
    parser.add_argument("--depth-max-mm", type=float, required=True)
    options = parser.parse_args()
    if (
        options.width <= 0
        or options.height <= 0
        or options.status_width < 480
        or options.status_height < 220
    ):
        parser.error("preview width and height must be positive")
    if any(
        value <= 0
        for value in (
            options.rgb_fps,
            options.depth_fps,
            options.goal_fps,
            options.arrival_fps,
            options.status_fps,
        )
    ):
        parser.error("preview frame rates must be positive")
    for label in (
        "rgb_jpeg_quality",
        "depth_jpeg_quality",
        "goal_jpeg_quality",
        "arrival_jpeg_quality",
        "status_jpeg_quality",
    ):
        if not 1 <= getattr(options, label) <= 100:
            parser.error(f"{label.replace('_', '-')} must be in [1, 100]")
    if options.depth_min_mm < 0 or options.depth_max_mm <= options.depth_min_mm:
        parser.error("depth range must satisfy 0 <= min < max")
    return options


def main() -> None:
    options = parse_args()
    rclpy.init(args=[])
    node = FoxgloveImageRelay(options)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
