import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "deployment/go2"))

from revisit_operator_service import (  # noqa: E402
    ContractError,
    allowed_actions_for_state,
    capture_start_command,
    capture_finalize_command,
    capture_stop_command,
    episode_identity,
    freeze_goal_pair,
    navigation_command,
    prepare_command,
    survey_prepare_command,
    survey_stop_command,
    validate_realsense_link,
    validate_start_contract,
)


def write_contract(tmp_path: Path, *, mode: str = "prepared") -> tuple[Path, dict]:
    dataset_id = "route_01"
    goal = tmp_path / "goal.png"
    goal.write_bytes(b"frozen-image")
    goal_sha = hashlib.sha256(goal.read_bytes()).hexdigest()
    experiment = tmp_path / "experiment.json"
    experiment.write_text("{}\n", encoding="utf-8")
    manifest_sha = "a" * 64
    state = {
        "schema": "memnav_revisit_debug_state_v1",
        "dataset_id": dataset_id,
        "mode": mode,
        "goal_path": str(goal),
        "goal_sha256": goal_sha,
        "experiment_path": str(experiment),
        "dataset_manifest_sha256": manifest_sha if mode == "sealed" else None,
    }
    state_path = tmp_path / "active.json"
    state_path.write_text(json.dumps(state), encoding="utf-8")
    receipt = {
        "schema_version": "cec_realworld_episodic_dataset_v1_20260825",
        "dataset_id": dataset_id,
        "manifest_sha256": manifest_sha,
        "recording_active": False,
        "motion_enabled": False,
        "estop": True,
        "evaluation_depth_consumed_by_policy": False,
        "goal_memory_exact_sha_overlap": 0,
    }
    receipt_path = (
        tmp_path / "runtime/go2/two_pass_revisit" / dataset_id / "survey_seal.json"
    )
    receipt_path.parent.mkdir(parents=True)
    receipt_path.write_text(json.dumps(receipt), encoding="utf-8")
    return state_path, receipt


def test_validates_foxglove_stop_survey_contract(tmp_path):
    state_path, _ = write_contract(tmp_path)

    contract = validate_start_contract(tmp_path, state_path)

    assert contract.dataset_id == "route_01"
    assert contract.mode == "prepared"
    assert contract.dataset_manifest_sha256 == "a" * 64


def test_rejects_missing_stop_survey_receipt(tmp_path):
    state_path, _ = write_contract(tmp_path)
    receipt = tmp_path / "runtime/go2/two_pass_revisit/route_01/survey_seal.json"
    receipt.unlink()

    with pytest.raises(ContractError, match="Survey stop receipt is missing"):
        validate_start_contract(tmp_path, state_path)


def test_rejects_changed_frozen_goal(tmp_path):
    state_path, _ = write_contract(tmp_path)
    state = json.loads(state_path.read_text(encoding="utf-8"))
    Path(state["goal_path"]).write_bytes(b"changed")

    with pytest.raises(ContractError, match="goal SHA-256 changed"):
        validate_start_contract(tmp_path, state_path)


def test_rejects_receipt_without_fail_closed_motion_state(tmp_path):
    state_path, receipt = write_contract(tmp_path)
    receipt["estop"] = False
    receipt_path = tmp_path / "runtime/go2/two_pass_revisit/route_01/survey_seal.json"
    receipt_path.write_text(json.dumps(receipt), encoding="utf-8")

    with pytest.raises(ContractError, match=r"disabled \+ estop"):
        validate_start_contract(tmp_path, state_path)


def test_fixed_commands_do_not_accept_browser_arguments(tmp_path):
    assert prepare_command(tmp_path, "route_01_cec_utc") == [
        "bash",
        str(tmp_path / "deployment/go2/offboard/revisit_debug.sh"),
        "revisit-prepare",
        "--run-id",
        "route_01_cec_utc",
    ]
    assert navigation_command(tmp_path, tmp_path / "formal.json", 300) == [
        "bash",
        str(tmp_path / "deployment/go2/scripts/run_navigation.sh"),
        "--config",
        str(tmp_path / "formal.json"),
        "--timeout-s",
        "300",
    ]
    assert survey_prepare_command(tmp_path, "route_01", tmp_path / "goal.png") == [
        "bash",
        str(tmp_path / "deployment/go2/offboard/revisit_debug.sh"),
        "record-prepare",
        "route_01",
        "--goal",
        str(tmp_path / "goal.png"),
        "--point-label",
        "M",
    ]
    assert survey_stop_command(tmp_path)[-1] == "record-stop"
    assert capture_start_command(tmp_path, "episode_01", "route_01") == [
        "bash",
        str(tmp_path / "deployment/go2/offboard/experiment_capture.sh"),
        "start",
        "episode_01",
        "--dataset",
        "route_01",
        "--trial-kind",
        "revisit",
        "--profile",
        "full",
        "--allow-observer",
        "--onboard-episode",
    ]
    assert capture_stop_command(tmp_path, "episode_01")[-2:] == ["stop", "episode_01"]
    assert capture_finalize_command(
        tmp_path, "episode_01", "success", allow_incomplete=False
    )[-3:] == [
        "success",
        "--notes",
        "Foxglove-managed RGB-D Episode",
    ]
    assert capture_finalize_command(
        tmp_path, "episode_01", "failure", allow_incomplete=True
    )[-1] == "--allow-incomplete"


def _image_message(
    image: np.ndarray, *, encoding: str, stamp_ns: int, frame_id: str
) -> SimpleNamespace:
    return SimpleNamespace(
        width=image.shape[1],
        height=image.shape[0],
        step=image.strides[0],
        encoding=encoding,
        is_bigendian=False,
        data=image.tobytes(),
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id=frame_id,
        ),
    )


def test_freezes_lossless_rgbd_goal_with_exact_sensor_timestamps(tmp_path):
    rgb = np.array(
        [[[10, 20, 30], [40, 50, 60]], [[70, 80, 90], [100, 110, 120]]],
        dtype=np.uint8,
    )
    depth = np.array([[1000, 1200], [1400, 1600]], dtype=np.uint16)
    receipt = freeze_goal_pair(
        rgb_message=_image_message(
            rgb, encoding="rgb8", stamp_ns=1_234_000_000, frame_id="color"
        ),
        depth_message=_image_message(
            depth, encoding="16UC1", stamp_ns=1_235_000_000, frame_id="aligned"
        ),
        episode_dir=tmp_path,
        rgb_topic="/rgb",
        depth_topic="/depth",
        captured_utc="2026-09-03T10:00:00Z",
    )

    frozen_rgb = cv2.imread(str(tmp_path / "revisit_goal.png"), cv2.IMREAD_COLOR)
    frozen_depth = cv2.imread(
        str(tmp_path / "revisit_goal_depth.png"), cv2.IMREAD_UNCHANGED
    )
    assert np.array_equal(frozen_rgb, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    assert np.array_equal(frozen_depth, depth)
    assert receipt["rgb"]["stamp_ns"] == 1_234_000_000
    assert receipt["depth"]["stamp_ns"] == 1_235_000_000
    assert receipt["pair_delta_ms"] == pytest.approx(1.0)
    assert receipt["rgb"]["policy_goal_authority"] is True
    assert receipt["depth"]["policy_goal_authority"] is False


def test_episode_identity_and_gui_state_machine_are_deterministic():
    episode_id, dataset_id = episode_identity(
        datetime(2026, 9, 3, 10, 11, 12, 345678, tzinfo=timezone.utc)
    )
    assert episode_id == "episode_20260903T101112_345678Z"
    assert dataset_id == "m_episode_20260903T101112_345678Z"
    assert allowed_actions_for_state(None, busy=False) == ["capture-goal"]
    assert allowed_actions_for_state("goal_captured", busy=False) == [
        "start-survey",
        "stop-navigation",
    ]
    assert allowed_actions_for_state("surveying", busy=False) == [
        "stop-survey",
        "stop-navigation",
    ]
    assert allowed_actions_for_state("survey_sealed", busy=False) == [
        "revisit",
        "stop-navigation",
    ]
    assert allowed_actions_for_state("surveying", busy=True) == [
        "stop-navigation"
    ]


def test_realsense_preflight_uses_negotiated_usb_video_speed():
    validate_realsense_link(
        "Intel RealSense D435I 344422071135",
        "|__ Port 1: Dev 3, If 0, Class=Video, Driver=uvcvideo, 5000M",
    )
    with pytest.raises(ContractError, match="USB SuperSpeed"):
        validate_realsense_link(
            "Intel RealSense D435I 344422071135",
            "|__ Port 1: Dev 3, If 0, Class=Video, Driver=uvcvideo, 480M",
        )


def test_supervised_runner_selects_the_fullmono_tmux_session():
    source = (
        REPO / "deployment/go2/scripts/run_navigation.sh"
    ).read_text(encoding="utf-8")

    assert "fullmono-lingbot-cec)" in source
    assert 'session="$CFG_FULLMONO_SESSION"' in source
    agent_source = (
        REPO / "deployment/go2/navigation_run_agent.py"
    ).read_text(encoding="utf-8")
    assert "signal.signal(signal.SIGTERM, interrupt_on_sigterm)" in agent_source
