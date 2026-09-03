import hashlib
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "deployment/go2"))

from revisit_operator_service import (  # noqa: E402
    ContractError,
    navigation_command,
    prepare_command,
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
