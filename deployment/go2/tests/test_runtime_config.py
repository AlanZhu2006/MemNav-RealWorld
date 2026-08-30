import base64
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "deployment"))

import runtime_config as rc  # noqa: E402


PNG_1X1 = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUB"
    "AScY42YAAAAASUVORK5CYII="
)


def make_source_config(tmp_path: Path) -> Path:
    system = json.loads(
        (REPO / "deployment/config/system.json").read_text(encoding="utf-8")
    )
    system_path = tmp_path / "system.json"
    system_path.write_text(json.dumps(system), encoding="utf-8")
    goal = tmp_path / "goal.png"
    goal.write_bytes(PNG_1X1)
    experiment = json.loads(
        (REPO / "deployment/config/experiments/fullmono_imagegoal.json").read_text(
            encoding="utf-8"
        )
    )
    experiment["system_config"] = str(system_path)
    experiment["experiment"]["navigation"]["image_goal"] = str(goal)
    experiment["experiment"]["arrival"]["image_goal"] = str(goal)
    experiment_path = tmp_path / "experiment.json"
    experiment_path.write_text(json.dumps(experiment), encoding="utf-8")
    return experiment_path


def test_resolve_records_image_and_stable_hash(tmp_path):
    experiment = make_source_config(tmp_path)
    first = rc.resolve(experiment, tmp_path / "first.json")
    second = rc.resolve(experiment, tmp_path / "second.json")
    one = rc.load_resolved(first)
    two = rc.load_resolved(second)
    assert one["config_id"] == two["config_id"]
    assert one["navigation"]["image_goal"]["width"] == 1
    assert one["navigation"]["image_goal"]["height"] == 1
    assert one["navigation"]["image_goal"]["sha256"] == rc._sha256_file(
        tmp_path / "goal.png"
    )


def test_modified_resolved_config_is_rejected(tmp_path):
    path = rc.resolve(make_source_config(tmp_path), tmp_path / "resolved.json")
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["control"]["max_linear_mps"] = 99
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(rc.ConfigError, match="config_id mismatch"):
        rc.load_resolved(path)


def test_unknown_experiment_field_is_rejected(tmp_path):
    experiment = make_source_config(tmp_path)
    payload = json.loads(experiment.read_text(encoding="utf-8"))
    payload["experiment"]["forgotten_override"] = True
    experiment.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(rc.ConfigError, match="unknown experiment field"):
        rc.resolve(experiment, tmp_path / "resolved.json")


def test_missing_system_field_is_rejected(tmp_path):
    experiment = make_source_config(tmp_path)
    experiment_payload = json.loads(experiment.read_text(encoding="utf-8"))
    system_path = Path(experiment_payload["system_config"])
    system = json.loads(system_path.read_text(encoding="utf-8"))
    del system["sites"]["gpu"]["models"]["navdp_checkpoint"]
    system_path.write_text(json.dumps(system), encoding="utf-8")
    with pytest.raises(rc.ConfigError, match="missing GPU models field"):
        rc.resolve(experiment, tmp_path / "resolved.json")


def test_survey_and_formal_are_derived_from_one_contract(tmp_path):
    base = rc.resolve(make_source_config(tmp_path), tmp_path / "base.json")
    survey = rc._derive(
        base, tmp_path / "survey.json", "survey", "route_01", None
    )
    formal_root = tmp_path / "formal"
    goal = tmp_path / "goal.png"
    goal_sha256 = rc._sha256_file(goal)
    formal = rc._derive(
        base,
        tmp_path / "formal.json",
        "formal",
        "route_01",
        formal_root,
        scene_id="scene01",
        run_id="scene01_pair01_cec",
        authority_mode="cec",
        frozen_goal=goal,
        expected_goal_sha256=goal_sha256,
        expected_dataset_sha256="a" * 64,
    )
    survey_payload = rc.load_resolved(survey)
    formal_payload = rc.load_resolved(formal)
    assert survey_payload["dataset"]["auto_open"] is True
    assert survey_payload["launch"]["go2_bridge"] is False
    assert formal_payload["dataset"]["auto_open"] is False
    assert formal_payload["launch"]["go2_bridge"] is True
    assert formal_payload["cec"]["authority_mode"] == "cec"
    assert formal_payload["formal"]["expected_goal_sha256"] == goal_sha256
    assert formal_payload["navigation"]["selected_goal_image_path"] == str(
        formal_root / "selected_goal.jpg"
    )
    assert survey_payload["source"]["derived_from_config_id"] == rc.load_resolved(
        base
    )["config_id"]


def test_shell_contract_has_explicit_imagegoal_and_no_legacy_names(tmp_path):
    path = rc.resolve(make_source_config(tmp_path), tmp_path / "resolved.json")
    exports = rc.shell_exports(rc.load_resolved(path), "jetson")
    assert "CFG_IMAGE_GOAL=" in exports
    assert "CFG_IMAGE_GOAL_SHA256=" in exports
    assert "NAVDP_IMAGE_GOAL_PATH" not in exports
    assert "CEC_CAMERA_HEIGHT_M" not in exports


def test_runtime_config_uses_the_public_profile_catalog():
    assert set(rc.PROFILES) == {
        "native-navdp-rgbd",
        "fullmono-lingbot-cec",
    }
    assert set(rc.ARRIVAL_MODULES) == {
        "operator",
        "external-topic",
        "rgb-homography",
    }


def test_gpu_shell_contract_exposes_only_gpu_and_shared_fields(tmp_path):
    path = rc.resolve(make_source_config(tmp_path), tmp_path / "resolved.json")
    exports = rc.shell_exports(rc.load_resolved(path), "gpu")
    assert "CFG_GPU_PYTHON=" in exports
    assert "CFG_MEMNAV_CKPT=" in exports
    assert "CFG_AUTHORITY_MODE=" in exports
    assert "CFG_JETSON_PYTHON=" not in exports
    assert "CFG_UNITREE_NET_IF=" not in exports
    assert "CFG_IMAGE_GOAL=" not in exports
