from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

from freeze_realworld_paired_campaign import (  # noqa: E402
    CampaignFreezeError,
    freeze_campaign,
)
from verify_realworld_paired_campaign import _validate_plan  # noqa: E402
from verify_realworld_formal_registration import (  # noqa: E402
    RegistrationError,
    verify_registration,
)


def _write(path: Path, payload: bytes) -> dict[str, str]:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    return {"path": str(path), "sha256": hashlib.sha256(payload).hexdigest()}


def _template(path: Path) -> Path:
    payload = json.loads(
        (ROOT / "manifests" / "realworld_paired_evaluation_plan_v2.json")
        .read_text(encoding="utf-8")
    )
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _registry(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    scenes = []
    for index in range(1, 5):
        scene_id = f"scene{index:02d}"
        scene_root = root / scene_id
        scenes.append({
            "scene_id": scene_id,
            "role": "Novel" if index <= 2 else "Revisit",
            "target": f"target-{index}",
            "navigation_setting": f"setting-{index}",
            "dataset_id": f"survey-{index}",
            "dataset_manifest": _write(scene_root / "dataset.json", b"dataset" + bytes([index])),
            "goal_image": _write(scene_root / "goal.jpg", b"goal" + bytes([index])),
            "start_pose_receipt": _write(scene_root / "start.json", b"start" + bytes([index])),
            "shortest_path_receipt": _write(scene_root / "path.json", b"path" + bytes([index])),
            "method_config": _write(scene_root / "method.json", b"config" + bytes([index])),
            "shortest_feasible_path_m": 2.0 + index,
            "start_region": "tape polygon v1",
            "start_yaw_deg": 15.0 * index,
            "start_yaw_tolerance_deg": 5.0,
            "goal_region": "Odin polygon v1",
            "time_budget_s": 180.0,
            "path_budget_m": 30.0,
            "collision_abort_rule": "operator estop counts as failure",
        })
    payload = {
        "schema": "memnav-realworld-scene-registry-v1",
        "campaign_id": "cec-four-scene-five-paired-block-v2",
        "formal_outcomes_read": False,
        "scenes": scenes,
    }
    path = root / "registry.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _calibration(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    evidence = _write(root / "calibration_evidence.json", b"heldout calibration")
    payload = {
        "schema": "memnav-realworld-arrival-calibration-v1",
        "status": "passed",
        "formal_outcomes_read": False,
        "held_out_from_formal_scenes": True,
        "calibration_scene_ids": ["calibration-a", "calibration-b"],
        "calibration_trials": 12,
        "termination_authority": "independent_rgb_homography_evaluator",
        "success_region_contract": "frozen Odin goal polygon",
        "stationary_hold_s": 1.0,
        "relocalization_stability_contract": "three stable Odin updates",
        "path_measurement_contract": "consecutive Odin odometry increments",
        "dropout_policy": "failure after frozen grace interval",
        "evidence": [evidence],
    }
    path = root / "arrival_calibration.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_freeze_closes_all_preformal_blockers_without_outcomes(tmp_path: Path):
    template = _template(tmp_path / "template.json")
    registry = _registry(tmp_path / "registry")
    calibration = _calibration(tmp_path / "calibration")

    frozen = freeze_campaign(template, registry, calibration)
    errors, blockers, runs = _validate_plan(frozen)

    assert errors == []
    assert blockers == []
    assert len(runs) == 40
    assert frozen["claims"]["completed_rollouts"] == 0
    assert frozen["claims"]["formal_realworld_sr_spl"] is False
    assert frozen["freeze_receipts"]["formal_outcomes_read"] is False
    assert [scene["role"] for scene in frozen["scenes"]] == [
        "Novel",
        "Novel",
        "Revisit",
        "Revisit",
    ]


def test_freeze_rejects_tampered_artifact(tmp_path: Path):
    template = _template(tmp_path / "template.json")
    registry = _registry(tmp_path / "registry")
    payload = json.loads(registry.read_text(encoding="utf-8"))
    Path(payload["scenes"][0]["goal_image"]["path"]).write_bytes(b"tampered")

    with pytest.raises(CampaignFreezeError, match="artifact SHA mismatch"):
        freeze_campaign(template, registry, _calibration(tmp_path / "calibration"))


def test_freeze_rejects_formal_scene_used_for_calibration(tmp_path: Path):
    template = _template(tmp_path / "template.json")
    registry = _registry(tmp_path / "registry")
    calibration = _calibration(tmp_path / "calibration")
    payload = json.loads(calibration.read_text(encoding="utf-8"))
    payload["calibration_scene_ids"].append("scene03")
    calibration.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(CampaignFreezeError, match="scene overlap"):
        freeze_campaign(template, registry, calibration)


def test_freeze_never_accepts_a_template_with_outcomes(tmp_path: Path):
    template = _template(tmp_path / "template.json")
    payload = json.loads(template.read_text(encoding="utf-8"))
    payload["scenes"][0]["paired_blocks"][0]["runs"]["mono_cec"]["success"] = True
    template.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(CampaignFreezeError, match="structural validation"):
        freeze_campaign(
            template,
            _registry(tmp_path / "registry"),
            _calibration(tmp_path / "calibration"),
        )


def test_launch_registration_binds_exact_arm_without_exposing_role(tmp_path: Path):
    frozen = freeze_campaign(
        _template(tmp_path / "template.json"),
        _registry(tmp_path / "registry"),
        _calibration(tmp_path / "calibration"),
    )
    plan = tmp_path / "frozen.json"
    plan.write_text(json.dumps(frozen), encoding="utf-8")
    scene = frozen["scenes"][2]

    receipt = verify_registration(
        plan,
        scene_id="scene03",
        run_id="scene03_pair02_cec",
        arm="mono_cec",
        dataset_id=scene["dataset_id"],
        goal_sha256=scene["goal_sha256"],
        dataset_manifest_sha256=scene["dataset_manifest_sha256"],
    )

    assert receipt["registered"] is True
    assert receipt["runtime_role_visibility"] == "none"
    assert "role" not in receipt
    assert receipt["pair_index"] == 2
    assert receipt["arm_order"] == ["mono_cec", "mono_native"]


def test_launch_registration_rejects_wrong_arm(tmp_path: Path):
    frozen = freeze_campaign(
        _template(tmp_path / "template.json"),
        _registry(tmp_path / "registry"),
        _calibration(tmp_path / "calibration"),
    )
    plan = tmp_path / "frozen.json"
    plan.write_text(json.dumps(frozen), encoding="utf-8")
    scene = frozen["scenes"][2]

    with pytest.raises(RegistrationError, match="differs from frozen plan"):
        verify_registration(
            plan,
            scene_id="scene03",
            run_id="scene03_pair02_cec",
            arm="mono_native",
            dataset_id=scene["dataset_id"],
            goal_sha256=scene["goal_sha256"],
            dataset_manifest_sha256=scene["dataset_manifest_sha256"],
        )
