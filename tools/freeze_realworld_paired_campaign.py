#!/usr/bin/env python3
"""Freeze the outcome-blank 20-pair real-robot campaign from field receipts.

This command is deliberately pre-experiment and motion-free.  It verifies the
exact bytes named by a four-scene registry and an independently collected
arrival-calibration receipt, copies only their immutable bindings into a new
campaign plan, and refuses to overwrite any existing file.  It never reads a
formal navigation outcome and never edits the preregistration template.
"""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

from verify_realworld_paired_campaign import (
    CampaignVerificationError,
    PLAN_SCHEMA,
    _validate_plan,
)


REGISTRY_SCHEMA = "memnav-realworld-scene-registry-v1"
CALIBRATION_SCHEMA = "memnav-realworld-arrival-calibration-v1"
FREEZE_SCHEMA = "memnav-realworld-paired-freeze-v1"
ROLES = ("Novel", "Revisit")


class CampaignFreezeError(RuntimeError):
    """A source receipt cannot support a frozen formal campaign."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _read_object(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CampaignFreezeError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(payload, dict):
        raise CampaignFreezeError(f"JSON root must be an object: {path}")
    return payload


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(payload: Mapping[str, Any]) -> str:
    encoded = (
        json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
        + "\n"
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _is_sha256(value: Any) -> bool:
    if not isinstance(value, str) or len(value) != 64:
        return False
    try:
        int(value, 16)
    except ValueError:
        return False
    return value == value.lower()


def _required_string(payload: Mapping[str, Any], key: str, context: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value.strip():
        raise CampaignFreezeError(f"{context}: {key} must be a non-empty string")
    return value.strip()


def _finite_number(
    payload: Mapping[str, Any],
    key: str,
    context: str,
    *,
    positive: bool = False,
) -> float:
    value = payload.get(key)
    if isinstance(value, bool):
        raise CampaignFreezeError(f"{context}: {key} must be numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise CampaignFreezeError(f"{context}: {key} must be numeric") from error
    if not math.isfinite(number) or (positive and number <= 0.0):
        qualifier = "finite and positive" if positive else "finite"
        raise CampaignFreezeError(f"{context}: {key} must be {qualifier}")
    return number


def _artifact_receipt(
    value: Any,
    *,
    owner: Path,
    context: str,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise CampaignFreezeError(f"{context}: expected {{path, sha256}} receipt")
    raw_path = value.get("path")
    expected = value.get("sha256")
    if not isinstance(raw_path, str) or not raw_path:
        raise CampaignFreezeError(f"{context}: artifact path is missing")
    if not _is_sha256(expected):
        raise CampaignFreezeError(f"{context}: artifact SHA-256 is invalid")
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        path = owner.parent / path
    path = path.resolve()
    if not path.is_file() or path.stat().st_size <= 0:
        raise CampaignFreezeError(f"{context}: artifact missing or empty: {path}")
    actual = _sha256_file(path)
    if actual != expected:
        raise CampaignFreezeError(
            f"{context}: artifact SHA mismatch: expected {expected}, got {actual}"
        )
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": actual}


def _validate_calibration(
    payload: Mapping[str, Any],
    *,
    path: Path,
    formal_scene_ids: set[str],
) -> dict[str, Any]:
    context = "arrival calibration"
    if payload.get("schema") != CALIBRATION_SCHEMA:
        raise CampaignFreezeError(f"{context}: unsupported schema")
    if payload.get("status") != "passed":
        raise CampaignFreezeError(f"{context}: status must be passed")
    if payload.get("formal_outcomes_read") is not False:
        raise CampaignFreezeError(f"{context}: formal_outcomes_read must be false")
    if payload.get("held_out_from_formal_scenes") is not True:
        raise CampaignFreezeError(
            f"{context}: held_out_from_formal_scenes must be true"
        )
    calibration_scenes = payload.get("calibration_scene_ids")
    if not isinstance(calibration_scenes, list) or not calibration_scenes:
        raise CampaignFreezeError(f"{context}: calibration_scene_ids are required")
    calibration_scene_set = {str(value) for value in calibration_scenes}
    overlap = formal_scene_ids & calibration_scene_set
    if overlap:
        raise CampaignFreezeError(
            f"{context}: formal/calibration scene overlap: {sorted(overlap)}"
        )
    trials = payload.get("calibration_trials")
    if isinstance(trials, bool) or not isinstance(trials, int) or trials <= 0:
        raise CampaignFreezeError(f"{context}: calibration_trials must be positive")
    _required_string(payload, "termination_authority", context)
    _required_string(payload, "success_region_contract", context)
    _finite_number(payload, "stationary_hold_s", context, positive=True)
    _required_string(payload, "relocalization_stability_contract", context)
    _required_string(payload, "path_measurement_contract", context)
    _required_string(payload, "dropout_policy", context)
    evidence = payload.get("evidence")
    if not isinstance(evidence, list) or not evidence:
        raise CampaignFreezeError(f"{context}: evidence receipts are required")
    verified_evidence = [
        _artifact_receipt(
            receipt,
            owner=path,
            context=f"{context} evidence[{index}]",
        )
        for index, receipt in enumerate(evidence)
    ]
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": _sha256_file(path),
        "canonical_payload_sha256": _canonical_sha256(payload),
        "calibration_scene_ids": sorted(calibration_scene_set),
        "calibration_trials": trials,
        "termination_authority": str(payload["termination_authority"]),
        "evidence": verified_evidence,
    }


def _validate_registry_scene(
    scene: Mapping[str, Any],
    *,
    registry_path: Path,
) -> dict[str, Any]:
    scene_id = _required_string(scene, "scene_id", "scene registry")
    context = f"scene registry {scene_id}"
    role = scene.get("role")
    if role not in ROLES:
        raise CampaignFreezeError(f"{context}: role must be Novel or Revisit")
    target = _required_string(scene, "target", context)
    navigation_setting = _required_string(scene, "navigation_setting", context)
    dataset_id = _required_string(scene, "dataset_id", context)
    start_region = _required_string(scene, "start_region", context)
    goal_region = _required_string(scene, "goal_region", context)
    collision_abort_rule = _required_string(scene, "collision_abort_rule", context)
    dataset = _artifact_receipt(
        scene.get("dataset_manifest"), owner=registry_path, context=f"{context} dataset"
    )
    goal = _artifact_receipt(
        scene.get("goal_image"), owner=registry_path, context=f"{context} goal"
    )
    start = _artifact_receipt(
        scene.get("start_pose_receipt"),
        owner=registry_path,
        context=f"{context} start pose",
    )
    shortest_receipt = _artifact_receipt(
        scene.get("shortest_path_receipt"),
        owner=registry_path,
        context=f"{context} shortest path",
    )
    method_config = _artifact_receipt(
        scene.get("method_config"),
        owner=registry_path,
        context=f"{context} method config",
    )
    shortest = _finite_number(
        scene, "shortest_feasible_path_m", context, positive=True
    )
    start_yaw = _finite_number(scene, "start_yaw_deg", context)
    yaw_tolerance = _finite_number(
        scene, "start_yaw_tolerance_deg", context, positive=True
    )
    time_budget = _finite_number(scene, "time_budget_s", context, positive=True)
    path_budget = _finite_number(scene, "path_budget_m", context, positive=True)
    if path_budget + 1e-9 < shortest:
        raise CampaignFreezeError(
            f"{context}: path_budget_m is shorter than the frozen shortest path"
        )
    return {
        "scene_id": scene_id,
        "role": role,
        "target": target,
        "navigation_setting": navigation_setting,
        "dataset_id": dataset_id,
        "dataset_manifest_sha256": dataset["sha256"],
        "goal_sha256": goal["sha256"],
        "start_pose_receipt_sha256": start["sha256"],
        "shortest_feasible_path_m": shortest,
        "start_region": start_region,
        "start_yaw_deg": start_yaw,
        "start_yaw_tolerance_deg": yaw_tolerance,
        "goal_region": goal_region,
        "time_budget_s": time_budget,
        "path_budget_m": path_budget,
        "collision_abort_rule": collision_abort_rule,
        "artifact_receipts": {
            "dataset_manifest": dataset,
            "goal_image": goal,
            "start_pose": start,
            "shortest_path": shortest_receipt,
            "method_config": method_config,
        },
    }


def freeze_campaign(
    template_path: Path,
    registry_path: Path,
    calibration_path: Path,
) -> dict[str, Any]:
    template_path = template_path.expanduser().resolve()
    registry_path = registry_path.expanduser().resolve()
    calibration_path = calibration_path.expanduser().resolve()
    template = _read_object(template_path)
    registry = _read_object(registry_path)
    calibration = _read_object(calibration_path)
    if template.get("schema_version") != PLAN_SCHEMA:
        raise CampaignFreezeError("template has an unsupported campaign schema")
    if template.get("status") != "planned_no_formal_runs":
        raise CampaignFreezeError("template is not an outcome-blank preregistration")
    if registry.get("schema") != REGISTRY_SCHEMA:
        raise CampaignFreezeError("scene registry has an unsupported schema")
    if registry.get("campaign_id") != template.get("campaign_id"):
        raise CampaignFreezeError("scene registry campaign_id differs from template")
    if registry.get("formal_outcomes_read") is not False:
        raise CampaignFreezeError("scene registry formal_outcomes_read must be false")
    source_scenes = registry.get("scenes")
    if not isinstance(source_scenes, list) or len(source_scenes) != 4:
        raise CampaignFreezeError("scene registry must contain exactly four scenes")
    frozen_scenes = [
        _validate_registry_scene(scene, registry_path=registry_path)
        for scene in source_scenes
    ]
    frozen_by_id = {scene["scene_id"]: scene for scene in frozen_scenes}
    expected_ids = {str(scene.get("scene_id")) for scene in template.get("scenes", [])}
    if len(frozen_by_id) != 4 or set(frozen_by_id) != expected_ids:
        raise CampaignFreezeError(
            "scene registry IDs must match the four preregistered scene IDs exactly"
        )
    roles = Counter(str(scene["role"]) for scene in frozen_scenes)
    if roles != Counter({"Novel": 2, "Revisit": 2}):
        raise CampaignFreezeError(
            f"scene registry must freeze 2 Novel and 2 Revisit scenes, got {dict(roles)}"
        )
    calibration_receipt = _validate_calibration(
        calibration,
        path=calibration_path,
        formal_scene_ids=set(frozen_by_id),
    )

    frozen = deepcopy(template)
    for scene in frozen["scenes"]:
        values = frozen_by_id[str(scene["scene_id"])]
        blocks = scene["paired_blocks"]
        scene.clear()
        scene.update(values)
        scene["paired_blocks"] = blocks
    frozen["freeze_receipts"] = {
        "schema": FREEZE_SCHEMA,
        "created_utc": _utc_now(),
        "formal_outcomes_read": False,
        "template": {
            "path": str(template_path),
            "bytes": template_path.stat().st_size,
            "sha256": _sha256_file(template_path),
            "canonical_payload_sha256": _canonical_sha256(template),
        },
        "scene_registry": {
            "path": str(registry_path),
            "bytes": registry_path.stat().st_size,
            "sha256": _sha256_file(registry_path),
            "canonical_payload_sha256": _canonical_sha256(registry),
        },
        "arrival_calibration": calibration_receipt,
    }
    frozen["execution_contract"]["independent_arrival_calibration_verified"] = True
    frozen["execution_contract"]["arrival_termination_authority"] = (
        calibration_receipt["termination_authority"]
    )
    frozen["metric_contract"]["success_threshold_status"] = (
        "frozen_from_heldout_arrival_calibration"
    )
    frozen["metric_contract"]["shortest_path_source"] = (
        "frozen_odin_occupancy_astar_before_formal_01"
    )
    errors, blockers, runs = _validate_plan(frozen)
    if errors or blockers or len(runs) != 40:
        raise CampaignFreezeError(
            "frozen plan does not pass independent structural validation: "
            + json.dumps(
                {"errors": errors, "blockers": blockers, "registered_runs": len(runs)},
                sort_keys=True,
            )
        )
    return frozen


def _write_new(path: Path, payload: Mapping[str, Any]) -> dict[str, Any]:
    path = path.expanduser().resolve()
    if path.exists() or path.with_name(path.name + ".sha256").exists():
        raise CampaignFreezeError(f"refusing to overwrite frozen output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    with tempfile.NamedTemporaryFile(
        dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as handle:
        handle.write(encoded)
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    os.replace(temporary, path)
    digest = hashlib.sha256(encoded).hexdigest()
    sidecar = path.with_name(path.name + ".sha256")
    sidecar.write_text(f"{digest}  {path.name}\n", encoding="utf-8")
    return {"path": str(path), "bytes": len(encoded), "sha256": digest}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--template",
        type=Path,
        required=True,
        help="Outcome-blank paired campaign template.",
    )
    parser.add_argument("--scene-registry", type=Path, required=True)
    parser.add_argument("--arrival-calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        frozen = freeze_campaign(
            args.template, args.scene_registry, args.arrival_calibration
        )
        receipt = _write_new(args.output, frozen)
    except (CampaignFreezeError, CampaignVerificationError, OSError) as error:
        parser.error(str(error))
    print(json.dumps({"frozen_plan": receipt}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
