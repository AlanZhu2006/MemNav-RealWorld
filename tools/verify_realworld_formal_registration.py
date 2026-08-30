#!/usr/bin/env python3
"""Verify one requested real-world arm against a fully frozen campaign plan.

The output intentionally omits the hidden Novel/Revisit role.  It is a launch
binding receipt, not a navigation result and not a runtime classifier.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from verify_realworld_paired_campaign import _validate_plan


class RegistrationError(RuntimeError):
    """The requested launch is absent from, or disagrees with, the plan."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_registration(
    plan_path: Path,
    *,
    scene_id: str,
    run_id: str,
    arm: str,
    dataset_id: str,
    goal_sha256: str,
    dataset_manifest_sha256: str,
) -> dict[str, Any]:
    plan_path = plan_path.expanduser().resolve()
    try:
        plan = json.loads(plan_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RegistrationError(f"cannot read frozen plan {plan_path}: {error}") from error
    if not isinstance(plan, dict):
        raise RegistrationError("frozen plan JSON root must be an object")
    errors, blockers, registered = _validate_plan(plan)
    if errors or blockers:
        raise RegistrationError(
            "campaign is not fully frozen: "
            + json.dumps({"errors": errors, "blockers": blockers}, sort_keys=True)
        )
    matches = [entry for entry in registered if entry["run_id"] == run_id]
    if len(matches) != 1:
        raise RegistrationError(f"run_id is not uniquely registered: {run_id}")
    entry = matches[0]
    scene = entry["scene"]
    checks = {
        "scene_id": (str(entry["scene_id"]), scene_id),
        "arm": (str(entry["arm"]), arm),
        "dataset_id": (str(scene["dataset_id"]), dataset_id),
        "goal_sha256": (str(scene["goal_sha256"]), goal_sha256),
        "dataset_manifest_sha256": (
            str(scene["dataset_manifest_sha256"]),
            dataset_manifest_sha256,
        ),
    }
    mismatches = {
        key: {"registered": expected, "requested": observed}
        for key, (expected, observed) in checks.items()
        if expected != observed
    }
    if mismatches:
        raise RegistrationError(
            "formal launch differs from frozen plan: "
            + json.dumps(mismatches, sort_keys=True)
        )
    method_config = scene.get("artifact_receipts", {}).get("method_config", {})
    method_sha = method_config.get("sha256")
    if not isinstance(method_sha, str) or len(method_sha) != 64:
        raise RegistrationError("frozen scene lacks a method-config SHA-256")
    return {
        "schema": "memnav-realworld-formal-registration-v1",
        "registered": True,
        "formal_outcomes_read": False,
        "runtime_role_visibility": "none",
        "plan_path": str(plan_path),
        "plan_sha256": _sha256(plan_path),
        "campaign_id": plan.get("campaign_id"),
        "scene_id": scene_id,
        "run_id": run_id,
        "arm": arm,
        "dataset_id": dataset_id,
        "goal_sha256": goal_sha256,
        "dataset_manifest_sha256": dataset_manifest_sha256,
        "method_config_sha256": method_sha,
        "pair_index": int(entry["pair_index"]),
        "arm_order": list(entry["order"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--arm", choices=("mono_native", "mono_cec"), required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--goal-sha256", required=True)
    parser.add_argument("--dataset-manifest-sha256", required=True)
    args = parser.parse_args()
    try:
        result = verify_registration(
            args.plan,
            scene_id=args.scene_id,
            run_id=args.run_id,
            arm=args.arm,
            dataset_id=args.dataset_id,
            goal_sha256=args.goal_sha256,
            dataset_manifest_sha256=args.dataset_manifest_sha256,
        )
    except RegistrationError as error:
        parser.error(str(error))
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
