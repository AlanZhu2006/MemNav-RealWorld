#!/usr/bin/env python3
"""Read-only verifier and aggregator for the paired real-robot campaign.

The registered plan remains an outcome-blank, pre-experiment artifact.  This
tool never edits it.  It binds finalized capture directories to the registered
run IDs, independently rechecks every hash seal and Odin SPL receipt, verifies
the explicit native/CEC authority mode, and only then derives aggregate SR,
SPL and paired McNemar statistics.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Iterable, Mapping


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment" / "go2"))

from experiment_capture_manifest import (  # noqa: E402
    CaptureManifestError,
    read_manifest,
    sha256_file,
    verify_manifest,
)


PLAN_SCHEMA = "memnav-realworld-paired-evaluation-plan-v2"
METHODS = ("mono_native", "mono_cec")
ROLES = ("Novel", "Revisit")
SUCCESS_OUTCOME = "success"


class CampaignVerificationError(RuntimeError):
    """One or more campaign artifacts violate the frozen contract."""


def _read_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CampaignVerificationError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(payload, dict):
        raise CampaignVerificationError(f"expected a JSON object: {path}")
    return payload


def _canonical_sha256(payload: Mapping[str, Any]) -> str:
    encoded = (
        json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
        + "\n"
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    with tempfile.NamedTemporaryFile(
        dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as handle:
        handle.write(encoded)
        temporary = Path(handle.name)
    os.replace(temporary, path)


def _is_sha256(value: Any) -> bool:
    if not isinstance(value, str) or len(value) != 64:
        return False
    try:
        int(value, 16)
    except ValueError:
        return False
    return True


def _iter_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    if not path.is_file():
        return
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as error:
            raise CampaignVerificationError(
                f"invalid JSONL at {path}:{line_number}: {error}"
            ) from error
        if isinstance(payload, dict):
            yield payload


def _walk_objects(value: Any) -> Iterable[Mapping[str, Any]]:
    if isinstance(value, Mapping):
        yield value
        for child in value.values():
            yield from _walk_objects(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk_objects(child)


def _exact_mcnemar(gains: int, losses: int) -> float:
    discordant = int(gains) + int(losses)
    if discordant == 0:
        return 1.0
    tail = min(int(gains), int(losses))
    probability = sum(math.comb(discordant, k) for k in range(tail + 1))
    return min(1.0, 2.0 * probability / (2**discordant))


def _validate_plan(plan: Mapping[str, Any]) -> tuple[list[str], list[str], list[dict[str, Any]]]:
    errors: list[str] = []
    blockers: list[str] = []
    registered: list[dict[str, Any]] = []
    if plan.get("schema_version") != PLAN_SCHEMA:
        errors.append(f"unexpected plan schema: {plan.get('schema_version')!r}")
    if plan.get("status") != "planned_no_formal_runs":
        errors.append("the immutable plan must retain status=planned_no_formal_runs")
    method_ids = [row.get("id") for row in plan.get("methods", [])]
    if method_ids != list(METHODS):
        errors.append(f"method order must be exactly {list(METHODS)}")
    expected_shape = {
        "scenes": 4,
        "paired_blocks_per_scene": 5,
        "paired_blocks": 20,
        "rollouts_per_block": 2,
        "total_rollouts": 40,
        "native_first_blocks": 10,
        "cec_first_blocks": 10,
    }
    if plan.get("expected_shape") != expected_shape:
        errors.append("expected_shape differs from the registered 4x5 paired campaign")

    scenes = plan.get("scenes", [])
    if not isinstance(scenes, list) or len(scenes) != 4:
        errors.append("the plan must contain exactly four scenes")
        scenes = []
    seen_runs: set[str] = set()
    first_arms: Counter[str] = Counter()
    roles: Counter[str] = Counter()
    for scene in scenes:
        scene_id = str(scene.get("scene_id", ""))
        role = scene.get("role")
        if role in ROLES:
            roles[str(role)] += 1
        elif role is None:
            blockers.append(f"{scene_id}: role is not frozen")
        else:
            errors.append(f"{scene_id}: invalid role {role!r}")
        registry_fields = {
            "target": scene.get("target"),
            "navigation_setting": scene.get("navigation_setting"),
            "dataset_id": scene.get("dataset_id"),
            "dataset_manifest_sha256": scene.get("dataset_manifest_sha256"),
            "goal_sha256": scene.get("goal_sha256"),
            "start_pose_receipt_sha256": scene.get("start_pose_receipt_sha256"),
            "shortest_feasible_path_m": scene.get("shortest_feasible_path_m"),
        }
        for field, value in registry_fields.items():
            if value is None:
                blockers.append(f"{scene_id}: {field} is not frozen")
        for field in (
            "dataset_manifest_sha256",
            "goal_sha256",
            "start_pose_receipt_sha256",
        ):
            value = registry_fields[field]
            if value is not None and not _is_sha256(value):
                errors.append(f"{scene_id}: {field} is not a SHA-256")
        shortest = registry_fields["shortest_feasible_path_m"]
        if shortest is not None and (
            isinstance(shortest, bool)
            or not math.isfinite(float(shortest))
            or float(shortest) <= 0.0
        ):
            errors.append(f"{scene_id}: shortest_feasible_path_m must be positive")

        blocks = scene.get("paired_blocks", [])
        if not isinstance(blocks, list) or len(blocks) != 5:
            errors.append(f"{scene_id}: expected five paired blocks")
            continue
        for expected_index, block in enumerate(blocks, 1):
            if block.get("pair_index") != expected_index:
                errors.append(f"{scene_id}: non-contiguous pair_index")
            order = block.get("order")
            if not isinstance(order, list) or set(order) != set(METHODS) or len(order) != 2:
                errors.append(f"{scene_id} pair {expected_index}: invalid arm order")
            else:
                first_arms[str(order[0])] += 1
            runs = block.get("runs", {})
            if not isinstance(runs, Mapping) or set(runs) != set(METHODS):
                errors.append(f"{scene_id} pair {expected_index}: missing registered arms")
                continue
            for arm in METHODS:
                run = runs[arm]
                run_id = str(run.get("run_id", ""))
                if not run_id or run_id in seen_runs:
                    errors.append(f"duplicate or empty run_id: {run_id!r}")
                seen_runs.add(run_id)
                # The preregistration stays outcome blank forever.  Completed
                # evidence is derived into a separate verifier output.
                for field in (
                    "success",
                    "actual_path_m",
                    "spl",
                    "evidence_manifest_sha256",
                ):
                    if run.get(field) is not None:
                        errors.append(
                            f"{run_id}: preregistered field {field} was modified"
                        )
                if run.get("status") != "planned":
                    errors.append(f"{run_id}: immutable plan status must remain planned")
                registered.append({
                    "scene_id": scene_id,
                    "role": role,
                    "scene": scene,
                    "pair_index": expected_index,
                    "order": list(order) if isinstance(order, list) else [],
                    "arm": arm,
                    "run_id": run_id,
                })
    if first_arms and first_arms != Counter({"mono_native": 10, "mono_cec": 10}):
        errors.append(f"arm-order balance is not 10/10: {dict(first_arms)}")
    if roles and roles != Counter({"Novel": 2, "Revisit": 2}):
        errors.append(f"frozen roles are not 2 Novel / 2 Revisit: {dict(roles)}")
    return errors, sorted(set(blockers)), registered


def _authority_audit(run_root: Path, arm: str) -> tuple[bool, bool, list[str]]:
    expected_mode = "native" if arm == "mono_native" else "cec"
    modes: list[str] = []
    takeover = False
    errors: list[str] = []
    for row in _iter_jsonl(run_root / "logs" / "cec_receipt.jsonl"):
        for obj in _walk_objects(row):
            if "cec_authority_mode" in obj:
                modes.append(str(obj.get("cec_authority_mode")))
                takeover = takeover or obj.get("cec_takeover") is True
    if not modes:
        errors.append("no cec_authority_mode receipt")
    elif set(modes) != {expected_mode}:
        errors.append(
            f"authority receipt mismatch: expected {expected_mode}, observed {sorted(set(modes))}"
        )
    if arm == "mono_native" and takeover:
        errors.append("native arm contains a CEC takeover")
    return bool(modes), takeover, errors


def _status_bindings(run_root: Path) -> tuple[set[str], set[str], set[str]]:
    goals: set[str] = set()
    datasets: set[str] = set()
    dataset_hashes: set[str] = set()
    for row in _iter_jsonl(run_root / "logs" / "status.jsonl"):
        for obj in _walk_objects(row):
            goal = obj.get("active_goal_sha256")
            if _is_sha256(goal):
                goals.add(str(goal))
            dataset = obj.get("loaded_dataset_id")
            if isinstance(dataset, str) and dataset:
                datasets.add(dataset)
            digest = obj.get("loaded_dataset_manifest_sha256")
            if _is_sha256(digest):
                dataset_hashes.add(str(digest))
    return goals, datasets, dataset_hashes


def _verify_run(entry: Mapping[str, Any], run_root: Path) -> dict[str, Any]:
    arm = str(entry["arm"])
    scene = entry["scene"]
    run_id = str(entry["run_id"])
    verification = verify_manifest(run_root)
    manifest = read_manifest(run_root)
    errors: list[str] = []
    if verification.get("run_id") != run_id:
        errors.append("capture manifest run_id mismatch")
    if manifest.get("dataset_id") != scene.get("dataset_id"):
        errors.append("capture dataset_id differs from the frozen scene registry")
    expected_trial_kind = str(entry["role"]).lower()
    if manifest.get("trial_kind") != expected_trial_kind:
        errors.append(
            f"trial_kind mismatch: expected {expected_trial_kind}, got {manifest.get('trial_kind')}"
        )
    if manifest.get("gt_source") != "odin1":
        errors.append("formal run lacks the registered Odin independent evaluator")
    if manifest.get("repository", {}).get("tracked_files_dirty") is not False:
        errors.append("formal run used a dirty or unverifiable tracked checkout")
    if manifest.get("capture_stop_clean") is not True:
        errors.append("capture did not stop cleanly")
    if manifest.get("completeness", {}).get("formal_complete") is not True:
        errors.append("capture evidence is incomplete")

    receipt_path = run_root / "receipts" / "odin_spl_receipt.json"
    receipt = _read_json(receipt_path)
    if receipt.get("schema") != "memnav-odin1-spl-receipt-v1":
        errors.append("unexpected Odin SPL receipt schema")
    if receipt.get("run_id") != run_id:
        errors.append("Odin SPL receipt run_id mismatch")
    metrics = receipt.get("metrics", {})
    try:
        success = int(metrics["S_i"])
        shortest = float(metrics["L_i_m"])
        actual = float(metrics["P_i_m"])
        spl = float(metrics["SPL_i"])
    except (KeyError, TypeError, ValueError) as error:
        raise CampaignVerificationError(
            f"{run_id}: invalid Odin metrics: {error}"
        ) from error
    if success not in (0, 1):
        errors.append("S_i is not binary")
    expected_spl = success * shortest / max(shortest, actual, 1e-9)
    if not all(math.isfinite(value) for value in (shortest, actual, spl)):
        errors.append("non-finite L_i/P_i/SPL_i")
    elif abs(spl - expected_spl) > 1.1e-6:
        errors.append("SPL_i does not reproduce from S_i/L_i/P_i")
    frozen_shortest = float(scene["shortest_feasible_path_m"])
    if abs(shortest - frozen_shortest) > 1e-4:
        errors.append("L_i differs from the frozen scene shortest path")
    outcome_success = manifest.get("outcome") == SUCCESS_OUTCOME
    if outcome_success != bool(success):
        errors.append("capture outcome and independent S_i disagree")

    authority_seen, takeover, authority_errors = _authority_audit(run_root, arm)
    errors.extend(authority_errors)
    goals, datasets, dataset_hashes = _status_bindings(run_root)
    if scene.get("goal_sha256") not in goals:
        errors.append("status receipts do not bind the frozen goal SHA")
    if scene.get("dataset_id") not in datasets:
        errors.append("status receipts do not bind the frozen dataset ID")
    if scene.get("dataset_manifest_sha256") not in dataset_hashes:
        errors.append("status receipts do not bind the frozen dataset manifest SHA")

    if errors:
        raise CampaignVerificationError(f"{run_id}: " + "; ".join(errors))
    return {
        "run_id": run_id,
        "scene_id": entry["scene_id"],
        "role": entry["role"],
        "pair_index": entry["pair_index"],
        "arm": arm,
        "order": entry["order"],
        "success": success,
        "L_i_m": shortest,
        "P_i_m": actual,
        "SPL_i": spl,
        "cec_takeover": takeover,
        "authority_receipt_seen": authority_seen,
        "capture_manifest_sha256": verification["manifest_sha256"],
        "capture_artifacts": verification["artifacts"],
        "odin_spl_receipt_sha256": sha256_file(receipt_path),
    }


def _aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_arm: dict[str, Any] = {}
    for arm in METHODS:
        arm_rows = [row for row in rows if row["arm"] == arm]
        by_role = {}
        for role in ROLES:
            subset = [row for row in arm_rows if row["role"] == role]
            by_role[role] = {
                "success": sum(row["success"] for row in subset),
                "n": len(subset),
                "sr": (
                    None if not subset else sum(row["success"] for row in subset) / len(subset)
                ),
            }
        by_arm[arm] = {
            "success": sum(row["success"] for row in arm_rows),
            "n": len(arm_rows),
            "sr": (
                None if not arm_rows else sum(row["success"] for row in arm_rows) / len(arm_rows)
            ),
            "mean_spl": (
                None if not arm_rows else sum(row["SPL_i"] for row in arm_rows) / len(arm_rows)
            ),
            "mean_path_m": (
                None if not arm_rows else sum(row["P_i_m"] for row in arm_rows) / len(arm_rows)
            ),
            "by_role": by_role,
        }

    indexed = {
        (row["scene_id"], row["pair_index"], row["arm"]): row for row in rows
    }
    gains = losses = ties_success = ties_failure = 0
    for scene_id, pair_index in sorted(
        {(row["scene_id"], row["pair_index"]) for row in rows}
    ):
        native = indexed.get((scene_id, pair_index, "mono_native"))
        cec = indexed.get((scene_id, pair_index, "mono_cec"))
        if native is None or cec is None:
            continue
        outcome = (native["success"], cec["success"])
        if outcome == (0, 1):
            gains += 1
        elif outcome == (1, 0):
            losses += 1
        elif outcome == (1, 1):
            ties_success += 1
        else:
            ties_failure += 1
    return {
        "arms": by_arm,
        "paired": {
            "complete_pairs": gains + losses + ties_success + ties_failure,
            "gains": gains,
            "losses": losses,
            "both_success": ties_success,
            "both_failure": ties_failure,
            "two_sided_exact_mcnemar_p": _exact_mcnemar(gains, losses),
        },
        "novel_cec_takeover_runs": sum(
            row["arm"] == "mono_cec"
            and row["role"] == "Novel"
            and row["cec_takeover"]
            for row in rows
        ),
    }


def verify_campaign(
    plan_path: Path,
    *,
    evidence_root: Path | None = None,
    require_complete: bool = False,
) -> dict[str, Any]:
    plan_path = plan_path.expanduser().resolve()
    plan = _read_json(plan_path)
    plan_errors, blockers, registered = _validate_plan(plan)
    evidence_rows: list[dict[str, Any]] = []
    evidence_errors: list[str] = []
    missing_runs: list[str] = []
    if evidence_root is not None:
        evidence_root = evidence_root.expanduser().resolve()
        for entry in registered:
            run_root = evidence_root / entry["run_id"]
            if not run_root.is_dir():
                missing_runs.append(str(entry["run_id"]))
                continue
            if blockers:
                evidence_errors.append(
                    f"{entry['run_id']}: scene registry is not frozen"
                )
                continue
            try:
                evidence_rows.append(_verify_run(entry, run_root))
            except (CampaignVerificationError, CaptureManifestError, OSError) as error:
                evidence_errors.append(str(error))
    complete = bool(
        not plan_errors
        and not blockers
        and not evidence_errors
        and len(evidence_rows) == 40
        and not missing_runs
    )
    result: dict[str, Any] = {
        "schema": "memnav-realworld-paired-verification-v1",
        "plan": {
            "path": str(plan_path),
            "sha256": sha256_file(plan_path),
            "canonical_payload_sha256": _canonical_sha256(plan),
            "campaign_id": plan.get("campaign_id"),
            "registered_runs": len(registered),
            "structural_errors": plan_errors,
            "freeze_blockers": blockers,
        },
        "evidence": {
            "root": None if evidence_root is None else str(evidence_root),
            "verified_runs": len(evidence_rows),
            "missing_runs": missing_runs,
            "errors": evidence_errors,
        },
        "complete": complete,
        "claim_boundary": (
            "formal_paired_result_verified"
            if complete
            else "incomplete_no_formal_sr_spl_claim"
        ),
        "rows": evidence_rows,
        "aggregate": _aggregate(evidence_rows) if evidence_rows else None,
    }
    if require_complete and not complete:
        raise CampaignVerificationError(json.dumps(result, sort_keys=True))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan",
        type=Path,
        default=ROOT / "manifests" / "realworld_paired_evaluation_plan_v2.json",
    )
    parser.add_argument("--evidence-root", type=Path)
    parser.add_argument("--require-complete", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        result = verify_campaign(
            args.plan,
            evidence_root=args.evidence_root,
            require_complete=args.require_complete,
        )
    except CampaignVerificationError as error:
        parser.error(str(error))
    if args.output is not None:
        _atomic_json(args.output, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
