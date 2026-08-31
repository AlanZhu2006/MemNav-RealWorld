#!/usr/bin/env python3
"""Recover a stopped one-way external-goal Survey into a sealed dataset.

This is deliberately narrower than the live dataset writer.  It exists for a
stopped engineering Survey whose only captured goal candidate is an invalid
exact-JPEG duplicate of causal memory.  The source staging tree and its
operator-provided backup are never modified.  A new sealed tree is built with
all verified memory frames, no Survey goal candidates, and explicit provenance
requiring an externally frozen goal during Revisit preparation.

An explicit engineering-only short-route mode also accepts an empty goals/
directory below the configured generic frame floor.  It records the rejected
floor, actual frame count and formal-ineligibility in the immutable manifest;
the source staging tree and its exact backup remain untouched.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
from typing import Any, Mapping

from deployment.gpu.episodic_dataset import (
    EXTERNAL_GOAL_CONTRACT,
    ONE_WAY_EXTERNAL_GOAL_MODE,
    SCHEMA_VERSION,
    EpisodicDatasetStore,
    validate_dataset_id,
)


_MEMORY_NAME = re.compile(r"(?P<index>[0-9]{6})_(?P<prefix>[0-9a-f]{16})\.jpg\Z")
_DIGEST = re.compile(r"[0-9a-f]{64}\Z")


class RecoveryError(RuntimeError):
    """The stopped staging tree does not satisfy the recovery contract."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_json(payload: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
        + "\n"
    ).encode("utf-8")


def _inventory(root: Path) -> dict[str, tuple[int, str]]:
    result: dict[str, tuple[int, str]] = {}
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise RecoveryError(f"recovery trees may not contain symlinks: {path}")
        if path.is_dir():
            continue
        if not path.is_file():
            raise RecoveryError(f"unsupported recovery artifact: {path}")
        payload = path.read_bytes()
        result[path.relative_to(root).as_posix()] = (len(payload), _sha256(payload))
    return result


def _write_exact(path: Path, payload: bytes) -> None:
    if not payload:
        raise RecoveryError(f"refusing to write an empty artifact: {path}")
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_bytes(payload)
    temporary.replace(path)


def _validate_utc(value: str) -> str:
    try:
        datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise RecoveryError(f"invalid created UTC timestamp: {value}") from error
    return value


def recover_one_way_debug_dataset(
    *,
    root: Path,
    dataset_id: str,
    backup_root: Path,
    expected_memory_frames: int,
    created_utc: str,
    external_goal_sha256: str,
    external_goal_point: str = "M",
    goal_min_frame_gap: int = 16,
    goal_min_inliers: int = 16,
    goal_max_cos: float = 0.9,
    short_route_engineering_override: bool = False,
    configured_minimum_frames: int | None = None,
) -> dict[str, Any]:
    dataset_id = validate_dataset_id(dataset_id)
    root = root.expanduser().resolve()
    source = root / f".{dataset_id}.staging"
    final = root / dataset_id
    recovered = root / f".{dataset_id}.recovered"
    backup_root = backup_root.expanduser().resolve()

    if expected_memory_frames < 1:
        raise RecoveryError("expected memory frame count must be positive")
    if short_route_engineering_override:
        if configured_minimum_frames is None:
            raise RecoveryError(
                "short-route override requires configured_minimum_frames"
            )
        if int(configured_minimum_frames) <= expected_memory_frames:
            raise RecoveryError(
                "short-route override requires an actual frame count below "
                "the configured minimum"
            )
    if not _DIGEST.fullmatch(external_goal_sha256):
        raise RecoveryError("external goal SHA-256 must be lowercase hexadecimal")
    if not source.is_dir():
        raise RecoveryError(f"source staging tree does not exist: {source}")
    if not backup_root.is_dir():
        raise RecoveryError(f"backup tree does not exist: {backup_root}")
    if source == backup_root:
        raise RecoveryError("backup tree must be distinct from source staging")
    if final.exists() or recovered.exists():
        raise RecoveryError(
            f"refusing overwrite; final or recovery path already exists: {dataset_id}"
        )

    source_inventory = _inventory(source)
    backup_inventory = _inventory(backup_root)
    if source_inventory != backup_inventory:
        raise RecoveryError("backup tree is not an exact byte-for-byte source copy")

    memory_dir = source / "memory"
    goals_dir = source / "goals"
    if not memory_dir.is_dir() or not goals_dir.is_dir():
        raise RecoveryError("source staging must contain memory/ and goals/")

    memory_records: list[dict[str, Any]] = []
    memory_hashes: dict[str, dict[str, Any]] = {}
    memory_paths = sorted(path for path in memory_dir.iterdir() if path.is_file())
    if len(memory_paths) != expected_memory_frames:
        raise RecoveryError(
            f"memory frame count changed: got {len(memory_paths)}, "
            f"expected {expected_memory_frames}"
        )
    if any(path.is_dir() or path.is_symlink() for path in memory_dir.iterdir()):
        raise RecoveryError("memory/ may contain only regular JPEG files")
    for expected_index, path in enumerate(memory_paths):
        match = _MEMORY_NAME.fullmatch(path.name)
        if match is None or int(match.group("index")) != expected_index:
            raise RecoveryError(f"non-contiguous or invalid memory name: {path.name}")
        payload = path.read_bytes()
        if not payload:
            raise RecoveryError(f"empty memory frame: {path.name}")
        digest = _sha256(payload)
        if not digest.startswith(match.group("prefix")):
            raise RecoveryError(f"memory filename SHA prefix changed: {path.name}")
        record = {
            "frame_index": expected_index,
            "path": f"memory/{path.name}",
            "sha256": digest,
            "bytes": len(payload),
        }
        memory_records.append(record)
        memory_hashes[digest] = record

    goal_paths = sorted(path for path in goals_dir.iterdir() if path.is_file())
    if any(path.is_dir() or path.is_symlink() for path in goals_dir.iterdir()):
        raise RecoveryError("goals/ may contain only regular files")
    goal_jpegs = [path for path in goal_paths if path.suffix.lower() in {".jpg", ".jpeg"}]
    invalid_goal_sha: str | None = None
    overlap_record: dict[str, Any] | None = None
    if short_route_engineering_override:
        if goal_paths:
            raise RecoveryError(
                "short-route external-goal override requires an empty goals directory"
            )
    else:
        if len(goal_jpegs) != 1:
            raise RecoveryError(
                "recovery requires exactly one invalid captured candidate JPEG"
            )
        invalid_goal = goal_jpegs[0]
        invalid_goal_payload = invalid_goal.read_bytes()
        invalid_goal_sha = _sha256(invalid_goal_payload)
        overlap_record = memory_hashes.get(invalid_goal_sha)
        if overlap_record is None:
            raise RecoveryError(
                "captured candidate is not the expected exact-JPEG memory duplicate"
            )

    recovered.mkdir(parents=False)
    (recovered / "memory").mkdir()
    (recovered / "goals").mkdir()
    discarded_root = recovered / "recovery_discarded" / "goals"
    if goal_paths:
        discarded_root.mkdir(parents=True)
    for path in memory_paths:
        shutil.copy2(path, recovered / "memory" / path.name)
    discarded_records: list[dict[str, Any]] = []
    for path in goal_paths:
        payload = path.read_bytes()
        destination = discarded_root / path.name
        shutil.copy2(path, destination)
        discarded_records.append({
            "original_path": f"goals/{path.name}",
            "preserved_path": f"recovery_discarded/goals/{path.name}",
            "sha256": _sha256(payload),
            "bytes": len(payload),
        })

    sealed_utc = _utc_now()
    recovery_metadata: dict[str, Any] = {
        "schema": "memnav_one_way_debug_recovery_v2",
        "recovered_utc": sealed_utc,
        "source_staging_root": str(source),
        "verified_backup_root": str(backup_root),
        "discarded_goal_artifacts": discarded_records,
    }
    if short_route_engineering_override:
        recovery_metadata.update({
            "reason": (
                "operator-approved engineering short route ended before the "
                "generic frame floor; external frozen M is mandatory"
            ),
            "short_route_engineering_override": True,
            "configured_minimum_frames": int(configured_minimum_frames),
            "accepted_memory_frames": len(memory_records),
            "formal_eligible": False,
            "engineering_unregistered_required": True,
        })
    else:
        assert overlap_record is not None
        recovery_metadata.update({
            "reason": (
                "unvalidated captured candidate exactly duplicated causal memory; "
                "preserved for audit and excluded from the manifest"
            ),
            "short_route_engineering_override": False,
            "overlapping_memory_frame": dict(overlap_record),
        })
    metadata = {
        "dataset_id": dataset_id,
        "collection_mode": ONE_WAY_EXTERNAL_GOAL_MODE,
        "robot": "unitree_go2",
        "motion_authority": "unitree_hand_controller_only",
        "adapter_enabled": False,
        "candidate_contract": "external_frozen_goal_only_no_survey_candidate",
        "goal_selection_contract": EXTERNAL_GOAL_CONTRACT,
        "goal_candidates_required": False,
        "formal_eligible": False,
        "engineering_unregistered_required": True,
        "external_goal": {
            "point_label": external_goal_point,
            "sha256": external_goal_sha256,
            "source": "operator_frozen_before_survey",
        },
        "recovery": recovery_metadata,
    }
    protocol = {
        "cec_protocol_version": 3,
        "navigation_sensor_contract": "causal_monocular_rgb_v1",
        "goal_min_frame_gap": int(goal_min_frame_gap),
        "goal_min_inliers": int(goal_min_inliers),
        "goal_max_cos": float(goal_max_cos),
        "metric_depth_sensor_consumed_by_policy": False,
        "engineering_short_route_override": bool(
            short_route_engineering_override
        ),
    }
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "dataset_id": dataset_id,
        "created_utc": _validate_utc(created_utc),
        "sealed_utc": sealed_utc,
        "metadata": metadata,
        "protocol": protocol,
        "summary": {
            "memory_frames": len(memory_records),
            "goal_candidates": 0,
            "goal_memory_exact_sha_overlap": 0,
            "evaluation_depth_consumed_by_policy": False,
        },
        "memory_frames": memory_records,
        "goal_candidates": [],
    }
    raw = _canonical_json(manifest)
    manifest_sha = _sha256(raw)
    _write_exact(recovered / "manifest.json", raw)
    _write_exact(
        recovered / "MANIFEST.sha256",
        f"{manifest_sha}  manifest.json\n".encode("ascii"),
    )
    _write_exact(
        recovered / "SEALED",
        f"{SCHEMA_VERSION}\n{manifest_sha}\n".encode("ascii"),
    )

    # Verify every copied policy artifact through the production loader before
    # publishing the recovered directory under its immutable final name.
    verification_store = EpisodicDatasetStore(root, minimum_frames=1)
    recovered.replace(final)
    try:
        loaded = verification_store.load(dataset_id)
    except Exception:
        final.replace(recovered)
        raise
    if len(list(loaded.memory_frames())) != expected_memory_frames:
        final.replace(recovered)
        raise RecoveryError("production loader returned an incomplete memory stream")

    return {
        "schema_version": SCHEMA_VERSION,
        "dataset_id": dataset_id,
        "dataset_root": str(final),
        "source_staging_preserved": str(source),
        "backup_root": str(backup_root),
        "manifest_sha256": manifest_sha,
        "memory_frames": len(memory_records),
        "goal_candidates": 0,
        "discarded_candidate_sha256": invalid_goal_sha,
        "overlapping_memory_frame_index": (
            None if overlap_record is None else overlap_record["frame_index"]
        ),
        "external_goal_sha256": external_goal_sha256,
        "short_route_engineering_override": bool(
            short_route_engineering_override
        ),
        "configured_minimum_frames": (
            int(configured_minimum_frames)
            if short_route_engineering_override
            else None
        ),
        "formal_eligible": False,
        "artifacts_verified": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--backup-root", required=True, type=Path)
    parser.add_argument("--expected-memory-frames", required=True, type=int)
    parser.add_argument("--created-utc", required=True)
    parser.add_argument("--external-goal-sha256", required=True)
    parser.add_argument("--external-goal-point", default="M")
    parser.add_argument("--short-route-engineering-override", action="store_true")
    parser.add_argument("--configured-minimum-frames", type=int)
    args = parser.parse_args()
    receipt = recover_one_way_debug_dataset(
        root=args.root,
        dataset_id=args.dataset_id,
        backup_root=args.backup_root,
        expected_memory_frames=args.expected_memory_frames,
        created_utc=args.created_utc,
        external_goal_sha256=args.external_goal_sha256,
        external_goal_point=args.external_goal_point,
        short_route_engineering_override=args.short_route_engineering_override,
        configured_minimum_frames=args.configured_minimum_frames,
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
