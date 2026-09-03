#!/usr/bin/env python3
"""Create and verify immutable evidence manifests for real-world trials."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
from typing import Any, Iterable


SCHEMA_VERSION = "memnav_realworld_capture_v3_20260831"
LEGACY_SCHEMA_VERSIONS = {
    "memnav_realworld_capture_v1_20260827",
    "memnav_realworld_capture_v2_20260830",
}
RUN_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")
OUTCOMES = (
    "success",
    "failure",
    "timeout",
    "operator_intervention",
    "system_failure",
    "collision",
    "aborted",
)
TRIAL_KINDS = ("revisit", "novel", "calibration", "debug")
VIDEO_SUFFIXES = {".mp4", ".mov", ".mkv"}
VIDEO_ROLES = {
    "foxglove_dashboard": "dashboard",
    "third_view": "third_view",
}
REFERENCE_ROLES = {
    "odin_gt_result": "odin_gt_result.json",
    "odin_spl_receipt": "odin_spl_receipt.json",
}


class CaptureManifestError(RuntimeError):
    """The requested operation violates the capture evidence contract."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def validate_run_id(run_id: str) -> str:
    value = str(run_id).strip()
    if not RUN_ID_PATTERN.fullmatch(value):
        raise CaptureManifestError(
            "run_id must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}"
        )
    return value


def calibration_label(
    *,
    trial_kind: str,
    scene_id: str | None,
    physical_distance_m: float | None,
    physical_yaw_deg: float | None,
    measurement_method: str | None,
    recorded_utc: str,
) -> dict[str, Any] | None:
    """Freeze independent physical labels before a calibration capture starts."""

    supplied = (
        scene_id is not None,
        physical_distance_m is not None,
        physical_yaw_deg is not None,
        measurement_method is not None,
    )
    if trial_kind != "calibration":
        if any(supplied):
            raise CaptureManifestError(
                "physical calibration labels are only valid for calibration trials"
            )
        return None
    if not all(supplied):
        raise CaptureManifestError(
            "calibration trials require scene ID, physical distance, physical "
            "yaw, and measurement method before capture"
        )
    assert scene_id is not None
    assert physical_distance_m is not None
    assert physical_yaw_deg is not None
    assert measurement_method is not None
    scene = validate_run_id(scene_id)
    method = str(measurement_method).strip()
    if not method:
        raise CaptureManifestError("calibration measurement method is empty")
    if isinstance(physical_distance_m, bool) or not math.isfinite(
        float(physical_distance_m)
    ) or float(physical_distance_m) < 0.0:
        raise CaptureManifestError(
            "calibration physical distance must be finite and non-negative"
        )
    if isinstance(physical_yaw_deg, bool) or not math.isfinite(
        float(physical_yaw_deg)
    ) or abs(float(physical_yaw_deg)) > 180.0:
        raise CaptureManifestError(
            "calibration physical yaw must be finite and within [-180, 180]"
        )
    return {
        "scene_id": scene,
        "physical_distance_m": float(physical_distance_m),
        "physical_yaw_deg": float(physical_yaw_deg),
        "measurement_method": method,
        "label_recorded_utc": recorded_utc,
        "recorded_before_capture": True,
        "arrival_score_logging_started_before_label": False,
    }


def repository_receipt(workspace: Path) -> dict[str, Any]:
    def git(*arguments: str) -> str:
        result = subprocess.run(
            ["git", *arguments],
            cwd=workspace,
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()

    try:
        commit = git("rev-parse", "HEAD")
        branch = git("rev-parse", "--abbrev-ref", "HEAD")
        dirty = bool(git("status", "--porcelain", "--untracked-files=no"))
    except (OSError, subprocess.CalledProcessError):
        commit = None
        branch = None
        dirty = None
    return {
        "workspace": str(workspace),
        "commit": commit,
        "branch": branch,
        "tracked_files_dirty": dirty,
    }


def create_manifest(
    run_root: Path,
    *,
    run_id: str,
    dataset_id: str | None,
    trial_kind: str,
    capture_profile: str,
    topics: Iterable[str],
    workspace: Path,
    gt_source: str = "none",
    media_policy: str = "formal_external",
    calibration_scene_id: str | None = None,
    physical_distance_m: float | None = None,
    physical_yaw_deg: float | None = None,
    physical_label_method: str | None = None,
) -> dict[str, Any]:
    run_id = validate_run_id(run_id)
    if trial_kind not in TRIAL_KINDS:
        raise CaptureManifestError(f"unsupported trial kind: {trial_kind}")
    created_utc = utc_now()
    frozen_calibration_label = calibration_label(
        trial_kind=trial_kind,
        scene_id=calibration_scene_id,
        physical_distance_m=physical_distance_m,
        physical_yaw_deg=physical_yaw_deg,
        measurement_method=physical_label_method,
        recorded_utc=created_utc,
    )
    run_root = run_root.resolve()
    if run_root.exists():
        raise CaptureManifestError(f"run path already exists: {run_root}")
    if gt_source not in {"none", "odin1"}:
        raise CaptureManifestError(f"unsupported GT source: {gt_source}")
    if media_policy not in {"formal_external", "onboard_episode"}:
        raise CaptureManifestError(f"unsupported media policy: {media_policy}")
    run_root.mkdir(parents=True)
    for child in ("logs", "media", "receipts"):
        (run_root / child).mkdir()
    payload = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "dataset_id": dataset_id or None,
        "trial_kind": trial_kind,
        "capture_profile": capture_profile,
        "gt_source": gt_source,
        "media_policy": media_policy,
        "state": "recording",
        "created_utc": created_utc,
        "captured_utc": None,
        "finalized_utc": None,
        "host": socket.gethostname(),
        "repository": repository_receipt(workspace.resolve()),
        "motion_authority_changed_by_capture": False,
        "calibration_label": frozen_calibration_label,
        "topics": sorted(set(str(topic) for topic in topics)),
        "media_contract": {
            "dashboard": {
                "required": media_policy == "formal_external",
                "capture": "external_foxglove_dashboard_then_sha256_import",
            },
            "third_view": {
                "required": media_policy == "formal_external",
                "capture": "external_camera_then_sha256_import",
            },
        },
        "capture_stop_clean": None,
        "outcome": None,
        "notes": "",
        "artifact_inventory": [],
        "completeness": None,
    }
    atomic_write_json(run_root / "manifest.json", payload)
    return payload


def read_manifest(run_root: Path) -> dict[str, Any]:
    path = run_root.resolve() / "manifest.json"
    if not path.is_file():
        raise CaptureManifestError(f"capture manifest is missing: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise CaptureManifestError(f"invalid capture manifest: {error}") from error
    if not isinstance(payload, dict) or payload.get("schema_version") not in {
        SCHEMA_VERSION,
        *LEGACY_SCHEMA_VERSIONS,
    }:
        raise CaptureManifestError("unexpected capture manifest schema")
    validate_run_id(str(payload.get("run_id", "")))
    return payload


def mark_captured(run_root: Path, *, clean: bool) -> dict[str, Any]:
    payload = read_manifest(run_root)
    if payload.get("state") != "recording":
        raise CaptureManifestError("only a recording run can be marked captured")
    payload["state"] = "captured"
    payload["captured_utc"] = utc_now()
    payload["capture_stop_clean"] = bool(clean)
    atomic_write_json(run_root.resolve() / "manifest.json", payload)
    return payload


def attach_video(run_root: Path, *, role: str, source: Path) -> dict[str, Any]:
    payload = read_manifest(run_root)
    if payload.get("state") == "finalized":
        raise CaptureManifestError("a finalized run cannot accept new media")
    if role not in VIDEO_ROLES:
        raise CaptureManifestError(f"unsupported video role: {role}")
    source = source.expanduser().resolve()
    if not source.is_file() or source.stat().st_size <= 0:
        raise CaptureManifestError(f"video is missing or empty: {source}")
    suffix = source.suffix.lower()
    if suffix not in VIDEO_SUFFIXES:
        raise CaptureManifestError(
            f"video suffix must be one of {sorted(VIDEO_SUFFIXES)}"
        )
    target = run_root.resolve() / "media" / f"{VIDEO_ROLES[role]}{suffix}"
    if target.exists():
        raise CaptureManifestError(f"media role already attached: {target}")
    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    shutil.copyfile(source, temporary)
    if sha256_file(temporary) != sha256_file(source):
        temporary.unlink(missing_ok=True)
        raise CaptureManifestError("copied video SHA-256 differs from its source")
    temporary.replace(target)
    receipt = {
        "role": role,
        "path": str(target.relative_to(run_root.resolve())),
        "bytes": target.stat().st_size,
        "sha256": sha256_file(target),
        "attached_utc": utc_now(),
    }
    atomic_write_json(run_root.resolve() / "receipts" / f"{role}.json", receipt)
    return receipt


def attach_reference(
    run_root: Path, *, role: str, source: Path
) -> dict[str, Any]:
    payload = read_manifest(run_root)
    if payload.get("state") == "finalized":
        raise CaptureManifestError("a finalized run cannot accept new reference evidence")
    if role not in REFERENCE_ROLES:
        raise CaptureManifestError(f"unsupported reference role: {role}")
    if payload.get("gt_source") != "odin1":
        raise CaptureManifestError("Odin reference evidence requires gt_source=odin1")
    source = source.expanduser().resolve()
    if not source.is_file() or source.stat().st_size <= 0:
        raise CaptureManifestError(f"reference evidence is missing or empty: {source}")
    try:
        reference_payload = json.loads(source.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise CaptureManifestError(f"invalid reference JSON: {error}") from error
    expected_schema = {
        "odin_gt_result": "memnav-odin1-gt-result-v1",
        "odin_spl_receipt": "memnav-odin1-spl-receipt-v1",
    }[role]
    if reference_payload.get("schema") != expected_schema:
        raise CaptureManifestError(
            f"{role} expected schema {expected_schema}, got "
            f"{reference_payload.get('schema')}"
        )
    if reference_payload.get("run_id") != payload.get("run_id"):
        raise CaptureManifestError(
            f"{role} run_id does not match the capture manifest"
        )
    if role == "odin_spl_receipt":
        result_path = run_root.resolve() / "receipts" / REFERENCE_ROLES[
            "odin_gt_result"
        ]
        if not result_path.is_file():
            raise CaptureManifestError(
                "attach odin_gt_result before the Odin SPL receipt"
            )
        if (
            reference_payload.get("inputs", {})
            .get("gt_result", {})
            .get("sha256")
            != sha256_file(result_path)
        ):
            raise CaptureManifestError(
                "Odin SPL receipt is not hash-bound to the attached GT result"
            )
        result_payload = json.loads(result_path.read_text(encoding="utf-8"))
        metrics = reference_payload.get("metrics", {})
        if metrics.get("S_i") != int(bool(result_payload.get("success"))):
            raise CaptureManifestError(
                "Odin SPL success does not match the attached GT result"
            )
    target = run_root.resolve() / "receipts" / REFERENCE_ROLES[role]
    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    shutil.copyfile(source, temporary)
    if sha256_file(temporary) != sha256_file(source):
        temporary.unlink(missing_ok=True)
        raise CaptureManifestError("copied reference SHA-256 differs from its source")
    temporary.replace(target)
    receipt = {
        "role": role,
        "path": str(target.relative_to(run_root.resolve())),
        "bytes": target.stat().st_size,
        "sha256": sha256_file(target),
        "attached_utc": utc_now(),
    }
    atomic_write_json(run_root.resolve() / "receipts" / f"{role}.receipt.json", receipt)
    return receipt


def artifact_inventory(run_root: Path) -> list[dict[str, Any]]:
    root = run_root.resolve()
    excluded = {"manifest.json", "MANIFEST.sha256", "FINALIZED"}
    artifacts = []
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        if relative in excluded or relative.startswith("."):
            continue
        artifacts.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return artifacts


def completeness(
    run_root: Path,
    inventory: list[dict[str, Any]],
    *,
    gt_source: str = "none",
    require_external_media: bool = True,
) -> dict[str, bool]:
    paths = {row["path"] for row in inventory if int(row["bytes"]) > 0}
    rosbag_metadata = "rosbag/metadata.yaml" in paths
    rosbag_storage = any(
        path.startswith("rosbag/") and Path(path).suffix in {".db3", ".mcap"}
        for path in paths
    )
    dashboard = any(
        path.startswith("media/dashboard")
        and Path(path).suffix.lower() in VIDEO_SUFFIXES
        for path in paths
    )
    third_view = any(
        path.startswith("media/third_view") and Path(path).suffix.lower() in VIDEO_SUFFIXES
        for path in paths
    )
    status_receipts = "logs/status.jsonl" in paths
    cec_receipts = "logs/cec_receipt.jsonl" in paths
    odin_gt_status = "logs/odin_gt_status.jsonl" in paths
    odin_gt_result = "receipts/odin_gt_result.json" in paths
    odin_spl_receipt = "receipts/odin_spl_receipt.json" in paths
    result = {
        "rosbag": rosbag_metadata and rosbag_storage,
        "dashboard": dashboard or not require_external_media,
        "third_view": third_view or not require_external_media,
        "status_receipts": status_receipts,
        "cec_receipts": cec_receipts,
        "odin_gt_status": gt_source != "odin1" or odin_gt_status,
        "odin_gt_result": gt_source != "odin1" or odin_gt_result,
        "odin_spl_receipt": gt_source != "odin1" or odin_spl_receipt,
    }
    result["formal_complete"] = all(result.values())
    return result


def finalize_manifest(
    run_root: Path,
    *,
    outcome: str,
    notes: str,
    allow_incomplete: bool,
) -> dict[str, Any]:
    payload = read_manifest(run_root)
    if payload.get("state") != "captured":
        raise CaptureManifestError("capture must be stopped before finalization")
    if outcome not in OUTCOMES:
        raise CaptureManifestError(f"unsupported outcome: {outcome}")
    inventory = artifact_inventory(run_root)
    gates = completeness(
        run_root,
        inventory,
        gt_source=str(payload.get("gt_source", "none")),
        require_external_media=payload.get("media_policy", "formal_external")
        == "formal_external",
    )
    if not gates["formal_complete"] and not allow_incomplete:
        missing = ", ".join(key for key, value in gates.items() if not value)
        raise CaptureManifestError(
            f"capture evidence is incomplete ({missing}); attach evidence or use "
            "--allow-incomplete for an explicitly incomplete engineering run"
        )
    payload.update(
        {
            "state": "finalized",
            "finalized_utc": utc_now(),
            "outcome": outcome,
            "notes": str(notes),
            "artifact_inventory": inventory,
            "completeness": gates,
        }
    )
    manifest_path = run_root.resolve() / "manifest.json"
    atomic_write_json(manifest_path, payload)
    digest = sha256_file(manifest_path)
    (run_root.resolve() / "MANIFEST.sha256").write_text(
        f"{digest}  manifest.json\n", encoding="ascii"
    )
    (run_root.resolve() / "FINALIZED").write_text(
        f"{payload['schema_version']}\n{digest}\n", encoding="ascii"
    )
    return payload


def verify_manifest(run_root: Path) -> dict[str, Any]:
    root = run_root.resolve()
    payload = read_manifest(root)
    if payload.get("state") != "finalized":
        raise CaptureManifestError("run is not finalized")
    manifest_path = root / "manifest.json"
    digest = sha256_file(manifest_path)
    if (root / "MANIFEST.sha256").read_text(encoding="ascii") != (
        f"{digest}  manifest.json\n"
    ):
        raise CaptureManifestError("MANIFEST.sha256 does not match manifest.json")
    if (root / "FINALIZED").read_text(encoding="ascii").splitlines() != [
        str(payload["schema_version"]),
        digest,
    ]:
        raise CaptureManifestError("FINALIZED receipt does not match manifest.json")
    current = artifact_inventory(root)
    if current != payload.get("artifact_inventory"):
        raise CaptureManifestError("capture artifact inventory changed after finalization")
    expected = completeness(
        root,
        current,
        gt_source=str(payload.get("gt_source", "none")),
        require_external_media=payload.get("media_policy", "formal_external")
        == "formal_external",
    )
    if expected != payload.get("completeness"):
        raise CaptureManifestError("capture completeness receipt changed")
    return {
        "run_id": payload["run_id"],
        "manifest_sha256": digest,
        "artifacts": len(current),
        "completeness": expected,
    }


def print_json(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    create = subparsers.add_parser("create")
    create.add_argument("--run-root", type=Path, required=True)
    create.add_argument("--run-id", required=True)
    create.add_argument("--dataset-id", default="")
    create.add_argument("--trial-kind", default="revisit")
    create.add_argument("--capture-profile", choices=("audit", "full"), default="audit")
    create.add_argument("--topic", action="append", default=[])
    create.add_argument("--gt-source", choices=("none", "odin1"), default="none")
    create.add_argument(
        "--media-policy",
        choices=("formal_external", "onboard_episode"),
        default="formal_external",
    )
    create.add_argument("--calibration-scene-id")
    create.add_argument("--physical-distance-m", type=float)
    create.add_argument("--physical-yaw-deg", type=float)
    create.add_argument("--physical-label-method")
    create.add_argument(
        "--workspace", type=Path, default=Path(__file__).resolve().parents[2]
    )

    captured = subparsers.add_parser("mark-captured")
    captured.add_argument("--run-root", type=Path, required=True)
    captured.add_argument("--clean", choices=("true", "false"), required=True)

    attach = subparsers.add_parser("attach-video")
    attach.add_argument("--run-root", type=Path, required=True)
    attach.add_argument("--role", choices=tuple(VIDEO_ROLES), required=True)
    attach.add_argument("--source", type=Path, required=True)

    attach_gt = subparsers.add_parser("attach-reference")
    attach_gt.add_argument("--run-root", type=Path, required=True)
    attach_gt.add_argument("--role", choices=tuple(REFERENCE_ROLES), required=True)
    attach_gt.add_argument("--source", type=Path, required=True)

    finalize = subparsers.add_parser("finalize")
    finalize.add_argument("--run-root", type=Path, required=True)
    finalize.add_argument("--outcome", choices=OUTCOMES, required=True)
    finalize.add_argument("--notes", default="")
    finalize.add_argument("--allow-incomplete", action="store_true")

    verify = subparsers.add_parser("verify")
    verify.add_argument("--run-root", type=Path, required=True)

    args = parser.parse_args()
    try:
        if args.command == "create":
            result = create_manifest(
                args.run_root,
                run_id=args.run_id,
                dataset_id=args.dataset_id,
                trial_kind=args.trial_kind,
                capture_profile=args.capture_profile,
                topics=args.topic,
                workspace=args.workspace,
                gt_source=args.gt_source,
                media_policy=args.media_policy,
                calibration_scene_id=args.calibration_scene_id,
                physical_distance_m=args.physical_distance_m,
                physical_yaw_deg=args.physical_yaw_deg,
                physical_label_method=args.physical_label_method,
            )
        elif args.command == "mark-captured":
            result = mark_captured(args.run_root, clean=args.clean == "true")
        elif args.command == "attach-video":
            result = attach_video(args.run_root, role=args.role, source=args.source)
        elif args.command == "attach-reference":
            result = attach_reference(
                args.run_root, role=args.role, source=args.source
            )
        elif args.command == "finalize":
            result = finalize_manifest(
                args.run_root,
                outcome=args.outcome,
                notes=args.notes,
                allow_incomplete=args.allow_incomplete,
            )
        else:
            result = verify_manifest(args.run_root)
    except (CaptureManifestError, FileNotFoundError, OSError) as error:
        parser.error(str(error))
    print_json(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
