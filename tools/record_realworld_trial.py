#!/usr/bin/env python3
"""Pre-register and seal operator-facing real-world trial records.

This ledger is deliberately separate from the publication-grade paired campaign
verifier.  It binds a human-readable table label to a finalized Episode without
turning pipeline outcomes into independently adjudicated metrics.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LEDGER = ROOT / "runtime" / "go2" / "formal_trial_ledger"
DEFAULT_CAPTURE_ROOT = ROOT / "runtime" / "go2" / "experiment_capture"
SCENES = {"small_room": "Small room", "large_suite": "Large suite"}
LEG2_TYPES = {"novel": "Novel", "revisit": "Revisit"}
METHODS = {
    "native": {"display": "Native", "campaign_arm": "mono_native"},
    "ours": {"display": "Ours", "campaign_arm": "mono_cec"},
}


class LedgerError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def parse_utc(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise LedgerError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise LedgerError(f"expected a JSON object: {path}")
    return value


def write_new_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as exc:
        raise LedgerError(f"refusing to overwrite immutable record: {path}") from exc


def replace_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def active_registration(ledger: Path) -> dict[str, Any] | None:
    active_path = ledger / "ACTIVE.json"
    if not active_path.is_file():
        return None
    active = read_json(active_path)
    trial_id = active.get("trial_id")
    if trial_id is None:
        return None
    registration_path = ledger / "registrations" / f"{trial_id}.json"
    registration = read_json(registration_path)
    expected = active.get("registration_sha256")
    actual = sha256_file(registration_path)
    if expected != actual:
        raise LedgerError("active registration SHA-256 does not match")
    return registration


def command_register(args: argparse.Namespace) -> dict[str, Any]:
    ledger = args.ledger.resolve()
    if active_registration(ledger) is not None:
        raise LedgerError("an active trial is already registered; seal it first")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    trial_id = args.trial_id or f"trial_{stamp}_{uuid.uuid4().hex[:6]}"
    label = {
        "scene": SCENES[args.scene],
        "scene_id": args.scene,
        "leg2_type": LEG2_TYPES[args.leg2_type],
        "leg2_type_id": args.leg2_type,
        "method": METHODS[args.method]["display"],
        "method_id": args.method,
        "campaign_arm": METHODS[args.method]["campaign_arm"],
        "pair_index": args.pair_index,
    }
    slot = (
        label["scene_id"], label["leg2_type_id"],
        label["method_id"], label["pair_index"],
    )
    for path in (ledger / "records").glob("*.json"):
        previous = read_json(path)["registration"]["label"]
        previous_slot = (
            previous["scene_id"], previous["leg2_type_id"],
            previous["method_id"], previous["pair_index"],
        )
        if previous_slot == slot:
            raise LedgerError(
                "this Scene/Leg-2/Method/Pair slot already has a sealed record"
            )
    registration = {
        "schema_version": "memnav_operator_trial_registration_v1",
        "classification": "operator_ledger_not_formal_campaign_result",
        "trial_id": trial_id,
        "registered_utc": utc_now(),
        "label": label,
        "notes": args.notes,
        "outcomes_visible_when_registered": False,
    }
    registration_path = ledger / "registrations" / f"{trial_id}.json"
    write_new_json(registration_path, registration)
    replace_json(
        ledger / "ACTIVE.json",
        {
            "trial_id": trial_id,
            "registration_sha256": sha256_file(registration_path),
        },
    )
    return registration


def verify_episode(run_root: Path) -> tuple[dict[str, Any], str]:
    manifest_path = run_root / "manifest.json"
    seal_path = run_root / "MANIFEST.sha256"
    if not (run_root / "FINALIZED").is_file():
        raise LedgerError(f"Episode is not finalized: {run_root.name}")
    manifest = read_json(manifest_path)
    digest = sha256_file(manifest_path)
    expected_line = f"{digest}  manifest.json\n"
    if not seal_path.is_file() or seal_path.read_text(encoding="ascii") != expected_line:
        raise LedgerError(f"invalid Episode manifest seal: {run_root.name}")
    if manifest.get("state") != "finalized":
        raise LedgerError(f"Episode manifest is not finalized: {run_root.name}")
    return manifest, digest


def resolve_episode(
    capture_root: Path, episode_id: str | None, registered_utc: str
) -> tuple[Path, dict[str, Any], str]:
    if episode_id:
        candidates = [capture_root / episode_id]
    else:
        candidates = [
            path.parent
            for path in capture_root.glob("*/manifest.json")
            if (path.parent / "FINALIZED").is_file()
        ]
        candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    registered_at = parse_utc(registered_utc)
    for run_root in candidates:
        if not run_root.is_dir():
            continue
        manifest, digest = verify_episode(run_root)
        created = manifest.get("created_utc")
        if not isinstance(created, str) or parse_utc(created) < registered_at:
            if episode_id:
                raise LedgerError("Episode predates the active trial registration")
            continue
        return run_root, manifest, digest
    raise LedgerError("no finalized Episode created after the active registration")


def write_csv(ledger: Path) -> None:
    rows = []
    for path in sorted((ledger / "records").glob("*.json")):
        record = read_json(path)
        label = record["registration"]["label"]
        evidence = record["evidence"]
        adjudication = record["adjudication"]
        rows.append(
            {
                "trial_id": record["trial_id"],
                "scene": label["scene"],
                "leg2_type": label["leg2_type"],
                "method": label["method"],
                "pair_index": label["pair_index"],
                "episode_id": evidence["episode_id"],
                "pipeline_outcome": evidence["pipeline_outcome"],
                "success": adjudication["success"],
                "final_distance_m": adjudication["final_distance_m"],
                "metric_source": adjudication["metric_source"],
                "record_status": record["record_status"],
                "manifest_sha256": evidence["manifest_sha256"],
            }
        )
    output = ledger / "trials.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "trial_id", "scene", "leg2_type", "method", "pair_index",
        "episode_id", "pipeline_outcome", "success", "final_distance_m",
        "metric_source", "record_status", "manifest_sha256",
    ]
    temporary = output.with_name(f".{output.name}.{uuid.uuid4().hex}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(output)


def command_record(args: argparse.Namespace) -> dict[str, Any]:
    ledger = args.ledger.resolve()
    registration = active_registration(ledger)
    if registration is None:
        raise LedgerError("no active pre-registered trial")
    run_root, manifest, manifest_sha = resolve_episode(
        args.capture_root.resolve(), args.episode_id, registration["registered_utc"]
    )
    expected_kind = registration["label"]["leg2_type_id"]
    if manifest.get("trial_kind") != expected_kind:
        raise LedgerError(
            f"trial_kind mismatch: label={expected_kind}, "
            f"manifest={manifest.get('trial_kind')}"
        )
    if args.final_distance_m is not None and args.final_distance_m < 0:
        raise LedgerError("--final-distance-m cannot be negative")
    if args.final_distance_m is not None and not args.metric_source:
        raise LedgerError("--metric-source is required with --final-distance-m")
    success = None if args.success is None else args.success == "1"
    independently_complete = (
        success is not None
        and args.final_distance_m is not None
        and bool(args.metric_source)
    )
    trial_id = registration["trial_id"]
    record = {
        "schema_version": "memnav_operator_trial_record_v1",
        "classification": "operator_ledger_not_formal_campaign_result",
        "trial_id": trial_id,
        "recorded_utc": utc_now(),
        "record_status": (
            "recorded_with_independent_metrics"
            if independently_complete
            else "recorded_pending_independent_metrics"
        ),
        "registration": registration,
        "evidence": {
            "episode_id": manifest.get("run_id", run_root.name),
            "episode_root": str(run_root),
            "manifest_sha256": manifest_sha,
            "dataset_id": manifest.get("dataset_id"),
            "trial_kind": manifest.get("trial_kind"),
            "pipeline_outcome": manifest.get("outcome"),
            "capture_profile": manifest.get("capture_profile"),
            "capture_formal_complete": manifest.get("completeness", {}).get(
                "formal_complete"
            ),
            "gt_source": manifest.get("gt_source"),
        },
        "adjudication": {
            "success": success,
            "final_distance_m": args.final_distance_m,
            "metric_source": args.metric_source,
            "notes": args.notes,
        },
        "warnings": [
            "Method is a pre-registered operator label; publication-grade use "
            "still requires verification against the frozen arm/config receipts."
        ],
    }
    record_path = ledger / "records" / f"{trial_id}.json"
    write_new_json(record_path, record)
    replace_json(ledger / "ACTIVE.json", {"trial_id": None})
    write_csv(ledger)
    return record


def command_status(args: argparse.Namespace) -> dict[str, Any]:
    ledger = args.ledger.resolve()
    records = list((ledger / "records").glob("*.json"))
    return {
        "ledger": str(ledger),
        "active_registration": active_registration(ledger),
        "sealed_records": len(records),
        "csv": str(ledger / "trials.csv"),
    }


def parser() -> argparse.ArgumentParser:
    main = argparse.ArgumentParser(description=__doc__)
    main.add_argument("--ledger", type=Path, default=DEFAULT_LEDGER)
    sub = main.add_subparsers(dest="command", required=True)

    register = sub.add_parser("register", help="freeze a label before the run")
    register.add_argument("--scene", choices=SCENES, required=True)
    register.add_argument("--leg2-type", choices=LEG2_TYPES, required=True)
    register.add_argument("--method", choices=METHODS, required=True)
    register.add_argument("--pair-index", type=int, choices=range(1, 6), required=True)
    register.add_argument("--trial-id")
    register.add_argument("--notes", default="")
    register.set_defaults(function=command_register)

    record = sub.add_parser("record", help="bind the active label to a finalized Episode")
    record.add_argument("--capture-root", type=Path, default=DEFAULT_CAPTURE_ROOT)
    record.add_argument("--episode-id")
    record.add_argument("--success", choices=("0", "1"))
    record.add_argument("--final-distance-m", type=float)
    record.add_argument("--metric-source")
    record.add_argument("--notes", default="")
    record.set_defaults(function=command_record)

    status = sub.add_parser("status", help="show active registration and record count")
    status.set_defaults(function=command_status)
    return main


def main() -> int:
    args = parser().parse_args()
    try:
        result = args.function(args)
    except LedgerError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
