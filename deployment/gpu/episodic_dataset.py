#!/usr/bin/env python3
"""Immutable real-world episodic RGB datasets for two-pass Revisit trials.

The policy consumes RGB only.  Optional aligned depth is stored next to a
goal candidate solely for the independent arrival evaluator; it is never
returned as a navigation observation.  A candidate image and every guarded
memory image are hash checked to remain disjoint, preventing an exact-JPEG
self-match from masquerading as Revisit localization.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import math
from typing import Any, Iterable, Mapping


SCHEMA_VERSION = "cec_realworld_episodic_dataset_v1_20260825"
_DATASET_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")
ONE_WAY_EXTERNAL_GOAL_MODE = "manual_one_way_external_goal_debug"
EXTERNAL_GOAL_CONTRACT = "operator_frozen_external_required"


class DatasetContractError(RuntimeError):
    """The requested operation would violate the frozen dataset contract."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_json(payload: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
        + "\n"
    ).encode("utf-8")


def _public_record(record: Mapping[str, Any]) -> dict[str, Any]:
    """Remove process-only bytes and absolute paths from a candidate receipt."""

    return {
        str(key): value
        for key, value in record.items()
        if key not in {"image", "evaluation_depth", "path"}
    }


def validate_dataset_id(dataset_id: str) -> str:
    value = str(dataset_id).strip()
    if not _DATASET_ID.fullmatch(value):
        raise DatasetContractError(
            "dataset_id must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}"
        )
    return value


def candidate_free_external_goal_debug(metadata: Mapping[str, Any]) -> bool:
    """Return whether this engineering dataset must use an external goal.

    Registered Survey/Formal datasets retain the stricter candidate rule.  A
    one-way M->query-start debug trace has no physical return leg and installs
    an exact externally frozen goal during formal preparation, so manufacturing
    an unrelated Survey candidate would weaken rather than strengthen its
    audit contract.
    """

    return (
        metadata.get("collection_mode") == ONE_WAY_EXTERNAL_GOAL_MODE
        and metadata.get("goal_selection_contract") == EXTERNAL_GOAL_CONTRACT
        and metadata.get("goal_candidates_required") is False
    )


@dataclass(frozen=True)
class LoadedDataset:
    root: Path
    manifest: dict[str, Any]

    def memory_frames(self) -> Iterable[tuple[dict[str, Any], bytes]]:
        for record in self.manifest["memory_frames"]:
            path = self.root / record["path"]
            yield dict(record), path.read_bytes()

    def goal_candidates(
        self,
    ) -> Iterable[tuple[dict[str, Any], bytes, bytes | None]]:
        for record in self.manifest["goal_candidates"]:
            image = (self.root / record["path"]).read_bytes()
            depth_path = record.get("evaluation_depth_path")
            depth = None if depth_path is None else (self.root / depth_path).read_bytes()
            yield dict(record), image, depth


class EpisodicDatasetStore:
    """Record, seal, verify and load one exact-byte causal RGB stream."""

    def __init__(self, root: str | Path, *, minimum_frames: int = 64) -> None:
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.minimum_frames = max(1, int(minimum_frames))
        self._dataset_id: str | None = None
        self._staging_root: Path | None = None
        self._created_utc: str | None = None
        self._metadata: dict[str, Any] = {}
        self._memory_frames: list[dict[str, Any]] = []
        self._goal_candidates: list[dict[str, Any]] = []
        self._memory_hashes: set[str] = set()

    @property
    def recording(self) -> bool:
        return self._staging_root is not None

    def status(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "root": str(self.root),
            "recording": self.recording,
            "dataset_id": self._dataset_id,
            "memory_frames": len(self._memory_frames),
            "goal_candidates": len(self._goal_candidates),
            "minimum_frames": self.minimum_frames,
        }

    def start(
        self,
        dataset_id: str,
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self.recording:
            raise DatasetContractError(
                f"dataset {self._dataset_id!r} is already recording"
            )
        dataset_id = validate_dataset_id(dataset_id)
        final_root = self.root / dataset_id
        staging_root = self.root / f".{dataset_id}.staging"
        if final_root.exists() or staging_root.exists():
            raise DatasetContractError(
                f"dataset path already exists; refusing overwrite: {dataset_id}"
            )
        (staging_root / "memory").mkdir(parents=True)
        (staging_root / "goals").mkdir(parents=True)
        self._dataset_id = dataset_id
        self._staging_root = staging_root
        self._created_utc = _utc_now()
        self._metadata = dict(metadata or {})
        self._memory_frames = []
        self._goal_candidates = []
        self._memory_hashes = set()
        return self.status()

    @staticmethod
    def _write_exact(path: Path, payload: bytes) -> None:
        if not payload:
            raise DatasetContractError("dataset artifacts must be non-empty")
        temporary = path.with_name(f".{path.name}.tmp")
        temporary.write_bytes(payload)
        temporary.replace(path)

    def append_memory(
        self,
        *,
        frame_index: int,
        image: bytes,
        upstream_sha256: str | None,
    ) -> dict[str, Any] | None:
        if not self.recording:
            return None
        assert self._staging_root is not None
        expected_index = len(self._memory_frames)
        if isinstance(frame_index, bool) or int(frame_index) != expected_index:
            raise DatasetContractError(
                f"non-contiguous memory index: got {frame_index}, expected {expected_index}"
            )
        digest = _sha256(image)
        if upstream_sha256 is not None and str(upstream_sha256) != digest:
            raise DatasetContractError("upstream and dataset frame SHA-256 disagree")
        name = f"{expected_index:06d}_{digest[:16]}.jpg"
        relative = Path("memory") / name
        self._write_exact(self._staging_root / relative, image)
        record = {
            "frame_index": expected_index,
            "path": relative.as_posix(),
            "sha256": digest,
            "bytes": len(image),
        }
        self._memory_frames.append(record)
        self._memory_hashes.add(digest)
        return dict(record)

    def append_candidate(
        self,
        *,
        record: Mapping[str, Any],
        image: bytes,
        evaluation_depth: bytes | None = None,
        evaluation_depth_scale_m: float | None = None,
    ) -> dict[str, Any] | None:
        if not self.recording:
            return None
        assert self._staging_root is not None
        digest = _sha256(image)
        if digest in self._memory_hashes:
            raise DatasetContractError(
                "goal candidate exactly duplicates a recorded memory JPEG"
            )
        candidate_id = int(record["candidate_id"])
        if candidate_id != len(self._goal_candidates):
            raise DatasetContractError(
                f"non-contiguous candidate id: got {candidate_id}, "
                f"expected {len(self._goal_candidates)}"
            )
        if record.get("appended_to_memory") is not False:
            raise DatasetContractError("goal candidate must be excluded from memory")
        if str(record.get("sha256")) != digest:
            raise DatasetContractError("candidate receipt and JPEG SHA-256 disagree")
        name = f"candidate_{candidate_id:03d}_{digest[:16]}.jpg"
        relative = Path("goals") / name
        self._write_exact(self._staging_root / relative, image)
        frozen = {
            **_public_record(record),
            "path": relative.as_posix(),
            "bytes": len(image),
        }
        if evaluation_depth is not None:
            if (
                evaluation_depth_scale_m is None
                or not math.isfinite(float(evaluation_depth_scale_m))
                or float(evaluation_depth_scale_m) <= 0.0
            ):
                raise DatasetContractError(
                    "evaluation depth requires a finite positive metre scale"
                )
            depth_digest = _sha256(evaluation_depth)
            depth_relative = Path("goals") / (
                f"candidate_{candidate_id:03d}_{digest[:16]}_depth.png"
            )
            self._write_exact(self._staging_root / depth_relative, evaluation_depth)
            frozen.update({
                "evaluation_depth_path": depth_relative.as_posix(),
                "evaluation_depth_sha256": depth_digest,
                "evaluation_depth_bytes": len(evaluation_depth),
                "evaluation_depth_scale_m": float(evaluation_depth_scale_m),
                "evaluation_depth_policy_authority": False,
            })
        else:
            frozen.update({
                "evaluation_depth_path": None,
                "evaluation_depth_sha256": None,
                "evaluation_depth_bytes": None,
                "evaluation_depth_scale_m": None,
                "evaluation_depth_policy_authority": False,
            })
        self._goal_candidates.append(frozen)
        return dict(frozen)

    def seal(self, *, protocol: Mapping[str, Any]) -> dict[str, Any]:
        if not self.recording:
            raise DatasetContractError("no dataset is recording")
        assert self._dataset_id is not None
        assert self._staging_root is not None
        if len(self._memory_frames) < self.minimum_frames:
            raise DatasetContractError(
                f"dataset has {len(self._memory_frames)} frames; "
                f"minimum is {self.minimum_frames}"
            )
        external_goal_only = candidate_free_external_goal_debug(self._metadata)
        if external_goal_only and self._goal_candidates:
            raise DatasetContractError(
                "external-goal-only debug dataset contains Survey candidates"
            )
        if not self._goal_candidates and not external_goal_only:
            raise DatasetContractError(
                "dataset has no supported, memory-excluded goal candidate"
            )
        candidate_hashes = {row["sha256"] for row in self._goal_candidates}
        overlap = sorted(candidate_hashes.intersection(self._memory_hashes))
        if overlap:
            raise DatasetContractError(
                "goal/memory exact-JPEG overlap detected: " + ",".join(overlap)
            )
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "dataset_id": self._dataset_id,
            "created_utc": self._created_utc,
            "sealed_utc": _utc_now(),
            "metadata": self._metadata,
            "protocol": dict(protocol),
            "summary": {
                "memory_frames": len(self._memory_frames),
                "goal_candidates": len(self._goal_candidates),
                "goal_memory_exact_sha_overlap": 0,
                "evaluation_depth_consumed_by_policy": False,
            },
            "memory_frames": list(self._memory_frames),
            "goal_candidates": list(self._goal_candidates),
        }
        raw = _canonical_json(manifest)
        digest = _sha256(raw)
        self._write_exact(self._staging_root / "manifest.json", raw)
        self._write_exact(
            self._staging_root / "MANIFEST.sha256",
            f"{digest}  manifest.json\n".encode("ascii"),
        )
        self._write_exact(
            self._staging_root / "SEALED",
            f"{SCHEMA_VERSION}\n{digest}\n".encode("ascii"),
        )
        final_root = self.root / self._dataset_id
        self._staging_root.replace(final_root)
        result = {
            "schema_version": SCHEMA_VERSION,
            "dataset_id": self._dataset_id,
            "dataset_root": str(final_root),
            "manifest_sha256": digest,
            **manifest["summary"],
        }
        self._dataset_id = None
        self._staging_root = None
        self._created_utc = None
        self._metadata = {}
        self._memory_frames = []
        self._goal_candidates = []
        self._memory_hashes = set()
        return result

    def load(self, dataset_id: str) -> LoadedDataset:
        dataset_id = validate_dataset_id(dataset_id)
        root = self.root / dataset_id
        manifest_path = root / "manifest.json"
        manifest_receipt_path = root / "MANIFEST.sha256"
        seal_path = root / "SEALED"
        if (
            not manifest_path.is_file()
            or not manifest_receipt_path.is_file()
            or not seal_path.is_file()
        ):
            raise DatasetContractError(f"dataset is not sealed: {dataset_id}")
        raw = manifest_path.read_bytes()
        digest = _sha256(raw)
        expected_manifest_receipt = f"{digest}  manifest.json\n"
        if manifest_receipt_path.read_text(encoding="ascii") != expected_manifest_receipt:
            raise DatasetContractError("MANIFEST.sha256 does not match manifest")
        seal_lines = seal_path.read_text(encoding="ascii").splitlines()
        if seal_lines != [SCHEMA_VERSION, digest]:
            raise DatasetContractError("SEALED receipt does not match manifest")
        try:
            manifest = json.loads(raw)
        except json.JSONDecodeError as error:
            raise DatasetContractError(f"invalid dataset manifest: {error}") from error
        if (
            not isinstance(manifest, dict)
            or manifest.get("schema_version") != SCHEMA_VERSION
            or manifest.get("dataset_id") != dataset_id
            or _canonical_json(manifest) != raw
        ):
            raise DatasetContractError("dataset manifest identity is invalid")

        memory_hashes: set[str] = set()
        for expected_index, record in enumerate(manifest.get("memory_frames", [])):
            if int(record.get("frame_index", -1)) != expected_index:
                raise DatasetContractError("dataset memory indices are not contiguous")
            path = root / str(record.get("path", ""))
            payload = path.read_bytes()
            if len(payload) != int(record.get("bytes", -1)):
                raise DatasetContractError(f"memory byte count changed: {path}")
            if _sha256(payload) != record.get("sha256"):
                raise DatasetContractError(f"memory SHA-256 changed: {path}")
            memory_hashes.add(record["sha256"])

        candidates = manifest.get("goal_candidates", [])
        metadata = manifest.get("metadata", {})
        external_goal_only = (
            isinstance(metadata, dict)
            and candidate_free_external_goal_debug(metadata)
        )
        if not isinstance(candidates, list):
            raise DatasetContractError("sealed dataset goal candidates are invalid")
        if external_goal_only and candidates:
            raise DatasetContractError(
                "external-goal-only debug dataset contains Survey candidates"
            )
        if not candidates and not external_goal_only:
            raise DatasetContractError("sealed dataset has no goal candidates")
        for expected_id, record in enumerate(candidates):
            if int(record.get("candidate_id", -1)) != expected_id:
                raise DatasetContractError("candidate ids are not contiguous")
            path = root / str(record.get("path", ""))
            payload = path.read_bytes()
            if len(payload) != int(record.get("bytes", -1)):
                raise DatasetContractError(f"candidate byte count changed: {path}")
            digest_value = _sha256(payload)
            if digest_value != record.get("sha256") or digest_value in memory_hashes:
                raise DatasetContractError(
                    f"candidate identity/exclusion check failed: {path}"
                )
            depth_path = record.get("evaluation_depth_path")
            if depth_path is not None:
                depth_scale_m = record.get("evaluation_depth_scale_m")
                if (
                    isinstance(depth_scale_m, bool)
                    or not isinstance(depth_scale_m, (int, float))
                    or not math.isfinite(float(depth_scale_m))
                    or float(depth_scale_m) <= 0.0
                ):
                    raise DatasetContractError(
                        "evaluation depth has an invalid metre scale"
                    )
                depth_payload = (root / depth_path).read_bytes()
                if (
                    _sha256(depth_payload) != record.get("evaluation_depth_sha256")
                    or len(depth_payload)
                    != int(record.get("evaluation_depth_bytes", -1))
                ):
                    raise DatasetContractError(
                        f"evaluation depth identity changed: {depth_path}"
                    )
            if record.get("evaluation_depth_policy_authority") is not False:
                raise DatasetContractError("evaluation depth gained policy authority")
        return LoadedDataset(root=root, manifest=manifest)

    def list_sealed(self) -> list[dict[str, Any]]:
        """List seal receipts without re-hashing every RGB artifact.

        This endpoint is used by the field status command and must remain
        responsive even after a long survey.  ``load`` is deliberately the
        stronger boundary: it verifies every frame, goal and evaluator-only
        depth artifact before any bytes are replayed into MemNav.
        """
        result = []
        for path in sorted(self.root.iterdir()):
            if path.is_dir() and not path.name.startswith("."):
                try:
                    raw = (path / "manifest.json").read_bytes()
                    digest = _sha256(raw)
                    manifest_receipt = (path / "MANIFEST.sha256").read_text(
                        encoding="ascii"
                    )
                    seal_lines = (path / "SEALED").read_text(
                        encoding="ascii"
                    ).splitlines()
                    manifest = json.loads(raw)
                    if (
                        manifest_receipt != f"{digest}  manifest.json\n"
                        or seal_lines != [SCHEMA_VERSION, digest]
                        or not isinstance(manifest, dict)
                        or manifest.get("schema_version") != SCHEMA_VERSION
                        or manifest.get("dataset_id") != path.name
                        or _canonical_json(manifest) != raw
                    ):
                        continue
                    summary = manifest["summary"]
                except (KeyError, json.JSONDecodeError, OSError):
                    continue
                result.append({
                    "dataset_id": path.name,
                    "manifest_sha256": digest,
                    "artifacts_verified": False,
                    **summary,
                })
        return result
