import hashlib
import shutil

import pytest

from deployment.gpu.episodic_dataset import EpisodicDatasetStore
from deployment.gpu.recover_one_way_debug_dataset import (
    RecoveryError,
    recover_one_way_debug_dataset,
)


def _write_staging(root, dataset_id="route"):
    staging = root / f".{dataset_id}.staging"
    (staging / "memory").mkdir(parents=True)
    (staging / "goals").mkdir()
    frames = [b"frame-0", b"candidate-and-frame-1", b"frame-2"]
    for index, payload in enumerate(frames):
        digest = hashlib.sha256(payload).hexdigest()
        (staging / "memory" / f"{index:06d}_{digest[:16]}.jpg").write_bytes(payload)
    candidate_sha = hashlib.sha256(frames[1]).hexdigest()
    (staging / "goals" / f"candidate_000_{candidate_sha[:16]}.jpg").write_bytes(
        frames[1]
    )
    (staging / "goals" / f"candidate_000_{candidate_sha[:16]}_depth.png").write_bytes(
        b"depth"
    )
    return staging, candidate_sha


def test_recovery_preserves_source_and_seals_external_goal_dataset(tmp_path):
    staging, candidate_sha = _write_staging(tmp_path)
    backup = tmp_path / "backup"
    shutil.copytree(staging, backup)
    external_goal_sha = hashlib.sha256(b"external-M").hexdigest()

    receipt = recover_one_way_debug_dataset(
        root=tmp_path,
        dataset_id="route",
        backup_root=backup,
        expected_memory_frames=3,
        created_utc="2026-08-30T14:22:36Z",
        external_goal_sha256=external_goal_sha,
    )

    assert receipt["memory_frames"] == 3
    assert receipt["goal_candidates"] == 0
    assert receipt["discarded_candidate_sha256"] == candidate_sha
    assert receipt["overlapping_memory_frame_index"] == 1
    assert staging.is_dir()
    loaded = EpisodicDatasetStore(tmp_path, minimum_frames=1).load("route")
    assert list(loaded.goal_candidates()) == []
    assert len(list(loaded.memory_frames())) == 3
    recovery = loaded.manifest["metadata"]["recovery"]
    assert recovery["overlapping_memory_frame"]["frame_index"] == 1
    assert (
        loaded.root
        / "recovery_discarded/goals"
        / f"candidate_000_{candidate_sha[:16]}.jpg"
    ).read_bytes() == b"candidate-and-frame-1"


def test_recovery_refuses_a_backup_that_no_longer_matches(tmp_path):
    staging, _ = _write_staging(tmp_path)
    backup = tmp_path / "backup"
    shutil.copytree(staging, backup)
    next((backup / "memory").glob("*.jpg")).write_bytes(b"changed")

    with pytest.raises(RecoveryError, match="byte-for-byte"):
        recover_one_way_debug_dataset(
            root=tmp_path,
            dataset_id="route",
            backup_root=backup,
            expected_memory_frames=3,
            created_utc="2026-08-30T14:22:36Z",
            external_goal_sha256=hashlib.sha256(b"external-M").hexdigest(),
        )
    assert not (tmp_path / "route").exists()
