import json

import pytest

from deployment.gpu.episodic_dataset import (
    DatasetContractError,
    EpisodicDatasetStore,
    SOURCE_OBSERVATION_SCHEMA,
)


def candidate(candidate_id=0, image=b"goal"):
    import hashlib

    return {
        "candidate_id": candidate_id,
        "captured_after_frame": 1,
        "sha256": hashlib.sha256(image).hexdigest(),
        "appended_to_memory": False,
        "registered": True,
        "capture_score": {
            "provisional_band": "provisional_weak_covis",
            "eligible_anchor_ceiling": 0,
        },
    }


def test_round_trip_is_canonical_and_keeps_depth_evaluator_only(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=2)
    store.start("hallway-out-back", metadata={"operator": "hand_controller"})
    first = store.append_memory(
        frame_index=0, image=b"memory-0", upstream_sha256=None
    )
    store.append_memory(frame_index=1, image=b"memory-1", upstream_sha256=None)
    store.append_candidate(
        record=candidate(image=b"goal"),
        image=b"goal",
        evaluation_depth=b"depth-png",
        evaluation_depth_scale_m=1.0e-3,
    )

    sealed = store.seal(protocol={"metric_depth_sensor_consumed": False})
    assert sealed["memory_frames"] == 2
    assert sealed["goal_candidates"] == 1
    assert sealed["goal_memory_exact_sha_overlap"] == 0
    assert first["frame_index"] == 0

    loaded = store.load("hallway-out-back")
    assert [payload for _, payload in loaded.memory_frames()] == [
        b"memory-0",
        b"memory-1",
    ]
    restored = list(loaded.goal_candidates())
    assert restored[0][1:] == (b"goal", b"depth-png")
    record = restored[0][0]
    assert record["evaluation_depth_policy_authority"] is False
    assert record["evaluation_depth_scale_m"] == 1.0e-3
    raw = (loaded.root / "manifest.json").read_bytes()
    assert raw.endswith(b"\n")
    assert json.loads(raw)["metadata"]["operator"] == "hand_controller"


def test_exact_goal_memory_jpeg_is_rejected_before_seal(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start("bad-self-match")
    store.append_memory(frame_index=0, image=b"same", upstream_sha256=None)
    with pytest.raises(DatasetContractError, match="exactly duplicates"):
        store.append_candidate(
            record=candidate(image=b"same"), image=b"same"
        )


def test_duplicate_memory_content_is_stored_once_but_replayed_twice(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=2)
    store.start("content-addressed")
    first = store.append_memory(
        frame_index=0, image=b"same", upstream_sha256=None
    )
    second = store.append_memory(
        frame_index=1, image=b"same", upstream_sha256=None
    )
    store.append_candidate(record=candidate(image=b"goal"), image=b"goal")
    receipt = store.seal(protocol={})

    assert first["path"] == second["path"]
    assert receipt["memory_frames"] == 2
    assert receipt["unique_memory_images"] == 1
    root = tmp_path / "content-addressed"
    assert len(list((root / "memory").glob("*.jpg"))) == 1
    assert [payload for _, payload in store.load("content-addressed").memory_frames()] == [
        b"same",
        b"same",
    ]


def test_memory_source_observation_binds_rgbd_stamps(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start(
        "timestamped",
        metadata={"source_observation_contract": SOURCE_OBSERVATION_SCHEMA},
    )
    record = store.append_memory(
        frame_index=0,
        image=b"memory",
        upstream_sha256=None,
        source_observation={
            "schema": SOURCE_OBSERVATION_SCHEMA,
            "rgb_stamp_ns": 123,
            "depth_stamp_ns": 124,
            "pair_received_ros_ns": 130,
        },
    )
    store.append_candidate(record=candidate(image=b"goal"), image=b"goal")
    receipt = store.seal(protocol={})

    assert receipt["source_observations"] == 1
    assert record["source_observation"]["rgb_stamp_ns"] == 123
    loaded_record, _ = next(store.load("timestamped").memory_frames())
    assert loaded_record["source_observation"]["depth_stamp_ns"] == 124


def test_required_memory_source_observation_cannot_be_omitted(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start(
        "missing-timestamp",
        metadata={"source_observation_contract": SOURCE_OBSERVATION_SCHEMA},
    )
    store.append_memory(frame_index=0, image=b"memory", upstream_sha256=None)
    store.append_candidate(record=candidate(image=b"goal"), image=b"goal")
    with pytest.raises(DatasetContractError, match="contract is incomplete"):
        store.seal(protocol={})


def test_short_or_candidate_free_dataset_cannot_be_sealed(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=2)
    store.start("short")
    store.append_memory(frame_index=0, image=b"m0", upstream_sha256=None)
    with pytest.raises(DatasetContractError, match="minimum is 2"):
        store.seal(protocol={})


def test_exact_one_way_external_goal_contract_allows_no_candidates(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start(
        "external-goal-debug",
        metadata={
            "collection_mode": "manual_one_way_external_goal_debug",
            "goal_selection_contract": "operator_frozen_external_required",
            "goal_candidates_required": False,
        },
    )
    store.append_memory(frame_index=0, image=b"m0", upstream_sha256=None)
    receipt = store.seal(protocol={})
    assert receipt["goal_candidates"] == 0
    assert list(store.load("external-goal-debug").goal_candidates()) == []


def test_one_way_external_goal_contract_rejects_survey_candidates(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start(
        "external-goal-with-candidate",
        metadata={
            "collection_mode": "manual_one_way_external_goal_debug",
            "goal_selection_contract": "operator_frozen_external_required",
            "goal_candidates_required": False,
        },
    )
    store.append_memory(frame_index=0, image=b"m0", upstream_sha256=None)
    store.append_candidate(record=candidate(), image=b"goal")
    with pytest.raises(DatasetContractError, match="external-goal-only"):
        store.seal(protocol={})


@pytest.mark.parametrize(
    "metadata",
    [
        {},
        {
            "collection_mode": "manual_one_way_external_goal_debug",
            "goal_selection_contract": "operator_frozen_external_required",
            "goal_candidates_required": True,
        },
        {
            "collection_mode": "manual_long_out_and_back",
            "goal_selection_contract": "operator_frozen_external_required",
            "goal_candidates_required": False,
        },
    ],
)
def test_candidate_free_exception_is_fail_closed(tmp_path, metadata):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start("strict", metadata=metadata)
    store.append_memory(frame_index=0, image=b"m0", upstream_sha256=None)
    with pytest.raises(DatasetContractError, match="no supported"):
        store.seal(protocol={})


def test_load_detects_artifact_tampering(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start("tamper")
    store.append_memory(frame_index=0, image=b"m0", upstream_sha256=None)
    store.append_candidate(record=candidate(), image=b"goal")
    store.seal(protocol={})
    memory_path = next((tmp_path / "tamper" / "memory").glob("*.jpg"))
    memory_path.write_bytes(b"changed")
    with pytest.raises(DatasetContractError, match="byte count changed|SHA-256 changed"):
        store.load("tamper")


def test_load_detects_manifest_receipt_tampering(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start("receipt-tamper")
    store.append_memory(frame_index=0, image=b"m0", upstream_sha256=None)
    store.append_candidate(record=candidate(), image=b"goal")
    store.seal(protocol={})
    (tmp_path / "receipt-tamper" / "MANIFEST.sha256").write_text(
        "0" * 64 + "  manifest.json\n",
        encoding="ascii",
    )
    with pytest.raises(DatasetContractError, match="MANIFEST.sha256"):
        store.load("receipt-tamper")


def test_dataset_ids_and_overwrite_are_fail_closed(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    with pytest.raises(DatasetContractError, match="dataset_id"):
        store.start("../escape")
    store.start("stable")
    store.append_memory(frame_index=0, image=b"m0", upstream_sha256=None)
    store.append_candidate(record=candidate(), image=b"goal")
    store.seal(protocol={})
    with pytest.raises(DatasetContractError, match="refusing overwrite"):
        store.start("stable")


def test_status_listing_is_lightweight_but_load_still_detects_tampering(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    store.start("listed")
    store.append_memory(frame_index=0, image=b"m0", upstream_sha256=None)
    store.append_candidate(record=candidate(), image=b"goal")
    store.seal(protocol={})

    memory_path = next((tmp_path / "listed" / "memory").glob("*.jpg"))
    memory_path.write_bytes(b"tampered-but-same-field-status")
    listed = store.list_sealed()
    assert listed[0]["dataset_id"] == "listed"
    assert listed[0]["artifacts_verified"] is False
    with pytest.raises(DatasetContractError, match="byte count changed|SHA-256 changed"):
        store.load("listed")
