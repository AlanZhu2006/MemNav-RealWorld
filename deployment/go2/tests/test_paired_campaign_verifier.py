from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "deployment" / "go2"))

from experiment_capture_manifest import (  # noqa: E402
    attach_reference,
    attach_video,
    atomic_write_json,
    create_manifest,
    finalize_manifest,
    mark_captured,
)
from verify_realworld_paired_campaign import (  # noqa: E402
    _aggregate,
    _authority_audit,
    _exact_mcnemar,
    _validate_plan,
    _verify_run,
)


def _current_plan() -> dict:
    return json.loads(
        (ROOT / "manifests" / "realworld_paired_evaluation_plan_v2.json")
        .read_text(encoding="utf-8")
    )


def test_registered_plan_is_structurally_valid_but_not_scene_frozen():
    errors, blockers, runs = _validate_plan(_current_plan())

    assert errors == []
    assert len(runs) == 40
    assert "scene01: role is not frozen" in blockers
    assert "scene04: shortest_feasible_path_m is not frozen" in blockers


def test_preregistration_rejects_posthoc_result_edits():
    plan = _current_plan()
    plan["scenes"][0]["paired_blocks"][0]["runs"]["mono_cec"]["success"] = True

    errors, _, _ = _validate_plan(plan)

    assert any("preregistered field success was modified" in row for row in errors)


def test_exact_mcnemar_matches_small_registered_cases():
    assert _exact_mcnemar(0, 0) == 1.0
    assert _exact_mcnemar(6, 0) == 0.03125
    assert _exact_mcnemar(12, 0) == 0.00048828125
    assert _exact_mcnemar(9, 3) == 0.14599609375


def test_authority_audit_rejects_native_takeover(tmp_path: Path):
    logs = tmp_path / "logs"
    logs.mkdir()
    (logs / "cec_receipt.jsonl").write_text(
        json.dumps({
            "payload": {
                "receipt": {
                    "cec_authority_mode": "native",
                    "cec_takeover": True,
                }
            }
        }) + "\n",
        encoding="utf-8",
    )

    seen, takeover, errors = _authority_audit(tmp_path, "mono_native")

    assert seen is True
    assert takeover is True
    assert errors == ["native arm contains a CEC takeover"]


def _populate_formal_run(
    root: Path,
    *,
    run_id: str,
    dataset_id: str,
    dataset_sha: str,
    goal_sha: str,
    authority_mode: str,
    success: bool,
) -> None:
    create_manifest(
        root,
        run_id=run_id,
        dataset_id=dataset_id,
        trial_kind="revisit",
        capture_profile="audit",
        topics=("/navdp/status", "/navdp/cec_receipt", "/navdp/gt/status"),
        workspace=ROOT,
        gt_source="odin1",
    )
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["repository"]["tracked_files_dirty"] = False
    atomic_write_json(manifest_path, manifest)

    (root / "rosbag").mkdir()
    (root / "rosbag" / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (root / "rosbag" / "capture_0.db3").write_bytes(b"paired evidence")
    (root / "media" / "dashboard.mp4").write_bytes(b"dashboard")
    third = root.parent / f"{run_id}.mp4"
    third.write_bytes(b"third view")
    attach_video(root, role="third_view", source=third)
    (root / "logs" / "status.jsonl").write_text(
        json.dumps({
            "payload": {
                "active_goal_sha256": goal_sha,
                "episodic_dataset": {
                    "loaded_dataset_id": dataset_id,
                    "loaded_dataset_manifest_sha256": dataset_sha,
                },
            }
        }) + "\n",
        encoding="utf-8",
    )
    (root / "logs" / "cec_receipt.jsonl").write_text(
        json.dumps({
            "payload": {
                "receipt": {
                    "cec_authority_mode": authority_mode,
                    "cec_takeover": authority_mode == "cec",
                }
            }
        }) + "\n",
        encoding="utf-8",
    )
    (root / "logs" / "odin_gt_status.jsonl").write_text(
        '{"reference_ready":true}\n', encoding="utf-8"
    )
    gt_result = root.parent / f"{run_id}.gt.json"
    gt_result.write_text(json.dumps({
        "schema": "memnav-odin1-gt-result-v1",
        "run_id": run_id,
        "success": success,
    }), encoding="utf-8")
    path_m = 2.5 if success else 4.0
    spl = 2.0 / max(2.0, path_m) if success else 0.0
    spl_receipt = root.parent / f"{run_id}.spl.json"
    spl_receipt.write_text(json.dumps({
        "schema": "memnav-odin1-spl-receipt-v1",
        "run_id": run_id,
        "inputs": {
            "gt_result": {
                "sha256": hashlib.sha256(gt_result.read_bytes()).hexdigest()
            }
        },
        "metrics": {
            "S_i": int(success),
            "L_i_m": 2.0,
            "P_i_m": path_m,
            "SPL_i": spl,
        },
    }), encoding="utf-8")
    attach_reference(root, role="odin_gt_result", source=gt_result)
    attach_reference(root, role="odin_spl_receipt", source=spl_receipt)
    mark_captured(root, clean=True)
    finalize_manifest(
        root,
        outcome="success" if success else "failure",
        notes="synthetic verifier fixture",
        allow_incomplete=False,
    )


def test_one_finalized_run_is_independently_reproduced():
    with tempfile.TemporaryDirectory() as temporary:
        base = Path(temporary)
        run_id = "scene03_pair02_cec"
        root = base / run_id
        dataset_sha = "a" * 64
        goal_sha = "b" * 64
        _populate_formal_run(
            root,
            run_id=run_id,
            dataset_id="survey03",
            dataset_sha=dataset_sha,
            goal_sha=goal_sha,
            authority_mode="cec",
            success=True,
        )
        scene = {
            "dataset_id": "survey03",
            "dataset_manifest_sha256": dataset_sha,
            "goal_sha256": goal_sha,
            "shortest_feasible_path_m": 2.0,
        }
        row = _verify_run({
            "scene_id": "scene03",
            "role": "Revisit",
            "scene": scene,
            "pair_index": 2,
            "order": ["mono_cec", "mono_native"],
            "arm": "mono_cec",
            "run_id": run_id,
        }, root)

        assert row["success"] == 1
        assert row["SPL_i"] == 0.8
        assert row["cec_takeover"] is True
        assert len(row["capture_manifest_sha256"]) == 64


def test_aggregate_is_paired_by_scene_and_block_not_row_order():
    rows = [
        {"scene_id": "s1", "pair_index": 1, "arm": "mono_cec", "role": "Novel", "success": 1, "SPL_i": 0.7, "P_i_m": 3.0, "cec_takeover": False},
        {"scene_id": "s2", "pair_index": 1, "arm": "mono_native", "role": "Revisit", "success": 1, "SPL_i": 0.8, "P_i_m": 2.5, "cec_takeover": False},
        {"scene_id": "s1", "pair_index": 1, "arm": "mono_native", "role": "Novel", "success": 0, "SPL_i": 0.0, "P_i_m": 5.0, "cec_takeover": False},
        {"scene_id": "s2", "pair_index": 1, "arm": "mono_cec", "role": "Revisit", "success": 0, "SPL_i": 0.0, "P_i_m": 5.0, "cec_takeover": True},
    ]

    summary = _aggregate(rows)

    assert summary["paired"]["gains"] == 1
    assert summary["paired"]["losses"] == 1
    assert summary["paired"]["two_sided_exact_mcnemar_p"] == 1.0
    assert summary["arms"]["mono_cec"]["success"] == 1
