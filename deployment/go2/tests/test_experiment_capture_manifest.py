from __future__ import annotations

import json
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from experiment_capture_manifest import (
    CaptureManifestError,
    attach_reference,
    attach_video,
    create_manifest,
    finalize_manifest,
    mark_captured,
    read_manifest,
    verify_manifest,
)
from experiment_topic_logger import ReceiptLogWriter


def create_capture(tmp_path: Path) -> Path:
    root = tmp_path / "formal-revisit-01"
    create_manifest(
        root,
        run_id="formal-revisit-01",
        dataset_id="survey-01",
        trial_kind="revisit",
        capture_profile="audit",
        topics=("/navdp/status", "/navdp/cec_receipt"),
        workspace=Path(__file__).resolve().parents[3],
    )
    return root


def populate_required_artifacts(root: Path, third_source: Path) -> None:
    (root / "rosbag").mkdir()
    (root / "rosbag" / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (root / "rosbag" / "capture_0.mcap").write_bytes(b"mcap evidence")
    dashboard_source = root.parent / "foxglove-dashboard.mp4"
    dashboard_source.write_bytes(b"foxglove dashboard h264")
    attach_video(root, role="foxglove_dashboard", source=dashboard_source)
    (root / "logs" / "status.jsonl").write_text('{"state":"ready"}\n')
    (root / "logs" / "cec_receipt.jsonl").write_text('{"takeover":true}\n')
    attach_video(root, role="third_view", source=third_source)


class ExperimentCaptureManifestTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.tmp_path = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def test_complete_capture_is_hash_sealed_and_verifiable(self):
        root = create_capture(self.tmp_path)
        third_source = self.tmp_path / "phone.mp4"
        third_source.write_bytes(b"external third view")
        populate_required_artifacts(root, third_source)
        mark_captured(root, clean=True)

        result = finalize_manifest(
            root,
            outcome="success",
            notes="operator confirmed",
            allow_incomplete=False,
        )

        self.assertEqual(result["state"], "finalized")
        self.assertIs(result["motion_authority_changed_by_capture"], False)
        self.assertIs(result["completeness"]["formal_complete"], True)
        verification = verify_manifest(root)
        self.assertEqual(verification["run_id"], "formal-revisit-01")
        self.assertGreaterEqual(verification["artifacts"], 7)

    def test_mutated_artifact_is_rejected_after_finalization(self):
        root = create_capture(self.tmp_path)
        third_source = self.tmp_path / "phone.mp4"
        third_source.write_bytes(b"external third view")
        populate_required_artifacts(root, third_source)
        mark_captured(root, clean=True)
        finalize_manifest(
            root,
            outcome="failure",
            notes="",
            allow_incomplete=False,
        )
        (root / "media" / "dashboard.mp4").write_bytes(b"changed")

        with self.assertRaisesRegex(CaptureManifestError, "inventory changed"):
            verify_manifest(root)

    def test_incomplete_capture_requires_explicit_override(self):
        root = create_capture(self.tmp_path)
        mark_captured(root, clean=False)

        with self.assertRaisesRegex(CaptureManifestError, "incomplete"):
            finalize_manifest(
                root,
                outcome="aborted",
                notes="screen recorder unavailable",
                allow_incomplete=False,
            )

        result = finalize_manifest(
            root,
            outcome="aborted",
            notes="screen recorder unavailable",
            allow_incomplete=True,
        )
        self.assertIs(result["completeness"]["formal_complete"], False)

    def test_attached_video_is_byte_preserved(self):
        root = create_capture(self.tmp_path)
        source = self.tmp_path / "external.mov"
        source.write_bytes(b"phone master bytes")

        receipt = attach_video(root, role="third_view", source=source)

        self.assertEqual((root / receipt["path"]).read_bytes(), source.read_bytes())
        self.assertEqual(receipt["bytes"], len(source.read_bytes()))

    def test_foxglove_dashboard_is_attachable(self):
        root = create_capture(self.tmp_path)
        source = self.tmp_path / "foxglove.mkv"
        source.write_bytes(b"foxglove dashboard master bytes")

        receipt = attach_video(root, role="foxglove_dashboard", source=source)

        self.assertEqual(receipt["path"], "media/dashboard.mkv")
        self.assertEqual((root / receipt["path"]).read_bytes(), source.read_bytes())

    def test_receipt_logger_parses_json_and_preserves_non_json(self):
        writer = ReceiptLogWriter(self.tmp_path)
        writer.append("/navdp/status", '{"phase":"revisit_query"}')
        writer.append("/navdp/cec_receipt", "not-json")
        writer.close()

        status = json.loads(
            (self.tmp_path / "status.jsonl").read_text().splitlines()[0]
        )
        receipt = json.loads(
            (self.tmp_path / "cec_receipt.jsonl").read_text().splitlines()[0]
        )
        self.assertEqual(status["payload"]["phase"], "revisit_query")
        self.assertEqual(receipt["payload"], {"raw": "not-json"})

    def test_capture_state_is_not_reopened(self):
        root = create_capture(self.tmp_path)
        mark_captured(root, clean=True)

        with self.assertRaisesRegex(CaptureManifestError, "only a recording"):
            mark_captured(root, clean=True)
        self.assertIs(read_manifest(root)["capture_stop_clean"], True)

    def test_calibration_requires_preobservation_physical_label(self):
        root = self.tmp_path / "calibration-missing-label"

        with self.assertRaisesRegex(
            CaptureManifestError, "calibration trials require"
        ):
            create_manifest(
                root,
                run_id="calibration-missing-label",
                dataset_id="heldout-calibration",
                trial_kind="calibration",
                capture_profile="audit",
                topics=("/navdp/rgb_arrival_status",),
                workspace=Path(__file__).resolve().parents[3],
            )

        self.assertFalse(root.exists())

    def test_calibration_label_is_frozen_before_capture(self):
        root = self.tmp_path / "calibration-labelled"
        payload = create_manifest(
            root,
            run_id="calibration-labelled",
            dataset_id="heldout-calibration",
            trial_kind="calibration",
            capture_profile="audit",
            topics=("/navdp/rgb_arrival_status",),
            workspace=Path(__file__).resolve().parents[3],
            calibration_scene_id="calibration-a",
            physical_distance_m=0.50,
            physical_yaw_deg=-20.0,
            physical_label_method="tape-and-angle-jig",
        )

        label = payload["calibration_label"]
        self.assertEqual(label["scene_id"], "calibration-a")
        self.assertEqual(label["physical_distance_m"], 0.50)
        self.assertEqual(label["physical_yaw_deg"], -20.0)
        self.assertEqual(label["measurement_method"], "tape-and-angle-jig")
        self.assertIs(label["recorded_before_capture"], True)
        self.assertIs(label["arrival_score_logging_started_before_label"], False)
        self.assertEqual(label["label_recorded_utc"], payload["created_utc"])

    def test_noncalibration_trial_rejects_physical_labels(self):
        with self.assertRaisesRegex(
            CaptureManifestError, "only valid for calibration"
        ):
            create_manifest(
                self.tmp_path / "revisit-with-calibration-label",
                run_id="revisit-with-calibration-label",
                dataset_id="survey-01",
                trial_kind="revisit",
                capture_profile="audit",
                topics=("/navdp/status",),
                workspace=Path(__file__).resolve().parents[3],
                calibration_scene_id="calibration-a",
                physical_distance_m=0.25,
                physical_yaw_deg=0.0,
                physical_label_method="tape-and-angle-jig",
            )

    def test_odin_gt_capture_requires_status_result_and_spl_receipts(self):
        root = self.tmp_path / "formal-odin-01"
        create_manifest(
            root,
            run_id="formal-odin-01",
            dataset_id="survey-01",
            trial_kind="revisit",
            capture_profile="audit",
            topics=("/navdp/status", "/navdp/cec_receipt", "/navdp/gt/status"),
            workspace=Path(__file__).resolve().parents[3],
            gt_source="odin1",
        )
        third_source = self.tmp_path / "phone-odin.mp4"
        third_source.write_bytes(b"external odin third view")
        populate_required_artifacts(root, third_source)
        (root / "logs" / "odin_gt_status.jsonl").write_text(
            '{"reference_ready":true}\n'
        )
        result = self.tmp_path / "result.json"
        result.write_text(
            json.dumps(
                {
                    "schema": "memnav-odin1-gt-result-v1",
                    "run_id": "formal-odin-01",
                    "success": True,
                }
            )
        )
        spl = self.tmp_path / "spl.json"
        spl.write_text(
            json.dumps(
                {
                    "schema": "memnav-odin1-spl-receipt-v1",
                    "run_id": "formal-odin-01",
                    "inputs": {
                        "gt_result": {
                            "sha256": hashlib.sha256(result.read_bytes()).hexdigest()
                        }
                    },
                    "metrics": {"S_i": 1},
                }
            )
        )
        attach_reference(root, role="odin_gt_result", source=result)
        attach_reference(root, role="odin_spl_receipt", source=spl)
        mark_captured(root, clean=True)

        finalized = finalize_manifest(
            root,
            outcome="success",
            notes="independent Odin reference lane",
            allow_incomplete=False,
        )

        self.assertEqual(finalized["gt_source"], "odin1")
        self.assertIs(finalized["completeness"]["odin_gt_status"], True)
        self.assertIs(finalized["completeness"]["odin_gt_result"], True)
        self.assertIs(finalized["completeness"]["odin_spl_receipt"], True)

    def test_odin_spl_must_match_run_and_attached_result(self):
        root = self.tmp_path / "formal-odin-02"
        create_manifest(
            root,
            run_id="formal-odin-02",
            dataset_id="survey-01",
            trial_kind="revisit",
            capture_profile="audit",
            topics=("/navdp/gt/status",),
            workspace=Path(__file__).resolve().parents[3],
            gt_source="odin1",
        )
        result = self.tmp_path / "result-02.json"
        result.write_text(
            json.dumps(
                {
                    "schema": "memnav-odin1-gt-result-v1",
                    "run_id": "formal-odin-02",
                    "success": False,
                }
            )
        )
        attach_reference(root, role="odin_gt_result", source=result)
        wrong_spl = self.tmp_path / "wrong-spl.json"
        wrong_spl.write_text(
            json.dumps(
                {
                    "schema": "memnav-odin1-spl-receipt-v1",
                    "run_id": "formal-odin-02",
                    "inputs": {"gt_result": {"sha256": "wrong"}},
                    "metrics": {"S_i": 0},
                }
            )
        )
        with self.assertRaisesRegex(CaptureManifestError, "hash-bound"):
            attach_reference(root, role="odin_spl_receipt", source=wrong_spl)
