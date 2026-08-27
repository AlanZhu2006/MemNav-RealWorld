from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from experiment_capture_manifest import (
    CaptureManifestError,
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
    (root / "rosbag" / "capture_0.db3").write_bytes(b"sqlite evidence")
    (root / "media" / "dashboard.mp4").write_bytes(b"dashboard h264")
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
