from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

from record_realworld_trial import parser  # noqa: E402


def run_command(arguments: list[str]):
    args = parser().parse_args(arguments)
    return args.function(args)


class RealworldTrialLedgerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.ledger = self.root / "ledger"
        self.capture = self.root / "captures"

    def tearDown(self):
        self.temporary.cleanup()

    def register(self):
        return run_command(
            [
                "--ledger", str(self.ledger), "register",
                "--scene", "small_room", "--leg2-type", "revisit",
                "--method", "ours", "--pair-index", "1",
            ]
        )

    def make_episode(self, registration):
        run_root = self.capture / "episode-test"
        run_root.mkdir(parents=True)
        manifest = {
            "run_id": "episode-test",
            "created_utc": registration["registered_utc"],
            "state": "finalized",
            "trial_kind": "revisit",
            "outcome": "success",
            "dataset_id": "m-test",
            "capture_profile": "full",
            "gt_source": "none",
            "completeness": {"formal_complete": True},
        }
        manifest_path = run_root / "manifest.json"
        manifest_path.write_text(json.dumps(manifest) + "\n", encoding="utf-8")
        digest = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
        (run_root / "MANIFEST.sha256").write_text(
            f"{digest}  manifest.json\n", encoding="ascii"
        )
        (run_root / "FINALIZED").write_text("\n", encoding="ascii")

    def test_registration_and_record_are_immutable_and_hash_bound(self):
        registration = self.register()
        self.make_episode(registration)
        record = run_command(
            [
                "--ledger", str(self.ledger), "record",
                "--capture-root", str(self.capture),
                "--success", "1", "--final-distance-m", "0.18",
                "--metric-source", "tape_measure",
            ]
        )
        self.assertEqual(record["registration"]["label"]["method"], "Ours")
        self.assertEqual(record["evidence"]["episode_id"], "episode-test")
        self.assertEqual(record["record_status"], "recorded_with_independent_metrics")
        self.assertTrue((self.ledger / "trials.csv").is_file())
        self.assertIsNone(run_command(
            ["--ledger", str(self.ledger), "status"]
        )["active_registration"])

    def test_pipeline_outcome_is_not_automatically_used_as_success(self):
        registration = self.register()
        self.make_episode(registration)
        record = run_command(
            [
                "--ledger", str(self.ledger), "record",
                "--capture-root", str(self.capture),
            ]
        )
        self.assertEqual(record["evidence"]["pipeline_outcome"], "success")
        self.assertIsNone(record["adjudication"]["success"])
        self.assertEqual(
            record["record_status"], "recorded_pending_independent_metrics"
        )


if __name__ == "__main__":
    unittest.main()
