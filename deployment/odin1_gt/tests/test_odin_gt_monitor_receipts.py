from __future__ import annotations

import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from odin_gt_monitor import driver_profile_semantics, verified_receipt_path


class MonitorReceiptTests(unittest.TestCase):
    def test_verified_receipt_rejects_mutation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "artifact.bin"
            path.write_bytes(b"original")
            receipt = {
                "path": str(path),
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            self.assertEqual(verified_receipt_path(receipt, "artifact"), path)
            path.write_bytes(b"mutated!")
            with self.assertRaisesRegex(ValueError, "SHA"):
                verified_receipt_path(receipt, "artifact")

    def test_driver_semantics_ignore_receipt_time_and_paths(self) -> None:
        first = {
            "profile": "native_0_14",
            "repository": "official",
            "commit": "abc",
            "tag": "v0.14.0",
            "firmware_contract": "0.14.x_native_mode1",
            "native_mode1": True,
            "created_utc": "first",
            "patches": [{"name": "fix.patch", "path": "/a", "sha256": "123"}],
            "modified_files": {"source.cpp": "456"},
        }
        second = {
            **first,
            "created_utc": "second",
            "patches": [{"name": "fix.patch", "path": "/b", "sha256": "123"}],
        }
        self.assertEqual(
            driver_profile_semantics(first), driver_profile_semantics(second)
        )


if __name__ == "__main__":
    unittest.main()
