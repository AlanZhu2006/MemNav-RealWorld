from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from make_scene_contract import build_contract


class SceneContractTests(unittest.TestCase):
    def write_json(self, root: Path, name: str, payload: dict) -> Path:
        path = root / name
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def fixtures(self, root: Path) -> tuple[Path, Path, Path]:
        calibration = root / "calib.yaml"
        calibration.write_text("camera: verified\n", encoding="utf-8")
        mount = self.write_json(
            root,
            "mount.json",
            {
                "schema": "memnav-odin1-go2-mount-v1",
                "validated": True,
                "sensor_serial": "ODIN-TEST-001",
                "rigid_mount_id": "go2-front-v1",
                "measurement_method": "measured fixture and holdout check",
                "validation_evidence": "receipt://holdout-001",
                "T_go2base_odin": [
                    [1, 0, 0, 0.2],
                    [0, 1, 0, 0.0],
                    [0, 0, 1, 0.4],
                    [0, 0, 0, 1],
                ],
            },
        )
        driver = self.write_json(
            root,
            "driver.json",
            {
                "schema": "memnav-odin1-driver-profile-v1",
                "profile": "native_0_14",
            },
        )
        return calibration, mount, driver

    def test_native_014_contract_hashes_all_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration, mount, driver = self.fixtures(root)
            payload = build_contract(
                "scene01",
                "ODIN-TEST-001",
                "0.14.0",
                calibration,
                mount,
                driver,
            )
            self.assertEqual(payload["schema"], "memnav-odin1-scene-contract-v1")
            self.assertEqual(payload["sensor_serial"], "ODIN-TEST-001")
            self.assertEqual(len(payload["calibration"]["sha256"]), 64)
            self.assertEqual(len(payload["mount"]["sha256"]), 64)
            self.assertFalse(payload["motion_authority"])

    def test_serial_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration, mount, driver = self.fixtures(root)
            with self.assertRaisesRegex(ValueError, "serial"):
                build_contract(
                    "scene01",
                    "ODIN-WRONG",
                    "0.14.0",
                    calibration,
                    mount,
                    driver,
                )

    def test_nonrigid_mount_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration, mount, driver = self.fixtures(root)
            payload = json.loads(mount.read_text(encoding="utf-8"))
            payload["T_go2base_odin"][0][0] = 2.0
            mount.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "orthonormal"):
                build_contract(
                    "scene01",
                    "ODIN-TEST-001",
                    "0.14.0",
                    calibration,
                    mount,
                    driver,
                )

    def test_native_profile_rejects_legacy_firmware(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration, mount, driver = self.fixtures(root)
            with self.assertRaisesRegex(ValueError, "0.14"):
                build_contract(
                    "scene01",
                    "ODIN-TEST-001",
                    "0.13.1",
                    calibration,
                    mount,
                    driver,
                )


if __name__ == "__main__":
    unittest.main()
