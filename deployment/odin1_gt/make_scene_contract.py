#!/usr/bin/env python3
"""Seal the Odin sensor, firmware, calibration, mount and driver contract."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
from typing import Any

from odin_gt_core import sha256_file


SCHEMA = "memnav-odin1-scene-contract-v1"
MOUNT_SCHEMA = "memnav-odin1-go2-mount-v1"
DRIVER_SCHEMA = "memnav-odin1-driver-profile-v1"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def load_object(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def file_receipt(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve()
    if not path.is_file() or path.stat().st_size <= 0:
        raise ValueError(f"required artifact is missing or empty: {path}")
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def validate_rigid_transform(transform: Any) -> None:
    if (
        not isinstance(transform, list)
        or len(transform) != 4
        or any(not isinstance(row, list) or len(row) != 4 for row in transform)
    ):
        raise ValueError("mount receipt requires a row-major 4x4 T_go2base_odin")
    try:
        matrix = [[float(value) for value in row] for row in transform]
    except (TypeError, ValueError) as error:
        raise ValueError("mount transform contains non-numeric values") from error
    if not all(math.isfinite(value) for row in matrix for value in row):
        raise ValueError("mount transform contains non-finite values")
    expected_last_row = (0.0, 0.0, 0.0, 1.0)
    if any(
        abs(matrix[3][index] - expected) > 1e-6
        for index, expected in enumerate(expected_last_row)
    ):
        raise ValueError("mount transform last row must be [0, 0, 0, 1]")
    rotation = [row[:3] for row in matrix[:3]]
    for first in range(3):
        for second in range(3):
            dot = sum(
                rotation[first][axis] * rotation[second][axis]
                for axis in range(3)
            )
            expected = 1.0 if first == second else 0.0
            if abs(dot - expected) > 0.02:
                raise ValueError("mount rotation is not orthonormal within 0.02")
    determinant = (
        rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 0.02:
        raise ValueError("mount rotation determinant is not +1 within 0.02")


def build_contract(
    mapping_session_id: str,
    sensor_serial: str,
    firmware_version: str,
    calibration_path: Path,
    mount_path: Path,
    driver_path: Path,
) -> dict[str, Any]:
    calibration = file_receipt(calibration_path)
    mount_receipt = file_receipt(mount_path)
    driver_receipt = file_receipt(driver_path)
    mount = load_object(Path(mount_receipt["path"]))
    if mount.get("schema") != MOUNT_SCHEMA:
        raise ValueError("unsupported Odin-to-Go2 mount receipt")
    if mount.get("validated") is not True:
        raise ValueError("mount receipt must be independently validated")
    if mount.get("sensor_serial") != sensor_serial:
        raise ValueError("mount receipt sensor serial does not match")
    for key in ("rigid_mount_id", "measurement_method", "validation_evidence"):
        if not mount.get(key):
            raise ValueError(f"mount receipt is missing {key}")
    validate_rigid_transform(mount.get("T_go2base_odin"))
    driver = load_object(Path(driver_receipt["path"]))
    if driver.get("schema") != DRIVER_SCHEMA:
        raise ValueError("unsupported Odin driver profile receipt")
    profile = driver.get("profile")
    if profile == "native_0_14":
        if re.fullmatch(r"0\.14(?:[.-].*)?", firmware_version) is None:
            raise ValueError("native_0_14 requires an exact 0.14 firmware version")
    elif profile == "legacy_0_13_1":
        if firmware_version != "0.13.1":
            raise ValueError("legacy_0_13_1 requires firmware 0.13.1")
    else:
        raise ValueError(f"unsupported driver profile: {profile}")
    return {
        "schema": SCHEMA,
        "created_utc": utc_now(),
        "mapping_session_id": mapping_session_id,
        "sensor_serial": sensor_serial,
        "firmware_version": firmware_version,
        "calibration": calibration,
        "mount": {**mount_receipt, "validated": True},
        "driver_profile": driver_receipt,
        "classification": "independent_reference_sensor_contract",
        "policy_input": False,
        "motion_authority": False,
    }


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping-session-id", required=True)
    parser.add_argument("--sensor-serial", required=True)
    parser.add_argument("--firmware-version", required=True)
    parser.add_argument("--calibration-file", type=Path, required=True)
    parser.add_argument("--mount-receipt", type=Path, required=True)
    parser.add_argument("--driver-profile-receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = build_contract(
        mapping_session_id=args.mapping_session_id,
        sensor_serial=args.sensor_serial,
        firmware_version=args.firmware_version,
        calibration_path=args.calibration_file,
        mount_path=args.mount_receipt,
        driver_path=args.driver_profile_receipt,
    )
    atomic_write_json(args.output, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
