#!/usr/bin/env python3
"""Generate immutable Odin1 mapping or relocalization driver configurations."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from typing import Any

import yaml

from odin_gt_core import sha256_file


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_write(path: Path, data: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, required=True)
    parser.add_argument("--mode", choices=("mapping", "relocalization"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--map-file", type=Path)
    parser.add_argument("--mapping-output-dir", type=Path)
    parser.add_argument("--mapping-output-name", default="odin_map.bin")
    args = parser.parse_args()

    base_config = args.base_config.expanduser().resolve()
    payload: dict[str, Any] = yaml.safe_load(base_config.read_text(encoding="utf-8"))
    register = payload.get("register_keys")
    if not isinstance(register, dict):
        raise ValueError("Odin base config has no register_keys mapping")
    register.update(
        {
            "strict_usb3.0_check": 1,
            "use_host_ros_time": 0,
            "streamctrl": 1,
            "sendrgbcompressed": 1,
            "sendrgb": 1,
            "sendrgbundistort": 0,
            "sendimu": 1,
            "sendodom": 1,
            "send_odom_baselink_tf": 1,
            "senddtof": 1,
            "sendcloudslam": 1,
            "sendcloudrender": 0,
            "senddepth": 0,
            "sendreprojection": 0,
            "sendoverlay": 0,
            "recorddata": 0,
            "devstatuslog": 1,
            "showpath": 0,
            "showcamerapose": 0,
            "resetalgo": 0,
        }
    )
    if args.mode == "mapping":
        if args.map_file is not None:
            raise ValueError("--map-file is invalid in mapping mode")
        if args.mapping_output_dir is None:
            raise ValueError("mapping mode requires --mapping-output-dir")
        mapping_dir = args.mapping_output_dir.expanduser().resolve()
        mapping_dir.mkdir(parents=True, exist_ok=True)
        if "/" in args.mapping_output_name or not args.mapping_output_name.endswith(".bin"):
            raise ValueError("mapping output name must be a plain .bin filename")
        register.update(
            {
                "custom_map_mode": 1,
                "relocalization_map_abs_path": "",
                "mapping_result_dest_dir": str(mapping_dir),
                "mapping_result_file_name": args.mapping_output_name,
            }
        )
    else:
        if args.mapping_output_dir is not None:
            raise ValueError("--mapping-output-dir is invalid in relocalization mode")
        if args.map_file is None:
            raise ValueError("relocalization mode requires --map-file")
        map_file = args.map_file.expanduser().resolve()
        if not map_file.is_file() or map_file.stat().st_size <= 0:
            raise ValueError(f"relocalization map is missing or empty: {map_file}")
        register.update(
            {
                "custom_map_mode": 2,
                "relocalization_map_abs_path": str(map_file),
                "mapping_result_dest_dir": "",
                "mapping_result_file_name": "",
            }
        )
    output = args.output.expanduser().resolve()
    rendered = yaml.safe_dump(payload, sort_keys=False)
    atomic_write(output, rendered)
    receipt = {
        "schema": "memnav-odin1-driver-config-v1",
        "created_utc": utc_now(),
        "mode": args.mode,
        "base_config": {
            "path": str(base_config),
            "sha256": sha256_file(base_config),
        },
        "generated_config": {
            "path": str(output),
            "sha256": sha256_file(output),
        },
        "relocalization_map": (
            None
            if args.map_file is None
            else {
                "path": str(args.map_file.expanduser().resolve()),
                "sha256": sha256_file(args.map_file.expanduser().resolve()),
            }
        ),
        "motion_authority": False,
    }
    receipt_path = output.with_suffix(output.suffix + ".receipt.json")
    atomic_write(receipt_path, json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
