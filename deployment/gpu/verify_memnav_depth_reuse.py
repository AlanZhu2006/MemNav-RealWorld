"""Verify the legacy patch or an explicitly reviewed GEM source layout.

This is a read-only source-integrity gate, not a GPU behavior test. The GEM
receipt pins the writer, readout, agent wiring and HTTP receipts together;
unknown edits fail closed and require review instead of an automatic re-pin.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


GPU_ROOT = Path(__file__).resolve().parent


def verify(source_root: Path) -> str:
    patch = GPU_ROOT / "patches/memnav_reuse_flow_depth.patch"
    if patch.is_file():
        legacy = subprocess.run(
            ["git", "-C", str(source_root), "apply", "--reverse", "--check", str(patch)],
            capture_output=True,
            text=True,
        )
        if legacy.returncode == 0:
            return "legacy applied patch"

    receipt_path = GPU_ROOT / "patches/memnav_depth_reuse_gem.json"
    receipt = json.loads(receipt_path.read_text())
    if receipt["schema"] != "memnav_depth_reuse_source_receipt_v1":
        raise ValueError("unsupported GEM source receipt")
    mismatches = []
    for relative, expected in receipt["files_sha256"].items():
        path = source_root / relative
        if not path.is_file():
            mismatches.append(f"missing {relative}")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            mismatches.append(f"changed {relative}")
    if mismatches:
        raise ValueError(
            "neither the legacy patch nor the reviewed GEM implementation matches: "
            + "; ".join(mismatches)
            + ". Review the external source; do not bypass the gate or regenerate hashes blindly."
        )
    return receipt["implementation"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True, type=Path)
    args = parser.parse_args()
    try:
        implementation = verify(args.source_root.resolve())
    except (OSError, ValueError, KeyError) as exc:
        print(f"MemNav depth reuse verification failed: {exc}")
        return 1
    print(f"MemNav current-frame depth reuse verified: {implementation}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
