#!/usr/bin/env python3
"""Compute the deterministic JPEG identity used for an ImageGoal wire upload.

The frozen source artifact may be PNG or JPEG.  NavDPClient decodes it to RGB
and encodes one quality-95 JPEG for the HTTP policy boundary.  Both identities
are meaningful: the source hash proves the operator-selected file did not
change, while the committed wire hash proves the exact bytes installed by CEC.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
from typing import Optional

from image_goal_io import load_rgb_image
from navdp_client import NavDPClient


def canonical_goal_wire_bytes(path: str | Path) -> bytes:
    return NavDPClient._encode_rgb(load_rgb_image(Path(path)))


def canonical_goal_wire_sha256(path: str | Path) -> str:
    return hashlib.sha256(canonical_goal_wire_bytes(path)).hexdigest()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sha256", required=True, type=Path, metavar="IMAGE")
    args = parser.parse_args(argv)
    if not args.sha256.is_file():
        parser.error(f"ImageGoal does not exist: {args.sha256}")
    print(canonical_goal_wire_sha256(args.sha256))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
