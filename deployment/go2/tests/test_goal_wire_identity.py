import hashlib
from pathlib import Path
import sys

import cv2
import numpy as np


GO2 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GO2))

from goal_wire_identity import (  # noqa: E402
    canonical_goal_wire_bytes,
    canonical_goal_wire_sha256,
)
from image_goal_io import load_rgb_image  # noqa: E402
from navdp_client import NavDPClient  # noqa: E402


def test_source_and_committed_wire_identities_are_both_deterministic(tmp_path):
    rgb = np.zeros((48, 80, 3), dtype=np.uint8)
    rgb[:, :, 0] = np.arange(80, dtype=np.uint8)
    rgb[8:40, 20:60, 1] = 210
    ok, png = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    assert ok
    source = tmp_path / "goal.png"
    source.write_bytes(png.tobytes())

    first = canonical_goal_wire_bytes(source)
    second = canonical_goal_wire_bytes(source)

    assert first == second
    assert first == NavDPClient._encode_rgb(load_rgb_image(source))
    assert canonical_goal_wire_sha256(source) == hashlib.sha256(first).hexdigest()
    assert hashlib.sha256(source.read_bytes()).hexdigest() != hashlib.sha256(
        first
    ).hexdigest()
