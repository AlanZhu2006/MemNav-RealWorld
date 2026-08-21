#!/usr/bin/env python3
"""Score real-world revisit goal candidates against the recorded memory.

The simulator's role-pair construction selects revisit goals whose
ground-truth covisibility with the recorded history falls in a frozen band
(standard: max covis in [0.55, 0.90]; hard: [0.25, 0.55)).  A real robot has
no ground-truth covisibility, so this tool computes the deployable proxy
using only the already-frozen server components:

1. a strided stateless DINO cosine sweep of each candidate against the
   recorded frames on disk (``/imagegoal_similarity``), giving
   ``max_cos`` / ``argmax_idx`` / ``argmax_gap_from_end``;
2. a LightGlue geometric verification of the candidate against the argmax
   history frame (``/retrieval_verify``), giving matches / inliers /
   inlier ratio without mutating any server state.

The proxy thresholds are NOT the simulator's ground-truth-covis numbers and
must be calibrated on the first camera-only disabled-adapter walk before any
band label from this tool is treated as final.  Until then the report is
descriptive: it tells the operator which candidate has bounded-but-real
support, and flags candidates that look either unsupported (no geometric
inliers) or trivially strong (near-duplicate of a stored frame).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

import requests


FRAME_NAME = re.compile(r"^(\d+)\.jpg$")

# Provisional proxy bands, pending disabled-adapter calibration.  The upper
# cosine bound guards against near-duplicate goals (trivial for native
# ImageGoal); the lower inlier bound guards against unsupported goals.
PROVISIONAL_MAX_COS_UPPER = 0.90
PROVISIONAL_MIN_INLIERS = 16


def list_recorded_frames(rgb_dir: Path) -> list[tuple[int, Path]]:
    frames = []
    for path in rgb_dir.iterdir():
        match = FRAME_NAME.match(path.name)
        if match:
            frames.append((int(match.group(1)), path))
    frames.sort()
    if not frames:
        raise SystemExit(f"no recorded {{idx}}.jpg frames under {rgb_dir}")
    return frames


def dino_cosine(session, memnav_url, image_bytes, goal_bytes, timeout):
    response = session.post(
        f"{memnav_url}/imagegoal_similarity",
        files={
            "image": ("image.jpg", image_bytes, "image/jpeg"),
            "goal": ("goal.jpg", goal_bytes, "image/jpeg"),
        },
        timeout=timeout,
    )
    response.raise_for_status()
    return float(response.json()["current_goal_cos"])


def classify(max_cos: float, inliers: int) -> str:
    if inliers < PROVISIONAL_MIN_INLIERS:
        return "reject_unsupported"
    if max_cos > PROVISIONAL_MAX_COS_UPPER:
        return "reject_near_duplicate"
    return "provisional_weak_covis"


def score_candidate(
    session, memnav_url, candidate_path, frames, stride, timeout,
):
    goal_bytes = candidate_path.read_bytes()
    swept = frames[::stride]
    if frames[-1] not in swept:
        swept = swept + [frames[-1]]
    best_cos, best_idx = -1.0, None
    sweep = []
    for idx, path in swept:
        value = dino_cosine(
            session, memnav_url, path.read_bytes(), goal_bytes, timeout)
        sweep.append({"frame_idx": idx, "cos": value})
        if value > best_cos:
            best_cos, best_idx = value, idx
    verify = session.post(
        f"{memnav_url}/retrieval_verify",
        files={"goal": ("goal.jpg", goal_bytes, "image/jpeg")},
        data={"anchor": str(best_idx)},
        timeout=timeout,
    )
    verify.raise_for_status()
    overlap = verify.json()
    inliers = int(overlap.get("inliers", 0))
    last_idx = frames[-1][0]
    return {
        "candidate": str(candidate_path),
        "max_cos": best_cos,
        "argmax_idx": best_idx,
        "argmax_gap_from_end": last_idx - best_idx,
        "frames_swept": len(swept),
        "frames_total": len(frames),
        "stride": stride,
        "lightglue": {
            "anchor": best_idx,
            "matches": overlap.get("matches"),
            "inliers": inliers,
            "inlier_ratio": overlap.get("inlier_ratio"),
        },
        "provisional_band": classify(best_cos, inliers),
        "dino_sweep": sweep,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--memnav-url", default="http://127.0.0.1:18888")
    parser.add_argument(
        "--rgb-dir", required=True,
        help="MemNav buffer episode dir holding the recorded {idx}.jpg frames")
    parser.add_argument(
        "--candidates", nargs="+", required=True,
        help="goal-candidate jpg files captured via the hub /goal_candidate")
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--out", default=None, help="write the JSON report here")
    args = parser.parse_args()

    frames = list_recorded_frames(Path(args.rgb_dir))
    session = requests.Session()
    memnav_url = args.memnav_url.rstrip("/")
    report = {
        "schema": "realworld_revisit_goal_score_v1_20260821",
        "provisional_thresholds": {
            "max_cos_upper": PROVISIONAL_MAX_COS_UPPER,
            "min_inliers": PROVISIONAL_MIN_INLIERS,
            "calibration_status": "pending_disabled_adapter_walk",
        },
        "simulator_reference_bands_gt_covis": {
            "standard": "[0.55, 0.90] with argmax gap <= 24",
            "hard": "[0.25, 0.55) with argmax gap <= 32",
            "note": "ground-truth covis bands; NOT directly comparable to "
                    "the DINO/LightGlue proxy reported here",
        },
        "candidates": [
            score_candidate(
                session, memnav_url, Path(path), frames,
                max(1, args.stride), (3.0, args.timeout_s),
            )
            for path in args.candidates
        ],
    }
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.out:
        Path(args.out).write_text(text + "\n")
    summary = [
        (entry["candidate"], entry["provisional_band"],
         round(entry["max_cos"], 3), entry["lightglue"]["inliers"])
        for entry in report["candidates"]
    ]
    for candidate, band, cos, inliers in summary:
        print(f"{band:26s} max_cos={cos:.3f} inliers={inliers} {candidate}",
              file=sys.stderr)
    if not args.out:
        print(text)


if __name__ == "__main__":
    main()
