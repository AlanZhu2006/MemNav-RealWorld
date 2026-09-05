#!/usr/bin/env python3
"""Collect scale-free ImageGoal convergence evidence from recorded RGBs.

This is deliberately an audit, not a STOP policy.  A strong fundamental
matrix or PnP certificate proves covisibility and a direction, but does not
prove that the camera has reached the goal pose.  This collector measures the
missing near-identity evidence directly in image coordinates:

* robust match support and spatial coverage;
* raw normalized correspondence displacement;
* near-identity affine and homography warp residuals;
* warp-conditioned grayscale disagreement.

It consumes only immutable JPEGs and never connects to ROS, the camera, or the
robot.  No arrival threshold is selected here; thresholds require physically
labelled target-neighbourhood calibration views.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Iterable

import cv2
import numpy as np


SCHEMA_VERSION = "realworld_visual_convergence_audit_v1_20260825"
GEOMETRY_RANSAC_THRESHOLD_PX = 3.0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(
        payload, sort_keys=True, indent=2, allow_nan=False,
    ) + "\n").encode("utf-8")
    with tempfile.NamedTemporaryFile(
        dir=path.parent, prefix=path.name + ".", delete=False,
    ) as handle:
        handle.write(encoded)
        temporary = Path(handle.name)
    os.replace(temporary, path)


def _atomic_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row})
    with tempfile.NamedTemporaryFile(
        dir=path.parent, prefix=path.name + ".", suffix=".csv",
        mode="w", encoding="utf-8", newline="", delete=False,
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
        temporary = Path(handle.name)
    os.replace(temporary, path)


def _hull_coverage(points: np.ndarray, height: int, width: int) -> float:
    points = np.asarray(points, dtype=np.float32)
    if len(points) < 3:
        return 0.0
    hull = cv2.convexHull(points.reshape(-1, 1, 2))
    return float(cv2.contourArea(hull) / max(float(height * width), 1.0))


def _project(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    homogeneous = np.concatenate(
        [points, np.ones((len(points), 1), dtype=np.float64)], axis=1,
    )
    projected = homogeneous @ np.asarray(matrix, dtype=np.float64).T
    denominator = projected[:, 2:3]
    valid = np.abs(denominator[:, 0]) > 1e-12
    output = np.full((len(points), 2), np.nan, dtype=np.float64)
    output[valid] = projected[valid, :2] / denominator[valid]
    return output


def _warp_deviation(
    matrix: np.ndarray,
    reference_hw: tuple[int, int],
    query_hw: tuple[int, int],
) -> tuple[float, float]:
    ref_h, ref_w = map(int, reference_hw)
    query_h, query_w = map(int, query_hw)
    reference = np.asarray([
        [0.0, 0.0], [ref_w - 1.0, 0.0],
        [ref_w - 1.0, ref_h - 1.0], [0.0, ref_h - 1.0],
        [(ref_w - 1.0) / 2.0, (ref_h - 1.0) / 2.0],
    ])
    expected = np.asarray([
        [0.0, 0.0], [query_w - 1.0, 0.0],
        [query_w - 1.0, query_h - 1.0], [0.0, query_h - 1.0],
        [(query_w - 1.0) / 2.0, (query_h - 1.0) / 2.0],
    ])
    warped = _project(matrix, reference)
    diagonal = math.hypot(query_w, query_h)
    deviations = np.linalg.norm(warped - expected, axis=1) / diagonal
    finite = np.isfinite(deviations)
    if not finite.all():
        return float("inf"), float("inf")
    return float(np.max(deviations[:4])), float(deviations[4])


def _robust_homography(
    reference: np.ndarray,
    query: np.ndarray,
    reference_hw: tuple[int, int],
    query_hw: tuple[int, int],
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "homography_valid": False,
        "homography_inliers": 0,
        "homography_inlier_ratio": 0.0,
        "homography_median_transfer_diag": None,
        "homography_p90_transfer_diag": None,
        "homography_corner_identity_max_diag": None,
        "homography_center_identity_diag": None,
    }
    if len(reference) < 4:
        return result
    cv2.setRNGSeed(0)
    try:
        matrix, mask = cv2.findHomography(
            reference.astype(np.float32), query.astype(np.float32),
            cv2.USAC_MAGSAC, GEOMETRY_RANSAC_THRESHOLD_PX,
            maxIters=10000, confidence=0.999,
        )
    except cv2.error:
        return result
    if matrix is None or mask is None or not np.isfinite(matrix).all():
        return result
    keep = np.asarray(mask).reshape(-1).astype(bool)
    if len(keep) != len(reference) or not keep.any():
        return result
    projected = _project(matrix, reference[keep])
    residual = np.linalg.norm(projected - query[keep], axis=1)
    diagonal = math.hypot(*query_hw)
    corner, center = _warp_deviation(matrix, reference_hw, query_hw)
    result.update({
        "homography_valid": True,
        "homography_inliers": int(keep.sum()),
        "homography_inlier_ratio": float(keep.mean()),
        "homography_median_transfer_diag": float(
            np.median(residual) / diagonal
        ),
        "homography_p90_transfer_diag": float(
            np.percentile(residual, 90) / diagonal
        ),
        "homography_corner_identity_max_diag": corner,
        "homography_center_identity_diag": center,
    })
    return result


def _robust_affine(
    reference: np.ndarray,
    query: np.ndarray,
    reference_hw: tuple[int, int],
    query_hw: tuple[int, int],
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "affine_valid": False,
        "affine_inliers": 0,
        "affine_inlier_ratio": 0.0,
        "affine_scale": None,
        "affine_rotation_deg": None,
        "affine_translation_diag": None,
        "affine_median_transfer_diag": None,
        "affine_p90_transfer_diag": None,
        "affine_corner_identity_max_diag": None,
        "affine_center_identity_diag": None,
    }
    if len(reference) < 3:
        return result
    cv2.setRNGSeed(0)
    try:
        affine, mask = cv2.estimateAffinePartial2D(
            reference.astype(np.float32), query.astype(np.float32),
            method=cv2.RANSAC,
            ransacReprojThreshold=GEOMETRY_RANSAC_THRESHOLD_PX,
            maxIters=10000, confidence=0.999, refineIters=10,
        )
    except cv2.error:
        return result
    if affine is None or mask is None or not np.isfinite(affine).all():
        return result
    keep = np.asarray(mask).reshape(-1).astype(bool)
    if len(keep) != len(reference) or not keep.any():
        return result
    matrix = np.eye(3, dtype=np.float64)
    matrix[:2] = affine
    projected = _project(matrix, reference[keep])
    residual = np.linalg.norm(projected - query[keep], axis=1)
    diagonal = math.hypot(*query_hw)
    a, b = float(affine[0, 0]), float(affine[1, 0])
    corner, center = _warp_deviation(matrix, reference_hw, query_hw)
    result.update({
        "affine_valid": True,
        "affine_inliers": int(keep.sum()),
        "affine_inlier_ratio": float(keep.mean()),
        "affine_scale": math.hypot(a, b),
        "affine_rotation_deg": math.degrees(math.atan2(b, a)),
        "affine_translation_diag": float(
            np.linalg.norm(affine[:, 2]) / diagonal
        ),
        "affine_median_transfer_diag": float(
            np.median(residual) / diagonal
        ),
        "affine_p90_transfer_diag": float(
            np.percentile(residual, 90) / diagonal
        ),
        "affine_corner_identity_max_diag": corner,
        "affine_center_identity_diag": center,
    })
    return result


def _warp_gray_mae(
    matrix: np.ndarray | None,
    reference_path: Path,
    query_path: Path,
) -> tuple[float | None, float]:
    if matrix is None:
        return None, 0.0
    reference = cv2.imread(str(reference_path), cv2.IMREAD_GRAYSCALE)
    query = cv2.imread(str(query_path), cv2.IMREAD_GRAYSCALE)
    if reference is None or query is None:
        return None, 0.0
    height, width = query.shape
    warped = cv2.warpPerspective(
        reference, matrix, (width, height), flags=cv2.INTER_LINEAR,
    )
    support = cv2.warpPerspective(
        np.full(reference.shape, 255, dtype=np.uint8), matrix,
        (width, height), flags=cv2.INTER_NEAREST,
    ) > 0
    overlap = float(support.mean())
    if not support.any():
        return None, overlap
    difference = np.abs(
        warped.astype(np.float32) - query.astype(np.float32)
    )
    return float(np.median(difference[support]) / 255.0), overlap


def visual_convergence_metrics(
    reference_points: np.ndarray,
    query_points: np.ndarray,
    scores: np.ndarray,
    reference_hw: tuple[int, int],
    query_hw: tuple[int, int],
    *,
    reference_path: Path | None = None,
    query_path: Path | None = None,
) -> dict[str, Any]:
    """Measure near-identity two-view evidence without choosing a STOP gate."""

    reference = np.asarray(reference_points, dtype=np.float64)
    query = np.asarray(query_points, dtype=np.float64)
    scores = np.asarray(scores, dtype=np.float64)
    if (
        reference.ndim != 2 or reference.shape[1:] != (2,)
        or query.shape != reference.shape or scores.shape != (len(reference),)
    ):
        raise ValueError("aligned reference/query points and scores are required")
    ref_h, ref_w = map(int, reference_hw)
    query_h, query_w = map(int, query_hw)
    if min(ref_h, ref_w, query_h, query_w) <= 0:
        raise ValueError("positive image dimensions are required")

    result: dict[str, Any] = {
        "matches": int(len(reference)),
        "match_score_median": float(np.median(scores)) if len(scores) else 0.0,
        "reference_match_hull_coverage": _hull_coverage(
            reference, ref_h, ref_w
        ),
        "query_match_hull_coverage": _hull_coverage(query, query_h, query_w),
        "identity_flow_median_diag": None,
        "identity_flow_p90_diag": None,
        "identity_flow_x_median_width": None,
        "identity_flow_y_median_height": None,
    }
    if len(reference):
        normalized_reference = reference / np.asarray([ref_w, ref_h])
        normalized_query = query / np.asarray([query_w, query_h])
        delta = normalized_query - normalized_reference
        # Both axes are normalized before the Euclidean norm, avoiding a
        # hidden dependence on the camera's landscape aspect ratio.
        flow = np.linalg.norm(delta, axis=1) / math.sqrt(2.0)
        result.update({
            "identity_flow_median_diag": float(np.median(flow)),
            "identity_flow_p90_diag": float(np.percentile(flow, 90)),
            "identity_flow_x_median_width": float(np.median(delta[:, 0])),
            "identity_flow_y_median_height": float(np.median(delta[:, 1])),
        })

    homography = _robust_homography(
        reference, query, reference_hw, query_hw
    )
    result.update(homography)
    result.update(_robust_affine(reference, query, reference_hw, query_hw))

    # Refit the homography only for the optional photometric diagnostic.  It
    # is never used to grant STOP and is omitted when paths are unavailable.
    gray_mae: float | None = None
    gray_overlap = 0.0
    if (
        homography["homography_valid"]
        and reference_path is not None and query_path is not None
    ):
        cv2.setRNGSeed(0)
        matrix, _ = cv2.findHomography(
            reference.astype(np.float32), query.astype(np.float32),
            cv2.USAC_MAGSAC, GEOMETRY_RANSAC_THRESHOLD_PX,
            maxIters=10000, confidence=0.999,
        )
        gray_mae, gray_overlap = _warp_gray_mae(
            matrix, Path(reference_path), Path(query_path)
        )
    result["homography_gray_median_abs_error"] = gray_mae
    result["homography_gray_overlap"] = gray_overlap
    return result


def _numeric_frame_paths(
    directory: Path, start: int | None, end: int | None, stride: int,
) -> list[tuple[int, Path]]:
    indexed = []
    for path in Path(directory).glob("*.jpg"):
        if not path.stem.isdigit():
            continue
        index = int(path.stem)
        if start is not None and index < start:
            continue
        if end is not None and index > end:
            continue
        indexed.append((index, path.resolve()))
    indexed.sort()
    if stride <= 0:
        raise ValueError("stride must be positive")
    return indexed[::stride]


def _ranked(
    rows: Iterable[dict[str, Any]],
    field: str,
    count: int = 12,
    *,
    require_precheck: bool = False,
) \
        -> list[dict[str, Any]]:
    valid = [
        row for row in rows
        if row.get(field) is not None
        and (
            not require_precheck
            or row.get("certificate_precheck_passed") is True
        )
    ]
    return [
        {"frame_index": int(row["frame_index"]), field: row[field]}
        for row in sorted(valid, key=lambda item: float(item[field]))[:count]
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", required=True, type=Path)
    parser.add_argument("--frames", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--start", type=int)
    parser.add_argument("--end", type=int)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--max-keypoints", type=int, default=2048)
    parser.add_argument(
        "--nav-graph-root", type=Path,
        default=Path(os.environ.get(
            "NAV_GRAPH_ROOT", "/home/asus/Research/Nav-graph-blind"
        )),
    )
    parser.add_argument(
        "--lightglue-repo", type=Path,
        default=Path(os.environ.get(
            "LIGHTGLUE_REPO",
            "/home/asus/Research/Nav-graph-blind/.diagnostics/dependencies/LightGlue",
        )),
    )
    parser.add_argument(
        "--dependency-root", type=Path,
        default=Path(os.environ.get(
            "DEPENDENCY_ROOT",
            "/home/asus/Research/Nav-graph-blind/.diagnostics/dependencies/python",
        )),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    goal = args.goal.resolve()
    frames = args.frames.resolve()
    if not goal.is_file():
        raise FileNotFoundError(goal)
    if not frames.is_dir():
        raise NotADirectoryError(frames)
    paths = _numeric_frame_paths(frames, args.start, args.end, args.stride)
    if not paths:
        raise RuntimeError("no numeric JPEG frames matched the requested range")
    sys.path.insert(0, str(args.nav_graph_root.resolve()))
    from MemNavData.certified_relocalization_runtime import (
        CERTIFIED_EPIPOLAR_THRESHOLD_PX,
        fundamental_can_reach_certificate,
        fundamental_support,
    )
    from MemNavData.lingbot_pnp_localization import LightGluePointMatcher

    matcher = LightGluePointMatcher(
        args.lightglue_repo,
        dependency_root=args.dependency_root,
        device=args.device,
        max_keypoints=args.max_keypoints,
        reference_cache_size=0,
    )
    rows: list[dict[str, Any]] = []
    started = time.monotonic()
    for ordinal, (index, path) in enumerate(paths, start=1):
        before = time.monotonic()
        matched = matcher.match_paths(
            path, goal, target_height=518, target_width=518, patch_size=14,
        )
        reference_hw = tuple(map(int, matched["reference_raw_hw"]))
        query_hw = tuple(map(int, matched["query_raw_hw"]))
        support = fundamental_support(
            matched["reference_raw_points"], matched["query_raw_points"],
            matched["scores"], reference_hw, query_hw,
            threshold_px=CERTIFIED_EPIPOLAR_THRESHOLD_PX,
        )
        precheck_passed, precheck_reason = (
            fundamental_can_reach_certificate(support)
        )
        metrics = visual_convergence_metrics(
            matched["reference_raw_points"], matched["query_raw_points"],
            matched["scores"], reference_hw, query_hw,
            reference_path=path, query_path=goal,
        )
        row = {
            "schema_version": SCHEMA_VERSION,
            "frame_index": int(index),
            "frame_path": str(path),
            "frame_sha256": _sha256(path),
            "elapsed_ms": 1000.0 * (time.monotonic() - before),
            "certificate_precheck_passed": bool(precheck_passed),
            "certificate_precheck_reason": str(precheck_reason),
            **support,
            **metrics,
        }
        rows.append(row)
        print(
            f"[{ordinal:04d}/{len(paths):04d}] frame={index} "
            f"matches={row['matches']} F={row['fundamental_inliers']} "
            f"flow={row['identity_flow_median_diag']!s} "
            f"H-id={row['homography_corner_identity_max_diag']!s}",
            flush=True,
        )

    # The exact goal self-match is a sensor/codec sanity reference only.  It
    # is not counted as a physically labelled arrival example.
    self_match = matcher.match_paths(
        goal, goal, target_height=518, target_width=518, patch_size=14,
    )
    self_metrics = visual_convergence_metrics(
        self_match["reference_raw_points"], self_match["query_raw_points"],
        self_match["scores"],
        tuple(map(int, self_match["reference_raw_hw"])),
        tuple(map(int, self_match["query_raw_hw"])),
        reference_path=goal, query_path=goal,
    )
    summary = {
        "schema_version": SCHEMA_VERSION,
        "status": "measurement_only_no_stop_threshold",
        "label_status": (
            "recorded frames are unlabeled for physical arrival; goal self-match "
            "is a codec sanity reference only"
        ),
        "goal": str(goal),
        "goal_sha256": _sha256(goal),
        "frames": str(frames),
        "frame_count": len(rows),
        "frame_range": [int(rows[0]["frame_index"]), int(rows[-1]["frame_index"])],
        "stride": int(args.stride),
        "device": str(args.device),
        "max_keypoints": int(args.max_keypoints),
        "geometry_ransac_threshold_px": GEOMETRY_RANSAC_THRESHOLD_PX,
        "fundamental_threshold_px": float(CERTIFIED_EPIPOLAR_THRESHOLD_PX),
        "elapsed_s": time.monotonic() - started,
        "certificate_precheck_frame_count": sum(
            row["certificate_precheck_passed"] is True for row in rows
        ),
        "goal_self_reference": self_metrics,
        "ranked_smallest_identity_flow": _ranked(
            rows, "identity_flow_median_diag"
        ),
        "ranked_smallest_homography_identity": _ranked(
            rows, "homography_corner_identity_max_diag"
        ),
        "ranked_smallest_affine_identity": _ranked(
            rows, "affine_corner_identity_max_diag"
        ),
        "ranked_prechecked_smallest_identity_flow": _ranked(
            rows, "identity_flow_median_diag", require_precheck=True
        ),
        "ranked_prechecked_smallest_homography_identity": _ranked(
            rows, "homography_corner_identity_max_diag",
            require_precheck=True,
        ),
        "ranked_prechecked_smallest_affine_identity": _ranked(
            rows, "affine_corner_identity_max_diag", require_precheck=True
        ),
        "rows_csv": "visual_convergence_rows.csv",
    }
    output = args.output_dir.resolve()
    _atomic_csv(output / "visual_convergence_rows.csv", rows)
    _atomic_json(output / "visual_convergence_summary.json", summary)
    print(json.dumps(summary, sort_keys=True, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
