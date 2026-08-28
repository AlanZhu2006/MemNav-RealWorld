#!/usr/bin/env python3
"""Compute frozen A* L_i, Odin odometry P_i, success and SPL receipts."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from odin_gt_core import (
    astar_grid,
    nearest_traversable,
    path_cells_to_xy,
    sha256_file,
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def world_to_row_column(
    x_m: float,
    y_m: float,
    *,
    height: int,
    resolution_m: float,
    origin_xy: tuple[float, float],
) -> tuple[int, int]:
    grid_x = math.floor((x_m - origin_xy[0]) / resolution_m)
    grid_y = math.floor((y_m - origin_xy[1]) / resolution_m)
    return height - 1 - grid_y, grid_x


def validate_hash(path: Path, expected: str, label: str) -> None:
    actual = sha256_file(path)
    if actual != expected:
        raise ValueError(f"{label} SHA mismatch: expected {expected}, got {actual}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt-result", type=Path, required=True)
    parser.add_argument("--goal-receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--overlay", type=Path)
    parser.add_argument("--robot-radius-m", type=float, required=True)
    parser.add_argument("--inflation-margin-m", type=float, default=0.05)
    parser.add_argument("--maximum-snap-m", type=float, default=0.20)
    args = parser.parse_args()

    result_path = args.gt_result.expanduser().resolve()
    goal_path = args.goal_receipt.expanduser().resolve()
    result = load_json(result_path)
    goal = load_json(goal_path)
    if result.get("schema") != "memnav-odin1-gt-result-v1":
        raise ValueError("unsupported Odin GT result schema")
    if goal.get("schema") != "memnav-odin1-goal-anchor-v1":
        raise ValueError("unsupported sealed goal receipt schema")
    if result.get("goal", {}).get("receipt_sha256") != sha256_file(goal_path):
        raise ValueError("GT result and goal receipt are not hash-bound")
    map_sha = goal["odin_map"]["sha256"]
    if result.get("map", {}).get("sha256") != map_sha:
        raise ValueError("GT result and goal receipt use different Odin maps")

    yaml_path = Path(goal["occupancy_yaml"]["path"])
    image_path = Path(goal["occupancy_image"]["path"])
    validate_hash(yaml_path, goal["occupancy_yaml"]["sha256"], "occupancy YAML")
    validate_hash(image_path, goal["occupancy_image"]["sha256"], "occupancy image")
    map_metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"cannot read occupancy image: {image_path}")
    resolution_m = float(map_metadata["resolution"])
    origin = map_metadata["origin"]
    origin_xy = (float(origin[0]), float(origin[1]))
    unknown_pixel = int(map_metadata.get("unknown_pixel", 205))
    known_free = image >= 250
    known_free &= image != unknown_pixel
    blocked = np.logical_not(known_free).astype(np.uint8)
    clearance_m = args.robot_radius_m + args.inflation_margin_m
    inflation_cells = math.ceil(clearance_m / resolution_m)
    if inflation_cells > 0:
        diameter = inflation_cells * 2 + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (diameter, diameter)
        )
        blocked = cv2.dilate(blocked, kernel)
    traversable = blocked == 0

    start_pose = result.get("start_pose_map")
    if not isinstance(start_pose, dict):
        raise ValueError("GT result has no frozen start_pose_map")
    goal_pose = goal["goal_pose_map"]
    start_requested = world_to_row_column(
        float(start_pose["x"]),
        float(start_pose["y"]),
        height=image.shape[0],
        resolution_m=resolution_m,
        origin_xy=origin_xy,
    )
    goal_requested = world_to_row_column(
        float(goal_pose["x"]),
        float(goal_pose["y"]),
        height=image.shape[0],
        resolution_m=resolution_m,
        origin_xy=origin_xy,
    )
    maximum_snap_cells = math.ceil(args.maximum_snap_m / resolution_m)
    start_cell, start_snap_cells = nearest_traversable(
        traversable, start_requested, maximum_snap_cells
    )
    goal_cell, goal_snap_cells = nearest_traversable(
        traversable, goal_requested, maximum_snap_cells
    )
    grid_cost, path_cells = astar_grid(traversable, start_cell, goal_cell)
    start_snap_m = start_snap_cells * resolution_m
    goal_snap_m = goal_snap_cells * resolution_m
    if start_snap_m > args.maximum_snap_m + 1e-9:
        raise ValueError("start snap exceeds the frozen metric limit")
    if goal_snap_m > args.maximum_snap_m + 1e-9:
        raise ValueError("goal snap exceeds the frozen metric limit")
    shortest_path_m = grid_cost * resolution_m + start_snap_m + goal_snap_m
    actual_path_m = float(result["actual_path_m"])
    success = bool(result.get("success"))
    spl = (
        shortest_path_m / max(shortest_path_m, actual_path_m, 1e-9)
        if success
        else 0.0
    )
    route_xy = path_cells_to_xy(
        path_cells,
        height=image.shape[0],
        resolution_m=resolution_m,
        origin_xy=origin_xy,
    )

    overlay_path = args.overlay
    if overlay_path is None:
        overlay_path = args.output.expanduser().resolve().with_suffix(".png")
    overlay_path = overlay_path.expanduser().resolve()
    overlay_path.parent.mkdir(parents=True, exist_ok=True)
    overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for row, column in path_cells:
        overlay[row, column] = (0, 255, 255)
    cv2.circle(overlay, (start_cell[1], start_cell[0]), 4, (255, 0, 0), -1)
    cv2.circle(overlay, (goal_cell[1], goal_cell[0]), 4, (0, 0, 255), -1)
    if not cv2.imwrite(str(overlay_path), overlay):
        raise RuntimeError(f"failed to write A* overlay: {overlay_path}")

    receipt = {
        "schema": "memnav-odin1-spl-receipt-v1",
        "created_utc": utc_now(),
        "run_id": result["run_id"],
        "classification": "independent_reference_slam_not_metrological_ground_truth",
        "inputs": {
            "gt_result": {
                "path": str(result_path),
                "sha256": sha256_file(result_path),
            },
            "goal_receipt": {
                "path": str(goal_path),
                "sha256": sha256_file(goal_path),
            },
            "odin_map_sha256": map_sha,
            "occupancy_yaml_sha256": sha256_file(yaml_path),
            "occupancy_image_sha256": sha256_file(image_path),
        },
        "frozen_astar": {
            "connectivity": 8,
            "diagonal_corner_cutting": False,
            "unknown_is_obstacle": True,
            "resolution_m": resolution_m,
            "robot_radius_m": args.robot_radius_m,
            "inflation_margin_m": args.inflation_margin_m,
            "clearance_m": clearance_m,
            "inflation_cells": inflation_cells,
            "maximum_snap_m": args.maximum_snap_m,
            "start_requested_row_col": list(start_requested),
            "start_used_row_col": list(start_cell),
            "start_snap_m": round(start_snap_m, 4),
            "goal_requested_row_col": list(goal_requested),
            "goal_used_row_col": list(goal_cell),
            "goal_snap_m": round(goal_snap_m, 4),
            "route_xy": route_xy,
            "overlay": {
                "path": str(overlay_path),
                "sha256": sha256_file(overlay_path),
            },
        },
        "metrics": {
            "S_i": int(success),
            "L_i_m": round(shortest_path_m, 4),
            "P_i_m": round(actual_path_m, 4),
            "SPL_i": round(spl, 6),
            "formula": "S_i * L_i / max(L_i, P_i)",
        },
        "policy_input": False,
        "motion_authority": False,
    }
    atomic_write_json(args.output, receipt)
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
