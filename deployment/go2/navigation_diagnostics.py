"""Observation-only navigation attribution; no collision or motion authority."""

import math
import numpy as np


def bearing_deg(point):
    if point is None:
        return None
    xy = np.asarray(point, dtype=float).reshape(-1)[:2]
    if len(xy) != 2 or not np.isfinite(xy).all() or np.linalg.norm(xy) < 1e-8:
        return None
    return float(np.degrees(np.arctan2(xy[1], xy[0])))


def depth_sectors(depth):
    """Optical Z statistics, NOT body clearance or a collision certificate."""
    d = np.asarray(depth)
    h, w = d.shape
    result = {}
    for name, (left, right, top, bottom) in {
        "center": (.35, .65, .30, .70),
        "left": (0, .35, .20, .90),
        "right": (.65, 1, .20, .90),
        "bottom": (.20, .80, .70, 1),
    }.items():
        roi = d[int(top*h):int(bottom*h), int(left*w):int(right*w)]
        # Deterministic subsampling limits diagnostics overhead on Jetson.
        roi = roi[::4, ::4]
        valid = np.isfinite(roi) & (roi > .05) & (roi <= 5)
        values = roi[valid]
        result[name] = {
            "valid_fraction": float(valid.mean()) if roi.size else 0.0,
            "p01_p10_p50_optical_z_m": (
                np.percentile(values, [1, 10, 50]).tolist() if values.size else None
            ),
        }
    return result


def plan_diagnostics(path, candidates, values, receipt, command, depth):
    xy = np.asarray(path, dtype=float)[:, :2]
    bearing = bearing_deg(receipt.get("memory_controller_pointgoal"))
    near_bearing = bearing_deg([command.target_x, command.target_y])
    delta = None if bearing is None or near_bearing is None else (
        (near_bearing - bearing + 180) % 360 - 180
    )
    summaries = []
    all_paths = np.asarray(candidates, dtype=float)
    if all_paths.size:
        all_paths = all_paths.reshape(-1, all_paths.shape[-2], all_paths.shape[-1])
        scores = np.asarray(values).reshape(-1)
        for i, candidate in enumerate(all_paths):
            p = candidate[:, :2]
            score = float(scores[i]) if i < len(scores) else math.nan
            summaries.append({
                "index": i, "endpoint_xy_m": p[-1].tolist(),
                "path_length_m": float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
                "critic": score if math.isfinite(score) else None,
            })
    return {
        "schema": "navigation_attribution_v1",
        "observation_only": True,
        "memory_bearing_deg": bearing,
        "lookahead_bearing_deg": near_bearing,
        "lookahead_minus_memory_bearing_deg": delta,
        "selected_path_xy_m": xy.tolist(),
        "lookahead_xy_m": [command.target_x, command.target_y],
        "trajectory_command": {"vx": command.linear_x, "wz": command.angular_z},
        "candidates_after_policy_postprocessing": summaries,
        "input_depth_sectors": depth_sectors(depth),
        "depth_semantics": "camera_optical_z_not_body_clearance",
        "pre_zeroing_candidates_available": False,
    }
