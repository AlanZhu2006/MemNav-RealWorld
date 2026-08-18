#!/usr/bin/env python3
"""Small helpers for deterministic NavDP ROS debug visualization."""

from __future__ import annotations

import numpy as np


def ranked_candidates(
    trajectories: np.ndarray,
    values: np.ndarray,
    limit: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Return finite candidate trajectories ordered from highest to lowest value."""
    candidates = np.asarray(trajectories, dtype=np.float32)
    while candidates.ndim > 3 and candidates.shape[0] == 1:
        candidates = candidates[0]
    if candidates.ndim == 2:
        candidates = candidates[np.newaxis, ...]
    if candidates.ndim != 3 or candidates.shape[1] < 1 or candidates.shape[2] < 2:
        return (
            np.empty((0, 0, 2), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
        )

    finite = np.isfinite(candidates[:, :, :2]).all(axis=(1, 2))
    candidates = candidates[finite]
    if candidates.shape[0] == 0:
        return candidates, np.empty((0,), dtype=np.float32)

    scores = np.asarray(values, dtype=np.float32).reshape(-1)
    if scores.size == finite.size:
        scores = scores[finite]
    else:
        scores = np.zeros((candidates.shape[0],), dtype=np.float32)
    sortable_scores = np.nan_to_num(scores, nan=-np.inf, neginf=-np.inf, posinf=np.inf)
    order = np.argsort(-sortable_scores, kind="stable")
    count = min(max(int(limit), 0), candidates.shape[0])
    order = order[:count]
    return candidates[order].copy(), scores[order].copy()


def score_rgb(score: float, minimum: float, maximum: float) -> tuple[float, float, float]:
    """Map a policy score to a blue-low, red-high RGB color."""
    if not np.isfinite(score) or not np.isfinite(minimum) or not np.isfinite(maximum):
        fraction = 0.5
    elif maximum - minimum <= 1e-8:
        fraction = 1.0
    else:
        fraction = float(np.clip((score - minimum) / (maximum - minimum), 0.0, 1.0))
    green = 0.15 + 0.35 * (1.0 - abs(2.0 * fraction - 1.0))
    return fraction, green, 1.0 - fraction
