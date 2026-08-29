#!/usr/bin/env python3
"""Pure geometry and metric primitives for the independent Odin1 GT lane."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import heapq
import math
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def distance(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def yaw_error(self, other: "Pose2D") -> float:
        return abs(wrap_angle(self.yaw - other.yaw))


def compose_pose(parent_from_child: Pose2D, child_pose: Pose2D) -> Pose2D:
    cosine = math.cos(parent_from_child.yaw)
    sine = math.sin(parent_from_child.yaw)
    return Pose2D(
        x=parent_from_child.x + cosine * child_pose.x - sine * child_pose.y,
        y=parent_from_child.y + sine * child_pose.x + cosine * child_pose.y,
        yaw=wrap_angle(parent_from_child.yaw + child_pose.yaw),
    )


def inverse_pose(parent_from_child: Pose2D) -> Pose2D:
    cosine = math.cos(parent_from_child.yaw)
    sine = math.sin(parent_from_child.yaw)
    return Pose2D(
        x=-cosine * parent_from_child.x - sine * parent_from_child.y,
        y=sine * parent_from_child.x - cosine * parent_from_child.y,
        yaw=wrap_angle(-parent_from_child.yaw),
    )


class RelocalizationGate:
    """Fail-closed stability gate for the vendor map->odom TF evidence."""

    def __init__(
        self,
        *,
        hold_s: float = 2.0,
        minimum_samples: int = 5,
        max_translation_change_m: float = 0.15,
        max_rotation_change_rad: float = math.radians(5.0),
    ) -> None:
        self.hold_s = float(hold_s)
        self.minimum_samples = int(minimum_samples)
        self.max_translation_change_m = float(max_translation_change_m)
        self.max_rotation_change_rad = float(max_rotation_change_rad)
        self.anchor: Optional[Pose2D] = None
        self.latest: Optional[Pose2D] = None
        self.stable_since_s: Optional[float] = None
        self.last_observed_s: Optional[float] = None
        self.sample_count = 0
        self.ever_ready = False
        self.invalid_reason = ""

    def update(self, observed_s: float, transform: Pose2D) -> None:
        values = (transform.x, transform.y, transform.yaw, observed_s)
        if not all(math.isfinite(value) for value in values):
            self.invalid_reason = "nonfinite_map_to_odom"
            return
        if self.invalid_reason:
            return
        if self.anchor is None:
            self.anchor = transform
            self.latest = transform
            self.stable_since_s = float(observed_s)
            self.last_observed_s = float(observed_s)
            self.sample_count = 1
            return
        changed = (
            self.anchor.distance(transform) > self.max_translation_change_m
            or self.anchor.yaw_error(transform) > self.max_rotation_change_rad
        )
        if changed:
            if self.ever_ready:
                self.invalid_reason = "map_to_odom_jump_after_ready"
                return
            self.anchor = transform
            self.stable_since_s = float(observed_s)
            self.sample_count = 1
        else:
            self.sample_count += 1
        self.latest = transform
        self.last_observed_s = float(observed_s)

    def ready(self, now_s: float) -> bool:
        ready = bool(
            not self.invalid_reason
            and self.latest is not None
            and self.stable_since_s is not None
            and self.sample_count >= self.minimum_samples
            and float(now_s) - self.stable_since_s >= self.hold_s
        )
        self.ever_ready = self.ever_ready or ready
        return ready

    def status(self, now_s: float) -> dict:
        return {
            "ready": self.ready(now_s),
            "ever_ready": self.ever_ready,
            "invalid_reason": self.invalid_reason,
            "sample_count": self.sample_count,
            "stable_for_s": (
                None
                if self.stable_since_s is None
                else round(max(0.0, now_s - self.stable_since_s), 3)
            ),
            "map_to_odom": (
                None
                if self.latest is None
                else {
                    "x": self.latest.x,
                    "y": self.latest.y,
                    "yaw_rad": self.latest.yaw,
                }
            ),
        }


class PathAccumulator:
    """Integrate consecutive Odin odom increments without map-correction jumps."""

    def __init__(
        self,
        *,
        max_step_m: float = 0.50,
        max_inferred_speed_mps: float = 2.0,
    ) -> None:
        self.max_step_m = float(max_step_m)
        self.max_inferred_speed_mps = float(max_inferred_speed_mps)
        self.started = False
        self.start_pose: Optional[Pose2D] = None
        self.latest_pose: Optional[Pose2D] = None
        self.latest_stamp_s: Optional[float] = None
        self.path_length_m = 0.0
        self.sample_count = 0
        self.invalid_reason = ""

    def reset(self) -> None:
        self.__init__(
            max_step_m=self.max_step_m,
            max_inferred_speed_mps=self.max_inferred_speed_mps,
        )

    def update(self, stamp_s: float, pose: Pose2D) -> bool:
        if self.invalid_reason:
            return False
        values = (stamp_s, pose.x, pose.y, pose.yaw)
        if not all(math.isfinite(value) for value in values):
            self.invalid_reason = "nonfinite_odometry"
            return False
        if self.latest_pose is None:
            self.started = True
            self.start_pose = pose
            self.latest_pose = pose
            self.latest_stamp_s = float(stamp_s)
            self.sample_count = 1
            return True
        if stamp_s <= float(self.latest_stamp_s):
            self.invalid_reason = "nonmonotonic_odometry_stamp"
            return False
        step_m = self.latest_pose.distance(pose)
        delta_s = stamp_s - float(self.latest_stamp_s)
        if step_m > self.max_step_m:
            self.invalid_reason = "odometry_position_jump"
            return False
        if delta_s > 0.0 and step_m / delta_s > self.max_inferred_speed_mps:
            self.invalid_reason = "odometry_inferred_speed_jump"
            return False
        self.path_length_m += step_m
        self.latest_pose = pose
        self.latest_stamp_s = float(stamp_s)
        self.sample_count += 1
        return True

    def status(self) -> dict:
        return {
            "started": self.started,
            "sample_count": self.sample_count,
            "path_length_m": round(self.path_length_m, 4),
            "invalid_reason": self.invalid_reason,
            "start_pose": None if self.start_pose is None else self.start_pose.__dict__,
            "latest_pose": (
                None if self.latest_pose is None else self.latest_pose.__dict__
            ),
        }


class ArrivalGate:
    """Combine independent metric, RGB-arrival and stationary evidence."""

    def __init__(
        self,
        *,
        distance_m: float = 0.85,
        speed_mps: float = 0.10,
        hold_s: float = 1.0,
    ) -> None:
        self.distance_m = float(distance_m)
        self.speed_mps = float(speed_mps)
        self.hold_s = float(hold_s)
        self.within_since_s: Optional[float] = None
        self.success = False

    def update(
        self,
        *,
        now_s: float,
        metric_distance_m: float,
        planar_speed_mps: float,
        rgb_arrival_confirmed: bool,
        reference_ready: bool,
    ) -> bool:
        conditions = (
            reference_ready
            and rgb_arrival_confirmed
            and math.isfinite(metric_distance_m)
            and metric_distance_m <= self.distance_m
            and math.isfinite(planar_speed_mps)
            and planar_speed_mps <= self.speed_mps
        )
        if not conditions:
            self.within_since_s = None
            return self.success
        if self.within_since_s is None:
            self.within_since_s = float(now_s)
        if now_s - self.within_since_s >= self.hold_s:
            self.success = True
        return self.success

    def status(self, now_s: float) -> dict:
        return {
            "success": self.success,
            "holding": self.within_since_s is not None,
            "hold_elapsed_s": (
                0.0
                if self.within_since_s is None
                else round(max(0.0, now_s - self.within_since_s), 3)
            ),
            "thresholds": {
                "distance_m": self.distance_m,
                "speed_mps": self.speed_mps,
                "hold_s": self.hold_s,
                "rgb_arrival_required": True,
            },
        }


NEIGHBORS_8 = (
    (-1, -1, math.sqrt(2.0)),
    (-1, 0, 1.0),
    (-1, 1, math.sqrt(2.0)),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (1, -1, math.sqrt(2.0)),
    (1, 0, 1.0),
    (1, 1, math.sqrt(2.0)),
)


def astar_grid(
    traversable: np.ndarray,
    start_row_col: tuple[int, int],
    goal_row_col: tuple[int, int],
) -> tuple[float, list[tuple[int, int]]]:
    if traversable.ndim != 2:
        raise ValueError("traversable grid must be two-dimensional")
    height, width = traversable.shape
    start = tuple(int(value) for value in start_row_col)
    goal = tuple(int(value) for value in goal_row_col)
    for label, cell in (("start", start), ("goal", goal)):
        row, column = cell
        if not (0 <= row < height and 0 <= column < width):
            raise ValueError(f"{label} cell is outside the map")
        if not bool(traversable[row, column]):
            raise ValueError(f"{label} cell is not traversable")
    queue: list[tuple[float, float, tuple[int, int]]] = []
    heapq.heappush(queue, (0.0, 0.0, start))
    costs = {start: 0.0}
    parents: dict[tuple[int, int], tuple[int, int]] = {}
    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current_cost != costs.get(current):
            continue
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parents[path[-1]])
            path.reverse()
            return current_cost, path
        row, column = current
        for delta_row, delta_column, step_cost in NEIGHBORS_8:
            neighbor = (row + delta_row, column + delta_column)
            neighbor_row, neighbor_column = neighbor
            if not (
                0 <= neighbor_row < height
                and 0 <= neighbor_column < width
                and bool(traversable[neighbor_row, neighbor_column])
            ):
                continue
            if delta_row and delta_column:
                if not (
                    traversable[row + delta_row, column]
                    and traversable[row, column + delta_column]
                ):
                    continue
            candidate_cost = current_cost + step_cost
            if candidate_cost >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = candidate_cost
            parents[neighbor] = current
            heuristic = math.hypot(
                neighbor_row - goal[0], neighbor_column - goal[1]
            )
            heapq.heappush(
                queue, (candidate_cost + heuristic, candidate_cost, neighbor)
            )
    raise ValueError("no traversable A* route connects start and goal")


def nearest_traversable(
    traversable: np.ndarray,
    requested: tuple[int, int],
    max_radius_cells: int,
) -> tuple[tuple[int, int], float]:
    row, column = requested
    height, width = traversable.shape
    candidates: list[tuple[float, int, int]] = []
    for candidate_row in range(max(0, row - max_radius_cells), min(height, row + max_radius_cells + 1)):
        for candidate_column in range(
            max(0, column - max_radius_cells), min(width, column + max_radius_cells + 1)
        ):
            if not traversable[candidate_row, candidate_column]:
                continue
            distance = math.hypot(candidate_row - row, candidate_column - column)
            if distance <= max_radius_cells:
                candidates.append((distance, candidate_row, candidate_column))
    if not candidates:
        raise ValueError("no traversable cell is within the snap radius")
    distance, result_row, result_column = min(candidates)
    return (result_row, result_column), distance


def path_cells_to_xy(
    cells: Iterable[tuple[int, int]],
    *,
    height: int,
    resolution_m: float,
    origin_xy: tuple[float, float],
) -> list[list[float]]:
    origin_x, origin_y = origin_xy
    return [
        [
            origin_x + (column + 0.5) * resolution_m,
            origin_y + (height - row - 0.5) * resolution_m,
        ]
        for row, column in cells
    ]
