#!/usr/bin/env python3
"""Build a frozen 2-D evaluation grid from the independent Odin1 survey."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Iterable

import cv2
import numpy as np

from odin_gt_core import sha256_file


UNKNOWN_PIXEL = 205
FREE_PIXEL = 254
OCCUPIED_PIXEL = 0


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def bresenham(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    delta_x = abs(x1 - x0)
    delta_y = -abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = delta_x + delta_y
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        doubled = 2 * error
        if doubled >= delta_y:
            error += delta_y
            x0 += step_x
        if doubled <= delta_x:
            error += delta_x
            y0 += step_y
    return cells


class OccupancyEvidence:
    def __init__(
        self,
        *,
        resolution_m: float,
        obstacle_min_z_m: float,
        obstacle_max_z_m: float,
        minimum_range_m: float,
        maximum_range_m: float,
        minimum_occupied_hits: int,
        maximum_sync_skew_s: float = 0.10,
    ) -> None:
        self.resolution_m = float(resolution_m)
        self.obstacle_min_z_m = float(obstacle_min_z_m)
        self.obstacle_max_z_m = float(obstacle_max_z_m)
        self.minimum_range_m = float(minimum_range_m)
        self.maximum_range_m = float(maximum_range_m)
        self.minimum_occupied_hits = int(minimum_occupied_hits)
        self.maximum_sync_skew_s = float(maximum_sync_skew_s)
        self.free_hits: dict[tuple[int, int], int] = defaultdict(int)
        self.occupied_hits: dict[tuple[int, int], int] = defaultdict(int)
        self.robot_cells: set[tuple[int, int]] = set()
        self.cloud_count = 0
        self.point_count = 0
        self.unsynchronized_clouds = 0
        self.maximum_observed_sync_skew_s = 0.0

    def metric_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        return (
            math.floor(float(x_m) / self.resolution_m),
            math.floor(float(y_m) / self.resolution_m),
        )

    def update(
        self,
        robot_xy: tuple[float, float],
        points_xyz: Iterable[tuple[float, float, float]],
    ) -> int:
        origin = self.metric_cell(*robot_xy)
        self.robot_cells.add(origin)
        accepted = 0
        for x_m, y_m, z_m in points_xyz:
            if not all(math.isfinite(value) for value in (x_m, y_m, z_m)):
                continue
            distance_m = math.hypot(x_m - robot_xy[0], y_m - robot_xy[1])
            if not self.minimum_range_m <= distance_m <= self.maximum_range_m:
                continue
            endpoint = self.metric_cell(x_m, y_m)
            ray = bresenham(origin, endpoint)
            for cell in ray[:-1]:
                self.free_hits[cell] += 1
            if self.obstacle_min_z_m <= z_m <= self.obstacle_max_z_m:
                self.occupied_hits[endpoint] += 1
            else:
                self.free_hits[endpoint] += 1
            accepted += 1
        self.cloud_count += 1
        self.point_count += accepted
        return accepted

    def render(self, margin_m: float) -> tuple[np.ndarray, tuple[float, float]]:
        cells = set(self.free_hits) | set(self.occupied_hits) | self.robot_cells
        if not cells:
            raise ValueError("no occupancy evidence was accumulated")
        margin_cells = max(1, math.ceil(float(margin_m) / self.resolution_m))
        minimum_x = min(cell[0] for cell in cells) - margin_cells
        maximum_x = max(cell[0] for cell in cells) + margin_cells
        minimum_y = min(cell[1] for cell in cells) - margin_cells
        maximum_y = max(cell[1] for cell in cells) + margin_cells
        width = maximum_x - minimum_x + 1
        height = maximum_y - minimum_y + 1
        image = np.full((height, width), UNKNOWN_PIXEL, dtype=np.uint8)
        for cell, hits in self.free_hits.items():
            if hits <= 0:
                continue
            column = cell[0] - minimum_x
            row = maximum_y - cell[1]
            image[row, column] = FREE_PIXEL
        for cell, hits in self.occupied_hits.items():
            if hits < self.minimum_occupied_hits:
                continue
            column = cell[0] - minimum_x
            row = maximum_y - cell[1]
            image[row, column] = OCCUPIED_PIXEL
        for cell in self.robot_cells:
            column = cell[0] - minimum_x
            row = maximum_y - cell[1]
            image[row, column] = FREE_PIXEL
        origin_xy = (
            minimum_x * self.resolution_m,
            minimum_y * self.resolution_m,
        )
        return image, origin_xy


def write_map(
    evidence: OccupancyEvidence,
    output_prefix: Path,
    *,
    margin_m: float,
    session_id: str,
    cloud_topic: str,
    odometry_topic: str,
    cloud_frame: str,
) -> dict:
    output_prefix = output_prefix.expanduser().resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    image, origin_xy = evidence.render(margin_m)
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    receipt_path = output_prefix.with_suffix(".receipt.json")
    if not cv2.imwrite(str(pgm_path), image):
        raise RuntimeError(f"failed to write occupancy image: {pgm_path}")
    yaml_payload = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": evidence.resolution_m,
        "origin": [origin_xy[0], origin_xy[1], 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "unknown_pixel": UNKNOWN_PIXEL,
    }
    yaml_path.write_text(
        "\n".join(
            (
                f"image: {yaml_payload['image']}",
                "mode: trinary",
                f"resolution: {yaml_payload['resolution']}",
                "origin: ["
                f"{origin_xy[0]:.6f}, {origin_xy[1]:.6f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                f"unknown_pixel: {UNKNOWN_PIXEL}",
                "",
            )
        ),
        encoding="utf-8",
    )
    receipt = {
        "schema": "memnav-odin1-occupancy-v1",
        "created_utc": utc_now(),
        "session_id": session_id,
        "classification": "independent_reference_slam_not_metrological_ground_truth",
        "frame": cloud_frame,
        "topics": {"cloud": cloud_topic, "odometry": odometry_topic},
        "parameters": {
            "resolution_m": evidence.resolution_m,
            "obstacle_min_z_m": evidence.obstacle_min_z_m,
            "obstacle_max_z_m": evidence.obstacle_max_z_m,
            "minimum_range_m": evidence.minimum_range_m,
            "maximum_range_m": evidence.maximum_range_m,
            "minimum_occupied_hits": evidence.minimum_occupied_hits,
            "maximum_sync_skew_s": evidence.maximum_sync_skew_s,
            "margin_m": margin_m,
        },
        "counts": {
            "clouds": evidence.cloud_count,
            "accepted_points": evidence.point_count,
            "known_free_cells": int(np.count_nonzero(image == FREE_PIXEL)),
            "occupied_cells": int(np.count_nonzero(image == OCCUPIED_PIXEL)),
            "unknown_cells": int(np.count_nonzero(image == UNKNOWN_PIXEL)),
            "unsynchronized_clouds": evidence.unsynchronized_clouds,
            "maximum_observed_sync_skew_s": round(
                evidence.maximum_observed_sync_skew_s, 6
            ),
        },
        "grid": {
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
            "origin_xy": list(origin_xy),
            "pgm_path": str(pgm_path),
            "pgm_sha256": sha256_file(pgm_path),
            "yaml_path": str(yaml_path),
            "yaml_sha256": sha256_file(yaml_path),
        },
        "policy_input": False,
        "motion_authority": False,
    }
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return receipt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--cloud-topic", default="/odin1/cloud_slam")
    parser.add_argument("--odometry-topic", default="/odin1/odometry")
    parser.add_argument("--expected-frame", default="odom")
    parser.add_argument("--resolution-m", type=float, default=0.05)
    parser.add_argument("--obstacle-min-z-m", type=float, required=True)
    parser.add_argument("--obstacle-max-z-m", type=float, required=True)
    parser.add_argument("--minimum-range-m", type=float, default=0.20)
    parser.add_argument("--maximum-range-m", type=float, default=8.0)
    parser.add_argument("--minimum-occupied-hits", type=int, default=2)
    parser.add_argument("--max-points-per-cloud", type=int, default=2500)
    parser.add_argument("--maximum-sync-skew-s", type=float, default=0.10)
    parser.add_argument("--margin-m", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.obstacle_min_z_m >= args.obstacle_max_z_m:
        raise ValueError("obstacle_min_z_m must be below obstacle_max_z_m")
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    evidence = OccupancyEvidence(
        resolution_m=args.resolution_m,
        obstacle_min_z_m=args.obstacle_min_z_m,
        obstacle_max_z_m=args.obstacle_max_z_m,
        minimum_range_m=args.minimum_range_m,
        maximum_range_m=args.maximum_range_m,
        minimum_occupied_hits=args.minimum_occupied_hits,
        maximum_sync_skew_s=args.maximum_sync_skew_s,
    )

    class BuilderNode(Node):
        def __init__(self) -> None:
            super().__init__("memnav_odin1_occupancy_builder")
            self.odometry_samples: deque[
                tuple[float, tuple[float, float]]
            ] = deque(maxlen=200)
            self.last_cloud_s = 0.0
            self.frame_error = ""
            self.create_subscription(
                Odometry,
                args.odometry_topic,
                self.on_odometry,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PointCloud2,
                args.cloud_topic,
                self.on_cloud,
                qos_profile_sensor_data,
            )

        def on_odometry(self, message: Odometry) -> None:
            frame = message.header.frame_id.lstrip("/")
            if frame != args.expected_frame.lstrip("/"):
                self.frame_error = f"unexpected_odometry_frame:{frame}"
                return
            position = message.pose.pose.position
            stamp = message.header.stamp
            stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            self.odometry_samples.append(
                (stamp_s, (float(position.x), float(position.y)))
            )

        def on_cloud(self, message: PointCloud2) -> None:
            if not self.odometry_samples:
                return
            frame = message.header.frame_id.lstrip("/")
            if frame != args.expected_frame.lstrip("/"):
                self.frame_error = f"unexpected_cloud_frame:{frame}"
                return
            stamp = message.header.stamp
            cloud_stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            odometry_stamp_s, robot_xy = min(
                self.odometry_samples,
                key=lambda sample: abs(sample[0] - cloud_stamp_s),
            )
            skew_s = abs(odometry_stamp_s - cloud_stamp_s)
            if skew_s > args.maximum_sync_skew_s:
                evidence.unsynchronized_clouds += 1
                return
            evidence.maximum_observed_sync_skew_s = max(
                evidence.maximum_observed_sync_skew_s, skew_s
            )
            raw_points = list(
                point_cloud2.read_points(
                    message, field_names=("x", "y", "z"), skip_nans=True
                )
            )
            if not raw_points:
                return
            stride = max(1, math.ceil(len(raw_points) / args.max_points_per_cloud))
            selected = (
                (float(point[0]), float(point[1]), float(point[2]))
                for point in raw_points[::stride]
            )
            accepted = evidence.update(robot_xy, selected)
            self.last_cloud_s = time.monotonic()
            if evidence.cloud_count % 20 == 0:
                self.get_logger().info(
                    f"clouds={evidence.cloud_count} accepted_points={evidence.point_count} "
                    f"latest={accepted}"
                )

    rclpy.init()
    node = BuilderNode()
    started_s = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.duration_s > 0.0 and time.monotonic() - started_s >= args.duration_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node.frame_error:
            node.destroy_node()
            rclpy.shutdown()
            raise RuntimeError(node.frame_error)
        receipt = write_map(
            evidence,
            args.output_prefix,
            margin_m=args.margin_m,
            session_id=args.session_id,
            cloud_topic=args.cloud_topic,
            odometry_topic=args.odometry_topic,
            cloud_frame=args.expected_frame,
        )
        print(json.dumps(receipt, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
