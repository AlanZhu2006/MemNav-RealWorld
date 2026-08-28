from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from odin_gt_core import (
    ArrivalGate,
    PathAccumulator,
    Pose2D,
    RelocalizationGate,
    astar_grid,
    compose_pose,
    inverse_pose,
)


class OdinGtCoreTests(unittest.TestCase):
    def test_pose_inverse_round_trip(self):
        transform = Pose2D(1.5, -0.25, 0.7)
        point = Pose2D(0.4, 2.0, -0.3)

        recovered = compose_pose(inverse_pose(transform), compose_pose(transform, point))

        self.assertAlmostEqual(recovered.x, point.x)
        self.assertAlmostEqual(recovered.y, point.y)
        self.assertAlmostEqual(recovered.yaw, point.yaw)

    def test_relocalization_requires_stable_tf_and_latches_late_jump(self):
        gate = RelocalizationGate(
            hold_s=1.0,
            minimum_samples=3,
            max_translation_change_m=0.1,
            max_rotation_change_rad=0.1,
        )
        for stamp in (0.0, 0.5, 1.0):
            gate.update(stamp, Pose2D(1.0, 2.0, 0.2))

        self.assertTrue(gate.ready(1.0))
        gate.update(1.1, Pose2D(1.5, 2.0, 0.2))

        self.assertFalse(gate.ready(2.0))
        self.assertEqual(gate.invalid_reason, "map_to_odom_jump_after_ready")

    def test_path_uses_local_odom_increments_and_rejects_jump(self):
        path = PathAccumulator(max_step_m=0.5, max_inferred_speed_mps=2.0)
        self.assertTrue(path.update(0.0, Pose2D(0.0, 0.0, 0.0)))
        self.assertTrue(path.update(0.5, Pose2D(0.3, 0.0, 0.0)))
        self.assertAlmostEqual(path.path_length_m, 0.3)

        self.assertFalse(path.update(1.0, Pose2D(1.0, 0.0, 0.0)))
        self.assertEqual(path.invalid_reason, "odometry_position_jump")

    def test_arrival_requires_metric_visual_stationary_hold(self):
        gate = ArrivalGate(distance_m=0.85, speed_mps=0.1, hold_s=1.0)
        self.assertFalse(
            gate.update(
                now_s=0.0,
                metric_distance_m=0.4,
                planar_speed_mps=0.0,
                visual_confirmed=False,
                reference_ready=True,
            )
        )
        self.assertFalse(
            gate.update(
                now_s=1.0,
                metric_distance_m=0.4,
                planar_speed_mps=0.0,
                visual_confirmed=True,
                reference_ready=True,
            )
        )
        self.assertTrue(
            gate.update(
                now_s=2.0,
                metric_distance_m=0.4,
                planar_speed_mps=0.0,
                visual_confirmed=True,
                reference_ready=True,
            )
        )

    def test_astar_rejects_diagonal_corner_cutting(self):
        grid = np.ones((3, 3), dtype=bool)
        grid[0, 1] = False
        grid[1, 0] = False

        with self.assertRaisesRegex(ValueError, "no traversable"):
            astar_grid(grid, (0, 0), (2, 2))

    def test_astar_returns_metric_cell_cost(self):
        cost, route = astar_grid(np.ones((3, 3), dtype=bool), (0, 0), (2, 2))

        self.assertAlmostEqual(cost, 2.0 * math.sqrt(2.0))
        self.assertEqual(route[0], (0, 0))
        self.assertEqual(route[-1], (2, 2))


if __name__ == "__main__":
    unittest.main()
