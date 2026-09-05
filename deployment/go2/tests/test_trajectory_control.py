import math
import pathlib
import sys
import unittest

import numpy as np


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from trajectory_control import (  # noqa: E402
    ControllerConfig,
    DepthSafetyConfig,
    VelocityCommand,
    apply_depth_safety,
    front_clearance,
    select_lookahead,
    slew_limit,
    trajectory_to_command,
)


class TrajectoryControlTest(unittest.TestCase):
    def test_select_lookahead_interpolates(self):
        target, length = select_lookahead(np.array([[0.4, 0.0], [0.8, 0.0]]), 0.6)
        np.testing.assert_allclose(target, [0.6, 0.0], atol=1e-6)
        self.assertAlmostEqual(length, 0.8)

    def test_forward_path_generates_forward_command(self):
        command = trajectory_to_command(np.array([[0.25, 0.0], [1.0, 0.0]]))
        self.assertGreater(command.linear_x, 0.0)
        self.assertAlmostEqual(command.angular_z, 0.0)

    def test_default_forward_limit_matches_validated_go2_speed(self):
        command = trajectory_to_command(np.array([[0.6, 0.0], [1.2, 0.0]]))
        self.assertAlmostEqual(command.linear_x, 0.30)

    def test_tinynav_heading_deadband_rejects_small_alternating_turns(self):
        distance = 0.60
        for angle_deg in (-7.0, 7.0):
            angle = math.radians(angle_deg)
            path = np.array(
                [
                    [distance * math.cos(angle), distance * math.sin(angle)],
                    [2.0 * distance * math.cos(angle), 2.0 * distance * math.sin(angle)],
                ]
            )
            command = trajectory_to_command(path)
            self.assertGreater(command.linear_x, 0.0)
            self.assertEqual(command.angular_z, 0.0)

    def test_heading_outside_tinynav_deadband_still_turns(self):
        distance = 0.60
        angle = math.radians(12.0)
        path = np.array(
            [
                [distance * math.cos(angle), distance * math.sin(angle)],
                [2.0 * distance * math.cos(angle), 2.0 * distance * math.sin(angle)],
            ]
        )
        command = trajectory_to_command(path)
        self.assertGreater(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_side_path_rotates_before_translation(self):
        command = trajectory_to_command(np.array([[0.0, 0.5], [0.0, 1.0]]))
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_reverse_is_disabled_by_default(self):
        command = trajectory_to_command(np.array([[-0.3, 0.0], [-1.0, 0.0]]))
        self.assertFalse(command.reverse)
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_reverse_can_be_enabled(self):
        config = ControllerConfig(allow_reverse=True)
        command = trajectory_to_command(np.array([[-0.3, 0.0], [-1.0, 0.0]]), config)
        self.assertTrue(command.reverse)
        self.assertLess(command.linear_x, 0.0)

    def test_depth_clearance_and_hard_stop(self):
        depth = np.full((100, 100), 2.0, dtype=np.float32)
        depth[30:82, 35:65] = 0.35
        config = DepthSafetyConfig()
        self.assertAlmostEqual(front_clearance(depth, config), 0.35, places=4)
        command = VelocityCommand(linear_x=0.2, angular_z=0.1)
        result = apply_depth_safety(command, depth, config)
        self.assertEqual(result.reason, "obstacle_stop")
        self.assertEqual(result.command.linear_x, 0.0)
        self.assertEqual(result.command.angular_z, 0.0)

    def test_turn_creep_remains_inside_forward_depth_stop(self):
        depth = np.full((100, 100), 0.34, dtype=np.float32)
        command = VelocityCommand(linear_x=0.10, angular_z=0.20)
        result = apply_depth_safety(command, depth)
        self.assertEqual(result.reason, "obstacle_stop")
        self.assertEqual(result.command.linear_x, 0.0)
        self.assertEqual(result.command.angular_z, 0.0)

    def test_invalid_depth_fails_closed(self):
        depth = np.zeros((100, 100), dtype=np.float32)
        result = apply_depth_safety(VelocityCommand(linear_x=0.2), depth)
        self.assertEqual(result.reason, "depth_unavailable_stop")
        self.assertEqual(result.command.linear_x, 0.0)

    def test_invalid_depth_stops_rotation(self):
        depth = np.zeros((100, 100), dtype=np.float32)
        result = apply_depth_safety(VelocityCommand(angular_z=0.4), depth)
        self.assertEqual(result.reason, "depth_unavailable_stop")
        self.assertEqual(result.command.angular_z, 0.0)

    def test_slew_limit(self):
        command = slew_limit(
            VelocityCommand(),
            VelocityCommand(linear_x=0.3, angular_z=0.6),
            dt=0.1,
            max_linear_accel=0.5,
            max_angular_accel=1.0,
        )
        self.assertAlmostEqual(command.linear_x, 0.05)
        self.assertAlmostEqual(command.angular_z, 0.1)

if __name__ == "__main__":
    unittest.main()
