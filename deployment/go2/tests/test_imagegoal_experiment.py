import pathlib
import sys
import unittest

import numpy as np


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from imagegoal_experiment import (  # noqa: E402
    EpisodeTracker,
    PolicyStopTracker,
    StateSample,
    safety_abort_reason,
)


def sample(time_s, x, y, vx=0.0, vy=0.0, yaw=0.0):
    return StateSample(
        monotonic_s=time_s,
        position_xyz=np.array([x, y, 0.0], dtype=np.float64),
        velocity_xyz=np.array([vx, vy, 0.0], dtype=np.float64),
        yaw_rad=yaw,
    )


class EpisodeTrackerTests(unittest.TestCase):
    def test_requires_continuous_arrival_hold(self):
        tracker = EpisodeTracker(np.array([2.0, 0.0]), 0.0, success_hold_s=0.5)
        self.assertEqual(tracker.update(sample(0.0, 0.0, 0.0)), "running")
        self.assertEqual(tracker.update(sample(0.2, 0.4, 0.0)), "running")
        self.assertEqual(tracker.update(sample(0.4, 0.8, 0.0)), "running")
        self.assertEqual(tracker.update(sample(0.6, 1.2, 0.0, vx=0.1)), "running")
        self.assertEqual(tracker.update(sample(0.9, 1.3, 0.0, vx=0.1)), "running")
        self.assertEqual(tracker.update(sample(1.2, 1.3, 0.0, vx=0.1)), "success")
        self.assertTrue(tracker.metrics()["success"])

    def test_leaving_radius_resets_arrival_hold(self):
        tracker = EpisodeTracker(np.array([1.0, 0.0]), 0.0, success_hold_s=0.5)
        tracker.update(sample(0.0, 0.0, 0.0))
        tracker.update(sample(0.2, 0.4, 0.0))
        tracker.update(sample(0.5, 0.0, 0.0))
        self.assertEqual(tracker.update(sample(0.8, 0.4, 0.0)), "running")

    def test_arrival_speed_matches_original_l1_metric(self):
        tracker = EpisodeTracker(
            np.array([0.0, 0.0]),
            0.0,
            success_distance_m=0.85,
            success_speed_mps=0.30,
            success_hold_s=0.50,
        )
        self.assertEqual(
            tracker.update(sample(0.0, 0.1, 0.0, vx=0.20, vy=0.20)),
            "running",
        )
        self.assertEqual(
            tracker.update(sample(0.6, 0.1, 0.0, vx=0.20, vy=0.20)),
            "running",
        )

    def test_rejects_implausible_position_jump(self):
        tracker = EpisodeTracker(np.array([3.0, 0.0]), 0.0, max_position_jump_m=0.5)
        tracker.update(sample(0.0, 0.0, 0.0))
        self.assertEqual(tracker.update(sample(0.1, 1.0, 0.0)), "invalid")
        self.assertEqual(tracker.invalid_reason, "position_jump")

    def test_success_metrics_include_spl_and_yaw_error(self):
        tracker = EpisodeTracker(np.array([1.0, 0.0]), 0.5, success_hold_s=0.0)
        tracker.update(sample(0.0, 0.0, 0.0))
        tracker.update(sample(1.0, 0.5, 0.0, yaw=0.2))
        metrics = tracker.metrics()
        self.assertEqual(metrics["spl"], 1.0)
        self.assertAlmostEqual(metrics["final_yaw_error_rad"], 0.3)


class PolicyStopTrackerTests(unittest.TestCase):
    @staticmethod
    def active_status(**overrides):
        payload = {
            "backend": "navdp",
            "mode": "imagegoal",
            "enabled": True,
            "estop": False,
            "server_initialized": True,
            "last_error": "",
            "stop_reason": "clear",
            "cmd_vx": 0.0,
            "cmd_wz": 0.0,
        }
        payload.update(overrides)
        return payload

    def test_requires_sustained_zero_command_and_path(self):
        tracker = PolicyStopTracker(hold_s=1.0)
        tracker.update_status(self.active_status(), 1.0)
        tracker.update_path(np.zeros((24, 2)), 1.0)
        self.assertFalse(tracker.snapshot(1.8)["confirmed"])
        self.assertTrue(tracker.snapshot(2.1)["confirmed"])

    def test_nonzero_command_resets_hold(self):
        tracker = PolicyStopTracker(hold_s=1.0)
        tracker.update_status(self.active_status(), 1.0)
        tracker.update_path(np.zeros((24, 2)), 1.0)
        tracker.update_status(self.active_status(cmd_vx=0.2), 1.6)
        self.assertFalse(tracker.snapshot(1.7)["confirmed"])
        self.assertEqual(tracker.snapshot(1.7)["reason"], "navdp_linear_command_nonzero")

    def test_estop_or_obstacle_zero_does_not_count(self):
        for status, reason in (
            (self.active_status(estop=True, stop_reason="estop"), "navdp_estop_asserted"),
            (
                self.active_status(stop_reason="obstacle_stop"),
                "navdp_motion_blocked",
            ),
        ):
            with self.subTest(reason=reason):
                tracker = PolicyStopTracker(hold_s=0.0)
                tracker.update_path(np.zeros((24, 2)), 1.0)
                tracker.update_status(status, 1.0)
                snapshot = tracker.snapshot(1.0)
                self.assertFalse(snapshot["confirmed"])
                self.assertEqual(snapshot["reason"], reason)

    def test_nonzero_or_stale_path_does_not_count(self):
        tracker = PolicyStopTracker(hold_s=0.0, path_timeout_s=1.0)
        tracker.update_status(self.active_status(), 1.0)
        tracker.update_path(np.array([[0.0, 0.0], [0.5, 0.0]]), 1.0)
        self.assertEqual(tracker.snapshot(1.0)["reason"], "navdp_path_nonzero")
        tracker.update_path(np.zeros((24, 2)), 2.0)
        tracker.update_status(self.active_status(), 2.0)
        self.assertEqual(tracker.snapshot(3.1)["reason"], "navdp_path_stale")


class SafetyAbortTests(unittest.TestCase):
    def test_path_limit_is_fail_closed(self):
        reason = safety_abort_reason(
            {
                "path_length_m": 2.21,
                "initial_distance_m": 1.5,
                "current_distance_m": 1.2,
            },
            max_path_length_m=2.2,
            max_target_distance_regression_m=0.3,
        )
        self.assertEqual(reason, "path_length_limit")

    def test_target_regression_is_fail_closed(self):
        reason = safety_abort_reason(
            {
                "path_length_m": 0.8,
                "initial_distance_m": 1.5,
                "current_distance_m": 1.81,
            },
            max_path_length_m=2.2,
            max_target_distance_regression_m=0.3,
        )
        self.assertEqual(reason, "target_distance_regression")

    def test_disabled_or_in_bound_limits_do_not_abort(self):
        metrics = {
            "path_length_m": 2.3,
            "initial_distance_m": 1.5,
            "current_distance_m": 1.9,
        }
        self.assertEqual(safety_abort_reason(metrics), "")
        self.assertEqual(
            safety_abort_reason(
                metrics,
                max_path_length_m=2.5,
                max_target_distance_regression_m=0.5,
            ),
            "",
        )


if __name__ == "__main__":
    unittest.main()
