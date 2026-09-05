from pathlib import Path
import sys
import json
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation_diagnostics import bearing_deg, depth_sectors, plan_diagnostics
from trajectory_control import trajectory_to_command


def test_attribution_detects_right_goal_left_near_path_without_mutating_input():
    path = np.array([[0., 0.], [.553, .216], [2.421, -.064]])
    before = path.copy()
    command = trajectory_to_command(path)
    result = plan_diagnostics(
        path, path[None, None], np.array([[-.4]]),
        {"memory_controller_pointgoal": [2.449, -.500]}, command,
        np.ones((480, 848)),
    )
    assert result["memory_bearing_deg"] < 0
    assert result["lookahead_bearing_deg"] > 0
    assert result["lookahead_minus_memory_bearing_deg"] > 20
    assert result["trajectory_command"]["wz"] > 0
    assert result["candidates_after_policy_postprocessing"][0]["critic"] == -.4
    np.testing.assert_array_equal(before, path)
    json.dumps(result, allow_nan=False)


def test_sector_depth_exposes_left_obstacle_hidden_from_center():
    depth = np.full((480, 848), 2.0)
    depth[:, :250] = .4
    sectors = depth_sectors(depth)
    assert sectors["left"]["p01_p10_p50_optical_z_m"][1] == pytest.approx(.4)
    assert sectors["center"]["p01_p10_p50_optical_z_m"][1] == 2.0


def test_invalid_depth_and_zero_path_are_explicit_not_fake_clearance():
    depth = np.full((40, 80), np.nan)
    path = np.zeros((24, 2))
    result = plan_diagnostics(path, [], [], {}, trajectory_to_command(path), depth)
    assert result["lookahead_bearing_deg"] is None
    assert result["memory_bearing_deg"] is None
    assert result["input_depth_sectors"]["left"]["valid_fraction"] == 0
    assert result["input_depth_sectors"]["left"]["p01_p10_p50_optical_z_m"] is None
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("point", [None, [0, 0], [np.nan, 1], [np.inf, 1]])
def test_unavailable_bearing_is_not_reported_as_forward(point):
    assert bearing_deg(point) is None
