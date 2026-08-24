import math

import numpy as np

from deployment.gpu.audit_visual_convergence import visual_convergence_metrics


def grid_points():
    x, y = np.meshgrid(
        np.linspace(80.0, 720.0, 6), np.linspace(60.0, 420.0, 5)
    )
    return np.stack([x.reshape(-1), y.reshape(-1)], axis=1)


def test_identity_views_have_zero_scale_free_deformation():
    points = grid_points()
    result = visual_convergence_metrics(
        points, points.copy(), np.ones(len(points)), (480, 848), (480, 848)
    )
    assert result["identity_flow_median_diag"] == 0.0
    assert result["homography_valid"] is True
    assert result["homography_corner_identity_max_diag"] < 1e-6
    assert result["affine_valid"] is True
    assert result["affine_corner_identity_max_diag"] < 1e-6
    assert result["affine_scale"] == 1.0
    assert abs(result["affine_rotation_deg"]) < 1e-8


def test_translation_remains_visible_after_robust_fitting():
    points = grid_points()
    shifted = points + np.asarray([84.8, -48.0])
    result = visual_convergence_metrics(
        points, shifted, np.ones(len(points)), (480, 848), (480, 848)
    )
    expected_flow = math.hypot(0.1, -0.1) / math.sqrt(2.0)
    assert result["identity_flow_median_diag"] == pytest.approx(expected_flow)
    assert result["homography_corner_identity_max_diag"] > 0.09
    assert result["affine_corner_identity_max_diag"] > 0.09
    assert result["affine_median_transfer_diag"] < 1e-6


def test_empty_correspondences_fail_closed_without_nan():
    empty = np.empty((0, 2), dtype=np.float64)
    result = visual_convergence_metrics(
        empty, empty.copy(), np.empty((0,)), (480, 848), (480, 848)
    )
    assert result["matches"] == 0
    assert result["identity_flow_median_diag"] is None
    assert result["homography_valid"] is False
    assert result["affine_valid"] is False


# Imported last so the first test remains readable as a pure geometry check.
import pytest
