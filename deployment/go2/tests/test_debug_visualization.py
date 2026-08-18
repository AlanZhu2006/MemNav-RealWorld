#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

GO2_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GO2_DIR))

from debug_visualization import ranked_candidates, score_rgb  # noqa: E402


class RankedCandidatesTests(unittest.TestCase):
    def test_squeezes_batch_and_ranks_scores(self) -> None:
        trajectories = np.array(
            [[
                [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                [[1.0, 1.0, 0.0], [2.0, 1.0, 0.0]],
                [[1.0, -1.0, 0.0], [2.0, -1.0, 0.0]],
            ]],
            dtype=np.float32,
        )
        candidates, scores = ranked_candidates(
            trajectories, np.array([[0.2, 1.5, -0.3]]), limit=2
        )
        self.assertEqual(candidates.shape, (2, 2, 3))
        np.testing.assert_allclose(scores, [1.5, 0.2])
        np.testing.assert_allclose(candidates[0, :, 1], [1.0, 1.0])

    def test_filters_nonfinite_paths(self) -> None:
        trajectories = np.array(
            [
                [[1.0, 0.0], [2.0, 0.0]],
                [[1.0, np.nan], [2.0, 1.0]],
            ],
            dtype=np.float32,
        )
        candidates, scores = ranked_candidates(trajectories, [0.4, 2.0], limit=8)
        self.assertEqual(candidates.shape, (1, 2, 2))
        np.testing.assert_allclose(scores, [0.4])

    def test_limit_zero_returns_no_candidates(self) -> None:
        candidates, scores = ranked_candidates(np.ones((2, 3, 2)), [1.0, 2.0], 0)
        self.assertEqual(candidates.shape[0], 0)
        self.assertEqual(scores.shape[0], 0)


class ScoreColorTests(unittest.TestCase):
    def test_high_score_is_red_and_low_score_is_blue(self) -> None:
        low = score_rgb(0.0, 0.0, 1.0)
        high = score_rgb(1.0, 0.0, 1.0)
        self.assertGreater(low[2], low[0])
        self.assertGreater(high[0], high[2])


if __name__ == "__main__":
    unittest.main()
