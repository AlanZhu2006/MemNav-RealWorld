from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from odin_occupancy_builder import (
    FREE_PIXEL,
    OCCUPIED_PIXEL,
    OccupancyEvidence,
    write_map,
)


class OdinOccupancyBuilderTests(unittest.TestCase):
    def test_rays_mark_free_space_and_height_band_marks_obstacle(self):
        evidence = OccupancyEvidence(
            resolution_m=0.1,
            obstacle_min_z_m=-0.3,
            obstacle_max_z_m=0.5,
            minimum_range_m=0.0,
            maximum_range_m=5.0,
            minimum_occupied_hits=1,
        )
        evidence.update((0.0, 0.0), [(1.0, 0.0, 0.0), (0.0, 1.0, -0.8)])

        image, _ = evidence.render(margin_m=0.1)

        self.assertGreater(int((image == FREE_PIXEL).sum()), 2)
        self.assertEqual(int((image == OCCUPIED_PIXEL).sum()), 1)

    def test_map_outputs_are_hash_receipted(self):
        evidence = OccupancyEvidence(
            resolution_m=0.1,
            obstacle_min_z_m=-0.3,
            obstacle_max_z_m=0.5,
            minimum_range_m=0.0,
            maximum_range_m=5.0,
            minimum_occupied_hits=1,
        )
        evidence.update((0.0, 0.0), [(1.0, 0.0, 0.0)])
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory) / "occupancy"

            receipt = write_map(
                evidence,
                prefix,
                margin_m=0.1,
                session_id="scene01",
                cloud_topic="/odin1/cloud_slam",
                odometry_topic="/odin1/odometry",
                cloud_frame="odom",
            )

            self.assertEqual(receipt["schema"], "memnav-odin1-occupancy-v1")
            self.assertIsNotNone(cv2.imread(str(prefix.with_suffix(".pgm"))))
            self.assertTrue(prefix.with_suffix(".yaml").is_file())
            self.assertTrue(prefix.with_suffix(".receipt.json").is_file())


if __name__ == "__main__":
    unittest.main()
