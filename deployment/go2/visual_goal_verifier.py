#!/usr/bin/env python3
"""Independent RGB-D view matching for NavDP ImageGoal arrival verification."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import cv2
import numpy as np

from image_goal_io import validate_depth_m, validate_rgb_image


@dataclass(frozen=True)
class VisualMatchResult:
    matched: bool
    confirmed: bool
    reason: str
    consecutive_matches: int
    goal_object_matched: bool
    goal_object_confirmed: bool
    goal_object_reason: str
    consecutive_goal_object_matches: int
    target_keypoints: int
    current_keypoints: int
    good_matches: int
    inliers: int
    inlier_ratio: float
    target_coverage: float
    current_coverage: float
    center_offset_norm: float
    image_scale: float
    rotation_deg: float
    reprojection_error_px: float
    depth_pairs: int
    median_depth_error_m: float
    median_depth_delta_m: float
    depth_delta_mad_m: float

    def to_dict(self) -> dict:
        payload = asdict(self)
        for key, value in payload.items():
            if isinstance(value, float):
                payload[key] = round(value, 4) if math.isfinite(value) else None
        return payload


class VisualGoalVerifier:
    def __init__(
        self,
        target_rgb: np.ndarray,
        target_depth_m: np.ndarray,
        image_width: int = 480,
        ratio_test: float = 0.72,
        min_good_matches: int = 30,
        min_inliers: int = 20,
        min_inlier_ratio: float = 0.45,
        min_coverage: float = 0.08,
        max_center_offset_norm: float = 0.18,
        min_image_scale: float = 0.70,
        max_image_scale: float = 1.45,
        max_rotation_deg: float = 25.0,
        max_reprojection_error_px: float = 3.0,
        min_depth_pairs: int = 12,
        max_median_depth_error_m: float = 0.40,
        required_consecutive_matches: int = 3,
        goal_object_min_good_matches: int = 30,
        goal_object_min_inliers: int = 20,
        goal_object_min_inlier_ratio: float = 0.45,
        goal_object_min_coverage: float = 0.02,
        goal_object_max_center_offset_norm: float = 0.45,
        goal_object_min_image_scale: float = 0.55,
        goal_object_max_image_scale: float = 2.25,
        goal_object_max_rotation_deg: float = 30.0,
        goal_object_max_reprojection_error_px: float = 3.0,
        goal_object_min_depth_pairs: int = 12,
        goal_object_min_depth_delta_m: float = -1.25,
        goal_object_max_depth_delta_m: float = 0.25,
        goal_object_max_depth_delta_mad_m: float = 0.20,
        goal_object_required_consecutive_matches: int = 3,
    ) -> None:
        if not hasattr(cv2, "SIFT_create"):
            raise RuntimeError("OpenCV SIFT support is required for visual goal verification")
        self.image_width = max(160, int(image_width))
        self.ratio_test = float(ratio_test)
        self.min_good_matches = int(min_good_matches)
        self.min_inliers = int(min_inliers)
        self.min_inlier_ratio = float(min_inlier_ratio)
        self.min_coverage = float(min_coverage)
        self.max_center_offset_norm = float(max_center_offset_norm)
        self.min_image_scale = float(min_image_scale)
        self.max_image_scale = float(max_image_scale)
        self.max_rotation_deg = float(max_rotation_deg)
        self.max_reprojection_error_px = float(max_reprojection_error_px)
        self.min_depth_pairs = int(min_depth_pairs)
        self.max_median_depth_error_m = float(max_median_depth_error_m)
        self.required_consecutive_matches = max(1, int(required_consecutive_matches))
        self.goal_object_min_good_matches = int(goal_object_min_good_matches)
        self.goal_object_min_inliers = int(goal_object_min_inliers)
        self.goal_object_min_inlier_ratio = float(goal_object_min_inlier_ratio)
        self.goal_object_min_coverage = float(goal_object_min_coverage)
        self.goal_object_max_center_offset_norm = float(
            goal_object_max_center_offset_norm
        )
        self.goal_object_min_image_scale = float(goal_object_min_image_scale)
        self.goal_object_max_image_scale = float(goal_object_max_image_scale)
        self.goal_object_max_rotation_deg = float(goal_object_max_rotation_deg)
        self.goal_object_max_reprojection_error_px = float(
            goal_object_max_reprojection_error_px
        )
        self.goal_object_min_depth_pairs = int(goal_object_min_depth_pairs)
        self.goal_object_min_depth_delta_m = float(goal_object_min_depth_delta_m)
        self.goal_object_max_depth_delta_m = float(goal_object_max_depth_delta_m)
        self.goal_object_max_depth_delta_mad_m = float(
            goal_object_max_depth_delta_mad_m
        )
        self.goal_object_required_consecutive_matches = max(
            1, int(goal_object_required_consecutive_matches)
        )
        self.detector = cv2.SIFT_create(nfeatures=1200, contrastThreshold=0.025)
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self.target_rgb, self.target_depth_m = self._resize_pair(
            target_rgb, target_depth_m
        )
        target_gray = cv2.cvtColor(self.target_rgb, cv2.COLOR_RGB2GRAY)
        self.target_keypoints, self.target_descriptors = self.detector.detectAndCompute(
            target_gray, None
        )
        if (
            self.target_descriptors is None
            or len(self.target_keypoints) < self.min_good_matches
        ):
            raise ValueError(
                "image goal has too little texture for reliable visual verification: "
                f"{len(self.target_keypoints)} SIFT features"
            )
        target_height, target_width = self.target_depth_m.shape
        target_points = np.float32(
            [keypoint.pt for keypoint in self.target_keypoints]
        )
        self.target_feature_coverage = self._coverage(
            target_points, target_width, target_height
        )
        if self.target_feature_coverage < self.min_coverage:
            raise ValueError(
                "image goal features are too spatially concentrated for reliable "
                f"visual verification: coverage={self.target_feature_coverage:.3f}"
            )
        valid_target_depth_features = 0
        for keypoint in self.target_keypoints:
            x = int(np.clip(round(keypoint.pt[0]), 0, target_width - 1))
            y = int(np.clip(round(keypoint.pt[1]), 0, target_height - 1))
            if 0.1 <= float(self.target_depth_m[y, x]) <= 5.0:
                valid_target_depth_features += 1
        if valid_target_depth_features < self.min_depth_pairs:
            raise ValueError(
                "image goal has too little valid aligned depth for visual verification: "
                f"{valid_target_depth_features} textured depth samples"
            )
        self.valid_target_depth_features = valid_target_depth_features
        self.consecutive_matches = 0
        self.consecutive_goal_object_matches = 0
        self.last_debug_rgb = self._side_by_side(self.target_rgb, self.target_rgb)

    def reset(self) -> None:
        self.consecutive_matches = 0
        self.consecutive_goal_object_matches = 0

    def settings(self) -> dict:
        return {
            "image_width": self.image_width,
            "ratio_test": self.ratio_test,
            "min_good_matches": self.min_good_matches,
            "min_inliers": self.min_inliers,
            "min_inlier_ratio": self.min_inlier_ratio,
            "min_coverage": self.min_coverage,
            "max_center_offset_norm": self.max_center_offset_norm,
            "min_image_scale": self.min_image_scale,
            "max_image_scale": self.max_image_scale,
            "max_rotation_deg": self.max_rotation_deg,
            "max_reprojection_error_px": self.max_reprojection_error_px,
            "min_depth_pairs": self.min_depth_pairs,
            "max_median_depth_error_m": self.max_median_depth_error_m,
            "required_consecutive_matches": self.required_consecutive_matches,
            "goal_object_min_good_matches": self.goal_object_min_good_matches,
            "goal_object_min_inliers": self.goal_object_min_inliers,
            "goal_object_min_inlier_ratio": self.goal_object_min_inlier_ratio,
            "goal_object_min_coverage": self.goal_object_min_coverage,
            "goal_object_max_center_offset_norm": (
                self.goal_object_max_center_offset_norm
            ),
            "goal_object_min_image_scale": self.goal_object_min_image_scale,
            "goal_object_max_image_scale": self.goal_object_max_image_scale,
            "goal_object_max_rotation_deg": self.goal_object_max_rotation_deg,
            "goal_object_max_reprojection_error_px": (
                self.goal_object_max_reprojection_error_px
            ),
            "goal_object_min_depth_pairs": self.goal_object_min_depth_pairs,
            "goal_object_min_depth_delta_m": self.goal_object_min_depth_delta_m,
            "goal_object_max_depth_delta_m": self.goal_object_max_depth_delta_m,
            "goal_object_max_depth_delta_mad_m": (
                self.goal_object_max_depth_delta_mad_m
            ),
            "goal_object_required_consecutive_matches": (
                self.goal_object_required_consecutive_matches
            ),
        }

    def _resize_pair(
        self, rgb: np.ndarray, depth_m: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        image = validate_rgb_image(rgb)
        depth = validate_depth_m(depth_m)
        if image.shape[:2] != depth.shape:
            raise ValueError(
                f"RGB and aligned depth shapes differ: {image.shape[:2]} vs {depth.shape}"
            )
        scale = self.image_width / float(image.shape[1])
        height = max(1, int(round(image.shape[0] * scale)))
        size = (self.image_width, height)
        return (
            cv2.resize(image, size, interpolation=cv2.INTER_AREA),
            cv2.resize(depth, size, interpolation=cv2.INTER_NEAREST),
        )

    @staticmethod
    def _coverage(points: np.ndarray, width: int, height: int) -> float:
        if len(points) < 3:
            return 0.0
        hull = cv2.convexHull(points.astype(np.float32))
        return float(cv2.contourArea(hull) / max(1.0, float(width * height)))

    @staticmethod
    def _side_by_side(first: np.ndarray, second: np.ndarray) -> np.ndarray:
        if first.shape[0] != second.shape[0]:
            second = cv2.resize(second, (second.shape[1], first.shape[0]))
        return np.concatenate((first, second), axis=1)

    def _empty_result(
        self, reason: str, current_keypoints: int, good_matches: int = 0
    ) -> VisualMatchResult:
        self.consecutive_matches = 0
        self.consecutive_goal_object_matches = 0
        return VisualMatchResult(
            matched=False,
            confirmed=False,
            reason=reason,
            consecutive_matches=0,
            goal_object_matched=False,
            goal_object_confirmed=False,
            goal_object_reason=reason,
            consecutive_goal_object_matches=0,
            target_keypoints=len(self.target_keypoints),
            current_keypoints=current_keypoints,
            good_matches=good_matches,
            inliers=0,
            inlier_ratio=0.0,
            target_coverage=0.0,
            current_coverage=0.0,
            center_offset_norm=math.inf,
            image_scale=0.0,
            rotation_deg=math.inf,
            reprojection_error_px=math.inf,
            depth_pairs=0,
            median_depth_error_m=math.inf,
            median_depth_delta_m=math.inf,
            depth_delta_mad_m=math.inf,
        )

    def evaluate(
        self, current_rgb: np.ndarray, current_depth_m: np.ndarray
    ) -> VisualMatchResult:
        current_rgb, current_depth = self._resize_pair(current_rgb, current_depth_m)
        if current_rgb.shape != self.target_rgb.shape:
            current_rgb = cv2.resize(
                current_rgb,
                (self.target_rgb.shape[1], self.target_rgb.shape[0]),
                interpolation=cv2.INTER_AREA,
            )
            current_depth = cv2.resize(
                current_depth,
                (self.target_rgb.shape[1], self.target_rgb.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        current_gray = cv2.cvtColor(current_rgb, cv2.COLOR_RGB2GRAY)
        current_keypoints, current_descriptors = self.detector.detectAndCompute(
            current_gray, None
        )
        self.last_debug_rgb = self._side_by_side(self.target_rgb, current_rgb)
        if current_descriptors is None or len(current_keypoints) < 2:
            return self._empty_result("insufficient_current_features", len(current_keypoints))

        candidate_pairs = self.matcher.knnMatch(
            self.target_descriptors, current_descriptors, k=2
        )
        good_matches = [
            first
            for pair in candidate_pairs
            if len(pair) == 2
            for first, second in [pair]
            if first.distance < self.ratio_test * second.distance
        ]
        if len(good_matches) < min(
            self.min_good_matches, self.goal_object_min_good_matches
        ):
            return self._empty_result(
                "insufficient_good_matches", len(current_keypoints), len(good_matches)
            )

        target_points = np.float32(
            [self.target_keypoints[match.queryIdx].pt for match in good_matches]
        )
        current_points = np.float32(
            [current_keypoints[match.trainIdx].pt for match in good_matches]
        )
        method = getattr(cv2, "USAC_MAGSAC", cv2.RANSAC)
        homography, mask = cv2.findHomography(
            target_points, current_points, method, 3.0
        )
        if homography is None or mask is None:
            return self._empty_result(
                "homography_failed", len(current_keypoints), len(good_matches)
            )
        inlier_mask = mask.reshape(-1).astype(bool)
        inlier_matches = [
            match for match, keep in zip(good_matches, inlier_mask) if keep
        ]
        inlier_target = target_points[inlier_mask]
        inlier_current = current_points[inlier_mask]
        inliers = int(inlier_mask.sum())
        if inliers < 4:
            return self._empty_result(
                "insufficient_inliers", len(current_keypoints), len(good_matches)
            )
        inlier_ratio = inliers / max(1, len(good_matches))
        height, width = self.target_rgb.shape[:2]
        target_coverage = self._coverage(inlier_target, width, height)
        current_coverage = self._coverage(inlier_current, width, height)

        projected = cv2.perspectiveTransform(
            inlier_target.reshape(-1, 1, 2), homography
        ).reshape(-1, 2)
        reprojection_error = float(
            np.median(np.linalg.norm(projected - inlier_current, axis=1))
        )
        corners = np.float32(
            [[0.0, 0.0], [width - 1.0, 0.0], [width - 1.0, height - 1.0], [0.0, height - 1.0]]
        ).reshape(-1, 1, 2)
        warped_corners = cv2.perspectiveTransform(corners, homography).reshape(-1, 2)
        if not np.isfinite(warped_corners).all():
            return self._empty_result(
                "invalid_homography", len(current_keypoints), len(good_matches)
            )
        image_center = np.array([(width - 1.0) / 2.0, (height - 1.0) / 2.0])
        warped_center = warped_corners.mean(axis=0)
        center_offset = float(
            np.linalg.norm(warped_center - image_center) / math.hypot(width, height)
        )
        warped_area = abs(float(cv2.contourArea(warped_corners.astype(np.float32))))
        image_scale = math.sqrt(warped_area / max(1.0, float(width * height)))
        top_edge = warped_corners[1] - warped_corners[0]
        rotation_deg = abs(math.degrees(math.atan2(float(top_edge[1]), float(top_edge[0]))))

        target_depth_values = []
        current_depth_values = []
        for target_point, current_point in zip(inlier_target, inlier_current):
            target_x = int(np.clip(round(float(target_point[0])), 0, width - 1))
            target_y = int(np.clip(round(float(target_point[1])), 0, height - 1))
            current_x = int(np.clip(round(float(current_point[0])), 0, width - 1))
            current_y = int(np.clip(round(float(current_point[1])), 0, height - 1))
            target_depth = float(self.target_depth_m[target_y, target_x])
            observed_depth = float(current_depth[current_y, current_x])
            if 0.1 <= target_depth <= 5.0 and 0.1 <= observed_depth <= 5.0:
                target_depth_values.append(target_depth)
                current_depth_values.append(observed_depth)
        depth_pairs = len(target_depth_values)
        median_depth_error = (
            float(
                np.median(
                    np.abs(
                        np.asarray(target_depth_values)
                        - np.asarray(current_depth_values)
                    )
                )
            )
            if depth_pairs
            else math.inf
        )
        depth_deltas = np.asarray(current_depth_values) - np.asarray(
            target_depth_values
        )
        median_depth_delta = (
            float(np.median(depth_deltas)) if depth_pairs else math.inf
        )
        depth_delta_mad = (
            float(np.median(np.abs(depth_deltas - median_depth_delta)))
            if depth_pairs
            else math.inf
        )

        exact_view_checks = (
            (len(good_matches) >= self.min_good_matches, "insufficient_good_matches"),
            (inliers >= self.min_inliers, "insufficient_inliers"),
            (inlier_ratio >= self.min_inlier_ratio, "low_inlier_ratio"),
            (
                min(target_coverage, current_coverage) >= self.min_coverage,
                "insufficient_image_coverage",
            ),
            (center_offset <= self.max_center_offset_norm, "view_center_mismatch"),
            (
                self.min_image_scale <= image_scale <= self.max_image_scale,
                "view_scale_mismatch",
            ),
            (rotation_deg <= self.max_rotation_deg, "view_rotation_mismatch"),
            (
                reprojection_error <= self.max_reprojection_error_px,
                "high_reprojection_error",
            ),
            (depth_pairs >= self.min_depth_pairs, "insufficient_depth_pairs"),
            (
                median_depth_error <= self.max_median_depth_error_m,
                "depth_mismatch",
            ),
        )
        reason = "matched"
        matched = True
        for passed, failure_reason in exact_view_checks:
            if not passed:
                matched = False
                reason = failure_reason
                break
        self.consecutive_matches = self.consecutive_matches + 1 if matched else 0
        confirmed = self.consecutive_matches >= self.required_consecutive_matches

        goal_object_checks = (
            (
                len(good_matches) >= self.goal_object_min_good_matches,
                "insufficient_goal_object_matches",
            ),
            (
                inliers >= self.goal_object_min_inliers,
                "insufficient_goal_object_inliers",
            ),
            (
                inlier_ratio >= self.goal_object_min_inlier_ratio,
                "low_goal_object_inlier_ratio",
            ),
            (
                min(target_coverage, current_coverage)
                >= self.goal_object_min_coverage,
                "insufficient_goal_object_coverage",
            ),
            (
                center_offset <= self.goal_object_max_center_offset_norm,
                "goal_object_off_center",
            ),
            (
                self.goal_object_min_image_scale
                <= image_scale
                <= self.goal_object_max_image_scale,
                "goal_object_scale_mismatch",
            ),
            (
                rotation_deg <= self.goal_object_max_rotation_deg,
                "goal_object_rotation_mismatch",
            ),
            (
                reprojection_error <= self.goal_object_max_reprojection_error_px,
                "goal_object_reprojection_error",
            ),
            (
                depth_pairs >= self.goal_object_min_depth_pairs,
                "insufficient_goal_object_depth_pairs",
            ),
            (
                median_depth_delta >= self.goal_object_min_depth_delta_m,
                "goal_object_too_close",
            ),
            (
                median_depth_delta <= self.goal_object_max_depth_delta_m,
                "goal_object_farther_than_reference",
            ),
            (
                depth_delta_mad <= self.goal_object_max_depth_delta_mad_m,
                "inconsistent_goal_object_depth",
            ),
        )
        goal_object_reason = "goal_object_matched"
        goal_object_matched = True
        for passed, failure_reason in goal_object_checks:
            if not passed:
                goal_object_matched = False
                goal_object_reason = failure_reason
                break
        self.consecutive_goal_object_matches = (
            self.consecutive_goal_object_matches + 1 if goal_object_matched else 0
        )
        goal_object_confirmed = (
            self.consecutive_goal_object_matches
            >= self.goal_object_required_consecutive_matches
        )

        debug_bgr = cv2.drawMatches(
            cv2.cvtColor(self.target_rgb, cv2.COLOR_RGB2BGR),
            self.target_keypoints,
            cv2.cvtColor(current_rgb, cv2.COLOR_RGB2BGR),
            current_keypoints,
            inlier_matches[:80],
            None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
        )
        color = (
            (0, 180, 0)
            if matched
            else ((0, 165, 255) if goal_object_matched else (0, 0, 220))
        )
        cv2.putText(
            debug_bgr,
            f"object={goal_object_reason} seq={self.consecutive_goal_object_matches}",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_bgr,
            f"view={reason} inliers={inliers} depth_delta={median_depth_delta:+.2f}m",
            (10, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
        self.last_debug_rgb = cv2.cvtColor(debug_bgr, cv2.COLOR_BGR2RGB)
        return VisualMatchResult(
            matched=matched,
            confirmed=confirmed,
            reason=reason,
            consecutive_matches=self.consecutive_matches,
            goal_object_matched=goal_object_matched,
            goal_object_confirmed=goal_object_confirmed,
            goal_object_reason=goal_object_reason,
            consecutive_goal_object_matches=(
                self.consecutive_goal_object_matches
            ),
            target_keypoints=len(self.target_keypoints),
            current_keypoints=len(current_keypoints),
            good_matches=len(good_matches),
            inliers=inliers,
            inlier_ratio=inlier_ratio,
            target_coverage=target_coverage,
            current_coverage=current_coverage,
            center_offset_norm=center_offset,
            image_scale=image_scale,
            rotation_deg=rotation_deg,
            reprojection_error_px=reprojection_error,
            depth_pairs=depth_pairs,
            median_depth_error_m=median_depth_error,
            median_depth_delta_m=median_depth_delta,
            depth_delta_mad_m=depth_delta_mad,
        )
