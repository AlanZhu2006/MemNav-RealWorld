#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_GOAL="${NAVDP_IMAGE_GOAL_PATH:-$SCRIPT_DIR/../goals/image_goal.png}"
IMAGE_GOAL_DEPTH="${NAVDP_IMAGE_GOAL_DEPTH_PATH:-$SCRIPT_DIR/../goals/image_goal_depth.png}"
IMAGE_GOAL_POSE="${NAVDP_IMAGE_GOAL_POSE_PATH:-${IMAGE_GOAL%.*}_pose.json}"

bash "$SCRIPT_DIR/capture_image_goal.sh" "$@"
bash "$SCRIPT_DIR/run_imagegoal_evaluator.sh" capture \
  --image-goal "$IMAGE_GOAL" \
  --image-goal-depth "$IMAGE_GOAL_DEPTH" \
  --output "$IMAGE_GOAL_POSE"
