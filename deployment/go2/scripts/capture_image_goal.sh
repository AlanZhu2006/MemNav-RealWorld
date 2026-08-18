#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros
navdp_activate_venv

RGB_TOPIC="${NAVDP_RGB_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${NAVDP_DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
OUTPUT="${NAVDP_IMAGE_GOAL_PATH:-$NAVDP_GO2_DIR/goals/image_goal.png}"
DEPTH_OUTPUT="${NAVDP_IMAGE_GOAL_DEPTH_PATH:-$NAVDP_GO2_DIR/goals/image_goal_depth.png}"

exec python "$NAVDP_GO2_DIR/capture_image_goal.py" \
  --rgb-topic "$RGB_TOPIC" \
  --depth-topic "$DEPTH_TOPIC" \
  --output "$OUTPUT" \
  --depth-output "$DEPTH_OUTPUT" \
  "$@"
