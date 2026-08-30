#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros
navdp_activate_venv

arrival_options=()
if [[ "$CFG_FOXGLOVE_PREVIEW_ARRIVAL_PRESERVE_RESOLUTION" == true ]]; then
  arrival_options+=(--arrival-preserve-resolution)
fi

exec "$CFG_JETSON_PYTHON" "$NAVDP_GO2_DIR/foxglove_image_relay.py" \
  --rgb-input "$CFG_RGB_TOPIC" \
  --depth-input "$CFG_DEPTH_TOPIC" \
  --rgb-output "$CFG_FOXGLOVE_PREVIEW_RGB_TOPIC" \
  --depth-output "$CFG_FOXGLOVE_PREVIEW_DEPTH_TOPIC" \
  --goal-input /navdp/image_goal \
  --arrival-input /navdp/rgb_arrival_debug \
  --status-input /navdp/status \
  --goal-output "$CFG_FOXGLOVE_PREVIEW_GOAL_TOPIC" \
  --arrival-output "$CFG_FOXGLOVE_PREVIEW_ARRIVAL_TOPIC" \
  --status-output "$CFG_FOXGLOVE_PREVIEW_STATUS_TOPIC" \
  --width "$CFG_FOXGLOVE_PREVIEW_WIDTH" \
  --height "$CFG_FOXGLOVE_PREVIEW_HEIGHT" \
  --rgb-fps "$CFG_FOXGLOVE_PREVIEW_RGB_FPS" \
  --depth-fps "$CFG_FOXGLOVE_PREVIEW_DEPTH_FPS" \
  --goal-fps "$CFG_FOXGLOVE_PREVIEW_GOAL_FPS" \
  --arrival-fps "$CFG_FOXGLOVE_PREVIEW_ARRIVAL_FPS" \
  --status-width "$CFG_FOXGLOVE_PREVIEW_STATUS_WIDTH" \
  --status-height "$CFG_FOXGLOVE_PREVIEW_STATUS_HEIGHT" \
  --status-fps "$CFG_FOXGLOVE_PREVIEW_STATUS_FPS" \
  --rgb-jpeg-quality "$CFG_FOXGLOVE_PREVIEW_RGB_JPEG_QUALITY" \
  --depth-jpeg-quality "$CFG_FOXGLOVE_PREVIEW_DEPTH_JPEG_QUALITY" \
  --goal-jpeg-quality "$CFG_FOXGLOVE_PREVIEW_GOAL_JPEG_QUALITY" \
  --arrival-jpeg-quality "$CFG_FOXGLOVE_PREVIEW_ARRIVAL_JPEG_QUALITY" \
  --status-jpeg-quality "$CFG_FOXGLOVE_PREVIEW_STATUS_JPEG_QUALITY" \
  --depth-min-mm "$CFG_FOXGLOVE_PREVIEW_DEPTH_MIN_MM" \
  --depth-max-mm "$CFG_FOXGLOVE_PREVIEW_DEPTH_MAX_MM" \
  "${arrival_options[@]}"
