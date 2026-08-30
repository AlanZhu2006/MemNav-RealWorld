#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros
navdp_activate_venv

exec python "$NAVDP_GO2_DIR/capture_image_goal.py" \
  --rgb-topic "$CFG_RGB_TOPIC" \
  --depth-topic "$CFG_DEPTH_TOPIC" \
  "$@"
