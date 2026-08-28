#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros
navdp_activate_venv

GOAL="${NAVDP_IMAGE_GOAL_PATH:-}"
[[ -f "$GOAL" ]] || {
  echo "RGB arrival ImageGoal missing: $GOAL" >&2
  exit 1
}

exec python "$NAVDP_GO2_DIR/rgb_goal_arrival.py" \
  --goal "$GOAL" \
  --rate-hz "${NAVDP_RGB_ARRIVAL_RATE_HZ:-6.0}" \
  --required-consecutive "${NAVDP_RGB_ARRIVAL_CONSECUTIVE:-3}" \
  --min-image-scale "${NAVDP_RGB_ARRIVAL_MIN_SCALE:-0.78}" \
  --max-image-scale "${NAVDP_RGB_ARRIVAL_MAX_SCALE:-1.25}"
