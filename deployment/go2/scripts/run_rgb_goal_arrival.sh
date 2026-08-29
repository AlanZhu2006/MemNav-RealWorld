#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros
navdp_activate_venv

GOAL="${NAVDP_ARRIVAL_GOAL_PATH:-${NAVDP_IMAGE_GOAL_PATH:-}}"
[[ -f "$GOAL" ]] || {
  echo "RGB arrival ImageGoal missing: $GOAL" >&2
  exit 1
}

exec python "$NAVDP_GO2_DIR/rgb_goal_arrival.py" \
  --goal "$GOAL" \
  --allowed-phases "${NAVDP_ARRIVAL_ALLOWED_PHASES:-memory_recording}" \
  --rate-hz "${NAVDP_RGB_ARRIVAL_RATE_HZ:-12.0}" \
  --required-consecutive "${NAVDP_RGB_ARRIVAL_CONSECUTIVE:-1}" \
  --min-image-scale "${NAVDP_RGB_ARRIVAL_MIN_SCALE:-0.60}" \
  --max-image-scale "${NAVDP_RGB_ARRIVAL_MAX_SCALE:-1.45}"
