#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros
navdp_activate_venv

GOAL="$CFG_ARRIVAL_GOAL"
[[ -f "$GOAL" ]] || {
  echo "RGB arrival ImageGoal missing: $GOAL" >&2
  exit 1
}

exec python "$NAVDP_GO2_DIR/rgb_goal_arrival.py" \
  --goal "$GOAL" \
  --allowed-phases "$CFG_ARRIVAL_PHASES" \
  --rate-hz "$CFG_ARRIVAL_RATE_HZ" \
  --required-consecutive "$CFG_ARRIVAL_CONSECUTIVE" \
  --min-image-scale "$CFG_ARRIVAL_MIN_SCALE" \
  --max-image-scale "$CFG_ARRIVAL_MAX_SCALE"
