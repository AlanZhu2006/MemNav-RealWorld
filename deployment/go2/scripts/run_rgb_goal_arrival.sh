#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
GOAL_OVERRIDE=""
CONFIG_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --goal)
      [[ $# -ge 2 ]] || {
        echo "--goal requires a path" >&2
        exit 2
      }
      GOAL_OVERRIDE="$2"
      shift 2
      ;;
    *)
      CONFIG_ARGS+=("$1")
      shift
      ;;
  esac
done
navdp_require_config_arg "${CONFIG_ARGS[@]}"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros
navdp_activate_venv

GOAL="$CFG_ARRIVAL_GOAL"
if [[ -n "$GOAL_OVERRIDE" ]]; then
  GOAL="$(readlink -f "$GOAL_OVERRIDE")"
fi
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
