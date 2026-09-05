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
MODULE="$CFG_ARRIVAL_MODULE"

case "$MODULE" in
  rgb-homography)
    EXTRA_ARGS=()
    if [[ -n "$GOAL_OVERRIDE" ]]; then
      EXTRA_ARGS+=(--goal "$GOAL_OVERRIDE")
    fi
    exec "$SCRIPT_DIR/run_rgb_goal_arrival.sh" \
      --config "$NAVDP_RUN_CONFIG" "${EXTRA_ARGS[@]}"
    ;;
  operator)
    echo "Arrival module '$MODULE' has no background process." >&2
    echo "Do not create an arrival tmux window for operator termination." >&2
    exit 2
    ;;
  external-topic)
    echo "Arrival module '$MODULE' is supplied by an independent process." >&2
    echo "It must publish std_msgs/Bool on /navdp/arrival." >&2
    exit 2
    ;;
  *)
    echo "Unknown arrival module in resolved config: $MODULE" >&2
    exit 2
    ;;
esac
