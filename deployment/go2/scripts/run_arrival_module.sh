#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
MODULE="$CFG_ARRIVAL_MODULE"

case "$MODULE" in
  rgb-homography)
    exec "$SCRIPT_DIR/run_rgb_goal_arrival.sh" --config "$NAVDP_RUN_CONFIG"
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
