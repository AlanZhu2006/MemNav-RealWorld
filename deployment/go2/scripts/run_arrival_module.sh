#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE="${NAVDP_ARRIVAL_MODULE:-operator}"

case "$MODULE" in
  rgb|rgb-homography)
    exec "$SCRIPT_DIR/run_rgb_goal_arrival.sh"
    ;;
  operator|none)
    echo "Arrival module '$MODULE' has no background process." >&2
    echo "Do not create an arrival tmux window for operator termination." >&2
    exit 2
    ;;
  external|external-topic)
    echo "Arrival module '$MODULE' is supplied by an independent process." >&2
    echo "It must publish std_msgs/Bool on /navdp/arrival." >&2
    exit 2
    ;;
  *)
    echo "Unknown NAVDP_ARRIVAL_MODULE: $MODULE" >&2
    exit 2
    ;;
esac
