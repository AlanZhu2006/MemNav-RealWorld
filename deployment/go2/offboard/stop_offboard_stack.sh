#!/usr/bin/env bash
set -euo pipefail

OFFBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$OFFBOARD_DIR/.." && pwd)"
source "$GO2_DIR/scripts/common.sh"
navdp_require_config_arg "$@"
navdp_read_config "$NAVDP_RUN_CONFIG"
SESSION="$CFG_FULLMONO_SESSION"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  navdp_lock_motion_before_shutdown
  tmux send-keys -t "$SESSION:adapter" C-c 2>/dev/null || true
  tmux send-keys -t "$SESSION:go2" C-c 2>/dev/null || true
  sleep 1
  tmux kill-session -t "$SESSION"
  echo "Stopped tmux session $SESSION"
else
  echo "No tmux session named $SESSION"
fi
