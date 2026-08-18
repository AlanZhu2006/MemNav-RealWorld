#!/usr/bin/env bash
set -euo pipefail

SESSION="${NAVDP_TMUX_SESSION:-navdp-go2}"
if ! command -v tmux >/dev/null 2>&1 || ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "NavDP tmux session is not running: $SESSION"
  exit 0
fi

tmux send-keys -t "$SESSION:adapter" C-c 2>/dev/null || true
tmux send-keys -t "$SESSION:go2" C-c 2>/dev/null || true
sleep 1
tmux kill-session -t "$SESSION"
echo "Stopped NavDP stack: $SESSION"
