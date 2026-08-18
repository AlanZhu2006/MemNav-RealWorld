#!/usr/bin/env bash
set -euo pipefail

SESSION="${NAVDP_TMUX_SESSION:-navdp-go2-offboard}"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
  echo "Stopped tmux session $SESSION"
else
  echo "No tmux session named $SESSION"
fi
