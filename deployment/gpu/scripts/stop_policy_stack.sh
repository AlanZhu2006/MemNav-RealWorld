#!/usr/bin/env bash
set -euo pipefail

SESSION="${CEC_TMUX_SESSION:-cec-realworld}"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
  echo "Stopped tmux session $SESSION"
else
  echo "No tmux session named $SESSION"
fi
