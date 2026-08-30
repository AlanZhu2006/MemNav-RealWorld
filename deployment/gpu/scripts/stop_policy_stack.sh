#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
if [[ $# -ne 2 || "$1" != --config ]]; then
  echo "Usage: ${0##*/} --config RESOLVED_CONFIG.json" >&2
  exit 2
fi
RUN_CONFIG="$(readlink -f "$2")"
gpu_read_config "$RUN_CONFIG"
SESSION="$CFG_GPU_SESSION"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
  echo "Stopped tmux session $SESSION"
else
  echo "No tmux session named $SESSION"
fi
