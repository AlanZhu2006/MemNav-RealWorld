#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
gpu_require_config "$@"
GO2_DIR="$REPO_ROOT/deployment/go2"
source "$GO2_DIR/offboard/runtime_contract.sh"
SESSION="$CFG_GPU_SESSION"
require_executable tmux

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  exit 1
fi
for port in "$MEMNAV_PORT" "$NAVDP_PORT" "$CEC_HUB_PORT"; do
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)$port$"; then
    echo "port already in use: $port" >&2
    exit 1
  fi
done
mkdir -p "$CEC_OUT_ROOT/logs" "$CEC_OUT_ROOT/buffer"

tmux new-session -d -s "$SESSION" -n memnav \
  "exec '$SCRIPT_DIR/run_memnav_server.sh' --config '$RUN_CONFIG' >'$CEC_OUT_ROOT/logs/memnav.log' 2>&1"
tmux new-window -t "$SESSION" -n navdp \
  "exec '$SCRIPT_DIR/run_navdp_server.sh' --config '$RUN_CONFIG' >'$CEC_OUT_ROOT/logs/navdp.log' 2>&1"
tmux new-window -t "$SESSION" -n hub \
  "exec '$SCRIPT_DIR/run_cec_hub.sh' --config '$RUN_CONFIG' >'$CEC_OUT_ROOT/logs/hub.log' 2>&1"
tmux set-environment -t "$SESSION" MEMNAV_RUN_CONFIG "$RUN_CONFIG"
tmux set-environment -t "$SESSION" MEMNAV_CONFIG_ID "$CFG_CONFIG_ID"

ready=false
for _ in $(seq 1 "$CFG_GPU_READY_TIMEOUT_S"); do
  health="$(curl -fsS --max-time 1 \
      "http://127.0.0.1:$CEC_HUB_PORT/healthz" 2>/dev/null || true)"
  if [[ -n "$health" ]] \
      && cec_validate_health_contract "$health" "$GO2_DIR" \
      && ss -ltn | awk '{print $4}' | grep -Eq "(^|:)$MEMNAV_PORT$" \
      && ss -ltn | awk '{print $4}' | grep -Eq "(^|:)$NAVDP_PORT$"; then
    ready=true
    break
  fi
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    break
  fi
  sleep 1
done
if [[ "$ready" != true ]]; then
  echo "CEC policy stack failed to become ready" >&2
  for log in "$CEC_OUT_ROOT"/logs/*.log; do
    echo "===== $log" >&2
    tail -n 100 "$log" >&2 || true
  done
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  exit 1
fi

echo "CEC real-world policy stack ready"
echo "  sensor: causal monocular RGB (client depth is local safety only)"
echo "  config: $RUN_CONFIG"
echo "  config_id: $CFG_CONFIG_ID"
echo "  camera optical-center height: ${CFG_CAMERA_HEIGHT_M} m"
echo "  hub:    http://127.0.0.1:$CEC_HUB_PORT"
echo "  memnav: http://127.0.0.1:$MEMNAV_PORT"
echo "  navdp:  http://127.0.0.1:$NAVDP_PORT"
echo "  logs:   $CEC_OUT_ROOT/logs"
echo "  tmux:   tmux attach -t $SESSION"
