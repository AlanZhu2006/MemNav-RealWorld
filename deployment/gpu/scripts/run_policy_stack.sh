#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
SESSION="${CEC_TMUX_SESSION:-cec-realworld}"
CEC_CAMERA_HEIGHT_M="${CEC_CAMERA_HEIGHT_M:?Set measured D435i optical-center height in metres}"
export CEC_CAMERA_HEIGHT_M
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
  "exec '$SCRIPT_DIR/run_memnav_server.sh' >'$CEC_OUT_ROOT/logs/memnav.log' 2>&1"
tmux new-window -t "$SESSION" -n navdp \
  "exec '$SCRIPT_DIR/run_navdp_server.sh' >'$CEC_OUT_ROOT/logs/navdp.log' 2>&1"
tmux new-window -t "$SESSION" -n hub \
  "exec '$SCRIPT_DIR/run_cec_hub.sh' >'$CEC_OUT_ROOT/logs/hub.log' 2>&1"

ready=false
for _ in $(seq 1 240); do
  if curl -fsS --max-time 1 "http://127.0.0.1:$CEC_HUB_PORT/healthz" \
      | python3 -c '
import json, sys
p = json.load(sys.stdin)
assert p.get("algo") == "cec_hybrid_navdp"
assert p.get("navigation_sensor_contract") == "causal_monocular_rgb_v1"
assert p.get("navdp_depth_source") == "monocular_sidecar"
assert p.get("metric_depth_sensor_consumed_by_policy") is False
' \
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
echo "  camera optical-center height: ${CEC_CAMERA_HEIGHT_M} m"
echo "  hub:    http://127.0.0.1:$CEC_HUB_PORT"
echo "  memnav: http://127.0.0.1:$MEMNAV_PORT"
echo "  navdp:  http://127.0.0.1:$NAVDP_PORT"
echo "  logs:   $CEC_OUT_ROOT/logs"
echo "  tmux:   tmux attach -t $SESSION"
