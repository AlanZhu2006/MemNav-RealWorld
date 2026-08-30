#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros

PARAMS="$NAVDP_GO2_DIR/config/foxglove_bridge.yaml"
[[ -f "$PARAMS" ]] || {
  echo "Foxglove bridge parameters not found: $PARAMS" >&2
  exit 1
}
[[ -f "$CFG_FOXGLOVE_LAYOUT" ]] || {
  echo "Foxglove layout not found: $CFG_FOXGLOVE_LAYOUT" >&2
  exit 1
}
ros2 pkg prefix foxglove_bridge >/dev/null 2>&1 || {
  echo "foxglove_bridge is not installed" >&2
  exit 1
}

TF_LOG="$CFG_JETSON_RUNTIME_ROOT/logs/foxglove_tf.log"
mkdir -p "$(dirname "$TF_LOG")"
ros2 run tf2_ros static_transform_publisher \
  --frame-id navdp_local --child-frame-id base_link >"$TF_LOG" 2>&1 &
TF_PID=$!
BRIDGE_PID=""
cleanup() {
  if [[ -n "$BRIDGE_PID" ]]; then
    kill -TERM "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
  kill -TERM "$TF_PID" 2>/dev/null || true
  wait "$TF_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting read-only Foxglove Bridge"
echo "  websocket: ws://$CFG_FOXGLOVE_ADDRESS:$CFG_FOXGLOVE_PORT"
echo "  layout:    $CFG_FOXGLOVE_LAYOUT"
echo "  control:   client publish, services and parameter mutation disabled"

ros2 run foxglove_bridge foxglove_bridge --ros-args \
  --params-file "$PARAMS" \
  -p address:="$CFG_FOXGLOVE_ADDRESS" \
  -p port:="$CFG_FOXGLOVE_PORT" &
BRIDGE_PID=$!
wait "$BRIDGE_PID"
