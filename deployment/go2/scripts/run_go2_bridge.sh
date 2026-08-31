#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros

UNITREE_NET_IF="$CFG_UNITREE_NET_IF"
UNITREE_SDK2PY_PATH="$CFG_UNITREE_SDK_PATH"
CYCLONEDDS_HOME="$CFG_CYCLONEDDS_HOME"
GO2_PYTHON="$CFG_GO2_PYTHON"
GO2_CMD_TOPIC="$CFG_GO2_CMD_TOPIC"
GO2_TIMEOUT_SEC="$CFG_GO2_TIMEOUT_S"
GO2_MAX_VX="$CFG_GO2_MAX_VX"
GO2_MAX_VY="$CFG_GO2_MAX_VY"
GO2_MAX_WZ="$CFG_GO2_MAX_WZ"
GO2_MIN_CMD_V="$CFG_GO2_MIN_CMD_V"
GO2_MIN_CMD_W="$CFG_GO2_MIN_CMD_W"

if [[ ! -x "$GO2_PYTHON" ]]; then
  echo "Working Unitree Python environment not found: $GO2_PYTHON" >&2
  echo "Fix sites.jetson.unitree.python in deployment/config/system.json." >&2
  exit 1
fi
if [[ ! -d "$CYCLONEDDS_HOME/lib" ]]; then
  echo "CycloneDDS installation not found: $CYCLONEDDS_HOME" >&2
  exit 1
fi
export CYCLONEDDS_HOME
export CMAKE_PREFIX_PATH="$CYCLONEDDS_HOME${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "Starting NavDP Go2 bridge"
echo "  topic:     $GO2_CMD_TOPIC"
echo "  interface: $UNITREE_NET_IF"
echo "  timeout:   ${GO2_TIMEOUT_SEC}s"
echo "  limits:    vx=$GO2_MAX_VX vy=$GO2_MAX_VY wz=$GO2_MAX_WZ"
echo "  floors:    v=$GO2_MIN_CMD_V w=$GO2_MIN_CMD_W"
echo "  remote:    rt/lowstate priority enabled"
echo "  offline:   wait and reconnect without stopping the rest of the stack"

bridge_command=(
  "$GO2_PYTHON" "$NAVDP_GO2_DIR/go2_cmd_bridge.py"
  --net-if "$UNITREE_NET_IF"
  --sdk-path "$UNITREE_SDK2PY_PATH"
  --ros-args
  -p cmd_vel_topic:="$GO2_CMD_TOPIC"
  -p timeout_sec:="$GO2_TIMEOUT_SEC"
  -p max_vx:="$GO2_MAX_VX"
  -p max_vy:="$GO2_MAX_VY"
  -p max_wz:="$GO2_MAX_WZ"
  -p min_cmd_v:="$GO2_MIN_CMD_V"
  -p min_cmd_w:="$GO2_MIN_CMD_W"
  -p enabled:=true
  -p send_zero_when_idle:=false
  -p stop_once_on_release:=true
  -p remote_priority:=true
  -p remote_topic:=rt/lowstate
  -p remote_deadband:=0.12
  -p remote_hold_sec:=0.8
  -p log_commands:=true
  -p log_interval_sec:=0.5
)

last_offline_reason=""
while true; do
  offline_reason=""
  if ! ip link show dev "$UNITREE_NET_IF" >/dev/null 2>&1; then
    offline_reason="interface $UNITREE_NET_IF is missing"
  elif ! ip -o -4 addr show dev "$UNITREE_NET_IF" \
      | grep -q '192\.168\.123\.'; then
    offline_reason="interface $UNITREE_NET_IF has no 192.168.123.x address"
  elif [[ "$(cat "/sys/class/net/$UNITREE_NET_IF/carrier" 2>/dev/null || true)" != 1 ]]; then
    offline_reason="interface $UNITREE_NET_IF has no carrier"
  fi

  if [[ -n "$offline_reason" ]]; then
    if [[ "$offline_reason" != "$last_offline_reason" ]]; then
      echo "GO2 OFFLINE: $offline_reason; waiting" >&2
      last_offline_reason="$offline_reason"
    fi
    sleep 2
    continue
  fi

  last_offline_reason=""
  echo "Go2 network is available; connecting command bridge"
  set +e
  "${bridge_command[@]}"
  bridge_status=$?
  set -e
  echo "GO2 OFFLINE: command bridge exited with status $bridge_status; retrying" >&2
  sleep 2
done
