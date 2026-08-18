#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros

UNITREE_NET_IF="${UNITREE_NET_IF:-eth0}"
UNITREE_SDK2PY_PATH="${UNITREE_SDK2PY_PATH:-/home/nvidia/unitree_ws/src/unitree_sdk2_python}"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/nvidia/twork/cyclonedds/install}"
GO2_PYTHON="${GO2_PYTHON:-/home/nvidia/twork/tinynav/.venv/bin/python}"
GO2_CMD_TOPIC="${GO2_CMD_TOPIC:-/navdp/cmd_vel}"
GO2_TIMEOUT_SEC="${GO2_TIMEOUT_SEC:-0.35}"
GO2_MAX_VX="${GO2_MAX_VX:-0.30}"
GO2_MAX_VY="${GO2_MAX_VY:-0.0}"
GO2_MAX_WZ="${GO2_MAX_WZ:-0.60}"
GO2_MIN_CMD_V="${GO2_MIN_CMD_V:-0.10}"
GO2_MIN_CMD_W="${GO2_MIN_CMD_W:-0.20}"

if [[ ! -x "$GO2_PYTHON" ]]; then
  echo "Working Unitree Python environment not found: $GO2_PYTHON" >&2
  echo "Set GO2_PYTHON to an environment containing cyclonedds and unitree_sdk2py." >&2
  exit 1
fi
if [[ ! -d "$CYCLONEDDS_HOME/lib" ]]; then
  echo "CycloneDDS installation not found: $CYCLONEDDS_HOME" >&2
  exit 1
fi
if ! ip -o -4 addr show dev "$UNITREE_NET_IF" | grep -q '192\.168\.123\.'; then
  echo "No 192.168.123.x address on $UNITREE_NET_IF." >&2
  echo "Current machine setup:" >&2
  echo "  sudo ip link set $UNITREE_NET_IF up" >&2
  echo "  sudo ip addr replace 192.168.123.100/24 dev $UNITREE_NET_IF" >&2
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

exec "$GO2_PYTHON" "$NAVDP_GO2_DIR/go2_cmd_bridge.py" \
  --net-if "$UNITREE_NET_IF" \
  --sdk-path "$UNITREE_SDK2PY_PATH" \
  --ros-args \
  -p cmd_vel_topic:="$GO2_CMD_TOPIC" \
  -p timeout_sec:="$GO2_TIMEOUT_SEC" \
  -p max_vx:="$GO2_MAX_VX" \
  -p max_vy:="$GO2_MAX_VY" \
  -p max_wz:="$GO2_MAX_WZ" \
  -p min_cmd_v:="$GO2_MIN_CMD_V" \
  -p min_cmd_w:="$GO2_MIN_CMD_W" \
  -p enabled:=true \
  -p send_zero_when_idle:=false \
  -p stop_once_on_release:=true \
  -p remote_priority:=true \
  -p remote_topic:=rt/lowstate \
  -p remote_deadband:=0.12 \
  -p remote_hold_sec:=0.8 \
  -p log_commands:=true \
  -p log_interval_sec:=0.5
