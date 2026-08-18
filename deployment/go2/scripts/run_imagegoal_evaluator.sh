#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros

UNITREE_NET_IF="${UNITREE_NET_IF:-eth0}"
UNITREE_SDK2PY_PATH="${UNITREE_SDK2PY_PATH:-/home/nvidia/unitree_ws/src/unitree_sdk2_python}"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/nvidia/twork/cyclonedds/install}"
GO2_PYTHON="${GO2_PYTHON:-/home/nvidia/twork/tinynav/.venv/bin/python}"

if [[ ! -x "$GO2_PYTHON" ]]; then
  echo "Working Unitree Python environment not found: $GO2_PYTHON" >&2
  exit 1
fi
if [[ ! -d "$CYCLONEDDS_HOME/lib" ]]; then
  echo "CycloneDDS installation not found: $CYCLONEDDS_HOME" >&2
  exit 1
fi
if ! ip -o -4 addr show dev "$UNITREE_NET_IF" | grep -q '192\.168\.123\.'; then
  echo "No 192.168.123.x address on $UNITREE_NET_IF." >&2
  exit 1
fi

export CYCLONEDDS_HOME
export CMAKE_PREFIX_PATH="$CYCLONEDDS_HOME${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec "$GO2_PYTHON" "$NAVDP_GO2_DIR/imagegoal_experiment.py" \
  --net-if "$UNITREE_NET_IF" \
  --sdk-path "$UNITREE_SDK2PY_PATH" \
  "$@"
