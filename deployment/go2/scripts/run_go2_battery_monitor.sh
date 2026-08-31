#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros

if [[ ! -x "$CFG_GO2_PYTHON" ]]; then
  echo "Working Unitree Python environment not found: $CFG_GO2_PYTHON" >&2
  exit 1
fi
if [[ ! -d "$CFG_CYCLONEDDS_HOME/lib" ]]; then
  echo "CycloneDDS installation not found: $CFG_CYCLONEDDS_HOME" >&2
  exit 1
fi
export CYCLONEDDS_HOME="$CFG_CYCLONEDDS_HOME"
export CMAKE_PREFIX_PATH="$CFG_CYCLONEDDS_HOME${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$CFG_CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "Starting observation-only Go2 battery monitor"
echo "  DDS input: rt/lowstate on $CFG_UNITREE_NET_IF"
echo "  ROS output: /navdp/go2/battery"
echo "  offline behavior: publish GO2 OFFLINE and wait for the configured link"

exec "$CFG_GO2_PYTHON" "$NAVDP_GO2_DIR/go2_battery_monitor.py" \
  --net-if "$CFG_UNITREE_NET_IF" \
  --sdk-path "$CFG_UNITREE_SDK_PATH" \
  --dds-topic rt/lowstate \
  --ros-args \
  -p battery_topic:=/navdp/go2/battery \
  -p publish_rate_hz:=2.0 \
  -p offline_timeout_s:=2.0
