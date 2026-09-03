#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

navdp_source_ros
if [[ "${1:-}" == --operator-stop ]]; then
  stop_jobs=()
  timeout 6 ros2 topic pub --once /navdp/enabled \
    std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 &
  stop_jobs+=("$!")
  timeout 6 ros2 topic pub --once /navdp/estop \
    std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 &
  stop_jobs+=("$!")
  timeout 6 ros2 service call /memnav_operator/operator_stop \
    std_srvs/srv/Trigger '{}' >/dev/null 2>&1 &
  stop_jobs+=("$!")
  timeout 6 ros2 service call /navdp_go2_adapter/operator_stop \
    std_srvs/srv/Trigger '{}' >/dev/null 2>&1 &
  stop_jobs+=("$!")
  for job in "${stop_jobs[@]}"; do
    wait "$job" || true
  done
  exit 0
fi

exec /usr/bin/python3 "$NAVDP_GO2_DIR/revisit_operator_service.py" \
  --repo-root "$NAVDP_ROOT" \
  --state "$NAVDP_ROOT/runtime/go2/revisit_debug/active.json" \
  --robot-ip 192.168.123.161 \
  --timeout-s 300
