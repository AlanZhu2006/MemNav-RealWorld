#!/usr/bin/env bash

NAVDP_GO2_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVDP_GO2_DIR="$(cd "$NAVDP_GO2_SCRIPT_DIR/.." && pwd)"
NAVDP_ROOT="$(cd "$NAVDP_GO2_DIR/../.." && pwd)"
NAVDP_RUNTIME_CONFIG_TOOL="$NAVDP_ROOT/deployment/runtime_config.py"
NAVDP_SYSTEM_CONFIG="$NAVDP_ROOT/deployment/config/system.json"
# Setup/capture utilities need the tracked machine bootstrap paths before an
# experiment is resolved. Runtime launchers replace these with the same values
# from their hash-verified resolved config.
NAVDP_BOOTSTRAP_CONFIG="$({ python3 "$NAVDP_RUNTIME_CONFIG_TOOL" system-shell \
  --config "$NAVDP_SYSTEM_CONFIG" --site jetson; })"
eval "$NAVDP_BOOTSTRAP_CONFIG"
NAVDP_VENV="$(dirname "$(dirname "$CFG_JETSON_PYTHON")")"
NAVDP_ROS_SETUP="$CFG_ROS_SETUP"
NAVDP_REALSENSE_SETUP="$CFG_REALSENSE_SETUP"
NAVDP_MESSAGE_FILTERS_SETUP="$CFG_MESSAGE_FILTERS_SETUP"
NAVDP_CUSPARSELT_DIR="$NAVDP_VENV/opt/cusparselt"

navdp_require_config_arg() {
  if [[ $# -ne 2 || "$1" != --config || -z "$2" ]]; then
    echo "Usage: ${0##*/} --config RESOLVED_CONFIG.json" >&2
    return 2
  fi
  NAVDP_RUN_CONFIG="$(readlink -f "$2")"
  [[ -f "$NAVDP_RUN_CONFIG" ]] || {
    echo "Resolved run config is missing: $NAVDP_RUN_CONFIG" >&2
    return 1
  }
}

navdp_load_config() {
  local config="$1"
  python3 "$NAVDP_RUNTIME_CONFIG_TOOL" verify \
    --config "$config" --site jetson >/dev/null
  navdp_read_config "$config"
}

navdp_read_config() {
  local config="$1"
  # This is generated from one hash-verified JSON file. CFG_* values are
  # internal shell locals, not an operator-facing environment configuration.
  local config_exports
  config_exports="$(python3 "$NAVDP_RUNTIME_CONFIG_TOOL" shell \
    --config "$config" --site jetson)"
  eval "$config_exports"
  NAVDP_VENV="$(dirname "$(dirname "$CFG_JETSON_PYTHON")")"
  NAVDP_ROS_SETUP="$CFG_ROS_SETUP"
  NAVDP_REALSENSE_SETUP="$CFG_REALSENSE_SETUP"
  NAVDP_MESSAGE_FILTERS_SETUP="$CFG_MESSAGE_FILTERS_SETUP"
  NAVDP_CUSPARSELT_DIR="$NAVDP_VENV/opt/cusparselt"
}

navdp_source_file() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    local had_nounset=0
    case $- in
      *u*) had_nounset=1; set +u ;;
    esac
    source "$setup_file"
    if [[ "$had_nounset" == "1" ]]; then
      set -u
    fi
  fi
}

navdp_source_ros() {
  if [[ ! -f "$NAVDP_ROS_SETUP" ]]; then
    echo "ROS setup not found: $NAVDP_ROS_SETUP" >&2
    return 1
  fi
  navdp_source_file "$NAVDP_ROS_SETUP"
  navdp_source_file "$NAVDP_MESSAGE_FILTERS_SETUP"
  navdp_source_file "$NAVDP_REALSENSE_SETUP"
}

navdp_activate_venv() {
  if [[ ! -x "$NAVDP_VENV/bin/python" ]]; then
    echo "NavDP virtual environment is missing: $NAVDP_VENV" >&2
    echo "Run: $NAVDP_GO2_SCRIPT_DIR/setup_jetson.sh" >&2
    return 1
  fi
  navdp_source_file "$NAVDP_VENV/bin/activate"
  if [[ -d "$NAVDP_CUSPARSELT_DIR/lib" ]]; then
    export LD_LIBRARY_PATH="$NAVDP_CUSPARSELT_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  export PYTHONPATH="$NAVDP_GO2_DIR${PYTHONPATH:+:$PYTHONPATH}"
  export MPLCONFIGDIR="${MPLCONFIGDIR:-$NAVDP_ROOT/.cache/matplotlib}"
  export CUDA_MODULE_LOADING="${CUDA_MODULE_LOADING:-LAZY}"
  export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
  export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
  mkdir -p "$MPLCONFIGDIR"
}

navdp_wait_for_camera_info() {
  local session="$1"
  local require_live_window="$2"
  # Keep one DDS subscriber alive while the USB reset and sensor bring-up run.
  # Repeated short-lived `ros2 topic echo` processes each pay discovery startup
  # and previously added roughly ten seconds after CameraInfo was already live.
  timeout "$CFG_CAMERA_READY_TIMEOUT_S" python3 - \
    "$CFG_CAMERA_INFO_TOPIC" >/dev/null 2>&1 <<'PY' &
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

rclpy.init(args=[])
node = Node("navdp_camera_info_waiter")
received = False

def on_message(_message):
    global received
    received = True

subscription = node.create_subscription(
    CameraInfo, sys.argv[1], on_message, qos_profile_sensor_data
)
while rclpy.ok() and not received:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_subscription(subscription)
node.destroy_node()
rclpy.shutdown()
PY
  local waiter_pid=$!
  while kill -0 "$waiter_pid" 2>/dev/null; do
    if [[ "$require_live_window" == true ]]; then
      if ! tmux list-windows -t "$session" -F '#{window_name}' \
          | grep -Fxq rgbd; then
        kill -TERM "$waiter_pid" 2>/dev/null || true
        wait "$waiter_pid" 2>/dev/null || true
        return 1
      fi
    fi
    sleep 0.25
  done
  wait "$waiter_pid"
}

navdp_start_adapter_and_wait() {
  local session="$1"
  local adapter_log="$2"
  : >"$adapter_log"
  tmux new-window -t "$session" -n adapter \
    "exec '$NAVDP_GO2_SCRIPT_DIR/run_adapter.sh' --config '$NAVDP_RUN_CONFIG' >'$adapter_log' 2>&1"
  local adapter_ready=false
  for _ in $(seq 1 "$CFG_ADAPTER_READY_TIMEOUT_S"); do
    if timeout 3 ros2 topic echo --once /navdp/status >/dev/null 2>&1; then
      adapter_ready=true
      break
    fi
    sleep 0.25
  done
  [[ "$adapter_ready" == true ]]
}

navdp_start_camera_recovery_and_wait() {
  local session="$1"
  local recovery_log="$2"
  : >"$recovery_log"
  tmux new-window -t "$session" -n camera-recovery \
    "exec '$NAVDP_GO2_SCRIPT_DIR/run_camera_recovery.sh' --config '$NAVDP_RUN_CONFIG' >'$recovery_log' 2>&1"
  local recovery_ready=false
  for _ in $(seq 1 "$CFG_ADAPTER_READY_TIMEOUT_S"); do
    if [[ "$(timeout 2 ros2 service type \
        /navdp_camera_recovery/restart 2>/dev/null || true)" \
        == "std_srvs/srv/Trigger" ]]; then
      recovery_ready=true
      break
    fi
    sleep 0.25
  done
  [[ "$recovery_ready" == true ]]
}

navdp_start_foxglove_windows() {
  local session="$1"
  if [[ "$CFG_WITH_FOXGLOVE" == true ]]; then
    tmux new-window -t "$session" -n battery \
      "exec '$NAVDP_GO2_SCRIPT_DIR/run_go2_battery_monitor.sh' --config '$NAVDP_RUN_CONFIG'"
    tmux new-window -t "$session" -n fox-preview \
      "exec '$NAVDP_GO2_SCRIPT_DIR/run_foxglove_preview.sh' --config '$NAVDP_RUN_CONFIG'"
    tmux new-window -t "$session" -n foxglove \
      "exec '$NAVDP_GO2_SCRIPT_DIR/run_foxglove_bridge.sh' --config '$NAVDP_RUN_CONFIG'"
  fi
}

navdp_start_optional_windows() {
  local session="$1"
  if [[ "$CFG_ARRIVAL_MODULE" == rgb-homography ]]; then
    tmux new-window -t "$session" -n arrival \
      "exec '$NAVDP_GO2_SCRIPT_DIR/run_arrival_module.sh' --config '$NAVDP_RUN_CONFIG'"
  fi
  if [[ "$CFG_WITH_GO2" == true ]]; then
    tmux new-window -t "$session" -n go2 \
      "exec '$NAVDP_GO2_SCRIPT_DIR/run_go2_bridge.sh' --config '$NAVDP_RUN_CONFIG'"
  fi
}

navdp_stamp_session_contract() {
  local session="$1"
  tmux set-environment -t "$session" MEMNAV_RUN_CONFIG "$NAVDP_RUN_CONFIG"
  tmux set-environment -t "$session" MEMNAV_CONFIG_ID "$CFG_CONFIG_ID"
}

navdp_assert_motion_locked() {
  # Reusing a process generation is only safe after the adapter confirms that
  # motion authority has been revoked and publishes the resulting locked state.
  navdp_source_ros >/dev/null 2>&1 || return 1
  local response payload
  response="$(timeout 8 ros2 service call \
    /navdp_go2_adapter/operator_stop std_srvs/srv/Trigger '{}' 2>&1)" \
    || return 1
  grep -Eiq 'success[=:][[:space:]]*(true|True)' <<<"$response" || return 1

  for _ in $(seq 1 3); do
    payload="$(timeout 3 ros2 topic echo --once /navdp/status \
      --field data 2>/dev/null || true)"
    if python3 - "$payload" <<'PY'
import ast
import json
import sys

raw = "\n".join(
    line for line in sys.argv[1].splitlines() if line.strip() != "---"
).strip()
try:
    decoded = ast.literal_eval(raw)
    if isinstance(decoded, str):
        raw = decoded
except (SyntaxError, ValueError):
    pass
status = json.loads(raw)
assert status.get("enabled") is False
assert status.get("estop") is True
PY
    then
      return 0
    fi
  done
  return 1
}

navdp_lock_motion_before_shutdown() {
  # Shutdown remains the fallback lock: failure to reach ROS cannot prevent
  # process teardown, whose watchdog and process exit also remove commands.
  navdp_assert_motion_locked >/dev/null 2>&1 || true
}

navdp_pause_boot_observer() {
  command -v systemctl >/dev/null 2>&1 || return 0
  systemctl --user cat memnav-observer.target >/dev/null 2>&1 || return 0
  if systemctl --user is-active --quiet memnav-observer.target; then
    systemctl --user stop memnav-observer.target
    echo "Paused the boot observer for exclusive navigation-stack ownership."
  fi
}

navdp_resume_boot_observer() {
  [[ "${NAVDP_OBSERVER_RESUME:-true}" == true ]] || return 0
  command -v systemctl >/dev/null 2>&1 || return 0
  systemctl --user is-enabled --quiet memnav-observer.target || return 0
  if command -v tmux >/dev/null 2>&1; then
    tmux has-session -t "${CFG_NATIVE_SESSION:-navdp-go2}" 2>/dev/null && return 0
    tmux has-session -t "${CFG_FULLMONO_SESSION:-navdp-go2-offboard}" 2>/dev/null \
      && return 0
  fi
  systemctl --user start memnav-observer.target
  echo "Restored the always-on camera and Foxglove observer."
}
