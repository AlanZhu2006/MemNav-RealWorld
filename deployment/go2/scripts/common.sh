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
  local camera_ready=false
  local camera_deadline=$((SECONDS + CFG_CAMERA_READY_TIMEOUT_S))
  while (( SECONDS < camera_deadline )); do
    if [[ "$require_live_window" == true ]]; then
      if ! tmux list-windows -t "$session" -F '#{window_name}' \
          | grep -Fxq rgbd; then
        break
      fi
    fi
    if timeout 3 ros2 topic echo --once "$CFG_CAMERA_INFO_TOPIC" \
        >/dev/null 2>&1; then
      camera_ready=true
      break
    fi
    sleep 0.25
  done
  [[ "$camera_ready" == true ]]
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
  if [[ "$CFG_WITH_FOXGLOVE" == true ]]; then
    tmux new-window -t "$session" -n fox-preview \
      "exec '$NAVDP_GO2_SCRIPT_DIR/run_foxglove_preview.sh' --config '$NAVDP_RUN_CONFIG'"
    tmux new-window -t "$session" -n foxglove \
      "exec '$NAVDP_GO2_SCRIPT_DIR/run_foxglove_bridge.sh' --config '$NAVDP_RUN_CONFIG'"
  fi
}

navdp_stamp_session_contract() {
  local session="$1"
  tmux set-environment -t "$session" MEMNAV_RUN_CONFIG "$NAVDP_RUN_CONFIG"
  tmux set-environment -t "$session" MEMNAV_CONFIG_ID "$CFG_CONFIG_ID"
}
