#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

usage() {
  cat <<'EOF'
Usage:
  camera_ui.sh start  [--config EXPERIMENT.json]
  camera_ui.sh status [--config EXPERIMENT.json]
  camera_ui.sh stop   [--config EXPERIMENT.json]

Starts only RealSense, the bandwidth-limited preview relay and Foxglove Bridge.
It does not load a policy, start the adapter or access the Unitree control link.
EOF
}

die() { echo "camera-ui: $*" >&2; exit 1; }

action="${1:-}"
[[ $# -eq 0 ]] || shift
source_config="$NAVDP_ROOT/deployment/config/experiments/native_imagegoal.json"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      [[ $# -ge 2 ]] || die "--config requires a value"
      source_config="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

resolved="$(python3 "$NAVDP_RUNTIME_CONFIG_TOOL" resolve --config "$source_config")"
navdp_load_config "$resolved"
NAVDP_RUN_CONFIG="$resolved"
SESSION="${CFG_NATIVE_SESSION}-camera-ui"

session_is_current() {
  tmux has-session -t "$SESSION" 2>/dev/null || return 1
  local active_id windows
  active_id="$(tmux show-environment -t "$SESSION" MEMNAV_CONFIG_ID 2>/dev/null \
    | sed -n 's/^MEMNAV_CONFIG_ID=//p' || true)"
  [[ "$active_id" == "$CFG_CONFIG_ID" ]] || return 1
  windows="$(tmux list-windows -t "$SESSION" -F '#{window_name} #{pane_dead}')" \
    || return 1
  grep -Fxq 'rgbd 0' <<<"$windows" \
    && grep -Fxq 'fox-preview 0' <<<"$windows" \
    && grep -Fxq 'foxglove 0' <<<"$windows"
}

start_ui() {
  command -v tmux >/dev/null || die "tmux is required"
  command -v timeout >/dev/null || die "timeout is required"
  [[ "$CFG_WITH_CAMERA" == true ]] || die "selected config disables the camera"
  [[ "$CFG_WITH_FOXGLOVE" == true ]] || die "selected config disables Foxglove"
  if session_is_current; then
    echo "FAST START: reusing camera-only Foxglove session $SESSION"
    echo "  ws://$CFG_FOXGLOVE_ADDRESS:$CFG_FOXGLOVE_PORT"
    return 0
  fi
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
  fi
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${CFG_FOXGLOVE_PORT}$"; then
    die "port $CFG_FOXGLOVE_PORT is already in use; a full Foxglove stack may already be running"
  fi

  navdp_source_ros
  ros2 pkg prefix foxglove_bridge >/dev/null 2>&1 \
    || die "foxglove_bridge is not installed"
  [[ -x "$NAVDP_VENV/bin/python" ]] || die "NavDP Python environment is missing"
  command -v rs-enumerate-devices >/dev/null \
    || die "RealSense tools are missing"

  local log_root="$CFG_JETSON_RUNTIME_ROOT/logs"
  mkdir -p "$log_root"
  : >"$log_root/realsense-camera-ui.log"
  : >"$log_root/foxglove-preview-camera-ui.log"
  : >"$log_root/foxglove-camera-ui.log"
  tmux new-session -d -s "$SESSION" -n rgbd \
    "exec '$SCRIPT_DIR/run_realsense.sh' --config '$resolved' >'$log_root/realsense-camera-ui.log' 2>&1"
  tmux new-window -t "$SESSION" -n fox-preview \
    "exec '$SCRIPT_DIR/run_foxglove_preview.sh' --config '$resolved' >'$log_root/foxglove-preview-camera-ui.log' 2>&1"
  tmux new-window -t "$SESSION" -n foxglove \
    "exec '$SCRIPT_DIR/run_foxglove_bridge.sh' --config '$resolved' >'$log_root/foxglove-camera-ui.log' 2>&1"
  navdp_stamp_session_contract "$SESSION"

  local complete=false
  rollback() {
    local status=$?
    [[ "$complete" == true ]] || tmux kill-session -t "$SESSION" 2>/dev/null || true
    return "$status"
  }
  trap rollback EXIT
  if ! navdp_wait_for_camera_info "$SESSION" true; then
    tail -n 80 "$log_root/realsense-camera-ui.log" >&2 || true
    die "RealSense did not publish CameraInfo"
  fi
  local bridge_ready=false
  for _ in $(seq 1 40); do
    if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${CFG_FOXGLOVE_PORT}$"; then
      bridge_ready=true
      break
    fi
    sleep 0.25
  done
  if [[ "$bridge_ready" != true ]]; then
    tail -n 80 "$log_root/foxglove-camera-ui.log" >&2 || true
    die "Foxglove Bridge did not listen on port $CFG_FOXGLOVE_PORT"
  fi
  complete=true
  trap - EXIT
  echo "Camera-only Foxglove is ready: session=$SESSION"
  echo "  websocket=ws://$CFG_FOXGLOVE_ADDRESS:$CFG_FOXGLOVE_PORT"
  echo "  available=Live RGB, Depth"
  echo "  unavailable=policy status, path, match and service controls"
  echo "  motion=not present"
}

status_ui() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "RUNNING session=$SESSION contract=$([[ $(tmux show-environment -t "$SESSION" MEMNAV_CONFIG_ID 2>/dev/null | sed -n 's/^MEMNAV_CONFIG_ID=//p') == "$CFG_CONFIG_ID" ]] && echo current || echo stale)"
    tmux list-windows -t "$SESSION" -F '  window=#{window_name} dead=#{pane_dead}'
  else
    echo "STOPPED session=$SESSION"
  fi
}

stop_ui() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
    echo "Stopped camera-only Foxglove session: $SESSION"
  else
    echo "No camera-only Foxglove session named $SESSION"
  fi
}

case "$action" in
  start) start_ui ;;
  status) status_ui ;;
  stop) stop_ui ;;
  -h|--help|help|"") usage ;;
  *) die "unknown action: $action" ;;
esac
