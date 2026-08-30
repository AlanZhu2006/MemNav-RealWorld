#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"

[[ "$CFG_PROFILE" == native-navdp-rgbd ]] || {
  echo "run_stack.sh requires profile=native-navdp-rgbd" >&2
  exit 2
}
[[ "$CFG_NAV_BACKEND" == navdp && "$CFG_NAV_MODE" == imagegoal ]] || {
  echo "native stack requires backend=navdp mode=imagegoal" >&2
  exit 2
}
[[ -f "$CFG_IMAGE_GOAL" ]] || {
  echo "ImageGoal missing: $CFG_IMAGE_GOAL" >&2
  exit 1
}
command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }
command -v timeout >/dev/null || { echo "timeout is required" >&2; exit 1; }
SESSION="$CFG_NATIVE_SESSION"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  exit 1
fi
bash "$SCRIPT_DIR/preflight.sh" --config "$NAVDP_RUN_CONFIG"
LOG_ROOT="$CFG_JETSON_RUNTIME_ROOT/logs"
mkdir -p "$LOG_ROOT"
policy_log="$LOG_ROOT/native_navdp.log"
camera_log="$LOG_ROOT/realsense.log"
adapter_log="$LOG_ROOT/adapter.log"
: >"$policy_log"

tmux new-session -d -s "$SESSION" -n policy \
  "exec '$SCRIPT_DIR/run_base_navdp_server.sh' --config '$NAVDP_RUN_CONFIG' >'$policy_log' 2>&1"
if [[ "$CFG_WITH_CAMERA" == true ]]; then
  : >"$camera_log"
  tmux new-window -t "$SESSION" -n rgbd \
    "exec '$SCRIPT_DIR/run_realsense.sh' --config '$NAVDP_RUN_CONFIG' >'$camera_log' 2>&1"
fi

start_complete=false
rollback_partial_start() {
  local status=$?
  if [[ "$start_complete" != true ]]; then
    tmux kill-session -t "$SESSION" 2>/dev/null || true
  fi
  return "$status"
}
trap rollback_partial_start EXIT

policy_ready=false
for _ in $(seq 1 "$CFG_NATIVE_READY_TIMEOUT_S"); do
  if curl -fsS --max-time 1 \
      "http://$CFG_NATIVE_HOST:$CFG_NATIVE_PORT/healthz" >/dev/null 2>&1; then
    policy_ready=true
    break
  fi
  sleep 1
done
if [[ "$policy_ready" != true ]]; then
  echo "Native NavDP policy did not become healthy." >&2
  tail -n 100 "$policy_log" >&2 || true
  exit 1
fi

if [[ "$CFG_WITH_CAMERA" == true ]]; then
  navdp_source_ros
  camera_ready=false
  camera_deadline=$((SECONDS + CFG_CAMERA_READY_TIMEOUT_S))
  while (( SECONDS < camera_deadline )); do
    if timeout 3 ros2 topic echo --once "$CFG_CAMERA_INFO_TOPIC" >/dev/null 2>&1; then
      camera_ready=true
      break
    fi
    sleep 0.25
  done
  if [[ "$camera_ready" != true ]]; then
    echo "D435i did not publish CameraInfo." >&2
    tail -n 100 "$camera_log" >&2 || true
    exit 1
  fi
fi

: >"$adapter_log"
tmux new-window -t "$SESSION" -n adapter \
  "exec '$SCRIPT_DIR/run_adapter.sh' --config '$NAVDP_RUN_CONFIG' >'$adapter_log' 2>&1"
navdp_source_ros
adapter_ready=false
for _ in $(seq 1 "$CFG_ADAPTER_READY_TIMEOUT_S"); do
  if timeout 3 ros2 topic echo --once /navdp/status >/dev/null 2>&1; then
    adapter_ready=true
    break
  fi
  sleep 0.25
done
if [[ "$adapter_ready" != true ]]; then
  echo "NavDP adapter did not publish status." >&2
  tail -n 100 "$adapter_log" >&2 || true
  exit 1
fi
if [[ "$CFG_ARRIVAL_MODULE" == rgb-homography ]]; then
  tmux new-window -t "$SESSION" -n arrival \
    "exec '$SCRIPT_DIR/run_arrival_module.sh' --config '$NAVDP_RUN_CONFIG'"
fi
if [[ "$CFG_WITH_GO2" == true ]]; then
  tmux new-window -t "$SESSION" -n go2 \
    "exec '$SCRIPT_DIR/run_go2_bridge.sh' --config '$NAVDP_RUN_CONFIG'"
fi
if [[ "$CFG_WITH_RVIZ" == true ]]; then
  tmux new-window -t "$SESSION" -n rviz \
    "exec '$SCRIPT_DIR/run_debug_ui.sh' --config '$NAVDP_RUN_CONFIG'"
fi
tmux set-environment -t "$SESSION" MEMNAV_RUN_CONFIG "$NAVDP_RUN_CONFIG"
tmux set-environment -t "$SESSION" MEMNAV_CONFIG_ID "$CFG_CONFIG_ID"
start_complete=true
trap - EXIT

echo "Native NavDP ImageGoal stack started: session=$SESSION"
echo "  config=$NAVDP_RUN_CONFIG"
echo "  config_id=$CFG_CONFIG_ID"
echo "  ImageGoal=$CFG_IMAGE_GOAL"
echo "  ImageGoal_sha256=$CFG_IMAGE_GOAL_SHA256"
echo "  camera=$CFG_WITH_CAMERA go2_bridge=$CFG_WITH_GO2 rviz=$CFG_WITH_RVIZ"
echo "  arrival=$CFG_ARRIVAL_MODULE"
echo "Motion remains disabled until the explicit ROS SetBool call."
