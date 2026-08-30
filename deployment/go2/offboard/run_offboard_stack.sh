#!/usr/bin/env bash
set -euo pipefail

OFFBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$OFFBOARD_DIR/.." && pwd)"
source "$GO2_DIR/scripts/common.sh"
source "$OFFBOARD_DIR/runtime_contract.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"

[[ "$CFG_PROFILE" == fullmono-lingbot-cec ]] || {
  echo "run_offboard_stack requires the Full-Mono profile" >&2
  exit 2
}
SESSION="$CFG_FULLMONO_SESSION"
LOCAL_PORT="$CFG_TUNNEL_LOCAL_PORT"
LOG_ROOT="$CFG_JETSON_RUNTIME_ROOT/logs"
[[ -f "$CFG_IMAGE_GOAL" ]] || { echo "ImageGoal missing: $CFG_IMAGE_GOAL" >&2; exit 1; }
command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }
command -v timeout >/dev/null || { echo "timeout is required" >&2; exit 1; }
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  exit 1
fi

tmux new-session -d -s "$SESSION" -n tunnel \
  "exec '$OFFBOARD_DIR/run_policy_tunnel.sh' --config '$NAVDP_RUN_CONFIG'"
healthy=false
for _ in $(seq 1 $((CFG_TUNNEL_READY_TIMEOUT_S * 4))); do
  health="$(curl -fsS --max-time 1 \
      "http://127.0.0.1:${LOCAL_PORT}/healthz" 2>/dev/null || true)"
  if cec_validate_health_contract "$health" "$GO2_DIR" 2>/dev/null; then
    healthy=true
    break
  fi
  sleep 0.25
done
if [[ "$healthy" != true ]]; then
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  echo "CEC hub did not become healthy through the SSH tunnel" >&2
  exit 1
fi

if [[ "$CFG_WITH_CAMERA" == true ]]; then
  mkdir -p "$LOG_ROOT"
  camera_log="$LOG_ROOT/realsense.log"
  : >"$camera_log"
  tmux new-window -t "$SESSION" -n rgbd \
    "exec '$GO2_DIR/scripts/run_realsense.sh' --config '$NAVDP_RUN_CONFIG' >'$camera_log' 2>&1"
  navdp_source_ros
  camera_ready=false
  camera_deadline=$((SECONDS + CFG_CAMERA_READY_TIMEOUT_S))
  while (( SECONDS < camera_deadline )); do
    if ! tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -Fxq rgbd; then
      break
    fi
    if timeout 3 ros2 topic echo --once "$CFG_CAMERA_INFO_TOPIC" >/dev/null 2>&1; then
      camera_ready=true
      break
    fi
    sleep 0.25
  done
  if [[ "$camera_ready" != true ]]; then
    echo "D435i did not publish CameraInfo; refusing to start the adapter." >&2
    [[ ! -s "$camera_log" ]] || tail -n 80 "$camera_log" >&2 || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    exit 1
  fi
fi

tmux new-window -t "$SESSION" -n adapter \
  "exec '$GO2_DIR/scripts/run_adapter.sh' --config '$NAVDP_RUN_CONFIG' >'$LOG_ROOT/adapter.log' 2>&1"
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
  echo "NavDP adapter did not publish status; rolling back local stack." >&2
  tail -n 100 "$LOG_ROOT/adapter.log" >&2 || true
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  exit 1
fi
if [[ "$CFG_ARRIVAL_MODULE" == rgb-homography ]]; then
  tmux new-window -t "$SESSION" -n arrival \
    "exec '$GO2_DIR/scripts/run_arrival_module.sh' --config '$NAVDP_RUN_CONFIG'"
fi
if [[ "$CFG_WITH_GO2" == true ]]; then
  tmux new-window -t "$SESSION" -n go2 \
    "exec '$GO2_DIR/scripts/run_go2_bridge.sh' --config '$NAVDP_RUN_CONFIG'"
fi
if [[ "$CFG_WITH_RVIZ" == true ]]; then
  tmux new-window -t "$SESSION" -n rviz \
    "exec '$GO2_DIR/scripts/run_debug_ui.sh' --config '$NAVDP_RUN_CONFIG'"
fi
tmux set-environment -t "$SESSION" MEMNAV_RUN_CONFIG "$NAVDP_RUN_CONFIG"
tmux set-environment -t "$SESSION" MEMNAV_CONFIG_ID "$CFG_CONFIG_ID"

echo "Offboard CEC/NavDP stack started: session=$SESSION"
echo "  config=$NAVDP_RUN_CONFIG"
echo "  config_id=$CFG_CONFIG_ID"
echo "  hub=http://127.0.0.1:${LOCAL_PORT} camera=$CFG_WITH_CAMERA go2_bridge=$CFG_WITH_GO2"
echo "  ImageGoal=$CFG_IMAGE_GOAL"
echo "  navigation=causal_monocular_rgb local_aligned_depth=safety_only"
echo "  control_profile=$CFG_CONTROL_PROFILE max_linear_mps=$CFG_MAX_LINEAR_MPS max_angular_rps=$CFG_MAX_ANGULAR_RPS"
echo "  arrival=$CFG_ARRIVAL_MODULE arrival_goal=$CFG_ARRIVAL_GOAL"
echo "Motion remains locked until an operator explicitly calls set_enabled=true."
