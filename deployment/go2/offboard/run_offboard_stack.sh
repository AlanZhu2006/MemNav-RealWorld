#!/usr/bin/env bash
set -euo pipefail

OFFBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$OFFBOARD_DIR/.." && pwd)"
source "$GO2_DIR/scripts/common.sh"
source "$OFFBOARD_DIR/runtime_contract.sh"
SESSION="${NAVDP_TMUX_SESSION:-navdp-go2-offboard}"
LOCAL_PORT="${CEC_LOCAL_PORT:-18889}"
CAMERA_READY_TIMEOUT_S="${NAVDP_CAMERA_READY_TIMEOUT_S:-60}"
LOG_ROOT="${NAVDP_GO2_LOG_ROOT:-$NAVDP_ROOT/runtime/go2/logs}"
with_go2=false
with_camera=true
with_rviz=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-go2) with_go2=true; shift ;;
    --with-rviz) with_rviz=true; shift ;;
    --no-camera) with_camera=false; shift ;;
    *) echo "Usage: $0 [--with-go2] [--with-rviz] [--no-camera]" >&2; exit 2 ;;
  esac
done

goal_path="${NAVDP_IMAGE_GOAL_PATH:-$GO2_DIR/goals/image_goal.png}"
revisit_goal_path="${NAVDP_REVISIT_IMAGE_GOAL_PATH:-}"
novel_recording_navigation="${NAVDP_NAVIGATE_DURING_MEMORY_RECORDING:-false}"
pause_memory_recording="${NAVDP_PAUSE_MEMORY_RECORDING:-false}"
auto_goal_interval="${NAVDP_AUTO_GOAL_CANDIDATE_INTERVAL_FRAMES:-24}"
auto_goal_max="${NAVDP_AUTO_GOAL_CANDIDATE_MAX:-6}"
auto_goal_guard="${NAVDP_AUTO_GOAL_CANDIDATE_POST_GUARD_FRAMES:-4}"
auto_goal_capture_enabled="${NAVDP_AUTO_GOAL_CANDIDATE_CAPTURE_ENABLED:-true}"
auto_select_goal="${NAVDP_AUTO_SELECT_GOAL_CANDIDATE:-true}"
selected_goal_image_path="${NAVDP_SELECTED_GOAL_IMAGE_PATH:-}"
selected_goal_depth_path="${NAVDP_SELECTED_GOAL_DEPTH_PATH:-}"
max_linear_mps="${NAVDP_MAX_LINEAR_MPS:-}"
max_angular_rps="${NAVDP_MAX_ANGULAR_RPS:-}"
control_profile="${NAVDP_CONTROL_PROFILE:-formal}"
rgb_arrival_enabled="${NAVDP_RGB_ARRIVAL_ENABLED:-false}"
arrival_module="${NAVDP_ARRIVAL_MODULE:-}"
if [[ -z "$arrival_module" ]]; then
  if [[ "$rgb_arrival_enabled" == true ]]; then
    arrival_module="rgb-homography"
  else
    arrival_module="operator"
  fi
fi
arrival_goal_path="${NAVDP_ARRIVAL_GOAL_PATH:-$goal_path}"
arrival_allowed_phases="${NAVDP_ARRIVAL_ALLOWED_PHASES:-memory_recording}"
[[ -f "$goal_path" ]] || { echo "Image goal missing: $goal_path" >&2; exit 1; }
if [[ -n "$revisit_goal_path" && ! -f "$revisit_goal_path" ]]; then
  echo "Revisit ImageGoal missing: $revisit_goal_path" >&2
  exit 1
fi
command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }
command -v timeout >/dev/null || { echo "timeout is required" >&2; exit 1; }
if [[ "$control_profile" != formal && "$control_profile" != acceptance ]]; then
  echo "NAVDP_CONTROL_PROFILE must be formal or acceptance" >&2
  exit 1
fi
if [[ "$rgb_arrival_enabled" != true && "$rgb_arrival_enabled" != false ]]; then
  echo "NAVDP_RGB_ARRIVAL_ENABLED must be true or false" >&2
  exit 1
fi
read -r _profile_name arrival_module < <(
  python3 "$GO2_DIR/stack_profiles.py" validate \
    fullmono-lingbot-cec "$arrival_module"
)
if [[ "$arrival_module" == rgb-homography && ! -f "$arrival_goal_path" ]]; then
  echo "Arrival reference missing: $arrival_goal_path" >&2
  exit 1
fi
if [[ "$with_go2" == true && "$control_profile" == formal ]]; then
  case "$max_linear_mps" in
    ""|0.3|0.30|0.300) ;;
    *)
      echo "Formal Go2 profile requires max_linear_mps=0.30; got $max_linear_mps" >&2
      echo "Use NAVDP_CONTROL_PROFILE=acceptance only for a bounded commissioning smoke." >&2
      exit 1
      ;;
  esac
  case "$max_angular_rps" in
    ""|0.55|0.550) ;;
    *)
      echo "Formal Go2 profile requires max_angular_rps=0.55; got $max_angular_rps" >&2
      echo "Use NAVDP_CONTROL_PROFILE=acceptance only for a bounded commissioning smoke." >&2
      exit 1
      ;;
  esac
fi
[[ "$CAMERA_READY_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] \
  || { echo "NAVDP_CAMERA_READY_TIMEOUT_S must be a positive integer" >&2; exit 1; }
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  exit 1
fi

tmux new-session -d -s "$SESSION" -n tunnel "exec '$OFFBOARD_DIR/run_policy_tunnel.sh'"
healthy=false
for _ in $(seq 1 20); do
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

if [[ "$with_camera" == true ]]; then
  mkdir -p "$LOG_ROOT"
  camera_log="$LOG_ROOT/realsense.log"
  : >"$camera_log"
  tmux new-window -t "$SESSION" -n rgbd \
    "exec '$GO2_DIR/scripts/run_realsense.sh' >'$camera_log' 2>&1"

  # A tmux window being created is not evidence that the camera survived its
  # firmware/device checks.  Require one real CameraInfo message before the
  # adapter is allowed to start, otherwise fail transactionally.
  navdp_source_ros
  camera_ready=false
  camera_deadline=$((SECONDS + CAMERA_READY_TIMEOUT_S))
  while (( SECONDS < camera_deadline )); do
    if ! tmux list-windows -t "$SESSION" -F '#{window_name}' \
        | grep -Fxq rgbd; then
      break
    fi
    # A fresh ROS CLI process can spend more than one second in DDS
    # discovery on the Jetson even after the RealSense node is publishing.
    # Keep the outer hard deadline, but give each probe enough time to
    # discover the publisher; otherwise a healthy camera is rejected forever
    # by a sequence of prematurely killed one-second probes.
    if timeout 3 ros2 topic echo --once \
        /camera/camera/color/camera_info >/dev/null 2>&1; then
      camera_ready=true
      break
    fi
    sleep 0.25
  done
  if [[ "$camera_ready" != true ]]; then
    echo "D435i did not publish CameraInfo; refusing to start the adapter." >&2
    if [[ -s "$camera_log" ]]; then
      echo "===== $camera_log" >&2
      tail -n 80 "$camera_log" >&2 || true
    fi
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    exit 1
  fi
fi
tmux new-window -t "$SESSION" -n adapter \
  "export NAVDP_BACKEND='navdp' NAVDP_MODE='imagegoal' NAVDP_TWO_PHASE='true' NAVDP_NAVIGATE_DURING_MEMORY_RECORDING='$novel_recording_navigation' NAVDP_PAUSE_MEMORY_RECORDING='$pause_memory_recording' NAVDP_AUTO_GOAL_CANDIDATE_INTERVAL_FRAMES='$auto_goal_interval' NAVDP_AUTO_GOAL_CANDIDATE_MAX='$auto_goal_max' NAVDP_AUTO_GOAL_CANDIDATE_POST_GUARD_FRAMES='$auto_goal_guard' NAVDP_AUTO_GOAL_CANDIDATE_CAPTURE_ENABLED='$auto_goal_capture_enabled' NAVDP_AUTO_SELECT_GOAL_CANDIDATE='$auto_select_goal' NAVDP_MAX_LINEAR_MPS='$max_linear_mps' NAVDP_MAX_ANGULAR_RPS='$max_angular_rps' NAVDP_SERVER_URL='http://127.0.0.1:${LOCAL_PORT}' NAVDP_IMAGE_GOAL_PATH='$goal_path' NAVDP_REVISIT_IMAGE_GOAL_PATH='$revisit_goal_path' NAVDP_SELECTED_GOAL_IMAGE_PATH='$selected_goal_image_path' NAVDP_SELECTED_GOAL_DEPTH_PATH='$selected_goal_depth_path'; exec '$GO2_DIR/scripts/run_adapter.sh'"
if [[ "$arrival_module" == rgb-homography ]]; then
  tmux new-window -t "$SESSION" -n arrival \
    "export NAVDP_ARRIVAL_MODULE='$arrival_module' NAVDP_ARRIVAL_GOAL_PATH='$arrival_goal_path' NAVDP_ARRIVAL_ALLOWED_PHASES='$arrival_allowed_phases'; exec '$GO2_DIR/scripts/run_arrival_module.sh'"
fi
if [[ "$with_go2" == true ]]; then
  tmux new-window -t "$SESSION" -n go2 "exec '$GO2_DIR/scripts/run_go2_bridge.sh'"
fi
if [[ "$with_rviz" == true ]]; then
  tmux new-window -t "$SESSION" -n rviz "exec '$GO2_DIR/scripts/run_debug_ui.sh'"
fi
tmux set-environment -t "$SESSION" NAVDP_STACK_PROFILE fullmono-lingbot-cec
tmux set-environment -t "$SESSION" NAVDP_ARRIVAL_MODULE "$arrival_module"
tmux set-environment -t "$SESSION" NAVDP_NAVIGATION_GOAL_PATH "$goal_path"
tmux set-environment -t "$SESSION" NAVDP_ARRIVAL_GOAL_PATH "$arrival_goal_path"
tmux set-environment -t "$SESSION" NAVDP_ARRIVAL_ALLOWED_PHASES "$arrival_allowed_phases"

echo "Offboard CEC/NavDP stack started in tmux session $SESSION"
echo "  hub=http://127.0.0.1:${LOCAL_PORT} camera=$with_camera go2_bridge=$with_go2"
echo "  navigation=causal_monocular_rgb local_aligned_depth=safety_only"
echo "  control_profile=$control_profile max_linear_mps=${max_linear_mps:-0.30(config)} max_angular_rps=${max_angular_rps:-0.55(config)}"
echo "  arrival=$arrival_module arrival_goal=${arrival_goal_path:-n/a}"
echo "  inspect: tmux attach -t $SESSION"
echo "Motion remains locked until an operator explicitly calls set_enabled=true."
