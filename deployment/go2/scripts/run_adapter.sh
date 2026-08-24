#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros
navdp_activate_venv

CONFIG="${NAVDP_GO2_CONFIG:-$NAVDP_GO2_DIR/config/navdp_go2.yaml}"
BACKEND="${NAVDP_BACKEND:-x_navdp}"
MODE="${NAVDP_MODE:-startgoal}"
SERVER_URL="${NAVDP_SERVER_URL:-http://127.0.0.1:8888}"
MAX_LINEAR_MPS="${NAVDP_MAX_LINEAR_MPS:-}"
MAX_ANGULAR_RPS="${NAVDP_MAX_ANGULAR_RPS:-}"
IMAGE_GOAL_PATH="${NAVDP_IMAGE_GOAL_PATH:-$NAVDP_GO2_DIR/goals/image_goal.png}"
REVISIT_IMAGE_GOAL_PATH="${NAVDP_REVISIT_IMAGE_GOAL_PATH:-}"
SELECTED_GOAL_IMAGE_PATH="${NAVDP_SELECTED_GOAL_IMAGE_PATH:-}"
SELECTED_GOAL_DEPTH_PATH="${NAVDP_SELECTED_GOAL_DEPTH_PATH:-}"
TWO_PHASE="${NAVDP_TWO_PHASE:-}"
NOVEL_RECORDING_NAVIGATION="${NAVDP_NAVIGATE_DURING_MEMORY_RECORDING:-}"
PAUSE_MEMORY_RECORDING="${NAVDP_PAUSE_MEMORY_RECORDING:-}"
AUTO_GOAL_INTERVAL="${NAVDP_AUTO_GOAL_CANDIDATE_INTERVAL_FRAMES:-}"
AUTO_GOAL_MAX="${NAVDP_AUTO_GOAL_CANDIDATE_MAX:-}"
AUTO_GOAL_GUARD="${NAVDP_AUTO_GOAL_CANDIDATE_POST_GUARD_FRAMES:-}"
AUTO_GOAL_CAPTURE_ENABLED="${NAVDP_AUTO_GOAL_CANDIDATE_CAPTURE_ENABLED:-}"
AUTO_SELECT_GOAL="${NAVDP_AUTO_SELECT_GOAL_CANDIDATE:-}"
EXTRA_PARAMS=()

if [[ -n "$MAX_LINEAR_MPS" ]]; then
  EXTRA_PARAMS+=(-p max_linear_mps:="$MAX_LINEAR_MPS")
fi
if [[ -n "$MAX_ANGULAR_RPS" ]]; then
  EXTRA_PARAMS+=(-p max_angular_rps:="$MAX_ANGULAR_RPS")
fi
if [[ "$MODE" == "imagegoal" ]]; then
  EXTRA_PARAMS+=(-p image_goal_path:="$IMAGE_GOAL_PATH")
  if [[ -n "$REVISIT_IMAGE_GOAL_PATH" ]]; then
    EXTRA_PARAMS+=(-p revisit_image_goal_path:="$REVISIT_IMAGE_GOAL_PATH")
  fi
  if [[ -n "$SELECTED_GOAL_IMAGE_PATH" ]]; then
    EXTRA_PARAMS+=(-p selected_goal_image_path:="$SELECTED_GOAL_IMAGE_PATH")
  fi
  if [[ -n "$SELECTED_GOAL_DEPTH_PATH" ]]; then
    EXTRA_PARAMS+=(-p selected_goal_depth_path:="$SELECTED_GOAL_DEPTH_PATH")
  fi
fi
if [[ -n "$TWO_PHASE" ]]; then
  EXTRA_PARAMS+=(-p two_phase_episode:="$TWO_PHASE")
fi
if [[ -n "$NOVEL_RECORDING_NAVIGATION" ]]; then
  EXTRA_PARAMS+=(
    -p navigate_during_memory_recording:="$NOVEL_RECORDING_NAVIGATION"
  )
fi
if [[ -n "$PAUSE_MEMORY_RECORDING" ]]; then
  EXTRA_PARAMS+=(-p pause_memory_recording:="$PAUSE_MEMORY_RECORDING")
fi
if [[ -n "$AUTO_GOAL_INTERVAL" ]]; then
  EXTRA_PARAMS+=(-p auto_goal_candidate_interval_frames:="$AUTO_GOAL_INTERVAL")
fi
if [[ -n "$AUTO_GOAL_MAX" ]]; then
  EXTRA_PARAMS+=(-p auto_goal_candidate_max:="$AUTO_GOAL_MAX")
fi
if [[ -n "$AUTO_GOAL_GUARD" ]]; then
  EXTRA_PARAMS+=(-p auto_goal_candidate_post_guard_frames:="$AUTO_GOAL_GUARD")
fi
if [[ -n "$AUTO_GOAL_CAPTURE_ENABLED" ]]; then
  EXTRA_PARAMS+=(
    -p auto_goal_candidate_capture_enabled:="$AUTO_GOAL_CAPTURE_ENABLED"
  )
fi
if [[ -n "$AUTO_SELECT_GOAL" ]]; then
  EXTRA_PARAMS+=(-p auto_select_goal_candidate:="$AUTO_SELECT_GOAL")
fi

if [[ ! -f "$CONFIG" ]]; then
  echo "Adapter config not found: $CONFIG" >&2
  exit 1
fi
echo "Starting ROS adapter: backend=$BACKEND mode=$MODE odometry=disabled"
if [[ "$MODE" == "imagegoal" ]]; then
  echo "  image goal: $IMAGE_GOAL_PATH"
  if [[ -n "$REVISIT_IMAGE_GOAL_PATH" ]]; then
    echo "  revisit image goal: $REVISIT_IMAGE_GOAL_PATH"
  fi
fi
if [[ -n "$MAX_LINEAR_MPS" || -n "$MAX_ANGULAR_RPS" ]]; then
  echo "  overrides: max_linear_mps=${MAX_LINEAR_MPS:-config} max_angular_rps=${MAX_ANGULAR_RPS:-config}"
fi
exec python "$NAVDP_GO2_DIR/navdp_ros_node.py" \
  --ros-args \
  --params-file "$CONFIG" \
  -p backend:="$BACKEND" \
  -p mode:="$MODE" \
  -p server_url:="$SERVER_URL" \
  "${EXTRA_PARAMS[@]}"
