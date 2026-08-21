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
TWO_PHASE="${NAVDP_TWO_PHASE:-}"
EXTRA_PARAMS=()

if [[ -n "$MAX_LINEAR_MPS" ]]; then
  EXTRA_PARAMS+=(-p max_linear_mps:="$MAX_LINEAR_MPS")
fi
if [[ -n "$MAX_ANGULAR_RPS" ]]; then
  EXTRA_PARAMS+=(-p max_angular_rps:="$MAX_ANGULAR_RPS")
fi
if [[ "$MODE" == "imagegoal" ]]; then
  EXTRA_PARAMS+=(-p image_goal_path:="$IMAGE_GOAL_PATH")
fi
if [[ -n "$TWO_PHASE" ]]; then
  EXTRA_PARAMS+=(-p two_phase_episode:="$TWO_PHASE")
fi

if [[ ! -f "$CONFIG" ]]; then
  echo "Adapter config not found: $CONFIG" >&2
  exit 1
fi
echo "Starting ROS adapter: backend=$BACKEND mode=$MODE odometry=disabled"
if [[ "$MODE" == "imagegoal" ]]; then
  echo "  image goal: $IMAGE_GOAL_PATH"
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
