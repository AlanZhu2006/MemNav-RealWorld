#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros
navdp_activate_venv

CONFIG="$CFG_ADAPTER_PARAMS"
BACKEND="$CFG_NAV_BACKEND"
MODE="$CFG_NAV_MODE"
if [[ "$CFG_PROFILE" == fullmono-lingbot-cec ]]; then
  SERVER_URL="http://127.0.0.1:$CFG_TUNNEL_LOCAL_PORT"
else
  SERVER_URL="http://$CFG_NATIVE_HOST:$CFG_NATIVE_PORT"
fi
MAX_LINEAR_MPS="$CFG_MAX_LINEAR_MPS"
MAX_ANGULAR_RPS="$CFG_MAX_ANGULAR_RPS"
IMAGE_GOAL_PATH="$CFG_IMAGE_GOAL"
REVISIT_IMAGE_GOAL_PATH="$CFG_REVISIT_IMAGE_GOAL"
SELECTED_GOAL_IMAGE_PATH="$CFG_SELECTED_GOAL_IMAGE"
SELECTED_GOAL_DEPTH_PATH="$CFG_SELECTED_GOAL_DEPTH"
TWO_PHASE="$CFG_TWO_PHASE"
NOVEL_RECORDING_NAVIGATION="$CFG_NAVIGATE_DURING_RECORDING"
PAUSE_MEMORY_RECORDING="$CFG_PAUSE_RECORDING"
AUTO_GOAL_INTERVAL="$CFG_AUTO_GOAL_INTERVAL"
AUTO_GOAL_MAX="$CFG_AUTO_GOAL_MAX"
AUTO_GOAL_GUARD="$CFG_AUTO_GOAL_GUARD"
AUTO_GOAL_CAPTURE_ENABLED="$CFG_AUTO_GOAL_CAPTURE"
AUTO_SELECT_GOAL="$CFG_AUTO_SELECT_GOAL"
SURVEY_DATASET_ID="$CFG_DATASET_ID"
SURVEY_SEAL_RECEIPT_PATH=""
if [[ -n "$SURVEY_DATASET_ID" ]]; then
  SURVEY_SEAL_RECEIPT_PATH="$CFG_JETSON_RUNTIME_ROOT/two_pass_revisit/$SURVEY_DATASET_ID/survey_seal.json"
fi
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
if [[ -n "$SURVEY_DATASET_ID" ]]; then
  EXTRA_PARAMS+=(
    -p survey_dataset_id:="$SURVEY_DATASET_ID"
    -p survey_seal_receipt_path:="$SURVEY_SEAL_RECEIPT_PATH"
  )
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
  -p rgb_topic:="$CFG_RGB_TOPIC" \
  -p depth_topic:="$CFG_DEPTH_TOPIC" \
  -p camera_info_topic:="$CFG_CAMERA_INFO_TOPIC" \
  -p cmd_vel_topic:="$CFG_GO2_CMD_TOPIC" \
  "${EXTRA_PARAMS[@]}"
