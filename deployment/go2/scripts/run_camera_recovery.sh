#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros
navdp_activate_venv

case "$CFG_PROFILE" in
  native-navdp-rgbd) SESSION="$CFG_NATIVE_SESSION" ;;
  fullmono-lingbot-cec) SESSION="$CFG_FULLMONO_SESSION" ;;
  *) echo "Camera recovery unsupported for profile: $CFG_PROFILE" >&2; exit 2 ;;
esac

CAMERA_LOG="$CFG_JETSON_RUNTIME_ROOT/logs/realsense.log"
mkdir -p "$(dirname "$CAMERA_LOG")"
CAMERA_OWNER_ARGS=()
camera_owner="tmux:$SESSION:rgbd"
if tmux show-environment -t "$SESSION" MEMNAV_USES_BOOT_OBSERVER \
    2>/dev/null | grep -Fxq 'MEMNAV_USES_BOOT_OBSERVER=true'; then
  CAMERA_OWNER_ARGS=(--camera-systemd-unit memnav-observer-camera.service)
  camera_owner="systemd:memnav-observer-camera.service"
fi
echo "Starting fail-closed camera recovery service: owner=$camera_owner"
exec python "$NAVDP_GO2_DIR/camera_recovery_service.py" \
  --session "$SESSION" \
  --camera-script "$SCRIPT_DIR/run_realsense.sh" \
  --config "$NAVDP_RUN_CONFIG" \
  --camera-log "$CAMERA_LOG" \
  --rgb-topic "$CFG_RGB_TOPIC" \
  --depth-topic "$CFG_DEPTH_TOPIC" \
  --cmd-vel-topic "$CFG_GO2_CMD_TOPIC" \
  "${CAMERA_OWNER_ARGS[@]}"
