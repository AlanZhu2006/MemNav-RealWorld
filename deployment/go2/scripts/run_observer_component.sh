#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

usage() {
  echo "Usage: ${0##*/} {camera|battery|preview|foxglove}" >&2
}

[[ $# -eq 1 ]] || { usage; exit 2; }
component="$1"
source_config="$NAVDP_ROOT/deployment/config/experiments/native_imagegoal.json"
resolved_config="$(python3 "$NAVDP_RUNTIME_CONFIG_TOOL" resolve --config "$source_config")"
navdp_load_config "$resolved_config"

case "$component" in
  camera)
    [[ "$CFG_WITH_CAMERA" == true ]] || {
      echo "Native experiment has camera disabled." >&2
      exit 1
    }
    exec bash "$SCRIPT_DIR/run_realsense.sh" --config "$resolved_config"
    ;;
  battery)
    exec bash "$SCRIPT_DIR/run_go2_battery_monitor.sh" --config "$resolved_config"
    ;;
  preview)
    exec bash "$SCRIPT_DIR/run_foxglove_preview.sh" \
      --observer-only --config "$resolved_config"
    ;;
  foxglove)
    [[ "$CFG_WITH_FOXGLOVE" == true ]] || {
      echo "Native experiment has Foxglove disabled." >&2
      exit 1
    }
    exec bash "$SCRIPT_DIR/run_foxglove_bridge.sh" --config "$resolved_config"
    ;;
  *)
    usage
    exit 2
    ;;
esac
