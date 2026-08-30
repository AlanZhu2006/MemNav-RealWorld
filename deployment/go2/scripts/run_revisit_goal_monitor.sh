#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

usage() {
  echo "Usage: run_revisit_goal_monitor.sh --config RESOLVED.json --goal IMAGE [--point-label M]" >&2
}

config=""
goal=""
point_label="M"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) [[ $# -ge 2 ]] || { usage; exit 2; }; config="$2"; shift 2 ;;
    --goal) [[ $# -ge 2 ]] || { usage; exit 2; }; goal="$2"; shift 2 ;;
    --point-label) [[ $# -ge 2 ]] || { usage; exit 2; }; point_label="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

[[ -n "$config" && -n "$goal" ]] || { usage; exit 2; }
NAVDP_RUN_CONFIG="$(readlink -f "$config")"
goal="$(readlink -f "$goal")"
[[ -f "$NAVDP_RUN_CONFIG" ]] || { echo "Resolved config missing: $NAVDP_RUN_CONFIG" >&2; exit 1; }
[[ -f "$goal" ]] || { echo "Frozen Revisit goal missing: $goal" >&2; exit 1; }
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros
navdp_activate_venv

exec "$CFG_JETSON_PYTHON" "$NAVDP_GO2_DIR/revisit_goal_monitor.py" \
  --goal "$goal" \
  --point-label "$point_label" \
  --rgb-topic "$CFG_RGB_TOPIC" \
  --rate-hz "$CFG_FOXGLOVE_PREVIEW_ARRIVAL_FPS" \
  --min-image-scale "$CFG_ARRIVAL_MIN_SCALE" \
  --max-image-scale "$CFG_ARRIVAL_MAX_SCALE"
