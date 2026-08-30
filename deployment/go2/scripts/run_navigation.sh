#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

usage() {
  cat <<'EOF'
Usage: run_navigation.sh --config RESOLVED_CONFIG.json [--timeout-s SECONDS]

Runs one supervised native NavDP episode. The command starts locked, verifies
one fresh post-reset trajectory, arms motion, monitors arrival, and asserts the
operator stop on every failure, interruption, or timeout.
EOF
}

config=""
timeout_s=60
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      config="$2"
      shift 2
      ;;
    --timeout-s)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      timeout_s="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$config" ]] || { usage >&2; exit 2; }
[[ "$timeout_s" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "--timeout-s must be a positive number" >&2
  exit 2
}
awk -v value="$timeout_s" 'BEGIN { exit !(value > 0 && value <= 900) }' || {
  echo "--timeout-s must be in (0, 900]" >&2
  exit 2
}

NAVDP_RUN_CONFIG="$(readlink -f "$config")"
[[ -f "$NAVDP_RUN_CONFIG" ]] || {
  echo "Resolved run config is missing: $NAVDP_RUN_CONFIG" >&2
  exit 1
}
navdp_load_config "$NAVDP_RUN_CONFIG"

[[ "$CFG_PROFILE" == native-navdp-rgbd ]] || {
  echo "One-command run currently requires profile=native-navdp-rgbd" >&2
  exit 2
}
[[ "$CFG_NAV_BACKEND" == navdp && "$CFG_NAV_MODE" == imagegoal ]] || {
  echo "One-command run requires backend=navdp mode=imagegoal" >&2
  exit 2
}
[[ "$CFG_WITH_GO2" == true ]] || {
  echo "One-command run requires launch.go2_bridge=true" >&2
  exit 2
}
[[ "$CFG_WITH_CAMERA" == true ]] || {
  echo "One-command run requires launch.camera=true" >&2
  exit 2
}
[[ "$CFG_ARRIVAL_MODULE" == rgb-homography ]] || {
  echo "One-command run requires arrival.module=rgb-homography" >&2
  exit 2
}
[[ -f "$CFG_ARRIVAL_GOAL" ]] || {
  echo "Arrival ImageGoal is missing: $CFG_ARRIVAL_GOAL" >&2
  exit 1
}

session="$CFG_NATIVE_SESSION"
tmux has-session -t "$session" 2>/dev/null || {
  echo "Native stack session is not running: $session" >&2
  exit 1
}
active_id="$(tmux show-environment -t "$session" MEMNAV_CONFIG_ID 2>/dev/null \
  | sed -n 's/^MEMNAV_CONFIG_ID=//p' || true)"
[[ "$active_id" == "$CFG_CONFIG_ID" ]] || {
  echo "Native stack contract changed: active=${active_id:-unknown} expected=$CFG_CONFIG_ID" >&2
  exit 1
}

navdp_source_ros
navdp_activate_venv

exec "$CFG_JETSON_PYTHON" "$NAVDP_GO2_DIR/navigation_run_agent.py" \
  --arrival-goal "$CFG_ARRIVAL_GOAL" \
  --rgb-topic "$CFG_RGB_TOPIC" \
  --arrival-phases "$CFG_ARRIVAL_PHASES" \
  --min-image-scale "$CFG_ARRIVAL_MIN_SCALE" \
  --max-image-scale "$CFG_ARRIVAL_MAX_SCALE" \
  --max-linear-mps "$CFG_MAX_LINEAR_MPS" \
  --max-angular-rps "$CFG_MAX_ANGULAR_RPS" \
  --timeout-s "$timeout_s"
