#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

usage() {
  cat <<'EOF'
Usage: run_navigation.sh --config RESOLVED_CONFIG.json [--timeout-s SECONDS]

Runs one supervised NavDP episode. The command starts locked, verifies one
fresh trajectory, arms motion, monitors arrival, and asserts the operator stop
on every failure, interruption, or timeout. Native runs reset the policy;
Formal Full-Mono runs preserve and verify the prepared Revisit state.
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

case "$CFG_PROFILE" in
  native-navdp-rgbd)
    session="$CFG_NATIVE_SESSION"
    stack_label="Native"
    policy_state_args=()
    ;;
  fullmono-lingbot-cec)
    session="$CFG_FULLMONO_SESSION"
    stack_label="Full-Mono"
    expected_dataset_id="$(python3 "$NAVDP_RUNTIME_CONFIG_TOOL" get \
      --config "$NAVDP_RUN_CONFIG" dataset.metadata.formal_dataset_id)"
    expected_dataset_sha256="$(python3 "$NAVDP_RUNTIME_CONFIG_TOOL" get \
      --config "$NAVDP_RUN_CONFIG" formal.expected_dataset_sha256)"
    [[ -s "$CFG_SELECTED_GOAL_IMAGE" ]] || {
      echo "Installed Full-Mono goal is missing: $CFG_SELECTED_GOAL_IMAGE" >&2
      exit 1
    }
    expected_goal_sha256="$(sha256sum "$CFG_SELECTED_GOAL_IMAGE" | awk '{print $1}')"
    [[ "$expected_dataset_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ \
        && "$expected_dataset_sha256" =~ ^[0-9a-f]{64}$ \
        && "$expected_goal_sha256" =~ ^[0-9a-f]{64}$ ]] || {
      echo "Full-Mono prepared Revisit identity is invalid" >&2
      exit 1
    }
    policy_state_args=(
      --preserve-policy-state
      --expected-dataset-id "$expected_dataset_id"
      --expected-dataset-sha256 "$expected_dataset_sha256"
      --expected-goal-sha256 "$expected_goal_sha256"
    )
    ;;
  *)
    echo "One-command run does not support profile=$CFG_PROFILE" >&2
    exit 2
    ;;
esac
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

tmux has-session -t "$session" 2>/dev/null || {
  echo "$stack_label stack session is not running: $session" >&2
  exit 1
}
active_id="$(tmux show-environment -t "$session" MEMNAV_CONFIG_ID 2>/dev/null \
  | sed -n 's/^MEMNAV_CONFIG_ID=//p' || true)"
[[ "$active_id" == "$CFG_CONFIG_ID" ]] || {
  echo "$stack_label stack contract changed: active=${active_id:-unknown} expected=$CFG_CONFIG_ID" >&2
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
  "${policy_state_args[@]}" \
  --timeout-s "$timeout_s"
