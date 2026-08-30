#!/usr/bin/env bash
set -euo pipefail

GO2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$GO2_DIR/../.." && pwd)"
CONFIG_TOOL="$REPO_ROOT/deployment/runtime_config.py"
PROFILE_TOOL="$GO2_DIR/stack_profiles.py"
DEFAULT_NATIVE="$REPO_ROOT/deployment/config/experiments/native_imagegoal.json"
DEFAULT_FULLMONO="$REPO_ROOT/deployment/config/experiments/fullmono_imagegoal.json"

usage() {
  cat <<'EOF'
Usage:
  nav_stack.sh list
  nav_stack.sh describe PROFILE
  nav_stack.sh resolve --config EXPERIMENT.json
  nav_stack.sh start --config EXPERIMENT.json [--dry-run]
  nav_stack.sh status [--config EXPERIMENT.json]
  nav_stack.sh stop [--config EXPERIMENT.json]

All behavior is read from one Git-managed experiment JSON and its referenced
system.json. The resolved, hash-verified JSON is the only runtime contract.
No NAVDP_*/CEC_* environment override is accepted.

Startup is motion-locked. It never clears estop or calls set_enabled=true.
EOF
}

die() { echo "nav_stack: $*" >&2; exit 1; }

resolve_config() {
  python3 "$CONFIG_TOOL" resolve --config "$1"
}

load_jetson_config() {
  local resolved="$1"
  python3 "$CONFIG_TOOL" verify --config "$resolved" --site jetson >/dev/null
  local config_exports
  config_exports="$(python3 "$CONFIG_TOOL" shell --config "$resolved" --site jetson)"
  eval "$config_exports"
}

show_contract() {
  local resolved="$1"
  load_jetson_config "$resolved"
  echo "Resolved stack contract:"
  echo "  config=$resolved"
  echo "  config_id=$CFG_CONFIG_ID"
  echo "  source_revision=$CFG_GIT_REVISION"
  echo "  experiment=$CFG_EXPERIMENT_ID phase=$CFG_EXPERIMENT_PHASE"
  echo "  profile=$CFG_PROFILE"
  echo "  authority_mode=$CFG_AUTHORITY_MODE"
  echo "  ImageGoal=$CFG_IMAGE_GOAL"
  echo "  ImageGoal_sha256=$CFG_IMAGE_GOAL_SHA256"
  echo "  arrival=$CFG_ARRIVAL_MODULE arrival_goal=$CFG_ARRIVAL_GOAL"
  echo "  camera=$CFG_WITH_CAMERA go2_bridge=$CFG_WITH_GO2 foxglove=$CFG_WITH_FOXGLOVE"
  if [[ "$CFG_WITH_FOXGLOVE" == true ]]; then
    echo "  Foxglove=ws://$CFG_FOXGLOVE_ADDRESS:$CFG_FOXGLOVE_PORT layout=$CFG_FOXGLOVE_LAYOUT"
  fi
  echo "  max_linear_mps=$CFG_MAX_LINEAR_MPS max_angular_rps=$CFG_MAX_ANGULAR_RPS"
}

start_stack() {
  local source="" dry_run=false
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config) [[ $# -ge 2 ]] || die "--config requires a value"; source="$2"; shift 2 ;;
      --dry-run) dry_run=true; shift ;;
      *) die "unknown start option: $1" ;;
    esac
  done
  [[ -n "$source" ]] || die "start requires --config EXPERIMENT.json"
  local resolved
  resolved="$(resolve_config "$source")"
  show_contract "$resolved"
  if [[ "$dry_run" == true ]]; then
    echo "DRY RUN: validated; no tmux session or process was started."
    return 0
  fi
  case "$CFG_PROFILE" in
    native-navdp-rgbd) bash "$GO2_DIR/scripts/run_stack.sh" --config "$resolved" ;;
    fullmono-lingbot-cec) bash "$GO2_DIR/offboard/fullmono.sh" start --config "$resolved" ;;
    *) die "profile has no launcher: $CFG_PROFILE" ;;
  esac
}

resolve_only() {
  [[ $# -eq 2 && "$1" == --config ]] || die "resolve requires --config EXPERIMENT.json"
  local resolved
  resolved="$(resolve_config "$2")"
  show_contract "$resolved"
}

status_one() {
  local resolved="$1"
  load_jetson_config "$resolved"
  local session="$CFG_NATIVE_SESSION"
  [[ "$CFG_PROFILE" != fullmono-lingbot-cec ]] || session="$CFG_FULLMONO_SESSION"
  if tmux has-session -t "$session" 2>/dev/null; then
    local active_id active_config
    active_id="$(tmux show-environment -t "$session" MEMNAV_CONFIG_ID 2>/dev/null | sed -n 's/^MEMNAV_CONFIG_ID=//p')"
    active_config="$(tmux show-environment -t "$session" MEMNAV_RUN_CONFIG 2>/dev/null | sed -n 's/^MEMNAV_RUN_CONFIG=//p')"
    echo "RUNNING session=$session profile=$CFG_PROFILE config_id=${active_id:-unknown}"
    echo "  config=${active_config:-unknown}"
    tmux list-windows -t "$session" -F '  window=#{window_name} dead=#{pane_dead}'
  else
    echo "STOPPED session=$session profile=$CFG_PROFILE"
  fi
}

status_stack() {
  if [[ $# -eq 0 ]]; then
    status_one "$(resolve_config "$DEFAULT_NATIVE")"
    status_one "$(resolve_config "$DEFAULT_FULLMONO")"
    echo "Motion authority must be checked from /navdp/status; startup is locked."
    return
  fi
  [[ $# -eq 2 && "$1" == --config ]] || die "status accepts [--config EXPERIMENT.json]"
  local resolved
  resolved="$(resolve_config "$2")"
  load_jetson_config "$resolved"
  if [[ "$CFG_PROFILE" == fullmono-lingbot-cec ]]; then
    bash "$GO2_DIR/offboard/fullmono.sh" status --config "$resolved"
  else
    status_one "$resolved"
  fi
}

stop_one() {
  local resolved="$1"
  load_jetson_config "$resolved"
  if [[ "$CFG_PROFILE" == fullmono-lingbot-cec ]]; then
    bash "$GO2_DIR/offboard/fullmono.sh" stop --config "$resolved"
  else
    bash "$GO2_DIR/scripts/stop_stack.sh" --config "$resolved"
  fi
}

stop_stack() {
  if [[ $# -eq 0 ]]; then
    local system_exports
    system_exports="$(python3 "$CONFIG_TOOL" system-shell \
      --config "$REPO_ROOT/deployment/config/system.json" --site jetson)"
    eval "$system_exports"
    for session in "$CFG_NATIVE_SESSION" "$CFG_FULLMONO_SESSION"; do
      if tmux has-session -t "$session" 2>/dev/null; then
        tmux kill-session -t "$session"
        echo "Stopped Jetson tmux session: $session"
      else
        echo "No Jetson tmux session named $session"
      fi
    done
    if ssh -o BatchMode=yes -o ConnectTimeout=5 "$CFG_GPU_HOST" \
        "tmux kill-session -t $(printf '%q' "$CFG_GPU_SESSION") 2>/dev/null"; then
      echo "Stopped RTX tmux session: $CFG_GPU_HOST:$CFG_GPU_SESSION"
    else
      echo "RTX session absent or host unreachable: $CFG_GPU_HOST:$CFG_GPU_SESSION" >&2
    fi
    return
  fi
  [[ $# -eq 2 && "$1" == --config ]] || die "stop accepts [--config EXPERIMENT.json]"
  stop_one "$(resolve_config "$2")"
}

[[ $# -gt 0 ]] || { usage; exit 2; }
command="$1"
shift
case "$command" in
  list) [[ $# -eq 0 ]] || die "list takes no arguments"; python3 "$PROFILE_TOOL" list ;;
  describe) [[ $# -eq 1 ]] || die "describe requires PROFILE"; python3 "$PROFILE_TOOL" show "$1" ;;
  resolve) resolve_only "$@" ;;
  start) start_stack "$@" ;;
  status) status_stack "$@" ;;
  stop) stop_stack "$@" ;;
  -h|--help|help) usage ;;
  *) die "unknown command: $command" ;;
esac
