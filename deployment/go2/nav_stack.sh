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
  nav_stack.sh start --config EXPERIMENT.json [--refresh] [--dry-run]
  nav_stack.sh run [--config EXPERIMENT.json] [--timeout-s SECONDS]
  nav_stack.sh status [--config EXPERIMENT.json]
  nav_stack.sh stop [--config EXPERIMENT.json]

All behavior is read from one Git-managed experiment JSON and its referenced
system.json. The resolved, hash-verified JSON is the only runtime contract.
No NAVDP_*/CEC_* environment override is accepted.

Startup is motion-locked. It never clears estop or calls set_enabled=true.
Starting an already-running, healthy instance of the exact same contract first
confirms disabled + estop and then reuses it. Use --refresh to deliberately
replace every process; stale, incomplete, or dead sessions are always replaced.

"run" is the explicit onsite motion command for the native profile. It reuses
a healthy current stack, cold-refreshes only when required, prints phase
timings, verifies a fresh post-reset plan, arms, monitors, and fails closed.
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

stop_existing_local_stack() {
  local resolved="$1" session="$2" profile="$3"
  tmux has-session -t "$session" 2>/dev/null || return 0

  local active_config stop_config
  active_config="$(tmux show-environment -t "$session" MEMNAV_RUN_CONFIG 2>/dev/null \
    | sed -n 's/^MEMNAV_RUN_CONFIG=//p' || true)"
  stop_config="$resolved"
  if [[ -n "$active_config" && -f "$active_config" ]]; then
    stop_config="$active_config"
  fi

  echo "Replacing the complete running stack: session=$session"
  echo "  old_config=${active_config:-unknown}"
  echo "  new_config=$resolved"
  case "$profile" in
    native-navdp-rgbd)
      NAVDP_OBSERVER_RESUME=false \
        bash "$GO2_DIR/scripts/stop_stack.sh" --config "$stop_config"
      ;;
    fullmono-lingbot-cec)
      NAVDP_OBSERVER_RESUME=false \
        bash "$GO2_DIR/offboard/fullmono.sh" stop --config "$stop_config"
      ;;
    *) die "profile has no stop path: $profile" ;;
  esac
}

start_stack() {
  local source="" dry_run=false refresh=false
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config) [[ $# -ge 2 ]] || die "--config requires a value"; source="$2"; shift 2 ;;
      --refresh) refresh=true; shift ;;
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
  if [[ "$refresh" != true ]] && profile_session_is_current_and_healthy; then
    if bash "$GO2_DIR/scripts/lock_running_stack.sh" --config "$resolved"; then
      echo "FAST START: reusing healthy $CFG_PROFILE stack config_id=$CFG_CONFIG_ID"
      return 0
    fi
    echo "Existing stack could not prove motion lock; replacing it fail-closed." >&2
  fi
  # Retire the former observation-only session transparently.  The normal
  # stack now stays available while the Go2 network is offline.
  local legacy_ui_session="${CFG_NATIVE_SESSION}-camera-ui"
  if tmux has-session -t "$legacy_ui_session" 2>/dev/null; then
    tmux kill-session -t "$legacy_ui_session"
    echo "Stopped legacy UI session before normal stack startup: $legacy_ui_session"
  fi
  case "$CFG_PROFILE" in
    native-navdp-rgbd)
      stop_existing_local_stack "$resolved" "$CFG_NATIVE_SESSION" "$CFG_PROFILE"
      bash "$GO2_DIR/scripts/run_stack.sh" --config "$resolved"
      ;;
    fullmono-lingbot-cec)
      stop_existing_local_stack "$resolved" "$CFG_FULLMONO_SESSION" "$CFG_PROFILE"
      bash "$GO2_DIR/offboard/fullmono.sh" start --config "$resolved"
      ;;
    *) die "profile has no launcher: $CFG_PROFILE" ;;
  esac
}

native_session_is_current_and_healthy() {
  local session="$CFG_NATIVE_SESSION"
  tmux has-session -t "$session" 2>/dev/null || return 1
  local active_id windows window_states
  active_id="$(tmux show-environment -t "$session" MEMNAV_CONFIG_ID 2>/dev/null \
    | sed -n 's/^MEMNAV_CONFIG_ID=//p' || true)"
  [[ "$active_id" == "$CFG_CONFIG_ID" ]] || return 1
  window_states="$(tmux list-windows -t "$session" \
    -F '#{window_name} #{pane_dead}' 2>/dev/null)" \
    || return 1
  grep -Eq ' 1$' <<<"$window_states" && return 1
  windows="$(cut -d' ' -f1 <<<"$window_states")"
  local required=(policy adapter)
  [[ "$CFG_WITH_CAMERA" != true ]] || required+=(rgbd camera-recovery)
  [[ "$CFG_ARRIVAL_MODULE" != rgb-homography ]] || required+=(arrival)
  [[ "$CFG_WITH_GO2" != true ]] || required+=(go2)
  [[ "$CFG_WITH_FOXGLOVE" != true ]] \
    || required+=(fox-preview foxglove)
  local window
  for window in "${required[@]}"; do
    grep -Fxq "$window" <<<"$windows" || return 1
  done
}

fullmono_session_is_current_and_healthy() {
  local session="$CFG_FULLMONO_SESSION"
  tmux has-session -t "$session" 2>/dev/null || return 1
  local active_id windows window_states uses_boot_observer
  active_id="$(tmux show-environment -t "$session" MEMNAV_CONFIG_ID 2>/dev/null \
    | sed -n 's/^MEMNAV_CONFIG_ID=//p' || true)"
  [[ "$active_id" == "$CFG_CONFIG_ID" ]] || return 1
  window_states="$(tmux list-windows -t "$session" \
    -F '#{window_name} #{pane_dead}' 2>/dev/null)" \
    || return 1
  grep -Eq ' 1$' <<<"$window_states" && return 1
  windows="$(cut -d' ' -f1 <<<"$window_states")"
  uses_boot_observer="$(tmux show-environment -t "$session" \
    MEMNAV_USES_BOOT_OBSERVER 2>/dev/null \
    | sed -n 's/^MEMNAV_USES_BOOT_OBSERVER=//p' || true)"
  local required=(tunnel adapter)
  if [[ "$CFG_WITH_CAMERA" == true ]]; then
    required+=(camera-recovery)
    [[ "$uses_boot_observer" == true ]] || required+=(rgbd)
  fi
  [[ "$CFG_ARRIVAL_MODULE" != rgb-homography ]] || required+=(arrival)
  [[ "$CFG_WITH_GO2" != true ]] || required+=(go2)
  if [[ "$CFG_WITH_FOXGLOVE" == true \
      && "$uses_boot_observer" != true ]]; then
    required+=(fox-preview foxglove)
  fi
  local window
  for window in "${required[@]}"; do
    grep -Fxq "$window" <<<"$windows" || return 1
  done
  if [[ "$uses_boot_observer" == true ]]; then
    if [[ "$CFG_WITH_GO2" == true ]]; then
      navdp_boot_observer_visuals_are_healthy || return 1
    else
      navdp_boot_observer_is_healthy || return 1
    fi
  fi
  curl -fsS --max-time 2 \
    "http://127.0.0.1:${CFG_TUNNEL_LOCAL_PORT}/healthz" >/dev/null 2>&1
}

profile_session_is_current_and_healthy() {
  case "$CFG_PROFILE" in
    native-navdp-rgbd) native_session_is_current_and_healthy ;;
    fullmono-lingbot-cec) fullmono_session_is_current_and_healthy ;;
    *) return 1 ;;
  esac
}

run_navigation() {
  local source="$DEFAULT_NATIVE" timeout_s=60
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config)
        [[ $# -ge 2 ]] || die "--config requires a value"
        source="$2"
        shift 2
        ;;
      --timeout-s)
        [[ $# -ge 2 ]] || die "--timeout-s requires a value"
        timeout_s="$2"
        shift 2
        ;;
      *) die "unknown run option: $1" ;;
    esac
  done

  [[ "$timeout_s" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
    die "--timeout-s must be a positive number"
  }
  awk -v value="$timeout_s" \
    'BEGIN { exit !(value > 0 && value <= 900) }' || {
    die "--timeout-s must be in (0, 900]"
  }

  local resolved
  resolved="$(resolve_config "$source")"
  load_jetson_config "$resolved"
  [[ "$CFG_PROFILE" == native-navdp-rgbd ]] || {
    die "run currently supports profile=native-navdp-rgbd only"
  }

  if native_session_is_current_and_healthy; then
    echo "FAST PATH: reusing current healthy stack config_id=$CFG_CONFIG_ID"
  else
    echo "COLD PATH: stack is absent, stale, incomplete, or unhealthy"
    start_stack --config "$source"
    load_jetson_config "$resolved"
  fi

  bash "$GO2_DIR/scripts/run_navigation.sh" \
    --config "$resolved" --timeout-s "$timeout_s"
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
    local contract_state="stale"
    [[ "$active_id" == "$CFG_CONFIG_ID" ]] && contract_state="current"
    echo "RUNNING session=$session profile=$CFG_PROFILE contract=$contract_state config_id=${active_id:-unknown}"
    echo "  config=${active_config:-unknown}"
    if [[ "$contract_state" == stale ]]; then
      echo "  expected_config_id=$CFG_CONFIG_ID"
      echo "  use nav_stack.sh run to refresh and navigate, or start to refresh locked"
    fi
    tmux list-windows -t "$session" -F '  window=#{window_name} dead=#{pane_dead}'
  else
    echo "STOPPED session=$session profile=$CFG_PROFILE"
  fi
}

status_observer() {
  if systemctl --user is-active --quiet memnav-observer.target 2>/dev/null; then
    echo "OBSERVER RUNNING camera + Foxglove + diagnostics + battery (navigation off)"
  elif systemctl --user is-enabled --quiet memnav-observer.target 2>/dev/null; then
    echo "OBSERVER STOPPED (enabled for boot; navigation may currently own devices)"
  else
    echo "OBSERVER NOT INSTALLED"
  fi
}

status_stack() {
  if [[ $# -eq 0 ]]; then
    status_one "$(resolve_config "$DEFAULT_NATIVE")"
    status_one "$(resolve_config "$DEFAULT_FULLMONO")"
    status_observer
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
  status_observer
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
    for session in "$CFG_NATIVE_SESSION" "$CFG_FULLMONO_SESSION" \
        "${CFG_NATIVE_SESSION}-camera-ui"; do
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
    if systemctl --user is-enabled --quiet memnav-observer.target 2>/dev/null; then
      systemctl --user start memnav-observer.target
      echo "Restored the always-on camera and Foxglove observer."
    fi
    return
  fi
  [[ $# -eq 2 && "$1" == --config ]] || die "stop accepts [--config EXPERIMENT.json]"
  stop_one "$(resolve_config "$2")"
}

main() {
  [[ $# -gt 0 ]] || { usage; return 2; }
  local command="$1"
  shift
  case "$command" in
    list) [[ $# -eq 0 ]] || die "list takes no arguments"; python3 "$PROFILE_TOOL" list ;;
    describe) [[ $# -eq 1 ]] || die "describe requires PROFILE"; python3 "$PROFILE_TOOL" show "$1" ;;
    resolve) resolve_only "$@" ;;
    start) start_stack "$@" ;;
    run) run_navigation "$@" ;;
    status) status_stack "$@" ;;
    stop) stop_stack "$@" ;;
    -h|--help|help) usage ;;
    *) die "unknown command: $command" ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
