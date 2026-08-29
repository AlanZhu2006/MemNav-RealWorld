#!/usr/bin/env bash
set -euo pipefail

GO2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_TOOL="$GO2_DIR/stack_profiles.py"
NATIVE_SESSION="${NAVDP_NATIVE_SESSION:-navdp-go2}"
FULLMONO_SESSION="${NAVDP_FULLMONO_SESSION:-navdp-go2-offboard}"

usage() {
  cat <<'EOF'
Usage:
  nav_stack.sh list
  nav_stack.sh describe PROFILE
  nav_stack.sh start --profile PROFILE --goal IMAGE [options]
  nav_stack.sh status
  nav_stack.sh stop [--profile PROFILE]

Profiles:
  native-navdp-rgbd       original NavDP + live D435i RGB-D; no CEC/LingBot
  fullmono-lingbot-cec    frozen NavDP + LingBot depth + CEC memory/proof

Arrival modules (independent from the navigation profile):
  operator                no autonomous termination process (default)
  external-topic          tag/SLAM/evaluator publishes /navdp/arrival
  rgb-homography          temporary RGB geometry gate

Start options:
  --arrival MODULE        termination module (default: operator)
  --arrival-goal IMAGE    independent arrival reference (default: --goal)
  --arrival-phases LIST   adapter phases that arm arrival matching
  --revisit-goal IMAGE    Full-Mono revisit target
  --camera-height METRES  Full-Mono measured optical-center height
  --novel-navigation      allow Full-Mono motion while recording Novel memory
  --max-linear MPS        adapter linear speed limit
  --max-angular RPS       adapter angular speed limit
  --with-go2              start the locked Go2 bridge
  --with-rviz             start RViz
  --no-camera             native profile only; use an existing camera process
  --dry-run               validate and print the resolved contract without starting

All profiles start motion-locked. This launcher never clears estop and never
calls set_enabled=true.
EOF
}

die() {
  echo "nav_stack: $*" >&2
  exit 1
}

require_value() {
  [[ $# -ge 2 && -n "$2" ]] || die "$1 requires a value"
}

canonical_profile() {
  python3 "$PROFILE_TOOL" get "$1" name
}

validate_arrival_phases() {
  local list="$1"
  local phase
  local phases=()
  IFS=',' read -r -a phases <<<"$list"
  [[ ${#phases[@]} -gt 0 ]] || die "arrival phase list is empty"
  for phase in "${phases[@]}"; do
    case "$phase" in
      memory_recording|revisit_query) ;;
      *) die "unknown arrival phase: $phase" ;;
    esac
  done
}

session_value() {
  local session="$1"
  local key="$2"
  tmux show-environment -t "$session" "$key" 2>/dev/null \
    | sed -n "s/^${key}=//p"
}

show_session() {
  local session="$1"
  local fallback_profile="$2"
  if ! tmux has-session -t "$session" 2>/dev/null; then
    echo "STOPPED  session=$session profile=$fallback_profile"
    return 1
  fi
  local profile arrival navigation_goal arrival_goal arrival_phases
  profile="$(session_value "$session" NAVDP_STACK_PROFILE)"
  arrival="$(session_value "$session" NAVDP_ARRIVAL_MODULE)"
  navigation_goal="$(session_value "$session" NAVDP_NAVIGATION_GOAL_PATH)"
  arrival_goal="$(session_value "$session" NAVDP_ARRIVAL_GOAL_PATH)"
  arrival_phases="$(session_value "$session" NAVDP_ARRIVAL_ALLOWED_PHASES)"
  echo "RUNNING  session=$session profile=${profile:-$fallback_profile} arrival=${arrival:-unknown}"
  echo "  navigation_goal=${navigation_goal:-unknown}"
  echo "  arrival_goal=${arrival_goal:-unknown}"
  echo "  arrival_phases=${arrival_phases:-unknown}"
  tmux list-windows -t "$session" -F '  window=#{window_name} dead=#{pane_dead}'
}

start_stack() {
  local profile=""
  local goal=""
  local arrival="operator"
  local arrival_goal=""
  local arrival_phases="${NAVDP_ARRIVAL_ALLOWED_PHASES:-}"
  local revisit_goal=""
  local camera_height="${CEC_CAMERA_HEIGHT_M:-}"
  local max_linear="${NAVDP_MAX_LINEAR_MPS:-}"
  local max_angular="${NAVDP_MAX_ANGULAR_RPS:-}"
  local with_go2=false
  local with_rviz=false
  local with_camera=true
  local novel_navigation=false
  local dry_run=false

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --profile) require_value "$@"; profile="$2"; shift 2 ;;
      --goal) require_value "$@"; goal="$2"; shift 2 ;;
      --arrival) require_value "$@"; arrival="$2"; shift 2 ;;
      --arrival-goal) require_value "$@"; arrival_goal="$2"; shift 2 ;;
      --arrival-phases) require_value "$@"; arrival_phases="$2"; shift 2 ;;
      --revisit-goal) require_value "$@"; revisit_goal="$2"; shift 2 ;;
      --camera-height) require_value "$@"; camera_height="$2"; shift 2 ;;
      --max-linear) require_value "$@"; max_linear="$2"; shift 2 ;;
      --max-angular) require_value "$@"; max_angular="$2"; shift 2 ;;
      --novel-navigation) novel_navigation=true; shift ;;
      --with-go2) with_go2=true; shift ;;
      --with-rviz) with_rviz=true; shift ;;
      --no-camera) with_camera=false; shift ;;
      --dry-run) dry_run=true; shift ;;
      -h|--help) usage; return 0 ;;
      *) die "unknown start option: $1" ;;
    esac
  done

  [[ -n "$profile" ]] || die "start requires --profile"
  [[ -n "$goal" ]] || die "start requires an explicit --goal"
  [[ -f "$goal" ]] || die "navigation ImageGoal does not exist: $goal"
  profile="$(canonical_profile "$profile")"
  read -r _ arrival < <(python3 "$PROFILE_TOOL" validate "$profile" "$arrival")
  goal="$(readlink -f "$goal")"
  if [[ -z "$arrival_goal" ]]; then
    arrival_goal="$goal"
  fi
  [[ -f "$arrival_goal" ]] || die "arrival reference does not exist: $arrival_goal"
  arrival_goal="$(readlink -f "$arrival_goal")"
  if [[ -n "$revisit_goal" ]]; then
    [[ -f "$revisit_goal" ]] || die "revisit ImageGoal does not exist: $revisit_goal"
    revisit_goal="$(readlink -f "$revisit_goal")"
  fi

  if tmux has-session -t "$NATIVE_SESSION" 2>/dev/null \
      || tmux has-session -t "$FULLMONO_SESSION" 2>/dev/null; then
    die "a navigation stack is already running; use '$0 status'"
  fi

  echo "Resolved stack contract:"
  python3 "$PROFILE_TOOL" show "$profile"
  echo "navigation_goal=$goal"
  echo "arrival_module=$arrival"
  echo "arrival_goal=$arrival_goal"

  case "$profile" in
    native-navdp-rgbd)
      [[ -z "$revisit_goal" ]] \
        || die "native-navdp-rgbd has no CEC/two-phase revisit target"
      [[ "$novel_navigation" == false ]] \
        || die "--novel-navigation belongs to the Full-Mono two-phase profile"
      if [[ -z "$arrival_phases" ]]; then
        arrival_phases="revisit_query"
      fi
      [[ "$arrival_phases" == revisit_query ]] \
        || die "native-navdp-rgbd reports phase=revisit_query"
      ;;
    fullmono-lingbot-cec)
      [[ "$with_camera" == true ]] \
        || die "Full-Mono owns its camera process; --no-camera is unsupported"
      [[ -n "$camera_height" ]] \
        || die "Full-Mono requires --camera-height or CEC_CAMERA_HEIGHT_M"
      if [[ -z "$arrival_phases" ]]; then
        arrival_phases="memory_recording"
      fi
      validate_arrival_phases "$arrival_phases"
      ;;
    *) die "profile has no launcher implementation: $profile" ;;
  esac
  echo "arrival_phases=$arrival_phases"
  if [[ "$dry_run" == true ]]; then
    echo "DRY RUN: contract validated; no process or tmux session was started."
    return 0
  fi

  local launch_args=()
  [[ "$with_go2" == true ]] && launch_args+=(--with-go2)
  [[ "$with_rviz" == true ]] && launch_args+=(--with-rviz)

  case "$profile" in
    native-navdp-rgbd)
      [[ "$with_camera" == true ]] || launch_args+=(--no-camera)
      env \
        NAVDP_TMUX_SESSION="$NATIVE_SESSION" \
        NAVDP_IMAGE_GOAL_PATH="$goal" \
        NAVDP_ARRIVAL_MODULE="$arrival" \
        NAVDP_ARRIVAL_GOAL_PATH="$arrival_goal" \
        NAVDP_ARRIVAL_ALLOWED_PHASES="$arrival_phases" \
        NAVDP_MAX_LINEAR_MPS="$max_linear" \
        NAVDP_MAX_ANGULAR_RPS="$max_angular" \
        bash "$GO2_DIR/scripts/run_stack.sh" \
          --backend base --mode imagegoal --arrival "$arrival" \
          "${launch_args[@]}"
      ;;
    fullmono-lingbot-cec)
      local fullmono_args=(start)
      [[ "$with_go2" == true ]] && fullmono_args+=(--with-go2)
      [[ "$with_rviz" == true ]] && fullmono_args+=(--with-rviz)
      env \
        NAVDP_TMUX_SESSION="$FULLMONO_SESSION" \
        CEC_CAMERA_HEIGHT_M="$camera_height" \
        NAVDP_IMAGE_GOAL_PATH="$goal" \
        NAVDP_REVISIT_IMAGE_GOAL_PATH="$revisit_goal" \
        NAVDP_NAVIGATE_DURING_MEMORY_RECORDING="$novel_navigation" \
        NAVDP_ARRIVAL_MODULE="$arrival" \
        NAVDP_ARRIVAL_GOAL_PATH="$arrival_goal" \
        NAVDP_ARRIVAL_ALLOWED_PHASES="$arrival_phases" \
        NAVDP_MAX_LINEAR_MPS="$max_linear" \
        NAVDP_MAX_ANGULAR_RPS="$max_angular" \
        bash "$GO2_DIR/offboard/fullmono.sh" "${fullmono_args[@]}"
      ;;
    *) die "profile has no launcher implementation: $profile" ;;
  esac
}

status_stack() {
  local active=0
  show_session "$NATIVE_SESSION" native-navdp-rgbd || true
  if tmux has-session -t "$NATIVE_SESSION" 2>/dev/null; then active=$((active + 1)); fi
  show_session "$FULLMONO_SESSION" fullmono-lingbot-cec || true
  if tmux has-session -t "$FULLMONO_SESSION" 2>/dev/null; then active=$((active + 1)); fi
  echo "active_stack_count=$active"
  echo "Motion authority must be checked from /navdp/status; startup is locked."
}

stop_stack() {
  local requested="all"
  if [[ $# -gt 0 ]]; then
    [[ "$1" == --profile ]] || die "stop accepts only --profile PROFILE"
    require_value "$@"
    requested="$(canonical_profile "$2")"
    shift 2
  fi
  [[ $# -eq 0 ]] || die "unexpected stop arguments"
  if [[ "$requested" == all || "$requested" == native-navdp-rgbd ]]; then
    NAVDP_TMUX_SESSION="$NATIVE_SESSION" \
      bash "$GO2_DIR/scripts/stop_stack.sh"
  fi
  if [[ "$requested" == all || "$requested" == fullmono-lingbot-cec ]]; then
    NAVDP_TMUX_SESSION="$FULLMONO_SESSION" \
      bash "$GO2_DIR/offboard/fullmono.sh" stop
  fi
}

[[ $# -gt 0 ]] || { usage; exit 2; }
command="$1"
shift
case "$command" in
  list) [[ $# -eq 0 ]] || die "list takes no arguments"; python3 "$PROFILE_TOOL" list ;;
  describe) [[ $# -eq 1 ]] || die "describe requires PROFILE"; python3 "$PROFILE_TOOL" show "$1" ;;
  start) start_stack "$@" ;;
  status) [[ $# -eq 0 ]] || die "status takes no arguments"; status_stack ;;
  stop) stop_stack "$@" ;;
  -h|--help|help) usage ;;
  *) die "unknown command: $command" ;;
esac
