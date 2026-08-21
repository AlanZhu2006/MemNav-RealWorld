#!/usr/bin/env bash
set -euo pipefail

# Jetson-side entry point for the two-machine Full-Mono stack.  Starting the
# stack never grants motion authority: the Go2 bridge is opt-in and the ROS
# adapter remains disabled until an operator explicitly enables it.

OFFBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$OFFBOARD_DIR/.." && pwd)"

GPU_HOST="${CEC_HUB_SSH_HOST:-work-pc}"
GPU_REPO="${CEC_GPU_REPO:-/home/asus/Research/Memnav_Realworld}"
GPU_SESSION="${CEC_TMUX_SESSION:-cec-realworld}"
LOCAL_SESSION="${NAVDP_TMUX_SESSION:-navdp-go2-offboard}"
LOCAL_PORT="${CEC_LOCAL_PORT:-18889}"
REMOTE_PORT="${CEC_REMOTE_PORT:-18889}"
SSH_OPTIONS=(
  -o BatchMode=yes
  -o ConnectTimeout=8
  -o ServerAliveInterval=5
  -o ServerAliveCountMax=2
)

usage() {
  cat <<'EOF'
Usage:
  fullmono.sh start [--with-rviz] [--with-go2]
  fullmono.sh status
  fullmono.sh stop

The command is run on the Jetson.  `start` first starts/reuses the RTX policy
stack through passwordless SSH, then starts the local tunnel, D435i and locked
ROS adapter.  `--with-go2` only starts the watchdog bridge; it does not enable
the adapter or authorize motion.

One-time RTX configuration:
  /home/asus/Research/Memnav_Realworld/deployment/gpu/.env

Optional overrides:
  CEC_HUB_SSH_HOST, CEC_GPU_REPO, CEC_TMUX_SESSION,
  NAVDP_TMUX_SESSION, CEC_LOCAL_PORT, CEC_REMOTE_PORT.
EOF
}

die() {
  echo "fullmono: $*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

shell_quote() {
  printf '%q' "$1"
}

remote_exec() {
  ssh "${SSH_OPTIONS[@]}" "$GPU_HOST" "$1"
}

remote_session_exists() {
  local quoted_session
  quoted_session="$(shell_quote "$GPU_SESSION")"
  remote_exec "tmux has-session -t ${quoted_session} 2>/dev/null"
}

remote_health() {
  remote_exec \
    "curl -fsS --max-time 3 http://127.0.0.1:${REMOTE_PORT}/healthz"
}

validate_health() {
  local payload="$1"
  python3 - "$payload" <<'PY'
import json
import math
import sys

p = json.loads(sys.argv[1])
assert p.get("algo") == "cec_hybrid_navdp"
assert p.get("navigation_sensor_contract") == "causal_monocular_rgb_v1"
assert p.get("navdp_depth_source") == "monocular_sidecar"
assert p.get("metric_depth_sensor_consumed_by_policy") is False
height = float(p["camera_height_m"])
assert math.isfinite(height) and 0.1 <= height <= 2.0
print(f"health=fullmono-v2 camera_height_m={height:.3f}")
PY
}

start_stack() {
  local with_rviz=false
  local with_go2=false
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --with-rviz) with_rviz=true ;;
      --with-go2) with_go2=true ;;
      -h|--help) usage; return 0 ;;
      *) die "unknown start option: $1" ;;
    esac
    shift
  done

  for cmd in ssh tmux curl python3; do
    require_command "$cmd"
  done

  local goal_path="${NAVDP_IMAGE_GOAL_PATH:-$GO2_DIR/goals/image_goal.png}"
  [[ -f "$goal_path" ]] || die "ImageGoal is missing: $goal_path"
  [[ -x "$OFFBOARD_DIR/run_offboard_stack.sh" ]] \
    || die "missing local offboard launcher"
  [[ -x "$OFFBOARD_DIR/preflight_offboard.sh" ]] \
    || die "missing local offboard preflight"
  if tmux has-session -t "$LOCAL_SESSION" 2>/dev/null; then
    die "Jetson session already exists: $LOCAL_SESSION (use: $0 status)"
  fi

  ssh "${SSH_OPTIONS[@]}" "$GPU_HOST" true \
    || die "passwordless SSH to RTX host failed: $GPU_HOST"

  local quoted_repo quoted_gpu_session
  quoted_repo="$(shell_quote "$GPU_REPO")"
  quoted_gpu_session="$(shell_quote "$GPU_SESSION")"
  remote_exec \
    "test -x ${quoted_repo}/deployment/gpu/scripts/preflight.sh && test -x ${quoted_repo}/deployment/gpu/scripts/run_policy_stack.sh" \
    || die "RTX Full-Mono release is missing under $GPU_REPO"

  local started_gpu=false
  local start_complete=false
  rollback_partial_start() {
    local status=$?
    if [[ "$start_complete" != true ]]; then
      bash "$OFFBOARD_DIR/stop_offboard_stack.sh" >/dev/null 2>&1 || true
      if [[ "$started_gpu" == true ]]; then
        remote_exec \
          "cd ${quoted_repo} && CEC_TMUX_SESSION=${quoted_gpu_session} bash deployment/gpu/scripts/stop_policy_stack.sh" \
          >/dev/null 2>&1 || true
      fi
    fi
    return "$status"
  }
  trap rollback_partial_start EXIT

  local health
  if remote_session_exists; then
    health="$(remote_health)" \
      || die "RTX tmux session exists but its hub is unhealthy; run '$0 stop'"
    validate_health "$health" >/dev/null \
      || die "existing RTX session advertises the wrong policy contract"
    echo "Reusing healthy RTX policy session: $GPU_SESSION"
  else
    echo "Starting RTX policy stack through $GPU_HOST ..."
    remote_exec \
      "cd ${quoted_repo} && CEC_TMUX_SESSION=${quoted_gpu_session} bash deployment/gpu/scripts/preflight.sh && CEC_TMUX_SESSION=${quoted_gpu_session} bash deployment/gpu/scripts/run_policy_stack.sh"
    started_gpu=true
    health="$(remote_health)" \
      || die "RTX policy stack started but health endpoint is unavailable"
    validate_health "$health"
  fi

  local local_args=()
  if [[ "$with_rviz" == true ]]; then
    local_args+=(--with-rviz)
  fi
  if [[ "$with_go2" == true ]]; then
    local_args+=(--with-go2)
  fi
  CEC_HUB_SSH_HOST="$GPU_HOST" \
    NAVDP_TMUX_SESSION="$LOCAL_SESSION" \
    CEC_LOCAL_PORT="$LOCAL_PORT" \
    CEC_REMOTE_PORT="$REMOTE_PORT" \
    NAVDP_IMAGE_GOAL_PATH="$goal_path" \
    bash "$OFFBOARD_DIR/run_offboard_stack.sh" "${local_args[@]}"

  CEC_HUB_SSH_HOST="$GPU_HOST" CEC_LOCAL_PORT="$LOCAL_PORT" \
    bash "$OFFBOARD_DIR/preflight_offboard.sh"

  start_complete=true
  trap - EXIT
  echo
  echo "Full-Mono stack is ready from the Jetson entry point."
  echo "  RTX session:    $GPU_HOST:$GPU_SESSION"
  echo "  Jetson session: $LOCAL_SESSION"
  echo "  Go2 bridge:     $with_go2"
  echo "  Motion:         LOCKED (adapter starts disabled)"
  echo "  Inspect:        tmux attach -t $LOCAL_SESSION"
  if [[ "$with_go2" == true ]]; then
    echo "  Enabling motion still requires the explicit ROS SetBool call."
  fi
}

status_stack() {
  for cmd in ssh tmux curl python3; do
    require_command "$cmd"
  done

  local failures=0
  if tmux has-session -t "$LOCAL_SESSION" 2>/dev/null; then
    echo "Jetson session: RUNNING ($LOCAL_SESSION)"
    tmux list-windows -t "$LOCAL_SESSION" -F '  window=#{window_name} active=#{window_active}'
  else
    echo "Jetson session: STOPPED ($LOCAL_SESSION)"
    failures=$((failures + 1))
  fi

  local local_health
  if local_health="$(curl -fsS --max-time 3 "http://127.0.0.1:${LOCAL_PORT}/healthz" 2>/dev/null)"; then
    echo -n "Jetson tunnel:  "
    validate_health "$local_health" || failures=$((failures + 1))
  else
    echo "Jetson tunnel:  UNAVAILABLE"
    failures=$((failures + 1))
  fi

  if ssh "${SSH_OPTIONS[@]}" "$GPU_HOST" true 2>/dev/null; then
    if remote_session_exists; then
      echo "RTX session:    RUNNING ($GPU_HOST:$GPU_SESSION)"
      local gpu_health
      if gpu_health="$(remote_health 2>/dev/null)"; then
        echo -n "RTX hub:        "
        validate_health "$gpu_health" || failures=$((failures + 1))
      else
        echo "RTX hub:        UNHEALTHY"
        failures=$((failures + 1))
      fi
    else
      echo "RTX session:    STOPPED ($GPU_HOST:$GPU_SESSION)"
      failures=$((failures + 1))
    fi
  else
    echo "RTX SSH:        UNREACHABLE ($GPU_HOST)"
    failures=$((failures + 1))
  fi

  echo "Motion state must be confirmed from /navdp/status; startup never enables it."
  return "$failures"
}

stop_stack() {
  require_command ssh
  bash "$OFFBOARD_DIR/stop_offboard_stack.sh"

  local quoted_repo quoted_gpu_session
  quoted_repo="$(shell_quote "$GPU_REPO")"
  quoted_gpu_session="$(shell_quote "$GPU_SESSION")"
  if ssh "${SSH_OPTIONS[@]}" "$GPU_HOST" true 2>/dev/null; then
    remote_exec \
      "cd ${quoted_repo} && CEC_TMUX_SESSION=${quoted_gpu_session} bash deployment/gpu/scripts/stop_policy_stack.sh"
  else
    echo "Warning: RTX host is unreachable; its policy-only session was not stopped." >&2
    return 1
  fi
  echo "Full-Mono Jetson and RTX sessions are stopped."
}

action="${1:-}"
if [[ $# -gt 0 ]]; then
  shift
fi
case "$action" in
  start) start_stack "$@" ;;
  status) [[ $# -eq 0 ]] || die "status takes no options"; status_stack ;;
  stop) [[ $# -eq 0 ]] || die "stop takes no options"; stop_stack ;;
  -h|--help|help) usage ;;
  "") usage; exit 2 ;;
  *) die "unknown action: $action" ;;
esac
