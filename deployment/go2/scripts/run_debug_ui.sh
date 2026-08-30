#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros

RVIZ_CONFIG="$CFG_RVIZ_CONFIG"
if [[ ! -f "$RVIZ_CONFIG" ]]; then
  echo "RViz config not found: $RVIZ_CONFIG" >&2
  exit 1
fi

if [[ -z "${XAUTHORITY:-}" ]]; then
  if [[ -r "$HOME/.Xauthority" ]]; then
    export XAUTHORITY="$HOME/.Xauthority"
  elif [[ -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  fi
fi
if [[ -z "${DISPLAY:-}" ]]; then
  best_display=""
  best_area=0
  for socket in /tmp/.X11-unix/X*; do
    [[ -S "$socket" ]] || continue
    candidate=":${socket##*X}"
    dimensions="$(
      DISPLAY="$candidate" xdpyinfo 2>/dev/null |
        awk '/dimensions:/ {value=$2} END {print value}'
    )"
    width="${dimensions%x*}"
    height="${dimensions#*x}"
    if [[ "$width" =~ ^[0-9]+$ && "$height" =~ ^[0-9]+$ ]]; then
      area=$((width * height))
      if (( area > best_area )); then
        best_area=$area
        best_display="$candidate"
      fi
    fi
  done
  export DISPLAY="$best_display"
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo "No graphical DISPLAY found. Run RViz on a desktop or use SSH X forwarding." >&2
  exit 1
fi

echo "Starting NavDP RViz debug UI"
echo "  display: $DISPLAY"
echo "  config:  $RVIZ_CONFIG"

TF_LOG="$CFG_JETSON_RUNTIME_ROOT/logs/debug_tf.log"
mkdir -p "$(dirname "$TF_LOG")"
ros2 run tf2_ros static_transform_publisher \
  --frame-id navdp_local --child-frame-id base_link >"$TF_LOG" 2>&1 &
TF_PID=$!
RVIZ_PID=""
cleanup() {
  if [[ -n "$RVIZ_PID" ]]; then
    kill -TERM "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  kill -TERM "$TF_PID" 2>/dev/null || true
  wait "$TF_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!
if command -v wmctrl >/dev/null 2>&1; then
  rviz_window=""
  for _ in $(seq 1 50); do
    rviz_window="$(
      wmctrl -lp 2>/dev/null |
        awk -v pid="$RVIZ_PID" '$3 == pid {print $1; exit}'
    )"
    [[ -n "$rviz_window" ]] && break
    sleep 0.1
  done
  if [[ -n "$rviz_window" ]]; then
    wmctrl -i -r "$rviz_window" -b remove,hidden || true
    wmctrl -i -r "$rviz_window" -b add,maximized_vert,maximized_horz || true
    wmctrl -i -a "$rviz_window" || true
  fi
fi
wait "$RVIZ_PID"
