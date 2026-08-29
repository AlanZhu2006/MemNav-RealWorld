#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${NAVDP_TMUX_SESSION:-navdp-go2}"
backend="x"
mode="startgoal"
with_go2=false
with_camera=true
with_rviz=false
arrival_module="${NAVDP_ARRIVAL_MODULE:-operator}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend) backend="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --with-go2) with_go2=true; shift ;;
    --with-rviz) with_rviz=true; shift ;;
    --no-camera) with_camera=false; shift ;;
    --arrival) arrival_module="$2"; shift 2 ;;
    *) echo "Usage: $0 [--backend x|base] [--mode startgoal|pointgoal|imagegoal|nogoal] [--arrival operator|external-topic|rgb-homography] [--with-go2] [--with-rviz] [--no-camera]" >&2; exit 2 ;;
  esac
done

case "$mode" in
  startgoal|pointgoal|imagegoal|nogoal) ;;
  *) echo "Unknown mode: $mode" >&2; exit 2 ;;
esac

case "$backend" in
  x|x_navdp) backend="x_navdp"; server_script="$SCRIPT_DIR/run_x_navdp_server.sh" ;;
  base|navdp) backend="navdp"; server_script="$SCRIPT_DIR/run_base_navdp_server.sh" ;;
  *) echo "Unknown backend: $backend" >&2; exit 2 ;;
esac
if [[ "$backend" == "x_navdp" && ( "$mode" == "nogoal" || "$mode" == "imagegoal" ) ]]; then
  echo "X-NavDP exposes PointGoal only; use --backend base for $mode." >&2
  exit 2
fi
image_goal_path="${NAVDP_IMAGE_GOAL_PATH:-$SCRIPT_DIR/../goals/image_goal.png}"
arrival_goal_path="${NAVDP_ARRIVAL_GOAL_PATH:-$image_goal_path}"
arrival_allowed_phases="${NAVDP_ARRIVAL_ALLOWED_PHASES:-revisit_query}"
if [[ "$mode" == "imagegoal" && ! -f "$image_goal_path" ]]; then
  echo "Image goal missing: $image_goal_path" >&2
  echo "Capture it first with: $SCRIPT_DIR/capture_image_goal.sh" >&2
  exit 1
fi
if [[ "$arrival_module" != operator && "$mode" != imagegoal ]]; then
  echo "Arrival modules are currently supported only in imagegoal mode." >&2
  exit 2
fi
read -r _profile_name arrival_module < <(
  python3 "$SCRIPT_DIR/../stack_profiles.py" validate \
    native-navdp-rgbd "$arrival_module"
)
if [[ "$arrival_module" == rgb-homography && ! -f "$arrival_goal_path" ]]; then
  echo "Arrival reference missing: $arrival_goal_path" >&2
  exit 1
fi
if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for run_stack.sh; individual run_*.sh scripts work without it." >&2
  exit 1
fi
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  exit 1
fi

tmux new-session -d -s "$SESSION" -n policy "exec '$server_script'"
if [[ "$with_camera" == true ]]; then
  tmux new-window -t "$SESSION" -n rgbd "exec '$SCRIPT_DIR/run_realsense.sh'"
fi
tmux new-window -t "$SESSION" -n adapter \
  "export NAVDP_BACKEND='$backend' NAVDP_MODE='$mode' NAVDP_IMAGE_GOAL_PATH='$image_goal_path'; exec '$SCRIPT_DIR/run_adapter.sh'"
if [[ "$arrival_module" == rgb-homography ]]; then
  tmux new-window -t "$SESSION" -n arrival \
    "export NAVDP_ARRIVAL_MODULE='$arrival_module' NAVDP_ARRIVAL_GOAL_PATH='$arrival_goal_path' NAVDP_ARRIVAL_ALLOWED_PHASES='$arrival_allowed_phases'; exec '$SCRIPT_DIR/run_arrival_module.sh'"
fi
if [[ "$with_go2" == true ]]; then
  tmux new-window -t "$SESSION" -n go2 "exec '$SCRIPT_DIR/run_go2_bridge.sh'"
fi
if [[ "$with_rviz" == true ]]; then
  tmux new-window -t "$SESSION" -n rviz "exec '$SCRIPT_DIR/run_debug_ui.sh'"
fi
tmux set-environment -t "$SESSION" NAVDP_STACK_PROFILE native-navdp-rgbd
tmux set-environment -t "$SESSION" NAVDP_ARRIVAL_MODULE "$arrival_module"
tmux set-environment -t "$SESSION" NAVDP_NAVIGATION_GOAL_PATH "$image_goal_path"
tmux set-environment -t "$SESSION" NAVDP_ARRIVAL_GOAL_PATH "$arrival_goal_path"
tmux set-environment -t "$SESSION" NAVDP_ARRIVAL_ALLOWED_PHASES "$arrival_allowed_phases"

echo "NavDP stack started in tmux session $SESSION"
echo "  backend=$backend mode=$mode camera=$with_camera go2_bridge=$with_go2 rviz=$with_rviz"
echo "  arrival=$arrival_module arrival_goal=${arrival_goal_path:-n/a}"
echo "  inspect: tmux attach -t $SESSION"
echo "Motion remains disabled until:"
echo "  ros2 service call /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool '{data: true}'"
