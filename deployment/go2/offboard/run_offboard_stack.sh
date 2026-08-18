#!/usr/bin/env bash
set -euo pipefail

OFFBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$OFFBOARD_DIR/.." && pwd)"
SESSION="${NAVDP_TMUX_SESSION:-navdp-go2-offboard}"
LOCAL_PORT="${CEC_LOCAL_PORT:-18889}"
with_go2=false
with_camera=true
with_rviz=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-go2) with_go2=true; shift ;;
    --with-rviz) with_rviz=true; shift ;;
    --no-camera) with_camera=false; shift ;;
    *) echo "Usage: $0 [--with-go2] [--with-rviz] [--no-camera]" >&2; exit 2 ;;
  esac
done

goal_path="${NAVDP_IMAGE_GOAL_PATH:-$GO2_DIR/goals/image_goal.png}"
[[ -f "$goal_path" ]] || { echo "Image goal missing: $goal_path" >&2; exit 1; }
command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  exit 1
fi

tmux new-session -d -s "$SESSION" -n tunnel "exec '$OFFBOARD_DIR/run_policy_tunnel.sh'"
healthy=false
for _ in $(seq 1 20); do
  if curl -fsS --max-time 1 "http://127.0.0.1:${LOCAL_PORT}/healthz" \
      | grep -q 'cec_hybrid_navdp'; then
    healthy=true
    break
  fi
  sleep 0.25
done
if [[ "$healthy" != true ]]; then
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  echo "CEC hub did not become healthy through the SSH tunnel" >&2
  exit 1
fi

if [[ "$with_camera" == true ]]; then
  tmux new-window -t "$SESSION" -n rgbd "exec '$GO2_DIR/scripts/run_realsense.sh'"
fi
tmux new-window -t "$SESSION" -n adapter \
  "export NAVDP_BACKEND='navdp' NAVDP_MODE='imagegoal' NAVDP_SERVER_URL='http://127.0.0.1:${LOCAL_PORT}' NAVDP_IMAGE_GOAL_PATH='$goal_path'; exec '$GO2_DIR/scripts/run_adapter.sh'"
if [[ "$with_go2" == true ]]; then
  tmux new-window -t "$SESSION" -n go2 "exec '$GO2_DIR/scripts/run_go2_bridge.sh'"
fi
if [[ "$with_rviz" == true ]]; then
  tmux new-window -t "$SESSION" -n rviz "exec '$GO2_DIR/scripts/run_debug_ui.sh'"
fi

echo "Offboard CEC/NavDP stack started in tmux session $SESSION"
echo "  hub=http://127.0.0.1:${LOCAL_PORT} camera=$with_camera go2_bridge=$with_go2"
echo "  inspect: tmux attach -t $SESSION"
echo "Motion remains locked until an operator explicitly calls set_enabled=true."
