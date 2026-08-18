#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros
navdp_activate_venv

CHECKPOINT="${NAVDP_CHECKPOINT:-$NAVDP_ROOT/baselines/x-navdp/checkpoints/x-navdp_posttrain.ckpt}"
EXPECTED_SHA256="267089a81bbbe7a913debda6603f3f1b66a79520370ce953b2d888d793b89f24"
HOST="${NAVDP_SERVER_HOST:-127.0.0.1}"
PORT="${NAVDP_SERVER_PORT:-8888}"
DEVICE="${NAVDP_DEVICE:-cuda:0}"

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "X-NavDP checkpoint missing. Run: $SCRIPT_DIR/download_weights.sh x" >&2
  exit 1
fi
if [[ "$CHECKPOINT" == "$NAVDP_ROOT/baselines/x-navdp/checkpoints/x-navdp_posttrain.ckpt" ]]; then
  echo "$EXPECTED_SHA256  $CHECKPOINT" | sha256sum --check --status || {
    echo "X-NavDP checkpoint checksum failed." >&2
    exit 1
  }
fi

cd "$NAVDP_ROOT/baselines/x-navdp"
echo "Starting X-NavDP quadruped server on $HOST:$PORT (real mode, no odometry guidance)"
exec python -m eval.src.policy_server \
  --host "$HOST" \
  --port "$PORT" \
  --checkpoint "$CHECKPOINT" \
  --device "$DEVICE" \
  --embodiment quadruped \
  --real \
  --no-visualization
