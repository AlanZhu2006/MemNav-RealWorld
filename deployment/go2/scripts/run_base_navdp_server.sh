#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros
navdp_activate_venv

CHECKPOINT="${NAVDP_CHECKPOINT:-$NAVDP_ROOT/baselines/navdp/checkpoints/navdp_pretrain.ckpt}"
EXPECTED_SHA256="3bb3ad4ab241e857bb57a4021cc6aab76d5263e81fbf80298d579053ef011947"
HOST="${NAVDP_SERVER_HOST:-127.0.0.1}"
PORT="${NAVDP_SERVER_PORT:-8888}"
DEVICE="${NAVDP_DEVICE:-cuda:0}"

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "NavDP checkpoint missing. Run: $SCRIPT_DIR/download_weights.sh base" >&2
  exit 1
fi
if [[ "$CHECKPOINT" == "$NAVDP_ROOT/baselines/navdp/checkpoints/navdp_pretrain.ckpt" ]]; then
  echo "$EXPECTED_SHA256  $CHECKPOINT" | sha256sum --check --status || {
    echo "NavDP checkpoint checksum failed." >&2
    exit 1
  }
fi

cd "$NAVDP_ROOT"
echo "Starting original NavDP server on $HOST:$PORT"
exec python "$NAVDP_GO2_DIR/navdp_base_server.py" \
  --host "$HOST" \
  --port "$PORT" \
  --checkpoint "$CHECKPOINT" \
  --device "$DEVICE"
