#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
NAVDP_CKPT="${NAVDP_CKPT:-$REPO_ROOT/baselines/navdp/checkpoints/navdp_pretrain.ckpt}"
NAVDP_DEVICE="${NAVDP_DEVICE:-cuda:0}"
require_executable "$MEMNAV_PY"
require_file "$NAVDP_CKPT"

cd "$REPO_ROOT"
exec env PYTHONUNBUFFERED=1 \
  "$MEMNAV_PY" -u "$REPO_ROOT/deployment/go2/navdp_base_server.py" \
    --host 127.0.0.1 --port "$NAVDP_PORT" \
    --checkpoint "$NAVDP_CKPT" --device "$NAVDP_DEVICE"
