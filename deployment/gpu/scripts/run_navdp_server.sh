#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
NAVDP_CKPT="${NAVDP_CKPT:-$REPO_ROOT/baselines/navdp/checkpoints/navdp_pretrain.ckpt}"
DEPENDENCY_ROOT="${DEPENDENCY_ROOT:?Set DEPENDENCY_ROOT in deployment/gpu/.env}"
INTERNNAV_ROOT="${INTERNNAV_ROOT:?Set INTERNNAV_ROOT in deployment/gpu/.env}"
require_executable "$MEMNAV_PY"
require_file "$NAVDP_CKPT"
require_dir "$DEPENDENCY_ROOT"
require_dir "$INTERNNAV_ROOT"

server_pythonpath="$REPO_ROOT:$REPO_ROOT/baselines/navdp:$DEPENDENCY_ROOT:$INTERNNAV_ROOT/src/diffusion-policy${PYTHONPATH:+:$PYTHONPATH}"
cd "$CEC_OUT_ROOT"
exec env NAVDP_DISABLE_VIDEO=1 PYTHONUNBUFFERED=1 \
  PYTHONPATH="$server_pythonpath" \
  "$MEMNAV_PY" -u "$REPO_ROOT/baselines/navdp/navdp_server.py" \
    --port "$NAVDP_PORT" --checkpoint "$NAVDP_CKPT" \
    --depth_source monocular_sidecar \
    --monocular_depth_url \
      "http://127.0.0.1:${MEMNAV_PORT}/monocular_depth_query"
