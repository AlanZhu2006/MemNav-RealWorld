#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
gpu_require_config "$@"
NAVDP_CKPT="$CFG_NAVDP_CKPT"
DEPENDENCY_ROOT="$CFG_DEPENDENCY_ROOT"
INTERNNAV_ROOT="$CFG_INTERNNAV_ROOT"
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
    --require_monocular_depth_transaction \
    --monocular_depth_url \
      "http://127.0.0.1:${MEMNAV_PORT}/monocular_depth_query"
