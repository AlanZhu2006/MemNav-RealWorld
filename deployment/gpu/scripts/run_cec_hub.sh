#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
require_executable "$MEMNAV_PY"
CEC_CAMERA_HEIGHT_M="${CEC_CAMERA_HEIGHT_M:?Set measured D435i optical-center height in metres}"
CEC_GOAL_CANDIDATE_DIR="${CEC_GOAL_CANDIDATE_DIR:-$CEC_OUT_ROOT/goal_candidates}"
mkdir -p "$CEC_GOAL_CANDIDATE_DIR"

cd "$REPO_ROOT"
exec env PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
  "$MEMNAV_PY" -u -m deployment.gpu.realworld_cec_hub \
    --host 127.0.0.1 --port "$CEC_HUB_PORT" \
    --memnav-url "http://127.0.0.1:$MEMNAV_PORT" \
    --navdp-url "http://127.0.0.1:$NAVDP_PORT" \
    --camera-height-m "$CEC_CAMERA_HEIGHT_M" \
    --goal-candidate-dir "$CEC_GOAL_CANDIDATE_DIR"
