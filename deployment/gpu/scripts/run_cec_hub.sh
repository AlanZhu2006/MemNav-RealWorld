#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
require_executable "$MEMNAV_PY"
CEC_CAMERA_HEIGHT_M="${CEC_CAMERA_HEIGHT_M:?Set measured D435i optical-center height in metres}"
CEC_GOAL_CANDIDATE_DIR="${CEC_GOAL_CANDIDATE_DIR:-$CEC_OUT_ROOT/goal_candidates}"
CEC_GOAL_SCORE_STRIDE="${CEC_GOAL_SCORE_STRIDE:-8}"
CEC_GOAL_MIN_FRAME_GAP="${CEC_GOAL_MIN_FRAME_GAP:-16}"
CEC_GOAL_MIN_INLIERS="${CEC_GOAL_MIN_INLIERS:-16}"
CEC_GOAL_MAX_COS="${CEC_GOAL_MAX_COS:-0.90}"
mkdir -p "$CEC_GOAL_CANDIDATE_DIR"

cd "$REPO_ROOT"
exec env PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
  "$MEMNAV_PY" -u -m deployment.gpu.realworld_cec_hub \
    --host 127.0.0.1 --port "$CEC_HUB_PORT" \
    --memnav-url "http://127.0.0.1:$MEMNAV_PORT" \
    --navdp-url "http://127.0.0.1:$NAVDP_PORT" \
    --camera-height-m "$CEC_CAMERA_HEIGHT_M" \
    --goal-candidate-dir "$CEC_GOAL_CANDIDATE_DIR" \
    --goal-score-stride "$CEC_GOAL_SCORE_STRIDE" \
    --goal-min-frame-gap "$CEC_GOAL_MIN_FRAME_GAP" \
    --goal-min-inliers "$CEC_GOAL_MIN_INLIERS" \
    --goal-max-cos "$CEC_GOAL_MAX_COS"
