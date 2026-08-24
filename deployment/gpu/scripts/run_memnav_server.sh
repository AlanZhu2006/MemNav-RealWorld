#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

: "${MEMNAV_SOURCE_ROOT:?Set MEMNAV_SOURCE_ROOT in deployment/gpu/.env}"
: "${MEMNAV_CKPT:?Set MEMNAV_CKPT in deployment/gpu/.env}"
: "${INTERNNAV_ROOT:?Set INTERNNAV_ROOT in deployment/gpu/.env}"
: "${LINGBOT_REPO:?Set LINGBOT_REPO in deployment/gpu/.env}"
: "${LINGBOT_WEIGHTS:?Set LINGBOT_WEIGHTS in deployment/gpu/.env}"
: "${LIGHTGLUE_REPO:?Set LIGHTGLUE_REPO in deployment/gpu/.env}"
: "${DEPENDENCY_ROOT:?Set DEPENDENCY_ROOT in deployment/gpu/.env}"

MEMNAV_SERVER="${MEMNAV_SERVER:-$MEMNAV_SOURCE_ROOT/NavDP/baselines/memnav/memnav_server.py}"
if [[ -n "${CEC_BUFFER_ROOT:-}" ]]; then
  BUFFER_ROOT="$CEC_BUFFER_ROOT"
else
  # The in-process episode counter restarts at zero with each service process.
  # A per-process namespace prevents a restart from erasing the preceding
  # real-world ep_0000/ep_0001 trace.
  RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)_$$"
  BUFFER_ROOT="$CEC_OUT_ROOT/buffer/run_$RUN_STAMP"
fi
require_executable "$MEMNAV_PY"
require_file "$MEMNAV_SERVER"
require_file "$MEMNAV_CKPT"
require_file "$LINGBOT_WEIGHTS"
require_dir "$INTERNNAV_ROOT"
require_dir "$LINGBOT_REPO"
require_dir "$LIGHTGLUE_REPO"
require_dir "$DEPENDENCY_ROOT"
mkdir -p "$BUFFER_ROOT"
echo "realworld_memnav_buffer_root=$BUFFER_ROOT"

extra_args=()
if [[ "${CEC_EAGER_DEPTH_CACHE:-0}" == "1" ]]; then
  extra_args+=(--certified_eager_depth_cache)
fi
server_pythonpath="$MEMNAV_SOURCE_ROOT:$DEPENDENCY_ROOT:$LIGHTGLUE_REPO:$INTERNNAV_ROOT/src/diffusion-policy${PYTHONPATH:+:$PYTHONPATH}"

cd "$CEC_OUT_ROOT"
exec env PYTHONUNBUFFERED=1 PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
  PYTHONPATH="$server_pythonpath" \
  LINGBOT_REPO="$LINGBOT_REPO" LINGBOT_WEIGHTS="$LINGBOT_WEIGHTS" \
  MEMNAV_WINDOW=32 MEMNAV_NUM_SCALE=8 MEMNAV_MAX_FRAME_NUM=2048 \
  MEMNAV_GROUND_SCALE_MAX=6.0 MEMNAV_GATE_FUSION=complementary \
  MEMNAV_AUX_POSE_CALIBRATION=empirical MEMNAV_COLLISION_SELECT=1 \
  MEMNAV_REPORT_TO=none \
  "$MEMNAV_PY" -u "$MEMNAV_SERVER" \
    --host 127.0.0.1 --port "$MEMNAV_PORT" --checkpoint "$MEMNAV_CKPT" \
    --internnav_root "$INTERNNAV_ROOT" --num_samples 16 \
    --exclude_recent 32 --retrieval raw \
    --retrieval_candidate_top_k 32 --retrieval_candidate_min_gap 16 \
    --graph_subgoal_spacing_m 0.0 --graph_subgoal_arrival_m 0.60 \
    --flow_gate auto --buffer_root "$BUFFER_ROOT" \
    --certified_relocalization \
    --lightglue_repo "$LIGHTGLUE_REPO" \
    --lightglue_dependency_root "$DEPENDENCY_ROOT" \
    --lightglue_max_keypoints 2048 \
    "${extra_args[@]}"
