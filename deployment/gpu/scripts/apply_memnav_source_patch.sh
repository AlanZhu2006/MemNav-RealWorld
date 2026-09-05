#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
gpu_require_config "$@"

PATCH_FILE="$REPO_ROOT/deployment/gpu/patches/memnav_reuse_flow_depth.patch"
require_file "$PATCH_FILE"
require_dir "$CFG_MEMNAV_SOURCE_ROOT"

if git -C "$CFG_MEMNAV_SOURCE_ROOT" apply --reverse --check "$PATCH_FILE"; then
  echo "MemNav current-frame depth reuse patch is already applied."
  exit 0
fi
if ! git -C "$CFG_MEMNAV_SOURCE_ROOT" apply --check "$PATCH_FILE"; then
  echo "MemNav source does not match either side of the tracked latency patch." >&2
  echo "Refusing to modify the external worktree." >&2
  exit 1
fi

git -C "$CFG_MEMNAV_SOURCE_ROOT" apply "$PATCH_FILE"
echo "Applied MemNav current-frame depth reuse patch."
