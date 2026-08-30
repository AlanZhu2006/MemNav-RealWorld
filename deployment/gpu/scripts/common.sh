#!/usr/bin/env bash
set -euo pipefail

GPU_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "$GPU_DIR/../.." && pwd)"
RUNTIME_CONFIG_TOOL="$REPO_ROOT/deployment/runtime_config.py"

gpu_require_config() {
  if [[ $# -ne 2 || "$1" != --config || -z "$2" ]]; then
    echo "Usage: ${0##*/} --config RESOLVED_CONFIG.json" >&2
    exit 2
  fi
  RUN_CONFIG="$(readlink -f "$2")"
  python3 "$RUNTIME_CONFIG_TOOL" verify \
    --config "$RUN_CONFIG" --site gpu >/dev/null
  gpu_read_config "$RUN_CONFIG"
}

gpu_read_config() {
  local config="$1"
  # CFG_* is generated atomically from the immutable resolved JSON; .env and
  # caller-provided model/path overrides are deliberately unsupported.
  local config_exports
  config_exports="$(python3 "$RUNTIME_CONFIG_TOOL" shell \
    --config "$RUN_CONFIG" --site gpu)"
  eval "$config_exports"
  MEMNAV_PY="$CFG_GPU_PYTHON"
  MEMNAV_PORT="$CFG_MEMNAV_PORT"
  NAVDP_PORT="$CFG_NAVDP_PORT"
  CEC_HUB_PORT="$CFG_HUB_PORT"
  CEC_OUT_ROOT="$CFG_GPU_RUNTIME_ROOT"
}

require_file() {
  [[ -f "$1" ]] || { echo "Missing file: $1" >&2; exit 1; }
}

require_dir() {
  [[ -d "$1" ]] || { echo "Missing directory: $1" >&2; exit 1; }
}

require_executable() {
  command -v "$1" >/dev/null 2>&1 || [[ -x "$1" ]] || {
    echo "Missing executable: $1" >&2
    exit 1
  }
}
