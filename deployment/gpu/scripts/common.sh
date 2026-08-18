#!/usr/bin/env bash
set -euo pipefail

GPU_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "$GPU_DIR/../.." && pwd)"
ENV_FILE="${CEC_ENV_FILE:-$GPU_DIR/.env}"

if [[ -f "$ENV_FILE" ]]; then
  set -a
  source "$ENV_FILE"
  set +a
fi

MEMNAV_PY="${MEMNAV_PY:-python3}"
MEMNAV_PORT="${MEMNAV_PORT:-18888}"
NAVDP_PORT="${NAVDP_PORT:-8888}"
CEC_HUB_PORT="${CEC_HUB_PORT:-18889}"
CEC_OUT_ROOT="${CEC_OUT_ROOT:-$REPO_ROOT/runtime/gpu}"

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
