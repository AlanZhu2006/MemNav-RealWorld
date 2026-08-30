#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
navdp_source_ros
navdp_activate_venv

CHECKPOINT="$CFG_NATIVE_CHECKPOINT"
EXPECTED_SHA256="$CFG_NATIVE_CHECKPOINT_SHA256"
HOST="$CFG_NATIVE_HOST"
PORT="$CFG_NATIVE_PORT"
DEVICE="$CFG_NATIVE_DEVICE"

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "NavDP checkpoint missing. Run: $SCRIPT_DIR/download_weights.sh base" >&2
  exit 1
fi
if [[ -n "$EXPECTED_SHA256" ]]; then
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
