#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"

if ! navdp_assert_motion_locked; then
  echo "Could not confirm disabled + estop on the running stack." >&2
  exit 1
fi
echo "Running stack is motion-locked (disabled + estop)."
