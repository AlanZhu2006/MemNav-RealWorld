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

gpu_wait_for_policy_ports_free() {
  # tmux teardown returns before a CUDA worker necessarily closes its socket.
  # Only wait after retiring our own session; never kill an unknown listener.
  python3 - "$MEMNAV_PORT" "$NAVDP_PORT" "$CEC_HUB_PORT" <<'PY'
import subprocess
import sys
import time

ports = set(sys.argv[1:])
started = time.monotonic()
deadline = started + 30.0
announced = False
while True:
    result = subprocess.run(["ss", "-H", "-ltn"], check=True,
                            capture_output=True, text=True)
    occupied = {fields[3].rsplit(":", 1)[-1]
                for line in result.stdout.splitlines()
                if len(fields := line.split()) >= 4} & ports
    if not occupied:
        print(f"GPU policy ports released after {time.monotonic() - started:.2f}s",
              flush=True)
        break
    if not announced:
        print("Waiting for retired GPU workers to release ports: "
              + ", ".join(sorted(occupied)), flush=True)
        announced = True
    if time.monotonic() >= deadline:
        raise SystemExit("Retired GPU ports still occupied after 30s: "
                         + ", ".join(sorted(occupied))
                         + "; no unrelated processes were terminated")
    time.sleep(0.1)
PY
}
