#!/usr/bin/env bash
set -uo pipefail

GO2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$GO2_DIR/scripts/common.sh"
source "$(dirname "${BASH_SOURCE[0]}")/runtime_contract.sh"
navdp_require_config_arg "$@"
navdp_load_config "$NAVDP_RUN_CONFIG"
LOCAL_PORT="$CFG_TUNNEL_LOCAL_PORT"
failures=0

pass() { printf '[PASS] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*"; failures=$((failures + 1)); }

[[ -x "$GO2_DIR/scripts/run_adapter.sh" ]] \
  && pass "existing NavDP Go2 adapter" \
  || fail "missing existing adapter"
[[ -x "$GO2_DIR/scripts/run_realsense.sh" ]] \
  && pass "existing D435i launcher" \
  || fail "missing D435i launcher"
[[ -x "$GO2_DIR/scripts/run_go2_bridge.sh" ]] \
  && pass "existing Go2 watchdog bridge" \
  || fail "missing Go2 bridge"
[[ -x "$GO2_DIR/scripts/run_go2_battery_monitor.sh" ]] \
  && pass "observation-only Go2 battery monitor" \
  || fail "missing Go2 battery monitor"
ssh -o BatchMode=yes -o ConnectTimeout=5 "$CFG_GPU_HOST" true \
  && pass "passwordless SSH to 4090 hub" \
  || fail "cannot SSH to 4090 hub"
health="$(curl -fsS --max-time 3 "http://127.0.0.1:${LOCAL_PORT}/healthz" 2>/dev/null || true)"
if cec_validate_health_contract "$health" "$GO2_DIR" 2>/dev/null; then
  pass "monocular CEC hub reachable through loopback tunnel"
else
  fail "hub on port ${LOCAL_PORT} does not advertise the frozen mono contract"
fi

printf '\nOffboard preflight complete: failures=%d\n' "$failures"
exit "$failures"
