#!/usr/bin/env bash
set -uo pipefail

GO2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$(dirname "${BASH_SOURCE[0]}")/runtime_contract.sh"
LOCAL_PORT="${CEC_LOCAL_PORT:-18889}"
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
ssh -o BatchMode=yes -o ConnectTimeout=5 "${CEC_HUB_SSH_HOST:-work-pc}" true \
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
