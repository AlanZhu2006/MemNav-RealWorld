#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
failures=0
pass() { printf '[PASS] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*"; failures=$((failures + 1)); }

[[ -x "$MEMNAV_PY" ]] || command -v "$MEMNAV_PY" >/dev/null 2>&1 \
  && pass "GPU Python" || fail "GPU Python: $MEMNAV_PY"
[[ -f "${MEMNAV_CKPT:-}" ]] && pass "MemNav checkpoint" || fail "MEMNAV_CKPT"
[[ -f "${NAVDP_CKPT:-}" ]] && pass "NavDP checkpoint" || fail "NAVDP_CKPT"
[[ -f "${MEMNAV_SERVER:-${MEMNAV_SOURCE_ROOT:-}/NavDP/baselines/memnav/memnav_server.py}" ]] \
  && pass "MemNav server" || fail "MEMNAV_SERVER"
[[ -f "${MEMNAV_SOURCE_ROOT:-}/MemNavData/monocular_depth_runtime.py" ]] \
  && pass "monocular depth runtime" || fail "MEMNAV_SOURCE_ROOT/monocular_depth_runtime.py"
grep -q 'monocular_depth_query' \
  "${MEMNAV_SERVER:-${MEMNAV_SOURCE_ROOT:-}/NavDP/baselines/memnav/memnav_server.py}" \
  2>/dev/null \
  && pass "MemNav protocol-v2 depth endpoint" \
  || fail "MemNav server lacks /monocular_depth_query"
grep -q 'goal_candidate_support' \
  "${MEMNAV_SERVER:-${MEMNAV_SOURCE_ROOT:-}/NavDP/baselines/memnav/memnav_server.py}" \
  2>/dev/null \
  && pass "MemNav read-only goal support endpoint" \
  || fail "MemNav server lacks /goal_candidate_support"
[[ -f "${LINGBOT_WEIGHTS:-}" ]] && pass "LingBot weights" || fail "LINGBOT_WEIGHTS"
[[ -d "${LIGHTGLUE_REPO:-}" ]] && pass "LightGlue source" || fail "LIGHTGLUE_REPO"
[[ -f "$REPO_ROOT/baselines/navdp/navdp_server.py" ]] \
  && pass "Full-Mono NavDP server" || fail "Full-Mono NavDP server"
if [[ -z "${CEC_CAMERA_HEIGHT_M:-}" ]]; then
  fail "CEC_CAMERA_HEIGHT_M is unset; measure the installed optical-center height"
elif python3 - "$CEC_CAMERA_HEIGHT_M" <<'PY'
import math
import sys

value = float(sys.argv[1])
assert math.isfinite(value) and 0.1 <= value <= 2.0
PY
then
  pass "measured camera height: ${CEC_CAMERA_HEIGHT_M} m"
else
  fail "CEC_CAMERA_HEIGHT_M must be finite and in [0.1, 2.0] m"
fi
case "${CEC_AUTHORITY_MODE:-cec}" in
  cec|native) pass "authority mode: ${CEC_AUTHORITY_MODE:-cec}" ;;
  *) fail "CEC_AUTHORITY_MODE must be cec or native" ;;
esac
for port in "$MEMNAV_PORT" "$NAVDP_PORT" "$CEC_HUB_PORT"; do
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)$port$"; then
    fail "port already in use: $port"
  else
    pass "port available: $port"
  fi
done

printf '\nGPU preflight complete: failures=%d\n' "$failures"
exit "$failures"
