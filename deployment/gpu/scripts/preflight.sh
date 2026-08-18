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
[[ -f "${LINGBOT_WEIGHTS:-}" ]] && pass "LingBot weights" || fail "LINGBOT_WEIGHTS"
[[ -d "${LIGHTGLUE_REPO:-}" ]] && pass "LightGlue source" || fail "LIGHTGLUE_REPO"
for port in "$MEMNAV_PORT" "$NAVDP_PORT" "$CEC_HUB_PORT"; do
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)$port$"; then
    fail "port already in use: $port"
  else
    pass "port available: $port"
  fi
done

printf '\nGPU preflight complete: failures=%d\n' "$failures"
exit "$failures"
