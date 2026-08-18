#!/usr/bin/env bash
set -euo pipefail

HUB_SSH_HOST="${CEC_HUB_SSH_HOST:-work-pc}"
LOCAL_PORT="${CEC_LOCAL_PORT:-18889}"
REMOTE_PORT="${CEC_REMOTE_PORT:-18889}"

exec ssh -NT \
  -o BatchMode=yes \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=5 \
  -o ServerAliveCountMax=2 \
  -L "127.0.0.1:${LOCAL_PORT}:127.0.0.1:${REMOTE_PORT}" \
  "$HUB_SSH_HOST"
