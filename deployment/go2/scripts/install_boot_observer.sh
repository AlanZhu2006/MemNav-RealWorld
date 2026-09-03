#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
UNIT_SOURCE_DIR="$GO2_DIR/systemd"
account_home="$(getent passwd "$(id -un)" | cut -d: -f6)"
user_config_root="${XDG_CONFIG_HOME:-$account_home/.config}"
unit_dir="$user_config_root/systemd/user"

[[ -d "$UNIT_SOURCE_DIR" ]] || {
  echo "Systemd unit source is missing: $UNIT_SOURCE_DIR" >&2
  exit 1
}
command -v systemctl >/dev/null || {
  echo "systemctl is required" >&2
  exit 1
}

mkdir -p "$unit_dir"
for unit in "$UNIT_SOURCE_DIR"/memnav-observer* \
    "$UNIT_SOURCE_DIR"/memnav-revisit-operator.service; do
  ln -sfn "$unit" "$unit_dir/$(basename "$unit")"
done

systemctl --user daemon-reload
systemctl --user enable --now memnav-revisit-operator.service
systemctl --user enable --now memnav-observer.target

if command -v loginctl >/dev/null 2>&1 \
    && [[ "$(loginctl show-user "$(id -un)" -p Linger --value)" != yes ]]; then
  sudo loginctl enable-linger "$(id -un)"
fi

echo "MemNav boot observer is enabled for $(id -un)."
echo "  target:    memnav-observer.target"
echo "  Episodes:  memnav-revisit-operator.service (persistent GUI workflow)"
echo "  Foxglove:  ws://<Jetson LAN or Tailscale address>:8765"
echo "  motion:    no policy, adapter, command bridge, or navigation until confirmed"
