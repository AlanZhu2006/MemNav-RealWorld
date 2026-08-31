#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
deep=false
config_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --deep) deep=true; shift ;;
    --config)
      [[ $# -ge 2 ]] || { echo "--config requires a value" >&2; exit 2; }
      config_args+=("$1" "$2")
      shift 2
      ;;
    *) echo "Unknown preflight option: $1" >&2; exit 2 ;;
  esac
done
navdp_require_config_arg "${config_args[@]}"
navdp_load_config "$NAVDP_RUN_CONFIG"

failures=0
warnings=0
pass() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; warnings=$((warnings + 1)); }
fail() { printf '[FAIL] %s\n' "$*"; failures=$((failures + 1)); }

[[ "$(uname -m)" == "aarch64" ]] && pass "aarch64 Jetson userspace" || fail "Expected aarch64"
[[ -r /etc/nv_tegra_release ]] && pass "L4T $(head -n1 /etc/nv_tegra_release)" || warn "L4T release file missing"
[[ -f "$NAVDP_ROS_SETUP" ]] && pass "ROS 2 Humble setup" || fail "ROS setup missing"
[[ -f "$NAVDP_REALSENSE_SETUP" ]] && pass "RealSense ROS workspace" || fail "RealSense workspace missing"
if [[ "$CFG_WITH_FOXGLOVE" == true ]]; then
  if command -v ros2 >/dev/null 2>&1 && navdp_source_ros >/dev/null 2>&1 \
      && ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    pass "Foxglove Bridge package"
  else
    fail "Foxglove Bridge package missing (install ros-humble-foxglove-bridge)"
  fi
  [[ -f "$CFG_FOXGLOVE_LAYOUT" ]] \
    && pass "Foxglove layout" || fail "Foxglove layout missing: $CFG_FOXGLOVE_LAYOUT"
  [[ -f "$NAVDP_GO2_DIR/foxglove_image_relay.py" ]] \
    && pass "Foxglove image preview relay" \
    || fail "Foxglove image preview relay missing"
fi

if command -v rs-enumerate-devices >/dev/null 2>&1; then
  pass "RealSense tools"
else
  fail "rs-enumerate-devices is unavailable"
fi

if [[ -x "$NAVDP_VENV/bin/python" ]]; then
  if navdp_source_ros >/dev/null 2>&1 && navdp_activate_venv >/dev/null 2>&1 && \
      python -c 'import importlib.util as i; assert all(i.find_spec(x) for x in ("torch", "diffusers", "flask", "cv2", "message_filters", "rclpy"))' >/dev/null 2>&1; then
    pass "NavDP Python modules available"
  else
    fail "NavDP Python modules missing"
  fi
else
  fail "Run setup_jetson.sh"
fi

checkpoint="$CFG_NATIVE_CHECKPOINT"
expected="$CFG_NATIVE_CHECKPOINT_SHA256"
if [[ -f "$checkpoint" ]]; then
  pass "Checkpoint present: $(basename "$checkpoint")"
else
  fail "Checkpoint missing: $checkpoint"
fi

if [[ "$deep" == true ]]; then
  realsense_devices="$(rs-enumerate-devices 2>/dev/null || true)"
  grep -q 'Intel RealSense' <<<"$realsense_devices" \
    && pass "RealSense device detected" || fail "RealSense device not detected"
  if navdp_source_ros >/dev/null 2>&1 && navdp_activate_venv >/dev/null 2>&1 \
      && python -c 'import torch; assert torch.cuda.is_available(); import diffusers, flask, cv2, message_filters, rclpy' >/dev/null 2>&1; then
    pass "NavDP imports and CUDA"
  else
    fail "NavDP import/CUDA check failed"
  fi
  if [[ -f "$checkpoint" ]] \
      && echo "$expected  $checkpoint" | sha256sum --check --status; then
    pass "Verified $(basename "$checkpoint")"
  else
    fail "Checkpoint invalid: $checkpoint"
  fi
fi

if [[ "$CFG_WITH_GO2" == true ]]; then
  net_if="$CFG_UNITREE_NET_IF"
  if ip -o -4 addr show dev "$net_if" | grep -q '192\.168\.123\.'; then
    pass "$net_if has Go2 subnet address"
  else
    warn "$net_if has no 192.168.123.x address; Go2 will remain OFFLINE"
  fi
  if ping -c 1 -W 1 192.168.123.161 >/dev/null 2>&1; then
    pass "Go2 reachable at 192.168.123.161"
  else
    warn "Go2 not reachable at 192.168.123.161; UI and policy will still start locked"
  fi
fi

printf '\nPreflight complete: failures=%d warnings=%d\n' "$failures" "$warnings"
exit "$failures"
