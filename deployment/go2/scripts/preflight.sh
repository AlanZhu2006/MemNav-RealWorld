#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

backend="x"
check_robot=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend) backend="$2"; shift 2 ;;
    --with-go2) check_robot=true; shift ;;
    *) echo "Usage: $0 [--backend x|base] [--with-go2]" >&2; exit 2 ;;
  esac
done

failures=0
warnings=0
pass() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; warnings=$((warnings + 1)); }
fail() { printf '[FAIL] %s\n' "$*"; failures=$((failures + 1)); }

[[ "$(uname -m)" == "aarch64" ]] && pass "aarch64 Jetson userspace" || fail "Expected aarch64"
[[ -r /etc/nv_tegra_release ]] && pass "L4T $(head -n1 /etc/nv_tegra_release)" || warn "L4T release file missing"
[[ -f "$NAVDP_ROS_SETUP" ]] && pass "ROS 2 Humble setup" || fail "ROS setup missing"
[[ -f "$NAVDP_REALSENSE_SETUP" ]] && pass "RealSense ROS workspace" || fail "RealSense workspace missing"

realsense_devices=""
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  realsense_devices="$(rs-enumerate-devices 2>/dev/null || true)"
fi
if grep -q 'Intel RealSense' <<<"$realsense_devices"; then
  pass "RealSense device detected"
else
  fail "RealSense device not detected"
fi

if [[ -x "$NAVDP_VENV/bin/python" ]]; then
  if navdp_source_ros >/dev/null 2>&1 && navdp_activate_venv >/dev/null 2>&1 && \
      python -c 'import torch; assert torch.cuda.is_available(); import diffusers, flask, cv2, message_filters, rclpy' >/dev/null 2>&1; then
    pass "NavDP Python environment and CUDA"
  else
    fail "NavDP environment import/CUDA check failed"
  fi
else
  fail "Run setup_jetson.sh"
fi

if [[ "$backend" == "x" ]]; then
  checkpoint="$NAVDP_ROOT/baselines/x-navdp/checkpoints/x-navdp_posttrain.ckpt"
  expected="267089a81bbbe7a913debda6603f3f1b66a79520370ce953b2d888d793b89f24"
else
  checkpoint="$NAVDP_ROOT/baselines/navdp/checkpoints/navdp_pretrain.ckpt"
  expected="3bb3ad4ab241e857bb57a4021cc6aab76d5263e81fbf80298d579053ef011947"
fi
if [[ -f "$checkpoint" ]] && echo "$expected  $checkpoint" | sha256sum --check --status; then
  pass "Verified $(basename "$checkpoint")"
else
  fail "Checkpoint missing or invalid: $checkpoint"
fi

if curl -fsS --max-time 1 http://127.0.0.1:8888/healthz >/dev/null 2>&1; then
  pass "Policy server health endpoint"
else
  warn "Policy server is not running yet"
fi

if command -v ros2 >/dev/null 2>&1; then
  topics="$(timeout 3 ros2 topic list 2>/dev/null || true)"
  grep -qx '/camera/camera/color/image_raw' <<<"$topics" && pass "RGB topic active" || warn "RGB topic not active"
  grep -qx '/camera/camera/aligned_depth_to_color/image_raw' <<<"$topics" && pass "Aligned depth topic active" || warn "Aligned depth topic not active"
fi

if [[ "$check_robot" == true ]]; then
  net_if="${UNITREE_NET_IF:-eth0}"
  if ip -o -4 addr show dev "$net_if" | grep -q '192\.168\.123\.'; then
    pass "$net_if has Go2 subnet address"
  else
    fail "$net_if has no 192.168.123.x address"
  fi
  if ping -c 1 -W 1 192.168.123.161 >/dev/null 2>&1; then
    pass "Go2 reachable at 192.168.123.161"
  else
    fail "Go2 not reachable at 192.168.123.161"
  fi
fi

printf '\nPreflight complete: failures=%d warnings=%d\n' "$failures" "$warnings"
exit "$failures"
