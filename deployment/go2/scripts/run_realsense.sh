#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"
navdp_source_ros

MIN_FW_VERSION="${REALSENSE_MIN_FW_VERSION:-5.17}"
if ! command -v rs-enumerate-devices >/dev/null 2>&1; then
  echo "rs-enumerate-devices is unavailable." >&2
  exit 1
fi
enumeration="$(rs-enumerate-devices 2>&1)" || {
  echo "$enumeration" >&2
  exit 1
}
firmware="$(awk -F: '/Firmware Version/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}' <<<"$enumeration")"
if [[ -z "$firmware" ]]; then
  echo "Could not read RealSense firmware." >&2
  exit 1
fi
if [[ "$(printf '%s\n%s\n' "$MIN_FW_VERSION" "$firmware" | sort -V | head -n1)" != "$MIN_FW_VERSION" ]]; then
  echo "RealSense firmware $firmware is older than $MIN_FW_VERSION." >&2
  exit 1
fi
echo "Starting aligned RGB-D only (no infrared/IMU/VIO), firmware=$firmware"

exec ros2 launch realsense2_camera rs_launch.py \
  initial_reset:=true \
  publish_tf:=true \
  tf_publish_rate:=1.0 \
  enable_depth:=true \
  enable_color:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  enable_sync:=true \
  align_depth.enable:=true \
  pointcloud.enable:=false \
  depth_module.depth_profile:=848x480x30 \
  rgb_camera.color_profile:=848x480x30
