#!/usr/bin/env bash

ODIN_GT_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ODIN_GT_ROOT="$(cd "$ODIN_GT_SCRIPT_DIR/.." && pwd)"
NAVDP_ROOT="$(cd "$ODIN_GT_ROOT/../.." && pwd)"
ODIN_WS="${ODIN_WS:-/home/unitree/.local/share/memnav/odin_ws}"
ODIN_DRIVER_ROOT="${ODIN_DRIVER_ROOT:-$ODIN_WS/src/odin_ros_driver}"
ODIN_ROS_SETUP="${ODIN_ROS_SETUP:-/opt/ros/humble/setup.bash}"
ODIN_RUNTIME_ROOT="${ODIN_RUNTIME_ROOT:-$NAVDP_ROOT/runtime/odin1_gt}"
ODIN_DRIVER_REPOSITORY="https://github.com/manifoldsdk/odin_ros_driver.git"
ODIN_DRIVER_PROFILE_RECEIPT="$ODIN_WS/.memnav_odin_driver_profile.json"
ODIN_DEFAULT_DRIVER_PROFILE="native_0_14"
ODIN_NATIVE_0_14_COMMIT="6f993ccc4ccad9395bfc68bc3235f993d83c4fe6"
ODIN_LEGACY_0_13_1_COMMIT="13aa528b1da581e2168ac858f8b144f0b4438a7a"
ODIN_LEGACY_MODE1_PATCH="$ODIN_GT_ROOT/vendor/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch"
ODIN_RUNTIME_CONFIG_PATCH="$ODIN_GT_ROOT/vendor/odin_ros_driver_runtime_config.patch"
ODIN_LEGACY_MODE1_PATCH_SHA256="2a73aa48d163e2a362670b7b9b778edf8328aba7323e1cc04dd6b8fb28ba5806"
ODIN_RUNTIME_CONFIG_PATCH_SHA256="953bd96ad3cea5c336f11882f92a428ff090ba13abd28c742314f072cd637f86"

odin_select_driver_profile() {
  local profile="${1:-$ODIN_DEFAULT_DRIVER_PROFILE}"
  case "$profile" in
    native_0_14)
      ODIN_SELECTED_PROFILE="$profile"
      ODIN_EXPECTED_COMMIT="$ODIN_NATIVE_0_14_COMMIT"
      ODIN_EXPECTED_TAG="v0.14.0"
      ODIN_FIRMWARE_CONTRACT="0.14.x_native_mode1"
      ;;
    legacy_0_13_1)
      ODIN_SELECTED_PROFILE="$profile"
      ODIN_EXPECTED_COMMIT="$ODIN_LEGACY_0_13_1_COMMIT"
      ODIN_EXPECTED_TAG="historical_pinned_commit"
      ODIN_FIRMWARE_CONTRACT="0.13.1_mode1_bootstrap_compatibility"
      ;;
    *)
      echo "unsupported Odin driver profile: $profile" >&2
      return 1
      ;;
  esac
}

odin_patch_is_applied() {
  local patch="$1"
  git -C "$ODIN_DRIVER_ROOT" apply --reverse --check "$patch" >/dev/null 2>&1
}

odin_source_file() {
  local path="$1"
  [[ -f "$path" ]] || return 1
  local restore_nounset=false
  case $- in
    *u*) restore_nounset=true; set +u ;;
  esac
  source "$path"
  [[ "$restore_nounset" == false ]] || set -u
}

odin_source_ros() {
  odin_source_file "$ODIN_ROS_SETUP" || {
    echo "ROS setup is missing: $ODIN_ROS_SETUP" >&2
    return 1
  }
  odin_source_file "$ODIN_WS/install/setup.bash" || {
    echo "Odin workspace is not built: $ODIN_WS/install/setup.bash" >&2
    return 1
  }
  export PYTHONPATH="$ODIN_GT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
}

odin_validate_id() {
  [[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] || {
    echo "invalid Odin session/run identifier: $1" >&2
    return 1
  }
}

odin_require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing command: $1" >&2
    return 1
  }
}
