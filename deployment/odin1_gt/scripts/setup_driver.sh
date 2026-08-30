#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

INSTALL_DEPS=false
DRIVER_PROFILE="$ODIN_DEFAULT_DRIVER_PROFILE"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-deps) INSTALL_DEPS=true ;;
    --driver-profile)
      [[ $# -ge 2 ]] || { echo "$1 requires a profile" >&2; exit 2; }
      DRIVER_PROFILE="$2"
      shift
      ;;
    *)
      echo "usage: $0 [--install-deps] [--driver-profile native_0_14|legacy_0_13_1]" >&2
      exit 2
      ;;
  esac
  shift
done
odin_select_driver_profile "$DRIVER_PROFILE"

if [[ "$INSTALL_DEPS" == true ]]; then
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake git libeigen3-dev libopencv-dev libpcl-dev \
    libssl-dev libusb-1.0-0-dev libyaml-cpp-dev python3-colcon-common-extensions
  printf '%s\n' \
    'SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"' |
    sudo tee /etc/udev/rules.d/99-odin-usb.rules >/dev/null
  sudo udevadm control --reload
  sudo udevadm trigger
fi

odin_require_command git
odin_require_command colcon
[[ -f "$ODIN_LEGACY_MODE1_PATCH" ]] || {
  echo "missing tracked legacy firmware patch: $ODIN_LEGACY_MODE1_PATCH" >&2
  exit 1
}
[[ -f "$ODIN_RUNTIME_CONFIG_PATCH" ]] || {
  echo "missing tracked runtime-config patch: $ODIN_RUNTIME_CONFIG_PATCH" >&2
  exit 1
}
mkdir -p "$ODIN_WS/src"
if [[ ! -d "$ODIN_DRIVER_ROOT/.git" ]]; then
  git clone "$ODIN_DRIVER_REPOSITORY" "$ODIN_DRIVER_ROOT"
fi
actual_remote="$(git -C "$ODIN_DRIVER_ROOT" remote get-url origin)"
[[ "$actual_remote" == "$ODIN_DRIVER_REPOSITORY" ]] || {
  echo "unexpected Odin driver remote: $actual_remote" >&2
  exit 1
}

if odin_patch_is_applied "$ODIN_RUNTIME_CONFIG_PATCH"; then
  git -C "$ODIN_DRIVER_ROOT" apply --reverse "$ODIN_RUNTIME_CONFIG_PATCH"
fi
if odin_patch_is_applied "$ODIN_LEGACY_MODE1_PATCH"; then
  git -C "$ODIN_DRIVER_ROOT" apply --reverse "$ODIN_LEGACY_MODE1_PATCH"
fi
if [[ -n "$(git -C "$ODIN_DRIVER_ROOT" status --porcelain --untracked-files=all)" ]]; then
  echo "Odin driver checkout contains changes not owned by this installer:" >&2
  git -C "$ODIN_DRIVER_ROOT" status --short >&2
  echo "Preserve or remove them manually; setup refuses destructive reset." >&2
  exit 1
fi

git -C "$ODIN_DRIVER_ROOT" fetch origin "$ODIN_EXPECTED_COMMIT"
git -C "$ODIN_DRIVER_ROOT" checkout --detach "$ODIN_EXPECTED_COMMIT"
if [[ "$ODIN_SELECTED_PROFILE" == "legacy_0_13_1" ]]; then
  git -C "$ODIN_DRIVER_ROOT" apply --check "$ODIN_LEGACY_MODE1_PATCH"
  git -C "$ODIN_DRIVER_ROOT" apply "$ODIN_LEGACY_MODE1_PATCH"
fi
git -C "$ODIN_DRIVER_ROOT" apply --check "$ODIN_RUNTIME_CONFIG_PATCH"
git -C "$ODIN_DRIVER_ROOT" apply "$ODIN_RUNTIME_CONFIG_PATCH"
cp "$ODIN_DRIVER_ROOT/package_ros2.xml" "$ODIN_DRIVER_ROOT/package.xml"
odin_source_file "$ODIN_ROS_SETUP"
export BUILD_SYSTEM=ROS2
colcon --log-base "$ODIN_WS/log" build \
  --base-paths "$ODIN_WS/src" \
  --build-base "$ODIN_WS/build" \
  --install-base "$ODIN_WS/install" \
  --packages-select odin_ros_driver \
  --parallel-workers "$(nproc)" \
  --cmake-args -DBUILD_SYSTEM=ROS2 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

python3 - "$ODIN_DRIVER_PROFILE_RECEIPT" "$ODIN_SELECTED_PROFILE" \
  "$ODIN_DRIVER_REPOSITORY" "$ODIN_EXPECTED_COMMIT" "$ODIN_EXPECTED_TAG" \
  "$ODIN_FIRMWARE_CONTRACT" "$ODIN_RUNTIME_CONFIG_PATCH" \
  "$ODIN_LEGACY_MODE1_PATCH" "$ODIN_DRIVER_ROOT" <<'PY'
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

(
    output_arg,
    profile,
    repository,
    commit,
    tag,
    firmware_contract,
    runtime_patch_arg,
    legacy_patch_arg,
    driver_root_arg,
) = sys.argv[1:]
output = Path(output_arg).expanduser().resolve()
runtime_patch = Path(runtime_patch_arg).resolve()
legacy_patch = Path(legacy_patch_arg).resolve()
driver_root = Path(driver_root_arg).resolve()

def receipt(path: Path) -> dict[str, object]:
    return {
        "name": path.name,
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }

patches = [receipt(runtime_patch)]
if profile == "legacy_0_13_1":
    patches.insert(0, receipt(legacy_patch))
expected_modified = {
    "native_0_14": {"src/host_sdk_sample.cpp"},
    "legacy_0_13_1": {
        "config/control_command.yaml",
        # The upstream patch still touches this unused legacy sample config.
        # MemNav never launches it; operator visualization uses Foxglove.
        "config/odin_ros2.rviz",
        "src/host_sdk_sample.cpp",
        "src/yaml_parser.cpp",
    },
}[profile]
modified = set(
    subprocess.run(
        ["git", "diff", "--name-only", "HEAD", "--"],
        cwd=driver_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
)
if modified != expected_modified:
    raise SystemExit(
        f"unexpected patched driver files: expected={sorted(expected_modified)} "
        f"actual={sorted(modified)}"
    )
payload = {
    "schema": "memnav-odin1-driver-profile-v1",
    "created_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    "profile": profile,
    "repository": repository,
    "commit": commit,
    "tag": tag,
    "firmware_contract": firmware_contract,
    "patches": patches,
    "modified_files": {
        relative: hashlib.sha256((driver_root / relative).read_bytes()).hexdigest()
        for relative in sorted(modified)
    },
    "native_mode1": profile == "native_0_14",
}
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
PY

echo "Odin1 driver built without starting hardware or robot motion"
echo "  workspace: $ODIN_WS"
echo "  profile:   $ODIN_SELECTED_PROFILE"
echo "  commit:    $ODIN_EXPECTED_COMMIT"
echo "  receipt:   $ODIN_DRIVER_PROFILE_RECEIPT"
echo "  next:      $ODIN_GT_SCRIPT_DIR/preflight.sh"
