#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

HARDWARE=false
EXPECTED_PROFILE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --hardware) HARDWARE=true ;;
    --expected-profile)
      [[ $# -ge 2 ]] || { echo "$1 requires a profile" >&2; exit 2; }
      EXPECTED_PROFILE="$2"
      shift
      ;;
    *)
      echo "usage: $0 [--hardware] [--expected-profile native_0_14|legacy_0_13_1]" >&2
      exit 2
      ;;
  esac
  shift
done

[[ -d "$ODIN_DRIVER_ROOT/.git" ]] || {
  echo "Odin driver checkout is missing: $ODIN_DRIVER_ROOT" >&2
  exit 1
}
[[ -s "$ODIN_DRIVER_PROFILE_RECEIPT" ]] || {
  echo "Odin driver profile receipt is missing: $ODIN_DRIVER_PROFILE_RECEIPT" >&2
  exit 1
}
installed_profile="$(python3 - "$ODIN_DRIVER_PROFILE_RECEIPT" <<'PY'
import json, pathlib, sys
payload = json.loads(pathlib.Path(sys.argv[1]).read_text())
if payload.get("schema") != "memnav-odin1-driver-profile-v1":
    raise SystemExit("unsupported Odin driver profile receipt")
print(payload["profile"])
PY
)"
odin_select_driver_profile "$installed_profile"
if [[ -n "$EXPECTED_PROFILE" && "$installed_profile" != "$EXPECTED_PROFILE" ]]; then
  echo "installed Odin profile is $installed_profile, expected $EXPECTED_PROFILE" >&2
  exit 1
fi
[[ "$(git -C "$ODIN_DRIVER_ROOT" rev-parse HEAD)" == "$ODIN_EXPECTED_COMMIT" ]] || {
  echo "Odin driver commit is not the pinned deployment commit" >&2
  exit 1
}
odin_patch_is_applied "$ODIN_RUNTIME_CONFIG_PATCH" || {
  echo "tracked runtime config override patch is not exactly applied" >&2
  exit 1
}
if [[ "$installed_profile" == "legacy_0_13_1" ]]; then
  odin_patch_is_applied "$ODIN_LEGACY_MODE1_PATCH" || {
    echo "tracked legacy firmware patch is not exactly applied" >&2
    exit 1
  }
fi
[[ "$(sha256sum "$ODIN_LEGACY_MODE1_PATCH" | cut -d' ' -f1)" == \
  "$ODIN_LEGACY_MODE1_PATCH_SHA256" ]] || {
  echo "tracked Odin compatibility patch SHA mismatch" >&2
  exit 1
}
[[ "$(sha256sum "$ODIN_RUNTIME_CONFIG_PATCH" | cut -d' ' -f1)" == \
  "$ODIN_RUNTIME_CONFIG_PATCH_SHA256" ]] || {
  echo "tracked Odin runtime-config patch SHA mismatch" >&2
  exit 1
}
python3 - "$ODIN_DRIVER_PROFILE_RECEIPT" "$ODIN_EXPECTED_COMMIT" \
  "$ODIN_RUNTIME_CONFIG_PATCH_SHA256" "$ODIN_LEGACY_MODE1_PATCH_SHA256" \
  "$ODIN_DRIVER_ROOT" <<'PY'
import hashlib
import json
import pathlib
import subprocess
import sys

receipt_path, expected_commit, runtime_sha, legacy_sha, driver_root_arg = sys.argv[1:]
driver_root = pathlib.Path(driver_root_arg)
payload = json.loads(pathlib.Path(receipt_path).read_text())
if payload.get("commit") != expected_commit:
    raise SystemExit("driver receipt commit mismatch")
actual = {item.get("name"): item.get("sha256") for item in payload.get("patches", [])}
if actual.get("odin_ros_driver_runtime_config.patch") != runtime_sha:
    raise SystemExit("driver receipt runtime patch mismatch")
if payload.get("profile") == "legacy_0_13_1":
    if actual.get("odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch") != legacy_sha:
        raise SystemExit("driver receipt legacy patch mismatch")
elif "odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch" in actual:
    raise SystemExit("native 0.14 profile must not include the legacy mode1 patch")
modified = set(
    subprocess.run(
        ["git", "diff", "--name-only", "HEAD", "--"],
        cwd=driver_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
)
expected_files = payload.get("modified_files", {})
if modified != set(expected_files):
    raise SystemExit("driver modified-file set no longer matches its profile receipt")
for relative, expected_sha in expected_files.items():
    actual_sha = hashlib.sha256((driver_root / relative).read_bytes()).hexdigest()
    if actual_sha != expected_sha:
        raise SystemExit(f"driver file changed after setup: {relative}")
PY
git -C "$ODIN_DRIVER_ROOT" diff --check
[[ -f "$ODIN_WS/install/setup.bash" ]] || {
  echo "Odin driver is not built: $ODIN_WS/install/setup.bash" >&2
  exit 1
}
odin_source_ros
driver_executable="$ODIN_WS/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample"
[[ -x "$driver_executable" ]] || {
  echo "Odin driver executable is missing: $driver_executable" >&2
  exit 1
}
driver_dependencies="$(ldd "$driver_executable")"
if grep -Fq "not found" <<<"$driver_dependencies"; then
  echo "Odin driver has unresolved shared libraries:" >&2
  grep -F "not found" <<<"$driver_dependencies" >&2
  exit 1
fi
if grep -Fq "libopencv_core.so.408" <<<"$driver_dependencies" && \
   grep -Fq "libopencv_core.so.4.5d" <<<"$driver_dependencies"; then
  echo "WARNING: Odin links JetPack OpenCV 4.8 and ROS cv_bridge OpenCV 4.5;" >&2
  echo "         live topic stability remains a mandatory hardware gate." >&2
fi
if [[ -n "${ODIN_CALIBRATION_FILE:-}" ]]; then
  [[ -r "$ODIN_CALIBRATION_FILE" ]] || {
    echo "ODIN_CALIBRATION_FILE is unreadable: $ODIN_CALIBRATION_FILE" >&2
    exit 1
  }
  calibration_sha="$(sha256sum "$ODIN_CALIBRATION_FILE" | cut -d' ' -f1)"
  if [[ -n "${ODIN_EXPECTED_CALIBRATION_SHA256:-}" && \
      "$calibration_sha" != "$ODIN_EXPECTED_CALIBRATION_SHA256" ]]; then
    echo "Odin serial-specific calibration SHA mismatch" >&2
    exit 1
  fi
  echo "  calibration sha256: $calibration_sha"
fi

if [[ "$HARDWARE" == true ]]; then
  lsusb -d 2207:0019 >/dev/null || {
    echo "Odin1 USB device 2207:0019 is not connected" >&2
    exit 1
  }
  graph="$(timeout 8 ros2 topic list -t 2>/dev/null || true)"
  for contract in \
    "/odin1/image [sensor_msgs/msg/Image]" \
    "/odin1/cloud_slam [sensor_msgs/msg/PointCloud2]" \
    "/odin1/odometry [nav_msgs/msg/Odometry]"; do
    grep -Fqx "$contract" <<<"$graph" || {
      echo "missing live Odin topic contract: $contract" >&2
      exit 1
    }
    timeout 5 ros2 topic echo --once "${contract%% *}" >/dev/null
  done
fi

echo "Odin1 GT preflight passed (hardware=$HARDWARE)"
echo "  driver profile: $installed_profile"
echo "  driver commit: $ODIN_EXPECTED_COMMIT"
echo "  policy input:  none"
echo "  motion change: none"
