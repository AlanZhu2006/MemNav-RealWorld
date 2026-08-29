#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

die() {
  echo "odin-gt: $*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Independent Odin1 evaluation lane; no command below publishes robot motion.

Setup and checks:
  odin_gt.sh setup [--install-deps] [--driver-profile native_0_14|legacy_0_13_1]
  odin_gt.sh preflight [--hardware]

One-time scene mapping survey:
  odin_gt.sh start-map SESSION_ID --sensor-serial SERIAL --firmware-version VERSION \
    --calibration-file FILE --mount-receipt JSON \
    --obstacle-min-z M --obstacle-max-z M
  odin_gt.sh capture-goal SESSION_ID GOAL_RGB [GOAL_DEPTH]
  odin_gt.sh finish-map SESSION_ID

Each formal run:
  odin_gt.sh start-formal RUN_ID SEALED_GOAL_RECEIPT
  odin_gt.sh wait-ready RUN_ID [TIMEOUT_S]
  odin_gt.sh status formal RUN_ID
  odin_gt.sh stop-formal RUN_ID
  odin_gt.sh score RUN_ID --robot-radius M [--inflation-margin M]

The formal lane starts only Odin relocalization, GT monitor and GT rosbag.
Start/stop NavDP and Go2 separately through their existing safety-gated stack.
EOF
}

map_root() {
  printf '%s/maps/%s' "$ODIN_RUNTIME_ROOT" "$1"
}

formal_root() {
  printf '%s/formal/%s' "$ODIN_RUNTIME_ROOT" "$1"
}

map_session() {
  printf 'memnav-odin-map-%s' "$1"
}

formal_session() {
  printf 'memnav-odin-formal-%s' "$1"
}

write_driver_launcher() {
  local path="$1"
  local config="$2"
  cat >"$path" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source $(printf '%q' "$SCRIPT_DIR/common.sh")
odin_source_ros
exec ros2 run odin_ros_driver host_sdk_sample --ros-args -p config_file:=$(printf '%q' "$config")
EOF
  chmod +x "$path"
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + ${2:-20}))
  while (( SECONDS < deadline )); do
    if timeout 4 ros2 topic echo --once "$topic" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_map() {
  local session_id="$1"
  shift
  odin_validate_id "$session_id" || exit 1
  local minimum_z=""
  local maximum_z=""
  local sensor_serial=""
  local firmware_version=""
  local calibration_file=""
  local mount_receipt=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --obstacle-min-z) [[ $# -ge 2 ]] || die "$1 requires metres"; minimum_z="$2"; shift ;;
      --obstacle-max-z) [[ $# -ge 2 ]] || die "$1 requires metres"; maximum_z="$2"; shift ;;
      --sensor-serial) [[ $# -ge 2 ]] || die "$1 requires a serial"; sensor_serial="$2"; shift ;;
      --firmware-version) [[ $# -ge 2 ]] || die "$1 requires a version"; firmware_version="$2"; shift ;;
      --calibration-file) [[ $# -ge 2 ]] || die "$1 requires a file"; calibration_file="$2"; shift ;;
      --mount-receipt) [[ $# -ge 2 ]] || die "$1 requires a JSON receipt"; mount_receipt="$2"; shift ;;
      *) die "unknown start-map option: $1" ;;
    esac
    shift
  done
  [[ -n "$minimum_z" && -n "$maximum_z" ]] || die \
    "measured Odin-frame obstacle z limits are mandatory; no unsafe defaults exist"
  [[ -n "$sensor_serial" ]] || die "--sensor-serial is mandatory"
  [[ -n "$firmware_version" ]] || die "--firmware-version is mandatory"
  [[ -n "$calibration_file" ]] || die "--calibration-file is mandatory"
  [[ -n "$mount_receipt" ]] || die "--mount-receipt is mandatory"
  calibration_file="$(readlink -f "$calibration_file")"
  mount_receipt="$(readlink -f "$mount_receipt")"
  [[ -s "$calibration_file" ]] || die "calibration file is missing or empty"
  [[ -s "$mount_receipt" ]] || die "mount receipt is missing or empty"
  "$SCRIPT_DIR/preflight.sh"
  local installed_profile
  installed_profile="$(python3 - "$ODIN_DRIVER_PROFILE_RECEIPT" <<'PY'
import json, pathlib, sys
print(json.loads(pathlib.Path(sys.argv[1]).read_text())["profile"])
PY
)"
  case "$installed_profile" in
    native_0_14)
      [[ "$firmware_version" =~ ^0\.14([.-].*)?$ ]] || die \
        "native_0_14 requires the exact reported 0.14 firmware version"
      ;;
    legacy_0_13_1)
      [[ "$firmware_version" == "0.13.1" ]] || die \
        "legacy_0_13_1 requires firmware version 0.13.1"
      ;;
    *) die "unsupported installed driver profile: $installed_profile" ;;
  esac
  lsusb -d 2207:0019 >/dev/null || die "Odin1 USB device 2207:0019 is not connected"
  local root session config
  root="$(map_root "$session_id")"
  session="$(map_session "$session_id")"
  [[ ! -e "$root" ]] || die "mapping session already exists: $root"
  ! tmux has-session -t "$session" 2>/dev/null || die "tmux session exists: $session"
  mkdir -p "$root"/{logs,receipts,rosbag}
  cp "$calibration_file" "$root/receipts/odin_calibration.yaml"
  cp "$mount_receipt" "$root/receipts/odin_mount.json"
  cp "$ODIN_DRIVER_PROFILE_RECEIPT" "$root/receipts/driver_profile.json"
  PYTHONPATH="$ODIN_GT_ROOT" python3 "$ODIN_GT_ROOT/make_scene_contract.py" \
    --mapping-session-id "$session_id" --sensor-serial "$sensor_serial" \
    --firmware-version "$firmware_version" \
    --calibration-file "$root/receipts/odin_calibration.yaml" \
    --mount-receipt "$root/receipts/odin_mount.json" \
    --driver-profile-receipt "$root/receipts/driver_profile.json" \
    --output "$root/scene_contract.json"
  config="$root/receipts/driver_mapping.yaml"
  PYTHONPATH="$ODIN_GT_ROOT" python3 "$ODIN_GT_ROOT/make_driver_config.py" \
    --base-config "$ODIN_DRIVER_ROOT/config/control_command.yaml" \
    --mode mapping --output "$config" \
    --mapping-output-dir "$root" --mapping-output-name odin_map.bin \
    >"$root/receipts/driver_config_receipt.stdout.json"
  write_driver_launcher "$root/receipts/launch_driver.sh" "$config"
  cat >"$root/receipts/launch_occupancy.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source $(printf '%q' "$SCRIPT_DIR/common.sh")
odin_source_ros
exec python3 $(printf '%q' "$ODIN_GT_ROOT/odin_occupancy_builder.py") \
  --session-id $(printf '%q' "$session_id") \
  --output-prefix $(printf '%q' "$root/occupancy") \
  --obstacle-min-z-m $(printf '%q' "$minimum_z") \
  --obstacle-max-z-m $(printf '%q' "$maximum_z")
EOF
  cat >"$root/receipts/launch_rosbag.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source $(printf '%q' "$SCRIPT_DIR/common.sh")
odin_source_ros
exec ros2 bag record --include-unpublished-topics -o $(printf '%q' "$root/rosbag/odin_mapping") \
  /odin1/image /odin1/image/compressed /odin1/imu /odin1/cloud_slam \
  /odin1/odometry /odin1/odometry_high /odin1/odometry_highfreq /odin1/path \
  /tf /tf_static
EOF
  chmod +x "$root/receipts/launch_"*.sh
  tmux new-session -d -s "$session" -n driver \
    "exec '$root/receipts/launch_driver.sh' >'$root/logs/driver.log' 2>&1"
  sleep 2
  tmux new-window -t "$session" -n occupancy \
    "exec '$root/receipts/launch_occupancy.sh' >'$root/logs/occupancy.log' 2>&1"
  tmux new-window -t "$session" -n rosbag \
    "exec '$root/receipts/launch_rosbag.sh' >'$root/logs/rosbag.log' 2>&1"
  odin_source_ros
  if ! wait_for_topic /odin1/odometry 25 || ! wait_for_topic /odin1/cloud_slam 25; then
    tmux kill-session -t "$session" 2>/dev/null || true
    die "Odin mapping topics did not become live; inspect $root/logs/driver.log"
  fi
  echo "Odin mapping survey is recording"
  echo "  session:       $session_id"
  echo "  root:          $root"
  echo "  obstacle z:    [$minimum_z, $maximum_z] m in odom"
  echo "  Odin sensor:   $sensor_serial firmware=$firmware_version"
  echo "  motion change: none; drive the out-and-back survey manually"
}

capture_goal_anchor() {
  local session_id="$1"
  local rgb="$2"
  local depth="${3:-}"
  odin_validate_id "$session_id" || exit 1
  local root session args
  root="$(map_root "$session_id")"
  session="$(map_session "$session_id")"
  [[ -d "$root" ]] || die "unknown mapping session: $session_id"
  tmux has-session -t "$session" 2>/dev/null || die "mapping session is not running"
  [[ ! -e "$root/goal_anchor.draft.json" ]] || die \
    "goal anchor draft already exists for this mapping session"
  rgb="$(readlink -f "$rgb")"
  [[ -s "$rgb" ]] || die "goal RGB is missing or empty"
  cp "$rgb" "$root/receipts/d435i_goal_rgb.bin"
  args=(capture-goal --mapping-session-id "$session_id"
    --goal-rgb "$root/receipts/d435i_goal_rgb.bin"
    --output "$root/goal_anchor.draft.json")
  if [[ -n "$depth" ]]; then
    depth="$(readlink -f "$depth")"
    [[ -s "$depth" ]] || die "goal depth is missing or empty"
    cp "$depth" "$root/receipts/d435i_goal_depth.bin"
    args+=(--goal-depth "$root/receipts/d435i_goal_depth.bin")
  fi
  odin_source_ros
  PYTHONPATH="$ODIN_GT_ROOT" python3 "$ODIN_GT_ROOT/odin_gt_monitor.py" "${args[@]}"
  echo "Goal anchor draft captured; finish the full out-and-back survey before sealing"
}

save_map() {
  local session_id="$1"
  local root session map_file
  root="$(map_root "$session_id")"
  session="$(map_session "$session_id")"
  map_file="$root/odin_map.bin"
  [[ -d "$root" ]] || die "unknown mapping session: $session_id"
  tmux has-session -t "$session" 2>/dev/null || die "mapping session is not running"
  [[ ! -e "$map_file" ]] || die "map output already exists: $map_file"
  printf 'set save_map 1\n' > /tmp/odin_command.txt
  local deadline=$((SECONDS + 180))
  local previous_size=-1
  local stable_count=0
  while (( SECONDS < deadline )); do
    if [[ -s "$map_file" ]]; then
      size="$(stat -c %s "$map_file")"
      if [[ "$size" == "$previous_size" ]]; then
        stable_count=$((stable_count + 1))
      else
        stable_count=0
      fi
      previous_size="$size"
      if (( stable_count >= 5 )); then
        break
      fi
    fi
    sleep 1
  done
  [[ -s "$map_file" && $stable_count -ge 5 ]] || die \
    "map transfer did not complete within 180 s; keep the driver running and inspect its log"
  python3 - "$map_file" "$root/receipts/driver_mapping.yaml" "$root/map.receipt.json" <<'PY'
import hashlib, json, pathlib, sys
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
map_path, config_path, output = map(pathlib.Path, sys.argv[1:])
payload = {
    "schema": "memnav-odin1-proprietary-map-v1",
    "map": {"path": str(map_path.resolve()), "bytes": map_path.stat().st_size, "sha256": sha(map_path)},
    "driver_config": {"path": str(config_path.resolve()), "sha256": sha(config_path)},
    "policy_input": False,
    "motion_authority": False,
}
output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
print(json.dumps(payload, indent=2, sort_keys=True))
PY
}

stop_tmux_gracefully() {
  local session="$1"
  shift
  local window
  for window in "$@"; do
    tmux send-keys -t "$session:$window" C-c 2>/dev/null || true
  done
  local deadline=$((SECONDS + 30))
  while tmux has-session -t "$session" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 1
  done
  if tmux has-session -t "$session" 2>/dev/null; then
    tmux kill-session -t "$session" 2>/dev/null || true
    return 1
  fi
}

finish_map() {
  local session_id="$1"
  local root session
  odin_validate_id "$session_id" || exit 1
  root="$(map_root "$session_id")"
  session="$(map_session "$session_id")"
  [[ -s "$root/odin_map.bin" ]] || save_map "$session_id"
  stop_tmux_gracefully "$session" occupancy rosbag driver || die \
    "mapping recorders required forced tmux cleanup; do not certify this survey"
  [[ -s "$root/occupancy.yaml" && -s "$root/occupancy.pgm" && \
      -s "$root/occupancy.receipt.json" ]] || die "occupancy builder did not seal its outputs"
  if [[ -s "$root/goal_anchor.draft.json" ]]; then
    PYTHONPATH="$ODIN_GT_ROOT" python3 "$ODIN_GT_ROOT/odin_gt_monitor.py" seal-goal \
      --draft "$root/goal_anchor.draft.json" --map-file "$root/odin_map.bin" \
      --occupancy-yaml "$root/occupancy.yaml" \
      --scene-contract "$root/scene_contract.json" --output "$root/goal_anchor.json"
  fi
  echo "Odin mapping survey sealed"
  echo "  map:       $root/odin_map.bin"
  echo "  occupancy: $root/occupancy.yaml"
  echo "  goal:      $root/goal_anchor.json"
}

start_formal() {
  local run_id="$1"
  local goal_receipt="$2"
  odin_validate_id "$run_id" || exit 1
  goal_receipt="$(readlink -f "$goal_receipt")"
  [[ -s "$goal_receipt" ]] || die "sealed goal receipt is missing: $goal_receipt"
  local map_file
  map_file="$(python3 - "$goal_receipt" <<'PY'
import json, pathlib, sys
p=json.loads(pathlib.Path(sys.argv[1]).read_text())
assert p.get("schema") == "memnav-odin1-goal-anchor-v1"
print(p["odin_map"]["path"])
PY
)" || die "invalid sealed goal receipt"
  [[ -s "$map_file" ]] || die "sealed Odin map is missing: $map_file"
  "$SCRIPT_DIR/preflight.sh"
  lsusb -d 2207:0019 >/dev/null || die "Odin1 USB device 2207:0019 is not connected"
  local root session config
  root="$(formal_root "$run_id")"
  session="$(formal_session "$run_id")"
  [[ ! -e "$root" ]] || die "formal GT run already exists: $root"
  ! tmux has-session -t "$session" 2>/dev/null || die "tmux session exists: $session"
  mkdir -p "$root"/{logs,receipts,rosbag}
  printf '%s\n' "$goal_receipt" >"$root/receipts/goal_receipt.path"
  config="$root/receipts/driver_relocalization.yaml"
  PYTHONPATH="$ODIN_GT_ROOT" python3 "$ODIN_GT_ROOT/make_driver_config.py" \
    --base-config "$ODIN_DRIVER_ROOT/config/control_command.yaml" \
    --mode relocalization --map-file "$map_file" --output "$config" \
    >"$root/receipts/driver_config_receipt.stdout.json"
  write_driver_launcher "$root/receipts/launch_driver.sh" "$config"
  cat >"$root/receipts/launch_monitor.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source $(printf '%q' "$SCRIPT_DIR/common.sh")
odin_source_ros
exec python3 $(printf '%q' "$ODIN_GT_ROOT/odin_gt_monitor.py") run \
  --run-id $(printf '%q' "$run_id") --goal-receipt $(printf '%q' "$goal_receipt") \
  --map-file $(printf '%q' "$map_file") \
  --driver-profile-receipt $(printf '%q' "$ODIN_DRIVER_PROFILE_RECEIPT") \
  --output-dir $(printf '%q' "$root/monitor")
EOF
  cat >"$root/receipts/launch_rosbag.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source $(printf '%q' "$SCRIPT_DIR/common.sh")
odin_source_ros
exec ros2 bag record --include-unpublished-topics -o $(printf '%q' "$root/rosbag/odin_formal") \
  /odin1/odometry /odin1/odometry_high /odin1/odometry_highfreq /odin1/path \
  /odin1/cloud_slam /tf /tf_static /navdp/gt/status /navdp/rgb_arrival_status \
  /navdp/status /navdp/cmd_vel /navdp/trajectory /navdp/estop
EOF
  chmod +x "$root/receipts/launch_"*.sh
  tmux new-session -d -s "$session" -n driver \
    "exec '$root/receipts/launch_driver.sh' >'$root/logs/driver.log' 2>&1"
  sleep 2
  tmux new-window -t "$session" -n monitor \
    "exec '$root/receipts/launch_monitor.sh' >'$root/logs/monitor.log' 2>&1"
  tmux new-window -t "$session" -n rosbag \
    "exec '$root/receipts/launch_rosbag.sh' >'$root/logs/rosbag.log' 2>&1"
  odin_source_ros
  if ! wait_for_topic /navdp/gt/status 15; then
    tmux kill-session -t "$session" 2>/dev/null || true
    die "GT monitor did not publish; inspect $root/logs"
  fi
  echo "Odin formal GT lane started; robot motion remains unchanged"
  echo "  run:       $run_id"
  echo "  root:      $root"
  echo "  next:      $0 wait-ready $run_id 120"
  echo "  IMPORTANT: do not enable NavDP until reference_ready=true"
}

wait_ready() {
  local run_id="$1"
  local timeout_s="${2:-120}"
  local log="$(formal_root "$run_id")/monitor/status.jsonl"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if [[ -s "$log" ]] && python3 - "$log" <<'PY'
import json, pathlib, sys
line=pathlib.Path(sys.argv[1]).read_text().splitlines()[-1]
raise SystemExit(0 if json.loads(line).get("reference_ready") is True else 1)
PY
    then
      echo "reference_ready=true"
      echo "Odin map->odom is stable; the onsite operator may now start the NavDP run"
      return 0
    fi
    sleep 1
  done
  die "Odin relocalization did not pass within ${timeout_s}s; keep Go2 disabled and inspect status"
}

status_stack() {
  local kind="$1"
  local identifier="$2"
  local root session
  case "$kind" in
    map) root="$(map_root "$identifier")"; session="$(map_session "$identifier")" ;;
    formal) root="$(formal_root "$identifier")"; session="$(formal_session "$identifier")" ;;
    *) die "status kind must be map or formal" ;;
  esac
  [[ -d "$root" ]] || die "unknown $kind identifier: $identifier"
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "session=RUNNING ($session)"
    tmux list-windows -t "$session" -F '  #{window_name}: #{pane_current_command}'
  else
    echo "session=STOPPED ($session)"
  fi
  if [[ -s "$root/monitor/status.jsonl" ]]; then
    tail -n 1 "$root/monitor/status.jsonl" | python3 -m json.tool
  fi
  du -sh "$root"
}

stop_formal() {
  local run_id="$1"
  local root session
  root="$(formal_root "$run_id")"
  session="$(formal_session "$run_id")"
  [[ -d "$root" ]] || die "unknown formal GT run: $run_id"
  tmux has-session -t "$session" 2>/dev/null || die "formal GT session is not running"
  stop_tmux_gracefully "$session" monitor rosbag driver || die \
    "formal GT recorders required forced cleanup; mark the run system_failure"
  [[ -s "$root/monitor/result.json" ]] || die "GT monitor did not seal result.json"
  echo "Odin formal GT lane stopped"
  echo "  result: $root/monitor/result.json"
  echo "  next:   $0 score $run_id --robot-radius <measured-m>"
}

score_run() {
  local run_id="$1"
  shift
  local robot_radius=""
  local inflation_margin="0.05"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --robot-radius) [[ $# -ge 2 ]] || die "$1 requires metres"; robot_radius="$2"; shift ;;
      --inflation-margin) [[ $# -ge 2 ]] || die "$1 requires metres"; inflation_margin="$2"; shift ;;
      *) die "unknown score option: $1" ;;
    esac
    shift
  done
  [[ -n "$robot_radius" ]] || die "--robot-radius is mandatory and must be physically measured"
  local root goal_receipt
  root="$(formal_root "$run_id")"
  [[ -s "$root/monitor/result.json" ]] || die "formal GT result is missing"
  goal_receipt="$(cat "$root/receipts/goal_receipt.path")"
  PYTHONPATH="$ODIN_GT_ROOT" python3 "$ODIN_GT_ROOT/score_odin_gt.py" \
    --gt-result "$root/monitor/result.json" --goal-receipt "$goal_receipt" \
    --output "$root/spl_receipt.json" --overlay "$root/astar_overlay.png" \
    --robot-radius-m "$robot_radius" --inflation-margin-m "$inflation_margin"
}

action="${1:-}"
[[ $# -eq 0 ]] || shift
case "$action" in
  setup) "$SCRIPT_DIR/setup_driver.sh" "$@" ;;
  preflight) "$SCRIPT_DIR/preflight.sh" "$@" ;;
  start-map) [[ $# -ge 1 ]] || die "start-map requires SESSION_ID"; id="$1"; shift; start_map "$id" "$@" ;;
  capture-goal) [[ $# -ge 2 && $# -le 3 ]] || die "capture-goal requires SESSION_ID RGB [DEPTH]"; capture_goal_anchor "$@" ;;
  finish-map) [[ $# -eq 1 ]] || die "finish-map requires SESSION_ID"; finish_map "$1" ;;
  start-formal) [[ $# -eq 2 ]] || die "start-formal requires RUN_ID SEALED_GOAL_RECEIPT"; start_formal "$1" "$2" ;;
  wait-ready) [[ $# -ge 1 && $# -le 2 ]] || die "wait-ready requires RUN_ID [TIMEOUT_S]"; wait_ready "$@" ;;
  status) [[ $# -eq 2 ]] || die "status requires map|formal ID"; status_stack "$1" "$2" ;;
  stop-formal) [[ $# -eq 1 ]] || die "stop-formal requires RUN_ID"; stop_formal "$1" ;;
  score) [[ $# -ge 1 ]] || die "score requires RUN_ID"; id="$1"; shift; score_run "$id" "$@" ;;
  help|-h|--help|"") usage ;;
  *) die "unknown action: $action" ;;
esac
