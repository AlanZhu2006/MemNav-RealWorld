#!/usr/bin/env bash
set -euo pipefail

# Safe Jetson entry point for a two-pass real-world Revisit experiment.
#
# Pass 1 records an immutable, exact-JPEG survey while the robot is driven by
# the hand controller.  Pass 2 restarts the stack, replays only that frozen
# long-term memory, installs a memory-excluded goal candidate and leaves the
# robot at disabled+estop.  The formal arm is explicit and hash-bound.  This
# script never grants motor authority.

OFFBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$OFFBOARD_DIR/.." && pwd)"
source "$GO2_DIR/scripts/common.sh"

FULLMONO="$OFFBOARD_DIR/fullmono.sh"
LOCAL_PORT="${CEC_LOCAL_PORT:-18889}"
SESSION="${NAVDP_TMUX_SESSION:-navdp-go2-offboard}"
RUNTIME_ROOT="${NAVDP_REVISIT_EXPERIMENT_ROOT:-$NAVDP_ROOT/runtime/go2/two_pass_revisit}"

usage() {
  cat <<'EOF'
Usage (run on Jetson):
  revisit_experiment.sh survey-start DATASET_ID [--with-rviz]
  revisit_experiment.sh survey-status
  revisit_experiment.sh survey-return DATASET_ID
  revisit_experiment.sh survey-seal DATASET_ID
  revisit_experiment.sh formal-start DATASET_ID --arm mono_native|mono_cec [--with-rviz]
  revisit_experiment.sh formal-status
  revisit_experiment.sh stop

survey-start:
  Starts RTX + D435i + a LOCKED adapter, resets the policy and opens an
  immutable dataset.  Drive a long outbound-and-return route with the Unitree
  hand controller.  Automatic goal candidates are captured on the return and
  are excluded from memory with a causal guard.

survey-seal:
  Reasserts disabled+estop and seals the exact RGB/candidate manifest.  It
  refuses short surveys, missing candidates, altered files and exact goal/
  memory JPEG overlap.

survey-return:
  Declares the physical turnaround.  Automatic supported-goal capture is
  disabled on the outbound leg and becomes active only after this command.

formal-start:
  Safely restarts both machines, loads and verifies the sealed survey, uses
  the current camera view to initialize only NavDP's short FIFO, installs the
  selected historical goal and starts the Go2 bridge.  The required arm is
  verified from RTX health.  Motion remains LOCKED; a field operator must
  verify the selected arrival module and explicitly arm.
EOF
}

die() {
  echo "revisit-experiment: $*" >&2
  exit 1
}

validate_id() {
  [[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] \
    || die "invalid DATASET_ID: $1"
}

hub_get() {
  curl -fsS --max-time 10 "http://127.0.0.1:${LOCAL_PORT}$1"
}

hub_post_json() {
  local route="$1"
  local payload="$2"
  local timeout_s="${3:-30}"
  curl -fsS --max-time "$timeout_s" \
    -H 'Content-Type: application/json' \
    -d "$payload" \
    "http://127.0.0.1:${LOCAL_PORT}${route}"
}

wait_empty_recording_hub() {
  local deadline=$((SECONDS + 180))
  local health=""
  while (( SECONDS < deadline )); do
    health="$(hub_get /healthz 2>/dev/null || true)"
    if python3 - "$health" <<'PY' >/dev/null 2>&1
import json, sys
p = json.loads(sys.argv[1])
assert p["initialized"] is True
assert p["phase"] == "memory_recording"
assert int(p["frames_recorded"]) == 0
PY
    then
      return 0
    fi
    sleep 1
  done
  die "hub did not reach an initialized, empty memory_recording state"
}

wait_survey_dataset() {
  local dataset_id="$1"
  local deadline=$((SECONDS + 180))
  local health=""
  while (( SECONDS < deadline )); do
    health="$(hub_get /healthz 2>/dev/null || true)"
    if python3 - "$health" "$dataset_id" <<'PY' >/dev/null 2>&1
import json, sys
p = json.loads(sys.argv[1])
assert p["initialized"] is True
assert p["phase"] == "memory_recording"
ds = p["episodic_dataset"]
assert ds["recording"] is True
assert ds["dataset_id"] == sys.argv[2]
PY
    then
      return 0
    fi
    sleep 1
  done
  die "hub did not atomically open survey dataset $dataset_id"
}

force_motion_lock() {
  navdp_source_ros
  timeout 8 ros2 service call \
    /navdp_go2_adapter/set_enabled std_srvs/srv/SetBool \
    '{data: false}' >/dev/null 2>&1 || true
  timeout 8 ros2 topic pub --once /navdp/estop std_msgs/msg/Bool \
    '{data: true}' >/dev/null 2>&1 || true
}

write_receipt() {
  local path="$1"
  local payload="$2"
  mkdir -p "$(dirname "$path")"
  python3 - "$path" "$payload" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
payload = json.loads(sys.argv[2])
temporary = path.with_name("." + path.name + ".tmp")
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
temporary.replace(path)
PY
}

survey_start() {
  local dataset_id="$1"
  shift
  validate_id "$dataset_id"
  local with_rviz=false
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --with-rviz) with_rviz=true ;;
      *) die "unknown survey-start option: $1" ;;
    esac
    shift
  done
  ! tmux has-session -t "$SESSION" 2>/dev/null \
    || die "stack is already running; seal or stop it first"
  local metadata
  metadata="$(python3 - "$dataset_id" <<'PY'
import json, socket, sys
print(json.dumps({
    "dataset_id": sys.argv[1],
    "collection_mode": "manual_long_out_and_back",
    "robot": "unitree_go2",
    "collector_host": socket.gethostname(),
    "motion_authority": "unitree_hand_controller_only",
    "adapter_enabled": False,
    "candidate_contract": "memory_excluded_with_post_guard",
}))
PY
)"
  local args=(start)
  [[ "$with_rviz" == false ]] || args+=(--with-rviz)
  CEC_EPISODIC_DATASET_ID="$dataset_id" \
    CEC_EPISODIC_DATASET_METADATA_JSON="$metadata" \
    NAVDP_NAVIGATE_DURING_MEMORY_RECORDING=false \
    NAVDP_AUTO_GOAL_CANDIDATE_CAPTURE_ENABLED=false \
    NAVDP_AUTO_SELECT_GOAL_CANDIDATE=true \
    bash "$FULLMONO" "${args[@]}"
  wait_survey_dataset "$dataset_id"
  local receipt
  receipt="$(hub_get /dataset/status)"
  write_receipt "$RUNTIME_ROOT/$dataset_id/survey_start.json" "$receipt"
  force_motion_lock
  echo "$receipt" | python3 -m json.tool
  echo
  echo "Survey recording is active and motion is policy-locked."
  echo "Drive with the Unitree hand controller: outbound first, then return over"
  echo "the same region with natural 10-30 degree viewpoint differences."
  echo "At the turnaround, run: $0 survey-return $dataset_id"
  echo "Monitor: $0 survey-status"
  echo "Seal:    $0 survey-seal $dataset_id"
}

survey_return() {
  local dataset_id="$1"
  validate_id "$dataset_id"
  local before
  before="$(hub_get /dataset/status)"
  python3 - "$before" "$dataset_id" <<'PY' >/dev/null
import json, sys
p = json.loads(sys.argv[1])
assert p["recording"] is True
assert p["dataset_id"] == sys.argv[2]
PY
  navdp_source_ros
  local receipt_path="$RUNTIME_ROOT/$dataset_id/survey_return.txt"
  mkdir -p "$(dirname "$receipt_path")"
  if ! timeout 30 ros2 service call \
      /navdp_go2_adapter/set_auto_goal_candidate_capture \
      std_srvs/srv/SetBool '{data: true}' >"$receipt_path" 2>&1; then
    die "failed to arm return-leg candidate capture; see $receipt_path"
  fi
  grep -Eq 'success[=:][[:space:]]*[Tt]rue' "$receipt_path" \
    || die "adapter rejected return-leg boundary; see $receipt_path"
  echo "Return leg declared for $dataset_id."
  echo "Continue hand-controller driving; supported candidates will now be"
  echo "captured automatically and excluded from causal memory."
}

survey_status() {
  bash "$FULLMONO" status || true
  echo
  hub_get /dataset/status | python3 -m json.tool
}

survey_seal() {
  local dataset_id="$1"
  validate_id "$dataset_id"
  force_motion_lock
  sleep 1
  local receipt
  receipt="$(hub_post_json /dataset/seal '{}')"
  python3 - "$dataset_id" "$receipt" <<'PY'
import json, sys
p = json.loads(sys.argv[2])
assert p["dataset_id"] == sys.argv[1]
assert int(p["memory_frames"]) >= 1
assert int(p["goal_candidates"]) >= 1
assert int(p["goal_memory_exact_sha_overlap"]) == 0
assert p["evaluation_depth_consumed_by_policy"] is False
PY
  write_receipt "$RUNTIME_ROOT/$dataset_id/survey_seal.json" "$receipt"
  echo "$receipt" | python3 -m json.tool
  echo
  echo "Dataset is sealed.  The current in-memory session may be inspected, but"
  echo "formal-start will restart both machines and prove persistent replay."
}

formal_start() {
  local dataset_id="$1"
  shift
  validate_id "$dataset_id"
  local with_rviz=false
  local arm=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --with-rviz) with_rviz=true ;;
      --arm)
        [[ $# -ge 2 ]] || die "--arm requires mono_native or mono_cec"
        arm="$2"
        shift
        ;;
      *) die "unknown formal-start option: $1" ;;
    esac
    shift
  done
  local authority_mode
  case "$arm" in
    mono_native) authority_mode="native" ;;
    mono_cec) authority_mode="cec" ;;
    "") die "formal-start requires --arm mono_native or --arm mono_cec" ;;
    *) die "unsupported formal arm: $arm" ;;
  esac

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    force_motion_lock
  fi
  # A sealed dataset is persistent.  Always rebuild both process trees so the
  # formal pass proves load/replay instead of accidentally reusing survey RAM.
  bash "$FULLMONO" stop
  local stamp run_root
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  run_root="$RUNTIME_ROOT/$dataset_id/formal_${arm}_$stamp"
  mkdir -p "$run_root"
  local args=(start --with-go2)
  [[ "$with_rviz" == false ]] || args+=(--with-rviz)
  NAVDP_SELECTED_GOAL_IMAGE_PATH="$run_root/selected_goal.jpg" \
    NAVDP_SELECTED_GOAL_DEPTH_PATH="$run_root/selected_goal_depth.png" \
    NAVDP_NAVIGATE_DURING_MEMORY_RECORDING=false \
    NAVDP_PAUSE_MEMORY_RECORDING=true \
    NAVDP_AUTO_SELECT_GOAL_CANDIDATE=true \
    CEC_AUTHORITY_MODE="$authority_mode" \
    bash "$FULLMONO" "${args[@]}"
  wait_empty_recording_hub
  force_motion_lock

  echo "Loading and verifying survey $dataset_id; long surveys can take minutes..."
  local payload load_receipt
  payload="$(python3 - "$dataset_id" <<'PY'
import json, sys
print(json.dumps({"dataset_id": sys.argv[1]}))
PY
)"
  load_receipt="$(hub_post_json /dataset/load "$payload" 3600)"
  write_receipt "$run_root/dataset_load.json" "$load_receipt"

  navdp_source_ros
  local prepare_log="$run_root/prepare_revisit.txt"
  if ! timeout 300 ros2 service call \
      /navdp_go2_adapter/begin_revisit std_srvs/srv/Trigger '{}' \
      >"$prepare_log" 2>&1; then
    force_motion_lock
    die "dataset loaded but Revisit prepare failed; see $prepare_log"
  fi
  local health
  health="$(hub_get /healthz)"
  python3 - "$health" "$dataset_id" "$authority_mode" <<'PY'
import json, sys
p = json.loads(sys.argv[1])
assert p["phase"] == "revisit_query"
assert p["active_goal_sha256"]
assert p["cec_authority_mode"] == sys.argv[3]
ds = p["episodic_dataset"]
assert ds["loaded_dataset_id"] == sys.argv[2]
assert ds["loaded_dataset_manifest_sha256"]
PY
  [[ -s "$run_root/selected_goal.jpg" ]] \
    || die "selected goal JPEG was not installed on Jetson"
  force_motion_lock
  write_receipt "$run_root/ready_health.json" "$health"

  echo "$load_receipt" | python3 -m json.tool
  echo
  echo "Formal software stack is READY."
  echo "  run root: $run_root"
  echo "  arm:      $arm (authority_mode=$authority_mode)"
  echo "  goal:     $run_root/selected_goal.jpg"
  if [[ -s "$run_root/selected_goal_depth.png" ]]; then
    echo "  offline depth: $run_root/selected_goal_depth.png (policy/arrival authority: none)"
  else
    echo "  offline depth: missing (optional; RGB arrival does not use it)"
  fi
  echo "  motion:   LOCKED (disabled + estop)"
  echo
  echo "Do not arm until the RGB arrival/physical termination procedure"
  echo "is running and a field operator is holding the Unitree controller."
}

formal_status() {
  bash "$FULLMONO" status || true
  echo
  hub_get /healthz | python3 -m json.tool
}

stop_all() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    force_motion_lock
  fi
  bash "$FULLMONO" stop
}

action="${1:-}"
[[ $# -eq 0 ]] || shift
case "$action" in
  survey-start) [[ $# -ge 1 ]] || die "survey-start requires DATASET_ID"; survey_start "$@" ;;
  survey-status) [[ $# -eq 0 ]] || die "survey-status takes no arguments"; survey_status ;;
  survey-return) [[ $# -eq 1 ]] || die "survey-return requires DATASET_ID"; survey_return "$1" ;;
  survey-seal) [[ $# -eq 1 ]] || die "survey-seal requires DATASET_ID"; survey_seal "$1" ;;
  formal-start) [[ $# -ge 1 ]] || die "formal-start requires DATASET_ID"; formal_start "$@" ;;
  formal-status) [[ $# -eq 0 ]] || die "formal-status takes no arguments"; formal_status ;;
  stop) [[ $# -eq 0 ]] || die "stop takes no arguments"; stop_all ;;
  -h|--help|help|"") usage ;;
  *) die "unknown action: $action" ;;
esac
