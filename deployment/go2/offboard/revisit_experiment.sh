#!/usr/bin/env bash
set -euo pipefail

# Safe Jetson entry point for a two-pass real-world Revisit experiment.
#
# Pass 1 records an immutable, exact-JPEG survey while the robot is driven by
# the hand controller.  Pass 2 restarts the stack, replays only that frozen
# long-term memory, installs an exact preregistered goal and leaves the robot
# at disabled+estop. The formal arm is explicit and hash-bound. This script
# never grants motor authority.

OFFBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$OFFBOARD_DIR/.." && pwd)"
source "$GO2_DIR/scripts/common.sh"

FULLMONO="$OFFBOARD_DIR/fullmono.sh"
CONFIG_TOOL="$NAVDP_ROOT/deployment/runtime_config.py"
BASE_EXPERIMENT="$NAVDP_ROOT/deployment/config/experiments/fullmono_imagegoal.json"
if [[ "${1:-}" == --config ]]; then
  [[ $# -ge 2 ]] || { echo "revisit-experiment: --config requires a value" >&2; exit 2; }
  BASE_EXPERIMENT="$2"
  shift 2
fi
BASE_RESOLVED="$(python3 "$CONFIG_TOOL" resolve --config "$BASE_EXPERIMENT")"
python3 "$CONFIG_TOOL" verify --config "$BASE_RESOLVED" --site jetson >/dev/null
BASE_CONFIG_EXPORTS="$(python3 "$CONFIG_TOOL" shell --config "$BASE_RESOLVED" --site jetson)"
eval "$BASE_CONFIG_EXPORTS"
[[ "$CFG_PROFILE" == fullmono-lingbot-cec ]] || {
  echo "revisit-experiment: config must select fullmono-lingbot-cec" >&2
  exit 2
}
LOCAL_PORT="$CFG_TUNNEL_LOCAL_PORT"
SESSION="$CFG_FULLMONO_SESSION"
RUNTIME_ROOT="$CFG_JETSON_RUNTIME_ROOT/two_pass_revisit"

usage() {
  cat <<'EOF'
Usage (run on Jetson):
  revisit_experiment.sh [--config EXPERIMENT.json] survey-start DATASET_ID
  revisit_experiment.sh survey-status
  revisit_experiment.sh survey-return DATASET_ID
  revisit_experiment.sh survey-seal DATASET_ID
  revisit_experiment.sh [--config EXPERIMENT.json] formal-start DATASET_ID
      --scene-id SCENE_ID --run-id RUN_ID
      --arm mono_native|mono_cec --goal FROZEN_GOAL_JPEG
      --expected-goal-sha256 SHA256 --expected-dataset-sha256 SHA256
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
  exact preregistered external goal and starts the Go2 bridge. Goal and dataset
  SHA-256 plus the required authority arm are verified from RTX health.
  No Novel/Revisit label is passed to runtime. Motion remains LOCKED.
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

validate_sha256() {
  [[ "$1" =~ ^[0-9a-f]{64}$ ]] \
    || die "expected a lowercase SHA-256, got: $1"
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

active_config() {
  local candidate=""
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    candidate="$(tmux show-environment -t "$SESSION" MEMNAV_RUN_CONFIG 2>/dev/null \
      | sed -n 's/^MEMNAV_RUN_CONFIG=//p')"
  fi
  if [[ -n "$candidate" && -f "$candidate" ]]; then
    printf '%s\n' "$candidate"
  else
    printf '%s\n' "$BASE_RESOLVED"
  fi
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
  [[ $# -eq 0 ]] || die "survey-start accepts only DATASET_ID; Foxglove belongs in config"
  ! tmux has-session -t "$SESSION" 2>/dev/null \
    || die "stack is already running; seal or stop it first"
  local config_path="$RUNTIME_ROOT/$dataset_id/survey_config.json"
  mkdir -p "$(dirname "$config_path")"
  python3 "$CONFIG_TOOL" derive-survey \
    --config "$BASE_RESOLVED" --dataset-id "$dataset_id" \
    --output "$config_path" >/dev/null
  bash "$FULLMONO" start --config "$config_path"
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
  bash "$FULLMONO" status --config "$(active_config)" || true
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
  local arm="" scene_id="" run_id="" frozen_goal=""
  local expected_goal_sha256="" expected_dataset_sha256=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --arm|--scene-id|--run-id|--goal|--expected-goal-sha256|--expected-dataset-sha256)
        [[ $# -ge 2 ]] || die "$1 requires a value"
        case "$1" in
          --arm) arm="$2" ;;
          --scene-id) scene_id="$2" ;;
          --run-id) run_id="$2" ;;
          --goal) frozen_goal="$2" ;;
          --expected-goal-sha256) expected_goal_sha256="$2" ;;
          --expected-dataset-sha256) expected_dataset_sha256="$2" ;;
        esac
        shift 2
        ;;
      *) die "unknown formal-start option: $1; Foxglove belongs in config" ;;
    esac
  done
  local authority_mode
  case "$arm" in
    mono_native) authority_mode="native" ;;
    mono_cec) authority_mode="cec" ;;
    "") die "formal-start requires --arm mono_native or --arm mono_cec" ;;
    *) die "unsupported formal arm: $arm" ;;
  esac
  [[ -n "$scene_id" ]] || die "formal-start requires --scene-id"
  [[ -n "$run_id" ]] || die "formal-start requires --run-id"
  [[ -n "$frozen_goal" ]] || die "formal-start requires --goal"
  [[ -n "$expected_goal_sha256" ]] || die "formal-start requires --expected-goal-sha256"
  [[ -n "$expected_dataset_sha256" ]] || die "formal-start requires --expected-dataset-sha256"
  validate_id "$scene_id"
  validate_id "$run_id"
  validate_sha256 "$expected_goal_sha256"
  validate_sha256 "$expected_dataset_sha256"
  [[ -f "$frozen_goal" ]] || die "frozen goal does not exist: $frozen_goal"
  frozen_goal="$(readlink -f "$frozen_goal")"
  local actual_goal_sha256
  actual_goal_sha256="$(sha256sum "$frozen_goal" | awk '{print $1}')"
  [[ "$actual_goal_sha256" == "$expected_goal_sha256" ]] \
    || die "frozen goal SHA mismatch: expected $expected_goal_sha256, got $actual_goal_sha256"
  local run_root="$RUNTIME_ROOT/$dataset_id/$run_id"
  [[ ! -e "$run_root" ]] || die "formal run root already exists: $run_root"

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    force_motion_lock
  fi
  # A sealed dataset is persistent.  Always rebuild both process trees so the
  # formal pass proves load/replay instead of accidentally reusing survey RAM.
  bash "$FULLMONO" stop --config "$(active_config)"
  mkdir -p "$run_root"
  local config_path="$run_root/formal_config.json"
  python3 "$CONFIG_TOOL" derive-formal \
    --config "$BASE_RESOLVED" --dataset-id "$dataset_id" \
    --run-root "$run_root" --scene-id "$scene_id" --run-id "$run_id" \
    --authority-mode "$authority_mode" --frozen-goal "$frozen_goal" \
    --expected-goal-sha256 "$expected_goal_sha256" \
    --expected-dataset-sha256 "$expected_dataset_sha256" \
    --output "$config_path" >/dev/null
  local formal_config_id
  formal_config_id="$(python3 "$CONFIG_TOOL" get --config "$config_path" config_id)"
  bash "$FULLMONO" start --config "$config_path"
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
  python3 - "$health" "$dataset_id" "$authority_mode" \
      "$expected_goal_sha256" "$expected_dataset_sha256" <<'PY'
import json, sys
p = json.loads(sys.argv[1])
assert p["phase"] == "revisit_query"
assert p["active_goal_sha256"] == sys.argv[4]
assert p["cec_authority_mode"] == sys.argv[3]
ds = p["episodic_dataset"]
assert ds["loaded_dataset_id"] == sys.argv[2]
assert ds["loaded_dataset_manifest_sha256"] == sys.argv[5]
prepare = p["last_prepare_receipt"]
assert prepare["goal_selection_contract"] == "operator_frozen_external_v1"
assert prepare["selected_goal"]["goal_source"] == "operator_frozen_external"
assert prepare["selected_goal"]["sha256"] == sys.argv[4]
PY
  [[ -s "$run_root/selected_goal.jpg" ]] \
    || die "selected goal JPEG was not installed on Jetson"
  local installed_goal_sha256
  installed_goal_sha256="$(sha256sum "$run_root/selected_goal.jpg" | awk '{print $1}')"
  [[ "$installed_goal_sha256" == "$expected_goal_sha256" ]] \
    || die "installed goal SHA differs from the frozen registry goal"
  force_motion_lock
  write_receipt "$run_root/ready_health.json" "$health"
  local formal_ready
  formal_ready="$(python3 - "$scene_id" "$run_id" "$dataset_id" "$arm" \
      "$authority_mode" "$frozen_goal" "$expected_goal_sha256" \
      "$expected_dataset_sha256" "$health" "$formal_config_id" <<'PY'
from datetime import datetime, timezone
import json, sys
health = json.loads(sys.argv[9])
print(json.dumps({
    "schema": "memnav_realworld_formal_ready_v1_20260830",
    "created_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    "scene_id": sys.argv[1], "run_id": sys.argv[2], "dataset_id": sys.argv[3],
    "arm": sys.argv[4], "cec_authority_mode": sys.argv[5],
    "runtime_role_visibility": "none", "frozen_goal_path": sys.argv[6],
    "goal_sha256": sys.argv[7], "dataset_manifest_sha256": sys.argv[8],
    "goal_selection_contract": "operator_frozen_external_v1",
    "motion_enabled": False, "estop_required": True,
    "active_goal_sha256": health["active_goal_sha256"],
    "loaded_dataset_id": health["episodic_dataset"]["loaded_dataset_id"],
    "loaded_dataset_manifest_sha256": health["episodic_dataset"]["loaded_dataset_manifest_sha256"],
    "resolved_config_id": sys.argv[10],
}, sort_keys=True))
PY
)"
  write_receipt "$run_root/formal_ready.json" "$formal_ready"

  echo "$load_receipt" | python3 -m json.tool
  echo
  echo "Formal software stack is READY."
  echo "  run root: $run_root"
  echo "  run id:   $run_id"
  echo "  scene:    $scene_id (role hidden from runtime)"
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
  bash "$FULLMONO" status --config "$(active_config)" || true
  echo
  hub_get /healthz | python3 -m json.tool
}

stop_all() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    force_motion_lock
  fi
  bash "$FULLMONO" stop --config "$(active_config)"
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
