#!/usr/bin/env bash
set -euo pipefail

# Evidence-only capture companion for a running NavDP/Go2 experiment.
# It never publishes velocity, changes estop, or calls the motion-enable service.

OFFBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_DIR="$(cd "$OFFBOARD_DIR/.." && pwd)"
source "$GO2_DIR/scripts/common.sh"

MANIFEST_TOOL="$GO2_DIR/experiment_capture_manifest.py"
TOPIC_LOGGER="$GO2_DIR/experiment_topic_logger.py"
CAPTURE_ROOT="$CFG_CAPTURE_ROOT"
CAPTURE_SESSION_PREFIX="$CFG_CAPTURE_SESSION_PREFIX"

AUDIT_TOPICS=(
  /navdp/status
  /navdp/cec_receipt
  /navdp/trajectory
  /navdp/cmd_vel
  /navdp/estop
  /navdp/enabled
  /navdp/image_goal
  /navdp/rgb_arrival_status
  /navdp/rgb_arrival_debug
  /navdp/debug/markers
  /navdp/experiment_event
  /navdp/operator/episode_event
  /navdp/operator/revisit_workflow
  /rt/sportmodestate
  /camera/camera/color/camera_info
)
FULL_SENSOR_TOPICS=(
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw
  /camera/camera/aligned_depth_to_color/camera_info
  /camera/camera/depth/metadata
)
ODIN_GT_TOPICS=(
  /navdp/gt/status
  /odin1/odometry
  /odin1/odometry_high
  /odin1/odometry_highfreq
  /odin1/path
  /odin1/cloud_slam
  /tf
  /tf_static
)

usage() {
  cat <<'EOF'
Usage (run on Jetson with Foxglove and either observer or NavDP stack running):
  experiment_capture.sh preflight
  experiment_capture.sh start RUN_ID [--dataset DATASET_ID]
      [--trial-kind revisit|novel|calibration|debug]
      [--profile audit|full] [--gt-source none|odin1]
      [--allow-observer] [--onboard-episode]
      [--calibration-scene-id ID --physical-distance-m M
       --physical-yaw-deg DEG --physical-label-method METHOD]
  experiment_capture.sh status RUN_ID
  experiment_capture.sh pause RUN_ID
  experiment_capture.sh resume RUN_ID
  experiment_capture.sh resume-survey RUN_ID
  experiment_capture.sh stop RUN_ID
  experiment_capture.sh attach-dashboard RUN_ID VIDEO
  experiment_capture.sh attach-third-view RUN_ID VIDEO
  experiment_capture.sh attach-odin-gt RUN_ID GT_RESULT SPL_RECEIPT
  experiment_capture.sh finalize RUN_ID OUTCOME [--notes TEXT]
      [--allow-incomplete]
  experiment_capture.sh verify RUN_ID

Profiles:
  audit  Records policy state, CEC receipts, trajectories, commands and RGB
         arrival output to MCAP. The RTX episodic dataset remains the causal RGB
         authority. This is the recommended formal-run profile.
  full   Adds raw D435i RGB and aligned depth to the rosbag. Use only when disk
         bandwidth and capacity have been checked; policy authority is unchanged.

--allow-observer permits the recorder to start before the policy stack exists.
It still requires live aligned RGB-D and Foxglove, then discovers policy topics
as the Episode advances. The GUI uses this for phase-gated Survey/Revisit capture.
--onboard-episode makes the internal RGB-D/receipt bundle self-contained; an
external dashboard or third-person video is optional rather than mandatory.

GT source:
  none   Preserves the current evidence contract.
  odin1  Adds the independent Odin odometry/map-TF/status lane. Finalization
         then requires attached GT result and frozen A* SPL receipts.

OUTCOME is one of: success, failure, timeout, operator_intervention,
system_failure, collision, aborted.

This tool is observational. It never enables motion and never clears estop.
EOF
}

die() {
  echo "experiment-capture: $*" >&2
  exit 1
}

validate_id() {
  [[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] \
    || die "invalid identifier: $1"
}

run_root() {
  printf '%s/%s' "$CAPTURE_ROOT" "$1"
}

session_name() {
  printf '%s-%s' "$CAPTURE_SESSION_PREFIX" "$1"
}

require_capture_commands() {
  local command
  for command in python3 tmux ros2; do
    command -v "$command" >/dev/null 2>&1 || die "missing command: $command"
  done
  [[ -f "$MANIFEST_TOOL" ]] || die "missing manifest tool: $MANIFEST_TOOL"
  [[ -f "$TOPIC_LOGGER" ]] || die "missing receipt logger: $TOPIC_LOGGER"
  navdp_source_ros
  ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1 \
    || die "rosbag2 MCAP storage plugin is missing"
}

require_live_topics() {
  local gt_source="${1:-none}"
  navdp_source_ros
  local topics
  topics="$(timeout 8 ros2 topic list 2>/dev/null || true)"
  grep -Fxq /navdp/status <<<"$topics" || die "/navdp/status is not live"
  grep -Fxq /navdp/cec_receipt <<<"$topics" || die "/navdp/cec_receipt is not live"
  if [[ "$gt_source" == odin1 ]]; then
    grep -Fxq /navdp/gt/status <<<"$topics" || die "/navdp/gt/status is not live"
    grep -Fxq /odin1/odometry <<<"$topics" || die "/odin1/odometry is not live"
  fi
  grep -Fxq /foxglove_bridge <<<"$(ros2 node list 2>/dev/null || true)" \
    || die "Foxglove Bridge is not running"
}

require_observer_topics() {
  navdp_source_ros
  local topics
  topics="$(timeout 8 ros2 topic list 2>/dev/null || true)"
  grep -Fxq /camera/camera/color/image_raw <<<"$topics" \
    || die "D435i RGB is not live"
  grep -Fxq /camera/camera/aligned_depth_to_color/image_raw <<<"$topics" \
    || die "D435i aligned depth is not live"
  grep -Fxq /foxglove_bridge <<<"$(ros2 node list 2>/dev/null || true)" \
    || die "Foxglove Bridge is not running"
}

preflight() {
  [[ $# -eq 0 ]] || die "preflight takes no options"
  require_capture_commands
  navdp_source_ros
  ros2 bag record --help >/dev/null
  echo "Experiment capture preflight passed"
  echo "  visualization: headless read-only Foxglove Bridge"
  echo "  rosbag:        MCAP storage available"
  echo "  capture root:  $CAPTURE_ROOT"
  echo "  motion change: none"
}

write_launchers() {
  local root="$1"
  shift
  local topics=("$@")
  local topic

  mkdir -p "$root/rosbag"
  {
    echo '#!/usr/bin/env bash'
    echo 'set -euo pipefail'
    echo '[[ $# -eq 1 ]] || { echo "one output path is required" >&2; exit 2; }'
    echo 'output="$1"'
    printf 'source %q\n' "$GO2_DIR/scripts/common.sh"
    echo 'navdp_source_ros'
    printf 'exec ros2 bag record --include-unpublished-topics -s mcap -o "$output"'
    for topic in "${topics[@]}"; do
      printf ' %q' "$topic"
    done
    printf '\n'
  } >"$root/receipts/launch_rosbag.sh"

  {
    echo '#!/usr/bin/env bash'
    echo 'set -euo pipefail'
    printf 'source %q\n' "$GO2_DIR/scripts/common.sh"
    echo 'navdp_source_ros'
    echo 'navdp_activate_venv'
    printf 'exec python %q --output-dir %q\n' "$TOPIC_LOGGER" "$root/logs"
  } >"$root/receipts/launch_receipts.sh"

  chmod +x "$root/receipts/launch_"*.sh
}

publish_event() {
  local run_id="$1"
  local event="$2"
  navdp_source_ros
  local utc payload
  utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  payload="{\"event\":\"$event\",\"run_id\":\"$run_id\",\"utc\":\"$utc\"}"
  timeout 8 ros2 topic pub --once \
    --qos-reliability reliable --qos-durability transient_local \
    /navdp/experiment_event \
    std_msgs/msg/String "{data: '$payload'}" >/dev/null 2>&1 || true
  printf '%s' "$payload"
}

stop_recorder_session() {
  local session="$1"
  local timeout_s="${2:-30}"
  tmux has-session -t "$session" 2>/dev/null || return 0
  local window
  for window in rosbag receipts; do
    tmux send-keys -t "$session:$window" C-c 2>/dev/null || true
  done
  local deadline=$((SECONDS + timeout_s))
  while tmux has-session -t "$session" 2>/dev/null \
      && (( SECONDS < deadline )); do
    sleep 1
  done
  if tmux has-session -t "$session" 2>/dev/null; then
    tmux kill-session -t "$session" 2>/dev/null || true
    return 1
  fi
  return 0
}

wait_rosbag_ready() {
  local session="$1"
  local log="$2"
  local timeout_s="${3:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    grep -Fq 'Recording...' "$log" 2>/dev/null && return 0
    tmux list-windows -t "$session" -F '#{window_name}' 2>/dev/null \
      | grep -Fxq rosbag || return 1
    sleep 0.2
  done
  return 1
}

start_capture() {
  local run_id="$1"
  shift
  validate_id "$run_id"
  local dataset_id=""
  local trial_kind="revisit"
  local profile="audit"
  local gt_source="none"
  local allow_observer=false
  local onboard_episode=false
  local calibration_scene_id=""
  local physical_distance_m=""
  local physical_yaw_deg=""
  local physical_label_method=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --dataset) [[ $# -ge 2 ]] || die "--dataset requires a value"; dataset_id="$2"; shift ;;
      --trial-kind) [[ $# -ge 2 ]] || die "--trial-kind requires a value"; trial_kind="$2"; shift ;;
      --profile) [[ $# -ge 2 ]] || die "--profile requires a value"; profile="$2"; shift ;;
      --gt-source) [[ $# -ge 2 ]] || die "--gt-source requires a value"; gt_source="$2"; shift ;;
      --allow-observer) allow_observer=true ;;
      --onboard-episode) onboard_episode=true ;;
      --calibration-scene-id) [[ $# -ge 2 ]] || die "--calibration-scene-id requires a value"; calibration_scene_id="$2"; shift ;;
      --physical-distance-m) [[ $# -ge 2 ]] || die "--physical-distance-m requires a value"; physical_distance_m="$2"; shift ;;
      --physical-yaw-deg) [[ $# -ge 2 ]] || die "--physical-yaw-deg requires a value"; physical_yaw_deg="$2"; shift ;;
      --physical-label-method) [[ $# -ge 2 ]] || die "--physical-label-method requires a value"; physical_label_method="$2"; shift ;;
      *) die "unknown start option: $1" ;;
    esac
    shift
  done
  [[ "$trial_kind" =~ ^(revisit|novel|calibration|debug)$ ]] \
    || die "unsupported trial kind: $trial_kind"
  [[ "$profile" =~ ^(audit|full)$ ]] || die "unsupported capture profile: $profile"
  [[ "$gt_source" =~ ^(none|odin1)$ ]] || die "unsupported GT source: $gt_source"
  [[ -z "$dataset_id" ]] || validate_id "$dataset_id"

  require_capture_commands
  if [[ "$allow_observer" == true ]]; then
    [[ "$gt_source" == none ]] \
      || die "--allow-observer does not support an Odin GT source"
    require_observer_topics
  else
    require_live_topics "$gt_source"
  fi
  local root session
  root="$(run_root "$run_id")"
  session="$(session_name "$run_id")"
  [[ ! -e "$root" ]] || die "run already exists: $root"
  ! tmux has-session -t "$session" 2>/dev/null || die "capture session already exists: $session"
  local topics=("${AUDIT_TOPICS[@]}")
  if [[ "$profile" == full ]]; then
    topics+=("${FULL_SENSOR_TOPICS[@]}")
  fi
  if [[ "$onboard_episode" == true ]]; then
    # The Episode manager already hash-preserves the frozen goal PNG under
    # media/. Do not write that same full-resolution frame at 2 Hz for both
    # phases. Keep arrival visualization evidence in the existing bounded
    # Foxglove JPEG stream instead of duplicating the raw debug image.
    local compact_topics=()
    local candidate_topic
    for candidate_topic in "${topics[@]}"; do
      case "$candidate_topic" in
        /navdp/image_goal|/navdp/rgb_arrival_debug) continue ;;
        *) compact_topics+=("$candidate_topic") ;;
      esac
    done
    topics=("${compact_topics[@]}" /navdp/foxglove/arrival/compressed)
  fi
  if [[ "$gt_source" == odin1 ]]; then
    topics+=("${ODIN_GT_TOPICS[@]}")
  fi
  local manifest_args=(
    create --run-root "$root" --run-id "$run_id"
    --dataset-id "$dataset_id" --trial-kind "$trial_kind"
    --capture-profile "$profile" --workspace "$NAVDP_ROOT"
    --gt-source "$gt_source"
  )
  if [[ "$onboard_episode" == true ]]; then
    manifest_args+=(--media-policy onboard_episode)
  fi
  if [[ "$trial_kind" == calibration ]]; then
    [[ -n "$calibration_scene_id" && -n "$physical_distance_m" \
      && -n "$physical_yaw_deg" && -n "$physical_label_method" ]] \
      || die "calibration requires scene ID, physical distance/yaw, and label method"
    manifest_args+=(
      --calibration-scene-id "$calibration_scene_id"
      --physical-distance-m "$physical_distance_m"
      --physical-yaw-deg "$physical_yaw_deg"
      --physical-label-method "$physical_label_method"
    )
  elif [[ -n "$calibration_scene_id$physical_distance_m$physical_yaw_deg$physical_label_method" ]]; then
    die "physical calibration labels require --trial-kind calibration"
  fi
  local topic
  for topic in "${topics[@]}"; do
    manifest_args+=(--topic "$topic")
  done
  python3 "$MANIFEST_TOOL" "${manifest_args[@]}" >/dev/null
  write_launchers "$root" "${topics[@]}"

  tmux new-session -d -s "$session" -n rosbag \
    "exec '$root/receipts/launch_rosbag.sh' '$root/rosbag/survey' >'$root/logs/rosbag_survey.log' 2>&1"
  tmux new-window -t "$session" -n receipts \
    "exec '$root/receipts/launch_receipts.sh' >'$root/logs/receipt_logger.log' 2>&1"
  sleep 3
  local windows
  windows="$(tmux list-windows -t "$session" -F '#{window_name}' 2>/dev/null || true)"
  for topic in rosbag receipts; do
    if ! grep -Fxq "$topic" <<<"$windows"; then
      stop_recorder_session "$session" 10 || true
      python3 "$MANIFEST_TOOL" mark-captured --run-root "$root" --clean false >/dev/null
      die "$topic recorder exited during startup; inspect $root/logs"
    fi
  done
  if ! wait_rosbag_ready "$session" "$root/logs/rosbag_survey.log" 20; then
    stop_recorder_session "$session" 10 || true
    python3 "$MANIFEST_TOOL" mark-captured --run-root "$root" --clean false >/dev/null
    die "Survey recorder did not confirm Recording state; inspect $root/logs/rosbag_survey.log"
  fi
  local event
  event="$(publish_event "$run_id" START)"
  printf '%s\n' "$event" >"$root/receipts/start_event.json"
  printf '%s\n' "$session" >"$root/receipts/tmux_session.txt"
  printf '%s\n' "$allow_observer" >"$root/receipts/started_from_observer.txt"
  echo "Experiment capture started"
  echo "  run id:       $run_id"
  echo "  run root:     $root"
  echo "  profile:      $profile"
  echo "  GT source:    $gt_source"
  echo "  dashboard:    attach a Foxglove screen recording after the run"
  echo "  motion change: none"
  echo
  echo "Start the external third-person camera now and make one visible sync clap."
  echo "Stop capture after estop/termination: $0 stop $run_id"
}

status_capture() {
  local run_id="$1"
  validate_id "$run_id"
  local root session
  root="$(run_root "$run_id")"
  session="$(session_name "$run_id")"
  python3 -m json.tool "$root/manifest.json"
  echo
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "capture_session=RUNNING ($session)"
    tmux list-windows -t "$session" -F '  #{window_name}: #{pane_current_command}'
  else
    echo "capture_session=STOPPED ($session)"
  fi
  du -sh "$root" 2>/dev/null || true
}

control_capture() {
  local run_id="$1"
  local action="$2"
  validate_id "$run_id"
  [[ "$action" =~ ^(pause|resume|resume-survey)$ ]] \
    || die "unsupported recorder action: $action"
  local root session windows event deadline segment segment_log
  root="$(run_root "$run_id")"
  session="$(session_name "$run_id")"
  [[ -f "$root/manifest.json" ]] || die "unknown run: $run_id"
  tmux has-session -t "$session" 2>/dev/null \
    || die "capture session is not running: $session"
  windows="$(tmux list-windows -t "$session" -F '#{window_name}' 2>/dev/null || true)"
  if [[ "$action" == pause ]]; then
    if ! grep -Fxq rosbag <<<"$windows"; then
      echo "Experiment Survey segment is already stopped"
      return 0
    fi
    tmux send-keys -t "$session:rosbag" C-c \
      || die "could not stop the Survey rosbag segment"
    deadline=$((SECONDS + 30))
    while (( SECONDS < deadline )); do
      windows="$(tmux list-windows -t "$session" -F '#{window_name}' 2>/dev/null || true)"
      ! grep -Fxq rosbag <<<"$windows" && break
      sleep 0.2
    done
    ! grep -Fxq rosbag <<<"$windows" \
      || die "Survey rosbag segment did not stop cleanly"
  else
    if grep -Fxq rosbag <<<"$windows"; then
      echo "Experiment Revisit segment is already recording"
      return 0
    fi
    grep -q '"event":"PAUSE"' "$root/receipts/capture_window_events.jsonl" 2>/dev/null \
      || die "cannot restart capture before the Survey segment stops"
    if [[ "$action" == resume ]]; then
      segment="revisit"
      segment_log="$root/logs/rosbag_revisit.log"
    else
      segment="survey_continued_$(date -u +%Y%m%dT%H%M%SZ)"
      segment_log="$root/logs/rosbag_${segment}.log"
    fi
    tmux new-window -t "$session" -n rosbag \
      "exec '$root/receipts/launch_rosbag.sh' '$root/rosbag/$segment' >'$segment_log' 2>&1"
    wait_rosbag_ready "$session" "$segment_log" 20 \
      || die "$action recorder did not confirm Recording state; inspect $segment_log"
  fi
  event="$(publish_event "$run_id" "${action^^}")"
  printf '%s\n' "$event" >>"$root/receipts/capture_window_events.jsonl"
  echo "Experiment capture $action accepted"
  echo "  run root: $root"
}

stop_capture() {
  local run_id="$1"
  validate_id "$run_id"
  local root session
  root="$(run_root "$run_id")"
  session="$(session_name "$run_id")"
  [[ -f "$root/manifest.json" ]] || die "unknown run: $run_id"
  tmux has-session -t "$session" 2>/dev/null || die "capture session is not running: $session"
  local event
  event="$(publish_event "$run_id" STOP)"
  printf '%s\n' "$event" >"$root/receipts/stop_event.json"
  sleep 1
  local clean=true
  if ! stop_recorder_session "$session" 30; then
    clean=false
  fi
  python3 "$MANIFEST_TOOL" mark-captured \
    --run-root "$root" --clean "$clean" >/dev/null
  echo "Experiment capture stopped"
  echo "  run root:   $root"
  echo "  clean stop: $clean"
  echo
  echo "Import the Foxglove dashboard and external video, then finalize:"
  echo "  $0 attach-dashboard $run_id /path/to/foxglove_dashboard.mp4"
  echo "  $0 attach-third-view $run_id /path/to/third_view.mp4"
  echo "  $0 finalize $run_id success --notes 'operator-confirmed outcome'"
}

attach_third_view() {
  local run_id="$1"
  local source="$2"
  validate_id "$run_id"
  python3 "$MANIFEST_TOOL" attach-video \
    --run-root "$(run_root "$run_id")" --role third_view --source "$source"
}

attach_dashboard() {
  local run_id="$1"
  local source="$2"
  validate_id "$run_id"
  python3 "$MANIFEST_TOOL" attach-video \
    --run-root "$(run_root "$run_id")" \
    --role foxglove_dashboard --source "$source"
}

attach_odin_gt() {
  local run_id="$1"
  local result="$2"
  local spl_receipt="$3"
  validate_id "$run_id"
  local root
  root="$(run_root "$run_id")"
  python3 "$MANIFEST_TOOL" attach-reference \
    --run-root "$root" --role odin_gt_result --source "$result"
  python3 "$MANIFEST_TOOL" attach-reference \
    --run-root "$root" --role odin_spl_receipt --source "$spl_receipt"
}

finalize_capture() {
  local run_id="$1"
  local outcome="$2"
  shift 2
  validate_id "$run_id"
  local notes=""
  local allow_incomplete=false
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --notes) [[ $# -ge 2 ]] || die "--notes requires text"; notes="$2"; shift ;;
      --allow-incomplete) allow_incomplete=true ;;
      *) die "unknown finalize option: $1" ;;
    esac
    shift
  done
  local args=(finalize --run-root "$(run_root "$run_id")" --outcome "$outcome" --notes "$notes")
  [[ "$allow_incomplete" == false ]] || args+=(--allow-incomplete)
  python3 "$MANIFEST_TOOL" "${args[@]}"
}

verify_capture() {
  local run_id="$1"
  validate_id "$run_id"
  python3 "$MANIFEST_TOOL" verify --run-root "$(run_root "$run_id")"
}

action="${1:-}"
[[ $# -eq 0 ]] || shift
case "$action" in
  preflight) preflight "$@" ;;
  start) [[ $# -ge 1 ]] || die "start requires RUN_ID"; run_id="$1"; shift; start_capture "$run_id" "$@" ;;
  status) [[ $# -eq 1 ]] || die "status requires RUN_ID"; status_capture "$1" ;;
  pause) [[ $# -eq 1 ]] || die "pause requires RUN_ID"; control_capture "$1" pause ;;
  resume) [[ $# -eq 1 ]] || die "resume requires RUN_ID"; control_capture "$1" resume ;;
  resume-survey) [[ $# -eq 1 ]] || die "resume-survey requires RUN_ID"; control_capture "$1" resume-survey ;;
  stop) [[ $# -eq 1 ]] || die "stop requires RUN_ID"; stop_capture "$1" ;;
  attach-dashboard) [[ $# -eq 2 ]] || die "attach-dashboard requires RUN_ID VIDEO"; attach_dashboard "$1" "$2" ;;
  attach-third-view) [[ $# -eq 2 ]] || die "attach-third-view requires RUN_ID VIDEO"; attach_third_view "$1" "$2" ;;
  attach-odin-gt) [[ $# -eq 3 ]] || die "attach-odin-gt requires RUN_ID GT_RESULT SPL_RECEIPT"; attach_odin_gt "$1" "$2" "$3" ;;
  finalize) [[ $# -ge 2 ]] || die "finalize requires RUN_ID OUTCOME"; run_id="$1"; outcome="$2"; shift 2; finalize_capture "$run_id" "$outcome" "$@" ;;
  verify) [[ $# -eq 1 ]] || die "verify requires RUN_ID"; verify_capture "$1" ;;
  -h|--help|help|"") usage ;;
  *) die "unknown action: $action" ;;
esac
