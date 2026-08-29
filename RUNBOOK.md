# Full-Mono Runtime Runbook

> 两次独立运行的长程 Revisit 数据集与一键 formal-ready 流程见
> `TWO_PASS_REVISIT_RUNBOOK_20260825.md`。新正式实验优先使用该流程；本文件后面的
> 单次在线 recording -> revisit 命令只用于机制调试，不是第二套正式流程。
> 每轮 ROS bag、CEC 收据、RViz dashboard 与第三人称视频采集见
> `EXPERIMENT_DATA_COLLECTION.md`。

This runbook is fail-closed. Completing a software deployment does not authorize
camera startup, ROS motion, or Go2 movement.

## 0. Jetson single-entry launcher

After the one-time RTX configuration in Section 1, the direct Full-Mono
lifecycle command is issued from the Jetson:

~~~bash
cd /home/nvidia/twork/NavDP
export NAVDP_GOAL=/absolute/path/to/image_goal.png
export CEC_CAMERA_HEIGHT_M=0.42
export NAVDP_IMAGE_GOAL_PATH="$NAVDP_GOAL"

bash deployment/go2/offboard/fullmono.sh start
~~~

This starts or reuses the RTX policy services, verifies the Full-Mono health
contract, creates the SSH tunnel, and starts the D435i plus the disabled ROS
adapter.  Startup succeeds only after the D435i publishes a real CameraInfo
message; a missing or disconnected camera rolls back both machines instead of
leaving a false-ready session.  It does not start the Go2 bridge and cannot
move the robot.

Useful variants are:

~~~bash
# Camera, locked adapter and RViz; still no Go2 bridge.
bash deployment/go2/offboard/fullmono.sh start --with-rviz

# Also start the watchdog Go2 bridge; the adapter still remains disabled.
bash deployment/go2/offboard/fullmono.sh start --with-go2 --with-rviz

bash deployment/go2/offboard/fullmono.sh status
bash deployment/go2/offboard/fullmono.sh stop
~~~

`--with-go2` is deliberately not an arming command.  Motion still requires the
separate onsite `SetBool` call in Section 6.  The RTX host and repository default
to `work-pc` and `/home/asus/Research/Memnav_Realworld`; override them with
`CEC_HUB_SSH_HOST` and `CEC_GPU_REPO` if the site layout changes.

`nav_stack.sh --profile fullmono-lingbot-cec` is the equivalent profile-oriented
facade: it validates explicit goal/arrival parameters and then calls
`fullmono.sh`. `run_offboard_stack.sh` is the Jetson-local inner launcher and
should not be called as a competing end-to-end workflow.

## 1. RTX 4090 workstation

~~~bash
cd Memnav_Realworld
cp deployment/gpu/env.example deployment/gpu/.env
nano deployment/gpu/.env
~~~

Set every external source/checkpoint path and the D435i optical-center
height (operator-confirmed 0.42 m on 2026-08-21; the explicit export is a
configuration gate, not an open measurement question):

~~~bash
export CEC_CAMERA_HEIGHT_M=0.42
bash deployment/gpu/scripts/preflight.sh
bash deployment/gpu/scripts/run_policy_stack.sh
curl -fsS http://127.0.0.1:18889/healthz
~~~

The health payload must contain:

~~~text
navigation_sensor_contract=causal_monocular_rgb_v1
navdp_depth_source=monocular_sidecar
metric_depth_sensor_consumed_by_policy=false
protocol_version=3
~~~

Inspect without exposing any service to the LAN:

~~~bash
tmux attach -t cec-realworld
tail -f runtime/gpu/logs/{memnav,navdp,hub}.log
~~~

## 2. Jetson tunnel and preflight diagnostics

The canonical Full-Mono start above creates and verifies this tunnel
automatically. Run the commands below only when diagnosing SSH transport while
the navigation stack is stopped:

~~~bash
cd /home/nvidia/twork/NavDP
export CEC_HUB_SSH_HOST=work-pc
tmux new -s cec-tunnel 'exec deployment/go2/offboard/run_policy_tunnel.sh'
~~~

In another Jetson terminal:

~~~bash
curl -fsS http://127.0.0.1:18889/healthz
bash deployment/go2/offboard/preflight_offboard.sh
~~~

All five preflight checks must pass. The contract parser rejects an old RGB-D
hub even if its port and `algo` field look healthy.

## 3. Goal capture

Keep navigation disabled. Move the Go2 with the hand controller and capture the
goal while stationary:

~~~bash
bash deployment/go2/scripts/run_realsense.sh
bash deployment/go2/scripts/capture_imagegoal_reference.sh
~~~

The aligned goal depth may be retained for offline arrival auditing, but it is
not a policy input.

## 4. Camera-only static acceptance

Do not start the Go2 bridge:

~~~bash
bash deployment/go2/offboard/fullmono.sh start --with-rviz
tmux attach -t navdp-go2-offboard
~~~

Run for at least ten minutes and verify:

- adapter state remains `enabled=false`;
- `/navdp/cmd_vel` remains zero;
- current RGB and local safety depth remain fresh;
- frames 0--39 carry `bootstrap_zero_depth`;
- the frame-40 transition freezes one scale receipt exactly once;
- every trajectory says `metric_depth_sensor_consumed=false`;
- the image SHA in the mono-depth receipt matches the current policy image;
- left/right certified bearings have the expected robot-frame sign.

## 5. Fault injection while motion is disabled

Test both failures independently:

1. stop the SSH tunnel;
2. stop the MemNav/LingBot service.

Each failure must expire the plan, output zero velocity and require a fresh
reset. A causal stream failure must never activate metric-depth NavDP.

## 6. First tethered motion

Only after all static gates pass:

~~~bash
bash deployment/go2/offboard/fullmono.sh stop
bash deployment/go2/offboard/fullmono.sh start --with-go2 --with-rviz
~~~

The adapter still starts disabled. With a clear area, tether, onsite operator
holding the Unitree controller and a `0.5--1.0 m` route, explicitly enable:

- the default `NAVDP_CONTROL_PROFILE=formal` rejects stale low-speed overrides;
- formal motion uses controller `max_linear_mps=0.30`, `max_angular_rps=0.55`
  and an `8 deg` heading-error deadband;
- the Go2 bridge retains `min_cmd_v=0.10` and `min_cmd_w=0.20` after that
  controller deadband;
- a bounded commissioning smoke must opt in with
  `NAVDP_CONTROL_PROFILE=acceptance`; it is not a formal episode.

~~~bash
source /opt/ros/humble/setup.bash
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
~~~

## 7. Immediate stop

~~~bash
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: true}"
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
bash deployment/go2/offboard/fullmono.sh stop
~~~

## 8. Protocol-v3 revisit mission flow

The hub enforces a two-phase episode contract (protocol v3). The first goal
query freezes the MemNav goal session and its candidate ceiling, so it must
happen only at the revisit start point. Issuing the goal from frame 0 was the
root cause of the 2026-08-21 `no_causal_candidate` field failure: the entire
recorded walk was excluded from Revisit candidacy and the robot silently
degraded to plain ImageGoal exploration.

The offboard stack starts the adapter with `NAVDP_TWO_PHASE=true`, so after
`reset` the adapter posts every frame to `/memory_step` (record-only, no goal)
and the hub rejects any goal query with HTTP 400 until the phase switch.

End-to-end mission:

~~~bash
# 1. Start the stack (adapter locked, recording phase after reset).
bash deployment/go2/offboard/fullmono.sh start --with-rviz

# 2. Recording walk: drive A -> B with the hand controller. The adapter
#    streams /memory_step automatically; watch frames_recorded grow:
ros2 topic echo --once /navdp/status   # "phase":"memory_recording"

# 3. Candidate-only frames are sampled automatically every 24 recorded
#    frames. The RTX read-only support query accepts only geometrically
#    supported, non-near-duplicate views. Watch accepted/rejected receipts:
ros2 topic echo /navdp/cec_receipt

# Optional controlled override: explicitly capture an unfiltered candidate.
ros2 service call /navdp_go2_adapter/capture_goal_candidate std_srvs/srv/Trigger

# 4. Stop at B and invoke the ONE explicit task-boundary transition. This
#    atomically scores all registered candidates, selects and installs the
#    target, warms NavDP, verifies its FIFO, and switches phase:
ros2 service call /navdp_go2_adapter/begin_revisit std_srvs/srv/Trigger
# Retry the same call after an ambiguous network response: prepare_revisit is
# idempotent and returns the already-committed goal without a second warm-up.

# 5. Verify the persistent receipt and the runtime target before enabling:
ros2 topic echo --once /navdp/status
# Require phase=revisit_query, active_goal_sha256, selected_goal,
# navdp_warmup_frames and navdp_queue_lengths. /navdp/image_goal now displays
# the selected online target. Goal queries then run automatically; motion
# still requires the explicit enable service.
~~~

Any out-of-order call fails fast: `/memory_step` after the switch or a goal
query during recording returns HTTP 400. A repeated `/prepare_revisit` is the
single exception: it is an idempotent recovery read for a possibly lost commit
response. If no candidate passes the frozen support band, phase remains
`memory_recording` and no NavDP warm-up or goal installation occurs.

## 9. Revisit experiment boundary

A formal real-world result requires frozen starts, unchanged goal assets,
causal online history and separately reported goal-object, exact-view,
policy-stop and auxiliary-pose outcomes. Do not report SR/SPL from deployment
or transport smoke tests.

The launcher allocates a new timestamped MemNav buffer namespace on every
service start, so restarting the process cannot erase the preceding RGB trace.
Set `CEC_BUFFER_ROOT` only when a formal run has already reserved an immutable,
empty destination.

## 10. Direct-bearing handoff gate

Before any tethered Revisit run, keep motion disabled and wait for one complete
terminal receipt:

~~~bash
ros2 topic echo --once /navdp/cec_receipt
ros2 topic echo --once /navdp/status
~~~

Require `last_error=""`, a committed goal SHA, schema
`cec_direct_bearing_handoff_v2_20260824`, and one of the audited bearing
dispositions. A rearward direct proof should report `terminal_atomic_turn`,
`terminal_proof_active=true`, `terminal_local_latched=false`,
`terminal_metric_scale_control_authority=false`, and
`terminal_stop_authorized=false`. While disabled, `cmd_vx=cmd_wz=0` and the
shadow override must have zero translation. The first uncached CEC anchor may
take about 20 seconds; never enable while `inference_busy=true` or before a
fresh receipt.

Adapter restart now asserts estop by default. Motion requires two explicit
onsite operations, in this order:

~~~bash
# Only with clear floor, tether and controller operator present:
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: false}"
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
~~~

The direct-bearing path is not an arrival endpoint. `/local_pose_query` may
include an MDTEC-scaled distance, but v2 uses only its certified direction;
that distance may not authorize local metric control or STOP. GOAT keeps the
separate `/arrival_query` strict-first-64 research contract. An opt-in RGB-only
commissioning gate now has one powered near-goal latch, but it has not passed
cross-scene calibration or repeated full-route acceptance. Formal runs still
require the pre-registered independent termination contract and must not turn
that commissioning result into a general autonomous ImageGoal claim.
