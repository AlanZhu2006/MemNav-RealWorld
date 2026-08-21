# Full-Mono Runtime Runbook

This runbook is fail-closed. Completing a software deployment does not authorize
camera startup, ROS motion, or Go2 movement.

## 0. Jetson single-entry launcher

After the one-time RTX configuration in Section 1, the normal two-machine
startup is issued only from the Jetson:

~~~bash
cd /home/nvidia/twork/NavDP
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

## 1. RTX 4090 workstation

~~~bash
cd Memnav_Realworld
cp deployment/gpu/env.example deployment/gpu/.env
nano deployment/gpu/.env
~~~

Set every external source/checkpoint path and record the physically measured
D435i optical-center height:

~~~bash
export CEC_CAMERA_HEIGHT_M=<measured-metres>
bash deployment/gpu/scripts/preflight.sh
bash deployment/gpu/scripts/run_policy_stack.sh
curl -fsS http://127.0.0.1:18889/healthz
~~~

The health payload must contain:

~~~text
navigation_sensor_contract=causal_monocular_rgb_v1
navdp_depth_source=monocular_sidecar
metric_depth_sensor_consumed_by_policy=false
protocol_version=2
~~~

Inspect without exposing any service to the LAN:

~~~bash
tmux attach -t cec-realworld
tail -f runtime/gpu/logs/{memnav,navdp,hub}.log
~~~

## 2. Jetson tunnel and preflight

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
bash deployment/go2/offboard/run_offboard_stack.sh --with-rviz
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
bash deployment/go2/offboard/stop_offboard_stack.sh
bash deployment/go2/offboard/run_offboard_stack.sh --with-go2 --with-rviz
~~~

The adapter still starts disabled. With a clear area, tether, onsite operator
holding the Unitree controller and a `0.5--1.0 m` route, explicitly enable:

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
bash deployment/go2/offboard/stop_offboard_stack.sh
bash deployment/gpu/scripts/stop_policy_stack.sh
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

# 3. (Optional, recommended) capture 2-3 goal-candidate photos during the
#    walk at deliberately rotated viewpoints. Candidates are NEVER appended
#    to memory; each returns a captured_after_frame + sha256 receipt:
source /opt/ros/humble/setup.bash
ros2 service call /navdp_go2_adapter/capture_goal_candidate \
  std_srvs/srv/Trigger

# 4. Stop at B, robot stationary. Score the candidates on the RTX host
#    against the recorded stream (frozen server components only):
python deployment/gpu/score_realworld_revisit_goal.py \
  --memnav-url http://127.0.0.1:18888 \
  --rgb-dir <memnav_buffer_episode_dir> \
  --candidates <goal_candidate_jpgs...> --out /tmp/goal_score.json
# Pick a provisional_weak_covis candidate; reject near-duplicates
# (max_cos > 0.90) and unsupported ones (inliers < 16). These proxy
# thresholds are provisional until the disabled-adapter walk calibration.

# 5. Switch phase (robot MUST be stationary; the hub replays a stride-8
#    tail into NavDP and verifies the queue, and the first certificate
#    afterwards can take seconds):
ros2 service call /navdp_go2_adapter/begin_revisit std_srvs/srv/Trigger
# The response carries navdp_warmup_frames / navdp_queue_lengths receipts.

# 6. Install the selected candidate as the image goal, then goal queries
#    run normally; motion still requires the explicit enable service.
~~~

Any out-of-order call fails fast: `/memory_step` after the switch, a second
`/begin_revisit`, or a goal query during recording all return HTTP 400 with
the contract explanation, and the adapter surfaces the error instead of
retrying.

## 9. Revisit experiment boundary

A formal real-world result requires frozen starts, unchanged goal assets,
causal online history and separately reported goal-object, exact-view,
policy-stop and auxiliary-pose outcomes. Do not report SR/SPL from deployment
or transport smoke tests.
