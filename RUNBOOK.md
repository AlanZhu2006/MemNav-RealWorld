# Runtime Runbook

## 1. GPU Workstation

~~~bash
cd Memnav_Realworld
cp deployment/gpu/env.example deployment/gpu/.env
nano deployment/gpu/.env
bash deployment/gpu/scripts/preflight.sh
bash deployment/gpu/scripts/run_policy_stack.sh
curl -fsS http://127.0.0.1:18889/healthz
~~~

Inspect logs with:

~~~bash
tmux attach -t cec-realworld
tail -f runtime/gpu/logs/{memnav,navdp,hub}.log
~~~

## 2. Jetson Tunnel

~~~bash
cd Memnav_Realworld
export CEC_HUB_SSH_HOST=user@gpu-workstation
bash deployment/go2/offboard/run_policy_tunnel.sh
~~~

In another terminal:

~~~bash
curl -fsS http://127.0.0.1:18889/healthz
bash deployment/go2/offboard/preflight_offboard.sh
~~~

## 3. Goal Capture

Keep navigation stopped, move the Go2 to the target with the hand controller
and keep the platform stationary:

~~~bash
bash deployment/go2/scripts/run_realsense.sh
bash deployment/go2/scripts/capture_imagegoal_reference.sh
~~~

Return to the start without rebooting the Go2.

## 4. Disabled Dry-Run

~~~bash
bash deployment/go2/offboard/run_offboard_stack.sh --with-rviz
tmux attach -t navdp-go2-offboard
~~~

Required observations:

- current RGB and aligned depth are live;
- <code>/navdp/status</code> points to the loopback CEC hub;
- selected and candidate paths are finite and visually plausible;
- <code>enabled=false</code> and <code>/navdp/cmd_vel</code> stays zero;
- hub logs show native fallback or a certified takeover reason.

## 5. Supervised Motion

~~~bash
bash deployment/go2/offboard/stop_offboard_stack.sh
bash deployment/go2/offboard/run_offboard_stack.sh --with-go2 --with-rviz
source /opt/ros/humble/setup.bash
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
~~~

An onsite operator must hold the Unitree controller throughout.

## 6. Emergency Stop

~~~bash
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: true}"
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
~~~

## 7. Revisit Episode

At first arrival, leave the same goal files unchanged:

~~~bash
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /navdp_go2_adapter/reset_policy std_srvs/srv/Trigger "{}"
bash deployment/go2/scripts/run_imagegoal_evaluator.sh run \
  --episode revisit --arrival-mode object --auto-estop
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: false}"
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
~~~

The main protocol requires independent <code>first</code> and
<code>revisit</code> episodes and reports goal-object, exact-view, policy-stop
and pose metrics separately.

## 8. Shutdown

~~~bash
bash deployment/go2/offboard/stop_offboard_stack.sh
bash deployment/gpu/scripts/stop_policy_stack.sh
~~~
