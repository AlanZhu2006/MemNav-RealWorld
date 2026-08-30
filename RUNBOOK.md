# Full-Mono Runtime Runbook

This runbook is fail-closed. Software readiness never authorizes Go2 motion.
The onsite operator retains the Unitree hand controller throughout powered
tests.

## 1. Configuration

There is one tracked configuration path:

- `deployment/config/system.json`: Jetson/RTX paths, models, ports, camera,
  `0.42 m` measured optical-center height and safety limits;
- `deployment/config/experiments/fullmono_imagegoal.json`: ImageGoal, arrival
  module and optional camera/Go2/RViz processes.

Do not create `deployment/gpu/.env` or export `CEC_*`/`NAVDP_*` overrides. If a
machine path changes, edit `system.json`, review the diff, commit it and pull
the same revision on both machines.

## 2. Goal capture

With navigation stopped and the camera publishing, use the hand controller to
place the stationary robot at the goal:

```bash
bash deployment/go2/scripts/capture_image_goal.sh \
  --output deployment/go2/goals/image_goal.png
```

Set this path in both `experiment.navigation.image_goal` and, when intended,
`experiment.arrival.image_goal`. The resolver records the final absolute path,
dimensions and SHA-256.

## 3. Resolve without starting

```bash
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/fullmono_imagegoal.json \
  --dry-run
```

Verify the printed profile, ImageGoal SHA, arrival, launch flags, source
revision and `config_id`.

## 4. Start the dual-machine stack

```bash
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/fullmono_imagegoal.json
```

The launcher performs these transactions:

1. resolve and verify one immutable JSON on Jetson;
2. copy the same file to RTX `runtime/config/`;
3. require the RTX Git revision and model paths to match;
4. preflight/start loopback MemNav, NavDP and CEC services;
5. create the SSH tunnel and require a valid health contract;
6. start D435i and require real CameraInfo before adapter startup;
7. start selected optional processes while retaining disabled + estop.

Any partial start is rolled back. GPU services never expose an actuator path.

## 5. Survey and Formal Revisit

```bash
bash deployment/go2/offboard/revisit_experiment.sh survey-start DATASET_ID
# Drive outbound with the hand controller, then at the turnaround:
bash deployment/go2/offboard/revisit_experiment.sh survey-return DATASET_ID
# Finish the return and stop physically:
bash deployment/go2/offboard/revisit_experiment.sh survey-seal DATASET_ID
bash deployment/go2/offboard/revisit_experiment.sh formal-start DATASET_ID \
  --scene-id SCENE_ID --run-id RUN_ID --arm mono_cec \
  --goal /absolute/path/to/frozen_goal.jpg \
  --expected-goal-sha256 "$GOAL_SHA256" \
  --expected-dataset-sha256 "$DATASET_SHA256"
```

The lifecycle derives separate hashed survey/formal configs from the base
experiment. Dataset metadata, candidate capture, selected goal output and
Go2-bridge selection are fields in those derived receipts—not environment
variables.

## 6. Motion authorization

Only after visual and safety checks:

```bash
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: false}"
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

Immediate stop:

```bash
ros2 topic pub --once /navdp/estop std_msgs/msg/Bool "{data: true}"
ros2 service call /navdp_go2_adapter/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
bash deployment/go2/nav_stack.sh stop
```

## 7. Status and evidence

```bash
bash deployment/go2/nav_stack.sh status
tmux attach -t navdp-go2-offboard
bash deployment/go2/offboard/experiment_capture.sh preflight
```

The exact ROS bag, CEC receipt, dashboard, third-view and manifest workflow is
in `EXPERIMENT_DATA_COLLECTION.md`. Formal scene registration and scoring are
defined in `REALWORLD_EXPERIMENT_HANDBOOK_CN.md` and
`REALWORLD_EVALUATION.md`.
