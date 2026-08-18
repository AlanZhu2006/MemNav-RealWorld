# Audited Architecture

## System Boundary

This repository implements a two-computer real-world ImageGoal stack:

- the RTX 4090 workstation owns memory retrieval, geometric certification and policy inference;
- the Jetson Orin NX owns sensors, ROS state, path tracking and every interface that can move the Go2.

It is not a metric global-navigation system. No component in the current
deployment produces a persistent <code>map -> odom -> base_link</code>
transform. MemNav contributes a verified camera-relative revisit direction;
NavDP emits a 24-point robot-local trajectory from current RGB-D and goal
conditioning.

## Data Plane

~~~text
Unitree Go2 / Jetson Orin NX                     RTX 4090 workstation

D435i aligned RGB-D
        │
        ├── ROS adapter ── current RGB + depth + goal ──┐
        │                                                │ SSH local forward
        │                                                ▼
        │                                      Unified CEC hub :18889
        │                                      ├─ MemNav :18888
        │                                      └─ NavDP  :8888
        │                                                │
        └── local tracker ◀──── 24-point trajectory ─────┘
                 │
        depth / age / estop guards
                 │
          /navdp/cmd_vel
                 │
       0.35 s watchdog + remote priority
                 │
         Unitree SportClient.Move()
~~~

The HTTP services bind only to <code>127.0.0.1</code> on the workstation. The
Jetson uses a local SSH forward, so the ROS adapter still addresses a loopback
URL.

## Stateful CEC Route

### Reset

<code>POST /navigator_reset</code> resets both upstream stateful services. The
hub is initialized only if both resets succeed. A partial or ambiguous reset
returns <code>503</code> and the client must retry a full reset.

### ImageGoal Step

For each <code>POST /imagegoal_step</code>:

1. <code>/retrieval_probe_step</code> appends the current observation exactly once and returns causal candidates.
2. <code>/certified_relocalize</code> verifies candidates using the frozen geometry-first route.
3. A certificate is eligible only when <code>ok=true</code>, <code>accepted=true</code> and its units are the scale-free LingBot direction contract.
4. The direction is normalized and projected onto a frozen <code>2.5 m</code> controller radius.
5. Eligible evidence calls NavDP <code>/navdp_step_ip_mixgoal</code>.
6. Rejected evidence calls the native NavDP <code>/imagegoal_step</code> route without changing the image goal.

This separation prevents an uncalibrated memory-vector norm from being
misrepresented as metric distance.

## Failure Semantics

| Failure | Behavior |
| --- | --- |
| MemNav probe fails | Native NavDP handles the current step; memory remains degraded until reset |
| Certificate fails or rejects | Exact native ImageGoal fallback |
| Native or mixed NavDP request is ambiguous | Latch <code>reset_required</code>; do not continue statefully |
| Concurrent hub request | Reject with HTTP <code>409</code> |
| SSH tunnel breaks | Jetson plan expires, command becomes zero |
| RGB-D pair is stale / skewed | Jetson command becomes zero |
| Depth ROI is invalid or blocked | Jetson command becomes zero or slows conservatively |
| Go2 command stream is stale | Bridge sends zero and calls <code>StopMove()</code> |
| Hand controller becomes active | Autonomous SportClient authority is released |

The hub never silently retries a state-mutating inference request because a
timeout cannot prove whether the upstream policy already advanced its memory.

## Robot-Local Control

The trajectory tracker uses robot-plane coordinates <code>x</code> forward and
<code>y</code> left. It selects a geometric lookahead point and produces
forward velocity and yaw rate; lateral velocity and reverse motion are
disabled by default.

The default linear limit is <code>0.30 m/s</code>. The Go2 bridge preserves the
observed hardware motion floors <code>0.10 m/s</code> translation and
<code>0.20 rad/s</code> rotation so small commands do not merely change leg
posture without translating.

Two separate watchdogs exist:

- the ROS adapter rejects sensor, goal and trajectory age violations;
- the Unitree bridge independently rejects a command older than <code>0.35 s</code>.

## Debug and Evaluation Boundary

RViz shows the current local trajectory, top policy candidates, aligned depth,
goal image, clearance, commands and stop reason. Its identity
<code>navdp_local -> base_link</code> transform is visualization-only and is
not odometry.

The ImageGoal evaluator may read <code>SportModeState</code> for stationary
gating and auxiliary path metrics. That state is never sent into NavDP.
Goal-object, exact-view, policy-stop and auxiliary-pose success remain separate
fields; no single motor-odometry threshold is treated as visual goal
recognition.

## Authority Boundary

The workstation is advisory:

- no Unitree SDK dependency;
- no ROS velocity publisher;
- no direct actuator network route.

The Jetson is authoritative:

- explicit enable and estop;
- live perception checks;
- trajectory freshness;
- local collision guard;
- operator takeover;
- final velocity publication and watchdog.
