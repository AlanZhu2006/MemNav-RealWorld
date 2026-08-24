# Audited Full-Mono Architecture

Snapshot: **2026-08-24**

## System boundary

This repository implements a two-computer real-world ImageGoal stack:

- the RTX 4090 workstation owns one causal RGB stream, episodic retrieval,
  geometric certification and frozen policy inference;
- the Jetson Orin NX owns camera transport, local collision sensing, trajectory
  tracking and every interface that can move the Unitree Go2.

The navigation policy is monocular. D435i aligned depth remains on the Jetson
as an independent collision-safety signal and is never consumed by CEC,
bearing generation or NavDP policy inference. The honest system description is
**monocular navigation with a local depth safety layer**, not a sensorless
robot.

## One stream, two time scales, one policy

~~~text
Unitree Go2 / Jetson Orin NX                  RTX 4090 workstation

D435i RGB ───────── SSH local forward ──────> protocol-v3 CEC hub
                                                    |
                                         exactly one RGB append
                                                    |
                                        frozen LingBot state
                                     /                        \
                         dense mono-depth readout       sparse CEC proof
                         first-40 scale receipt         history retrieval
                                     \                        /
                                      frozen NavDP controller
                                  ImageGoal or Image+PointGoal
                                                    |
Jetson local tracker <──── 24-point local path ─────┘
  + aligned-depth collision stop
  + stale-plan guard + estop
  + 0.35 s Go2 watchdog
  + hand-controller priority
~~~

LingBot does not compete with NavDP for control. It supplies two readouts from
the same causal RGB state:

1. a dense short-range depth observation for NavDP's unchanged RGB-D encoder;
2. sparse long-range evidence used by CEC to prove a revisit direction.

NavDP remains the only component that generates trajectories.

## Stateful protocol-v3 route

### Two-phase episode contract (v3)

Protocol v3 splits every episode into two server-enforced phases. The reason
is causal: MemNav freezes a goal session at the FIRST query for that goal
image (`goal_start_frame`, candidate ceiling), so a goal issued from frame 0
excludes the entire subsequent walk from Revisit candidacy. That is exactly
what happened in the 2026-08-21 field trial: a valid 613-frame memory
returned `no_causal_candidate` on every plan, and the system silently
degraded to plain ImageGoal exploration. The hub now makes the mistake
structurally impossible instead of procedurally avoidable.

~~~text
navigator_reset
   -> phase = memory_recording
        /memory_step        record-only causal RGB append (no goal)
        /goal_candidate     support-check and register a goal photo;
                            accepted candidate RGB is NEVER appended to memory
        /imagegoal_step     REJECTED (HTTP 400, no upstream traffic)
   -> /prepare_revisit  (explicit task boundary; robot stationary)
        read-only cached-DINO whole-history score + SIFT/epipolar top-1 check
        selects one non-trivial supported candidate and freezes its SHA-256
        replays a stride-8 tail of recorded frames through NavDP
        /memory_replay_step and hard-verifies queue_lengths, mirroring the
        simulator shared-trace boundary; any failure latches
        native_state_uncertain and requires reset
   -> phase = revisit_query
        /imagegoal_step     legal; the first goal query freezes the MemNav
                            goal session at the revisit start point with the
                            full recorded history eligible
        /memory_step        REJECTED
~~~

The Jetson samples candidate-only frames every 24 recorded memory frames, up
to six candidates. Each candidate freezes its causal support ceiling at 16
frames before its capture boundary; later recording can never widen that
eligible history. A candidate is registered only when the read-only support
query reports at least 16 geometric inliers and DINO cosine at most 0.90; four
immediately following camera frames are omitted from memory to avoid a trivial
near-self match.  These thresholds remain provisional until calibrated on the
disabled-adapter walk.  Manual candidate capture and the offline scoring script
remain available for controlled ablations.

`/prepare_revisit` is idempotent.  A lost HTTP response can be retried without
opening a second goal session or repeating NavDP warm-up.  Its single receipt
contains candidate scores, selected id/SHA-256, the selected JPEG, warm-up
indices and queue lengths.  The Jetson verifies the JPEG hash, updates the
runtime ImageGoal and acknowledges the installed SHA on every query; the hub
uses its committed goal bytes rather than trusting a stale client upload.
`/healthz` reports the active goal and last prepare receipt.  ROS mirrors the
same state on `/navdp/status`, publishes full event receipts on
`/navdp/cec_receipt`, and shows phase/goal/CEC state in the RViz marker.

### Reset

`POST /navigator_reset` atomically resets MemNav and NavDP. A successful
receipt must prove all of the following:

- CEC is enabled;
- the LingBot monocular-depth stream is enabled;
- `metric_depth_sensor_consumed=false`;
- NavDP is frozen to `depth_source=monocular_sidecar`;
- a sidecar endpoint is configured.

The camera optical-center height is supplied by the workstation launch
environment. There is intentionally no default. It must be physically measured
on the installed D435i before a real model reset.

### ImageGoal step (revisit_query phase only)

For each `POST /imagegoal_step`:

1. the hub accepts the old client depth field only for wire compatibility, then
   discards it;
2. `/retrieval_probe_step` appends current RGB exactly once and advances the
   shared LingBot state;
3. NavDP retrieves mono depth for exactly that RGB SHA-256 from
   `/monocular_depth_query`;
4. CEC retrieves temporally diverse history candidates and verifies them with
   SuperPoint/LightGlue, LingBot depth and PnP;
5. accepted proof produces a scale-free bearing, normalized onto a fixed
   `2.5 m` PointGoal residual;
6. accepted evidence calls frozen mixed ImageGoal + PointGoal NavDP;
7. rejected evidence calls exact mono-native ImageGoal NavDP.

CEC does not implement a semantic Novel/Revisit classifier. Acceptance means
only that the current online history supports a self-certified localization
hypothesis; rejection means abstention.

Every returned trajectory must carry a mono-depth receipt. A response that
cannot prove the configured depth source is rejected and latches reset.

## Scale contract

The first 40 causal frames form the frozen scale bootstrap. Before scale is
ready, the sidecar returns the explicit bootstrap-zero-depth state. At the
transition, the scale receipt must freeze exactly once from the installed
camera optical-center height. Later frames may consume that frozen scale but
must not silently recalibrate it.

The `0.5 m` value used in the 2026-08-20 health-only smoke was not a
calibration and is not a deployment default.

## Failure semantics

| Failure | Required behavior |
| --- | --- |
| Certificate rejects | Exact mono-native ImageGoal NavDP |
| Certificate endpoint errors after a successful RGB append | Exact mono-native ImageGoal NavDP, with an error receipt |
| Causal RGB/LingBot append fails | Latch `reset_required`; issue no new trajectory |
| NavDP request or mono-depth receipt is ambiguous | Latch `reset_required` |
| Concurrent hub request | Reject with HTTP `409` |
| SSH tunnel breaks | Jetson plan becomes stale and velocity becomes zero |
| RGB or local depth becomes stale | Jetson velocity becomes zero |
| Local aligned-depth ROI is blocked | Jetson slows or stops independently |
| Go2 command stream is stale | Bridge sends zero and calls `StopMove()` |
| Hand controller becomes active | Autonomous SportClient authority is released |

A stream failure cannot fall back to metric NavDP: the same LingBot state owns
both the current mono-depth observation and the history proof.

## Robot-local control and authority

The trajectory tracker uses robot-plane coordinates `x` forward and `y`
left. It emits forward velocity and yaw rate; reverse and lateral velocity are
disabled by default. The default linear limit is `0.30 m/s`. The bridge keeps
the observed hardware command floors of `0.10 m/s` translation and
`0.20 rad/s` rotation.

The workstation is advisory and has no Unitree SDK, ROS velocity publisher or
direct actuator route. The Jetson retains explicit enable, estop, trajectory
freshness, collision guard, operator takeover and the final watchdog.

### Proof-gated direct-bearing handoff

Long-range CEC owns Revisit content addressing, while native NavDP owns the
unsupported/Novel route. Once the live image and goal become directly
covisible, `/local_pose_query` may refine either route with a certified
scale-free current-to-goal bearing. Bearings inside the measured `+/-60 deg`
NavDP point-token support are normalized onto the same validated `2.5 m`
residual used by CEC. Rearward bearings never enter that token interface: the
Jetson atomically emits bounded zero-translation yaw. Proof loss returns to the
preceding native or long-range route.

The low-latency query still reports a first-40-MDTEC-scaled translation for
diagnostics, but real trace replay showed that value can underestimate the
physical residual by at least `7.9x`. Schema
`cec_direct_bearing_handoff_v2_20260824` therefore grants metric translation
no control authority and no STOP authority. Automatic arrival remains
fail-closed until a separately validated scale-free visual-convergence proof
exists. The separate GOAT `/arrival_query` retains its strict-first-64 research
contract; its claim must not be promoted into the robot execution boundary.

Every direct-bearing disposition is returned in the plan receipt and
revalidated at the robot execution boundary. A stale v1 schema, malformed
bearing or unproven atomic turn cannot actuate the robot.

The adapter starts with both `enable_on_start=false` and
`estop_on_start=true`. Clearing estop and enabling motion are distinct onsite
operator actions.
