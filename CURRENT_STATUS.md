# Current Full-Mono Real-World Status

Snapshot: **2026-08-21 protocol-v3 update**

## Bottom line

The Full-Mono software release is synchronized and auditable, but the physical
navigation campaign is not complete. This is **code deployed, transport
verified, robot stopped**—not a real-world SR result.

## Verified

| Gate | Result |
| --- | --- |
| Research source | protocol-v3 two-phase hub, goal-candidate capture, scorer and NavDP warm-up ported from the research workspace |
| Standalone contract tests | 35 passed (gpu) + 40 passed (go2, non-ROS) |
| Research hub/runtime tests | 31 passed (hub 15 + runtime + composition contract) |
| Python compile and shell syntax | Passed |
| Architecture SVG XML | Valid |
| Jetson SSH | `tegra-ubuntu` reachable |
| Jetson live overlay | All four payload hashes match the immutable release |
| Immutable release | `cec_mono_20260820_d656b9d9ae30de73` present |
| Rollback | `rollback_pre_mono_20260820_d656b9d9ae30de73` present |
| Jetson → workstation SSH | Passwordless route previously verified |
| Protocol-v2 loopback health smoke | 5/5 passed without model, camera, ROS or motion (pre-v3; re-run against the v3 hub before the next trial) |
| Jetson single-entry launcher | Implemented; missing-D435i test failed closed and rolled back both machines |
| Current process state | No MemNav/NavDP/hub/Go2 navigation stack running |
| Motion during deployment | None |

## Frozen policy contract

- episode protocol: v3 two-phase (memory_recording -> begin_revisit -> revisit_query); goal queries during recording are rejected server-side;
- navigation input: one causal monocular RGB stream plus ImageGoal;
- short-range readout: LingBot mono-depth sidecar;
- long-range readout: CEC proof and scale-free bearing;
- controller: frozen NavDP only;
- CEC reject: exact mono-native NavDP;
- stream append or ambiguous policy failure: latch `reset_required`;
- D435i aligned depth: Jetson collision safety only;
- policy metric-depth consumption: forbidden and receipt-audited.

## Not yet verified

1. A provisional D435i optical-center height of `0.42 m` has been recorded;
   formal experiments still require a standard-pose measurement to ±1 cm.
2. The full weighted MemNav + NavDP stack has not been reset under the measured
   camera-height contract.
3. Camera plus disabled adapter has not completed the required 10-minute
   Full-Mono static run.
4. Frame 0--39 bootstrap and the one-time frame-40 scale freeze have not been
   audited on the robot.
5. Static left/right CEC bearing-sign calibration is pending.
6. Tunnel-kill and MemNav-kill fault injection under the final release is
   pending.
7. Tethered `0.5--1.0 m` powered Full-Mono motion is pending.
8. No formal real-world Novel/Revisit SR or SPL exists. The 2026-08-21
   tethered revisit trial FAILED under protocol v2 (goal issued from frame 0
   excluded all history: `no_causal_candidate`); the v3 two-phase contract
   fixes the root cause but has not yet been exercised on the robot.
9. The weak-covisibility goal-candidate proxy thresholds are provisional
   until calibrated on the disabled-adapter walk.

## Next safe action

Use the provisional `CEC_CAMERA_HEIGHT_M=0.42` only for static acceptance and
reconnect the D435i, then run `deployment/go2/offboard/fullmono.sh start` from
the Jetson. The launcher now requires a real CameraInfo frame and rolls both
machines back if the camera is unavailable. Complete the static and
fault-injection gates in `RUNBOOK.md`; do not start the Go2 bridge until those
receipts pass. Re-measure to ±1 cm before formal motion trials.
