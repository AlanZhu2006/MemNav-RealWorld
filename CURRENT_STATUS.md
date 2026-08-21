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

## Established by prior field operation

- The D435i optical-center height is operator-confirmed at `0.42 m`
  (2026-08-21); `CEC_CAMERA_HEIGHT_M=0.42` is the deployed value.  The
  launcher still requires the explicit export -- the env gate is a
  configuration check, not an open measurement question.
- The base RGB-D NavDP ImageGoal deployment on this Go2 was genuinely
  effective in earlier field operation: camera pipeline, adapter, trajectory
  tracking, stale-plan stop, watchdog, and bridge are field-proven
  components.  The untested-on-robot surface is therefore confined to the
  monocular additions, not the deployment stack.

## Not yet verified (remaining real gates)

1. Frame 0--39 bootstrap and the one-time frame-40 scale freeze have not been
   audited on real D435i RGB at 0.42 m (all prior field operation was
   metric RGB-D; the mono scale path has only run in simulation and in the
   RTX live-stack smoke).
2. Static left/right CEC bearing-sign calibration on the real mount is
   pending.
3. The v3 two-phase contract has passed a full live-stack smoke on the RTX
   host but has not yet been exercised from the robot.  The 2026-08-21
   tethered revisit trial FAILED under protocol v2 (goal issued from frame 0
   excluded all history: `no_causal_candidate`); v3 fixes the root cause.
4. The weak-covisibility goal-candidate proxy thresholds are provisional
   until calibrated on real recorded frames.
5. No formal real-world Full-Mono Novel/Revisit SR or SPL exists.

Recommended but no longer treated as blocking, because the fail-closed paths
are test-covered and the watchdog/stale-plan layer is field-proven from the
RGB-D deployment: tunnel-kill / MemNav-kill fault injection under the final
release.

## Next safe action

Reconnect the D435i and run `deployment/go2/offboard/fullmono.sh start` from
the Jetson (after fast-forwarding the Jetson checkout to `f9a1e37`).  Gates
1, 2, and 4 fold into one combined pre-trial recording walk: record a short
A->B leg with the adapter disabled, verify the frame-40 scale receipt and
left/right bearing sign from the diagnostic output, and score the captured
goal candidates to calibrate the proxy thresholds.  Only after those
receipts pass, authorize tethered `0.5--1.0 m` motion for the first v3
revisit trial.
