# Full-Mono Protocol-v3 Release (2026-08-21)

This release upgrades the deployed episode contract from protocol v2 to v3.
It is a direct response to the first tethered revisit trial, which failed
without any hardware or model fault.

## Root cause of the field failure

The v2 hub exposed only goal-conditioned stepping, so the Goal-A image was
queried from frame 0 of the walk. MemNav freezes a goal session at the first
query for a goal image (`goal_start_frame=0`, candidate ceiling `-1`), which
excluded the entire A->B history from Revisit candidacy. CEC returned
`no_causal_candidate` on every plan over a valid 613-frame memory, and the
system silently degraded to plain ImageGoal exploration (assist distance grew
2.81 m -> ~6.07 m before e-stop). A post-hoc diagnostic over the same intact
memory produced 8 candidates and an accepted certificate in ~4 s, proving the
pipeline was healthy and only the session ordering was wrong.

The same bug class was found and fixed in the simulation harness on
2026-08-18 (leg A must run without opening a goal session). Protocol v3
mirrors the simulator's two-phase boundary on the robot.

## What changed

- **Two-phase episode contract, enforced server-side.** Reset enters
  `memory_recording`; only `/memory_step` (record-only append) and
  `/goal_candidate` are legal. `/begin_revisit` (>=1 recorded frame, robot
  stationary) switches to `revisit_query`; only then is `/imagegoal_step`
  accepted. Out-of-order calls fail with HTTP 400 and produce zero upstream
  traffic.
- **NavDP warm-up parity.** `/begin_revisit` replays a stride-8 tail of the
  recorded frames (max 8) through NavDP `/memory_replay_step` and
  hard-verifies `queue_lengths`, mirroring the simulator shared-trace
  boundary. Warm-up failure latches `native_state_uncertain` and requires a
  reset. Receipts (`navdp_warmup_frames`, frame indices, queue lengths) are
  returned to the operator.
- **Goal-candidate capture.** `/goal_candidate` registers goal photos taken
  during the walk that are never appended to memory, with
  `captured_after_frame` and SHA-256 receipts, mirroring the simulator rule
  that revisit goals come from the walk but are excluded from memory.
- **Weak-covisibility scorer.** `deployment/gpu/score_realworld_revisit_goal.py`
  scores candidates against the recorded frames using only frozen server
  components (stateless DINO cosine sweep + LightGlue verification of the
  argmax frame). Provisional bands: reject `inliers < 16` (unsupported) and
  `max_cos > 0.90` (near-duplicate); thresholds are provisional until the
  disabled-adapter walk calibration. The simulator's ground-truth covis bands
  (standard `[0.55, 0.90]`, hard `[0.25, 0.55)`) are reference semantics
  only.
- **Jetson adapter v3 flow.** With `two_phase_episode=true` (set by the
  offboard stack), the adapter streams `/memory_step` during recording,
  exposes `~/capture_goal_candidate` and `~/begin_revisit` Trigger services,
  reports `phase` / `frames_recorded` / `goal_candidates_captured` in
  `/navdp/status`, and surfaces hub contract rejections instead of retrying.
  Motion authorization, e-stop, watchdog and bridge logic are unchanged.
- **Manifest and verifier.** `manifests/realworld_fullmono_v3.json` declares
  the phase contract, endpoints and warm-up receipts (v2 retained as
  history); `tools/verify_public_baseline.py` now asserts the v3 contract
  and passes with `failures=0`.

## Verification (this workstation, no robot, no motion)

- gpu contract tests: **35 passed** (hub 15, incl. phase contract, goal
  candidates, warm-up parity and fail-closed paths; monocular runtime incl.
  depth-transaction binding).
- go2 tests: **40 passed** (client v3 endpoints incl. 400-surfacing; other
  non-ROS suites). `test_rgbd_sync.py` requires ROS `message_filters` and
  runs on the Jetson only (pre-existing).
- `tools/verify_public_baseline.py`: failures=0.
- Python compile and shell syntax: passed.

## Unchanged boundaries

All physical gates from `CURRENT_STATUS.md` remain open: measured camera
height, disabled-adapter static acceptance, frame-40 scale audit, bearing
sign check, fault injection, tethered motion. There is still no real-world
SR/SPL, and this release does not claim one.
