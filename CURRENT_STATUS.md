# Current Full-Mono Real-World Status

Updated 2026-09-17. This page describes the checked-in implementation and recorded
evidence. The previous status is preserved in
[CURRENT_STATUS_20260907.md](CURRENT_STATUS_20260907.md).

## Current implementation

- Both paired arms run frozen NavDP on the RTX 4090 with the same causal
  monocular depth. `mono_native` disables memory guidance; `mono_cec` enables it.
- Survey preparation verifies the ordered RGB dataset, installs the exact goal,
  and initializes the controller buffer from the current query view.
- Compatible local engineering Surveys can restore a sealed, goal-free state.
  A missing cache causes RGB replay; a damaged or incompatible cache requires
  reset. Model weights remain separate from the saved episode state.
- Storage configuration explicitly selects the writer, historical geometry and
  KV representation. Production defaults remain `legacy / dense / native`.
  `native_interval7` uses RGB replay rather than legacy state checkpoints.
- The active-run recovery logic pauses at stale RGB-D or plan input, discards
  old actions and waits for fresh observations. The Revisit budget is 500 s,
  including recovery pauses; STOP and hard faults still terminate the run.
- Sensor depth remains local to safety. IMU feedback controls turning, while
  the streaming model estimates visual geometry.

Details: [GPU services](deployment/gpu/README.md),
[storage integration](REALWORLD_MEMORY_STORAGE_SYNC_20260916_CN.md),
[operation rules](AGENTS.md).

## Recorded evidence

The current manuscript reports 30 paired trials per method across eight indoor
and two outdoor settings, with 4/30 goals reached by Base and 25/30 by GEM.
Those are the paper's field results, maintained in the independent
[paper repository](https://github.com/AlanZhu2006/Memnav_Paper).
Individual run labels and dependencies remain bound to the local pair registry
and capture records. The old four-scene, 20-pair plan in `REALWORLD_EVALUATION.md`
is a separate protocol and does not define this manuscript's denominator.

The 2026-09-16 stationary replay used the same 120-frame Survey and 196 frozen
RGB requests per configuration. Of 136 planning calls, 121 follow 15 warmup
calls. Median/P95 prepared-JPEG round trips were:

| Configuration | Median / P95 |
| --- | --- |
| Full depth maps | 387.7 / 490.8 ms |
| Compact archives | 336.0 / 394.1 ms |
| Compact archives + lossless KV | 334.4 / 393.7 ms |

The three configurations returned identical depth, bearings and trajectories
in this replay. The [measurement record](REALWORLD_MEMORY_STORAGE_SYNC_20260916_CN.md)
contains the comparison setup, output checks and original evidence paths.
This is a stationary service measurement, not an additional navigation trial.

## Workspace and data

This checkout now lives at `Nav-graph-blind/projects/realworld`. The previous
`/home/asus/Research/MemNav-RealWorld` path is a compatibility link; existing
machine configuration remains valid. Code synchronization does not change the
revision running on the Jetson or replace an active GPU service.

The 2026-09-17 storage cleanup removed rebuildable Survey snapshots and verified
expanded duplicates. Original Survey RGB, manifests, recordings and retained
compressed archives remain. A subsequent Prepare may replay the Survey and
recreate its cache. See [local storage notes](docs/LOCAL_STORAGE.md).

## Validation and operation

Use the source compatibility verifier and resolved-configuration preflight in
[deployment/gpu/README.md](deployment/gpu/README.md). Historical release manifests
and `verify_public_baseline.py` retain their August snapshot hashes. Current
code, runtime contracts and dated measurement records describe later changes.

The protected registry and run-specific configuration govern new collection.
Use [TWO_PASS_REVISIT_RUNBOOK.md](TWO_PASS_REVISIT_RUNBOOK.md) for the selected
Episode and [AGENTS.md](AGENTS.md) for operation rules. No robot motion or new
navigation experiment was performed during this repository consolidation.
