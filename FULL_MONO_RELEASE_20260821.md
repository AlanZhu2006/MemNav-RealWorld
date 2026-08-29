# Full-Mono CEC Dual-Machine Release — 2026-08-21

> Historical protocol-v2 release receipt. Do not use this file as the current
> launch guide; use `RUNBOOK.md`, `deployment/go2/STACK_MODULES_CN.md` and
> `CURRENT_STATUS.md`.

## Why this synchronization was necessary

The Jetson already contained the 2026-08-20 protocol-v2 offboard overlay, but
its standalone `Memnav_Realworld` checkout still pointed to the 2026-08-18
RGB-D release. Two tracked scripts were locally modified and the immutable
release directory was untracked. The robot could pass the new health preflight,
but a fresh clone of the public repository could not reproduce that state.

This release closes that provenance gap. The research workspace, standalone
repository and Jetson payload now describe the same Full-Mono policy contract.

## Frozen architecture

~~~text
Jetson D435i RGB -> SSH tunnel -> one causal LingBot RGB stream
                                      |                |
                              dense mono depth     CEC proof/bearing
                                      \                /
                                      frozen NavDP
                                           |
Jetson tracker <- 24-point local trajectory
     + aligned-depth collision guard
     + stale-plan stop / watchdog / gamepad authority
~~~

The policy consumes no metric sensor depth. The D435i depth stream remains
enabled locally because collision safety and optional arrival auditing are
outside the learned navigation policy.

## What changed from the 2026-08-18 repository

- upgraded the unified hub from protocol 1 to protocol 2;
- removed metric-native fallback after a causal stream failure;
- required reset receipts to prove the mono-sidecar contract;
- added SHA-bound mono-depth payloads and first-40 scale receipts;
- upgraded frozen NavDP's server, agent and network interfaces for a
  `monocular_sidecar` depth source without changing checkpoint weights;
- required a physically measured camera optical-center height;
- made both RTX and Jetson health checks parse the complete sensor contract;
- synchronized the two Jetson scripts that had previously existed only as
  device-local modifications;
- updated architecture, runbook, status, source manifest and public diagram.

## Three-way synchronization receipt

| Location | State |
| --- | --- |
| Research workspace | `/home/asus/Research/Nav-graph-blind`, commit `70387b6` |
| Standalone release workspace | `/home/asus/Research/Memnav_Realworld`, branch `sync/fullmono-cec-20260821` |
| Jetson live overlay | `/home/nvidia/twork/NavDP/deployment/go2/offboard` |
| Jetson immutable release | `cec_mono_20260820_d656b9d9ae30de73` |
| Jetson rollback | `rollback_pre_mono_20260820_d656b9d9ae30de73` |

All four lower-computer payload hashes match the immutable receipt. No camera,
ROS adapter, Go2 bridge or motor command was started during synchronization.

## Verification

- standalone GPU contract tests: **26 passed**;
- research protocol-v2 hub/runtime tests: **10 passed**;
- Python compile: passed;
- all RTX and Jetson shell syntax checks: passed;
- architecture SVG: valid XML;
- Git whitespace check: passed;
- live Jetson hashes: matched.

## Remaining physical gates

The deployment is intentionally blocked until the D435i optical-center height
is measured. After that, the required order is:

1. reset the full weighted RTX stack and inspect mono receipts;
2. run camera + disabled adapter for at least ten minutes;
3. audit bootstrap and one-time scale freeze;
4. calibrate left/right bearing sign;
5. inject tunnel and MemNav failures and confirm zero velocity;
6. perform only then a tethered, low-speed short motion test.

No real-world SR/SPL is claimed by this release.
