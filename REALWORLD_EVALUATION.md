# Planned Paired Real-World Evaluation

Snapshot: **2026-08-29** · Status: **frozen-shape protocol; no formal results**

This page registers the real-robot experiment required by the conference
matrix.  It compares frozen monocular NavDP with the same controller plus CEC
in matched physical blocks.  Every blank value is deliberately unresolved; it
does not mean zero, failure, or missing-at-random data.

The campaign contains four static scenes and five paired blocks per scene.
Each block contains two independent physical rollouts:

1. `mono_native`: frozen monocular NavDP with memory authority disabled;
2. `mono_cec`: the same controller and inputs with CEC authorization enabled.

The total remains 40 rollouts, but unlike the archived v1 plan, the arms are
interleaved as 20 matched pairs rather than collected in separate campaigns.
The controlling machine-readable plan is
[`manifests/realworld_paired_evaluation_plan_v2.json`](manifests/realworld_paired_evaluation_plan_v2.json).
This preregistration remains outcome-blank after data collection.  Formal
numbers are derived into a separate report by the read-only verifier; no
success, path, SPL, or evidence hash is written back into the plan.

The executable arm boundary is explicit.  `formal-start --arm mono_native`
keeps the same sealed Survey, goal image, current RGB, and causal-monocular
depth transaction but skips certificate and direct-local bearing authority;
every plan records `cec_authority_mode=native` and calls the native ImageGoal
endpoint.  `--arm mono_cec` enables the frozen certificate/bearing path.  A
certificate that happens to reject is not accepted as a substitute for the
native arm.

The query-goal boundary is equally explicit.  `formal-start` requires a
pre-registered external goal path, its exact SHA-256, the sealed dataset
SHA-256, scene ID, and registered run ID.  It refuses to start on any mismatch
and always uses `operator_frozen_external_v1`; automatic Survey-candidate
selection is reserved for engineering/lifelong demos.  Novel versus Revisit is
therefore a property of historical support for the same kind of frozen goal,
not a runtime input or a different launcher branch.

## Why the design is paired

Within one pair, both arms must share:

- the sealed Survey dataset and exact goal JPEG;
- the declared Novel/Revisit role, hidden from the runtime;
- the physical start region and yaw tolerance;
- the controller checkpoint, camera mount, speed, path, and time budgets;
- the independent arrival and path evaluator.

The robot is physically reset between arms and model processes are restarted.
Formal-query frames never enter the sealed Survey memory.  Arm order is frozen
before any outcome: ten pairs are native-first and ten are CEC-first.  This
controls short-term changes in illumination, battery, floor condition, and
localization quality better than a 20-run CEC campaign followed later by a
20-run native campaign.

## Role and scene contract

Before Formal 01, the scene registry must freeze exactly:

- two Novel scenes, whose target has no certifiable support in Survey history;
- two Revisit scenes, whose target is supported by the causal Survey history;
- one exact goal SHA, dataset SHA, start pose receipt, and shortest feasible
  path per scene;
- five paired reset blocks per scene.

CEC receives no Novel/Revisit label.  On Novel, the intended behavior is
certificate rejection and exact native fallback; on Revisit, the certificate
may authorize only the registered scale-free bearing interface.

## Balanced execution schedule

| Scene | Pair 1 | Pair 2 | Pair 3 | Pair 4 | Pair 5 |
|---|---|---|---|---|---|
| Scene 01 | Native→CEC | CEC→Native | Native→CEC | CEC→Native | Native→CEC |
| Scene 02 | CEC→Native | Native→CEC | CEC→Native | Native→CEC | CEC→Native |
| Scene 03 | Native→CEC | CEC→Native | Native→CEC | CEC→Native | Native→CEC |
| Scene 04 | CEC→Native | Native→CEC | CEC→Native | Native→CEC | CEC→Native |

The schedule has ten blocks in each order.  Scene identities, roles, goals,
and block order cannot be changed after the first formal rollout.

## Arrival gate

No formal campaign may start until `CURRENT_STATUS.md`'s independent arrival
calibration gate is closed.  In particular:

- monocular PnP translation has no metric control or STOP authority;
- Odin1 is an evaluation-only reference SLAM, never a policy input;
- the success region, stationary hold, relocalization stability, path source,
  and dropout handling must be frozen from calibration data collected outside
  all four formal scenes;
- operator intervention is a failure in the intention-to-treat denominator;
- if an external evaluator terminates motion, the paper must say
  “navigation with independent evaluator termination,” not autonomous STOP.

## Metric and statistical contract

For rollout `i`:

```text
SR = sum(S_i) / N
SPL_i = S_i * L_i / max(L_i, P_i)
SPL = sum(SPL_i) / N
```

`S_i` is the independently adjudicated binary success, `L_i` the frozen
shortest feasible path, and `P_i` the independently measured physical path.
A failure contributes zero SPL.

Report each arm by role and overall.  The primary paired comparison is
native→CEC gain/loss with two-sided exact McNemar; SPL and path differences
use paired blocks and scene-cluster uncertainty.  With only 20 pairs, these
trials are primarily external closed-loop evidence and qualitative validation,
not a high-powered superiority study.

## Registered result table

| Method | Novel SR | Revisit SR | Overall SR | SPL | Mean path |
|---|---:|---:|---:|---:|---:|
| Frozen mono NavDP | — | — | — | — | — |
| Frozen mono NavDP + CEC | — | — | — | — | — |

Paired gain/loss, exact McNemar, Novel takeover count, Revisit accept/reject,
manual interventions, collisions, and failure attribution remain blank until
all evidence for a block is finalized.

The registered shape and current freeze blockers can be audited without any
robot or result access:

```bash
python tools/verify_realworld_paired_campaign.py
```

After all run directories have been independently finalized, derive the result
without modifying the preregistration:

```bash
python tools/verify_realworld_paired_campaign.py \
  --evidence-root /path/to/finalized_runs \
  --require-complete \
  --output /path/to/paired_campaign_verification.json
```

The verifier rechecks every capture seal and artifact inventory, the frozen
scene/goal/dataset bindings, Odin `S_i/L_i/P_i/SPL_i`, the explicit
`cec_authority_mode`, and native-arm non-takeover.  It reports no formal
aggregate unless all 40 registered run IDs pass.

## Scene registry

| Scene | Role | Target / setting | Survey SHA | Goal SHA | Start receipt | `L` |
|---|---|---|---|---|---|---:|
| Scene 01 | TBD | TBD | TBD | TBD | TBD | — |
| Scene 02 | TBD | TBD | TBD | TBD | TBD | — |
| Scene 03 | TBD | TBD | TBD | TBD | TBD | — |
| Scene 04 | TBD | TBD | TBD | TBD | TBD | — |

Exactly two role cells must become Novel and two Revisit before the registry is
sealed.  A dash may be replaced only from a finalized receipt.

## Per-block evidence rule

Each of the 20 paired blocks must bind both arm run IDs to:

1. the same frozen scene, Survey, goal, start, and budget receipts;
2. Jetson and RTX Git commits and method configuration hashes;
3. independent success, `L`, `P`, and SPL receipts;
4. ROS bag, CEC/status JSONL, Foxglove dashboard, and third-person video;
5. CEC accept/reject, selected anchor, certificate fields, and fallback audit;
6. reset tolerance and arm-order compliance;
7. collision, intervention, network, hardware, and policy failure attribution.

The pair is incomplete until both arms have valid evidence.  Hardware faults
remain in the intention-to-treat log and may only be additionally reported in
a separately labeled per-protocol sensitivity analysis.

## Claim boundary

No formal runs have been completed.  The archived v1 single-arm template and
the powered engineering traces in `CURRENT_STATUS.md` are not result rows.
Until the arrival gate passes and all receipts are independently reviewed, the
project has no publishable real-world SR, SPL, or autonomous STOP claim.
