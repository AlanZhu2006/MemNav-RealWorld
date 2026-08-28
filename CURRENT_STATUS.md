# Current Full-Mono Real-World Status

Snapshot: **2026-08-28, protocol-v3 + direct-bearing-v2 + optional Odin1 reference lane**

现场实验、交接、双机架构、两阶段数据、控制安全、证据采集和SR/SPL的统一操作入口为
`REALWORLD_EXPERIMENT_HANDBOOK_CN.md`。本文件继续作为最新claim boundary；若旧日期文档
与本文件冲突，以本文件和当前代码为准。

新增的两阶段真机框架已经把长程手柄 survey 冻结为 exact-byte episodic dataset，并能在
独立的第二次运行中校验、重放、安装目标和初始化 formal query。入口与剩余边界见
`TWO_PASS_REVISIT_RUNBOOK_20260825.md`。该更新解决实验生命周期和持久化，不改变下文
“自动 arrival/STOP 尚未建立”的结论。

2026-08-27 增加了不具备运动权限的实验采集侧车：每轮自动保存 ROS bag、CEC/status
JSONL 和 RViz dashboard，第三人称原片导入后与同一 run ID、Git revision 和 SHA-256
manifest 绑定。当前仓库中的双视角视频只是 engineering reference demo，不改变正式结果
边界。完整操作见 `EXPERIMENT_DATA_COLLECTION.md`。

同日注册了 `4 scenes x 5 repeats = 20 runs` 的首轮 CEC-certified bearing + frozen
NavDP 评测模板，公开页、逐 run 媒体槽位和机器 manifest 均保持空白。该登记只冻结
未来结果的形状，不代表已经执行实验，也不改变 arrival/STOP 和 SR/SPL 尚未建立的结论。
见 `REALWORLD_EVALUATION.md`。

2026-08-28 增加了完全隔离的Odin1参考评测栈：mode-1往返建图、D435i目标图与Odin
地图位姿绑定、mode-2 `map -> odom`重定位门、局部odom路径积分、融合到达证据和冻结
A* SPL收据。Odin不进入CEC/NavDP/Go2控制。当前Odin未连接，本轮没有硬件标定或正式
结果；正确命名是independent reference SLAM，不是计量级absolute GT。完整边界和命令见
`deployment/odin1_gt/README_CN.md`。

## Bottom line

The two-machine Full-Mono CEC stack is synchronized and fail-closed, and it
has completed real powered navigation trials.  It has **not** completed one
autonomous ImageGoal arrival: all powered trials below remain failures.  The
remaining P0 is no longer transport, camera height, controller sign, or
monocular-depth wiring.  It is a separately validated, scale-free terminal
visual-servo / arrival contract.

The robot is currently stopped.  No camera, NavDP adapter, Go2 bridge or RTX
policy service is running.

## Current architecture and authority

- episode protocol: server-enforced v3
  `memory_recording -> prepare_revisit -> revisit_query`;
- policy observation: causal monocular RGB plus ImageGoal;
- short-range expert: frozen LingBot dense mono-depth readout into frozen
  NavDP's existing depth encoder;
- long-range expert: CEC history retrieval, LightGlue/PnP certificate and a
  scale-free bearing;
- direct-local expert: current-to-goal certified scale-free bearing;
- controller: frozen NavDP trajectory decoder; rearward direct bearings are
  executed only as a bounded Jetson atomic turn;
- D435i metric depth: Jetson collision safety only, never a policy input;
- certificate/proof loss: return to the preceding native or long-range route;
- metric PnP translation: diagnostic only, with no control or STOP authority;
- automatic STOP: disabled until an independent convergence proof is
  calibrated and confirmed.
- optional Odin1 lane: evaluation-only map/relocalization/path/arrival evidence;
  it has no policy, motion or estop authority.

The terminal wire schema is
`cec_direct_bearing_handoff_v2_20260824`.  Both reset and launcher preflight
now compare the hub-advertised schema with the schema imported from the actual
Jetson executor source.  A partial file copy (v2 hub with v1 executor, or the
reverse) therefore refuses startup instead of silently changing motion
authority.

## Powered field evidence

| Trial | Observed result | Formal outcome |
| --- | --- | --- |
| Q -> R, CEC Revisit | moved 3.01 m; auxiliary distance 3.507 -> min 1.498 m; long-range and direct bearings became inconsistent near the goal | failure: `safety_abort_path_length_limit` |
| R -> Q, native Novel | 1.167 -> min 1.019 -> final 1.022 m; only 0.615 m path | failure: old controller/Go2 velocity-floor ordering caused left-right hunting; execution contract subsequently fixed |
| S -> Q, native Full-Mono after controller fix | first command 0.297 m/s forward; 1.226 -> min 0.993 -> final 3.729 m; 18.54 m path | failure: `operator_stop`; the robot passed the high-covisibility window without a valid arrival decision |

The controller repair restored the formal `0.30/0.55` limits and applies the
`8 deg` heading deadband before the Go2 `0.10/0.20` command floors.  It removes
the earlier hunting mechanism, but does not solve arrival.

## Why direct PnP cannot authorize STOP

On the S -> Q trace, frames 325--328 pass the full
LightGlue -> LingBot depth -> PnP -> certificate chain.  Its predicted metric
distances fall from `0.769` to `0.125 m`, while the independent evaluator says
the entire run never came closer than `0.993 m`.  The terminal metric scale
therefore underestimates by at least `7.9x` on this trace.  V2 keeps only the
certified direction and projects it to the frozen `2.5 m` residual.

A new read-only audit scanned all 431 recorded frames against the same goal:

- only 15 frames reached the already-frozen two-view certificate precheck;
- frame 326 was the strongest supported near-view, with 331 LightGlue
  matches, 299 fundamental inliers, query/reference hull coverage
  `0.712/0.398`, and normalized median identity flow `0.0613`;
- nevertheless, the run's true minimum distance remained `0.993 m`;
- low identity-flow values from other frames were often supported by only a
  few spurious matches, so view error must always be conditioned on proof;
- three additional disabled/static traces contain strong covisibility but no
  physical arrival labels and therefore cannot freeze a success threshold.

The collector is
`deployment/gpu/audit_visual_convergence.py`; immutable outputs are under
`Nav-graph-blind/.diagnostics/realworld_visual_convergence_20260825/`.
This is measurement-only evidence, not a deployed STOP policy.

## Causal goal-selection repair

The first automated goal lifecycle smoke selected online anchor 215 even
though candidate construction had frozen an eligible ceiling of 200.  That
transport smoke is not a causal Revisit result.  The hub now carries the
selected candidate's `eligible_anchor_ceiling` into every retrieval probe and
rejects an accepted candidate whose server receipt omits or widens that
ceiling.  Operator-supplied frozen ImageGoals remain the formal benchmark
route; automatic goal selection remains an optional lifelong demo.

## Verification and synchronization

- focused RTX/standalone regression: **71 passed**;
- Python compile, shell syntax and `git diff --check`: passed;
- Jetson targeted runtime regression after synchronization: **31 passed**;
- matching v2 schema accepted; stale v1 schema rejected;
- updated Jetson files match the workstation SHA-256 values;
- pre-sync Jetson files are recoverable from
  `.deployment_backups/20260825_bearing_v2_contract_pre_sync/`;
- one stale ROS process that only published `estop=true` was removed by exact
  PID; no navigation process remains.

Synchronization did not start the camera, ROS adapter, Go2 bridge or motors.

## What is established and what is not

Established:

1. the real camera/transport/Full-Mono transaction reaches the frozen policy;
2. the repaired Go2 controller produces forward motion without the previous
   left-right hunting mechanism;
3. CEC can recover a useful Revisit direction and direct two-view proof can
   refine it;
4. monocular PnP direction and metric distance require different authority;
5. stale or partially synchronized terminal schemas now fail closed.
6. formal-run evidence now has a single run ID and hash-bound ROS/dashboard/
   third-view collection contract.
7. an Odin1 reference implementation can bind `S_i/L_i/P_i/SPL_i` without
   exposing Odin observations to the navigation method.
8. the official Odin driver `v0.14.0` native-Mode1 profile is compiled in
   `/home/nvidia/twork/odin_ws` and passes its no-hardware source/dependency
   preflight.

Not established:

1. autonomous Novel or Revisit arrival and STOP;
2. a physically calibrated relationship from visual convergence to target
   success radius;
3. real-world SR/SPL or a statistically meaningful number of trials;
4. that automatic target selection satisfies the same causal contract as an
   externally frozen benchmark goal beyond the repaired software check.
5. Odin1 0.14 Mode1 has been reported normal in a prior operator test, but this
   release still lacks a hash-bound live USB/topic receipt, current-Go2
   mount/extrinsic calibration, relocalization repeatability, metric arrival
   thresholds and path-accuracy validation.

## Next safe experiment

Do not run another formal navigation trial. First connect Odin1 with the robot
disabled, use the installed native `v0.14.0` driver profile, verify live
USB/topics and measure the current mount,
obstacle height band and Go2 footprint. Then record one debug out-and-back map,
test mode-2 relocalization across restarts and collect the predeclared
`0, 0.25, 0.5, 1.0 m` by `0, +/-10, +/-20 deg` arrival offsets with independent
tape labels. Use one location to choose the combined metric/visual convergence
rule and different locations to test it.

Only after that gate passes should the terminal controller gain a two-stage
authority:

1. strong direct proof may enter a zero-translation visual-alignment hold;
2. persistent, independently calibrated convergence may authorize STOP.

Until then, real trials require an external evaluator/operator termination
and cannot be reported as autonomous ImageGoal success.
