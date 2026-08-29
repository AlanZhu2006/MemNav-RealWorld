# Source and Artifact Manifest

Snapshot: **2026-08-29**

## Repository base

- Current repository: `https://github.com/AlanZhu2006/MemNav-RealWorld`
- Upstream repository: `https://github.com/InternRobotics/NavDP`
- Imported upstream commit: `878740a2011856d0e3782dd6ccd880fd2eccd70f`
- Full-Mono research source: `AlanZhu2006/Nav`,
  branch `feat/memnav-graph-blind-20260806`, commit
  `70387b63db65fedf1eb74bbe995631139d7b8e18`
- Standalone real-world base before this release:
  `ab4bf8e58ed21b070e3b0b8f01236d11921684ca`

The upstream history is retained. Full-Mono changes are recorded as an
auditable overlay instead of an anonymous source dump.

## Full-Mono overlay

| Path | Provenance and role |
| --- | --- |
| `deployment/gpu/realworld_cec_hub.py` | Protocol-v3 two-phase CEC hub adapted from the frozen research source |
| `deployment/gpu/monocular_depth_runtime.py` | First-40 scale and SHA-bound mono-depth receipt contract |
| `deployment/gpu/revisit_bearing_adapter.py` | Frozen scale-free bearing to 2.5 m PointGoal boundary |
| `deployment/gpu/revisit_local_pose_adapter.py` | Direct current-to-goal proof authority: bearing/turn only, no metric STOP |
| `deployment/gpu/audit_visual_convergence.py` | Read-only LightGlue near-view evidence collector; never grants runtime STOP |
| `deployment/gpu/score_realworld_revisit_goal.py` | Weak-covisibility goal-candidate scorer (frozen server components only) |
| `deployment/gpu/scripts/` | Path-parameterized RTX 4090 launch and fail-closed preflight |
| `baselines/navdp/{navdp_server,policy_agent,policy_network}.py` | Frozen NavDP with mono-sidecar and state-safe inference interfaces |
| `deployment/go2/` | Jetson Orin NX, D435i and Unitree Go2 integration |
| `deployment/go2/nav_stack.sh` + `stack_profiles.py` | Canonical composition boundary for two navigation profiles and three independent arrival authorities |
| `deployment/go2/offboard/` | SSH-forward and protocol-v3 offboard launcher with an executor-schema compatibility gate |
| `deployment/go2/terminal_motion_override.py` | The sole typed boundary for bounded direct-bearing atomic turns |
| `deployment/go2/offboard/experiment_capture.sh` | Evidence-only ROS bag, receipt and RViz/dashboard recorder; no motion calls |
| `deployment/go2/experiment_capture_manifest.py` | Run identity, third-view import, artifact SHA-256 and finalization contract |
| `deployment/go2/experiment_topic_logger.py` | Human-readable JSONL mirror of status, CEC and evaluator receipts |
| `deployment/odin1_gt/` | Independent Odin1 native-0.14/legacy-0.13 driver profiles, mapping/relocalization monitor, 2-D occupancy, arrival and A* SPL evidence lane |
| `deployment/odin1_gt/make_scene_contract.py` | Hash-sealed serial/firmware/calibration/driver/mount contract with rigid-transform validation |
| `deployment/odin1_gt/config/go2_odin_mount_receipt.template.json` | Fail-closed template for the measured, independently validated Odin-to-Go2 rigid transform |
| `deployment/odin1_gt/vendor/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch` | Historical, non-default firmware-0.13.1 cold-start compatibility patch previously validated in the local TopoFocus deployment |
| `deployment/odin1_gt/vendor/odin_ros_driver_runtime_config.patch` | Minimal ROS 2 parameter fix so hash-sealed per-session mode-1/mode-2 configs are actually consumed by the pinned driver |
| `tools/{transcode_demo_media.sh,build_demo_previews.py}` | Browser H.264, poster and inline-GIF publication helpers |
| `REALWORLD_EXPERIMENT_HANDBOOK_CN.md` | Unified Chinese architecture, Survey/Formal, safety, evidence, metric and handoff manual |
| `REALWORLD_EVALUATION.md` | Planned four-scene, five-paired-block SR/SPL and dual-view publication registry; contains no result claims |
| `docs/archive/` | Superseded dated release and integration receipts retained only for audit history |
| `manifests/realworld_evaluation_plan_v1.json` | Archived pre-meeting single-arm 20-run template; all metrics remain null |
| `manifests/realworld_paired_evaluation_plan_v2.json` | Controlling machine-readable plan: 20 balanced native/CEC pairs, 40 physical rollouts, all metrics null until evidence is finalized |
| `manifests/odin1_gt_reference_v1.json` | Machine-readable Odin authority boundary, implemented defaults and null field-calibration gates |

The MemNav model service remains an external research dependency because its
licensed checkpoints, LingBot weights, LightGlue dependency tree and research
workspace are not redistributed. `deployment/gpu/.env` points to that source
and is ignored by Git.

## Selected release SHA-256

| File | SHA-256 |
| --- | --- |
| `deployment/gpu/realworld_cec_hub.py` | `1964c64e171b1e9976dad666df8c82be364182ca23a90e87161b4a7dd1f60be6` |
| `deployment/gpu/monocular_depth_runtime.py` | `9b88cbd091b83dbe15846ec0b47d329d715273f0557abffe319a463936c9c138` |
| `deployment/gpu/revisit_bearing_adapter.py` | `46c10132db7b00711ca3c781f18fcb9e04c4061bab9b44b8017d99c0c09bc6fd` |
| `deployment/gpu/revisit_local_pose_adapter.py` | `ab58913fff760182b1945d1a26c5dbb2bba58f040d38c28f030b19fc1bc569cd` |
| `deployment/gpu/audit_visual_convergence.py` | `807ba6b1ba3a9395ce0a89fbe79b368276479efb92c56be18ac4332b6b0f4af7` |
| `baselines/navdp/navdp_server.py` | `8f215345c9a1e9ed8fec3636e27d35c33949f4d14881209fadccc951a17f8057` |
| `deployment/go2/terminal_motion_override.py` | `1a0ea960c36e231d4424c1a3837d7b3cf88dce0ef7d4737068d371bfa888054e` |
| `deployment/go2/navdp_client.py` | `ded9824071dd022a914260283972a8995d86d2feb59a3fb8384a69a9d3d88e6e` |
| `deployment/go2/offboard/runtime_contract.sh` | `bfa64b010a335e5bd1528c6033a636773d4631d443631da7f4c5e0d135858f97` |
| `deployment/go2/offboard/preflight_offboard.sh` | `da337f20bdc98c7ad8714ddb948c637a6451f32f394726a3bb07bc3aee2bbf45` |
| `deployment/go2/offboard/run_offboard_stack.sh` | `5298ef53d5eeee2e53cf6d784df8743cad8a9c0a2372bed7cc24b13612ea5ba0` |
| `deployment/go2/offboard/run_policy_tunnel.sh` | `eb65fb3c88c0976b17ddc87ee99e6481e6d4d0c718cc7121630446f76006c2c3` |
| `deployment/go2/offboard/stop_offboard_stack.sh` | `e6b239f1cd2c51d59bd09c57348e037697a7bd4de47c0c9316860c608ed798c3` |
| `deployment/go2/offboard/fullmono.sh` | `530dfcaf62cfa5395381470dd990ffa883d142ece5dcec0f5092cdb4efd6a1f8` |
| `deployment/odin1_gt/vendor/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch` | `2a73aa48d163e2a362670b7b9b778edf8328aba7323e1cc04dd6b8fb28ba5806` |
| `deployment/odin1_gt/vendor/odin_ros_driver_runtime_config.patch` | `953bd96ad3cea5c336f11882f92a428ff090ba13abd28c742314f072cd637f86` |

The original four runtime payloads remain preserved as content-addressed
release `d656b9d9ae30de73f1d70a52b0150318f3dda238d6631dbae42f0a98dec973c2`.
`fullmono.sh`, `preflight_offboard.sh` and `run_offboard_stack.sh` now source
the same `runtime_contract.sh`.  The helper reads the terminal schema from the
actual executor source, so copying only one side of a wire-contract update
cannot pass health preflight.  This adds no motion authority.

Curated architecture and reference-demo derivatives are indexed with exact
sizes and SHA-256 values in `media/README.md`. The source recordings remain
local masters; their browser H.264/GIF/poster derivatives are presentation
evidence only and are not treated as a formal run.

## Deliberate exclusions

- model checkpoints and weight caches;
- Conda/venv environments and compiled dependencies;
- MemNav causal buffers and service logs;
- captured goal RGB/depth, Go2 reference poses and raw result JSON;
- raw ROS bags, receipt logs, third-view masters and RViz recordings under
  `runtime/` (only reviewed publication derivatives are committed);
- robot/network credentials and SSH private keys;
- simulator datasets and unrelated diagnostics.

Runtime exclusions are enforced by `.gitignore`. Immutable releases remain
on the robot but `deployment/go2/releases/` is not committed.
