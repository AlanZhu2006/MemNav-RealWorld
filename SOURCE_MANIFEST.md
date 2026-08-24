# Source and Artifact Manifest

Snapshot: **2026-08-25**

## Repository base

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
| `deployment/go2/offboard/` | SSH-forward and protocol-v3 offboard launcher with an executor-schema compatibility gate |
| `deployment/go2/terminal_motion_override.py` | The sole typed boundary for bounded direct-bearing atomic turns |

The MemNav model service remains an external research dependency because its
licensed checkpoints, LingBot weights, LightGlue dependency tree and research
workspace are not redistributed. `deployment/gpu/.env` points to that source
and is ignored by Git.

## Selected release SHA-256

| File | SHA-256 |
| --- | --- |
| `deployment/gpu/realworld_cec_hub.py` | `501858a5961f844098ad20d26e4a28763ac29937b37e3a9501c1caead746ee61` |
| `deployment/gpu/monocular_depth_runtime.py` | `9b88cbd091b83dbe15846ec0b47d329d715273f0557abffe319a463936c9c138` |
| `deployment/gpu/revisit_bearing_adapter.py` | `46c10132db7b00711ca3c781f18fcb9e04c4061bab9b44b8017d99c0c09bc6fd` |
| `deployment/gpu/revisit_local_pose_adapter.py` | `ab58913fff760182b1945d1a26c5dbb2bba58f040d38c28f030b19fc1bc569cd` |
| `deployment/gpu/audit_visual_convergence.py` | `807ba6b1ba3a9395ce0a89fbe79b368276479efb92c56be18ac4332b6b0f4af7` |
| `baselines/navdp/navdp_server.py` | `8f215345c9a1e9ed8fec3636e27d35c33949f4d14881209fadccc951a17f8057` |
| `deployment/go2/terminal_motion_override.py` | `1a0ea960c36e231d4424c1a3837d7b3cf88dce0ef7d4737068d371bfa888054e` |
| `deployment/go2/navdp_client.py` | `1d4cc28b7c8a5d9d864d0443bc9ab32c0e7124a19b39e2481ff41de6d5fefcf9` |
| `deployment/go2/offboard/runtime_contract.sh` | `bfa64b010a335e5bd1528c6033a636773d4631d443631da7f4c5e0d135858f97` |
| `deployment/go2/offboard/preflight_offboard.sh` | `da337f20bdc98c7ad8714ddb948c637a6451f32f394726a3bb07bc3aee2bbf45` |
| `deployment/go2/offboard/run_offboard_stack.sh` | `5298ef53d5eeee2e53cf6d784df8743cad8a9c0a2372bed7cc24b13612ea5ba0` |
| `deployment/go2/offboard/run_policy_tunnel.sh` | `eb65fb3c88c0976b17ddc87ee99e6481e6d4d0c718cc7121630446f76006c2c3` |
| `deployment/go2/offboard/stop_offboard_stack.sh` | `e6b239f1cd2c51d59bd09c57348e037697a7bd4de47c0c9316860c608ed798c3` |
| `deployment/go2/offboard/fullmono.sh` | `530dfcaf62cfa5395381470dd990ffa883d142ece5dcec0f5092cdb4efd6a1f8` |

The original four runtime payloads remain preserved as content-addressed
release `d656b9d9ae30de73f1d70a52b0150318f3dda238d6631dbae42f0a98dec973c2`.
`fullmono.sh`, `preflight_offboard.sh` and `run_offboard_stack.sh` now source
the same `runtime_contract.sh`.  The helper reads the terminal schema from the
actual executor source, so copying only one side of a wire-contract update
cannot pass health preflight.  This adds no motion authority.

## Deliberate exclusions

- model checkpoints and weight caches;
- Conda/venv environments and compiled dependencies;
- MemNav causal buffers and service logs;
- captured goal RGB/depth, Go2 reference poses and result JSON;
- robot/network credentials and SSH private keys;
- simulator datasets and unrelated diagnostics.

Runtime exclusions are enforced by `.gitignore`. Immutable releases remain
on the robot but `deployment/go2/releases/` is not committed.
