# Source and Artifact Manifest

Snapshot: **2026-08-21**

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
| `deployment/gpu/realworld_cec_hub.py` | Protocol-v2 CEC hub adapted from the frozen research source |
| `deployment/gpu/monocular_depth_runtime.py` | First-40 scale and SHA-bound mono-depth receipt contract |
| `deployment/gpu/revisit_bearing_adapter.py` | Frozen scale-free bearing to 2.5 m PointGoal boundary |
| `deployment/gpu/scripts/` | Path-parameterized RTX 4090 launch and fail-closed preflight |
| `baselines/navdp/{navdp_server,policy_agent,policy_network}.py` | Frozen NavDP with mono-sidecar and state-safe inference interfaces |
| `deployment/go2/` | Jetson Orin NX, D435i and Unitree Go2 integration |
| `deployment/go2/offboard/` | SSH-forward and protocol-v2 offboard launcher synchronized to the robot |

The MemNav model service remains an external research dependency because its
licensed checkpoints, LingBot weights, LightGlue dependency tree and research
workspace are not redistributed. `deployment/gpu/.env` points to that source
and is ignored by Git.

## Selected release SHA-256

| File | SHA-256 |
| --- | --- |
| `deployment/gpu/realworld_cec_hub.py` | `09ef562f11b6a0c1e0dcf63d021dee5ebcb0b88a5b2f951308cfb73fad15c993` |
| `deployment/gpu/monocular_depth_runtime.py` | `709a4ad200a5778317bb314e87e398ba6da8398939d96c100f235fe1ce98c9fc` |
| `deployment/gpu/revisit_bearing_adapter.py` | `46c10132db7b00711ca3c781f18fcb9e04c4061bab9b44b8017d99c0c09bc6fd` |
| `baselines/navdp/navdp_server.py` | `8f215345c9a1e9ed8fec3636e27d35c33949f4d14881209fadccc951a17f8057` |
| `deployment/go2/offboard/preflight_offboard.sh` | `770fe4eb205b6054d6ab50b9bff7fd12b5b587f8eefd1d7fe9bad6e3db8b1d0d` |
| `deployment/go2/offboard/run_offboard_stack.sh` | `ad4c3329a67f6b9ce1d5ab0f205f04b97c6758a41fd97ffb0fdcc603fb99a694` |
| `deployment/go2/offboard/run_policy_tunnel.sh` | `eb65fb3c88c0976b17ddc87ee99e6481e6d4d0c718cc7121630446f76006c2c3` |
| `deployment/go2/offboard/stop_offboard_stack.sh` | `e6b239f1cd2c51d59bd09c57348e037697a7bd4de47c0c9316860c608ed798c3` |

The four Jetson payloads form content-addressed release
`d656b9d9ae30de73f1d70a52b0150318f3dda238d6631dbae42f0a98dec973c2`.

## Deliberate exclusions

- model checkpoints and weight caches;
- Conda/venv environments and compiled dependencies;
- MemNav causal buffers and service logs;
- captured goal RGB/depth, Go2 reference poses and result JSON;
- robot/network credentials and SSH private keys;
- simulator datasets and unrelated diagnostics.

Runtime exclusions are enforced by `.gitignore`. Immutable releases remain
on the robot but `deployment/go2/releases/` is not committed.
