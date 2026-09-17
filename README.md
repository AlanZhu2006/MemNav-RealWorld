# MemNav Real-World

Monocular image-goal and revisit navigation on a Unitree Go2. An RTX 4090 runs
streaming geometry, verified historical goal readout and frozen NavDP. The
Jetson Orin NX handles RGB capture, trajectory tracking and the local safety
layer. Sensor depth is used by that safety layer; both paired navigation arms
receive the same predicted monocular depth.

The deployment retains **CEC** and `mono_cec` as the configuration names for
GEM's memory guidance. **Baseline** means the offboard `mono_native` arm,
which uses the same Survey and depth pipeline with memory guidance disabled.
The Jetson-local RGB-D profile is a separate diagnostic entry point.

## Start here

- [Current implementation and evidence](CURRENT_STATUS.md)
- [Experiment handbook](REALWORLD_EXPERIMENT_HANDBOOK_CN.md)
- [Survey and Revisit workflow](TWO_PASS_REVISIT_RUNBOOK.md)
- [GPU services and configuration](deployment/gpu/README.md)
- [Memory storage integration and measured latency](REALWORLD_MEMORY_STORAGE_SYNC_20260916_CN.md)
- [Documentation index](docs/README.md)
- [Consolidation checks](docs/VALIDATION_20260917.md)

## Platform and workflow

<p align="center">
  <img src="media/go2_showcase.jpg" width="720" alt="Unitree Go2 with D435i and Jetson Orin NX">
</p>

| Component | Responsibility |
| --- | --- |
| RTX 4090 | Causal LingBot geometry, historical goal readout and frozen NavDP |
| Jetson Orin NX | RGB transport, replanning, local tracking and stopping |
| D435i depth | Local collision safety and recordings |
| Sealed Survey | Ordered RGB history shared by paired runs |

A Survey is recorded and sealed before query preparation. Preparation verifies
its manifest, reconstructs or restores the compatible geometric state, binds
the selected goal, and initializes NavDP's short observation buffer from the
query-start view. Subsequent RGB observations extend the stream. Accepted
historical geometry provides a goal direction; rejected queries use the native
ImageGoal controller.

The configuration resolver combines `deployment/config/system.json` with an
experiment JSON into an immutable runtime contract. Both machines use the
same resolved contract. Machine-local paths, ports and network interfaces stay
in the system configuration.

## Current storage options

The production defaults remain `legacy / dense / native`. The explicit
`native_interval7` option supports compact historical geometry and optional
lossless KV encoding, with Survey initialization by RGB replay. Legacy local
Survey checkpoints have their own compatibility and integrity checks.

The stationary Jetson-to-RTX replay measured median planning round trips of
388 ms with full maps, 336 ms with compact archives and 334 ms with compact
archives plus lossless KV. These timings begin with prepared JPEGs and include
geometry, goal readout, NavDP inference and the response. Detailed counts and
configuration comparisons are in the [measurement record](REALWORLD_MEMORY_STORAGE_SYNC_20260916_CN.md).

## Repository layout

| Path | Contents |
| --- | --- |
| `deployment/go2/` | Camera, ROS adapter, trajectory execution, Foxglove and stopping |
| `deployment/go2/offboard/` | Paired Episode preparation, SSH tunnel and capture |
| `deployment/gpu/` | Resident models, Survey cache, CEC hub and replay measurement |
| `deployment/runtime_config.py` | Configuration validation and immutable contracts |
| `deployment/odin1_gt/` | Optional independent reference/evaluation workflow |
| `baselines/navdp/` | Frozen NavDP integration |
| `runtime/` | Local datasets, resolved configs, recordings and experiment registry; ignored by Git |
| `docs/` | Current index and historical records |

The external geometry implementation is maintained in
[AlanZhu2006/Nav](https://github.com/AlanZhu2006/Nav), also synchronized to
[glbreeze/Nav](https://github.com/glbreeze/Nav). In the unified local workspace,
this repository lives at `projects/realworld`; its Git history remains separate.
Model weights, scenes and raw recordings remain outside Git.

## Checkout validation

```bash
python3 -m compileall -q deployment/go2 deployment/gpu deployment/odin1_gt
python3 deployment/gpu/verify_memnav_depth_reuse.py --source-root /path/to/Nav
```

For a resolved configuration, the GPU preflight checks source compatibility,
model paths and ports:

```bash
bash deployment/gpu/scripts/preflight.sh --config runtime/config/CONFIG_ID.json
```

The repository uses syntax, configuration and data-integrity checks rather than
a unit-test suite. `tools/verify_public_baseline.py` checks the frozen August
release described in [SOURCE_MANIFEST.md](SOURCE_MANIFEST.md); its pinned hashes
are historical, not the current checkout's verification target.

## Operation and evidence

Use the selected Episode's workflow in the runbook. Startup remains
`enabled=false, estop=true`; a Prepare or source sync does not start motion.
The Jetson retains final motion authority and the bridge watchdog. The RTX
services bind to loopback and are reached through the existing SSH tunnel.

The protected pair registry is local `runtime/go2/experiment_pairs/index.json`.
Capture manifests bind each run's configuration, goals, receipts and recordings.
See [EXPERIMENT_DATA_COLLECTION.md](EXPERIMENT_DATA_COLLECTION.md) for recording
and outcome provenance. The old four-scene evaluation plan remains a separate
historical protocol; current evidence is summarized in
[CURRENT_STATUS.md](CURRENT_STATUS.md).

Curated videos are indexed in [media/README.md](media/README.md). The previous
entry document is preserved as [README_20260907.md](README_20260907.md).

## Upstream

This project builds on [InternRobotics/NavDP](https://github.com/InternRobotics/NavDP).
Upstream and third-party license files are retained; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

```bibtex
@misc{navdp,
  title={NavDP: Learning Sim-to-Real Navigation Diffusion Policy with Privileged Information Guidance},
  author={Wenzhe Cai and Jiaqi Peng and Yuqiang Yang and Yujian Zhang and Meng Wei and Hanqing Wang and Yilun Chen and Tai Wang and Jiangmiao Pang},
  year={2025},
  booktitle={arXiv}
}
```
