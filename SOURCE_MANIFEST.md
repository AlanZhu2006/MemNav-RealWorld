# Source and Artifact Manifest

## Repository Base

- Upstream repository: <code>https://github.com/InternRobotics/NavDP</code>
- Imported base commit: <code>878740a2011856d0e3782dd6ccd880fd2eccd70f</code>
- Base commit date: <code>2026-07-31T13:43:54+08:00</code>
- Base history is retained rather than copied as an anonymous source dump.

## Real-World Overlay

| Path | Provenance |
| --- | --- |
| <code>deployment/go2/</code> | Jetson Orin NX / D435i / Unitree Go2 integration developed and exercised in this workspace |
| <code>deployment/gpu/realworld_cec_hub.py</code> | Selected from the 2026-08-18 RTX workstation working tree, then package import generalized |
| <code>deployment/gpu/revisit_bearing_adapter.py</code> | Frozen verified-bearing controller boundary selected from the same working tree |
| <code>deployment/gpu/scripts/</code> | Public path-parameterized launch form of the observed workstation scripts |
| <code>deployment/go2/offboard/</code> | SSH-forward and offboard Jetson launch overlay synchronized during the 2026-08-18 dry-run |

The workstation source snapshot was an uncommitted research working tree. It
therefore has no honest source Git object to cite. This repository records that
boundary instead of inventing provenance. The selected router and adapter are
covered by this repository's Git history after import.

Selected release SHA-256 values:

| File | SHA-256 |
| --- | --- |
| <code>deployment/gpu/realworld_cec_hub.py</code> | <code>17603f6e86d3ae94eabebeda3bde84ac3efb229184a7e142b2d3e88e2295dc5a</code> |
| <code>deployment/gpu/revisit_bearing_adapter.py</code> | <code>46c10132db7b00711ca3c781f18fcb9e04c4061bab9b44b8017d99c0c09bc6fd</code> |
| <code>media/go2_showcase.jpg</code> | <code>a7b5a226e3e89d08aa04d932a4531dce7b2593e4a5d7e2693b5997f89652cd08</code> |
| <code>media/system_architecture.svg</code> | <code>8d7989b2a7f2eedad3b026c68bdefd0492c2576cf1ac4252efa3550a43700211</code> |

## External Runtime Requirements

The public repository is not model-artifact complete. The GPU MemNav service
expects the operator to provide:

- a compatible MemNav research workspace and server;
- MemNav and NavDP checkpoints;
- InternNav diffusion-policy source;
- LingBot Map source and weights;
- an official compatible LightGlue checkout;
- the locally assembled Python dependency root.

All locations are configured in <code>deployment/gpu/.env</code>; the template
contains no credentials.

The Jetson additionally requires ROS 2 Humble, the RealSense ROS driver,
CycloneDDS, Unitree SDK2 Python and the model checkpoints named in
<code>deployment/go2/README_CN.md</code>.

## Deliberate Exclusions

- model checkpoints and weight caches;
- Conda/venv environments and compiled dependencies;
- MemNav causal buffers and service logs;
- captured goal RGB/depth, Go2 reference poses and experiment result JSON;
- robot/network credentials and SSH private keys;
- datasets, simulator assets and unrelated research diagnostics.

Runtime exclusions are enforced by <code>.gitignore</code>. The goal directory
retains only <code>deployment/go2/goals/.gitkeep</code>.
