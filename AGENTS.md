# MemNav-RealWorld working and operation rules

These instructions apply to `/home/unitree/MemNav-RealWorld` and its deployed
counterpart on `work-pc`.

## Validation policy

- Do not create, restore, or run unit tests unless the user explicitly asks for
  them in a later request. Repository-owned unit-test files have been removed.
- Use syntax/import checks, existing configuration and data-integrity tools,
  documented preflight, and motion-locked observation as appropriate.
- Keep experiment evaluation, data capture, and hardware diagnostic tools. They
  are not disposable unit tests. Preserve runtime data unless its exact removal
  is authorized.
- Report what was actually verified. Static checks and stationary telemetry do
  not prove navigation success. Never initiate motion just to validate code.

## Motion authorization: no repeated confirmation

- A direct user request to start Revisit/navigation is the motion authorization
  for that run. A Foxglove `REVISIT` click has the same meaning. Carry forward
  the user's already-stated onsite safety, controller, and emergency-stop
  readiness; do not ask for a second confirmation or a fixed authorization
  phrase merely because preparation took time or code was synchronized.
- Perform the existing automated preflight before motion. Authorization does
  not override actual sensor, feedback, connection, or emergency-stop faults.
  If evidence contradicts the stated onsite conditions, stop and explain the
  concrete issue rather than issuing a generic reconfirmation request.
- Requests to edit code, commit, push, synchronize, capture a goal, Prepare, or
  record Survey do not implicitly authorize autonomous motion.
- A user Stop request cancels the current motion authorization. Keep the robot
  locked until the user issues a new motion request; do not auto-resume after a
  fault or deploy merely because an earlier run was authorized.
- Exception explicitly requested by the user: a temporary RGB-D freshness pause
  inside an active run preserves that run's authorization. At age >2 seconds,
  command zero, discard the old action, wait for post-stop RGB-D and a new
  accepted plan, then continue. This never clears estop or re-enables a stopped
  run. Hard faults and the existing overall run timeout still terminate it.
- `nav_stack.sh start` remains observation-only: `enabled=false`, `estop=true`.
  Do not change this default. Only use `run`, clear estop, enable execution, or
  publish motion commands within a user-requested motion run.

## Deployment identity and preservation

- Jetson host: `unitree-dog`, Orin NX 16 GB, Ubuntu 22.04, L4T R36.4.3,
  ROS 2 Humble, JetPack 6.2.1 user-space components.
- Preserve machine-local paths, hostname, and interface in
  `deployment/config/system.json`. Verify actual deployed revisions with Git;
  do not reset a newer checkout to a historical revision.
- Support workspaces live under `/home/unitree/.local/share/memnav`:
  CycloneDDS, Unitree SDK2 Python, Tinynav, RealSense ROS, message_filters, Odin.
- Runtime evidence, checkpoints, and resolved contracts live under `runtime/`.
  Use the current run's resolved contract; do not substitute a historical hash.
- NavDP uses `.venv-navdp` and NVIDIA PyTorch/CUDA 12.6.
- D435i validated stream: aligned RGB-D 848x480x30, USB SuperSpeed;
  librealsense/realsense-ros 2.58.1, firmware 5.17.0.10.
- Go2 link: `enP8p1s0`, local `192.168.123.164/24`, robot `192.168.123.161`.
  Prefer observation-only SDK telemetry for link checks.
- Odin1 native 0.14 is installed; do not claim live hardware validation until
  USB device `2207:0019` is actually observed.
- `ssh work-pc` accesses the RTX 4090 with a dedicated key. Preserve unrelated
  pre-existing `cec-realworld` GPU sessions; ports may already be occupied.
  GPU checkout: `/home/asus/Research/MemNav-RealWorld`.
- Last documented Jetson power mode: 15 W, four CPU cores. Verify current
  state when relevant. Switching to 25 W/MAXN requires explicit confirmation
  that power delivery and cooling are adequate.

## Canonical locked operation

Before starting the stack, check `rs-enumerate-devices`, USB SuperSpeed, and
`ping 192.168.123.161`. These are automated checks, not conversational approval
steps. Immediately after a locked start, verify `/navdp/status` reports
`enabled:false` and `estop:true` before proceeding. An offline Go2 may be
observed in camera-only mode but cannot begin navigation.

```bash
source /opt/ros/humble/setup.bash
bash deployment/go2/nav_stack.sh start --config deployment/config/experiments/native_imagegoal.json --refresh
bash deployment/go2/nav_stack.sh status --config deployment/config/experiments/native_imagegoal.json
bash deployment/go2/nav_stack.sh stop --config deployment/config/experiments/native_imagegoal.json
```

Use the active episode's documented offboard workflow and resolved contract
for Revisit; the native example above is not a replacement for that workflow.
