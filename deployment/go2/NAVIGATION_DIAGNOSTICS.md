# Navigation attribution receipts

These fields are observation-only. They do not authorize motion, implement
collision avoidance, or certify that the memory map is correct.

The existing `/navdp/cec_receipt` logger automatically saves each plan's
`receipt.navigation_diagnostics`. The mixed-goal policy server also returns
`receipt.pointgoal_diagnostic` through the hub unchanged. Restart both the
Jetson adapter and GPU NavDP server after deploying this schema.

| Layer | Evidence |
| --- | --- |
| Memory | Existing `memory_unbounded_pointgoal`, its units, fixed-radius `memory_controller_pointgoal`, anchor, certificate, and direct-localization evidence |
| Policy input | `pointgoal_diagnostic.received_xyz` versus `processed_xyz`, including forward-component clipping |
| Local plan | Full selected XY path, all postprocessing candidate endpoints/lengths/critic scores, lookahead bearing and its signed difference from the memory bearing |
| Observation | Input RGB/depth ROS stamps, authoritative pair source stamp and source age, pair reception in ROS and monotonic time, plan completion in monotonic time, and subsampled center/left/right/bottom depth statistics |
| Control | `/navdp/status`: `plan_monotonic_s`, `target_command_before_safety`, terminal override, measured trajectory progress and heading error, action phase/reason, published `cmd_vx/cmd_wz`, stop reason and latest `rgbd_diagnostic` |

Use the plan's input stamps to retrieve the exact RGB and depth frames from a
full rosbag. Use its completion time to join status/command records. Calculate
pair reception minus sensor stamp only in the ROS clock; calculate processing
and execution intervals only in the shared Jetson monotonic clock.

Ordinary trajectory execution freezes each full local path in `go2_odom`,
using the SportModeState XY position and yaw aligned to the input RGB exposure.
At 20 Hz the controller projects measured position onto that path, advances
lookahead, and recomputes velocity in the current body frame. It stops when
both remaining arc length and endpoint distance are within 8 cm. That is an
endpoint tolerance, not a repeated displacement budget. There is no 10 cm,
10 degree, or 0.80 second action slicing. Acceleration limits smooth startup;
speed tapers near the path endpoint. The experiment-wide run timeout remains.

`/navdp/go2/odometry` uses locally received SportModeState with increasing
robot source timestamps; the robot clock is not synchronized to Jetson time.
Missing/stale position or discontinuities fail closed. This local estimator
is used only for path execution, not policy inference or independent GT.
`status.trajectory_execution` reports measured progress, remaining distance,
endpoint distance, phase and feedback age. After completion, an explicit zero
and 0.15 second settle precede admission of a newly captured RGB-D frame.
Queued pre-stop images cannot start the next plan. No previous velocity is
held through inference, and in-flight results crossing execution/stop boundaries
are discarded. The frozen 2.5 m memory bearing is not used as a path endpoint.

Runtime RGB-D reception or source age above 2.0 seconds pauses with zero
commands, abandoning the current path/turn. `status.rgbd_recovery` records the
pending pause, cause, elapsed time and threshold. Only a post-stop RGB-D pair
and newly accepted plan clear this pause. The supervisor logs `WAIT-RGBD` and
does not treat the deliberately abandoned old plan as a trajectory timeout.
Foxglove displays `PAUSED · WAIT RGB-D`; local feedback execution does not show
an intentionally held plan as a policy freshness error. No second authorization
is needed within this still-active run. Stop, estop, feedback faults, inference
errors and the experiment-wide timeout still terminate motion. Initial arming
retains its 0.60 second freshness check; this change is the runtime pause limit.

Rear-goal turns use a separate continuous body-heading feedback controller.
The bridge publishes `/navdp/go2/body_heading` from `LowState.imu_state.rpy[2]`
at up to 50 Hz, stamped on local DDS reception. The adapter aligns that yaw
to the plan's RGB capture stamp (within 150 ms), latches target yaw once, and
commands pure rotation at 20 Hz. Angular speed is proportional to remaining
measured error, bounded by the experiment limit. It stops within 8 degrees,
then waits for post-stop RGB-D before planning again. No translational creep,
command-integrated turn completion, or 10-degree pulse slicing is used here.
Yaw feedback older than 350 ms, a yaw discontinuity, or 20 seconds without
completion locks motion. Existing estop and RGB-D/depth validity checks apply.

`status.heading_turn` records target/error yaw, feedback age, phase and
completion age. This IMU is a local control input, not metric localization or
ground truth for the paper. Camera/IMU alignment is based on DDS reception,
not calibrated hardware synchronization. The recorder saves this topic.
Normal zero-velocity pauses call nonblocking `Move(0,0,0)`; explicit bridge
release/timeout/shutdown still uses `StopMove`, with a 200 ms RPC timeout.
The bridge applies no angular velocity floor that could alter deceleration.

An angular discrepancy is a diagnostic signal, not automatically a planning
error: a safe local path can legitimately deviate from the target direction.
Depth sector statistics describe optical Z in metres, not camera-to-body
clearance. They use stride-4 samples and valid depths in (0.05, 5] metres;
missing depth is represented explicitly and never treated as free space.

Limitations: candidates are recorded after policy postprocessing; pre-zeroing
candidates are not available. The fixed 2.5 m PointGoal radius is not measured
distance-to-goal. Certifying map direction/scale requires an independent pose
reference or a calibrated surveyed scene. Software command receipts alone do
not prove physical motion or identify a contact instant. No automatic recovery
is authorized by these diagnostics.
