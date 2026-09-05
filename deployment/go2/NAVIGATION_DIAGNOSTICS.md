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
| Control | `/navdp/status`: `plan_monotonic_s`, `target_command_before_safety`, terminal override, action phase/reason/integrated translation and heading, actual `cmd_vx/cmd_wz`, stop reason and latest `rgbd_diagnostic` |

Use the plan's input stamps to retrieve the exact RGB and depth frames from a
full rosbag. Use its completion time to join status/command records. Calculate
pair reception minus sensor stamp only in the ROS clock; calculate processing
and execution intervals only in the shared Jetson monotonic clock.

Ordinary trajectory control uses event-driven stop-plan-act execution. Its action clock
starts with the first nonzero command actually published, not with inference
completion. The command stops after 0.10 m integrated translation, 10 degrees
integrated heading, or a final 0.80 s wall-clock bound. After the explicit zero
and 0.15 s settle interval, a new plan can use only an RGB-D pair whose sensor
capture timestamp is newer than the zero-command timestamp. A queued pre-stop
frame cannot qualify merely because its callback arrived late, and the previous
command is never held through inference.

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
