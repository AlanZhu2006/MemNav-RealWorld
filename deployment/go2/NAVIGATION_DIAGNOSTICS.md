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
| Observation | Input RGB/depth ROS stamps, pair reception in ROS and monotonic time, plan completion in monotonic time, and subsampled center/left/right/bottom depth statistics |
| Control | `/navdp/status`: `plan_monotonic_s`, `target_command_before_safety`, terminal override, `latency_motion_guard` decision, actual `cmd_vx/cmd_wz`, stop reason and latest `rgbd_diagnostic` |

Use the plan's input stamps to retrieve the exact RGB and depth frames from a
full rosbag. Use its completion time to join status/command records. Calculate
pair reception minus sensor stamp only in the ROS clock; calculate processing
and execution intervals only in the shared Jetson monotonic clock.

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
