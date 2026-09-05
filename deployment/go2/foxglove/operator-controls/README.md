# MemNav Operator Controls

Compact React controls for the Foxglove `Operate` tab. The panel owns the full
repeatable Episode sequence through five fixed, fail-closed ROS 2 services:

- `CAPTURE GOAL` calls `/memnav_operator/capture_goal`. It creates a unique
  Episode/Dataset, freezes the current aligned RGB-D pair with both ROS sensor
  timestamps, displays the RGB frame as **Revisit Goal**, and starts full MCAP
  recording before the robot leaves that location.
- `START SURVEY` calls `/memnav_operator/start_survey`, which prepares the
  reusable RTX stack and begins causal RGB memory recording.
- `STOP SURVEY` calls `/memnav_operator/stop_survey`, which locks motion,
  validates the minimum history, seals the Dataset and stops the Survey stack.
- `REVISIT` directly calls
  `/memnav_operator/start_revisit` to validate the sealed Survey, restart the
  fixed Full-Mono stack, run fresh-plan/clearance checks and supervise return.
- `STOP` calls `/memnav_operator/operator_stop`, cancelling any
  stage, locking motion, closing the MCAP and hash-finalizing the Episode.

The Episode ID, stage and `REC RGB-D` state remain visible in the control panel.
Buttons are enabled only for the next valid state transition. The extension
never calls a service on mount and never accepts browser-provided file paths or
motion parameters. Clicking Revisit is the explicit onsite motion authorization;
the operator must already have a clear area, the Unitree controller in hand,
and the emergency stop ready. Structured service responses are reduced to a
short operator summary, so the status strip never displays raw JSON.

Each Episode stores lossless goal RGB/depth PNGs, exact RGB/depth header stamps,
an event timeline, the CEC Dataset identity and manifest hash, raw D435i color
plus aligned-depth MCAP, policy/safety/trajectory topics, receipts, final outcome
and a SHA-256 artifact inventory. A completed Episode can be followed by a new
`CAPTURE GOAL` without changing source or restarting the persistent operator.

## Local build

```bash
npm ci
npm run lint
npm run package
```

`npm run package` writes a `.foxe` file in this directory. The organization
workflow assigns a unique CI version, uploads that file to Foxglove, and only
then updates the tracked organization layout.
