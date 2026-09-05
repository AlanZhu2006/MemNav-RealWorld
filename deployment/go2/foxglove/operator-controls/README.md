# MemNav Operator Controls

Compact React controls for the Foxglove `Operate` tab. The panel owns the full
repeatable Episode sequence through fixed ROS 2 services:

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
  It does not declare experiment failure.
- Once capture is saved, the row offers `MARK SUCCESS` and `MARK FAILURE`.
  These call `/memnav_operator/review_success` and `/memnav_operator/review_failure`
  for the displayed Episode. They only annotate data and never change motion.
  A manual Stop can be marked successful. Labels can be corrected; each revision
  retains the earlier judgment and its timestamp.

The Episode ID, stage and `REC RGB-D` state remain visible in the control panel.
Buttons are enabled only for the next valid state transition. The extension
never calls a service on mount and never accepts browser-provided file paths or
motion parameters. Clicking Revisit is the explicit onsite motion authorization;
the operator must already have a clear area, the Unitree controller in hand,
and the emergency stop ready. A direct user request to the assistant to start
Revisit also authorizes that run; do not require a second confirmation phrase.
Automatic preflight and fault stops still apply. Preparation alone does not arm
the robot, and Stop requires a new start request before resuming.
Structured service responses are reduced to a
short operator summary, so the status strip never displays raw JSON.

Each Episode stores lossless goal RGB/depth PNGs, exact RGB/depth header stamps,
an event timeline, the CEC Dataset identity and manifest hash, raw D435i color
plus aligned-depth MCAP, policy/safety/trajectory topics, receipts, stop reason
and a SHA-256 artifact inventory. A completed Episode can be followed by a new
`CAPTURE GOAL` without changing source or restarting the persistent operator.

Automatic finalization writes `outcome=unreviewed` and a separate
`termination_reason`, including `automatic_arrival`, `operator_stop`, `timeout`
or `navigation_error`. Arrival and CEC are control evidence, not experiment
success labels. Human judgments live in `evaluation.json`, bound to the frozen
manifest SHA with reviewer, notes and revision history. That annotation is
explicitly outside the immutable sensor artifact inventory, so re-labeling does
not rehash or rewrite multi-GB recordings. Old records are not relabeled on upgrade.

For a previous run or a separately recorded retry, use its exact capture path:

```bash
python3 deployment/go2/experiment_capture_manifest.py review \
  --run-root runtime/go2/experiment_capture/RUN_ID \
  --outcome success --reviewer operator --notes 'Manual stop at the goal'
```

Use `failure` or `unreviewed` to correct/clear a label. An annotation is not
independent ground truth: formal Odin SR/SPL validation remains separate, and
disagreements are reported instead of rewriting the GT result.

## Local build

```bash
npm ci
npm run lint
npm run package
```

`npm run package` writes a `.foxe` file in this directory. The organization
workflow assigns a unique CI version, uploads that file to Foxglove, and only
then updates the tracked organization layout.
