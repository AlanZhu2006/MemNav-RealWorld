# Real-World Experiment Data Collection

Snapshot: **2026-08-30**

This workflow records one NavDP/Go2 trial as a synchronized evidence bundle:

1. ROS 2 MCAP data for policy state, CEC receipts, trajectories, commands,
   RGB arrival output and safety state;
2. line-oriented JSON copies of `/navdp/status`, `/navdp/cec_receipt`,
   `/navdp/rgb_arrival_status` and the experiment start/stop events;
3. an operator-workstation recording of the live Foxglove dashboard, imported
   byte-for-byte after the run;
4. an externally recorded third-person video, also imported byte-for-byte;
5. a frozen manifest containing the Git revision, configuration identity,
   outcome, file sizes and SHA-256 values.

The collector is observational. It does not publish velocity, enable the
adapter, clear estop or call the Unitree SDK.

## Evidence roles

| Evidence | Capture authority | Purpose |
| --- | --- | --- |
| RTX episodic dataset | Full-Mono hub | Exact causal memory RGBs, excluded goal candidates and their immutable manifest |
| ROS 2 MCAP | Jetson collector | Policy/status/control/RGB-arrival timeline; optional full RGB-D replay |
| Receipt JSONL | Jetson collector | Human-readable phase, goal-selection, CEC and arrival-audit events |
| Foxglove dashboard video | Operator workstation | First-person camera, goal, visual match, aligned safety depth, paths and live state |
| Third-view master | External camera/operator | Physical motion, contacts, operator intervention and scene outcome |
| Capture manifest | Jetson collector | Binds all artifacts to one run ID and Git revision with SHA-256 |
| Optional Odin1 reference | Independent GT sidecar | Stable relocalization, metric goal region, odometry `P_i`, frozen A* `L_i` and SPL receipt; never a policy input |

The third-view camera is intentionally independent of the robot. The Jetson
cannot start a phone recording, so the operator starts it immediately after
the collector and performs one visible sync clap. ROS `/navdp/experiment_event`
records exact software-side `START` and `STOP` UTC events.

## One-time preflight

Start the formal stack with the headless Foxglove Bridge first. Motion remains
locked:

~~~bash
cd /home/nvidia/twork/MemNav-RealWorld
# Set launch.foxglove=true in fullmono_imagegoal.json first.
bash deployment/go2/offboard/revisit_experiment.sh formal-start DATASET_ID

bash deployment/go2/offboard/experiment_capture.sh preflight
~~~

Preflight requires the Foxglove Bridge node, the ROS 2 MCAP storage plugin,
`tmux` and the required ROS topics. It does not require X11/VNC, start a
dashboard recording or touch motion authority.

## Start a formal capture

Use one unique run ID across the dashboard, third-view camera, arrival output
and final notes:

~~~bash
bash deployment/go2/offboard/experiment_capture.sh start \
  revisit_scene01_trial01 \
  --dataset DATASET_ID \
  --trial-kind revisit \
  --profile audit \
  --gt-source none
~~~

The recommended `audit` profile records:

- `/navdp/status`, `/navdp/cec_receipt` and `/navdp/rgb_arrival_status`;
- selected/candidate paths, command velocity, enable/estop state and Go2 state;
- ImageGoal, RGB-arrival debug image, camera calibration and Foxglove markers.

The collector is intentionally headless. On the operator workstation, connect
Foxglove to `ws://JETSON_IP:8765`, import
`deployment/go2/config/navdp_debug.foxglove-layout.json`, and start a screen
recording before motion authorization.

The dashboard layout uses display-only JPEG previews: RGB is 640x360 at 15 Hz,
and aligned depth is colorized over 200--4000 mm at 640x360 and 10 Hz. The
ImageGoal repeats at 2 Hz and arrival debug is capped at 5 Hz as JPEG previews.
The original image topics remain unchanged for policy, arrival and the optional
`full` MCAP profile. Re-import the versioned layout after an upgrade; Foxglove
does not automatically replace a previously imported local copy. Do not expect
raw image panels to work remotely: the Bridge whitelist excludes all four raw
image topics so a stale layout cannot bypass the bandwidth limit.

It does not duplicate the raw camera stream because the RTX episodic dataset
already owns the exact causal RGB memory. Use `--profile full` only when raw
D435i RGB and aligned depth are required for an offline sensor replay. The full
profile can consume several gigabytes per minute and must be preceded by a
disk-bandwidth and free-space check.

For the optional independent Odin1 evaluation lane, start it first and wait
for `reference_ready=true`, then replace the last flag with
`--gt-source odin1`. This adds Odin odometry/cloud/path/TF and
`/navdp/gt/status` to the bag and JSONL logger. It does not expose Odin data to
NavDP. The full setup and run order are in `deployment/odin1_gt/README_CN.md`.

After the command reports `START`, begin the independent third-person camera
and make one visible sync clap. Only then arm the arrival module and perform the
separate onsite estop-clear and motion-enable procedure from the runbook.

## Stop and seal

First assert estop or complete the arrival module's terminal procedure. Then stop
the evidence processes gracefully:

~~~bash
bash deployment/go2/offboard/experiment_capture.sh stop \
  revisit_scene01_trial01

bash deployment/go2/offboard/experiment_capture.sh attach-dashboard \
  revisit_scene01_trial01 /path/from/operator/foxglove_dashboard.mp4

bash deployment/go2/offboard/experiment_capture.sh attach-third-view \
  revisit_scene01_trial01 /path/from/camera/third_view.mp4

# Required only when start used --gt-source odin1:
bash deployment/go2/offboard/experiment_capture.sh attach-odin-gt \
  revisit_scene01_trial01 \
  runtime/odin1_gt/formal/revisit_scene01_trial01/monitor/result.json \
  runtime/odin1_gt/formal/revisit_scene01_trial01/spl_receipt.json

bash deployment/go2/offboard/experiment_capture.sh finalize \
  revisit_scene01_trial01 success \
  --notes "operator-confirmed physical outcome; arrival report attached"

bash deployment/go2/offboard/experiment_capture.sh verify \
  revisit_scene01_trial01
~~~

`finalize` refuses a formal bundle unless all of the following exist:

- a closed MCAP rosbag with storage and `metadata.yaml`;
- a non-empty, byte-preserved Foxglove dashboard video;
- a byte-preserved imported third-view video;
- non-empty status and CEC JSONL logs.

An Odin-backed bundle additionally requires non-empty
`logs/odin_gt_status.jsonl`, `receipts/odin_gt_result.json` and
`receipts/odin_spl_receipt.json`. These files must be attached from the
hash-sealed Odin result/scorer; hand-written metrics do not satisfy the gate.

An aborted engineering run may use `--allow-incomplete`; the manifest then
records `formal_complete=false`. Missing evidence is never silently promoted
to a formal result.

## Runtime layout

By default every run is written under:

~~~text
runtime/go2/experiment_capture/<RUN_ID>/
├── manifest.json
├── MANIFEST.sha256
├── FINALIZED
├── rosbag/
├── logs/
│   ├── status.jsonl
│   ├── cec_receipt.jsonl
│   ├── rgb_arrival_status.jsonl
│   ├── odin_gt_status.jsonl            # when gt_source=odin1
│   └── experiment_event.jsonl
├── media/
│   ├── dashboard.mp4
│   └── third_view.mp4
└── receipts/
    ├── odin_gt_result.json             # when gt_source=odin1
    └── odin_spl_receipt.json           # when gt_source=odin1
~~~

Runtime evidence remains ignored by Git. Formal publication should copy only
the selected, independently reviewed derivatives into `media/demo/` and bind
their hashes in `media/README.md` or a dated experiment manifest.

## Browser-ready publication

The repository includes deterministic local helpers for the same publication
pattern used by TopoFocus Real-World: a browser H.264 MP4, a poster and an
accelerated inline GIF.

~~~bash
# Normalize a third-view phone master (example: H.265 portrait input).
bash tools/transcode_demo_media.sh \
  /path/to/third_view_master.mp4 /tmp/third_view_browser.mp4 \
  360 640 1200 15

.venv-navdp/bin/python tools/build_demo_previews.py \
  --video /tmp/third_view_browser.mp4 \
  --poster /tmp/third_view_poster.jpg \
  --gif /tmp/third_view_preview.gif \
  --width 360 --frames 48 --duration-s 8
~~~

The source master must remain separately archived. A README preview is display
evidence, not a substitute for the sealed runtime manifest or raw recording.
