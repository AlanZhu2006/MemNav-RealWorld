# MemNav Operator Controls

Compact React controls for the Foxglove `Operate` tab. The panel exposes four
operator actions backed by fixed, fail-closed ROS 2 services:

- `START SURVEY` calls `/navdp_go2_adapter/survey_start`.
- `STOP SURVEY` calls `/navdp_go2_adapter/survey_seal`.
- `REVISIT` requires a second safety-confirmation click, then calls
  `/memnav_operator/start_revisit` to validate the sealed Survey, restart the
  fixed Full-Mono stack, run fresh-plan/clearance checks and supervise return.
- `STOP NAVIGATION` calls both the persistent Revisit supervisor and the active
  adapter, cancelling pending preparation as well as stopping motion.

The display name deliberately says **Stop Survey**; `survey_seal` remains the
internal service contract because it pauses, validates, and seals the captured
Survey dataset. The extension never calls a service on mount. The Revisit
confirmation is the explicit onsite assertion that the area is clear, the
operator holds the Unitree controller, and the emergency stop is ready.

## Local build

```bash
npm ci
npm run lint
npm run package
```

`npm run package` writes a `.foxe` file in this directory. The organization
workflow assigns a unique CI version, uploads that file to Foxglove, and only
then updates the tracked organization layout.
