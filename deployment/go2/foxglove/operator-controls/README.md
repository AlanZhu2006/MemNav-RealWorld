# MemNav Operator Controls

Compact React controls for the Foxglove `Operate` tab. The panel exposes only
three fail-closed ROS 2 service calls:

- `START SURVEY` calls `/navdp_go2_adapter/survey_start`.
- `STOP SURVEY` calls `/navdp_go2_adapter/survey_seal`.
- `STOP NAVIGATION` calls `/navdp_go2_adapter/operator_stop`.

The display name deliberately says **Stop Survey**; `survey_seal` remains the
internal service contract because it pauses, validates, and seals the captured
Survey dataset. The extension never calls a service on mount and contains no
motion-enabling control.

## Local build

```bash
npm ci
npm run lint
npm run package
```

`npm run package` writes a `.foxe` file in this directory. The organization
workflow assigns a unique CI version, uploads that file to Foxglove, and only
then updates the tracked organization layout.
