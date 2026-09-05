# Changelog

## Unreleased

- Rename Stop Navigation to Stop and remove decorative icons from every action button.
- Show front clearance as Clear or Stop after removing the intermediate soft-slowdown state.

## 0.1.0

- Add compact React controls for Start Survey, Stop Survey, and Stop Navigation.
- Keep the panel limited to the existing fail-closed operator services.
- Add a guarded one-click Revisit action, live workflow status, and
  cancellation through Stop Navigation.
- Render concise operator summaries instead of raw service-response JSON.
- Add the complete Capture Goal → Survey → Revisit Episode state machine.
- Show the active Episode, workflow stage and full RGB-D recording state.
- Persist goal timestamps and hash-finalize onboard Episode evidence.
