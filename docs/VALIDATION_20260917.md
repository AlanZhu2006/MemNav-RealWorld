# Repository consolidation checks — 2026-09-17

Completed without connecting to or moving the robot:

- Python syntax checks for the Go2, GPU and Odin deployment directories.
- Syntax checks for changed Shell scripts and JSON files.
- Imports of configuration, dataset, hub, resident lifecycle and Survey cache
  modules in `memnav-realworld`, with the configuration CLI's `deployment/`
  import path included.
- Validation of the tracked system configuration through
  `runtime_config.py system-shell`.
- `verify_memnav_depth_reuse.py` against the current navigation source,
  without changing its pinned hashes.
- Entry-document local links and `git diff --check`.

No unit tests or navigation experiments were run. The default experiment JSON
references a machine-local historical goal image absent from this checkout;
resolving a run requires the selected Episode's actual goal and dataset.
No replacement image was fabricated for a launch check.

The August `verify_public_baseline.py` retains historical payload hashes and
wire schema. It is documented as a snapshot verifier; its expected hashes were
not regenerated to match current code. Its executable-bit finding was repaired
for `park_policy_stack.sh`.

This update includes the existing runtime and Survey changes, current entry
documentation, dated copies of the former entry documents and storage recovery
notes. Machine paths and production defaults remain as configured. No service
was deployed or restarted.
