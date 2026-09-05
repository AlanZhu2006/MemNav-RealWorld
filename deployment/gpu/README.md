# RTX 4090 Policy Stack

The RTX side runs the loopback-only MemNav, frozen NavDP and CEC hub services.
It is an internal half of the Full-Mono stack; normal operation starts it from
the Jetson:

```bash
bash deployment/go2/nav_stack.sh start \
  --config deployment/config/experiments/fullmono_imagegoal.json
```

There is no `deployment/gpu/.env`. Licensed external source/checkpoint paths,
ports, Python, runtime root and the measured camera height are tracked in
`deployment/config/system.json`. The Jetson resolves that file with the
experiment file, records a `config_id`, copies the exact resolved JSON to
`runtime/config/` on this machine and requires matching Git revisions before
startup.

The GPU leaf commands accept only that resolved file:

```bash
bash deployment/gpu/scripts/preflight.sh --config runtime/config/CONFIG_ID.json
bash deployment/gpu/scripts/run_policy_stack.sh --config runtime/config/CONFIG_ID.json
bash deployment/gpu/scripts/stop_policy_stack.sh --config runtime/config/CONFIG_ID.json
```

The licensed MemNav source remains in its separate research worktree. Apply
the tracked current-frame depth-reuse patch once after checking out or updating
that worktree; the GPU preflight then verifies the applied patch on every start:

```bash
bash deployment/gpu/scripts/apply_memnav_source_patch.sh \
  --config runtime/config/CONFIG_ID.json
```

The patch reuses the depth tensor already produced by the post-warmup flow
gate for the same frame-bound MDTEC transaction. It also exposes append,
retrieval, depth-prediction and depth-materialization timings in the runtime
receipt. It never reuses depth across frame indices or RGB hashes.

Ports `8888`, `18888` and `18889` bind to loopback. Do not expose them on the
LAN; the Jetson launcher owns the SSH local forward. These services have no
Unitree dependency and no actuator path.

Tests are non-motion:

```bash
/home/asus/miniconda3/envs/memnav-realworld/bin/python -m pytest -q \
  deployment/gpu/tests
```
