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

Ports `8888`, `18888` and `18889` bind to loopback. Do not expose them on the
LAN; the Jetson launcher owns the SSH local forward. These services have no
Unitree dependency and no actuator path.

Tests are non-motion:

```bash
/home/asus/miniconda3/envs/memnav-realworld/bin/python -m pytest -q \
  deployment/gpu/tests
```
