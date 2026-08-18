# RTX 4090 Policy Stack

This directory contains the public dual-machine control seam:

- <code>realworld_cec_hub.py</code>: single stateful ImageGoal endpoint;
- <code>revisit_bearing_adapter.py</code>: verified scale-free bearing to frozen 2.5 m PointGoal;
- <code>scripts/</code>: loopback-only MemNav, NavDP and hub launchers;
- <code>tests/</code>: fallback, takeover and fail-closed contract tests.

The MemNav model implementation and weights are external research artifacts.
They are not copied into this repository.

## Configure

~~~bash
cp deployment/gpu/env.example deployment/gpu/.env
nano deployment/gpu/.env
bash deployment/gpu/scripts/preflight.sh
~~~

Every path in <code>.env</code> must refer to a locally licensed artifact.
The file is ignored by Git.

## Start

~~~bash
bash deployment/gpu/scripts/run_policy_stack.sh
curl -fsS http://127.0.0.1:18889/healthz
tmux attach -t cec-realworld
~~~

Ports <code>8888</code>, <code>18888</code> and <code>18889</code> bind to
loopback. Do not expose them on the LAN; use the Jetson SSH local-forward
script.

## Stop

~~~bash
bash deployment/gpu/scripts/stop_policy_stack.sh
~~~

## Test

~~~bash
python3 -m pip install -r deployment/gpu/requirements.txt pytest
python3 -m pytest -q deployment/gpu/tests
~~~

Tests use fake HTTP upstreams and never contact a robot.
