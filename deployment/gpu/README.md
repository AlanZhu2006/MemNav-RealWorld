# RTX 4090 Policy Stack

This directory contains the public protocol-v2 Full-Mono dual-machine seam:

- <code>realworld_cec_hub.py</code>: single stateful ImageGoal endpoint;
- <code>monocular_depth_runtime.py</code>: frozen mono-depth payload and scale-receipt contract;
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
The external MemNav source must expose <code>/monocular_depth_query</code> and
protocol-v2 CEC receipts. The file is ignored by Git.

## Start

~~~bash
export CEC_CAMERA_HEIGHT_M=<physically-measured-metres>
bash deployment/gpu/scripts/run_policy_stack.sh
curl -fsS http://127.0.0.1:18889/healthz
tmux attach -t cec-realworld
~~~

There is no camera-height default. A health payload is accepted only when it
proves <code>causal_monocular_rgb_v1</code>,
<code>navdp_depth_source=monocular_sidecar</code>, and no metric policy depth.

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
