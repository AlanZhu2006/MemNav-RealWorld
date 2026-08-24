#!/usr/bin/env bash

# Read the executor's actual schema from source with Python's stdlib AST. Do
# not duplicate the value in launch scripts: a partially copied Jetson tree
# must fail before camera or adapter startup.
cec_local_terminal_schema() {
  local go2_dir="$1"
  python3 - "$go2_dir/terminal_motion_override.py" <<'PY'
import ast
from pathlib import Path
import sys

path = Path(sys.argv[1])
tree = ast.parse(path.read_text(), filename=str(path))
for node in tree.body:
    if not isinstance(node, (ast.Assign, ast.AnnAssign)):
        continue
    targets = node.targets if isinstance(node, ast.Assign) else [node.target]
    if not any(
        isinstance(target, ast.Name)
        and target.id == "EXPECTED_HANDOFF_SCHEMA"
        for target in targets
    ):
        continue
    value = ast.literal_eval(node.value)
    if not isinstance(value, str) or not value:
        raise SystemExit("invalid EXPECTED_HANDOFF_SCHEMA")
    print(value)
    raise SystemExit(0)
raise SystemExit("EXPECTED_HANDOFF_SCHEMA not found")
PY
}

cec_validate_health_contract() {
  local payload="$1"
  local go2_dir="$2"
  local expected_schema
  expected_schema="$(cec_local_terminal_schema "$go2_dir")" || return 1
  python3 - "$payload" "$expected_schema" <<'PY'
import json
import sys

p = json.loads(sys.argv[1])
expected_schema = sys.argv[2]
assert p.get("algo") == "cec_hybrid_navdp"
assert p.get("protocol_version") == 3
assert p.get("navigation_sensor_contract") == "causal_monocular_rgb_v1"
assert p.get("navdp_depth_source") == "monocular_sidecar"
assert p.get("metric_depth_sensor_consumed_by_policy") is False
assert p.get("terminal_handoff_schema") == expected_schema
PY
}
