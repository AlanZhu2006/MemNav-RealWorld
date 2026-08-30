import subprocess
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
NAV_STACK = REPO / "deployment/go2/nav_stack.sh"


def test_existing_native_session_is_replaced_with_its_active_contract(tmp_path):
    active_config = tmp_path / "active.json"
    active_config.write_text("{}\n", encoding="utf-8")
    command = f"""
source {NAV_STACK!s}
tmux() {{
  if [[ "$1" == has-session ]]; then
    return 0
  fi
  if [[ "$1" == show-environment ]]; then
    echo 'MEMNAV_RUN_CONFIG={active_config!s}'
    return 0
  fi
  return 1
}}
bash() {{ printf 'child=%s\n' "$*"; }}
stop_existing_local_stack /tmp/new.json navdp-go2 native-navdp-rgbd
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "Replacing the complete running stack: session=navdp-go2" in result.stdout
    assert f"old_config={active_config}" in result.stdout
    assert "new_config=/tmp/new.json" in result.stdout
    assert f"scripts/stop_stack.sh --config {active_config}" in result.stdout


def test_absent_session_needs_no_stop():
    command = f"""
source {NAV_STACK!s}
tmux() {{ return 1; }}
bash() {{ echo unexpected-child; return 99; }}
stop_existing_local_stack /tmp/new.json navdp-go2 native-navdp-rgbd
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.stdout == ""
