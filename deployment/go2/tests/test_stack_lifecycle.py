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


def test_run_fast_path_reuses_current_stack_and_forwards_timeout():
    command = f"""
source {NAV_STACK!s}
resolve_config() {{ echo /tmp/resolved.json; }}
load_jetson_config() {{
  CFG_PROFILE=native-navdp-rgbd
  CFG_CONFIG_ID=current-id
}}
native_session_is_current_and_healthy() {{ return 0; }}
start_stack() {{ echo unexpected-cold-start; return 99; }}
bash() {{ printf 'child=%s\n' "$*"; }}
run_navigation --timeout-s 45
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "FAST PATH: reusing current healthy stack config_id=current-id" in result.stdout
    assert "unexpected-cold-start" not in result.stdout
    assert "scripts/run_navigation.sh --config /tmp/resolved.json --timeout-s 45" in result.stdout


def test_start_fast_path_relocks_and_reuses_exact_healthy_stack():
    command = f"""
source {NAV_STACK!s}
resolve_config() {{ echo /tmp/resolved.json; }}
show_contract() {{
  CFG_PROFILE=native-navdp-rgbd
  CFG_CONFIG_ID=current-id
  CFG_NATIVE_SESSION=navdp-go2
}}
profile_session_is_current_and_healthy() {{ return 0; }}
tmux() {{ return 1; }}
bash() {{ printf 'child=%s\n' "$*"; }}
stop_existing_local_stack() {{ echo unexpected-stop; return 99; }}
start_stack --config /tmp/experiment.json
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "scripts/lock_running_stack.sh --config /tmp/resolved.json" in result.stdout
    assert "FAST START: reusing healthy native-navdp-rgbd stack" in result.stdout
    assert "unexpected-stop" not in result.stdout


def test_start_refresh_replaces_even_a_healthy_exact_stack():
    command = f"""
source {NAV_STACK!s}
resolve_config() {{ echo /tmp/resolved.json; }}
show_contract() {{
  CFG_PROFILE=native-navdp-rgbd
  CFG_CONFIG_ID=current-id
  CFG_NATIVE_SESSION=navdp-go2
}}
profile_session_is_current_and_healthy() {{ echo unexpected-health-check; return 99; }}
tmux() {{ return 1; }}
stop_existing_local_stack() {{ printf 'stop=%s\n' "$*"; }}
bash() {{ printf 'child=%s\n' "$*"; }}
start_stack --config /tmp/experiment.json --refresh
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "unexpected-health-check" not in result.stdout
    assert "stop=/tmp/resolved.json navdp-go2 native-navdp-rgbd" in result.stdout
    assert "scripts/run_stack.sh --config /tmp/resolved.json" in result.stdout


def test_run_cold_path_refreshes_once_before_agent():
    command = f"""
source {NAV_STACK!s}
resolve_config() {{ echo /tmp/resolved.json; }}
load_jetson_config() {{
  CFG_PROFILE=native-navdp-rgbd
  CFG_CONFIG_ID=expected-id
}}
native_session_is_current_and_healthy() {{ return 1; }}
start_stack() {{ printf 'cold=%s\n' "$*"; }}
bash() {{ printf 'child=%s\n' "$*"; }}
run_navigation --config /tmp/experiment.json --timeout-s 30
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "COLD PATH:" in result.stdout
    assert "cold=--config /tmp/experiment.json" in result.stdout
    assert "scripts/run_navigation.sh --config /tmp/resolved.json --timeout-s 30" in result.stdout


def test_native_session_health_checks_every_required_window_for_dead_panes():
    command = f"""
source {NAV_STACK!s}
CFG_NATIVE_SESSION=navdp-go2
CFG_CONFIG_ID=expected-id
CFG_WITH_CAMERA=true
CFG_ARRIVAL_MODULE=rgb-homography
CFG_WITH_GO2=true
CFG_WITH_FOXGLOVE=true
tmux() {{
  case "$1" in
    has-session) return 0 ;;
    show-environment) echo MEMNAV_CONFIG_ID=expected-id ;;
    list-windows) printf '%s\n' \
      'policy 0' 'rgbd 0' 'adapter 0' 'camera-recovery 0' \
      'arrival 0' 'go2 0' 'battery 0' 'fox-preview 0' 'foxglove 1' ;;
  esac
}}
if native_session_is_current_and_healthy; then
  echo unexpectedly-healthy
else
  echo correctly-unhealthy
fi
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.stdout.strip() == "correctly-unhealthy"


def test_fullmono_health_accepts_healthy_reused_observer_without_duplicate_windows():
    command = f"""
source {NAV_STACK!s}
CFG_FULLMONO_SESSION=navdp-go2-offboard
CFG_CONFIG_ID=expected-id
CFG_TUNNEL_LOCAL_PORT=18889
CFG_WITH_CAMERA=true
CFG_ARRIVAL_MODULE=rgb-homography
CFG_WITH_GO2=true
CFG_WITH_FOXGLOVE=true
tmux() {{
  case "$1" in
    has-session) return 0 ;;
    show-environment)
      if [[ "$4" == MEMNAV_USES_BOOT_OBSERVER ]]; then
        echo MEMNAV_USES_BOOT_OBSERVER=true
      else
        echo MEMNAV_CONFIG_ID=expected-id
      fi ;;
    list-windows) printf '%s\n' \
      'tunnel 0' 'adapter 0' 'camera-recovery 0' 'arrival 0' 'go2 0' ;;
  esac
}}
navdp_boot_observer_is_healthy() {{ return 0; }}
curl() {{ return 0; }}
if fullmono_session_is_current_and_healthy; then
  echo healthy
else
  echo unhealthy
fi
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.stdout.strip() == "healthy"


def test_fullmono_reused_observer_health_fails_when_systemd_plane_is_down():
    command = f"""
source {NAV_STACK!s}
CFG_FULLMONO_SESSION=navdp-go2-offboard
CFG_CONFIG_ID=expected-id
CFG_TUNNEL_LOCAL_PORT=18889
CFG_WITH_CAMERA=true
CFG_ARRIVAL_MODULE=none
CFG_WITH_GO2=false
CFG_WITH_FOXGLOVE=true
tmux() {{
  case "$1" in
    has-session) return 0 ;;
    show-environment)
      if [[ "$4" == MEMNAV_USES_BOOT_OBSERVER ]]; then
        echo MEMNAV_USES_BOOT_OBSERVER=true
      else
        echo MEMNAV_CONFIG_ID=expected-id
      fi ;;
    list-windows) printf '%s\n' 'tunnel 0' 'adapter 0' 'camera-recovery 0' ;;
  esac
}}
navdp_boot_observer_is_healthy() {{ return 1; }}
curl() {{ return 0; }}
if fullmono_session_is_current_and_healthy; then
  echo unexpectedly-healthy
else
  echo correctly-unhealthy
fi
"""

    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.stdout.strip() == "correctly-unhealthy"


def test_invalid_run_timeout_is_rejected_before_cold_start():
    command = f"""
source {NAV_STACK!s}
start_stack() {{ echo unexpected-cold-start; }}
run_navigation --timeout-s invalid
"""
    result = subprocess.run(
        ["bash", "-c", command],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "--timeout-s must be a positive number" in result.stderr
    assert "unexpected-cold-start" not in result.stdout
