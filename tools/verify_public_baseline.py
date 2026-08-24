#!/usr/bin/env python3
"""Read-only repository checks for the public real-world deployment."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import stat
import subprocess
import sys


REQUIRED_PATHS = (
    "README.md",
    "ARCHITECTURE.md",
    "CURRENT_STATUS.md",
    "RUNBOOK.md",
    "SOURCE_MANIFEST.md",
    "THIRD_PARTY_NOTICES.md",
    "media/go2_showcase.jpg",
    "media/system_architecture.svg",
    "manifests/realworld_deployment_v1.json",
    "manifests/realworld_fullmono_v2.json",
    "manifests/realworld_fullmono_v3.json",
    "manifests/realworld_fullmono_v4.json",
    "FULL_MONO_RELEASE_20260821.md",
    "deployment/go2/navdp_ros_node.py",
    "deployment/go2/navdp_client.py",
    "deployment/go2/terminal_motion_override.py",
    "deployment/go2/offboard/runtime_contract.sh",
    "deployment/go2/go2_cmd_bridge.py",
    "deployment/go2/offboard/run_offboard_stack.sh",
    "deployment/gpu/realworld_cec_hub.py",
    "deployment/gpu/monocular_depth_runtime.py",
    "deployment/gpu/revisit_bearing_adapter.py",
    "deployment/gpu/revisit_local_pose_adapter.py",
    "deployment/gpu/audit_visual_convergence.py",
    "deployment/gpu/env.example",
)

FORBIDDEN_SUFFIXES = (".ckpt", ".pth", ".pt", ".pyc", ".download")
FORBIDDEN_PARTS = ("__pycache__", ".venv-navdp", ".cache", "runtime")
ALLOWED_GOAL_FILE = "deployment/go2/goals/.gitkeep"
EXPECTED_HASHES = {
    "deployment/gpu/realworld_cec_hub.py": (
        "501858a5961f844098ad20d26e4a28763ac29937b37e3a9501c1caead746ee61"
    ),
    "deployment/gpu/monocular_depth_runtime.py": (
        "9b88cbd091b83dbe15846ec0b47d329d715273f0557abffe319a463936c9c138"
    ),
    "deployment/gpu/revisit_bearing_adapter.py": (
        "46c10132db7b00711ca3c781f18fcb9e04c4061bab9b44b8017d99c0c09bc6fd"
    ),
    "deployment/gpu/revisit_local_pose_adapter.py": (
        "ab58913fff760182b1945d1a26c5dbb2bba58f040d38c28f030b19fc1bc569cd"
    ),
    "deployment/gpu/audit_visual_convergence.py": (
        "807ba6b1ba3a9395ce0a89fbe79b368276479efb92c56be18ac4332b6b0f4af7"
    ),
    "deployment/go2/terminal_motion_override.py": (
        "1a0ea960c36e231d4424c1a3837d7b3cf88dce0ef7d4737068d371bfa888054e"
    ),
    "deployment/go2/navdp_client.py": (
        "1d4cc28b7c8a5d9d864d0443bc9ab32c0e7124a19b39e2481ff41de6d5fefcf9"
    ),
    "deployment/go2/offboard/runtime_contract.sh": (
        "bfa64b010a335e5bd1528c6033a636773d4631d443631da7f4c5e0d135858f97"
    ),
    "media/go2_showcase.jpg": (
        "a7b5a226e3e89d08aa04d932a4531dce7b2593e4a5d7e2693b5997f89652cd08"
    ),
    "media/system_architecture.svg": (
        "741f627a7557d1dd7c1018790702eb8528bd47904114d5ef265a97ae157783f2"
    ),
}


def tracked_files(root: Path) -> tuple[str, ...]:
    result = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=root,
        check=True,
        capture_output=True,
    )
    return tuple(
        item.decode("utf-8")
        for item in result.stdout.split(b"\0")
        if item
    )


def check(condition: bool, label: str, failures: list[str]) -> None:
    if condition:
        print(f"[PASS] {label}")
    else:
        print(f"[FAIL] {label}")
        failures.append(label)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    args = parser.parse_args()
    root = args.workspace.resolve()
    failures: list[str] = []

    check((root / ".git").is_dir(), "Git checkout", failures)
    for relative in REQUIRED_PATHS:
        check((root / relative).is_file(), f"required: {relative}", failures)

    for relative, expected in EXPECTED_HASHES.items():
        digest = hashlib.sha256((root / relative).read_bytes()).hexdigest()
        check(digest == expected, f"SHA-256: {relative}", failures)

    try:
        tracked = tracked_files(root)
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"[FAIL] cannot inspect tracked files: {error}")
        return 1

    forbidden_runtime = [
        path
        for path in tracked
        if path != ALLOWED_GOAL_FILE
        and (
            path.startswith("deployment/go2/goals/")
            or path.endswith(FORBIDDEN_SUFFIXES)
            or any(part in Path(path).parts for part in FORBIDDEN_PARTS)
            or path == "deployment/gpu/.env"
        )
    ]
    check(not forbidden_runtime, "no tracked runtime artifacts", failures)
    if forbidden_runtime:
        for path in forbidden_runtime:
            print(f"       {path}")

    scripts = tuple(
        path
        for path in tracked
        if path.startswith("deployment/") and path.endswith(".sh")
    )
    non_executable = [
        path
        for path in scripts
        if not ((root / path).stat().st_mode & stat.S_IXUSR)
    ]
    check(not non_executable, "deployment scripts are executable", failures)
    if non_executable:
        for path in non_executable:
            print(f"       {path}")

    manifest = json.loads(
        (root / "manifests/realworld_fullmono_v4.json").read_text()
    )
    check(
        manifest["motion"]["enabled_on_start"] is False,
        "motion defaults disabled",
        failures,
    )
    check(
        manifest["motion"]["max_linear_mps"] == 0.3,
        "published default speed is 0.30 m/s",
        failures,
    )
    check(
        manifest["claims"]["formal_realworld_sr_spl"] is False
        and manifest["claims"]["autonomous_imagegoal_arrival_verified"] is False,
        "formal result boundary is explicit",
        failures,
    )
    navigation = manifest["navigation_contract"]
    check(
        navigation["protocol_version"] == 3
        and navigation["sensor_contract"] == "causal_monocular_rgb_v1",
        "protocol-v3 monocular sensor contract",
        failures,
    )
    phases = navigation.get("episode_phases", [])
    check(
        phases == ["memory_recording", "revisit_query"],
        "two-phase episode contract is declared",
        failures,
    )
    check(
        navigation.get("terminal_handoff_schema")
        == "cec_direct_bearing_handoff_v2_20260824"
        and navigation.get("metric_pnp_translation_authority")
        == "diagnostic_only"
        and navigation.get("stop_authority")
        == "none_until_independent_visual_convergence",
        "bearing-v2 authority boundary is declared",
        failures,
    )
    check(
        manifest["runtime_compatibility"]["partial_file_copy"]
        == "cannot_start",
        "partial terminal-runtime synchronization fails closed",
        failures,
    )
    check(
        navigation["policy_depth_source"] == "monocular_sidecar"
        and navigation["metric_depth_sensor_consumed_by_policy"] is False,
        "policy cannot consume metric sensor depth",
        failures,
    )
    check(
        manifest["robot"]["camera_optical_center_height_m"] == 0.42
        and manifest["deployment_gate"]["status"]
        == "motion_locked_pending_arrival_calibration",
        "arrival-calibration gate remains closed",
        failures,
    )

    adapter_config = (
        root / "deployment/go2/config/navdp_go2.yaml"
    ).read_text()
    bridge_script = (
        root / "deployment/go2/scripts/run_go2_bridge.sh"
    ).read_text()
    hub_source = (
        root / "deployment/gpu/realworld_cec_hub.py"
    ).read_text()
    check("enable_on_start: false" in adapter_config, "adapter lock", failures)
    check("max_linear_mps: 0.30" in adapter_config, "adapter speed", failures)
    check('GO2_TIMEOUT_SEC="${GO2_TIMEOUT_SEC:-0.35}"' in bridge_script,
          "bridge watchdog", failures)
    check('GO2_MAX_VX="${GO2_MAX_VX:-0.30}"' in bridge_script,
          "bridge speed", failures)
    check(
        'args.host not in {"127.0.0.1", "::1", "localhost"}' in hub_source,
        "hub rejects non-loopback bind",
        failures,
    )
    check(
        "PROTOCOL_VERSION = 3" in hub_source
        and 'PHASE_RECORDING = "memory_recording"' in hub_source
        and "/begin_revisit" in hub_source
        and "/memory_step" in hub_source
        and "/goal_candidate" in hub_source,
        "hub implements the protocol-v3 phase contract",
        failures,
    )
    check(
        "memory_replay_step" in hub_source
        and "queue length mismatch" in hub_source,
        "hub performs verified NavDP warm-up at begin_revisit",
        failures,
    )
    terminal_source = (
        root / "deployment/go2/terminal_motion_override.py"
    ).read_text()
    client_source = (root / "deployment/go2/navdp_client.py").read_text()
    contract_source = (
        root / "deployment/go2/offboard/runtime_contract.sh"
    ).read_text()
    check(
        'EXPECTED_HANDOFF_SCHEMA = "cec_direct_bearing_handoff_v2_20260824"'
        in terminal_source
        and "EXPECTED_TERMINAL_HANDOFF_SCHEMA" in client_source
        and "terminal_handoff_schema" in contract_source,
        "hub/executor schema handshake is implemented",
        failures,
    )

    print(f"\nPublic baseline verification: failures={len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
