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
    "deployment/go2/navdp_ros_node.py",
    "deployment/go2/go2_cmd_bridge.py",
    "deployment/go2/offboard/run_offboard_stack.sh",
    "deployment/gpu/realworld_cec_hub.py",
    "deployment/gpu/revisit_bearing_adapter.py",
    "deployment/gpu/env.example",
)

FORBIDDEN_SUFFIXES = (".ckpt", ".pth", ".pt", ".pyc", ".download")
FORBIDDEN_PARTS = ("__pycache__", ".venv-navdp", ".cache", "runtime")
ALLOWED_GOAL_FILE = "deployment/go2/goals/.gitkeep"
EXPECTED_HASHES = {
    "deployment/gpu/realworld_cec_hub.py": (
        "17603f6e86d3ae94eabebeda3bde84ac3efb229184a7e142b2d3e88e2295dc5a"
    ),
    "deployment/gpu/revisit_bearing_adapter.py": (
        "46c10132db7b00711ca3c781f18fcb9e04c4061bab9b44b8017d99c0c09bc6fd"
    ),
    "media/go2_showcase.jpg": (
        "a7b5a226e3e89d08aa04d932a4531dce7b2593e4a5d7e2693b5997f89652cd08"
    ),
    "media/system_architecture.svg": (
        "8d7989b2a7f2eedad3b026c68bdefd0492c2576cf1ac4252efa3550a43700211"
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
        (root / "manifests/realworld_deployment_v1.json").read_text()
    )
    services = manifest["policy_workstation"]["services"]
    check(
        all(service["bind"] == "127.0.0.1" for service in services.values()),
        "GPU services are loopback-only",
        failures,
    )
    check(
        manifest["policy_workstation"]["direct_actuator_authority"] is False,
        "workstation has no actuator authority",
        failures,
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
        manifest["validation"]["formal_realworld_sr_spl"] == "not_claimed",
        "formal result boundary is explicit",
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

    print(f"\nPublic baseline verification: failures={len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
