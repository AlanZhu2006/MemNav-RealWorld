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
    "EXPERIMENT_DATA_COLLECTION.md",
    "REALWORLD_EVALUATION.md",
    "SOURCE_MANIFEST.md",
    "THIRD_PARTY_NOTICES.md",
    "media/go2_showcase.jpg",
    "media/system_architecture.png",
    "media/demo/revisit_reference_third_view.mp4",
    "media/demo/revisit_reference_third_view.gif",
    "media/demo/revisit_reference_dashboard.mp4",
    "media/demo/revisit_reference_dashboard.gif",
    "manifests/realworld_deployment_v1.json",
    "manifests/realworld_fullmono_v2.json",
    "manifests/realworld_fullmono_v3.json",
    "manifests/realworld_fullmono_v4.json",
    "manifests/realworld_evaluation_plan_v1.json",
    "FULL_MONO_RELEASE_20260821.md",
    "deployment/go2/navdp_ros_node.py",
    "deployment/go2/navdp_client.py",
    "deployment/go2/terminal_motion_override.py",
    "deployment/go2/offboard/runtime_contract.sh",
    "deployment/go2/go2_cmd_bridge.py",
    "deployment/go2/offboard/run_offboard_stack.sh",
    "deployment/go2/offboard/experiment_capture.sh",
    "deployment/go2/experiment_capture_manifest.py",
    "deployment/go2/experiment_topic_logger.py",
    "deployment/gpu/realworld_cec_hub.py",
    "deployment/gpu/monocular_depth_runtime.py",
    "deployment/gpu/revisit_bearing_adapter.py",
    "deployment/gpu/revisit_local_pose_adapter.py",
    "deployment/gpu/audit_visual_convergence.py",
    "deployment/gpu/env.example",
    "tools/transcode_demo_media.sh",
    "tools/build_demo_previews.py",
)

FORBIDDEN_SUFFIXES = (".ckpt", ".pth", ".pt", ".pyc", ".download")
FORBIDDEN_PARTS = ("__pycache__", ".venv-navdp", ".cache", "runtime")
ALLOWED_GOAL_FILE = "deployment/go2/goals/.gitkeep"
EXPECTED_HASHES = {
    "deployment/gpu/realworld_cec_hub.py": (
        "1964c64e171b1e9976dad666df8c82be364182ca23a90e87161b4a7dd1f60be6"
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
        "ded9824071dd022a914260283972a8995d86d2feb59a3fb8384a69a9d3d88e6e"
    ),
    "deployment/go2/offboard/runtime_contract.sh": (
        "bfa64b010a335e5bd1528c6033a636773d4631d443631da7f4c5e0d135858f97"
    ),
    "media/go2_showcase.jpg": (
        "a7b5a226e3e89d08aa04d932a4531dce7b2593e4a5d7e2693b5997f89652cd08"
    ),
    "media/system_architecture.png": (
        "1dd30e72de523a4a14dc307dc7f0522687fa2c82f85ca9368873d9b9ce172298"
    ),
    "media/demo/revisit_reference_third_view.mp4": (
        "6f91aa9b9f95fb47ebc1529a24e055d37b087463b5e9ba7290fa642502dd8819"
    ),
    "media/demo/revisit_reference_third_view.gif": (
        "eb4238dd4f2920d6c4d85857dbb0274f5a6457fab1ce7e917ac3cd9cd1d913b5"
    ),
    "media/demo/revisit_reference_dashboard.mp4": (
        "1aa1496da2517fcb2fe656c56a9097d2294948802d2fd1853ed0b8c10f40d7e0"
    ),
    "media/demo/revisit_reference_dashboard.gif": (
        "cec03c51334f2a397f3c78462e750c881ae27244a7425705658a5fadd9e67df9"
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
    capture_source = (
        root / "deployment/go2/offboard/experiment_capture.sh"
    ).read_text()
    capture_manifest_source = (
        root / "deployment/go2/experiment_capture_manifest.py"
    ).read_text()
    check(
        "/navdp/cec_receipt" in capture_source
        and "ros2 bag record" in capture_source
        and "gst-launch-1.0" in capture_source
        and "attach-third-view" in capture_source,
        "dual-view experiment capture is declared",
        failures,
    )
    check(
        "motion_authority_changed_by_capture" in capture_manifest_source
        and '"motion_authority_changed_by_capture": False' in capture_manifest_source
        and "artifact_inventory" in capture_manifest_source
        and "sha256_file" in capture_manifest_source,
        "capture evidence is hash-bound without motion authority",
        failures,
    )

    evaluation_plan = json.loads(
        (root / "manifests/realworld_evaluation_plan_v1.json").read_text()
    )
    evaluation_shape = evaluation_plan.get("expected_shape", {})
    evaluation_scenes = evaluation_plan.get("scenes", [])
    evaluation_trials = [
        trial
        for scene in evaluation_scenes
        for trial in scene.get("trials", [])
    ]
    check(
        evaluation_plan.get("status") == "planned_no_formal_runs"
        and evaluation_shape
        == {"scenes": 4, "trials_per_scene": 5, "total_episodes": 20}
        and len(evaluation_scenes) == 4
        and all(len(scene.get("trials", [])) == 5 for scene in evaluation_scenes),
        "four-scene five-repeat evaluation is registered",
        failures,
    )
    check(
        evaluation_plan.get("claims", {}).get("formal_realworld_sr_spl")
        is False
        and evaluation_plan.get("claims", {}).get(
            "autonomous_imagegoal_arrival_verified"
        )
        is False
        and evaluation_plan.get("claims", {}).get("completed_formal_runs") == 0
        and all(trial.get("status") == "planned" for trial in evaluation_trials)
        and all(trial.get("success") is None for trial in evaluation_trials)
        and all(trial.get("spl") is None for trial in evaluation_trials),
        "planned evaluation contains no fabricated SR/SPL",
        failures,
    )

    print(f"\nPublic baseline verification: failures={len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
