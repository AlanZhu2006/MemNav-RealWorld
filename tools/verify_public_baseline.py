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
    "REALWORLD_EXPERIMENT_HANDBOOK_CN.md",
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
    "manifests/realworld_paired_evaluation_plan_v2.json",
    "manifests/odin1_gt_reference_v1.json",
    "docs/README.md",
    "docs/archive/FULL_MONO_RELEASE_20260821.md",
    "deployment/go2/navdp_ros_node.py",
    "deployment/go2/navdp_client.py",
    "deployment/go2/terminal_motion_override.py",
    "deployment/go2/offboard/runtime_contract.sh",
    "deployment/go2/go2_cmd_bridge.py",
    "deployment/go2/offboard/run_offboard_stack.sh",
    "deployment/go2/offboard/experiment_capture.sh",
    "deployment/go2/experiment_capture_manifest.py",
    "deployment/go2/experiment_topic_logger.py",
    "deployment/odin1_gt/README_CN.md",
    "deployment/odin1_gt/odin_gt_core.py",
    "deployment/odin1_gt/odin_gt_monitor.py",
    "deployment/odin1_gt/odin_occupancy_builder.py",
    "deployment/odin1_gt/score_odin_gt.py",
    "deployment/odin1_gt/make_driver_config.py",
    "deployment/odin1_gt/make_scene_contract.py",
    "deployment/odin1_gt/config/go2_odin_mount_receipt.template.json",
    "deployment/odin1_gt/config/odin_gt.rviz",
    "deployment/odin1_gt/scripts/odin_gt.sh",
    "deployment/odin1_gt/scripts/setup_driver.sh",
    "deployment/odin1_gt/scripts/preflight.sh",
    "deployment/odin1_gt/vendor/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch",
    "deployment/odin1_gt/vendor/odin_ros_driver_runtime_config.patch",
    "deployment/gpu/realworld_cec_hub.py",
    "deployment/gpu/monocular_depth_runtime.py",
    "deployment/gpu/revisit_bearing_adapter.py",
    "deployment/gpu/revisit_local_pose_adapter.py",
    "deployment/gpu/audit_visual_convergence.py",
    "deployment/gpu/env.example",
    "tools/transcode_demo_media.sh",
    "tools/build_demo_previews.py",
    "tools/verify_realworld_paired_campaign.py",
)

FORBIDDEN_SUFFIXES = (".ckpt", ".pth", ".pt", ".pyc", ".download")
FORBIDDEN_PARTS = ("__pycache__", ".venv-navdp", ".cache", "runtime")
ALLOWED_GOAL_FILE = "deployment/go2/goals/.gitkeep"
EXPECTED_HASHES = {
    "deployment/gpu/realworld_cec_hub.py": (
        "58c3f23c568adb7d2997d05cb429cda507c50fd96bf103294ac31370eece62bb"
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
        "423ff3b22cd94f192a0aa47e2c3cb2277b3b50c817f82546213eb9f052bbf0ef"
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
    "deployment/odin1_gt/vendor/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch": (
        "2a73aa48d163e2a362670b7b9b778edf8328aba7323e1cc04dd6b8fb28ba5806"
    ),
    "deployment/odin1_gt/vendor/odin_ros_driver_runtime_config.patch": (
        "953bd96ad3cea5c336f11882f92a428ff090ba13abd28c742314f072cd637f86"
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
    check(
        "--gt-source" in capture_source
        and "/navdp/gt/status" in capture_source
        and "attach-odin-gt" in capture_source
        and "odin_gt_result" in capture_manifest_source
        and "odin_spl_receipt" in capture_manifest_source,
        "Odin reference evidence is integrated into capture manifests",
        failures,
    )

    odin_manifest = json.loads(
        (root / "manifests/odin1_gt_reference_v1.json").read_text()
    )
    odin_authority = odin_manifest.get("authority", {})
    odin_claims = odin_manifest.get("claims", {})
    odin_driver = odin_manifest.get("driver", {})
    check(
        odin_manifest.get("status") == "implemented_not_hardware_validated_on_go2"
        and odin_manifest.get("classification")
        == "independent_reference_slam_not_metrological_ground_truth"
        and odin_authority.get("policy_input") is False
        and odin_authority.get("motion_authority") is False
        and odin_authority.get("evaluation_only") is True,
        "Odin lane remains evaluation-only and honestly classified",
        failures,
    )
    check(
        odin_driver.get("default_profile") == "native_0_14"
        and odin_driver.get("native_0_14", {}).get("commit")
        == "6f993ccc4ccad9395bfc68bc3235f993d83c4fe6"
        and odin_driver.get("native_0_14", {}).get("mode1_bootstrap_patch")
        is None
        and odin_driver.get("legacy_0_13_1", {}).get("default") is False,
        "Odin 0.14 native Mode1 is default; 0.13.1 patch is legacy-only",
        failures,
    )
    check(
        odin_claims.get("odin1_connected_during_this_release") is False
        and odin_claims.get("go2_mount_calibrated") is False
        and odin_claims.get("formal_sr_spl_available") is False
        and all(
            value is None
            for value in odin_manifest.get("mandatory_field_freeze", {}).values()
        ),
        "Odin hardware/calibration/result claims remain unfilled",
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

    paired_plan = json.loads(
        (root / "manifests/realworld_paired_evaluation_plan_v2.json").read_text()
    )
    paired_shape = paired_plan.get("expected_shape", {})
    paired_scenes = paired_plan.get("scenes", [])
    paired_blocks = [
        block
        for scene in paired_scenes
        for block in scene.get("paired_blocks", [])
    ]
    paired_runs = [
        run
        for block in paired_blocks
        for run in block.get("runs", {}).values()
    ]
    expected_methods = {"mono_native", "mono_cec"}
    check(
        paired_plan.get("status") == "planned_no_formal_runs"
        and paired_plan.get("supersedes") == "realworld_evaluation_plan_v1.json"
        and paired_shape == {
            "scenes": 4,
            "paired_blocks_per_scene": 5,
            "paired_blocks": 20,
            "rollouts_per_block": 2,
            "total_rollouts": 40,
            "native_first_blocks": 10,
            "cec_first_blocks": 10,
        }
        and len(paired_scenes) == 4
        and len(paired_blocks) == 20
        and len(paired_runs) == 40
        and all(set(block.get("runs", {})) == expected_methods
                for block in paired_blocks)
        and sum(block.get("order", [None])[0] == "mono_native"
                for block in paired_blocks) == 10
        and sum(block.get("order", [None])[0] == "mono_cec"
                for block in paired_blocks) == 10,
        "conference campaign is registered as 20 balanced paired blocks",
        failures,
    )
    check(
        paired_plan.get("claims", {}).get("formal_realworld_sr_spl")
        is False
        and paired_plan.get("claims", {}).get(
            "autonomous_imagegoal_arrival_verified"
        ) is False
        and paired_plan.get("claims", {}).get("completed_paired_blocks") == 0
        and paired_plan.get("claims", {}).get("completed_rollouts") == 0
        and all(run.get("status") == "planned" for run in paired_runs)
        and all(run.get("success") is None for run in paired_runs)
        and all(run.get("spl") is None for run in paired_runs),
        "paired campaign contains no fabricated SR/SPL",
        failures,
    )

    print(f"\nPublic baseline verification: failures={len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
