import json
from pathlib import Path
import re


REPO = Path(__file__).resolve().parents[3]
EXTENSION = REPO / "deployment/go2/foxglove/operator-controls"
LAYOUT = REPO / "deployment/go2/config/navdp_debug.foxglove-layout.json"
WORKFLOW = REPO / ".github/workflows/sync-foxglove-layout.yml"


def test_layout_panel_identity_matches_extension_registration():
    manifest = json.loads((EXTENSION / "package.json").read_text(encoding="utf-8"))
    index_source = (EXTENSION / "src/index.ts").read_text(encoding="utf-8")
    panel_name = re.search(r'name: "([a-z0-9-]+)"', index_source)
    assert panel_name is not None

    panel_type = f"{manifest['name']}.{panel_name.group(1)}"
    panel_id = f"{panel_type}!operate"
    layout = json.loads(LAYOUT.read_text(encoding="utf-8"))
    panels = layout["configById"]

    assert manifest["displayName"] == "MemNav Operator Controls"
    assert panels[panel_id]["foxglovePanelTitle"] == "Controls"
    operate = panels["Tab!navdp"]["tabs"][0]["layout"]
    assert operate["second"]["second"]["second"] == panel_id


def test_operator_controls_expose_only_fixed_fail_closed_services():
    source = (EXTENSION / "src/OperatorControls.tsx").read_text(encoding="utf-8")
    service_names = set(re.findall(r'"(/[a-z0-9_/-]+)"', source))

    assert service_names == {
        "/memnav_operator/capture_goal",
        "/memnav_operator/start_survey",
        "/memnav_operator/stop_survey",
        "/memnav_operator/start_revisit",
        "/memnav_operator/operator_stop",
        "/navdp/operator/revisit_workflow",
    }
    assert 'label: "CAPTURE GOAL"' in source
    assert 'label: "START SURVEY"' in source
    assert 'label: "STOP SURVEY"' in source
    assert 'label: "REVISIT"' in source
    assert 'label: "STOP NAVIGATION"' in source
    assert '"CONFIRM"' not in source
    assert "confirmingRevisit" not in source
    assert "payload.operator_summary" in source
    assert "SEAL SURVEY" not in source
    assert "set_enabled" not in source
    assert "clear_estop" not in source


def test_episode_capture_is_full_rgbd_but_phase_gated_to_survey_and_revisit():
    source = (
        REPO / "deployment/go2/offboard/experiment_capture.sh"
    ).read_text(encoding="utf-8")
    operator = (
        REPO / "deployment/go2/revisit_operator_service.py"
    ).read_text(encoding="utf-8")
    logger = (
        REPO / "deployment/go2/experiment_topic_logger.py"
    ).read_text(encoding="utf-8")

    assert "--allow-observer" in source
    assert "/camera/camera/color/image_raw" in source
    assert "/camera/camera/aligned_depth_to_color/image_raw" in source
    assert "/camera/camera/depth/metadata" in source
    assert "/navdp/go2/battery" in source
    assert "/navdp/operator/episode_event" in source
    assert "/navdp/foxglove/arrival/compressed" in source
    assert "/navdp/image_goal|/navdp/rgb_arrival_debug" in source
    assert "'$root/rosbag/survey'" in source
    assert "'$root/rosbag/$segment'" in source
    assert '"/navdp/operator/episode_event"' in logger
    assert '"/navdp/operator/revisit_workflow"' in logger
    goal_capture = operator[
        operator.index("def _run_goal_capture"):operator.index("def _start_survey")
    ]
    survey_start = operator[
        operator.index("def _run_survey_start"):operator.index("def _stop_survey")
    ]
    survey_stop = operator[
        operator.index("def _run_survey_stop"):operator.index("def _start_revisit")
    ]
    revisit = operator[
        operator.index("def _run_transaction"):operator.index("def close")
    ]
    assert "capture_start_command" not in goal_capture
    assert "capture_start_command" in survey_start
    assert "capture_pause_command" in survey_stop
    assert "capture_resume_command" in revisit
    assert revisit.index("capture_resume_command") < revisit.index("navigation_command")
    assert revisit.index("_finish_capture") < revisit.index("_cleanup_stack")


def test_revisit_supervisor_is_installed_as_an_idle_boot_service():
    installer = (
        REPO / "deployment/go2/scripts/install_boot_observer.sh"
    ).read_text(encoding="utf-8")
    unit = (
        REPO / "deployment/go2/systemd/memnav-revisit-operator.service"
    ).read_text(encoding="utf-8")

    assert "memnav-revisit-operator.service" in installer
    assert "run_revisit_operator_service.sh" in unit
    assert "ExecStop=" in unit
    assert "Restart=always" in unit


def test_ci_packages_and_uploads_foxe_before_updating_layout():
    workflow = WORKFLOW.read_text(encoding="utf-8")
    package_step = workflow.index("npm run package")
    upload_step = workflow.index("https://api.foxglove.dev/v1/extension-upload")
    verify_step = workflow.index("- name: Verify active organization extension")
    layout_step = workflow.index("- name: Create or update organization layout")

    assert package_step < upload_step < verify_step < layout_step
    assert "actions/upload-artifact@v4" in workflow
    assert "Content-Type: application/octet-stream" in workflow
    assert "GITHUB_RUN_ATTEMPT" in workflow
    assert "deployment/go2/foxglove/operator-controls/**" in workflow
    assert 'extension.get("activeVersion") == expected_version' in workflow
