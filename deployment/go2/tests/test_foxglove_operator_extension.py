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


def test_operator_controls_expose_only_the_three_fixed_fail_closed_services():
    source = (EXTENSION / "src/OperatorControls.tsx").read_text(encoding="utf-8")
    service_names = set(re.findall(r'serviceName: "([^"]+)"', source))

    assert service_names == {
        "/navdp_go2_adapter/survey_start",
        "/navdp_go2_adapter/survey_seal",
        "/navdp_go2_adapter/operator_stop",
    }
    assert 'label: "START SURVEY"' in source
    assert 'label: "STOP SURVEY"' in source
    assert 'label: "STOP NAVIGATION"' in source
    assert "SEAL SURVEY" not in source
    assert "set_enabled" not in source
    assert "clear_estop" not in source


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
