import base64
import json
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "deployment"))

import runtime_config as rc  # noqa: E402


PNG_1X1 = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUB"
    "AScY42YAAAAASUVORK5CYII="
)


def make_source_config(tmp_path: Path) -> Path:
    system = json.loads(
        (REPO / "deployment/config/system.json").read_text(encoding="utf-8")
    )
    system_path = tmp_path / "system.json"
    system_path.write_text(json.dumps(system), encoding="utf-8")
    goal = tmp_path / "goal.png"
    goal.write_bytes(PNG_1X1)
    experiment = json.loads(
        (REPO / "deployment/config/experiments/fullmono_imagegoal.json").read_text(
            encoding="utf-8"
        )
    )
    experiment["system_config"] = str(system_path)
    experiment["experiment"]["navigation"]["image_goal"] = str(goal)
    experiment["experiment"]["arrival"]["image_goal"] = str(goal)
    experiment_path = tmp_path / "experiment.json"
    experiment_path.write_text(json.dumps(experiment), encoding="utf-8")
    return experiment_path


def test_resolve_records_image_and_stable_hash(tmp_path):
    experiment = make_source_config(tmp_path)
    first = rc.resolve(experiment, tmp_path / "first.json")
    second = rc.resolve(experiment, tmp_path / "second.json")
    one = rc.load_resolved(first)
    two = rc.load_resolved(second)
    assert one["config_id"] == two["config_id"]
    assert one["navigation"]["image_goal"]["width"] == 1
    assert one["navigation"]["image_goal"]["height"] == 1
    assert one["navigation"]["image_goal"]["sha256"] == rc._sha256_file(
        tmp_path / "goal.png"
    )


def test_modified_resolved_config_is_rejected(tmp_path):
    path = rc.resolve(make_source_config(tmp_path), tmp_path / "resolved.json")
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["control"]["max_linear_mps"] = 99
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(rc.ConfigError, match="config_id mismatch"):
        rc.load_resolved(path)


def test_unknown_experiment_field_is_rejected(tmp_path):
    experiment = make_source_config(tmp_path)
    payload = json.loads(experiment.read_text(encoding="utf-8"))
    payload["experiment"]["forgotten_override"] = True
    experiment.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(rc.ConfigError, match="unknown experiment field"):
        rc.resolve(experiment, tmp_path / "resolved.json")


def test_missing_system_field_is_rejected(tmp_path):
    experiment = make_source_config(tmp_path)
    experiment_payload = json.loads(experiment.read_text(encoding="utf-8"))
    system_path = Path(experiment_payload["system_config"])
    system = json.loads(system_path.read_text(encoding="utf-8"))
    del system["sites"]["gpu"]["models"]["navdp_checkpoint"]
    system_path.write_text(json.dumps(system), encoding="utf-8")
    with pytest.raises(rc.ConfigError, match="missing GPU models field"):
        rc.resolve(experiment, tmp_path / "resolved.json")


def test_survey_and_formal_are_derived_from_one_contract(tmp_path):
    base = rc.resolve(make_source_config(tmp_path), tmp_path / "base.json")
    survey = rc._derive(
        base, tmp_path / "survey.json", "survey", "route_01", None
    )
    formal_root = tmp_path / "formal"
    goal = tmp_path / "goal.png"
    goal_sha256 = rc._sha256_file(goal)
    formal = rc._derive(
        base,
        tmp_path / "formal.json",
        "formal",
        "route_01",
        formal_root,
        scene_id="scene01",
        run_id="scene01_pair01_cec",
        authority_mode="cec",
        frozen_goal=goal,
        expected_goal_sha256=goal_sha256,
        expected_dataset_sha256="a" * 64,
    )
    survey_payload = rc.load_resolved(survey)
    formal_payload = rc.load_resolved(formal)
    assert survey_payload["dataset"]["auto_open"] is True
    assert survey_payload["memory"]["pause_recording"] is True
    assert survey_payload["launch"]["go2_bridge"] is False
    assert formal_payload["dataset"]["auto_open"] is False
    assert formal_payload["launch"]["go2_bridge"] is True
    assert formal_payload["cec"]["authority_mode"] == "cec"
    assert formal_payload["formal"]["expected_goal_sha256"] == goal_sha256
    assert formal_payload["navigation"]["selected_goal_image_path"] == str(
        formal_root / "selected_goal.jpg"
    )
    assert survey_payload["source"]["derived_from_config_id"] == rc.load_resolved(
        base
    )["config_id"]


def test_one_way_external_goal_debug_is_explicitly_labeled(tmp_path):
    base = rc.resolve(make_source_config(tmp_path), tmp_path / "base.json")
    survey = rc._derive(
        base,
        tmp_path / "survey.json",
        "survey",
        "route_debug_01",
        None,
        collection_mode="manual_one_way_external_goal_debug",
    )

    payload = rc.load_resolved(survey)

    assert payload["dataset"]["metadata"]["collection_mode"] == (
        "manual_one_way_external_goal_debug"
    )
    assert payload["dataset"]["metadata"]["candidate_contract"] == (
        "external_frozen_goal_only_no_survey_candidate"
    )
    assert payload["dataset"]["metadata"]["goal_selection_contract"] == (
        "operator_frozen_external_required"
    )
    assert payload["dataset"]["metadata"]["goal_candidates_required"] is False
    assert payload["launch"]["go2_bridge"] is False


def test_shell_contract_has_explicit_imagegoal_and_no_legacy_names(tmp_path):
    path = rc.resolve(make_source_config(tmp_path), tmp_path / "resolved.json")
    exports = rc.shell_exports(rc.load_resolved(path), "jetson")
    assert "CFG_IMAGE_GOAL=" in exports
    assert "CFG_IMAGE_GOAL_SHA256=" in exports
    assert "CFG_WITH_FOXGLOVE=true" in exports
    assert "CFG_FOXGLOVE_LAYOUT=" in exports
    assert "CFG_FOXGLOVE_PORT=8765" in exports
    assert "CFG_FOXGLOVE_PREVIEW_RGB_TOPIC=/navdp/foxglove/rgb/compressed" in exports
    assert "CFG_FOXGLOVE_PREVIEW_DEPTH_FPS=10" in exports
    assert "CFG_FOXGLOVE_PREVIEW_GOAL_TOPIC=/navdp/foxglove/goal/compressed" in exports
    assert "CFG_FOXGLOVE_PREVIEW_ARRIVAL_FPS=5" in exports
    assert "CFG_FOXGLOVE_PREVIEW_STATUS_TOPIC=/navdp/foxglove/status/compressed" in exports
    assert "CFG_FOXGLOVE_PREVIEW_STATUS_WIDTH=720" in exports
    assert "CFG_FOXGLOVE_PREVIEW_ARRIVAL_PRESERVE_RESOLUTION=true" in exports
    assert "CFG_WITH_RVIZ" not in exports
    assert "NAVDP_IMAGE_GOAL_PATH" not in exports
    assert "CEC_CAMERA_HEIGHT_M" not in exports


def test_runtime_config_uses_the_public_profile_catalog():
    assert set(rc.PROFILES) == {
        "native-navdp-rgbd",
        "fullmono-lingbot-cec",
    }
    assert set(rc.ARRIVAL_MODULES) == {
        "operator",
        "external-topic",
        "rgb-homography",
    }


def test_gpu_shell_contract_exposes_only_gpu_and_shared_fields(tmp_path):
    path = rc.resolve(make_source_config(tmp_path), tmp_path / "resolved.json")
    exports = rc.shell_exports(rc.load_resolved(path), "gpu")
    assert "CFG_GPU_PYTHON=" in exports
    assert "CFG_MEMNAV_CKPT=" in exports
    assert "CFG_AUTHORITY_MODE=" in exports
    assert "CFG_JETSON_PYTHON=" not in exports
    assert "CFG_UNITREE_NET_IF=" not in exports
    assert "CFG_IMAGE_GOAL=" not in exports


def test_foxglove_layout_limits_control_to_fail_closed_services():
    layout = json.loads(
        (
            REPO
            / "deployment/go2/config/navdp_debug.foxglove-layout.json"
        ).read_text(encoding="utf-8")
    )
    panel_kinds = {panel_id.split("!", 1)[0] for panel_id in layout["configById"]}
    assert "Publish" not in panel_kinds
    assert "ServiceCall" not in panel_kinds
    assert "Teleop" not in panel_kinds
    assert {"3D", "Image", "CallService"} <= panel_kinds
    assert "RawMessages" not in panel_kinds
    service_panels = {
        panel_id: panel
        for panel_id, panel in layout["configById"].items()
        if panel_id.startswith("CallService!")
    }
    assert set(service_panels) == {
        "CallService!stop",
        "CallService!camera-recovery",
        "CallService!survey-start",
        "CallService!survey-seal",
    }
    assert service_panels["CallService!stop"]["serviceName"] == (
        "/navdp_go2_adapter/operator_stop"
    )
    assert service_panels["CallService!stop"]["requestPayload"] == "{}"
    assert service_panels["CallService!stop"]["buttonText"] == "STOP NAVIGATION"
    camera_panel = service_panels["CallService!camera-recovery"]
    assert camera_panel["serviceName"] == "/navdp_camera_recovery/restart"
    assert camera_panel["requestPayload"] == "{}"
    assert camera_panel["buttonText"] == "CAMERA RESET"
    survey_start = service_panels["CallService!survey-start"]
    assert survey_start["serviceName"] == "/navdp_go2_adapter/survey_start"
    assert survey_start["requestPayload"] == "{}"
    assert survey_start["buttonText"] == "START SURVEY"
    survey_seal = service_panels["CallService!survey-seal"]
    assert survey_seal["serviceName"] == "/navdp_go2_adapter/survey_seal"
    assert survey_seal["requestPayload"] == "{}"
    assert survey_seal["buttonText"] == "SEAL SURVEY"
    root = layout["layout"]
    assert root["direction"] == "column"
    assert root["splitPercentage"] == 88
    main_area = root["first"]
    assert main_area["direction"] == "row"
    assert main_area["first"] == "Image!rgb"
    assert main_area["splitPercentage"] == 58
    diagnostics = root["second"]
    assert diagnostics == {
        "direction": "row",
        "first": "3D!navdp",
        "second": "Image!arrival",
        "splitPercentage": 45,
    }
    operator_area = main_area["second"]["second"]
    assert operator_area["direction"] == "column"
    assert operator_area["first"] == "Image!status"
    assert operator_area["splitPercentage"] == 70
    controls = operator_area["second"]
    assert controls["direction"] == "column"
    survey_button_row = controls["first"]
    assert survey_button_row == {
        "direction": "row",
        "first": "CallService!survey-start",
        "second": "CallService!survey-seal",
        "splitPercentage": 50,
    }


def test_foxglove_layout_maps_every_legacy_rviz_display_to_current_panels():
    layout = json.loads(
        (
            REPO
            / "deployment/go2/config/navdp_debug.foxglove-layout.json"
        ).read_text(encoding="utf-8")
    )
    panels = layout["configById"]
    expected_images = {
        "Image!rgb": "/navdp/foxglove/rgb/compressed",
        "Image!depth": "/navdp/foxglove/depth_color/compressed",
        "Image!goal": "/navdp/foxglove/goal/compressed",
        "Image!arrival": "/navdp/foxglove/arrival/compressed",
        "Image!status": "/navdp/foxglove/status/compressed",
    }
    for panel_id, topic in expected_images.items():
        panel = panels[panel_id]
        assert panel["imageMode"]["imageTopic"] == topic
        assert "cameraTopic" not in panel
        assert panel["imageMode"]["publish"] == {
            "clickEnabled": False,
            "hoverEnabled": False,
        }
    assert panels["3D!navdp"]["topics"]["/navdp/trajectory"]["visible"]
    trajectory = panels["3D!navdp"]["topics"]["/navdp/trajectory"]
    assert trajectory["type"] == "line"
    assert trajectory["lineWidth"] <= 0.05
    assert len(trajectory["gradient"]) == 2
    assert not panels["3D!navdp"]["topics"]["/navdp/debug/markers"]["visible"]


def test_foxglove_bridge_does_not_expose_raw_camera_images():
    parameters = (
        REPO / "deployment/go2/config/foxglove_bridge.yaml"
    ).read_text(encoding="utf-8")
    assert '"^/navdp/foxglove/.*"' in parameters
    assert '"^/navdp/go2/battery$"' in parameters
    assert '"^/camera/camera/color/image_raw$"' not in parameters
    assert '"^/camera/camera/aligned_depth_to_color/image_raw$"' not in parameters
    assert '"^/navdp/.*"' not in parameters
    assert "/navdp/image_goal" not in parameters
    assert "/navdp/rgb_arrival_debug" not in parameters
    services = parameters.split("service_whitelist:", 1)[1].split(
        "param_whitelist:", 1
    )[0]
    assert '"^/navdp_go2_adapter/operator_stop$"' in services
    assert '"^/navdp_go2_adapter/survey_start$"' in services
    assert '"^/navdp_go2_adapter/survey_seal$"' in services
    assert '"^/navdp_camera_recovery/restart$"' in services
    assert "set_enabled" not in services
    assert "reset_policy" not in services
    assert "begin_revisit" not in services
    capabilities = parameters.split("capabilities:", 1)[1].split(
        "service_whitelist:", 1
    )[0]
    assert "- services" in capabilities
    assert "clientPublish" not in capabilities
