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
    assert "CFG_FOXGLOVE_PREVIEW_RGB_FPS=10" in exports
    assert "CFG_FOXGLOVE_PREVIEW_DEPTH_FPS=5" in exports
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
    assert "CallService" not in panel_kinds
    assert "Teleop" not in panel_kinds
    assert {
        "3D",
        "DiagnosticStatusPanel",
        "DiagnosticSummary",
        "Image",
        "memnav-operator-controls.operator-controls",
        "Plot",
        "StateTransitions",
        "Tab",
        "TopicGraph",
    } <= panel_kinds
    controls_panel = "memnav-operator-controls.operator-controls!operate"
    assert layout["configById"][controls_panel] == {
        "foxglovePanelTitle": "Controls"
    }
    assert layout["layout"] == "Tab!navdp"
    tab_panel = layout["configById"][layout["layout"]]
    assert tab_panel["activeTabIdx"] == 0
    assert [tab["title"] for tab in tab_panel["tabs"]] == [
        "Operate",
        "Planning",
        "System",
    ]
    root = tab_panel["tabs"][0]["layout"]
    assert root["direction"] == "row"
    assert root["splitPercentage"] == 65
    main_area = root["first"]
    assert main_area == {
        "direction": "column",
        "first": "Image!rgb",
        "second": {
            "direction": "row",
            "first": "Image!arrival",
            "second": {
                "direction": "row",
                "first": "Image!goal",
                "second": "Image!depth",
                "splitPercentage": 50,
            },
            "splitPercentage": 32,
        },
        "splitPercentage": 62,
    }
    operator_area = root["second"]
    assert operator_area["direction"] == "column"
    assert operator_area["first"] == "3D!navdp"
    assert operator_area["splitPercentage"] == 40
    status_and_controls = operator_area["second"]
    status_strip = status_and_controls["first"]
    assert status_strip == "DiagnosticSummary!operate"
    compact_status = layout["configById"][status_strip]
    assert compact_status["topicToRender"] == "/navdp/operator/diagnostics"
    assert compact_status["sortByLevel"] is False
    assert compact_status["foxglovePanelTitle"] == "Operator"
    assert not any(
        panel_id.startswith("Indicator!")
        for panel_id in layout["configById"]
    )
    assert status_and_controls["splitPercentage"] == 78
    assert status_and_controls["second"] == controls_panel


def test_foxglove_layout_keeps_status_readable_on_small_16_by_9_display():
    document = json.loads(
        (
            REPO
            / "deployment/go2/config/navdp_debug.foxglove-layout.json"
        ).read_text(encoding="utf-8")
    )
    layout = document["configById"]["Tab!navdp"]["tabs"][0]["layout"]
    viewport_width, viewport_height = 1280.0, 720.0
    main_width = viewport_width * layout["splitPercentage"] / 100.0
    side_width = viewport_width - main_width
    main_height = viewport_height * layout["first"]["splitPercentage"] / 100.0
    diagnostics_height = viewport_height - main_height
    match_width = (
        main_width
        * layout["first"]["second"]["splitPercentage"]
        / 100.0
    )
    trajectory_height = (
        viewport_height * layout["second"]["splitPercentage"] / 100.0
    )
    status_height = (
        (viewport_height - trajectory_height)
        * layout["second"]["second"]["splitPercentage"]
        / 100.0
    )
    button_height = (
        viewport_height - trajectory_height - status_height
    )
    viewport_area = viewport_width * viewport_height
    rgb_area_fraction = main_width * main_height / viewport_area
    match_area_fraction = match_width * diagnostics_height / viewport_area

    assert 1.50 < side_width / trajectory_height < 1.60
    assert side_width > 400
    assert status_height > 330
    assert 1.25 < side_width / status_height < 1.40
    assert 90 < button_height < 110
    assert 0.35 < rgb_area_fraction < 0.42
    assert match_area_fraction < 0.08


def test_foxglove_debug_tabs_use_only_read_only_builtin_panels():
    document = json.loads(
        (
            REPO
            / "deployment/go2/config/navdp_debug.foxglove-layout.json"
        ).read_text(encoding="utf-8")
    )
    panels = document["configById"]
    tabs = {
        tab["title"]: tab["layout"]
        for tab in panels["Tab!navdp"]["tabs"]
    }

    assert tabs["Planning"]["first"] == "3D!planning"
    assert panels["3D!planning"]["topics"]["/navdp/debug/markers"][
        "visible"
    ]
    assert {
        path["value"] for path in panels["Plot!commands"]["paths"]
    } == {
        "/navdp/cmd_vel.linear.x",
        "/navdp/cmd_vel.angular.z",
    }
    planning_detail = tabs["Planning"]["second"]["second"]["second"]
    assert planning_detail == "DiagnosticStatusPanel!arrival"
    assert panels[planning_detail]["topicToRender"] == (
        "/navdp/operator/arrival_diagnostics"
    )
    assert panels[planning_detail]["selectedName"] == "MemNav/Arrival"

    assert tabs["System"]["first"] == "TopicGraph!navdp"
    assert {
        path["value"] for path in panels["StateTransitions!safety"]["paths"]
    } == {
        "/navdp/operator/mode.data",
        "/navdp/operator/activity.data",
        "/navdp/operator/safety.data",
        "/navdp/operator/go2.data",
        "/navdp/operator/arrival.data",
    }
    assert panels["DiagnosticSummary!operator"]["topicToRender"] == (
        "/navdp/operator/diagnostics"
    )
    workflow_detail = tabs["System"]["second"]["second"]["second"]
    assert workflow_detail == "DiagnosticStatusPanel!workflow"
    assert panels[workflow_detail]["topicToRender"] == (
        "/navdp/operator/diagnostics"
    )
    assert panels[workflow_detail]["selectedName"] == "MemNav/Overall"
    assert not any(panel_id.startswith("RawMessages!") for panel_id in panels)


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
    assert '"^/navdp/operator/.*$"' in parameters
    assert '"^/camera/camera/color/image_raw$"' not in parameters
    assert '"^/camera/camera/aligned_depth_to_color/image_raw$"' not in parameters
    assert '"^/navdp/.*"' not in parameters
    assert "/navdp/image_goal" not in parameters
    assert "/navdp/rgb_arrival_debug" not in parameters
    services = parameters.split("service_whitelist:", 1)[1].split(
        "param_whitelist:", 1
    )[0]
    assert '"^/navdp_camera_recovery/restart$"' in services
    assert '"^/memnav_operator/capture_goal$"' in services
    assert '"^/memnav_operator/start_survey$"' in services
    assert '"^/memnav_operator/stop_survey$"' in services
    assert '"^/memnav_operator/start_revisit$"' in services
    assert '"^/memnav_operator/operator_stop$"' in services
    assert "/navdp_go2_adapter/" not in services
    assert "set_enabled" not in services
    assert "reset_policy" not in services
    assert "begin_revisit" not in services
    capabilities = parameters.split("capabilities:", 1)[1].split(
        "service_whitelist:", 1
    )[0]
    assert "- services" in capabilities
    assert "clientPublish" not in capabilities
