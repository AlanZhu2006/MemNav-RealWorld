import io
import json

import pytest
import requests

from deployment.gpu.realworld_cec_hub import (
    CecHybridRouter,
    HybridBackendError,
    UpstreamConfig,
    create_app,
)


class FakeResponse:
    def __init__(self, payload, status=200):
        self.payload = payload
        self.status_code = status

    def raise_for_status(self):
        if self.status_code >= 400:
            raise requests.HTTPError(str(self.status_code))

    def json(self):
        return self.payload


class FakeSession:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def post(self, url, **kwargs):
        self.calls.append((url, kwargs))
        response = self.responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return response


def config():
    return UpstreamConfig("http://mem", "http://nav")


def reset_responses():
    return [FakeResponse({"algo": "memnav", "certified_relocalization": {"enabled": True}}),
            FakeResponse({"algo": "navdp"})]


def nav_result(marker):
    return FakeResponse({
        "trajectory": [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]],
        "all_trajectory": [],
        "all_values": [],
        "marker": marker,
    })


def do_reset(router):
    router.reset({
        "intrinsic": [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        "stop_threshold": -2.0,
        "batch_size": 1,
    })


def test_certificate_reject_calls_exact_native():
    session = FakeSession(reset_responses() + [
        FakeResponse({"frame_idx": 3, "certified_visual_candidates": []}),
        FakeResponse({"ok": True, "accepted": False, "reason": "no_candidate"}),
        nav_result("native"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")
    assert result["marker"] == "native"
    assert result["cec_takeover"] is False
    assert [call[0] for call in session.calls[-3:]] == [
        "http://mem/retrieval_probe_step",
        "http://mem/certified_relocalize",
        "http://nav/imagegoal_step",
    ]


def test_certificate_accept_projects_to_frozen_radius_and_calls_mixed():
    session = FakeSession(reset_responses() + [
        FakeResponse({"frame_idx": 7, "certified_visual_candidates": [{"anchor": 2}]}),
        FakeResponse({
            "ok": True,
            "accepted": True,
            "reason": "accepted",
            "pointgoal_units": "lingbot_raw_direction_only",
            "aux_pose": [3.0, 4.0],
            "selected_anchor": 2,
        }),
        nav_result("mixed"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")
    assert result["marker"] == "mixed"
    assert result["cec_takeover"] is True
    mixed_data = session.calls[-1][1]["data"]
    point = json.loads(mixed_data["goal_data"])
    assert point == {"goal_x": [1.5], "goal_y": [2.0]}


def test_probe_failure_degrades_memory_but_preserves_native():
    session = FakeSession(reset_responses() + [
        requests.ConnectionError("mem down"),
        nav_result("first-native"),
        nav_result("second-native"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    first = router.plan_imagegoal(image=b"i1", goal=b"g", depth=b"d")
    second = router.plan_imagegoal(image=b"i2", goal=b"g", depth=b"d")
    assert first["cec_reason"] == "retrieval_probe_failure_native_only"
    assert second["cec_reason"] == "memory_session_degraded"
    assert router.memory_degraded is True
    assert [call[0] for call in session.calls[-3:]] == [
        "http://mem/retrieval_probe_step",
        "http://nav/imagegoal_step",
        "http://nav/imagegoal_step",
    ]


def test_native_failure_latches_reset_required():
    session = FakeSession(reset_responses() + [
        FakeResponse({"frame_idx": 1, "certified_visual_candidates": []}),
        FakeResponse({"ok": True, "accepted": False, "reason": "no_candidate"}),
        requests.Timeout("ambiguous"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    with pytest.raises(HybridBackendError, match="reset is required"):
        router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")
    with pytest.raises(HybridBackendError, match="reset is required"):
        router.plan_imagegoal(image=b"i2", goal=b"g", depth=b"d")


def test_http_contract_and_busy_safe_validation():
    session = FakeSession(reset_responses())
    router = CecHybridRouter(config(), session=session)
    client = create_app(router).test_client()
    bad = client.post("/navigator_reset", json={"intrinsic": [[1.0]]})
    assert bad.status_code == 400
    good = client.post("/navigator_reset", json={
        "intrinsic": [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        "stop_threshold": -2.0,
        "batch_size": 1,
    })
    assert good.status_code == 200
    missing = client.post(
        "/imagegoal_step",
        data={"image": (io.BytesIO(b"i"), "image.jpg")},
        content_type="multipart/form-data",
    )
    assert missing.status_code == 400
