import io
import json

import pytest
import requests

from deployment.gpu.realworld_cec_hub import (
    NAVIGATION_SENSOR_CONTRACT,
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
    return UpstreamConfig("http://mem", "http://nav", camera_height_m=0.5)


def reset_responses():
    return [FakeResponse({
                "algo": "memnav",
                "certified_relocalization": {"enabled": True},
                "monocular_depth": {
                    "enabled": True,
                    "metric_depth_sensor_consumed": False,
                },
            }),
            FakeResponse({
                "algo": "navdp",
                "depth_source": "monocular_sidecar",
                "metric_depth_sensor_consumed_by_config": False,
                "monocular_depth_url_configured": True,
            })]


def nav_result(marker):
    return FakeResponse({
        "trajectory": [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]],
        "all_trajectory": [],
        "all_values": [],
        "marker": marker,
        "depth_source": "monocular_sidecar",
        "metric_depth_sensor_consumed": False,
        "monocular_depth_receipt": {"frame_index": 40},
    })


def do_reset(router):
    return router.reset({
        "intrinsic": [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        "stop_threshold": -2.0,
        "batch_size": 1,
    })


def memory_step_response(frame_idx=0):
    return FakeResponse({"frame_idx": frame_idx})


def warmup_response(queue_length=1, memory_size=8):
    return FakeResponse({
        "queue_lengths": [queue_length],
        "memory_size": memory_size,
        "diffusion_sampled": False,
    })


def enter_revisit(router):
    router.memory_step(b"m")
    return router.begin_revisit()


def test_certificate_reject_calls_exact_native():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({"frame_idx": 3, "certified_visual_candidates": []}),
        FakeResponse({"ok": True, "accepted": False, "reason": "no_candidate"}),
        nav_result("native"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    enter_revisit(router)
    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")
    assert result["marker"] == "native"
    assert result["cec_takeover"] is False
    assert result["client_metric_depth_forwarded"] is False
    assert "depth" not in session.calls[-1][1]["files"]
    assert [call[0] for call in session.calls[-3:]] == [
        "http://mem/retrieval_probe_step",
        "http://mem/certified_relocalize",
        "http://nav/imagegoal_step",
    ]


def test_certificate_accept_projects_to_frozen_radius_and_calls_mixed():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
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
    enter_revisit(router)
    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")
    assert result["marker"] == "mixed"
    assert result["cec_takeover"] is True
    mixed_data = session.calls[-1][1]["data"]
    point = json.loads(mixed_data["goal_data"])
    assert point == {"goal_x": [1.5], "goal_y": [2.0]}
    assert "depth" not in session.calls[-1][1]["files"]


def test_probe_failure_fails_closed_because_mono_depth_stream_is_shared():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        requests.ConnectionError("mem down"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    enter_revisit(router)
    with pytest.raises(HybridBackendError, match="stream update failed"):
        router.plan_imagegoal(image=b"i1", goal=b"g", depth=b"d")
    with pytest.raises(HybridBackendError, match="stream is degraded"):
        router.plan_imagegoal(image=b"i2", goal=b"g", depth=b"d")
    assert router.memory_degraded is True
    assert session.calls[-1][0] == "http://mem/retrieval_probe_step"


def test_native_failure_latches_reset_required():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({"frame_idx": 1, "certified_visual_candidates": []}),
        FakeResponse({"ok": True, "accepted": False, "reason": "no_candidate"}),
        requests.Timeout("ambiguous"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    enter_revisit(router)
    with pytest.raises(HybridBackendError, match="reset is required"):
        router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")
    with pytest.raises(HybridBackendError, match="reset is required"):
        router.plan_imagegoal(image=b"i2", goal=b"g", depth=b"d")


def test_goal_query_rejected_during_memory_recording():
    session = FakeSession(reset_responses())
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    with pytest.raises(ValueError, match="begin_revisit"):
        router.plan_imagegoal(image=b"i", goal=b"g")
    # No upstream traffic may result from the rejected query.
    assert len(session.calls) == 2


def test_memory_step_records_and_is_rejected_after_begin_revisit():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        memory_step_response(1),
        warmup_response(),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    first = router.memory_step(b"m0")
    second = router.memory_step(b"m1")
    assert (first["frames_recorded"], second["frames_recorded"]) == (1, 2)
    switch = router.begin_revisit()
    assert switch["revisit_started_after_frame"] == 2
    assert switch["navdp_warmup_frames"] == 1
    assert switch["navdp_warmup_frame_indices"] == [2]
    assert session.calls[-1][0] == "http://nav/memory_replay_step"
    with pytest.raises(ValueError, match="only valid during memory recording"):
        router.memory_step(b"m2")
    with pytest.raises(ValueError, match="memory recording phase"):
        router.begin_revisit()


def test_begin_revisit_requires_recorded_frames():
    router = CecHybridRouter(config(), session=FakeSession(reset_responses()))
    do_reset(router)
    with pytest.raises(ValueError, match="at least one recorded memory frame"):
        router.begin_revisit()


def test_goal_candidate_recorded_without_memory_append(tmp_path):
    session = FakeSession(
        reset_responses() + [memory_step_response(0), warmup_response()])
    router = CecHybridRouter(config(), session=session)
    router.goal_candidate_dir = str(tmp_path)
    do_reset(router)
    router.memory_step(b"m0")
    upstream_calls = len(session.calls)
    record = router.goal_candidate(b"candidate-jpg")
    # Candidate capture must not touch MemNav or NavDP.
    assert len(session.calls) == upstream_calls
    assert record["candidate_id"] == 0
    assert record["captured_after_frame"] == 1
    assert record["appended_to_memory"] is False
    with open(record["path"], "rb") as handle:
        assert handle.read() == b"candidate-jpg"
    router.begin_revisit()
    with pytest.raises(ValueError, match="during memory recording"):
        router.goal_candidate(b"too-late")


def test_memory_step_failure_fails_closed():
    session = FakeSession(reset_responses() + [
        requests.ConnectionError("mem down"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    with pytest.raises(HybridBackendError, match="stream update failed"):
        router.memory_step(b"m0")
    assert router.memory_degraded is True
    with pytest.raises(HybridBackendError, match="stream is degraded"):
        router.begin_revisit()


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
    payload = good.get_json()
    assert payload["navigation_sensor_contract"] == NAVIGATION_SENSOR_CONTRACT
    assert payload["metric_depth_sensor_consumed_by_policy"] is False
    navdp_reset = session.calls[1][1]["json"]
    assert navdp_reset["depth_source"] == "monocular_sidecar"
    assert session.calls[0][1]["json"]["camera_height"] == pytest.approx(0.5)
    health = client.get("/healthz").get_json()
    assert health["navigation_sensor_contract"] == NAVIGATION_SENSOR_CONTRACT
    assert health["phase"] == "memory_recording"
    assert health["frames_recorded"] == 0
    missing = client.post(
        "/imagegoal_step",
        data={"image": (io.BytesIO(b"i"), "image.jpg")},
        content_type="multipart/form-data",
    )
    assert missing.status_code == 400


def test_reset_rejects_upstream_that_does_not_prove_monocular_contract():
    responses = reset_responses()
    responses[1] = FakeResponse({
        "algo": "navdp",
        "depth_source": "metric_request",
        "metric_depth_sensor_consumed_by_config": True,
        "monocular_depth_url_configured": False,
    })
    router = CecHybridRouter(config(), session=FakeSession(responses))
    with pytest.raises(HybridBackendError, match="frozen monocular CEC contract"):
        do_reset(router)
    assert router.initialized is False
    assert router.native_state_uncertain is True


def test_http_step_accepts_rgb_only_and_discards_legacy_client_depth():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({"frame_idx": 3, "certified_visual_candidates": []}),
        FakeResponse({"ok": True, "accepted": False, "reason": "no_candidate"}),
        nav_result("native"),
    ])
    client = create_app(CecHybridRouter(config(), session=session)).test_client()
    reset = client.post("/navigator_reset", json={
        "intrinsic": [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        "stop_threshold": -2.0,
        "batch_size": 1,
    })
    assert reset.status_code == 200
    assert reset.get_json()["phase"] == "memory_recording"
    blocked = client.post(
        "/imagegoal_step",
        data={
            "image": (io.BytesIO(b"i"), "image.jpg"),
            "goal": (io.BytesIO(b"g"), "goal.jpg"),
        },
        content_type="multipart/form-data",
    )
    assert blocked.status_code == 400
    assert "begin_revisit" in blocked.get_json()["error"]
    recorded = client.post(
        "/memory_step",
        data={"image": (io.BytesIO(b"m"), "image.jpg")},
        content_type="multipart/form-data",
    )
    assert recorded.status_code == 200
    assert recorded.get_json()["frames_recorded"] == 1
    switched = client.post("/begin_revisit")
    assert switched.status_code == 200
    assert switched.get_json()["phase"] == "revisit_query"
    assert switched.get_json()["navdp_warmup_frames"] == 1
    response = client.post(
        "/imagegoal_step",
        data={
            "image": (io.BytesIO(b"i"), "image.jpg"),
            "goal": (io.BytesIO(b"g"), "goal.jpg"),
            "depth": (io.BytesIO(b"sensor-depth-must-not-pass"), "depth.png"),
        },
        content_type="multipart/form-data",
    )
    assert response.status_code == 200
    assert response.get_json()["client_metric_depth_forwarded"] is False
    assert "depth" not in session.calls[-1][1]["files"]


def test_select_warmup_frames_stride_and_order():
    from deployment.gpu.realworld_cec_hub import select_warmup_frames
    tail = [(i, bytes([i])) for i in range(1, 21)]
    picked = select_warmup_frames(tail, 8, 8)
    assert [index for index, _ in picked] == [4, 12, 20]
    long_tail = [(i, b"x") for i in range(7, 71)]
    picked = select_warmup_frames(long_tail, 8, 8)
    assert [index for index, _ in picked] == [14, 22, 30, 38, 46, 54, 62, 70]


def test_warmup_failure_latches_native_state_uncertain():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        requests.ConnectionError("nav down"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    router.memory_step(b"m0")
    with pytest.raises(HybridBackendError, match="warm-up failed"):
        router.begin_revisit()
    assert router.native_state_uncertain is True
    assert router.phase == "memory_recording"
    with pytest.raises(HybridBackendError, match="reset is required"):
        router.begin_revisit()


def test_warmup_queue_mismatch_latches_native_state_uncertain():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        FakeResponse({"queue_lengths": [0], "memory_size": 8}),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    router.memory_step(b"m0")
    with pytest.raises(HybridBackendError, match="queue length mismatch"):
        router.begin_revisit()
    assert router.native_state_uncertain is True
