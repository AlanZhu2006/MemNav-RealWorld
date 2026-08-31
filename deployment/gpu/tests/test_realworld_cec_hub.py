import io
import hashlib
import json

import pytest
import requests
import numpy as np

from deployment.gpu.episodic_dataset import EpisodicDatasetStore
from deployment.gpu.monocular_depth_runtime import (
    bind_monocular_depth_transaction,
    build_monocular_depth_payload,
    canonical_sha256,
)
from deployment.gpu.realworld_cec_hub import (
    NAVIGATION_SENSOR_CONTRACT,
    TERMINAL_HANDOFF_SCHEMA,
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
        self.latest_depth_payload = None

    def post(self, url, **kwargs):
        self.calls.append((url, kwargs))
        if url.endswith("/monocular_depth_query"):
            if self.latest_depth_payload is None:
                return FakeResponse({"error": "no fake depth transaction"}, 409)
            return FakeResponse(dict(self.latest_depth_payload))
        response = self.responses.pop(0)
        if isinstance(response, Exception):
            raise response
        if isinstance(response, FakeResponse) and isinstance(response.payload, dict):
            payload = dict(response.payload)
            data = kwargs.get("data") or {}
            files = kwargs.get("files") or {}
            image_item = files.get("image")
            image_bytes = (
                image_item[1].getvalue()
                if image_item is not None and hasattr(image_item[1], "getvalue")
                else None
            )
            if (
                image_bytes is not None
                and data.get("materialize_monocular_depth") == "1"
                and url.endswith(("/memory_step", "/retrieval_probe_step"))
            ):
                frame_idx = int(payload.get("frame_idx", 0))
                scale_receipt = first40_scale_receipt()
                depth_payload = bind_monocular_depth_transaction(
                    build_monocular_depth_payload(
                        relative_depth=np.ones((2, 3), dtype=np.float32),
                        depth_shape=(2, 3),
                        image_sha256_value=hashlib.sha256(
                            image_bytes
                        ).hexdigest(),
                        frame_index=frame_idx,
                        scale_receipt=scale_receipt,
                    )
                )
                self.latest_depth_payload = depth_payload
                token = depth_payload[
                    "monocular_depth_transaction_token"
                ]
                payload.update({
                    "image_sha256": hashlib.sha256(image_bytes).hexdigest(),
                    "monocular_depth_transaction_token": token,
                    "monocular_depth_frame_index": frame_idx,
                })
            if (
                image_bytes is not None
                and isinstance(payload.get("monocular_depth_receipt"), dict)
            ):
                receipt = dict(payload["monocular_depth_receipt"])
                receipt.update({
                    "image_sha256": hashlib.sha256(image_bytes).hexdigest(),
                    "monocular_depth_transaction_token": data.get(
                        "monocular_depth_transaction_token"
                    ),
                    "frame_index": int(data.get(
                        "monocular_depth_frame_index", receipt.get("frame_index", 0)
                    )),
                })
                payload["monocular_depth_receipt"] = receipt
            response = FakeResponse(payload, response.status_code)
        return response


def first40_scale_receipt(*, scale=2.0):
    return {
        "schema": "mdtec_first40_scale_receipt_v1_20260819",
        "scale_evidence_contract": "causal_first_prefix_rgb_only_v1",
        "scale_prefix_frames": 40,
        "scale_prefix_first_frame": 0,
        "scale_prefix_last_frame": 39,
        "frozen_after_observation_count": 40,
        "active_from_frame_index": 40,
        "whole_episode_ground_cache_consumed": False,
        "camera_height_m": 0.5,
        "ground_h_est_raw": 0.25,
        "scale_valid": True,
        "scale_hat": scale,
        "valid_frame_ratio": 1.0,
        "relative_floor_iqr": 0.05,
        "scale_clamped": False,
        "freeze_error": None,
    }


def config():
    return UpstreamConfig("http://mem", "http://nav", camera_height_m=0.5)


def short_gap_config():
    """Compact unit fixture with one frame still forming a causal window."""
    return UpstreamConfig(
        "http://mem", "http://nav", camera_height_m=0.5,
        goal_min_frame_gap=1,
    )


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
                "monocular_depth_transaction_required": True,
            })]


def nav_result(marker, queue_length=1, memory_size=8):
    return FakeResponse({
        "trajectory": [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]],
        "all_trajectory": [],
        "all_values": [],
        "marker": marker,
        "depth_source": "monocular_sidecar",
        "metric_depth_sensor_consumed": False,
        "monocular_depth_receipt": {"frame_index": 40},
        "queue_lengths": [queue_length],
        "memory_size": memory_size,
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


def local_reject_response():
    return FakeResponse({
        "status": "precheck_fundamental_inliers",
        "certificate_accepted": False,
        "metric_scale_available": False,
        "predicted_relative_xy_m": None,
        "predicted_distance_m": None,
    })


def local_pose_response(
    x,
    y,
    *,
    yaw_right=0.0,
    metric_scale=2.0,
    frame_index=0,
    receipt_sha256=None,
):
    metric_x = metric_scale * x
    metric_y = metric_scale * y
    if receipt_sha256 is None:
        receipt_sha256 = canonical_sha256(
            first40_scale_receipt(scale=metric_scale)
        )
    return FakeResponse({
        "status": "ok",
        "frame_index": frame_index,
        "certificate_accepted": True,
        "scale_free_direction_available": True,
        "predicted_scale_free_relative_xy": [x, y],
        "metric_scale_available": True,
        "metric_scale_policy": "mdtec_first40",
        "metric_scale": {
            "available": True,
            "reason": "mdtec_first40_causal_scale_available",
            "frame_count": 40,
            "metric_scale_m_per_raw": metric_scale,
            "scale_receipt_sha256": receipt_sha256,
            "scale_evidence_contract": "causal_first_prefix_rgb_only_v1",
        },
        "predicted_relative_xy_m": [metric_x, metric_y],
        "predicted_distance_m": (metric_x * metric_x + metric_y * metric_y) ** 0.5,
        "terminal_yaw_right_deg": yaw_right,
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
        local_reject_response(),
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
    assert [call[0] for call in session.calls[-4:]] == [
        "http://mem/retrieval_probe_step",
        "http://mem/certified_relocalize",
        "http://mem/local_pose_query",
        "http://nav/imagegoal_step",
    ]


def test_formal_native_authority_mode_skips_all_bearing_authority():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({
            "frame_idx": 3,
            "certified_visual_candidates": [{"anchor": 2}],
        }),
        nav_result("native-authority-disabled"),
    ])
    router = CecHybridRouter(
        UpstreamConfig(
            "http://mem",
            "http://nav",
            camera_height_m=0.5,
            authority_mode="native",
        ),
        session=session,
    )
    do_reset(router)
    enter_revisit(router)

    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")

    assert result["marker"] == "native-authority-disabled"
    assert result["cec_authority_mode"] == "native"
    assert result["cec_takeover"] is False
    assert result["cec_controller"] == "navdp_image_authority_disabled"
    assert result["cec_reason"] == "authority_disabled_formal_native_arm"
    assert result["terminal_handoff_disposition"] == "native"
    assert result["terminal_proof_active"] is False
    assert result["client_metric_depth_forwarded"] is False
    assert [call[0] for call in session.calls[-2:]] == [
        "http://mem/retrieval_probe_step",
        "http://nav/imagegoal_step",
    ]
    assert not any(
        call[0].endswith(("/certified_relocalize", "/local_pose_query"))
        for call in session.calls
    )


def test_invalid_authority_mode_is_rejected_before_runtime():
    with pytest.raises(ValueError, match="unsupported authority mode"):
        UpstreamConfig(
            "http://mem",
            "http://nav",
            camera_height_m=0.5,
            authority_mode="raw",
        )


def test_certificate_accept_uses_first40_scale_and_bounded_metric_step():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({"frame_idx": 40, "certified_visual_candidates": [{"anchor": 2}]}),
        FakeResponse({
            "ok": True,
            "accepted": True,
            "reason": "accepted",
            "pointgoal_units": "lingbot_raw_direction_only",
            "aux_pose": [3.0, 4.0],
            "selected_anchor": 2,
        }),
        local_reject_response(),
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
    assert point["goal_x"] == pytest.approx([0.48])
    assert point["goal_y"] == pytest.approx([0.64])
    assert result["cec_controller"] == "navdp_image_metric_bounded_mix"
    assert result["memory_metric_scale_control_authority"] is True
    assert result["memory_controller_pointgoal_step_cap_m"] == 0.8
    assert result["cec_metric_scale"]["available"] is True
    assert "depth" not in session.calls[-1][1]["files"]


def test_inactive_first40_scale_falls_back_to_frozen_bearing_radius():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({
            "frame_idx": 7,
            "certified_visual_candidates": [{"anchor": 2}],
        }),
        FakeResponse({
            "ok": True,
            "accepted": True,
            "reason": "accepted",
            "pointgoal_units": "lingbot_raw_direction_only",
            "aux_pose": [3.0, 4.0],
            "selected_anchor": 2,
        }),
        local_reject_response(),
        nav_result("fixed-bearing-fallback"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    enter_revisit(router)

    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")

    point = json.loads(session.calls[-1][1]["data"]["goal_data"])
    assert point == {"goal_x": [1.5], "goal_y": [2.0]}
    assert result["cec_controller"] == "navdp_image_point_mix"
    assert result["memory_metric_scale_control_authority"] is False
    assert result["cec_metric_scale"]["available"] is False
    assert result["cec_metric_scale"]["reason"] == (
        "mdtec_first40_scale_unavailable"
    )


def test_direct_certified_bearing_supersedes_long_range_when_covisible():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({
            "frame_idx": 40,
            "certified_visual_candidates": [{"anchor": 2}],
        }),
        FakeResponse({
            "ok": True,
            "accepted": True,
            "reason": "accepted",
            "pointgoal_units": "lingbot_raw_direction_only",
            "aux_pose": [3.0, 4.0],
            "selected_anchor": 2,
        }),
        local_pose_response(0.60, 0.20, frame_index=40),
        nav_result("local-mixed"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    enter_revisit(router)

    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")

    assert result["marker"] == "local-mixed"
    assert result["cec_controller"] == (
        "navdp_image_direct_metric_bounded_mix"
    )
    assert result["terminal_handoff_disposition"] == "bearing_local"
    assert result["terminal_local_latched"] is False
    assert result["terminal_metric_scale_control_authority"] is True
    assert session.calls[-3][0] == "http://mem/local_pose_query"
    assert session.calls[-2][0] == "http://mem/monocular_depth_query"
    point = json.loads(session.calls[-1][1]["data"]["goal_data"])
    assert point["goal_x"] == pytest.approx([0.758946638])
    assert point["goal_y"] == pytest.approx([0.252982213])


def test_direct_metric_receipt_mismatch_falls_back_to_scale_free_bearing():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({
            "frame_idx": 40,
            "certified_visual_candidates": [{"anchor": 2}],
        }),
        FakeResponse({
            "ok": True,
            "accepted": True,
            "reason": "accepted",
            "pointgoal_units": "lingbot_raw_direction_only",
            "aux_pose": [3.0, 4.0],
            "selected_anchor": 2,
        }),
        local_pose_response(
            0.60,
            0.20,
            frame_index=40,
            receipt_sha256="b" * 64,
        ),
        nav_result("local-fixed-bearing"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    enter_revisit(router)

    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")

    assert result["cec_metric_scale"]["available"] is True
    assert result["terminal_localization"][
        "metric_scale_transaction_bound"
    ] is False
    assert result["terminal_metric_scale_control_authority"] is False
    assert result["cec_controller"] == (
        "navdp_image_direct_certified_bearing_mix"
    )
    point = json.loads(session.calls[-1][1]["data"]["goal_data"])
    assert point["goal_x"] == pytest.approx([2.371708245])
    assert point["goal_y"] == pytest.approx([0.790569415])


def test_rear_direct_pose_requests_atomic_turn_without_sending_bad_point_token():
    session = FakeSession(reset_responses() + [
        memory_step_response(),
        warmup_response(),
        FakeResponse({
            "frame_idx": 7,
            "certified_visual_candidates": [{"anchor": 2}],
        }),
        FakeResponse({
            "ok": True,
            "accepted": True,
            "reason": "accepted",
            "pointgoal_units": "lingbot_raw_direction_only",
            "aux_pose": [3.0, 4.0],
            "selected_anchor": 2,
        }),
        local_pose_response(-0.695, -0.024),
        nav_result("native-under-turn"),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    enter_revisit(router)

    result = router.plan_imagegoal(image=b"i", goal=b"g", depth=b"d")

    assert result["marker"] == "native-under-turn"
    assert result["cec_controller"] == "terminal_atomic_turn"
    assert result["terminal_handoff_disposition"] == "atomic_turn"
    assert result["terminal_turn_error_left_rad"] < -3.0
    assert session.calls[-1][0] == "http://nav/imagegoal_step"


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


def test_live_novel_plan_records_same_frame_and_skips_replay_at_switch():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        local_reject_response(),
        nav_result("novel", queue_length=1),
        memory_step_response(1),
        local_reject_response(),
        nav_result("novel", queue_length=2),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)

    first = router.plan_novel_and_record(image=b"m0", goal=b"q")
    second = router.plan_novel_and_record(image=b"m1", goal=b"q")

    assert first["novel_recording"] is True
    assert second["frames_recorded"] == 2
    assert router.navdp_live_recording_steps == 2
    assert [call[0] for call in session.calls[2:]] == [
        "http://mem/memory_step",
        "http://mem/local_pose_query",
        "http://nav/imagegoal_step",
        "http://mem/memory_step",
        "http://mem/local_pose_query",
        "http://nav/imagegoal_step",
    ]
    calls_before_switch = len(session.calls)
    switch = router.begin_revisit()
    assert len(session.calls) == calls_before_switch
    assert switch["navdp_warmup_mode"] == "live_novel_fifo"
    assert switch["navdp_warmup_frames"] == 0
    assert switch["navdp_queue_lengths"] == [2]


def test_novel_direct_pose_uses_bounded_metric_without_role_or_cec():
    session = FakeSession(reset_responses() + [
        memory_step_response(40),
        local_pose_response(0.60, 0.20, frame_index=40),
        nav_result("novel-local", queue_length=1),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)

    result = router.plan_novel_and_record(image=b"novel", goal=b"goal")

    assert result["marker"] == "novel-local"
    assert result["cec_takeover"] is False
    assert result["terminal_proof_active"] is True
    assert result["terminal_handoff_disposition"] == "bearing_local"
    assert result["cec_controller"] == (
        "navdp_image_direct_metric_bounded_mix"
    )
    assert result["terminal_metric_scale_control_authority"] is True
    assert result["terminal_stop_authorized"] is False
    assert session.calls[-3][0] == "http://mem/local_pose_query"
    assert session.calls[-2][0] == "http://mem/monocular_depth_query"
    assert session.calls[-1][0] == "http://nav/navdp_step_ip_mixgoal"
    point = json.loads(session.calls[-1][1]["data"]["goal_data"])
    assert point["goal_x"] == pytest.approx([0.758946638])
    assert point["goal_y"] == pytest.approx([0.252982213])


def test_novel_rear_direct_pose_emits_certified_atomic_turn_receipt():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        local_pose_response(-0.695, -0.024),
        nav_result("native-under-turn", queue_length=1),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)

    result = router.plan_novel_and_record(image=b"novel", goal=b"goal")

    assert result["cec_takeover"] is False
    assert result["terminal_proof_active"] is True
    assert result["terminal_handoff_disposition"] == "atomic_turn"
    assert result["terminal_turn_error_left_rad"] < -3.0
    assert result["cec_controller"] == "terminal_atomic_turn"
    assert session.calls[-1][0] == "http://nav/imagegoal_step"


def test_novel_direct_pose_never_authorizes_stop_from_metric_scale_alone():
    responses = reset_responses()
    for frame_idx, queue_length in ((0, 1), (1, 2), (2, 3)):
        responses.extend([
            memory_step_response(frame_idx),
            local_pose_response(0.03, 0.0, yaw_right=4.0),
            nav_result("native-under-terminal", queue_length=queue_length),
        ])
    router = CecHybridRouter(config(), session=FakeSession(responses))
    do_reset(router)

    results = [
        router.plan_novel_and_record(
            image=f"novel-{index}".encode(), goal=b"goal"
        )
        for index in range(3)
    ]

    assert [item["terminal_handoff_disposition"] for item in results] == [
        "bearing_local", "bearing_local", "bearing_local"
    ]
    assert all(item["terminal_stop_authorized"] is False for item in results)
    assert results[-1]["cec_takeover"] is False
    assert results[-1]["terminal_proof_active"] is True
    assert results[-1]["terminal_stop_authority"] == (
        "none_until_independent_visual_convergence"
    )


def test_novel_direct_proof_loss_returns_to_native_fallback():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        local_pose_response(0.60, 0.20),
        nav_result("novel-local", queue_length=1),
        memory_step_response(1),
        local_reject_response(),
        nav_result("native-under-hold", queue_length=2),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    router.plan_novel_and_record(image=b"novel-0", goal=b"goal")

    result = router.plan_novel_and_record(image=b"novel-1", goal=b"goal")

    assert result["terminal_handoff_disposition"] == "native"
    assert result["terminal_local_latched"] is False
    assert result["terminal_proof_active"] is False
    assert result["cec_controller"] == "navdp_image_router"


def test_live_novel_plan_rejects_missing_fifo_receipt():
    bad_plan = nav_result("novel")
    del bad_plan.payload["queue_lengths"]
    session = FakeSession(
        reset_responses() + [
            memory_step_response(0), local_reject_response(), bad_plan
        ]
    )
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    with pytest.raises(HybridBackendError, match="live FIFO receipt"):
        router.plan_novel_and_record(image=b"m0", goal=b"q")
    assert router.native_state_uncertain is True


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


def test_auto_candidate_validation_rejects_without_registering():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        FakeResponse({
            "ok": True, "max_cos": 0.96, "argmax_idx": 0,
            "frames_swept": 1, "frames_total": 1, "state_mutated": False,
            "geometry": {"matches": 40, "inliers": 30,
                         "inlier_ratio": 0.75},
            "geometry_backend": "sift_fundamental_ransac",
        }),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    router.memory_step(b"memory-frame")

    receipt = router.goal_candidate(
        b"near-duplicate", validate_support=True
    )

    assert receipt["registered"] is False
    assert receipt["capture_score"]["provisional_band"] == (
        "reject_near_duplicate"
    )
    assert router.goal_candidates == []
    assert router.frames_recorded == 1
    support_form = session.calls[-1][1]["data"]
    assert support_form["candidate_frame_idx"] == "1"
    assert support_form["min_frame_gap"] == "16"


def test_prepare_revisit_scores_selects_and_atomically_installs_goal():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        # Candidate 0: supported and non-trivial.
        FakeResponse({
            "ok": True, "max_cos": 0.88, "argmax_idx": 0,
            "frames_swept": 1, "frames_total": 1, "state_mutated": False,
            "eligible_anchor_ceiling": 0,
            "geometry": {"matches": 30, "inliers": 20,
                         "inlier_ratio": 0.67},
            "geometry_backend": "sift_fundamental_ransac",
        }),
        # Candidate 1: stronger geometry but a near duplicate, so rejected.
        FakeResponse({
            "ok": True, "max_cos": 0.95, "argmax_idx": 0,
            "frames_swept": 1, "frames_total": 1, "state_mutated": False,
            "geometry": {"matches": 60, "inliers": 50,
                         "inlier_ratio": 0.83},
            "geometry_backend": "sift_fundamental_ransac",
        }),
        warmup_response(),
    ])
    router = CecHybridRouter(short_gap_config(), session=session)
    do_reset(router)
    router.memory_step(b"memory-frame")
    first = router.goal_candidate(b"supported-goal")
    router.goal_candidate(b"near-duplicate-goal")

    receipt = router.prepare_revisit()

    assert receipt["phase"] == "revisit_query"
    assert receipt["selected_goal"]["candidate_id"] == first["candidate_id"]
    assert receipt["candidate_scores"][1]["provisional_band"] == (
        "reject_near_duplicate"
    )
    assert router.active_goal["image"] == b"supported-goal"
    assert router.last_prepare_receipt is not None
    assert receipt["goal_min_frame_gap"] == 1
    assert receipt["selected_goal"]["candidate_ceiling_override"] == 0
    assert router.active_goal["candidate_ceiling_override"] == 0
    # Both candidates are rescored against their immutable capture boundary;
    # later history must never widen the eligible anchor set.
    support_calls = [
        kwargs["data"] for url, kwargs in session.calls
        if url == "http://mem/goal_candidate_support"
    ]
    assert [row["candidate_frame_idx"] for row in support_calls] == ["1", "1"]
    assert "goal_image_jpeg_base64" not in router.last_prepare_receipt
    calls_after_commit = len(session.calls)
    replay = router.prepare_revisit()
    assert replay["idempotent_replay"] is True
    assert replay["selected_goal"]["candidate_id"] == first["candidate_id"]
    assert len(session.calls) == calls_after_commit


def test_prepare_revisit_without_eligible_goal_is_non_mutating():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        FakeResponse({
            "ok": True, "max_cos": 0.70, "argmax_idx": 0,
            "frames_swept": 1, "frames_total": 1, "state_mutated": False,
            "geometry": {"matches": 12, "inliers": 8,
                         "inlier_ratio": 0.67},
            "geometry_backend": "sift_fundamental_ransac",
        }),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    router.memory_step(b"memory-frame")
    router.goal_candidate(b"unsupported-goal")

    with pytest.raises(ValueError, match="no goal candidate passed"):
        router.prepare_revisit()

    assert router.phase == "memory_recording"
    assert router.active_goal is None
    assert router.native_state_uncertain is False


def test_pre_episode_goal_is_installed_without_forging_candidate_time():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        local_reject_response(),
        nav_result("novel", queue_length=1),
    ])
    router = CecHybridRouter(config(), session=session)
    do_reset(router)
    router.plan_novel_and_record(image=b"start-s", goal=b"novel-q")

    receipt = router.prepare_revisit_goal(b"frozen-revisit-r")

    assert receipt["phase"] == "revisit_query"
    assert receipt["navdp_warmup_mode"] == "live_novel_fifo"
    assert receipt["navdp_warmup_frames"] == 0
    assert receipt["goal_selection_contract"] == (
        "operator_frozen_external_v1"
    )
    assert receipt["selected_goal"]["candidate_id"] is None
    assert receipt["selected_goal"]["captured_after_frame"] is None
    assert receipt["selected_goal"]["goal_source"] == (
        "operator_frozen_external"
    )
    assert router.goal_candidates == []
    assert router.active_goal["image"] == b"frozen-revisit-r"

    calls_after_commit = len(session.calls)
    replay = router.prepare_revisit_goal(b"frozen-revisit-r")
    assert replay["idempotent_replay"] is True
    assert len(session.calls) == calls_after_commit
    with pytest.raises(ValueError, match="different committed goal"):
        router.prepare_revisit_goal(b"different-r")


def test_prepared_goal_requires_client_ack_and_overrides_uploaded_bytes():
    session = FakeSession(reset_responses() + [
        memory_step_response(0),
        FakeResponse({
            "ok": True, "max_cos": 0.80, "argmax_idx": 0,
            "frames_swept": 1, "frames_total": 1, "state_mutated": False,
            "eligible_anchor_ceiling": 0,
            "geometry": {"matches": 30, "inliers": 20,
                         "inlier_ratio": 0.67},
            "geometry_backend": "sift_fundamental_ransac",
        }),
        warmup_response(),
            FakeResponse({"frame_idx": 3, "certified_visual_candidates": []}),
            FakeResponse({"ok": True, "accepted": False, "reason": "no_candidate"}),
            local_reject_response(),
            nav_result("native"),
    ])
    router = CecHybridRouter(short_gap_config(), session=session)
    do_reset(router)
    router.memory_step(b"memory-frame")
    candidate = router.goal_candidate(b"committed-goal")
    router.prepare_revisit()

    with pytest.raises(ValueError, match="has not acknowledged"):
        router.plan_imagegoal(image=b"current", goal=b"stale-goal")

    result = router.plan_imagegoal(
        image=b"current",
        goal=b"stale-goal",
        form={"installed_goal_sha256": candidate["sha256"]},
    )
    assert result["marker"] == "native"
    # The retrieval probe must receive the hub-owned committed target, not the
    # stale compatibility upload supplied by the client.
    _, probe_kwargs = next(
        (url, kwargs)
        for url, kwargs in reversed(session.calls)
        if url == "http://mem/retrieval_probe_step"
    )
    probe_goal = probe_kwargs["files"]["goal"][1].read()
    assert probe_goal == b"committed-goal"
    probe_form = probe_kwargs["data"]
    assert probe_form["candidate_ceiling_override"] == "0"
    assert result["cec_candidate_ceiling_override"] == 0


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
    assert payload["terminal_handoff_schema"] == TERMINAL_HANDOFF_SCHEMA
    assert payload["metric_depth_sensor_consumed_by_policy"] is False
    navdp_reset = session.calls[1][1]["json"]
    assert navdp_reset["depth_source"] == "monocular_sidecar"
    assert session.calls[0][1]["json"]["camera_height"] == pytest.approx(0.5)
    health = client.get("/healthz").get_json()
    assert health["navigation_sensor_contract"] == NAVIGATION_SENSOR_CONTRACT
    assert health["terminal_handoff_schema"] == TERMINAL_HANDOFF_SCHEMA
    assert health["cec_authority_mode"] == "cec"
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
        local_reject_response(),
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


def test_sealed_dataset_replay_keeps_long_memory_out_of_navdp_fifo(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    recording = CecHybridRouter(
        short_gap_config(),
        session=FakeSession(reset_responses() + [memory_step_response(0)]),
        dataset_store=store,
    )
    do_reset(recording)
    recording.start_dataset("out-back")
    recording.memory_step(b"historical-memory")
    recording.goal_candidate(
        b"memory-excluded-goal",
        evaluation_depth=b"evaluator-depth",
        evaluation_depth_scale_m=1.0e-3,
    )
    sealed = recording.seal_dataset()
    assert sealed["goal_memory_exact_sha_overlap"] == 0

    replay_session = FakeSession(
        reset_responses() + [memory_step_response(0), warmup_response()]
    )
    replay = CecHybridRouter(
        short_gap_config(), session=replay_session, dataset_store=store
    )
    do_reset(replay)
    loaded = replay.load_dataset("out-back")
    assert loaded["frames_replayed"] == 1
    assert loaded["navdp_fifo_replayed_from_dataset"] is False
    assert replay.goal_candidates[0]["evaluation_depth_scale_m"] == 1.0e-3
    switch = replay.begin_revisit(query_start_image=b"physical-query-start")
    assert switch["navdp_warmup_mode"] == "independent_formal_query_start"
    assert switch["navdp_warmup_frame_indices"] == ["query_start_current"]
    warmup_url, warmup_kwargs = replay_session.calls[-1]
    assert warmup_url == "http://nav/memory_replay_step"
    assert (
        warmup_kwargs["files"]["image"][1].read()
        == b"physical-query-start"
    )


def test_auto_dataset_starts_inside_first_reset(tmp_path):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    router = CecHybridRouter(
        config(),
        session=FakeSession(reset_responses()),
        dataset_store=store,
        auto_dataset_id="survey-01",
        auto_dataset_metadata={"motion": "hand_controller"},
    )
    receipt = do_reset(router)
    assert receipt["episodic_dataset"]["dataset_id"] == "survey-01"
    assert store.status()["recording"] is True


def test_empty_auto_dataset_survives_reset_retry_but_nonempty_one_blocks_reset(
    tmp_path,
):
    store = EpisodicDatasetStore(tmp_path, minimum_frames=1)
    session = FakeSession(reset_responses() + reset_responses() + [
        memory_step_response(0)
    ])
    router = CecHybridRouter(
        config(),
        session=session,
        dataset_store=store,
        auto_dataset_id="survey-retry",
    )

    first = do_reset(router)
    second = do_reset(router)
    assert first["episodic_dataset"]["dataset_id"] == "survey-retry"
    assert second["episodic_dataset"]["dataset_id"] == "survey-retry"
    assert len(session.calls) == 4

    router.memory_step(b"first-recorded-frame")
    with pytest.raises(ValueError, match="still recording"):
        do_reset(router)
    # The rejected reset performs no upstream mutation.
    assert len(session.calls) == 5
