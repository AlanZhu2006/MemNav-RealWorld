import threading
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navdp_ros_node import NavDPGo2Adapter


class _Response:
    success = None
    message = None


class _SetBoolRequest:
    def __init__(self, data):
        self.data = data


class _Logger:
    def info(self, _message):
        return None

    def warning(self, _message):
        return None


class _BlockingClient:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()

    def begin_revisit(self, query_start_rgb=None):
        assert query_start_rgb is not None
        self.entered.set()
        assert self.release.wait(timeout=2.0)
        return {"phase": "revisit_query", "frames_recorded": 7}


class _SelectedGoalClient:
    last_goal_jpeg = b"exact-goal-jpeg"
    last_goal_evaluation_depth_png = b"offline-depth-png"
    last_goal_evaluation_depth_scale_m = 0.001


class _SurveyClient:
    def __init__(self, *, frames=0, seal_error=None):
        self.frames = frames
        self.seal_error = seal_error
        self.seal_calls = 0

    def dataset_status(self):
        return {
            "recording": True,
            "dataset_id": "survey_01",
            "memory_frames": self.frames,
            "sealed_datasets": [],
        }

    def seal_dataset(self):
        self.seal_calls += 1
        if self.seal_error is not None:
            raise RuntimeError(self.seal_error)
        return {
            "dataset_id": "survey_01",
            "memory_frames": self.frames,
            "goal_candidates": 2,
            "manifest_sha256": "a" * 64,
        }


def _adapter(client):
    adapter = object.__new__(NavDPGo2Adapter)
    adapter._lock = threading.RLock()
    adapter._client_lock = threading.Lock()
    adapter._client = client
    adapter._server_initialized = True
    adapter._phase = "memory_recording"
    adapter._inference_busy = False
    adapter._frames_recorded = 0
    adapter._goal_candidates_captured = 0
    import numpy as np
    adapter._rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    adapter._revisit_image_goal = None
    adapter.auto_select_goal_candidate = False
    adapter._last_phase_receipt = {}
    adapter._active_goal_id = None
    adapter._active_goal_sha256 = None
    adapter._publish_receipt = lambda *_args: None
    adapter._publish_image_goal = lambda: None
    adapter.get_logger = lambda: _Logger()
    return adapter


def _survey_adapter(client):
    adapter = _adapter(client)
    adapter.pause_memory_recording = True
    adapter._enabled = False
    adapter._estop = True
    adapter._target_command = None
    adapter._stop_reason = "memory_recording_paused"
    adapter._survey_seal_receipt = {}
    adapter._survey_last_action = ""
    adapter._survey_last_success = None
    adapter._survey_last_message = ""
    adapter._survey_recording_active = None
    adapter.published_survey_receipts = []
    adapter._publish_receipt = (
        lambda event, receipt: adapter.published_survey_receipts.append(
            (event, receipt)
        )
    )
    adapter.survey_dataset_id = "survey_01"
    adapter.survey_seal_receipt_path = ""
    adapter.request_timeout_s = 1.0
    adapter._publish_zero = lambda *_args: None
    return adapter


def test_begin_revisit_reserves_inference_slot_before_client_transaction():
    """A timer worker cannot slip a final Novel step into the phase switch."""

    client = _BlockingClient()
    adapter = _adapter(client)
    response = _Response()
    callback = threading.Thread(
        target=adapter._begin_revisit_service,
        args=(object(), response),
        daemon=True,
    )

    callback.start()
    assert client.entered.wait(timeout=2.0)
    with adapter._lock:
        # This is the exact predicate used by the inference worker.  The
        # reservation must already be visible while the phase transaction is
        # still blocked under the client lock.
        assert adapter._inference_busy is True
        assert adapter._phase == "memory_recording"

    client.release.set()
    callback.join(timeout=2.0)

    assert not callback.is_alive()
    assert response.success is True
    assert adapter._phase == "revisit_query"
    assert adapter._inference_busy is False


def test_busy_begin_revisit_does_not_clear_worker_ownership():
    client = _BlockingClient()
    adapter = _adapter(client)
    adapter._inference_busy = True
    response = _Response()

    adapter._begin_revisit_service(object(), response)

    assert response.success is False
    assert "inference busy" in response.message
    assert adapter._phase == "memory_recording"
    assert adapter._inference_busy is True
    assert not client.entered.is_set()


def test_return_boundary_arms_candidates_without_changing_motion_authority():
    adapter = _adapter(_BlockingClient())
    adapter._frames_recorded = 137
    adapter.auto_goal_candidate_capture_enabled = False
    adapter._last_auto_candidate_after_frame = 96
    adapter._auto_candidate_guard_remaining = 3
    adapter._auto_candidate_capture_started_after_frame = 0

    response = _Response()
    adapter._set_auto_goal_candidate_capture_service(
        _SetBoolRequest(True), response
    )

    assert response.success is True
    assert adapter.auto_goal_candidate_capture_enabled is True
    assert adapter._auto_candidate_capture_started_after_frame == 137
    assert adapter._last_auto_candidate_after_frame == -1
    assert adapter._auto_candidate_guard_remaining == 0
    assert '"motion_authority_changed": false' in response.message


def test_selected_goal_artifacts_use_filesystem_paths(tmp_path):
    adapter = object.__new__(NavDPGo2Adapter)
    adapter._client = _SelectedGoalClient()
    adapter.selected_goal_image_path = str(tmp_path / "selected" / "goal.jpg")
    adapter.selected_goal_depth_path = str(tmp_path / "selected" / "depth.png")
    receipt = {}

    adapter._persist_selected_goal_artifacts(receipt)

    assert (tmp_path / "selected" / "goal.jpg").read_bytes() == b"exact-goal-jpeg"
    assert (tmp_path / "selected" / "depth.png").read_bytes() == b"offline-depth-png"
    assert receipt["selected_goal_depth_policy_authority"] is False


def test_survey_start_unpauses_only_the_config_bound_dataset():
    client = _SurveyClient(frames=0)
    adapter = _survey_adapter(client)
    response = _Response()

    adapter._survey_start_service(object(), response)

    assert response.success is True
    assert adapter.pause_memory_recording is False
    assert adapter._enabled is False
    assert adapter._estop is True
    assert adapter._inference_busy is False
    assert '"recording_active": true' in response.message
    assert "SURVEY STARTED" in response.message
    assert adapter._survey_last_success is True
    assert adapter.published_survey_receipts[-1][0] == "survey_start"


def test_survey_start_rejects_a_different_active_dataset():
    client = _SurveyClient(frames=0)
    adapter = _survey_adapter(client)
    adapter.survey_dataset_id = "survey_02"
    response = _Response()

    adapter._survey_start_service(object(), response)

    assert response.success is False
    assert adapter.pause_memory_recording is True
    assert adapter._enabled is False
    assert adapter._estop is True
    assert "identity/recording state" in response.message
    assert adapter._survey_last_success is False
    assert adapter.published_survey_receipts[-1][0] == "survey_start_rejected"


def test_survey_seal_locks_motion_and_is_idempotent():
    client = _SurveyClient(frames=200)
    adapter = _survey_adapter(client)
    adapter.pause_memory_recording = False
    first = _Response()
    second = _Response()

    adapter._survey_seal_service(object(), first)
    adapter._survey_seal_service(object(), second)

    assert first.success is True
    assert second.success is True
    assert client.seal_calls == 1
    assert adapter.pause_memory_recording is True
    assert adapter._enabled is False
    assert adapter._estop is True
    assert adapter._stop_reason == "survey_sealed"
    assert '"manifest_sha256": "' in first.message
    assert adapter._survey_last_success is True
    assert adapter.published_survey_receipts[-1][0] == "survey_seal"


def test_failed_survey_seal_remains_paused_and_start_can_resume():
    client = _SurveyClient(frames=12, seal_error="dataset is too short")
    adapter = _survey_adapter(client)
    adapter.pause_memory_recording = False
    seal_response = _Response()

    adapter._survey_seal_service(object(), seal_response)

    assert seal_response.success is False
    assert adapter.pause_memory_recording is True
    assert adapter._inference_busy is False
    assert "click START SURVEY to resume" in seal_response.message
    assert adapter._survey_last_success is False
    assert adapter.published_survey_receipts[-1][0] == "survey_seal_rejected"

    start_response = _Response()
    adapter._survey_start_service(object(), start_response)

    assert start_response.success is True
    assert adapter.pause_memory_recording is False
    assert '"resumed": true' in start_response.message
