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


class _BlockingClient:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()

    def begin_revisit(self, query_start_rgb=None):
        assert query_start_rgb is not None
        self.entered.set()
        assert self.release.wait(timeout=2.0)
        return {"phase": "revisit_query", "frames_recorded": 7}


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
