import threading
import time
from pathlib import Path
import sys
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from camera_recovery_service import (  # noqa: E402
    CameraRecoveryService,
    restart_tmux_camera,
)


def _camera_files(tmp_path):
    script = tmp_path / "run_realsense.sh"
    config = tmp_path / "resolved.json"
    script.write_text("#!/bin/sh\n", encoding="utf-8")
    config.write_text("{}\n", encoding="utf-8")
    return script, config, tmp_path / "realsense.log"


def test_restart_tmux_camera_respawns_an_existing_window(tmp_path):
    script, config, log = _camera_files(tmp_path)
    calls = []

    def runner(args, **kwargs):
        calls.append((args, kwargs))
        if args[1] == "list-windows":
            return SimpleNamespace(stdout="policy\nrgbd\nadapter\n")
        return SimpleNamespace(stdout="")

    restart_tmux_camera("navdp-go2", script, config, log, runner=runner)

    assert calls[1][0][:5] == [
        "tmux",
        "respawn-window",
        "-k",
        "-t",
        "navdp-go2:rgbd",
    ]
    assert "run_realsense.sh" in calls[1][0][5]
    assert ">>" in calls[1][0][5]


def test_restart_tmux_camera_recreates_a_missing_window(tmp_path):
    script, config, log = _camera_files(tmp_path)
    calls = []

    def runner(args, **kwargs):
        calls.append((args, kwargs))
        if args[1] == "list-windows":
            return SimpleNamespace(stdout="policy\nadapter\n")
        return SimpleNamespace(stdout="")

    restart_tmux_camera("navdp-go2", script, config, log, runner=runner)

    assert calls[1][0][:7] == [
        "tmux",
        "new-window",
        "-d",
        "-t",
        "navdp-go2:",
        "-n",
        "rgbd",
    ]
    assert "run_realsense.sh" in calls[1][0][7]


class _Logger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


def test_recovery_verifies_both_streams_and_never_unlocks_motion():
    service = object.__new__(CameraRecoveryService)
    service._restart_lock = threading.Lock()
    service._frame_condition = threading.Condition()
    service._verification_after = float("inf")
    service._rgb_frames = 0
    service._depth_frames = 0
    service._minimum_frames = 3
    service._recovery_timeout_s = 0.5
    service._verification_grace_s = 0.0
    motion_latches = []
    service._latch_motion_stop = lambda: motion_latches.append("locked")
    logger = _Logger()
    service.get_logger = lambda: logger

    def send_fresh_frames():
        time.sleep(0.02)
        for _ in range(3):
            service._record_frame("rgb")
            service._record_frame("depth")

    def restart_camera():
        threading.Thread(target=send_fresh_frames, daemon=True).start()

    service._restart_camera = restart_camera
    response = SimpleNamespace(success=False, message="")

    assert service._restart_service(None, response) is response
    assert response.success is True
    assert "fresh RGB and aligned-depth" in response.message
    assert "motion remains disabled and estop asserted" in response.message
    assert motion_latches == ["locked", "locked"]
    assert logger.warnings == [response.message]
