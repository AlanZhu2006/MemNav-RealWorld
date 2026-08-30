import threading
from pathlib import Path
import sys
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navdp_ros_node import NavDPGo2Adapter  # noqa: E402
from trajectory_control import VelocityCommand  # noqa: E402


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def test_operator_stop_is_idempotent_and_only_removes_motion_authority():
    adapter = object.__new__(NavDPGo2Adapter)
    adapter._lock = threading.RLock()
    adapter._enabled = True
    adapter._estop = False
    adapter._target_command = VelocityCommand(linear_x=0.24, angular_z=-0.31)
    zero_reasons = []
    logger = _Logger()
    adapter._publish_zero = zero_reasons.append
    adapter.get_logger = lambda: logger

    first = SimpleNamespace(success=False, message="")
    second = SimpleNamespace(success=False, message="")
    assert adapter._operator_stop_service(None, first) is first
    assert adapter._operator_stop_service(None, second) is second

    assert adapter._enabled is False
    assert adapter._estop is True
    assert adapter._target_command == VelocityCommand()
    assert zero_reasons == ["operator_stop", "operator_stop"]
    assert first.success is True
    assert second.success is True
    assert "disabled" in first.message
    assert "estop asserted" in first.message
    assert logger.warnings == [first.message, second.message]
