from types import SimpleNamespace

from go2_cmd_bridge import Go2CmdBridge


class Client:
    def __init__(self):
        self.calls = []

    def Move(self, *args):
        self.calls.append(("move", args))

    def StopMove(self):
        self.calls.append(("stop", ()))


def bridge():
    node = object.__new__(Go2CmdBridge)
    node.sport_client = Client()
    node.command_active = True
    node.send_zero_when_idle = False
    node.stop_once_on_release = True
    node.get_logger = lambda: SimpleNamespace(info=lambda _: None, error=lambda _: None)
    return node


def test_plan_pause_sends_zero_without_blocking_stop_rpc():
    node = bridge()
    node.release_control("zero cmd_vel", normal_pause=True)
    assert node.sport_client.calls == [("move", (0, 0, 0))]
    assert not node.command_active


def test_hand_controller_release_keeps_explicit_stop():
    node = bridge()
    node.release_control("hand controller activity")
    assert node.sport_client.calls == [("move", (0, 0, 0)), ("stop", ())]
