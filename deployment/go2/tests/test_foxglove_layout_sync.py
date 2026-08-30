from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from sync_foxglove_layout import ApiError, sync_layout  # noqa: E402


LAYOUT_ID = "6f3394a6-b25e-5988-8b75-0c6f348b47c3"
LAYOUT_DATA = {"configById": {"3D!navdp": {"topics": {}}}}


class FakeClient:
    def __init__(self, existing=None):
        self.existing = existing
        self.calls = []

    def request(self, method, path, payload=None):
        self.calls.append((method, path, payload))
        if method == "GET":
            if self.existing is None:
                raise ApiError(404, "not found")
            return self.existing
        return {
            "id": LAYOUT_ID,
            "name": payload["name"],
            "folderName": payload["folderName"],
            "permission": payload["permission"],
            "data": payload["data"],
        }


def sync(client):
    return sync_layout(
        client,
        layout_id=LAYOUT_ID,
        name="MemNav Go2 Navigation",
        folder_name="MemNav-RealWorld",
        permission="ORG_WRITE",
        data=LAYOUT_DATA,
    )


def test_missing_layout_is_created_with_stable_id():
    client = FakeClient()
    result = sync(client)

    assert result["action"] == "created"
    assert client.calls[0] == (
        "GET",
        f"layouts/{LAYOUT_ID}?includeData=true",
        None,
    )
    method, path, payload = client.calls[1]
    assert (method, path) == ("POST", "layouts")
    assert payload["id"] == LAYOUT_ID
    assert payload["permission"] == "ORG_WRITE"
    assert payload["data"] == LAYOUT_DATA


def test_changed_layout_is_patched_in_place():
    client = FakeClient(
        {
            "id": LAYOUT_ID,
            "name": "Old name",
            "folderName": "",
            "permission": "ORG_READ",
            "data": {"old": True},
        }
    )
    result = sync(client)

    assert result["action"] == "updated"
    method, path, payload = client.calls[1]
    assert (method, path) == ("PATCH", f"layouts/{LAYOUT_ID}")
    assert "id" not in payload
    assert payload["data"] == LAYOUT_DATA


def test_identical_layout_does_not_create_history_noise():
    existing = {
        "id": LAYOUT_ID,
        "name": "MemNav Go2 Navigation",
        "folderName": "MemNav-RealWorld",
        "permission": "ORG_WRITE",
        "data": LAYOUT_DATA,
    }
    client = FakeClient(existing)
    result = sync(client)

    assert result == {"action": "unchanged", "layout": existing}
    assert len(client.calls) == 1
