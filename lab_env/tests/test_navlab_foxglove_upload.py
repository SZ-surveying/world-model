from __future__ import annotations

import json
from pathlib import Path
from urllib.request import Request

from lab_env.navlab.orchestration.config import FoxgloveUploadConfig
from lab_env.navlab.orchestration.foxglove_upload import _upload_acceptance_rosbag


class _FakeResponse:
    def __init__(self, payload: bytes = b"{}") -> None:
        self._payload = payload

    def __enter__(self) -> _FakeResponse:
        return self

    def __exit__(self, *_args: object) -> None:
        return None

    def read(self) -> bytes:
        return self._payload


def _config() -> FoxgloveUploadConfig:
    return FoxgloveUploadConfig(
        enabled=True,
        api_url="https://api.foxglove.dev/v1",
        token_env="FOXGLOVE_API_TOKEN",
        project_id="project-a",
        device_id="",
        device_name="navlab_test",
        key_prefix="navlab",
        filename_prefix="navlab",
    )


def test_upload_acceptance_rosbag_skips_when_token_missing(tmp_path, monkeypatch) -> None:
    mcap = tmp_path / "rosbag_0.mcap"
    mcap.write_bytes(b"mcap")
    monkeypatch.delenv("FOXGLOVE_API_TOKEN", raising=False)

    result = _upload_acceptance_rosbag(
        upload_config=_config(),
        mcap_path=mcap,
        session_id="session",
        run_id="run",
    )

    assert result.ok is False
    assert result.state == "skipped"
    assert "FOXGLOVE_API_TOKEN" in result.reason


def test_upload_acceptance_rosbag_posts_link_then_puts_mcap(tmp_path, monkeypatch) -> None:
    mcap = tmp_path / "rosbag_0.mcap"
    mcap.write_bytes(b"mcap-bytes")
    monkeypatch.setenv("FOXGLOVE_API_TOKEN", "token-a")
    requests: list[Request] = []

    def fake_urlopen(request: Request, timeout: int):
        requests.append(request)
        if request.get_method() == "POST":
            payload = json.dumps({"link": "https://upload.example/put", "requestId": "request-1"}).encode()
            return _FakeResponse(payload)
        return _FakeResponse(b"")

    monkeypatch.setattr("lab_env.navlab.orchestration.foxglove_upload.urlopen", fake_urlopen)

    result = _upload_acceptance_rosbag(
        upload_config=_config(),
        mcap_path=mcap,
        session_id="session",
        run_id="run",
    )

    assert result.ok is True
    assert result.request_id == "request-1"
    assert [request.get_method() for request in requests] == ["POST", "PUT"]
    assert requests[0].headers["Authorization"] == "Bearer token-a"
    assert json.loads(requests[0].data.decode())["deviceName"] == "navlab_test"
    assert requests[1].data == b"mcap-bytes"
    assert requests[1].headers["Content-type"] == "application/octet-stream"
