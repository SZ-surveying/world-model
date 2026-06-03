from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from lab_env.navlab.orchestration.config import FoxgloveUploadConfig, RunConfig


@dataclass(frozen=True, slots=True)
class FoxgloveUploadResult:
    ok: bool
    state: str
    reason: str
    mcap_path: str
    request_id: str = ""
    upload_url_present: bool = False

    def as_dict(self) -> dict[str, object]:
        return {
            "ok": self.ok,
            "state": self.state,
            "reason": self.reason,
            "mcap_path": self.mcap_path,
            "request_id": self.request_id,
            "upload_url_present": self.upload_url_present,
        }


def upload_acceptance_rosbag(config: RunConfig) -> FoxgloveUploadResult:
    upload_config = config.orchestration.foxglove_upload
    mcap_path = config.artifact_dir / "rosbag" / "rosbag_0.mcap"
    result = _upload_acceptance_rosbag(
        upload_config=upload_config,
        mcap_path=mcap_path,
        session_id=config.session_id,
        run_id=config.run_id,
    )
    _write_upload_summary(config.artifact_dir, result)
    return result


def _upload_acceptance_rosbag(
    *,
    upload_config: FoxgloveUploadConfig,
    mcap_path: Path,
    session_id: str,
    run_id: str,
) -> FoxgloveUploadResult:
    if not upload_config.enabled:
        return FoxgloveUploadResult(False, "skipped", "foxglove upload disabled", str(mcap_path))
    if not mcap_path.is_file():
        return FoxgloveUploadResult(False, "skipped", "mcap file missing", str(mcap_path))
    token = os.environ.get(upload_config.token_env, "")
    if not token:
        return FoxgloveUploadResult(
            False,
            "skipped",
            f"missing token env {upload_config.token_env}",
            str(mcap_path),
        )

    try:
        link = _create_upload_link(
            upload_config=upload_config,
            token=token,
            mcap_path=mcap_path,
            session_id=session_id,
            run_id=run_id,
        )
        upload_url = _field(link, "link", "uploadUrl", "upload_url", "url")
        request_id = _field(link, "requestId", "request_id")
        if not upload_url:
            return FoxgloveUploadResult(False, "failed", "upload URL missing in Foxglove response", str(mcap_path))
        _put_file(upload_url=upload_url, mcap_path=mcap_path)
    except (HTTPError, URLError, OSError, ValueError) as exc:
        return FoxgloveUploadResult(False, "failed", str(exc), str(mcap_path))
    return FoxgloveUploadResult(True, "uploaded", "uploaded to Foxglove cloud", str(mcap_path), request_id, True)


def _create_upload_link(
    *,
    upload_config: FoxgloveUploadConfig,
    token: str,
    mcap_path: Path,
    session_id: str,
    run_id: str,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "filename": f"{upload_config.filename_prefix}_{run_id}.mcap",
        "key": f"{upload_config.key_prefix}/{session_id}/{run_id}/{mcap_path.name}",
    }
    if upload_config.project_id:
        payload["projectId"] = upload_config.project_id
    if upload_config.device_id:
        payload["deviceId"] = upload_config.device_id
    elif upload_config.device_name:
        payload["deviceName"] = upload_config.device_name

    url = f"{upload_config.api_url.rstrip('/')}/data/upload"
    request = Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    with urlopen(request, timeout=30) as response:
        return json.loads(response.read().decode("utf-8"))


def _put_file(*, upload_url: str, mcap_path: Path) -> None:
    request = Request(
        upload_url,
        data=mcap_path.read_bytes(),
        headers={"Content-Type": "application/octet-stream"},
        method="PUT",
    )
    with urlopen(request, timeout=300) as response:
        response.read()


def _write_upload_summary(artifact_dir: Path, result: FoxgloveUploadResult) -> None:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    (artifact_dir / "foxglove_upload_summary.json").write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True),
        encoding="utf-8",
    )


def _field(data: dict[str, Any], *keys: str) -> str:
    for key in keys:
        value = data.get(key)
        if value is not None:
            return str(value)
    return ""
