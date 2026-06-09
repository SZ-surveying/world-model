from __future__ import annotations

import http.client
import json
import os
import ssl
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from rich.console import Console
from rich.progress import (
    BarColumn,
    DownloadColumn,
    Progress,
    TextColumn,
    TimeElapsedColumn,
    TimeRemainingColumn,
    TransferSpeedColumn,
)
from rich.table import Table

REPO_ROOT = Path(__file__).resolve().parents[4]
ARTIFACT_ROOT = REPO_ROOT / "artifacts/ros/navlab_companion_sitl_gazebo"
ENV_PATH = REPO_ROOT / ".env"
SESSION_ID = "navlab_companion_sitl_gazebo"
UPLOAD_ENABLED = True
API_URL = "https://api.foxglove.dev/v1"
TOKEN_ENV = "FOXGLOVE_API_TOKEN"
PROJECT_ID = ""
DEVICE_NAME = "navlab_companion_sitl_gazebo"
KEY_PREFIX = "navlab"
MCAP_RELATIVE = Path("rosbag/rosbag_0.mcap")
FOXGLOVE_MCAP_RELATIVE = Path("rosbag_foxglove/rosbag_foxglove_0.mcap")
SUMMARY_FILENAME = "summary.json"
REPLAY_SUMMARY_FILENAME = "foxglove_replay_summary.json"
ATTACHMENT_PREFIX = "attachments"
MAX_UPLOAD_ATTEMPTS = 3
CONSOLE = Console()
ERROR_CONSOLE = Console(stderr=True)


@dataclass(frozen=True, slots=True)
class UploadTarget:
    kind: str
    path: Path
    filename: str
    key: str


class ProgressReader:
    def __init__(self, body, progress: Progress, task_id: int) -> None:
        self._body = body
        self._progress = progress
        self._task_id = task_id

    def read(self, size: int = -1) -> bytes:
        chunk = self._body.read(size)
        if chunk:
            self._progress.update(self._task_id, advance=len(chunk))
        return chunk


def upload_run(run: str | None = None, *, dry_run: bool = False, force: bool = False, lite: bool = False) -> dict[str, Any]:
    run_dir = resolve_run_dir(run)

    if lite:
        lite_path = run_dir / FOXGLOVE_MCAP_RELATIVE
        if not lite_path.is_file():
            CONSOLE.print(f"[yellow]warn:[/yellow] lite MCAP missing, generating first: {lite_path.relative_to(run_dir)}")
            if not _generate_lite_mcap(run_dir):
                raise FileNotFoundError("failed to generate lite MCAP")

    targets = _build_targets(run_dir, lite=lite)

    missing = [target.path for target in targets if not target.path.is_file()]
    if missing:
        raise FileNotFoundError(f"required upload file missing: {missing[0]}")

    if not UPLOAD_ENABLED and not force:
        raise PermissionError("upload disabled; pass --force to upload anyway")

    if dry_run:
        _print_targets("Dry Run", run_dir, targets)
        return _summary(True, "dry_run", run_dir, targets, "resolved upload targets only")

    _load_dotenv(ENV_PATH)
    token = os.environ.get(TOKEN_ENV, "")
    if not token:
        raise EnvironmentError(f"missing token env {TOKEN_ENV}")

    uploaded: list[dict[str, Any]] = []
    try:
        _print_targets("Foxglove Upload", run_dir, targets)
        with _progress() as progress:
            for target in targets:
                link = _upload_with_retries(token, target, progress)
                uploaded.append(
                    {
                        "kind": target.kind,
                        "path": str(target.path),
                        "filename": target.filename,
                        "key": target.key,
                        "request_id": link.get("requestId") or link.get("request_id") or "",
                    }
                )
    except (HTTPError, URLError, OSError, ValueError) as exc:
        raise OSError(f"upload failed: {exc}") from exc

    result = _summary(True, "uploaded", run_dir, targets, "uploaded to Foxglove cloud")
    result["uploaded"] = uploaded
    (run_dir / "foxglove_upload_summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    CONSOLE.print("[bold green]uploaded to Foxglove[/bold green]")
    CONSOLE.print_json(json.dumps(result, sort_keys=True))
    return result


def resolve_run_dir(value: str | None) -> Path:
    if value:
        path = Path(value).expanduser()
        if path.is_dir():
            return path
        repo_relative = REPO_ROOT / path
        if repo_relative.is_dir():
            return repo_relative
        run_dir = ARTIFACT_ROOT / value
        if run_dir.is_dir():
            return run_dir
        raise FileNotFoundError(f"run directory not found: {run_dir}")
    return _latest_run_dir()


def _resolve_run_dir(value: str | None) -> Path:
    return resolve_run_dir(value)


def _latest_run_dir() -> Path:
    if not ARTIFACT_ROOT.is_dir():
        raise FileNotFoundError(f"artifact root not found: {ARTIFACT_ROOT}")
    candidates = [path for path in ARTIFACT_ROOT.iterdir() if path.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"no run directories under: {ARTIFACT_ROOT}")
    return max(candidates, key=lambda path: path.name)


def _load_dotenv(path: Path) -> None:
    if not path.is_file():
        return
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def _generate_lite_mcap(run_dir: Path) -> bool:
    cmd = [
        "uv",
        "run",
        "--project",
        "scripts/command",
        "python",
        "scripts/command/main.py",
        "foxglove",
        "build-replay",
        str(run_dir),
    ]
    result = subprocess.run(cmd, cwd=REPO_ROOT, check=False)
    return result.returncode == 0 and (run_dir / FOXGLOVE_MCAP_RELATIVE).is_file()


def _build_targets(run_dir: Path, *, lite: bool = False) -> list[UploadTarget]:
    run_id = run_dir.name
    task_kind = _task_kind(run_dir)
    base_key = f"{KEY_PREFIX}/{SESSION_ID}/{run_id}"
    mcap_path = run_dir / (FOXGLOVE_MCAP_RELATIVE if lite else MCAP_RELATIVE)
    mcap_source_name = "rosbag_foxglove_0.mcap" if lite else "rosbag_0.mcap"
    targets = [
        UploadTarget("mcap", mcap_path, f"navlab_{task_kind}_{run_id}.mcap", f"{base_key}/{mcap_source_name}"),
        UploadTarget(
            "summary",
            run_dir / SUMMARY_FILENAME,
            f"navlab_{task_kind}_{run_id}_summary.json",
            f"{base_key}/{ATTACHMENT_PREFIX}/{SUMMARY_FILENAME}",
        ),
    ]
    replay_summary = run_dir / REPLAY_SUMMARY_FILENAME
    if replay_summary.is_file():
        targets.append(
            UploadTarget(
                "replay_summary",
                replay_summary,
                f"navlab_{task_kind}_{run_id}_foxglove_replay_summary.json",
                f"{base_key}/{ATTACHMENT_PREFIX}/{REPLAY_SUMMARY_FILENAME}",
            )
        )
    return targets


def _task_kind(run_dir: Path) -> str:
    summary_path = run_dir / SUMMARY_FILENAME
    if not summary_path.is_file():
        return "run"
    try:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return "run"
    if "hover_gate" in summary or ("hover" in summary and "p8_exploration" not in summary):
        return "hover"
    if "p8_exploration" in summary or "p8_exploration_gate" in summary:
        return "p8"
    if "p12_airframe_disturbance_gate" in summary or "scan_robustness" in summary:
        return "scan_robustness"
    return "run"


def _upload_with_retries(token: str, target: UploadTarget, progress: Progress) -> dict[str, Any]:
    last_error: Exception | None = None
    for attempt in range(1, MAX_UPLOAD_ATTEMPTS + 1):
        try:
            CONSOLE.print(f"[cyan]request link[/cyan] {target.filename} [dim]attempt {attempt}/{MAX_UPLOAD_ATTEMPTS}[/dim]")
            return _upload_one(token, target, progress, attempt)
        except (HTTPError, URLError, OSError, TimeoutError, ValueError, ssl.SSLError) as exc:
            last_error = exc
            if attempt >= MAX_UPLOAD_ATTEMPTS:
                break
            CONSOLE.print(f"[yellow]retry[/yellow] {target.filename}: {exc}")
    raise OSError(f"{target.filename} failed after {MAX_UPLOAD_ATTEMPTS} attempts: {last_error}")


def _upload_one(token: str, target: UploadTarget, progress: Progress, attempt: int) -> dict[str, Any]:
    payload: dict[str, Any] = {"filename": target.filename, "key": target.key}
    if PROJECT_ID:
        payload["projectId"] = PROJECT_ID
    device_id = os.environ.get("FOXGLOVE_DEVICE_ID", "")
    if device_id:
        payload["deviceId"] = device_id
    elif DEVICE_NAME:
        payload["deviceName"] = DEVICE_NAME

    api_url = API_URL.rstrip("/")
    link_request = Request(
        f"{api_url}/data/upload",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(link_request, timeout=30) as response:
        link = json.loads(response.read().decode("utf-8"))

    upload_url = _field(link, "link", "uploadUrl", "upload_url", "url")
    if not upload_url:
        raise ValueError(f"upload URL missing in Foxglove response for {target.kind}")

    _put_file(upload_url, target, progress, attempt)
    return link


def _put_file(upload_url: str, target: UploadTarget, progress: Progress, attempt: int) -> None:
    parsed = urlsplit(upload_url)
    if parsed.scheme not in {"http", "https"}:
        raise ValueError(f"unsupported upload URL scheme: {parsed.scheme}")

    connection_class = http.client.HTTPSConnection if parsed.scheme == "https" else http.client.HTTPConnection
    path = parsed.path or "/"
    if parsed.query:
        path = f"{path}?{parsed.query}"

    headers = {
        "Content-Type": _content_type(target.path),
        "Content-Length": str(target.path.stat().st_size),
    }
    task_id = progress.add_task(
        f"[bold]{target.kind}[/bold] {target.filename} [dim]try {attempt}/{MAX_UPLOAD_ATTEMPTS}[/dim]",
        total=target.path.stat().st_size,
    )
    try:
        connection = connection_class(parsed.netloc, timeout=900, blocksize=1024 * 1024)
    except TypeError:
        connection = connection_class(parsed.netloc, timeout=900)
        connection.blocksize = 1024 * 1024
    success = False
    try:
        with target.path.open("rb") as body:
            reader = ProgressReader(body, progress, task_id)
            connection.request("PUT", path, body=reader, headers=headers)
            response = connection.getresponse()
            response_body = response.read().decode("utf-8", errors="replace")
        if response.status >= 400:
            detail = response_body.strip() or response.reason
            raise OSError(f"PUT failed for {target.kind}: HTTP {response.status} {detail}")
        progress.update(task_id, completed=target.path.stat().st_size)
        success = True
    finally:
        connection.close()
        if not success:
            progress.remove_task(task_id)


def _field(data: dict[str, Any], *keys: str) -> str:
    for key in keys:
        value = data.get(key)
        if value is not None:
            return str(value)
    return ""


def _content_type(path: Path) -> str:
    # Foxglove signed upload URLs include content-type in the signature.
    # Keep every PUT on the same binary content type to match the signed URL.
    return "application/octet-stream"


def _progress() -> Progress:
    return Progress(
        TextColumn("{task.description}"),
        BarColumn(),
        DownloadColumn(),
        TransferSpeedColumn(),
        TimeElapsedColumn(),
        TimeRemainingColumn(),
        console=CONSOLE,
    )


def _print_targets(title: str, run_dir: Path, targets: list[UploadTarget]) -> None:
    table = Table(title=f"{title}: {run_dir.name}")
    table.add_column("kind", style="cyan")
    table.add_column("filename")
    table.add_column("size", justify="right")
    table.add_column("source")
    table.add_column("key")
    for target in targets:
        table.add_row(
            target.kind,
            target.filename,
            _format_bytes(target.path.stat().st_size),
            str(target.path.relative_to(run_dir)),
            target.key,
        )
    CONSOLE.print(table)


def _format_bytes(size: int) -> str:
    value = float(size)
    for unit in ("B", "KiB", "MiB", "GiB"):
        if value < 1024.0 or unit == "GiB":
            return f"{value:.1f} {unit}"
        value /= 1024.0
    return f"{size} B"


def _summary(ok: bool, state: str, run_dir: Path, targets: list[UploadTarget], reason: str) -> dict[str, Any]:
    return {
        "ok": ok,
        "state": state,
        "reason": reason,
        "run_dir": str(run_dir),
        "files": [
            {"kind": target.kind, "path": str(target.path), "filename": target.filename, "key": target.key}
            for target in targets
        ],
    }
