from __future__ import annotations

import os
import signal
import subprocess
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from src.runtime.errors import ServiceStartError, ServiceWaitError


@dataclass(frozen=True, slots=True)
class ManagedProcess:
    name: str
    pid: int
    pgid: int | None
    command: tuple[str, ...]
    cwd: Path | None
    log_path: Path
    started_at: float


@dataclass(frozen=True, slots=True)
class CapturedProcessResult:
    name: str
    return_code: int
    stdout: str
    stderr: str


class ProcessManager:
    def __init__(self) -> None:
        self._processes: dict[int, subprocess.Popen[str]] = {}
        self._stdout_handles: dict[int, object] = {}
        self._managed: dict[int, ManagedProcess] = {}

    def start(
        self,
        *,
        name: str,
        command: tuple[str, ...],
        cwd: Path | str | None,
        env: Mapping[str, str],
        log_path: Path,
        start_new_session: bool = True,
    ) -> ManagedProcess:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        stdout = log_path.open("a", encoding="utf-8")
        merged_env = {**os.environ, **dict(env)}
        resolved_cwd = Path(cwd) if cwd is not None else None
        try:
            process = subprocess.Popen(
                list(command),
                cwd=(str(resolved_cwd) if resolved_cwd is not None else None),
                env=merged_env,
                stdout=stdout,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=start_new_session,
            )
        except OSError as exc:
            stdout.close()
            raise ServiceStartError(f"process service {name} failed to start: {exc}") from exc
        pid = int(process.pid)
        try:
            pgid = os.getpgid(pid)
        except ProcessLookupError:
            pgid = None
        managed = ManagedProcess(
            name=name,
            pid=pid,
            pgid=pgid,
            command=command,
            cwd=resolved_cwd,
            log_path=log_path,
            started_at=time.time(),
        )
        self._processes[pid] = process
        self._stdout_handles[pid] = stdout
        self._managed[pid] = managed
        return managed

    def run_capture(
        self,
        *,
        name: str,
        command: tuple[str, ...],
        cwd: Path | str | None,
        env: Mapping[str, str],
        timeout_sec: float | None = None,
    ) -> CapturedProcessResult:
        merged_env = {**os.environ, **dict(env)}
        try:
            completed = subprocess.run(
                list(command),
                cwd=(str(cwd) if cwd is not None else None),
                env=merged_env,
                text=True,
                capture_output=True,
                timeout=timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            return CapturedProcessResult(
                name=name,
                return_code=124,
                stdout=exc.stdout or "",
                stderr=exc.stderr or f"process probe timed out after {timeout_sec}s",
            )
        return CapturedProcessResult(
            name=name,
            return_code=int(completed.returncode),
            stdout=completed.stdout,
            stderr=completed.stderr,
        )

    def get(self, pid: int) -> ManagedProcess:
        managed = self._managed.get(pid)
        if managed is None:
            raise ServiceWaitError(f"process pid={pid} is not managed by this process manager")
        return managed

    def wait(self, managed: ManagedProcess, *, timeout_sec: float | None = None) -> int:
        process = self._processes.get(managed.pid)
        if process is None:
            raise ServiceWaitError(f"process service {managed.name} is not managed by this process manager")
        try:
            rc = process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired as exc:
            message = f"process service {managed.name} wait timed out after {timeout_sec}s"
            raise ServiceWaitError(message) from exc
        self._close_stdout(managed.pid)
        return int(rc)

    def terminate_group(self, managed: ManagedProcess, *, timeout_sec: float = 5.0) -> None:
        process = self._processes.get(managed.pid)
        if process is None:
            raise ServiceWaitError(f"process service {managed.name} is not managed by this process manager")
        if process.poll() is None:
            self._signal_group_or_process(managed, signal.SIGTERM)
            deadline = time.monotonic() + timeout_sec
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if process.poll() is None:
                self.kill_group(managed)
                process.wait(timeout=5)
        self._close_stdout(managed.pid)

    def kill_group(self, managed: ManagedProcess) -> None:
        process = self._processes.get(managed.pid)
        if process is None:
            raise ServiceWaitError(f"process service {managed.name} is not managed by this process manager")
        if process.poll() is None:
            self._signal_group_or_process(managed, signal.SIGKILL)

    def tail_logs(self, handle_or_path: ManagedProcess | Path | str, *, tail: int = 400) -> str:
        if isinstance(handle_or_path, ManagedProcess):
            path = handle_or_path.log_path
        else:
            path = Path(handle_or_path)
        if not path.is_file():
            return ""
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        return "\n".join(lines[-tail:])

    def _signal_group_or_process(self, managed: ManagedProcess, sig: signal.Signals) -> None:
        try:
            if managed.pgid is not None:
                os.killpg(managed.pgid, sig)
            else:
                os.kill(managed.pid, sig)
        except ProcessLookupError:
            return

    def _close_stdout(self, pid: int) -> None:
        stdout = self._stdout_handles.pop(pid, None)
        if stdout is not None:
            stdout.close()
