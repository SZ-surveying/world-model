from __future__ import annotations

import time
from pathlib import Path

from src.runtime.errors import ServiceWaitError
from src.runtime.process_manager import ManagedProcess, ProcessManager
from src.runtime.specs import ProbeResult, ProbeSpec, RosbagSpec, RuntimeHandle, ServiceSpec


class ProcessBackend:
    name = "process"

    def __init__(
        self,
        *,
        default_log_dir: Path | None = None,
        dry_run: bool = False,
        manager: ProcessManager | None = None,
    ) -> None:
        self._default_log_dir = default_log_dir or Path("artifacts/runtime_logs")
        self._dry_run = dry_run
        self._manager = manager or ProcessManager()
        self._dry_run_handles: set[str] = set()

    def start_service(self, spec: ServiceSpec) -> RuntimeHandle:
        spec.validate_for_process()
        log_path = spec.log_path or self._default_log_dir / f"{spec.name}.log"
        if self._dry_run:
            _write_text(log_path, _format_dry_run(spec.name, spec.command, spec.cwd, spec.env))
            identifier = f"dry-run:{spec.name}"
            self._dry_run_handles.add(identifier)
            return RuntimeHandle(
                backend=self.name,
                service_name=spec.name,
                identifier=identifier,
                command=spec.command,
                started_at=time.time(),
                log_path=log_path,
            )
        managed = self._manager.start(
            name=spec.name,
            command=spec.command,
            cwd=spec.cwd,
            env=spec.env,
            log_path=log_path,
            start_new_session=True,
        )
        return self._handle_from_managed(managed)

    def start_rosbag(self, spec: RosbagSpec) -> RuntimeHandle:
        command = spec.command()
        return self.start_service(
            ServiceSpec(
                name=spec.name,
                command=command,
                env=spec.env,
                cwd=spec.cwd,
                required=spec.required,
                log_path=spec.log_path,
            )
        )

    def run_probe(self, spec: ProbeSpec) -> ProbeResult:
        spec.validate_for_process()
        if self._dry_run:
            stdout = _format_dry_run(spec.name, spec.command, spec.cwd, spec.env)
            _write_text(spec.log_path, stdout)
            return ProbeResult(backend=self.name, name=spec.name, return_code=0, stdout=stdout, log_path=spec.log_path)
        completed = self._manager.run_capture(
            name=spec.name,
            command=spec.command,
            cwd=spec.cwd,
            env=spec.env,
            timeout_sec=spec.timeout_sec,
        )
        _write_text(spec.log_path, _join_output(completed.stdout, completed.stderr))
        return ProbeResult(
            backend=self.name,
            name=spec.name,
            return_code=completed.return_code,
            stdout=completed.stdout,
            stderr=completed.stderr,
            log_path=spec.log_path,
        )

    def wait(self, handle: RuntimeHandle, *, timeout_sec: float | None = None) -> int:
        if handle.identifier in self._dry_run_handles:
            return 0
        return self._manager.wait(self._managed_from_handle(handle), timeout_sec=timeout_sec)

    def stop(self, handle: RuntimeHandle, *, timeout_sec: float = 5.0) -> None:
        if handle.identifier in self._dry_run_handles:
            return
        self._manager.terminate_group(self._managed_from_handle(handle), timeout_sec=timeout_sec)

    def logs(self, handle: RuntimeHandle | str, *, tail: int = 400) -> str:
        if isinstance(handle, RuntimeHandle):
            if handle.log_path is None:
                return ""
            if handle.identifier in self._dry_run_handles:
                return self._manager.tail_logs(handle.log_path, tail=tail)
            return self._manager.tail_logs(self._managed_from_handle(handle), tail=tail)
        return self._manager.tail_logs(handle, tail=tail)

    def _managed_from_handle(self, handle: RuntimeHandle) -> ManagedProcess:
        if handle.pid is None:
            raise ServiceWaitError(f"process service {handle.service_name} has no pid in runtime handle")
        return self._manager.get(handle.pid)

    def _handle_from_managed(self, managed: ManagedProcess) -> RuntimeHandle:
        return RuntimeHandle(
            backend=self.name,
            service_name=managed.name,
            identifier=str(managed.pid),
            pid=managed.pid,
            command=managed.command,
            started_at=managed.started_at,
            log_path=managed.log_path,
        )


def _write_text(path: Path | None, text: str) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _join_output(stdout: str, stderr: str) -> str:
    if stderr:
        return f"{stdout}\n--- stderr ---\n{stderr}"
    return stdout


def _format_dry_run(name: str, command: tuple[str, ...], cwd: Path | str | None, env: object) -> str:
    return f"dry-run service={name}\ncwd={cwd or '<inherit>'}\nenv={dict(env)}\ncommand={' '.join(command)}\n"
