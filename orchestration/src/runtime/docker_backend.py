from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException

from src.runtime.errors import ServiceStartError, ServiceWaitError
from src.runtime.specs import ProbeResult, ProbeSpec, RosbagSpec, RuntimeHandle, ServiceSpec

DockerClientFactory = Callable[[], Any]


class DockerBackend:
    name = "docker"

    def __init__(
        self,
        *,
        client_factory: DockerClientFactory | None = None,
        compose_files: tuple[Path, ...] = (),
        compose_profiles: tuple[str, ...] = (),
        compose_project_name: str | None = None,
        compose_project_directory: Path | None = None,
        compose_env_files: tuple[Path, ...] = (),
    ) -> None:
        self._client_factory = client_factory or DockerClient
        self._compose_files = compose_files
        self._compose_profiles = compose_profiles
        self._compose_project_name = compose_project_name
        self._compose_project_directory = compose_project_directory
        self._compose_env_files = compose_env_files

    def _client(self) -> Any:
        if self._compose_files:
            return self._client_factory(
                compose_files=list(self._compose_files),
                compose_profiles=list(self._compose_profiles),
                compose_project_name=self._compose_project_name,
                compose_project_directory=self._compose_project_directory,
                compose_env_files=list(self._compose_env_files),
            )
        return self._client_factory()

    def compose_up(self, *, services: tuple[str, ...], detach: bool = True, build: bool = False) -> None:
        self._client().compose.up(services=list(services), detach=detach, build=build)

    def compose_stop(self, *, services: tuple[str, ...]) -> None:
        self._client().compose.stop(services=list(services))

    def compose_logs(self, *, services: tuple[str, ...], tail: int) -> str:
        return str(self._client().compose.logs(services=list(services), tail=str(tail)))

    def compose_ps_status(self, *, services: tuple[str, ...]) -> list[dict[str, object]]:
        containers = self._client().compose.ps(services=list(services), all=True)
        statuses: list[dict[str, object]] = []
        for container in containers:
            container_config = getattr(container, "config", None)
            container_state = getattr(container, "state", None)
            statuses.append(
                {
                    "id": getattr(container, "id", ""),
                    "name": getattr(container, "name", ""),
                    "image": getattr(container_config, "image", ""),
                    "status": getattr(container_state, "status", ""),
                    "running": bool(getattr(container_state, "running", False)),
                }
            )
        return statuses

    def remove_container(self, name: str, *, force: bool = True) -> None:
        try:
            self._client().remove(name, force=force)
        except DockerException as exc:
            raise ServiceWaitError(f"docker remove failed for {name}: {exc}") from exc

    def execute(self, container_name: str, command: tuple[str, ...]) -> int:
        try:
            self._client().execute(container_name, list(command))
        except DockerException as exc:
            return exc.return_code or 1
        return 0

    def start_service(self, spec: ServiceSpec) -> RuntimeHandle:
        spec.validate_for_docker()
        try:
            result = self._client().run(
                spec.image,
                list(spec.command),
                detach=spec.detach,
                remove=spec.remove,
                name=spec.container_name,
                networks=list(spec.networks),
                volumes=[mount.as_docker_tuple() for mount in spec.volumes],
                workdir=(str(spec.cwd) if spec.cwd is not None else None),
                user=spec.user,
                envs=dict(spec.env),
            )
        except DockerException as exc:
            raise ServiceStartError(f"docker service {spec.name} failed to start: {exc}") from exc
        container_id = str(getattr(result, "id", "") or spec.container_name or spec.name)
        return RuntimeHandle(
            backend=self.name,
            service_name=spec.name,
            identifier=spec.container_name or container_id,
            container_id=container_id,
            command=spec.command,
            started_at=time.time(),
            log_path=spec.log_path,
        )

    def start_rosbag(self, spec: RosbagSpec) -> RuntimeHandle:
        command = spec.command()
        service = ServiceSpec(
            name=spec.name,
            command=command,
            image="ros:jazzy",
            env=spec.env,
            cwd=spec.cwd,
            detach=True,
            required=spec.required,
            log_path=spec.log_path,
        )
        return self.start_service(service)

    def run_probe(self, spec: ProbeSpec) -> ProbeResult:
        spec.validate_for_docker()
        try:
            output = self._client().run(
                spec.image,
                list(spec.command),
                remove=True,
                name=spec.container_name,
                networks=list(spec.networks),
                volumes=[mount.as_docker_tuple() for mount in spec.volumes],
                workdir=(str(spec.cwd) if spec.cwd is not None else None),
                envs=dict(spec.env),
            )
        except DockerException as exc:
            stdout = str(exc)
            _write_probe_log(spec.log_path, stdout, "")
            return ProbeResult(
                backend=self.name,
                name=spec.name,
                return_code=exc.return_code or 1,
                stdout=stdout,
                log_path=spec.log_path,
            )
        stdout = str(output)
        _write_probe_log(spec.log_path, stdout, "")
        return ProbeResult(backend=self.name, name=spec.name, return_code=0, stdout=stdout, log_path=spec.log_path)

    def wait(self, handle: RuntimeHandle, *, timeout_sec: float | None = None) -> int:
        del timeout_sec
        try:
            rc = self._client().wait(handle.identifier)
        except DockerException as exc:
            raise ServiceWaitError(f"docker service {handle.service_name} wait failed: {exc}") from exc
        if isinstance(rc, list):
            if not rc:
                return 0
            first = rc[0]
            return int(getattr(first, "status_code", first))
        return int(rc or 0)

    def stop(self, handle: RuntimeHandle, *, timeout_sec: float = 5.0) -> None:
        del timeout_sec
        try:
            self._client().remove(handle.identifier, force=True)
        except DockerException as exc:
            raise ServiceWaitError(f"docker service {handle.service_name} stop failed: {exc}") from exc

    def logs(self, handle: RuntimeHandle | str, *, tail: int = 400) -> str:
        identifier = handle.identifier if isinstance(handle, RuntimeHandle) else handle
        try:
            return str(self._client().logs(identifier, tail=tail))
        except DockerException as exc:
            raise ServiceWaitError(f"docker logs failed for {identifier}: {exc}") from exc


def _write_probe_log(path: Path | None, stdout: str, stderr: str) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    text = stdout
    if stderr:
        text = f"{text}\n--- stderr ---\n{stderr}"
    path.write_text(text, encoding="utf-8")
