from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

from src.runtime.errors import BackendConfigError
from src.runtime.mode_policy import SERVICE_ROLE_GENERIC


@dataclass(frozen=True, slots=True)
class VolumeMount:
    source: Path
    target: str
    mode: str | None = None

    def as_docker_tuple(self) -> tuple[Path, str] | tuple[Path, str, str]:
        if not self.target:
            raise BackendConfigError("volume mount target must not be empty")
        if self.mode:
            return (self.source, self.target, self.mode)
        return (self.source, self.target)


@dataclass(frozen=True, slots=True)
class ServiceSpec:
    name: str
    command: tuple[str, ...]
    image: str | None = None
    compose_service: str | None = None
    container_name: str | None = None
    env: Mapping[str, str] = field(default_factory=dict)
    cwd: Path | str | None = None
    volumes: tuple[VolumeMount, ...] = ()
    networks: tuple[str, ...] = ()
    user: str | None = None
    detach: bool = True
    remove: bool = False
    required: bool = True
    log_path: Path | None = None
    service_role: str = SERVICE_ROLE_GENERIC

    def validate_for_process(self) -> None:
        _validate_name(self.name)
        if not self.command:
            raise BackendConfigError(f"service {self.name}: process backend requires command")
        if self.image:
            raise BackendConfigError(f"service {self.name}: process backend does not accept image={self.image!r}")
        _validate_env(self.name, self.env)

    def validate_for_docker(self) -> None:
        _validate_name(self.name)
        if not self.image:
            raise BackendConfigError(f"service {self.name}: docker backend requires image")
        if not self.command:
            raise BackendConfigError(f"service {self.name}: docker backend requires command")
        _validate_env(self.name, self.env)


@dataclass(frozen=True, slots=True)
class RosbagSpec:
    name: str
    topics_profile: Path
    output_path: Path
    duration_sec: float | None = None
    storage: str = "mcap"
    command_prefix: tuple[str, ...] = ("ros2", "bag", "record")
    required_topics: tuple[str, ...] = ()
    env: Mapping[str, str] = field(default_factory=dict)
    cwd: Path | str | None = None
    log_path: Path | None = None
    required: bool = True
    service_role: str = SERVICE_ROLE_GENERIC

    def validate(self) -> None:
        _validate_name(self.name)
        if not self.topics_profile.is_file():
            raise BackendConfigError(f"rosbag {self.name}: missing topics profile {self.topics_profile}")
        if not self.output_path:
            raise BackendConfigError(f"rosbag {self.name}: output_path is required")
        if self.duration_sec is not None and self.duration_sec <= 0:
            raise BackendConfigError(f"rosbag {self.name}: duration_sec must be positive")
        _validate_env(self.name, self.env)

    def topics(self) -> tuple[str, ...]:
        self.validate()
        topics = tuple(
            line.strip()
            for line in self.topics_profile.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.strip().startswith("#")
        )
        if not topics:
            raise BackendConfigError(f"rosbag {self.name}: topics profile {self.topics_profile} is empty")
        return topics

    def command(self) -> tuple[str, ...]:
        topics = self.topics()
        return (*self.command_prefix, "--storage", self.storage, "-o", str(self.output_path), *topics)


@dataclass(frozen=True, slots=True)
class ProbeSpec:
    name: str
    command: tuple[str, ...]
    image: str | None = None
    container_name: str | None = None
    networks: tuple[str, ...] = ()
    volumes: tuple[VolumeMount, ...] = ()
    timeout_sec: float | None = None
    env: Mapping[str, str] = field(default_factory=dict)
    cwd: Path | str | None = None
    log_path: Path | None = None
    required: bool = True
    service_role: str = SERVICE_ROLE_GENERIC

    def validate(self) -> None:
        _validate_name(self.name)
        if not self.command:
            raise BackendConfigError(f"probe {self.name}: command is required")
        if self.timeout_sec is not None and self.timeout_sec <= 0:
            raise BackendConfigError(f"probe {self.name}: timeout_sec must be positive")
        _validate_env(self.name, self.env)

    def validate_for_docker(self) -> None:
        self.validate()
        if not self.image:
            raise BackendConfigError(f"probe {self.name}: docker backend requires image")

    def validate_for_process(self) -> None:
        self.validate()
        if self.image:
            raise BackendConfigError(f"probe {self.name}: process backend does not accept image={self.image!r}")


@dataclass(frozen=True, slots=True)
class RuntimeHandle:
    backend: str
    service_name: str
    identifier: str
    command: tuple[str, ...]
    started_at: float | None = None
    log_path: Path | None = None
    pid: int | None = None
    container_id: str | None = None


@dataclass(frozen=True, slots=True)
class ProbeResult:
    backend: str
    name: str
    return_code: int
    stdout: str
    stderr: str = ""
    log_path: Path | None = None

    @property
    def ok(self) -> bool:
        return self.return_code == 0


def _validate_name(name: str) -> None:
    if not name or not name.strip():
        raise BackendConfigError("runtime spec name must not be empty")


def _validate_env(name: str, env: Mapping[str, str]) -> None:
    for key, value in env.items():
        if not isinstance(key, str) or not key:
            raise BackendConfigError(f"{name}: env keys must be non-empty strings")
        if not isinstance(value, str):
            raise BackendConfigError(f"{name}: env {key} must be a string")
