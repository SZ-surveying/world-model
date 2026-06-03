from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import tomllib


@dataclass(slots=True)
class ValueWithSource:
    value: str
    source: str


@dataclass(slots=True)
class FloatWithSource:
    value: float
    source: str


@dataclass(slots=True)
class RuntimeConfig:
    lab_root: Path
    ardupilot_root: Path
    mavlink_router_root: Path
    venv_path: Path
    config_file: Path
    config_loaded: bool
    standalone_entrypoint: ValueWithSource


@dataclass(slots=True)
class RouterConfig:
    listen: ValueWithSource
    tcp_port: ValueWithSource
    endpoints: list[RouterEndpoint]
    endpoints_source: str


@dataclass(slots=True)
class RouterEndpoint:
    name: str
    endpoint: str


@dataclass(slots=True)
class GazeboConfig:
    container_name: ValueWithSource
    world: ValueWithSource


@dataclass(slots=True)
class FoxgloveConfig:
    container_name: ValueWithSource
    port: ValueWithSource


@dataclass(slots=True)
class FastLioConfig:
    container_name: ValueWithSource
    config_path: ValueWithSource


@dataclass(slots=True)
class ComposeConfig:
    compose_file: Path
    project_name: ValueWithSource
    default_profile: ValueWithSource


@dataclass(slots=True)
class SimConfig:
    stop_distance: FloatWithSource
    console_log_level: ValueWithSource
    file_log_level: ValueWithSource


@dataclass(slots=True)
class NavLabImageConfig:
    dockerfile: ValueWithSource
    context: ValueWithSource
    target: ValueWithSource
    repository: ValueWithSource
    tag_strategy: ValueWithSource

    def tag(self, *, cli_tag: str | None = None, cwd: Path | None = None) -> str:
        if cli_tag:
            return cli_tag
        return resolve_navlab_image_tag(self.tag_strategy.value, cwd=cwd)

    def image(self, *, cli_tag: str | None = None, cwd: Path | None = None) -> str:
        return f"{self.repository.value}:{self.tag(cli_tag=cli_tag, cwd=cwd)}"


@dataclass(slots=True)
class NavLabImagesConfig:
    companion: NavLabImageConfig
    slam: NavLabImageConfig


PACKAGE_PATH = Path(__file__).parent
PROJECT_PATH = PACKAGE_PATH.parent

DEFAULT_CONFIG_FILE = PACKAGE_PATH / "config.toml"
DEFAULT_STANDALONE_ENTRYPOINT = "127.0.0.1:14552"
DEFAULT_ROUTER_LISTEN = "0.0.0.0:14550"
DEFAULT_ROUTER_TCP_PORT = "0"
DEFAULT_COMPOSE_FILE = PROJECT_PATH / "compose" / "docker-compose.yaml"
DEFAULT_COMPOSE_PROJECT_NAME = "lab_env"
DEFAULT_COMPOSE_PROFILE = "base_env"
COMPOSE_PROFILE_SERVICES: dict[str, tuple[str, ...]] = {
    "base_env": ("mavlink-router", "sitl", "gazebo", "foxglove"),
    "rosbag_play": ("rosbag-play",),
    "fast-lio": ("fast-lio",),
}

DEFAULT_GAZEBO_CONTAINER_NAME = "gazebo"
DEFAULT_GAZEBO_WORLD = "/workspace/worlds/uav_obstacle_5m.sdf"
DEFAULT_FOXGLOVE_CONTAINER_NAME = "foxglove"
DEFAULT_FOXGLOVE_PORT = "8765"
DEFAULT_FAST_LIO_CONTAINER_NAME = "fast-lio"
DEFAULT_FAST_LIO_CONFIG_PATH = "/workspace/profiles/fast-lio/config.yaml"
DEFAULT_SIM_STOP_DISTANCE = 0.5
DEFAULT_SIM_CONSOLE_LOG_LEVEL = "DEBUG"
DEFAULT_SIM_FILE_LOG_LEVEL = "INFO"
DEFAULT_NAVLAB_TAG_STRATEGY = "latest"
DEFAULT_NAVLAB_DOCKERFILE = "docker/Dockerfile.companion"
DEFAULT_NAVLAB_CONTEXT = "."
DEFAULT_NAVLAB_COMPANION_TARGET = "navlab-companion"
DEFAULT_NAVLAB_COMPANION_REPOSITORY = "world-model/navlab-companion"
DEFAULT_NAVLAB_COMPANION_IMAGE = f"{DEFAULT_NAVLAB_COMPANION_REPOSITORY}:latest"
DEFAULT_NAVLAB_SLAM_TARGET = "navlab-slam-cartographer"
DEFAULT_NAVLAB_SLAM_REPOSITORY = "world-model/navlab-slam-cartographer"
DEFAULT_NAVLAB_SLAM_IMAGE = f"{DEFAULT_NAVLAB_SLAM_REPOSITORY}:latest"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _resolve_router_value(
    router_config: dict[str, Any],
    key: str,
    default: str,
) -> ValueWithSource:
    if key in router_config and router_config[key] not in (None, ""):
        return ValueWithSource(str(router_config[key]), "config.toml")
    return ValueWithSource(default, "default")


def _resolve_float_value(
    section: dict[str, Any],
    key: str,
    default: float,
    *,
    source_name: str = "config.toml",
) -> FloatWithSource:
    value = section.get(key)
    if value not in (None, ""):
        try:
            return FloatWithSource(float(value), source_name)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid value for '{key}': expected a number") from exc
    return FloatWithSource(default, "default")


def _resolve_standalone_entrypoint(config: dict[str, Any]) -> ValueWithSource:
    endpoint = config.get("standalone_entrypoint")
    if endpoint not in (None, ""):
        return ValueWithSource(str(endpoint), "config.toml")

    return ValueWithSource(DEFAULT_STANDALONE_ENTRYPOINT, "default")


def split_endpoint(endpoint: str) -> tuple[str, str]:
    host, separator, port = endpoint.rpartition(":")
    if not separator or not host or not port:
        raise ValueError(f"Invalid endpoint '{endpoint}': expected HOST:PORT")
    return host, port


def load_project_config(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open("rb") as handle:
        data = tomllib.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid config in {path}")
    return data


def load_runtime_config() -> RuntimeConfig:
    root = repo_root()
    config_file = root / DEFAULT_CONFIG_FILE
    config = load_project_config(config_file)

    return RuntimeConfig(
        lab_root=root,
        ardupilot_root=root / "ardupilot",
        mavlink_router_root=root / "mavlink-router",
        venv_path=root / ".venv",
        config_file=config_file,
        config_loaded=config_file.is_file(),
        standalone_entrypoint=_resolve_standalone_entrypoint(config),
    )


def load_router_config(runtime: RuntimeConfig) -> RouterConfig:
    config = load_project_config(runtime.config_file)
    raw_router = config.get("router", {})
    if raw_router is None:
        raw_router = {}
    if not isinstance(raw_router, dict):
        raise ValueError(f"Invalid [router] section in {runtime.config_file}")

    default_endpoints = [RouterEndpoint(name="default", endpoint=runtime.standalone_entrypoint.value)]
    if "endpoints" in raw_router and raw_router["endpoints"]:
        if not isinstance(raw_router["endpoints"], list):
            raise ValueError(f"Invalid [router].endpoints in {runtime.config_file}: expected a list")
        endpoints: list[RouterEndpoint] = []
        for index, item in enumerate(raw_router["endpoints"], start=1):
            if not isinstance(item, dict):
                raise ValueError(f"Invalid [router].endpoints[{index}] in {runtime.config_file}: expected a table")
            name = item.get("name")
            endpoint = item.get("endpoint")
            if not name or not endpoint:
                raise ValueError(
                    f"Invalid [router].endpoints[{index}] in {runtime.config_file}: name and endpoint are required"
                )
            endpoints.append(RouterEndpoint(name=str(name), endpoint=str(endpoint)))
        endpoint_source = "config.toml"
    else:
        endpoints = default_endpoints
        endpoint_source = "default"

    return RouterConfig(
        listen=_resolve_router_value(raw_router, "listen", DEFAULT_ROUTER_LISTEN),
        tcp_port=_resolve_router_value(raw_router, "tcp_port", DEFAULT_ROUTER_TCP_PORT),
        endpoints=endpoints,
        endpoints_source=endpoint_source,
    )


def load_gazebo_config(runtime: RuntimeConfig) -> GazeboConfig:
    config = load_project_config(runtime.config_file)
    raw_gazebo = config.get("gazebo", {})
    if raw_gazebo is None:
        raw_gazebo = {}
    if not isinstance(raw_gazebo, dict):
        raise ValueError(f"Invalid [gazebo] section in {runtime.config_file}")

    return GazeboConfig(
        container_name=_resolve_router_value(raw_gazebo, "container_name", DEFAULT_GAZEBO_CONTAINER_NAME),
        world=_resolve_router_value(raw_gazebo, "world", DEFAULT_GAZEBO_WORLD),
    )


def load_foxglove_config(runtime: RuntimeConfig) -> FoxgloveConfig:
    config = load_project_config(runtime.config_file)
    raw_foxglove = config.get("foxglove", {})
    if raw_foxglove is None:
        raw_foxglove = {}
    if not isinstance(raw_foxglove, dict):
        raise ValueError(f"Invalid [foxglove] section in {runtime.config_file}")

    return FoxgloveConfig(
        container_name=_resolve_router_value(raw_foxglove, "container_name", DEFAULT_FOXGLOVE_CONTAINER_NAME),
        port=_resolve_router_value(raw_foxglove, "port", DEFAULT_FOXGLOVE_PORT),
    )


def load_fast_lio_config(runtime: RuntimeConfig) -> FastLioConfig:
    config = load_project_config(runtime.config_file)
    raw_fast_lio = config.get("fast_lio", {})
    if raw_fast_lio is None:
        raw_fast_lio = {}
    if not isinstance(raw_fast_lio, dict):
        raise ValueError(f"Invalid [fast_lio] section in {runtime.config_file}")

    return FastLioConfig(
        container_name=_resolve_router_value(raw_fast_lio, "container_name", DEFAULT_FAST_LIO_CONTAINER_NAME),
        config_path=_resolve_router_value(raw_fast_lio, "config_path", DEFAULT_FAST_LIO_CONFIG_PATH),
    )


def load_compose_config(runtime: RuntimeConfig) -> ComposeConfig:
    config = load_project_config(runtime.config_file)
    raw_compose = config.get("compose", {})
    if raw_compose is None:
        raw_compose = {}
    if not isinstance(raw_compose, dict):
        raise ValueError(f"Invalid [compose] section in {runtime.config_file}")

    compose_file = runtime.lab_root / str(raw_compose.get("file") or DEFAULT_COMPOSE_FILE)
    return ComposeConfig(
        compose_file=compose_file,
        project_name=_resolve_router_value(raw_compose, "project_name", DEFAULT_COMPOSE_PROJECT_NAME),
        default_profile=_resolve_router_value(raw_compose, "default_profile", DEFAULT_COMPOSE_PROFILE),
    )


def load_sim_config(runtime: RuntimeConfig) -> SimConfig:
    config = load_project_config(runtime.config_file)
    raw_sim = config.get("sim", {})
    if raw_sim is None:
        raw_sim = {}
    if not isinstance(raw_sim, dict):
        raise ValueError(f"Invalid [sim] section in {runtime.config_file}")
    raw_logging = raw_sim.get("logging", {})
    if raw_logging is None:
        raw_logging = {}
    if not isinstance(raw_logging, dict):
        raise ValueError(f"Invalid [sim.logging] section in {runtime.config_file}")

    return SimConfig(
        stop_distance=_resolve_float_value(raw_sim, "stop_distance", DEFAULT_SIM_STOP_DISTANCE),
        console_log_level=_resolve_router_value(raw_logging, "console_level", DEFAULT_SIM_CONSOLE_LOG_LEVEL),
        file_log_level=_resolve_router_value(raw_logging, "file_level", DEFAULT_SIM_FILE_LOG_LEVEL),
    )


def resolve_navlab_image_tag(strategy: str, *, cwd: Path | None = None) -> str:
    normalized = strategy.strip().lower()
    if normalized == "latest":
        return "latest"
    if normalized == "git-commit":
        try:
            result = subprocess.run(
                ["git", "rev-parse", "--short=12", "HEAD"],
                cwd=cwd or repo_root(),
                check=True,
                text=True,
                capture_output=True,
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            raise ValueError("Could not resolve NavLab image tag from git commit") from exc
        tag = result.stdout.strip()
        if not tag:
            raise ValueError("Could not resolve NavLab image tag from git commit")
        return tag
    raise ValueError(f"Invalid NavLab image tag_strategy '{strategy}': expected latest or git-commit")


def _resolve_navlab_image_config(
    images: dict[str, Any],
    key: str,
    *,
    default_target: str,
    default_repository: str,
) -> NavLabImageConfig:
    raw_image = images.get(key, {})
    if raw_image is None:
        raw_image = {}
    if not isinstance(raw_image, dict):
        raise ValueError(f"Invalid [navlab.images.{key}] section: expected a table")

    return NavLabImageConfig(
        dockerfile=_resolve_router_value(raw_image, "dockerfile", DEFAULT_NAVLAB_DOCKERFILE),
        context=_resolve_router_value(raw_image, "context", DEFAULT_NAVLAB_CONTEXT),
        target=_resolve_router_value(raw_image, "target", default_target),
        repository=_resolve_router_value(raw_image, "repository", default_repository),
        tag_strategy=_resolve_router_value(
            raw_image,
            "tag_strategy",
            _resolve_router_value(images, "tag_strategy", DEFAULT_NAVLAB_TAG_STRATEGY).value,
        ),
    )


def load_navlab_images_config(runtime: RuntimeConfig) -> NavLabImagesConfig:
    config = load_project_config(runtime.config_file)
    raw_navlab = config.get("navlab", {})
    if raw_navlab is None:
        raw_navlab = {}
    if not isinstance(raw_navlab, dict):
        raise ValueError(f"Invalid [navlab] section in {runtime.config_file}")
    raw_images = raw_navlab.get("images", {})
    if raw_images is None:
        raw_images = {}
    if not isinstance(raw_images, dict):
        raise ValueError(f"Invalid [navlab.images] section in {runtime.config_file}")

    return NavLabImagesConfig(
        companion=_resolve_navlab_image_config(
            raw_images,
            "companion",
            default_target=DEFAULT_NAVLAB_COMPANION_TARGET,
            default_repository=DEFAULT_NAVLAB_COMPANION_REPOSITORY,
        ),
        slam=_resolve_navlab_image_config(
            raw_images,
            "slam",
            default_target=DEFAULT_NAVLAB_SLAM_TARGET,
            default_repository=DEFAULT_NAVLAB_SLAM_REPOSITORY,
        ),
    )


def services_for_profile(profile: str) -> tuple[str, ...]:
    return COMPOSE_PROFILE_SERVICES.get(profile, ())


def all_services() -> tuple[str, ...]:
    ordered: list[str] = []
    for services in COMPOSE_PROFILE_SERVICES.values():
        for service in services:
            if service not in ordered:
                ordered.append(service)
    return tuple(ordered)


def build_clean_env() -> dict[str, str]:
    env = dict(os.environ)
    for key in ("TMUX", "STY", "ZELLIJ", "DISPLAY"):
        env.pop(key, None)
    return env


def find_binary(name: str) -> str | None:
    return shutil.which(name)
