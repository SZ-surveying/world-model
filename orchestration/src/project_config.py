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
class RuntimeConfig:
    lab_root: Path
    ardupilot_root: Path
    mavlink_router_root: Path
    venv_path: Path
    config_file: Path
    config_loaded: bool


@dataclass(slots=True)
class RouterConfig:
    listen: ValueWithSource
    tcp_port: ValueWithSource


@dataclass(slots=True)
class GazeboConfig:
    container_name: ValueWithSource
    world: ValueWithSource


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
class ProcessRuntimeServiceConfig:
    name: str
    command: tuple[str, ...]
    cwd: Path | None
    env: dict[str, str]


@dataclass(slots=True)
class DockerRuntimeBackendConfig:
    compose_file: Path
    project_name: ValueWithSource
    workspace_container_path: ValueWithSource


@dataclass(slots=True)
class ProcessRuntimeBackendConfig:
    workspace_host_path: Path
    log_dir: Path
    require_explicit_services: bool
    services: dict[str, ProcessRuntimeServiceConfig]


@dataclass(slots=True)
class RealRuntimeSourceConfig:
    scan_source_claim: ValueWithSource
    scan_source_topic: ValueWithSource
    fcu_source_claim: ValueWithSource
    imu_source_claim: ValueWithSource
    rangefinder_source_claim: ValueWithSource
    slam_source_claim: ValueWithSource
    required_real_topics: tuple[str, ...]
    forbidden_simulation_input_topics: tuple[str, ...]


@dataclass(slots=True)
class OrchestrationRuntimeBackendConfig:
    backend: ValueWithSource
    mode: ValueWithSource
    fail_on_missing_backend_config: bool
    fail_on_mode_violation: bool
    docker: DockerRuntimeBackendConfig
    process: ProcessRuntimeBackendConfig
    real_sources: RealRuntimeSourceConfig


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
    gazebo_sensor: NavLabImageConfig
    official_baseline: NavLabImageConfig


PACKAGE_PATH = Path(__file__).parent
PROJECT_PATH = PACKAGE_PATH.parent
REPO_PATH = PROJECT_PATH.parent

DEFAULT_ROUTER_LISTEN = "0.0.0.0:14550"
DEFAULT_ROUTER_TCP_PORT = "0"
DEFAULT_COMPOSE_FILE = REPO_PATH / "compose" / "docker-compose.yaml"
DEFAULT_COMPOSE_PROJECT_NAME = "navlab"
DEFAULT_COMPOSE_PROFILE = "base_env"
DEFAULT_RUNTIME_BACKEND = "docker"
DEFAULT_RUNTIME_MODE = "simulation"
DEFAULT_WORKSPACE_CONTAINER_PATH = "/workspace"
DEFAULT_PROCESS_RUNTIME_LOG_DIR = REPO_PATH / "artifacts" / "runtime_logs"
DEFAULT_REAL_REQUIRED_TOPICS = (
    "/scan",
    "/tf",
    "/tf_static",
    "/slam/odom",
    "/ap/v1/status",
    "/ap/v1/pose/filtered",
)
DEFAULT_FORBIDDEN_SIMULATION_INPUT_TOPICS = (
    "/gazebo/*",
    "/scan_ideal",
    "/sim/x2/status",
    "/rangefinder/down/scan_ideal",
)
COMPOSE_PROFILE_SERVICES: dict[str, tuple[str, ...]] = {
    "base_env": ("mavlink-router", "sitl", "gazebo"),
    "fast-lio": ("fast-lio",),
    "x2_sensor": ("gazebo-sensor",),
}

DEFAULT_GAZEBO_CONTAINER_NAME = "gazebo"
DEFAULT_GAZEBO_WORLD = "/workspace/worlds/navlab_iq_quad_figure8.sdf"
DEFAULT_FAST_LIO_CONTAINER_NAME = "fast-lio"
DEFAULT_FAST_LIO_CONFIG_PATH = "/workspace/profiles/fast-lio/config.yaml"
DEFAULT_NAVLAB_TAG_STRATEGY = "latest"
DEFAULT_NAVLAB_CONTEXT = "."
DEFAULT_NAVLAB_COMPANION_DOCKERFILE = "docker/Dockerfile.companion"
DEFAULT_NAVLAB_COMPANION_TARGET = "navlab-companion"
DEFAULT_NAVLAB_COMPANION_REPOSITORY = "world-model/navlab-companion"
DEFAULT_NAVLAB_COMPANION_IMAGE = f"{DEFAULT_NAVLAB_COMPANION_REPOSITORY}:latest"
DEFAULT_NAVLAB_SLAM_DOCKERFILE = "docker/Dockerfile.slam"
DEFAULT_NAVLAB_SLAM_TARGET = "navlab-slam-cartographer"
DEFAULT_NAVLAB_SLAM_REPOSITORY = "world-model/navlab-slam-cartographer"
DEFAULT_NAVLAB_SLAM_IMAGE = f"{DEFAULT_NAVLAB_SLAM_REPOSITORY}:latest"
DEFAULT_NAVLAB_GAZEBO_SENSOR_DOCKERFILE = "docker/Dockerfile.gazebo-sensor"
DEFAULT_NAVLAB_GAZEBO_SENSOR_TARGET = "navlab-gazebo-sensor"
DEFAULT_NAVLAB_GAZEBO_SENSOR_REPOSITORY = "world-model/navlab-gazebo-sensor"
DEFAULT_NAVLAB_GAZEBO_SENSOR_IMAGE = f"{DEFAULT_NAVLAB_GAZEBO_SENSOR_REPOSITORY}:latest"
DEFAULT_NAVLAB_OFFICIAL_BASELINE_DOCKERFILE = "docker/Dockerfile.official-baseline"
DEFAULT_NAVLAB_OFFICIAL_BASELINE_TARGET = "navlab-official-baseline"
DEFAULT_NAVLAB_OFFICIAL_BASELINE_REPOSITORY = "world-model/navlab-official-baseline"
DEFAULT_NAVLAB_OFFICIAL_BASELINE_IMAGE = f"{DEFAULT_NAVLAB_OFFICIAL_BASELINE_REPOSITORY}:latest"


def repo_root() -> Path:
    return REPO_PATH


def _resolve_router_value(
    router_config: dict[str, Any],
    key: str,
    default: str,
) -> ValueWithSource:
    if key in router_config and router_config[key] not in (None, ""):
        return ValueWithSource(str(router_config[key]), "config.toml")
    return ValueWithSource(default, "default")


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
    config_file = default_project_config_file()

    return RuntimeConfig(
        lab_root=root,
        ardupilot_root=root / "ardupilot",
        mavlink_router_root=root / "mavlink-router",
        venv_path=root / ".venv",
        config_file=config_file,
        config_loaded=config_file.is_file(),
    )


def default_project_config_file() -> Path:
    mode = _resolve_runtime_mode({}).value
    return PROJECT_PATH / f"config.{mode}.toml"


def load_router_config(runtime: RuntimeConfig) -> RouterConfig:
    config = load_project_config(runtime.config_file)
    raw_router = config.get("router", {})
    if raw_router is None:
        raw_router = {}
    if not isinstance(raw_router, dict):
        raise ValueError(f"Invalid [router] section in {runtime.config_file}")

    return RouterConfig(
        listen=_resolve_router_value(raw_router, "listen", DEFAULT_ROUTER_LISTEN),
        tcp_port=_resolve_router_value(raw_router, "tcp_port", DEFAULT_ROUTER_TCP_PORT),
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


def load_orchestration_runtime_backend_config(runtime: RuntimeConfig) -> OrchestrationRuntimeBackendConfig:
    config = load_project_config(runtime.config_file)
    raw_orchestration = config.get("orchestration", {})
    if raw_orchestration is None:
        raw_orchestration = {}
    if not isinstance(raw_orchestration, dict):
        raise ValueError(f"Invalid [orchestration] section in {runtime.config_file}")
    raw_runtime = raw_orchestration.get("runtime", {})
    if raw_runtime is None:
        raw_runtime = {}
    if not isinstance(raw_runtime, dict):
        raise ValueError(f"Invalid [orchestration.runtime] section in {runtime.config_file}")

    backend = _resolve_runtime_backend(raw_runtime)
    mode = _resolve_runtime_mode(raw_runtime)
    raw_docker = _optional_table(raw_runtime, "docker", runtime.config_file, "orchestration.runtime")
    raw_process = _optional_table(raw_runtime, "process", runtime.config_file, "orchestration.runtime")
    raw_real = _optional_table(raw_runtime, "real", runtime.config_file, "orchestration.runtime")
    raw_real_sources = _optional_table(raw_real, "sources", runtime.config_file, "orchestration.runtime.real")
    compose = load_compose_config(runtime)

    fail_on_missing_backend_config = bool(raw_runtime.get("fail_on_missing_backend_config", True))
    fail_on_mode_violation = bool(raw_runtime.get("fail_on_mode_violation", True))
    if fail_on_mode_violation:
        _validate_runtime_backend_mode(backend.value, mode.value)
    docker = DockerRuntimeBackendConfig(
        compose_file=_resolve_path(runtime.lab_root, str(raw_docker.get("compose_file") or compose.compose_file)),
        project_name=_resolve_router_value(raw_docker, "project_name", compose.project_name.value),
        workspace_container_path=_resolve_router_value(
            raw_docker,
            "workspace_container_path",
            DEFAULT_WORKSPACE_CONTAINER_PATH,
        ),
    )
    process = ProcessRuntimeBackendConfig(
        workspace_host_path=_resolve_path(
            runtime.lab_root,
            str(raw_process.get("workspace_host_path") or runtime.lab_root),
        ),
        log_dir=_resolve_path(runtime.lab_root, str(raw_process.get("log_dir") or DEFAULT_PROCESS_RUNTIME_LOG_DIR)),
        require_explicit_services=bool(raw_process.get("require_explicit_services", True)),
        services=_parse_process_services(raw_process, runtime.lab_root, runtime.config_file),
    )
    real_sources = RealRuntimeSourceConfig(
        scan_source_claim=_resolve_router_value(raw_real_sources, "scan_source_claim", "real_lidar_driver"),
        scan_source_topic=_resolve_router_value(raw_real_sources, "scan_source_topic", "/scan"),
        fcu_source_claim=_resolve_router_value(raw_real_sources, "fcu_source_claim", "real_ardupilot_dds"),
        imu_source_claim=_resolve_router_value(raw_real_sources, "imu_source_claim", "real_fcu_or_sensor"),
        rangefinder_source_claim=_resolve_router_value(
            raw_real_sources,
            "rangefinder_source_claim",
            "real_or_not_required",
        ),
        slam_source_claim=_resolve_router_value(raw_real_sources, "slam_source_claim", "real_slam"),
        required_real_topics=_resolve_string_tuple(
            raw_real_sources.get("required_real_topics"),
            DEFAULT_REAL_REQUIRED_TOPICS,
            "required_real_topics",
        ),
        forbidden_simulation_input_topics=_resolve_string_tuple(
            raw_real_sources.get("forbidden_simulation_input_topics"),
            DEFAULT_FORBIDDEN_SIMULATION_INPUT_TOPICS,
            "forbidden_simulation_input_topics",
        ),
    )
    if backend.value == "process" and fail_on_missing_backend_config and process.require_explicit_services:
        if not process.services:
            raise ValueError("Invalid [orchestration.runtime.process]: process backend requires explicit services")
    return OrchestrationRuntimeBackendConfig(
        backend=backend,
        mode=mode,
        fail_on_missing_backend_config=fail_on_missing_backend_config,
        fail_on_mode_violation=fail_on_mode_violation,
        docker=docker,
        process=process,
        real_sources=real_sources,
    )


def _resolve_runtime_backend(raw_runtime: dict[str, Any]) -> ValueWithSource:
    env_backend = os.environ.get("NAVLAB_RUNTIME_BACKEND")
    if env_backend:
        backend = env_backend.strip().lower()
        source = "NAVLAB_RUNTIME_BACKEND"
    else:
        backend = DEFAULT_RUNTIME_BACKEND
        source = "default"
    if backend not in {"docker", "process"}:
        raise ValueError(f"Invalid orchestration runtime backend '{backend}': expected docker or process")
    return ValueWithSource(backend, source)


def _resolve_runtime_mode(raw_runtime: dict[str, Any]) -> ValueWithSource:
    env_mode = os.environ.get("NAVLAB_RUNTIME_MODE")
    if env_mode:
        mode = env_mode.strip().lower()
        source = "NAVLAB_RUNTIME_MODE"
    else:
        mode = DEFAULT_RUNTIME_MODE
        source = "default"
    if mode not in {"simulation", "real"}:
        raise ValueError(f"Invalid orchestration runtime mode '{mode}': expected simulation or real")
    return ValueWithSource(mode, source)


def _validate_runtime_backend_mode(backend: str, mode: str) -> None:
    if (backend, mode) in {("docker", "simulation"), ("process", "real")}:
        return
    raise ValueError(
        "Invalid orchestration runtime backend/mode combination "
        f"{backend}+{mode}: supported combinations are docker+simulation and process+real"
    )


def _optional_table(parent: dict[str, Any], key: str, config_file: Path, parent_name: str) -> dict[str, Any]:
    raw = parent.get(key, {})
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError(f"Invalid [{parent_name}.{key}] section in {config_file}")
    return raw


def _resolve_path(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return root / path


def _resolve_string_tuple(raw: Any, default: tuple[str, ...], key: str) -> tuple[str, ...]:
    if raw in (None, ""):
        return default
    if not isinstance(raw, list) or not raw or not all(isinstance(item, str) and item for item in raw):
        raise ValueError(f"Invalid orchestration runtime real sources {key}: expected non-empty string array")
    return tuple(raw)


def _parse_process_services(
    raw_process: dict[str, Any],
    root: Path,
    config_file: Path,
) -> dict[str, ProcessRuntimeServiceConfig]:
    raw_services = raw_process.get("services", {})
    if raw_services is None:
        raw_services = {}
    if not isinstance(raw_services, dict):
        raise ValueError(f"Invalid [orchestration.runtime.process.services] section in {config_file}")
    services: dict[str, ProcessRuntimeServiceConfig] = {}
    for name, raw_service in raw_services.items():
        if not isinstance(raw_service, dict):
            raise ValueError(f"Invalid [orchestration.runtime.process.services.{name}] section in {config_file}")
        raw_command = raw_service.get("command")
        if not isinstance(raw_command, list) or not raw_command or not all(isinstance(arg, str) for arg in raw_command):
            raise ValueError(f"Invalid process runtime service {name}: command must be a non-empty string array")
        raw_env = raw_service.get("env", {})
        if raw_env is None:
            raw_env = {}
        if not isinstance(raw_env, dict) or not all(
            isinstance(k, str) and isinstance(v, str) for k, v in raw_env.items()
        ):
            raise ValueError(f"Invalid process runtime service {name}: env must be a string table")
        raw_cwd = raw_service.get("cwd")
        services[str(name)] = ProcessRuntimeServiceConfig(
            name=str(name),
            command=tuple(raw_command),
            cwd=(_resolve_path(root, str(raw_cwd)) if raw_cwd not in (None, "") else None),
            env=dict(raw_env),
        )
    return services


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
    default_dockerfile: str,
    default_target: str,
    default_repository: str,
) -> NavLabImageConfig:
    raw_image = images.get(key, {})
    if raw_image is None:
        raw_image = {}
    if not isinstance(raw_image, dict):
        raise ValueError(f"Invalid [navlab.images.{key}] section: expected a table")

    return NavLabImageConfig(
        dockerfile=_resolve_router_value(raw_image, "dockerfile", default_dockerfile),
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
            default_dockerfile=DEFAULT_NAVLAB_COMPANION_DOCKERFILE,
            default_target=DEFAULT_NAVLAB_COMPANION_TARGET,
            default_repository=DEFAULT_NAVLAB_COMPANION_REPOSITORY,
        ),
        slam=_resolve_navlab_image_config(
            raw_images,
            "slam",
            default_dockerfile=DEFAULT_NAVLAB_SLAM_DOCKERFILE,
            default_target=DEFAULT_NAVLAB_SLAM_TARGET,
            default_repository=DEFAULT_NAVLAB_SLAM_REPOSITORY,
        ),
        gazebo_sensor=_resolve_navlab_image_config(
            raw_images,
            "gazebo_sensor",
            default_dockerfile=DEFAULT_NAVLAB_GAZEBO_SENSOR_DOCKERFILE,
            default_target=DEFAULT_NAVLAB_GAZEBO_SENSOR_TARGET,
            default_repository=DEFAULT_NAVLAB_GAZEBO_SENSOR_REPOSITORY,
        ),
        official_baseline=_resolve_navlab_image_config(
            raw_images,
            "official_baseline",
            default_dockerfile=DEFAULT_NAVLAB_OFFICIAL_BASELINE_DOCKERFILE,
            default_target=DEFAULT_NAVLAB_OFFICIAL_BASELINE_TARGET,
            default_repository=DEFAULT_NAVLAB_OFFICIAL_BASELINE_REPOSITORY,
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
