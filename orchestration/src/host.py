from __future__ import annotations

import os
import shlex
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException
from rich.console import Console
from rich.table import Table

from src.config import (
    NAVLAB_PROFILES,
    NAVLAB_SERVICES,
    NAVLAB_STOP_SERVICES,
    RunConfig,
)
from src.project_config import (
    load_compose_config,
    load_runtime_config,
)

COMPANION_CONTAINER = "navlab-companion"
SLAM_CONTAINER = "navlab-slam"
OFFICIAL_BASELINE_CONTAINER = "navlab-official-baseline"
GAZEBO_SENSOR_SERVICE = "gazebo-sensor"
DEFAULT_COMPANION_RUNTIME_CONFIG = Path("navlab/config.toml")
OFFICIAL_SITL_WORKDIR = "sitl_work"
ROS_SHELL_STDERR_FILTER_PATTERN = r"Failed to parse type hash|share an exact name"


def _compose_client() -> DockerClient:
    runtime = load_runtime_config()
    compose = load_compose_config(runtime)
    env_file = runtime.lab_root / ".env"
    return DockerClient(
        compose_files=[compose.compose_file],
        compose_profiles=list(NAVLAB_PROFILES),
        compose_project_name=compose.project_name.value,
        compose_project_directory=runtime.lab_root,
        compose_env_files=([env_file] if env_file.is_file() else []),
    )


@contextmanager
def _compose_environment(config: RunConfig) -> Iterator[None]:
    overrides = {
        **config.orchestration.compose_env(),
        "SESSION_ID": f"{config.session_id}/{config.run_id}",
    }
    previous = {key: os.environ.get(key) for key in overrides}
    os.environ.update(overrides)
    try:
        yield
    finally:
        for key, old_value in previous.items():
            if old_value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old_value


def _compose_up(config: RunConfig) -> None:
    with _compose_environment(config):
        _compose_client().compose.up(services=list(NAVLAB_SERVICES), detach=True, build=False)


def _compose_stop(config: RunConfig) -> None:
    with _compose_environment(config):
        _compose_client().compose.stop(services=list(NAVLAB_STOP_SERVICES))


def _compose_logs(config: RunConfig, *, tail: int) -> str:
    with _compose_environment(config):
        output = _compose_client().compose.logs(services=list(NAVLAB_SERVICES), tail=str(tail))
    return str(output)


def _compose_ps_status(config: RunConfig) -> list[dict[str, object]]:
    with _compose_environment(config):
        containers = _compose_client().compose.ps(services=list(NAVLAB_SERVICES), all=True)
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


def _workspace_path(path: Path) -> str:
    runtime = load_runtime_config()
    try:
        relative = path.resolve().relative_to(runtime.lab_root.resolve())
    except ValueError:
        return str(path)
    return str(Path("/workspace") / relative)


def _official_sitl_workdir(config: RunConfig) -> str:
    return _workspace_path(config.artifact_dir / OFFICIAL_SITL_WORKDIR)


def _companion_runtime_config_path() -> str:
    raw = os.environ.get("NAVLAB_RUNTIME_CONFIG")
    if raw:
        return raw
    return _workspace_path(load_runtime_config().lab_root / DEFAULT_COMPANION_RUNTIME_CONFIG)


def _render_run_config(console: Console, config: RunConfig) -> None:
    table = Table(title="NavLab", show_header=False)
    table.add_column("Key", style="bold")
    table.add_column("Value")
    table.add_row("config", str(config.orchestration.path))
    table.add_row("artifact", str(config.artifact_dir))
    table.add_row("duration", f"{config.duration_sec:g}s")
    table.add_row("ROS_DOMAIN_ID", config.ros_domain_id)
    table.add_row("companion image", config.companion_image)
    table.add_row("slam image", config.slam_image)
    table.add_row("gazebo sensor image", config.gazebo_sensor_image)
    table.add_row("scan source", config.scan_source)
    table.add_row("rosbag profile", config.rosbag_profile)
    console.print(table)


def _capture_stack_logs(*, config: RunConfig) -> None:
    config.artifact_dir.mkdir(parents=True, exist_ok=True)
    try:
        output = _compose_logs(config, tail=400)
    except DockerException as exc:
        output = str(exc)
    try:
        companion_output = DockerClient().logs(COMPANION_CONTAINER, tail=400)
        output = f"{output}\n\n--- {COMPANION_CONTAINER} ---\n{companion_output}"
    except DockerException:
        pass
    try:
        slam_output = DockerClient().logs(SLAM_CONTAINER, tail=400)
        output = f"{output}\n\n--- {SLAM_CONTAINER} ---\n{slam_output}"
    except DockerException:
        pass
    try:
        compose = load_compose_config(load_runtime_config())
        sensor_output = DockerClient().logs(f"{compose.project_name.value}-{GAZEBO_SENSOR_SERVICE}-1", tail=400)
        output = f"{output}\n\n--- {GAZEBO_SENSOR_SERVICE} ---\n{sensor_output}"
    except DockerException:
        pass
    (config.artifact_dir / "navlab_stack_tail.log").write_text(output, encoding="utf-8")


def _capture_compose_service_log(*, config: RunConfig, service: str, output_path: Path, tail: int = 2000) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    compose = load_compose_config(load_runtime_config())
    container_name = f"{compose.project_name.value}-{service}-1"
    try:
        output = DockerClient().logs(container_name, tail=tail)
    except DockerException as exc:
        output = str(exc)
    output_path.write_text(str(output), encoding="utf-8")


def _session_log_dir(config: RunConfig) -> Path:
    return Path("artifacts/sessions") / config.session_id / config.run_id


def _remove_companion_container() -> None:
    try:
        DockerClient().remove(COMPANION_CONTAINER, force=True)
    except DockerException:
        pass


def _remove_slam_container() -> None:
    try:
        DockerClient().remove(SLAM_CONTAINER, force=True)
    except DockerException:
        pass


def _remove_official_baseline_container() -> None:
    try:
        DockerClient().remove(OFFICIAL_BASELINE_CONTAINER, force=True)
    except DockerException:
        pass


def _start_official_baseline_container(
    config: RunConfig,
    *,
    volume_overrides: list[tuple[Path, str]] | None = None,
) -> None:
    _remove_official_baseline_container()
    baseline = config.orchestration.official_baseline
    (config.artifact_dir / OFFICIAL_SITL_WORKDIR).mkdir(parents=True, exist_ok=True)
    sitl_workdir = _official_sitl_workdir(config)
    launch_command = (
        f"{baseline.gazebo_launch} "
        "use_gz_sim_gui:=false "
        "rviz:=false "
        "use_dds_agent:=true "
        "use_gz_sim_server:=true "
        "spawn_robot:=true"
    )
    DockerClient().run(
        baseline.runtime_image,
        [
            "bash",
            "-lc",
            (
                "source /opt/ros/jazzy/setup.bash && "
                "source /opt/navlab_official_ws/install/setup.bash && "
                "export NAVLAB_OFFICIAL_SDF_ROOTS="
                "/opt/navlab_official_ws/install/ardupilot_gazebo/share:"
                "/opt/navlab_official_ws/install/ardupilot_gz_description/share && "
                "export SDF_PATH=${NAVLAB_OFFICIAL_SDF_ROOTS}:${SDF_PATH:-} && "
                "export GZ_SIM_RESOURCE_PATH=${NAVLAB_OFFICIAL_SDF_ROOTS}:${GZ_SIM_RESOURCE_PATH:-} && "
                f"exec {launch_command}"
            ),
        ],
        detach=True,
        name=OFFICIAL_BASELINE_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace"), *(volume_overrides or [])],
        workdir=sitl_workdir,
        envs={
            "SESSION_ID": config.session_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "PYTHONPATH": "/workspace",
        },
    )


def _capture_official_baseline_log(*, config: RunConfig, tail: int = 2000) -> None:
    config.artifact_dir.mkdir(parents=True, exist_ok=True)
    try:
        output = DockerClient().logs(OFFICIAL_BASELINE_CONTAINER, tail=tail)
    except DockerException as exc:
        output = str(exc)
    (config.artifact_dir / "official_baseline_tail.log").write_text(str(output), encoding="utf-8")


def _gazebo_network_namespace() -> str:
    containers = _compose_client().compose.ps(services=["gazebo"], all=True)
    running = [container for container in containers if container.state.running]
    if not running:
        raise RuntimeError("gazebo container is not running")
    return f"container:{running[0].id}"


def _start_slam_container(config: RunConfig) -> None:
    if not config.orchestration.slam.autostart:
        return
    _remove_slam_container()
    slam = config.orchestration.slam
    launch_command = " ".join(
        shlex.quote(arg)
        for arg in [
            "python3",
            "-m",
            "navlab.slam.cli",
            "launch",
            "--config",
            slam.runtime_config,
            "--backend",
            slam.backend,
        ]
    )
    DockerClient().run(
        slam.image,
        [
            "bash",
            "-lc",
            (f"source /opt/ros/jazzy/setup.bash && source /opt/navlab_ws/install/setup.bash && exec {launch_command}"),
        ],
        detach=True,
        name=SLAM_CONTAINER,
        networks=[_gazebo_network_namespace()],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        user=f"{os.getuid()}:{os.getgid()}",
        envs={
            "SESSION_ID": config.session_id,
            "ROS_DOMAIN_ID": config.ros_domain_id,
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "PYTHONPATH": "/workspace",
            "NAVLAB_SLAM_RUNTIME_CONFIG": slam.runtime_config,
        },
    )


def _start_companion_container(config: RunConfig) -> None:
    _remove_companion_container()
    DockerClient().run(
        config.companion_image,
        ["bash", "/usr/local/bin/start-navlab-companion.sh"],
        detach=True,
        name=COMPANION_CONTAINER,
        networks=[_gazebo_network_namespace()],
        volumes=[
            (Path.cwd(), "/workspace"),
            (
                Path.cwd() / "docker/entrypoints/start-navlab-companion.sh",
                "/usr/local/bin/start-navlab-companion.sh",
                "ro",
            ),
        ],
        workdir="/workspace",
        user=f"{os.getuid()}:{os.getgid()}",
        envs={
            "SESSION_ID": config.session_id,
            "ROS_DOMAIN_ID": config.ros_domain_id,
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "PYTHONPATH": "/workspace",
            "NAVLAB_RUNTIME_CONFIG": _companion_runtime_config_path(),
            "MAVLINK20": "1",
        },
    )


def _docker_run_runtime_command(
    *,
    config: RunConfig,
    args: list[str],
    name: str | None = None,
    network: str | None = None,
    module: str = "navlab.companion.cli",
) -> int:
    runtime_command = " ".join(shlex.quote(arg) for arg in ["/opt/companion-venv/bin/python", "-m", module, *args])
    command = [
        "bash",
        "-lc",
        (
            "source /opt/ros/jazzy/setup.bash && "
            "if [ -f /opt/navlab_ws/install/setup.bash ]; then "
            "source /opt/navlab_ws/install/setup.bash; fi && "
            f"{runtime_command}"
        ),
    ]
    try:
        DockerClient().run(
            config.companion_image,
            command,
            remove=True,
            name=name,
            networks=([network] if network else []),
            volumes=[(Path.cwd(), "/workspace")],
            workdir="/workspace",
            envs={
                "NAVLAB_RUNTIME_CONFIG": _companion_runtime_config_path(),
                "ROS_DOMAIN_ID": config.ros_domain_id,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "MAVLINK20": "1",
                "PYTHONPATH": "/workspace",
            },
        )
    except DockerException as exc:
        return exc.return_code or 1
    return 0


def _docker_run_ros_shell_capture(
    *,
    config: RunConfig,
    image: str,
    shell_command: str,
    name: str | None = None,
    network: str | None = None,
    envs: dict[str, str] | None = None,
) -> tuple[int, str]:
    filtered_shell_command = _with_ros_shell_stderr_filter(shell_command)
    command = [
        "bash",
        "-lc",
        (
            "source /opt/ros/jazzy/setup.bash && "
            "if [ -f /opt/navlab_ws/install/setup.bash ]; then "
            "source /opt/navlab_ws/install/setup.bash; fi && "
            "if [ -f /opt/navlab_official_ws/install/setup.bash ]; then "
            "source /opt/navlab_official_ws/install/setup.bash; fi && "
            f"{filtered_shell_command}"
        ),
    ]
    try:
        output = DockerClient().run(
            image,
            command,
            remove=True,
            name=name,
            networks=([network] if network else []),
            volumes=[(Path.cwd(), "/workspace")],
            workdir="/workspace",
            envs={
                "ROS_DOMAIN_ID": config.ros_domain_id,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "PYTHONPATH": "/workspace",
                **(envs or {}),
            },
        )
    except DockerException as exc:
        return exc.return_code or 1, str(exc)
    return 0, str(output)


def _with_ros_shell_stderr_filter(shell_command: str) -> str:
    pattern = shlex.quote(ROS_SHELL_STDERR_FILTER_PATTERN)
    # Use a brace group instead of parentheses so heredoc terminators stay on
    # their own line. `(python - <<'PY' ... PY)` turns the terminator into
    # `PY)`, which breaks bash parsing.
    return f"{{\n{shell_command}\n}} 2> >(grep -v -E {pattern} >&2)"


def _docker_exec_runtime_command(*, args: list[str], module: str = "navlab.companion.cli") -> int:
    runtime_command = " ".join(shlex.quote(arg) for arg in ["/opt/companion-venv/bin/python", "-m", module, *args])
    command = [
        "bash",
        "-lc",
        (
            "source /opt/ros/jazzy/setup.bash && "
            "if [ -f /opt/navlab_ws/install/setup.bash ]; then "
            "source /opt/navlab_ws/install/setup.bash; fi && "
            f"{runtime_command}"
        ),
    ]
    try:
        DockerClient().execute(COMPANION_CONTAINER, command)
    except DockerException as exc:
        return exc.return_code or 1
    return 0
