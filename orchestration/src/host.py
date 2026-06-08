from __future__ import annotations

import os
import shlex
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

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
    RuntimeConfig as ProjectRuntimeConfig,
)
from src.project_config import (
    load_compose_config,
    load_orchestration_runtime_backend_config,
    load_runtime_config,
)
from src.runtime import (
    SERVICE_ROLE_OFFICIAL_BASELINE,
    SERVICE_ROLE_SIMULATION_PROBE,
    SERVICE_ROLE_SIMULATION_RUNTIME,
    SERVICE_ROLE_SIMULATION_STACK,
    DockerBackend,
    ProbeSpec,
    RuntimeModePolicy,
    ServiceSpec,
    ServiceWaitError,
    VolumeMount,
    WorkspacePathMapper,
)

COMPANION_CONTAINER = "navlab-companion"
SLAM_CONTAINER = "navlab-slam"
OFFICIAL_BASELINE_CONTAINER = "navlab-official-baseline"
GAZEBO_SENSOR_SERVICE = "gazebo-sensor"
DEFAULT_COMPANION_RUNTIME_CONFIG = Path("navlab/config.toml")
OFFICIAL_SITL_WORKDIR = "sitl_work"
ROS_SHELL_STDERR_FILTER_PATTERN = r"Failed to parse type hash|share an exact name"


def _compose_backend() -> DockerBackend:
    runtime = load_runtime_config()
    compose = load_compose_config(runtime)
    env_file = runtime.lab_root / ".env"
    return DockerBackend(
        compose_files=(compose.compose_file,),
        compose_profiles=tuple(NAVLAB_PROFILES),
        compose_project_name=compose.project_name.value,
        compose_project_directory=runtime.lab_root,
        compose_env_files=((env_file,) if env_file.is_file() else ()),
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
    _assert_runtime_service_role(config, service_name="compose_stack", service_role=SERVICE_ROLE_SIMULATION_STACK)
    with _compose_environment(config):
        _compose_backend().compose_up(services=tuple(NAVLAB_SERVICES), detach=True, build=False)


def _compose_stop(config: RunConfig) -> None:
    with _compose_environment(config):
        _compose_backend().compose_stop(services=tuple(NAVLAB_STOP_SERVICES))


def _compose_logs(config: RunConfig, *, tail: int) -> str:
    with _compose_environment(config):
        return _compose_backend().compose_logs(services=tuple(NAVLAB_SERVICES), tail=tail)


def _compose_ps_status(config: RunConfig) -> list[dict[str, object]]:
    with _compose_environment(config):
        return _compose_backend().compose_ps_status(services=tuple(NAVLAB_SERVICES))


def _workspace_path(path: Path) -> str:
    runtime = load_runtime_config()
    mapper = WorkspacePathMapper(host_root=runtime.lab_root, backend_workspace_root="/workspace", backend="docker")
    try:
        return mapper.backend_path(path)
    except Exception:
        return str(path)


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
    table.add_row("runtime backend", _runtime_backend_name(config))
    table.add_row("runtime mode", _runtime_mode_name(config))
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
    backend = DockerBackend()
    try:
        companion_output = backend.logs(COMPANION_CONTAINER, tail=400)
        output = f"{output}\n\n--- {COMPANION_CONTAINER} ---\n{companion_output}"
    except ServiceWaitError:
        pass
    try:
        slam_output = backend.logs(SLAM_CONTAINER, tail=400)
        output = f"{output}\n\n--- {SLAM_CONTAINER} ---\n{slam_output}"
    except ServiceWaitError:
        pass
    try:
        compose = load_compose_config(load_runtime_config())
        sensor_output = backend.logs(f"{compose.project_name.value}-{GAZEBO_SENSOR_SERVICE}-1", tail=400)
        output = f"{output}\n\n--- {GAZEBO_SENSOR_SERVICE} ---\n{sensor_output}"
    except ServiceWaitError:
        pass
    (config.artifact_dir / "navlab_stack_tail.log").write_text(output, encoding="utf-8")


def _capture_compose_service_log(*, config: RunConfig, service: str, output_path: Path, tail: int = 2000) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    compose = load_compose_config(load_runtime_config())
    container_name = f"{compose.project_name.value}-{service}-1"
    try:
        output = DockerBackend().logs(container_name, tail=tail)
    except ServiceWaitError as exc:
        output = str(exc)
    output_path.write_text(str(output), encoding="utf-8")


def _selected_runtime_backend_config(config: RunConfig):
    runtime = load_runtime_config()
    return load_orchestration_runtime_backend_config(
        ProjectRuntimeConfig(
            lab_root=runtime.lab_root,
            ardupilot_root=runtime.ardupilot_root,
            mavlink_router_root=runtime.mavlink_router_root,
            venv_path=runtime.venv_path,
            config_file=config.orchestration.path,
            config_loaded=config.orchestration.path.is_file(),
        )
    )


def _runtime_backend_summary(config: RunConfig) -> dict[str, str | bool]:
    selected = _selected_runtime_backend_config(config)
    return {
        "backend": selected.backend.value,
        "mode": selected.mode.value,
        "backend_source": selected.backend.source,
        "mode_source": selected.mode.source,
        "backend_config_path": str(config.orchestration.path),
        "fail_on_missing_backend_config": selected.fail_on_missing_backend_config,
        "fail_on_mode_violation": selected.fail_on_mode_violation,
    }


def _runtime_source_claims(config: RunConfig) -> dict[str, str]:
    selected = _selected_runtime_backend_config(config)
    if selected.mode.value == "real":
        return {
            "fcu": selected.real_sources.fcu_source_claim.value,
            "scan": selected.real_sources.scan_source_claim.value,
            "scan_topic": selected.real_sources.scan_source_topic.value,
            "imu": selected.real_sources.imu_source_claim.value,
            "rangefinder": selected.real_sources.rangefinder_source_claim.value,
            "slam": selected.real_sources.slam_source_claim.value,
        }
    return {
        "fcu": "simulation_sitl_dds",
        "scan": "gazebo_lidar_via_x2",
        "scan_topic": "/scan",
        "imu": "simulation_fcu_or_gazebo",
        "rangefinder": "simulation_gazebo_rangefinder",
        "slam": "simulation_slam",
    }


def _runtime_backend_name(config: RunConfig) -> str:
    return str(_runtime_backend_summary(config)["backend"])


def _runtime_mode_name(config: RunConfig) -> str:
    return str(_runtime_backend_summary(config)["mode"])


def _runtime_mode_policy(config: RunConfig) -> RuntimeModePolicy:
    selected = _selected_runtime_backend_config(config)
    return RuntimeModePolicy(
        backend=selected.backend.value,
        mode=selected.mode.value,
        fail_on_mode_violation=selected.fail_on_mode_violation,
    )


def _assert_runtime_service_role(config: RunConfig, *, service_name: str, service_role: str) -> None:
    _runtime_mode_policy(config).assert_service_allowed(service_name=service_name, service_role=service_role)


def _session_log_dir(config: RunConfig) -> Path:
    return Path("artifacts/sessions") / config.session_id / config.run_id


def _volume_mounts(items: list[tuple[Path, str]] | list[tuple[Path, str, str]]) -> tuple[VolumeMount, ...]:
    mounts: list[VolumeMount] = []
    for item in items:
        mounts.append(VolumeMount(*item))
    return tuple(mounts)


def _remove_companion_container() -> None:
    try:
        DockerBackend().remove_container(COMPANION_CONTAINER, force=True)
    except ServiceWaitError:
        pass


def _remove_slam_container() -> None:
    try:
        DockerBackend().remove_container(SLAM_CONTAINER, force=True)
    except ServiceWaitError:
        pass


def _remove_official_baseline_container() -> None:
    try:
        DockerBackend().remove_container(OFFICIAL_BASELINE_CONTAINER, force=True)
    except ServiceWaitError:
        pass


def _start_official_baseline_container(
    config: RunConfig,
    *,
    volume_overrides: list[tuple[Path, str]] | None = None,
) -> None:
    _assert_runtime_service_role(
        config,
        service_name="official_baseline",
        service_role=SERVICE_ROLE_OFFICIAL_BASELINE,
    )
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
    DockerBackend().start_service(
        ServiceSpec(
            name="official_baseline",
            image=baseline.runtime_image,
            command=(
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
            ),
            container_name=OFFICIAL_BASELINE_CONTAINER,
            networks=("host",),
            volumes=(VolumeMount(Path.cwd(), "/workspace"), *(_volume_mounts(volume_overrides or []))),
            cwd=sitl_workdir,
            env={
                "SESSION_ID": config.session_id,
                "ROS_DOMAIN_ID": baseline.dds_domain_id,
                "RMW_IMPLEMENTATION": baseline.rmw_implementation,
                "DDS_ENABLE": baseline.dds_enable,
                "DDS_DOMAIN_ID": baseline.dds_domain_id,
                "PYTHONPATH": "/workspace",
            },
            service_role=SERVICE_ROLE_OFFICIAL_BASELINE,
        )
    )


def _capture_official_baseline_log(*, config: RunConfig, tail: int = 2000) -> None:
    config.artifact_dir.mkdir(parents=True, exist_ok=True)
    try:
        output = DockerBackend().logs(OFFICIAL_BASELINE_CONTAINER, tail=tail)
    except ServiceWaitError as exc:
        output = str(exc)
    (config.artifact_dir / "official_baseline_tail.log").write_text(str(output), encoding="utf-8")


def _gazebo_network_namespace() -> str:
    running = [item for item in _compose_backend().compose_ps_status(services=("gazebo",)) if item.get("running")]
    if not running:
        raise RuntimeError("gazebo container is not running")
    return f"container:{running[0]['id']}"


def _start_slam_container(config: RunConfig) -> None:
    if not config.orchestration.slam.autostart:
        return
    _assert_runtime_service_role(config, service_name="slam", service_role=SERVICE_ROLE_SIMULATION_RUNTIME)
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
    DockerBackend().start_service(
        ServiceSpec(
            name="slam",
            image=slam.image,
            command=(
                "bash",
                "-lc",
                f"source /opt/ros/jazzy/setup.bash && source /opt/navlab_ws/install/setup.bash && exec {launch_command}",
            ),
            container_name=SLAM_CONTAINER,
            networks=(_gazebo_network_namespace(),),
            volumes=(VolumeMount(Path.cwd(), "/workspace"),),
            cwd="/workspace",
            user=f"{os.getuid()}:{os.getgid()}",
            env={
                "SESSION_ID": config.session_id,
                "ROS_DOMAIN_ID": config.ros_domain_id,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "PYTHONPATH": "/workspace",
                "NAVLAB_SLAM_RUNTIME_CONFIG": slam.runtime_config,
            },
            service_role=SERVICE_ROLE_SIMULATION_RUNTIME,
        )
    )


def _start_companion_container(config: RunConfig) -> None:
    _assert_runtime_service_role(config, service_name="companion", service_role=SERVICE_ROLE_SIMULATION_RUNTIME)
    _remove_companion_container()
    DockerBackend().start_service(
        ServiceSpec(
            name="companion",
            image=config.companion_image,
            command=("bash", "/usr/local/bin/start-navlab-companion.sh"),
            container_name=COMPANION_CONTAINER,
            networks=(_gazebo_network_namespace(),),
            volumes=(
                VolumeMount(Path.cwd(), "/workspace"),
                VolumeMount(
                    Path.cwd() / "docker/entrypoints/start-navlab-companion.sh",
                    "/usr/local/bin/start-navlab-companion.sh",
                    "ro",
                ),
            ),
            cwd="/workspace",
            user=f"{os.getuid()}:{os.getgid()}",
            env={
                "SESSION_ID": config.session_id,
                "ROS_DOMAIN_ID": config.ros_domain_id,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "PYTHONPATH": "/workspace",
                "NAVLAB_RUNTIME_CONFIG": _companion_runtime_config_path(),
                "MAVLINK20": "1",
            },
            service_role=SERVICE_ROLE_SIMULATION_RUNTIME,
        )
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
        _assert_runtime_service_role(
            config,
            service_name=name or "runtime_command",
            service_role=SERVICE_ROLE_SIMULATION_RUNTIME,
        )
        DockerBackend().start_service(
            ServiceSpec(
                name=name or "runtime_command",
                image=config.companion_image,
                command=tuple(command),
                container_name=name,
                networks=((network,) if network else ()),
                volumes=(VolumeMount(Path.cwd(), "/workspace"),),
                cwd="/workspace",
                remove=True,
                detach=False,
                env={
                    "NAVLAB_RUNTIME_CONFIG": _companion_runtime_config_path(),
                    "ROS_DOMAIN_ID": config.ros_domain_id,
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                    "MAVLINK20": "1",
                    "PYTHONPATH": "/workspace",
                },
                service_role=SERVICE_ROLE_SIMULATION_RUNTIME,
            )
        )
    except Exception:
        return 1
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
    _assert_runtime_service_role(
        config,
        service_name=name or "ros_shell_capture",
        service_role=SERVICE_ROLE_SIMULATION_PROBE,
    )
    result = DockerBackend().run_probe(
        ProbeSpec(
            name=name or "ros_shell_capture",
            image=image,
            command=tuple(command),
            container_name=name,
            networks=((network,) if network else ()),
            volumes=(VolumeMount(Path.cwd(), "/workspace"),),
            cwd="/workspace",
            env={
                "ROS_DOMAIN_ID": config.ros_domain_id,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "PYTHONPATH": "/workspace",
                **(envs or {}),
            },
            service_role=SERVICE_ROLE_SIMULATION_PROBE,
        )
    )
    return result.return_code, result.stdout


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
        return DockerBackend().execute(COMPANION_CONTAINER, tuple(command))
    except ServiceWaitError:
        return 1
