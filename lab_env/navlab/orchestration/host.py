from __future__ import annotations

import os
import shlex
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Literal

from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException
from rich.console import Console
from rich.table import Table

from lab_env.config import (
    NavLabImageConfig,
    NavLabImagesConfig,
    load_compose_config,
    load_navlab_images_config,
    load_runtime_config,
)
from lab_env.navlab.orchestration.artifacts import finalize_navlab_artifact
from lab_env.navlab.orchestration.config import (
    NAVLAB_PROFILES,
    NAVLAB_SERVICES,
    NAVLAB_STOP_SERVICES,
    RunConfig,
)
from lab_env.navlab.orchestration.foxglove_upload import upload_acceptance_rosbag

COMPANION_CONTAINER = "navlab-companion"
SLAM_CONTAINER = "navlab-slam"
ImageKind = Literal["companion", "slam", "all"]


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


def _repo_path(runtime_root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return runtime_root / path


def _image_build_specs(
    image_config: NavLabImagesConfig,
    kind: ImageKind,
) -> tuple[tuple[str, NavLabImageConfig], ...]:
    specs = (
        ("companion", image_config.companion),
        ("slam", image_config.slam),
    )
    if kind == "all":
        return specs
    for spec in specs:
        if spec[0] == kind:
            return (spec,)
    raise ValueError(f"Invalid NavLab image kind '{kind}': expected companion, slam, or all")


def _render_image_build_config(
    console: Console,
    *,
    kind: ImageKind,
    runtime_root: Path,
    specs: tuple[tuple[str, NavLabImageConfig], ...],
    tag: str | None,
) -> None:
    table = Table(title="NavLab Image Build", show_header=False)
    table.add_column("Key", style="bold")
    table.add_column("Value")
    table.add_row("kind", kind)
    table.add_row("tag override", tag or "<none>")
    for label, image_config in specs:
        table.add_row(f"{label} context", str(_repo_path(runtime_root, image_config.context.value)))
        table.add_row(f"{label} dockerfile", str(_repo_path(runtime_root, image_config.dockerfile.value)))
        table.add_row(f"{label} target", image_config.target.value)
        table.add_row(f"{label} tag strategy", image_config.tag_strategy.value)
        table.add_row(f"{label} image", image_config.image(cli_tag=tag, cwd=runtime_root))
    console.print(table)


def build_navlab_images(*, kind: ImageKind = "all", tag: str | None = None, console: Console | None = None) -> int:
    console = console or Console()
    runtime = load_runtime_config()
    image_config = load_navlab_images_config(runtime)
    try:
        specs = _image_build_specs(image_config, kind)
        resolved_specs = tuple(
            (
                label,
                _repo_path(runtime.lab_root, spec.context.value),
                _repo_path(runtime.lab_root, spec.dockerfile.value),
                spec.target.value,
                spec.image(cli_tag=tag, cwd=runtime.lab_root),
            )
            for label, spec in specs
        )
    except ValueError as exc:
        console.print(f"[red]{exc}[/red]")
        return 2
    _render_image_build_config(
        console,
        kind=kind,
        runtime_root=runtime.lab_root,
        specs=specs,
        tag=tag,
    )
    try:
        for label, context_path, dockerfile, target, image in resolved_specs:
            console.print(f"[bold cyan]Building NavLab {label} image[/bold cyan] {image}")
            logs = DockerClient().build(
                context_path,
                file=dockerfile,
                target=target,
                tags=image,
                stream_logs=True,
            )
            if logs is not None:
                for line in logs:
                    console.print(str(line).rstrip())
    except DockerException as exc:
        console.print(f"[red]Docker build failed:[/red] {exc}")
        return exc.return_code or 1
    console.print("[green]NavLab image build completed[/green]")
    return 0


@contextmanager
def _compose_environment(config: RunConfig) -> Iterator[None]:
    overrides = config.orchestration.compose_env()
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


def _runtime_config_path(config: RunConfig) -> str:
    runtime = load_runtime_config()
    profile_path = config.orchestration.path
    try:
        relative = profile_path.resolve().relative_to(runtime.lab_root.resolve())
    except ValueError:
        return str(profile_path)
    return str(Path("/workspace") / relative)


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
    (config.artifact_dir / "navlab_stack_tail.log").write_text(output, encoding="utf-8")


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
    launch_args = [
        "launch_fake_external_nav:=false",
        "launch_cartographer_backend:=true",
        "publish_placeholder_odom:=false",
        "imu_source_mode:=topic",
        f"imu_source_topic:={slam.imu_source_topic}",
        f"imu_source_label:={slam.imu_source_label}",
        f"imu_min_input_rate_hz:={slam.imu_min_input_rate_hz}",
        "require_imu_for_external_nav:=true",
        *slam.args,
    ]
    launch_command = " ".join(
        shlex.quote(arg)
        for arg in [
            "ros2",
            "launch",
            "indoor_bringup",
            "indoor_bringup.launch.py",
            *launch_args,
        ]
    )
    DockerClient().run(
        slam.image,
        [
            "bash",
            "-lc",
            (
                "source /opt/ros/jazzy/setup.bash && "
                "source /opt/navlab_ws/install/setup.bash && "
                f"exec {launch_command}"
            ),
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
            "NAVLAB_CONFIG": _runtime_config_path(config),
            "MAVLINK20": "1",
        },
    )


def _docker_run_runtime_command(
    *,
    config: RunConfig,
    args: list[str],
    name: str | None = None,
    network: str | None = None,
) -> int:
    runtime_command = " ".join(
        shlex.quote(arg) for arg in ["/opt/companion-venv/bin/python", "-m", "lab_env.navlab.runtime.cli", *args]
    )
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
                "NAVLAB_CONFIG": _runtime_config_path(config),
                "ROS_DOMAIN_ID": config.ros_domain_id,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "MAVLINK20": "1",
                "PYTHONPATH": "/workspace",
            },
        )
    except DockerException as exc:
        return exc.return_code or 1
    return 0


def _docker_exec_runtime_command(*, args: list[str]) -> int:
    runtime_command = " ".join(
        shlex.quote(arg) for arg in ["/opt/companion-venv/bin/python", "-m", "lab_env.navlab.runtime.cli", *args]
    )
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


def companion_doctor(*, config_path: str | Path | None = None, console: Console | None = None) -> int:
    console = console or Console()
    config = RunConfig.from_profile(profile_path=config_path)
    artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_companion_doctor/{config.run_id}"))
    artifact_dir.mkdir(parents=True, exist_ok=True)
    console.print("[bold cyan]Checking NavLab companion image[/bold cyan]")
    rc = _docker_run_runtime_command(
        config=config,
        args=[
            "doctor",
            "--summary-file",
            str(artifact_dir / "summary.json"),
            "--image",
            config.companion_image,
        ],
    )
    console.print(f"[green]Doctor summary:[/green] {artifact_dir / 'summary.json'}")
    return rc


def orchestrate_companion_gazebo_acceptance(
    *,
    config_path: str | Path | None = None,
    duration_sec: float = 90.0,
    console: Console | None = None,
) -> int:
    console = console or Console()
    config = RunConfig.from_profile(profile_path=config_path, duration_sec=duration_sec)
    config.artifact_dir.mkdir(parents=True, exist_ok=True)
    _render_run_config(console, config)
    rc = 1
    try:
        console.print("[bold cyan]Starting SITL + Gazebo + companion stack[/bold cyan]")
        _compose_up(config)
        _start_slam_container(config)
        _start_companion_container(config)
        console.print(f"[bold cyan]Running acceptance inside companion for {duration_sec:g}s[/bold cyan]")
        rc = _docker_exec_runtime_command(
            args=[
                "execute-acceptance",
                "--artifact-dir",
                str(config.artifact_dir),
                "--duration-sec",
                str(duration_sec),
                "--rosbag-profile",
                config.rosbag_profile,
                "--companion-image",
                config.companion_image,
                "--config",
                _runtime_config_path(config),
            ],
        )
    finally:
        _capture_stack_logs(config=config)
        finalize_navlab_artifact(
            artifact_dir=config.artifact_dir,
            session_id=config.session_id,
            run_id=config.run_id,
            duration_sec=duration_sec,
            ros_domain_id=config.ros_domain_id,
            rosbag_profile=config.rosbag_profile,
        )
        _remove_companion_container()
        _remove_slam_container()
        try:
            _compose_stop(config)
        except DockerException:
            pass
        upload = upload_acceptance_rosbag(config)
        upload_color = "green" if upload.ok else "yellow"
        console.print(f"[{upload_color}]Foxglove upload:[/{upload_color}] {upload.state} ({upload.reason})")
    color = "green" if rc == 0 else "red"
    console.print(f"[{color}]NavLab acceptance completed rc={rc}[/{color}]")
    console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
    console.print(f"[bold]Foxglove notes:[/bold] {config.artifact_dir / 'foxglove_notes.md'}")
    return rc
