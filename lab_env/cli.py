from __future__ import annotations

from typing import Annotated

import typer
from python_on_whales.exceptions import DockerException
from rich.console import Console
from rich.table import Table

from lab_env.config import (
    find_binary,
    load_compose_config,
    load_fast_lio_config,
    load_foxglove_config,
    load_gazebo_config,
    load_rosbag_config,
    load_router_config,
    load_runtime_config,
)
from lab_env.docker_utils import (
    compose_down,
    compose_up,
    get_compose_service_state,
    get_fast_lio_status,
    get_foxglove_status,
    get_gazebo_status,
    get_mavlink_router_status,
    get_rosbag_play_status,
    get_rosbag_status,
    get_sitl_status,
    resolve_service_names,
    services_for_selected_profile,
    summarize_service,
)

app = typer.Typer(add_completion=False, no_args_is_help=True)
console = Console()

SERVICE_TITLES = {
    "fast-lio": "FAST-LIO",
    "foxglove": "Foxglove",
    "gazebo": "Gazebo",
    "mavlink-router": "Mavlink Router",
    "rosbag-play": "Rosbag Play",
    "rosbag-record": "Rosbag Record",
    "sitl": "SITL",
}


def _status(
    label: str,
    detail: str,
    *,
    ok: bool = True,
    warning: bool = False,
) -> tuple[str, str, str]:
    if ok:
        return ("ok", label, detail)
    if warning:
        return ("warn", label, detail)
    return ("fail", label, detail)


def _from_label(source: str) -> str:
    return f"(from {source})"


def _format_router_endpoints(endpoints: list[object]) -> str:
    formatted: list[str] = []
    for endpoint in endpoints:
        name = getattr(endpoint, "name", None)
        value = getattr(endpoint, "endpoint", None)
        if isinstance(name, str) and isinstance(value, str):
            formatted.append(f"{name}={value}")
    return ", ".join(formatted)


def _bool_label(value: bool | None) -> str:
    if value is None:
        return "<not checked>"
    return str(value).lower()


def _count_label(value: int | None) -> str:
    if value is None:
        return "<not checked>"
    return str(value)


def _type_match_label(actual: str | None, expected: str) -> str:
    if actual is None:
        return "<not checked>"
    return "true" if actual == expected else "false"


def _topic_probe_label(
    value: str | None,
    *,
    topic_present: bool | None,
    empty_message: str,
) -> str:
    if value:
        return value
    if topic_present:
        return empty_message
    if topic_present is None:
        return "<not checked>"
    return "<unavailable>"


def _joined_rows(values: list[str] | None) -> str:
    if values is None:
        return "<not checked>"
    if not values:
        return "<none>"
    return "\n".join(values)


def _cpu_mem_label(cpu_percent: float | None, memory_used: str, memory_limit: str) -> str:
    if cpu_percent is None:
        return f"<not running> / {memory_used} / {memory_limit}"
    return f"{cpu_percent:.2f}% / {memory_used} / {memory_limit}"


def _render_status_table(title: str, rows: list[tuple[str, str, str]]) -> None:
    console.rule(title)
    table = Table(show_header=False, box=None)
    table.add_column(style="bold")
    table.add_column()
    for state, label, detail in rows:
        style = {"ok": "green", "warn": "yellow", "fail": "red"}[state]
        table.add_row(f"[{style}][{state}][/{style}] {label}", detail)
    console.print(table)


def _detail_rows_for_service(service: str, *, verbose: bool) -> list[tuple[str, str]]:
    runtime = load_runtime_config()

    if service == "gazebo":
        gazebo = load_gazebo_config(runtime)
        status = get_gazebo_status(gazebo)
        return [
            ("container name", status.container_name),
            ("pid", str(status.pid or "<unknown>")),
            ("world path", status.world_path),
            ("uptime", status.uptime),
            ("cpu / mem", _cpu_mem_label(status.cpu_percent, status.memory_used, status.memory_limit)),
            ("gz topic count", str(status.gz_topic_count)),
            ("gz service count", str(status.gz_service_count)),
        ]

    if service == "foxglove":
        foxglove = load_foxglove_config(runtime)
        status = get_foxglove_status(foxglove)
        return [
            ("container name", status.container_name),
            ("pid", str(status.pid or "<unknown>")),
            ("port", status.port),
            ("uptime", status.uptime),
            ("cpu / mem", _cpu_mem_label(status.cpu_percent, status.memory_used, status.memory_limit)),
            ("websocket", f"ws://<server-ip>:{status.port}"),
        ]

    if service == "rosbag-record":
        rosbag = load_rosbag_config(runtime)
        status = get_rosbag_status(rosbag)
        return [
            ("container name", status.container_name),
            ("pid", str(status.pid or "<unknown>")),
            ("session id", status.session_id),
            ("topic file", status.topic_file),
            ("output dir", status.output_dir),
            ("uptime", status.uptime),
            ("cpu / mem", _cpu_mem_label(status.cpu_percent, status.memory_used, status.memory_limit)),
            ("topic count", str(status.topic_count)),
        ]

    if service == "fast-lio":
        fast_lio = load_fast_lio_config(runtime)
        status = get_fast_lio_status(fast_lio)
        rows = [
            ("container name", status.container_name),
            ("state", status.state),
            ("pid", str(status.pid or "<unknown>")),
            ("exit code", str(status.exit_code if status.exit_code is not None else "<unknown>")),
            ("config path", status.config_path),
            ("config exists", "<not checked>" if status.config_exists is None else str(status.config_exists).lower()),
            ("rviz enabled", status.rviz_enabled),
            ("package prefix", status.package_prefix),
            ("uptime", status.uptime),
            ("cpu / mem", _cpu_mem_label(status.cpu_percent, status.memory_used, status.memory_limit)),
            ("ros node count", _count_label(status.ros_node_count)),
            ("ros topic count", _count_label(status.ros_topic_count)),
            ("laser_mapping node", _bool_label(status.fastlio_node_present)),
            ("topic /points", _bool_label(status.input_points_present)),
            (
                "latest /points stamp",
                _topic_probe_label(
                    status.input_points_stamp,
                    topic_present=status.input_points_present,
                    empty_message="<no messages in probe window>",
                ),
            ),
            ("topic /imu", _bool_label(status.input_imu_present)),
            (
                "latest /imu stamp",
                _topic_probe_label(
                    status.input_imu_stamp,
                    topic_present=status.input_imu_present,
                    empty_message="<no messages in probe window>",
                ),
            ),
            ("topic /path", _bool_label(status.output_path_present)),
            ("topic /cloud_registered", _bool_label(status.output_cloud_registered_present)),
            (
                "topic /cloud_registered rate",
                (
                    ("<no messages in probe window>" if status.output_cloud_registered_present else "<unavailable>")
                    if status.output_cloud_registered_rate_hz is None
                    else f"{status.output_cloud_registered_rate_hz:.2f} Hz"
                ),
            ),
        ]
        if not verbose:
            return rows

        rows.extend(
            [
                ("topic /points publishers", _count_label(status.points_publisher_count)),
                ("topic /points subscribers", _count_label(status.points_subscriber_count)),
                ("topic /points type", status.points_type or "<not checked>"),
                ("topic /points type ok", _type_match_label(status.points_type, "sensor_msgs/msg/PointCloud2")),
                ("topic /imu publishers", _count_label(status.imu_publisher_count)),
                ("topic /imu subscribers", _count_label(status.imu_subscriber_count)),
                ("topic /imu type", status.imu_type or "<not checked>"),
                ("topic /imu type ok", _type_match_label(status.imu_type, "sensor_msgs/msg/Imu")),
                ("topic /path publishers", _count_label(status.path_publisher_count)),
                ("topic /path subscribers", _count_label(status.path_subscriber_count)),
                ("topic /path type", status.path_type or "<not checked>"),
                ("topic /cloud_registered publishers", _count_label(status.cloud_registered_publisher_count)),
                ("topic /cloud_registered subscribers", _count_label(status.cloud_registered_subscriber_count)),
                ("topic /cloud_registered type", status.cloud_registered_type or "<not checked>"),
                ("laser_mapping subscribes", _joined_rows(status.fastlio_subscriptions)),
                ("laser_mapping publishes", _joined_rows(status.fastlio_publishers)),
            ]
        )
        return rows

    if service == "sitl":
        status = get_sitl_status()
        return [
            ("container name", status.container_name),
            ("pid", str(status.pid or "<unknown>")),
            ("session id", status.session_id),
            ("upstream endpoint", status.upstream_endpoint),
            ("router only", status.router_only),
            ("uptime", status.uptime),
            ("cpu / mem", _cpu_mem_label(status.cpu_percent, status.memory_used, status.memory_limit)),
        ]

    if service == "mavlink-router":
        router = load_router_config(runtime)
        status = get_mavlink_router_status(router)
        return [
            ("container name", status.container_name),
            ("pid", str(status.pid or "<unknown>")),
            ("session id", status.session_id),
            ("listen", status.listen),
            ("tcp port", status.tcp_port),
            ("downstream endpoints", status.downstream_endpoints),
            ("uptime", status.uptime),
            ("cpu / mem", _cpu_mem_label(status.cpu_percent, status.memory_used, status.memory_limit)),
        ]

    if service == "rosbag-play":
        status = get_rosbag_play_status()
        return [
            ("container name", status.container_name),
            ("pid", str(status.pid or "<unknown>")),
            ("session id", status.session_id),
            ("play bag dir", status.play_bag_dir),
            ("play args", status.play_args),
            ("uptime", status.uptime),
            ("cpu / mem", _cpu_mem_label(status.cpu_percent, status.memory_used, status.memory_limit)),
        ]

    raise ValueError(f"unsupported service: {service}")


def _doctor_rows_for_service(service: str, *, verbose: bool) -> tuple[str, list[tuple[str, str, str]]]:
    try:
        compose_state = get_compose_service_state(service)
        summary = summarize_service(service)
    except (DockerException, IndexError, ValueError) as exc:
        return "fail", [_status("compose", str(exc), ok=False)]

    rows = [_status("compose", summary.detail, ok=summary.level == "ok", warning=summary.level == "warn")]
    if compose_state.state != "running":
        return summary.level, rows

    try:
        for label, detail in _detail_rows_for_service(service, verbose=verbose):
            rows.append(_status(label, detail))
    except (DockerException, IndexError, ValueError) as exc:
        rows.append(_status("status", str(exc), ok=False))
        return "fail", rows
    return summary.level, rows


def _run_doctor(
    selected_services: tuple[str, ...],
    *,
    active_profile: str,
    profile_source: str,
    target_label: str,
    verbose: bool,
) -> None:
    runtime = load_runtime_config()
    router = load_router_config(runtime)
    compose = load_compose_config(runtime)

    failures = 0
    warnings = 0

    preflight_rows: list[tuple[str, str, str]] = []
    if runtime.config_loaded:
        preflight_rows.append(_status("config.toml", str(runtime.config_file)))
    else:
        warnings += 1
        preflight_rows.append(
            _status(
                "config.toml",
                f"not found: {runtime.config_file}, using built-in defaults",
                ok=False,
                warning=True,
            )
        )

    docker_path = find_binary("docker")
    if docker_path:
        preflight_rows.append(_status("docker", docker_path))
    else:
        failures += 1
        preflight_rows.append(_status("docker", "not found", ok=False))

    if compose.compose_file.is_file():
        preflight_rows.append(_status("compose file", str(compose.compose_file)))
    else:
        failures += 1
        preflight_rows.append(_status("compose file", f"missing: {compose.compose_file}", ok=False))

    console.print("Remote SITL Lab doctor")
    _render_status_table("Preflight", preflight_rows)

    console.rule("Runtime")
    runtime_table = Table(show_header=False, box=None)
    runtime_table.add_column(style="bold")
    runtime_table.add_column()
    runtime_table.add_column(style="dim")
    runtime_table.add_row("LAB_ROOT", str(runtime.lab_root), "(derived from repo root)")
    runtime_table.add_row("COMPOSE_PROJECT", compose.project_name.value, _from_label(compose.project_name.source))
    runtime_table.add_row("COMPOSE_PROFILE", active_profile, _from_label(profile_source))
    runtime_table.add_row("DOCTOR_TARGET", target_label, "(resolved selection)")
    runtime_table.add_row("ROUTER_LISTEN", router.listen.value, _from_label(router.listen.source))
    runtime_table.add_row("ROUTER_TCP_PORT", router.tcp_port.value, _from_label(router.tcp_port.source))
    runtime_table.add_row(
        "ROUTER_ENDPOINTS",
        _format_router_endpoints(router.endpoints) or "<none>",
        _from_label(router.endpoints_source),
    )
    console.print(runtime_table)

    for service in selected_services:
        level, rows = _doctor_rows_for_service(service, verbose=verbose)
        if level == "fail":
            failures += 1
        elif level == "warn":
            warnings += 1
        _render_status_table(f"Service: {SERVICE_TITLES.get(service, service)}", rows)

    console.rule("Summary")
    if failures:
        console.print(f"[red]Doctor finished: {failures} failure(s), {warnings} warning(s)[/red]")
        console.print("Next steps:")
        console.print("  - Start the required compose services with `up` before running doctor again")
        console.print("  - Use `doctor --profile <profile> --service <service>` to focus on one container")
        raise typer.Exit(1)

    console.print(f"[green]Doctor finished: 0 failure(s), {warnings} warning(s)[/green]")


@app.command("doctor")
def doctor(
    service: Annotated[
        str,
        typer.Option(
            "--service", help="Use 'all' to check the selected profile, or pass a single compose service name"
        ),
    ] = "all",
    profile: Annotated[
        str | None,
        typer.Option("--profile", help="Compose profile to inspect before optionally narrowing to one service"),
    ] = None,
    verbose: Annotated[
        bool,
        typer.Option("--verbose", "-v", help="Show extended diagnostics when a service exposes them"),
    ] = False,
) -> None:
    runtime = load_runtime_config()
    compose = load_compose_config(runtime)
    final_profile = profile or compose.default_profile.value
    profile_source = "cli override" if profile is not None else compose.default_profile.source

    try:
        selected_services = resolve_service_names(profile=profile, service=service)
    except ValueError as exc:
        console.print(f"[red]Doctor failed:[/red] {exc}")
        raise typer.Exit(1) from exc
    _run_doctor(
        selected_services,
        active_profile=final_profile,
        profile_source=profile_source,
        target_label=f"profile={final_profile}, service={service}",
        verbose=verbose,
    )


@app.command("up")
def up(
    profile: Annotated[str | None, typer.Option("--profile", help="Compose profile to start")] = None,
    skip_doctor: Annotated[
        bool,
        typer.Option("--skip-doctor", help="Skip service checks after startup"),
    ] = False,
) -> None:
    runtime = load_runtime_config()
    compose = load_compose_config(runtime)
    final_profile = profile or compose.default_profile.value

    try:
        compose_up(final_profile)
    except DockerException as exc:
        console.print(f"[red]Up failed:[/red] {exc}")
        raise typer.Exit(1) from exc

    if skip_doctor:
        return
    try:
        selected_services = services_for_selected_profile(final_profile)
    except ValueError as exc:
        console.print(f"[red]Up failed:[/red] {exc}")
        raise typer.Exit(1)
    _run_doctor(
        selected_services,
        active_profile=final_profile,
        profile_source=("cli override" if profile is not None else compose.default_profile.source),
        target_label=final_profile,
        verbose=False,
    )


@app.command("down")
def down(
    remove_orphans: Annotated[
        bool,
        typer.Option("--remove-orphans", help="Also remove services left from old profiles"),
    ] = False,
) -> None:
    try:
        compose_down(remove_orphans=remove_orphans)
    except DockerException as exc:
        console.print(f"[red]Down failed:[/red] {exc}")
        raise typer.Exit(1) from exc
