from __future__ import annotations

import importlib.util
import json
import os
import shlex
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, ClassVar

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from src.config import RealPreflightDependencyConfig, RunConfig, SerialMavlinkConfig
from src.project_config import RuntimeConfig as ProjectRuntimeConfig
from src.project_config import load_orchestration_runtime_backend_config, load_runtime_config
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry

DEFAULT_REAL_ROS_DISTRO = "jazzy"
APT_ROS_PACKAGE_MAP = {
    "mavros": "ros-{distro}-mavros",
    "mavros_msgs": "ros-{distro}-mavros-msgs",
}
ROS_DISTRO_BASE_PACKAGES = ("ros-{distro}-ros-base",)
LOCAL_ROS_BUILD_PACKAGE_TEMPLATES = (
    "ros-{distro}-ament-cmake",
    "ros-{distro}-geometry-msgs",
    "ros-{distro}-nav-msgs",
    "ros-{distro}-rclcpp",
    "ros-{distro}-sensor-msgs",
    "ros-{distro}-std-msgs",
    "ros-{distro}-tf2-msgs",
)
LOCAL_ROS_PACKAGES = {
    "navlab_slam_bringup",
    "navlab_cartographer_adapter",
    "navlab_external_nav_bridge",
    "navlab_slam_imu_bridge",
}
LOCAL_ROS_BUILD_PACKAGES = (
    "navlab_cartographer_adapter",
    "navlab_external_nav_bridge",
    "navlab_fake_odom",
    "navlab_slam_bringup",
    "navlab_slam_imu_bridge",
    "ydlidar_interfaces",
)
LOCAL_ROS_PACKAGE_BASE_PATHS = ("navlab/interfaces", "navlab/slam/ros")


@dataclass(frozen=True, slots=True)
class SerialMavlinkProbe:
    summary: dict[str, Any]
    blockers: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class DependencyProbe:
    summary: dict[str, Any]
    blockers: tuple[str, ...]


def _load_runtime_selection(config: RunConfig):
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


def _build_real_preflight_summary(
    config: RunConfig,
    *,
    serial_mavlink_probe: SerialMavlinkProbe | None = None,
    dependency_probe: DependencyProbe | None = None,
) -> dict[str, Any]:
    blockers: list[str] = []
    if serial_mavlink_probe is None:
        serial_mavlink_probe = _probe_serial_mavlink(config.orchestration.real_preflight.serial_mavlink)
    if dependency_probe is None:
        dependency_probe = _probe_real_preflight_dependencies(
            config.orchestration.real_preflight.dependencies,
            ros_distro=config.orchestration.real_preflight.ros_distro,
            process_service_names=(),
        )
    checked_at = datetime.now(UTC).isoformat().replace("+00:00", "Z")
    try:
        selected = _load_runtime_selection(config)
    except Exception as exc:
        return {
            "ok": False,
            "blocked": True,
            "blockers": [f"runtime_config_invalid:{exc}"],
            "runtime_backend": "unknown",
            "runtime_mode": "unknown",
            "preflight_claim": "evaluated",
            "flight_claim": "not_evaluated",
            "landing_claim": "not_evaluated",
            "checked_at": checked_at,
            "valid_for_sec": config.orchestration.real_preflight.valid_for_sec,
            "ros_distro": config.orchestration.real_preflight.ros_distro,
            "real_preflight": {
                "serial_mavlink": serial_mavlink_probe.summary,
                "dependencies": dependency_probe.summary,
            },
        }

    if selected.backend.value != "process":
        blockers.append(f"runtime_backend_must_be_process:{selected.backend.value}")
    if selected.mode.value != "real":
        blockers.append(f"runtime_mode_must_be_real:{selected.mode.value}")

    if not dependency_probe.summary.get("configured_process_services") and selected.process.services:
        dependency_probe = _probe_real_preflight_dependencies(
            config.orchestration.real_preflight.dependencies,
            ros_distro=config.orchestration.real_preflight.ros_distro,
            process_service_names=tuple(selected.process.services),
        )
    blockers.extend(dependency_probe.blockers)
    blockers.extend(serial_mavlink_probe.blockers)

    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "runtime_backend": selected.backend.value,
        "runtime_mode": selected.mode.value,
        "preflight_claim": "evaluated",
        "flight_claim": "not_evaluated",
        "landing_claim": "not_evaluated",
        "checked_at": checked_at,
        "valid_for_sec": config.orchestration.real_preflight.valid_for_sec,
        "ros_distro": config.orchestration.real_preflight.ros_distro,
        "real_preflight": {
            "serial_mavlink": serial_mavlink_probe.summary,
            "dependencies": dependency_probe.summary,
        },
    }


def _probe_real_preflight_dependencies(
    settings: RealPreflightDependencyConfig,
    *,
    ros_distro: str,
    process_service_names: tuple[str, ...],
) -> DependencyProbe:
    blockers: list[str] = []
    command_groups: list[dict[str, Any]] = []
    for group in settings.required_command_groups:
        selected = ""
        selected_path = ""
        for command in group:
            candidate_path = _command_path_for_distro(command, ros_distro)
            if candidate_path:
                selected = command
                selected_path = candidate_path
                break
        command_groups.append(
            {
                "candidates": list(group),
                "found": bool(selected),
                "selected": selected,
                "path": selected_path,
            }
        )
        if not selected:
            blockers.append(f"required_command_missing:{'|'.join(group)}")

    python_modules: dict[str, bool] = {}
    for module in settings.required_python_modules:
        present = importlib.util.find_spec(module) is not None
        python_modules[module] = present
        if not present:
            blockers.append(f"required_python_module_missing:{module}")

    ros_packages: dict[str, dict[str, Any]] = {}
    ros2 = _command_path_for_distro("ros2", ros_distro)
    if settings.required_ros_packages and not ros2:
        blockers.append("required_command_missing:ros2")
        for package in settings.required_ros_packages:
            ros_packages[package] = {"present": False, "error": f"ros2_distro_not_found:{ros_distro}"}
            blockers.append(f"required_ros_package_missing:{package}")
    else:
        for package in settings.required_ros_packages:
            try:
                result = _run_ros2_pkg_prefix(package, ros_distro=ros_distro)
            except (OSError, subprocess.TimeoutExpired) as exc:
                ros_packages[package] = {"present": False, "error": str(exc)}
                blockers.append(f"required_ros_package_probe_failed:{package}")
                continue
            present = result.returncode == 0 and bool(result.stdout.strip())
            ros_packages[package] = {
                "present": present,
                "prefix": result.stdout.strip(),
                "error": "" if present else (result.stderr or result.stdout or f"rc={result.returncode}").strip(),
            }
            if not present:
                blockers.append(f"required_ros_package_missing:{package}")

    configured_services = sorted(process_service_names)
    service_set = set(process_service_names)
    required_services: dict[str, bool] = {}
    for service in settings.required_process_services:
        present = service in service_set
        required_services[service] = present
        if not present:
            blockers.append(f"required_process_service_missing:{service}")

    return DependencyProbe(
        summary={
            "ros_distro": ros_distro,
            "required_command_groups": command_groups,
            "required_python_modules": python_modules,
            "required_ros_packages": ros_packages,
            "required_process_services": required_services,
            "configured_process_services": configured_services,
        },
        blockers=tuple(dict.fromkeys(blockers)),
    )


def _command_path_for_distro(command: str, ros_distro: str) -> str:
    if command == "ros2":
        path = _ros2_path_for_distro(ros_distro)
        return str(path) if path.exists() else ""
    return shutil.which(command) or ""


def _ros2_path_for_distro(ros_distro: str) -> Path:
    return Path("/opt/ros") / ros_distro / "bin" / "ros2"


def _run_ros2_pkg_prefix(package: str, *, ros_distro: str) -> subprocess.CompletedProcess[str]:
    ros2_command = shlex.quote(str(_ros2_path_for_distro(ros_distro)))
    workspace_setup = Path("install/setup.bash")
    source_commands = _ros_source_commands(ros_distro)
    if workspace_setup.exists():
        source_commands.append(f"source {shlex.quote(str(workspace_setup))}")
        command = " && ".join([*source_commands, f"{ros2_command} pkg prefix {shlex.quote(package)}"])
        return subprocess.run(
            ["bash", "-lc", command],
            check=False,
            capture_output=True,
            text=True,
            timeout=4.0,
        )
    if source_commands:
        command = " && ".join([*source_commands, f"{ros2_command} pkg prefix {shlex.quote(package)}"])
        return subprocess.run(
            ["bash", "-lc", command],
            check=False,
            capture_output=True,
            text=True,
            timeout=4.0,
        )
    return subprocess.run(
        [str(_ros2_path_for_distro(ros_distro)), "pkg", "prefix", package],
        check=False,
        capture_output=True,
        text=True,
        timeout=4.0,
    )


def _probe_serial_mavlink(settings: SerialMavlinkConfig) -> SerialMavlinkProbe:
    summary: dict[str, Any] = {
        "enabled": settings.enabled,
        "port": settings.port,
        "baud": settings.baud,
        "serial_open_ok": False,
        "heartbeat_seen": False,
        "system_id": None,
        "component_id": None,
        "autopilot": "",
        "vehicle_type": "",
        "armed": None,
        "mode": "",
        "message_counts": {},
        "required_messages": list(settings.required_messages),
        "optional_messages": list(settings.optional_messages),
    }
    blockers: list[str] = []
    if not settings.enabled:
        summary["skipped"] = "serial_mavlink disabled"
        return SerialMavlinkProbe(summary=summary, blockers=())

    port = settings.port.strip()
    if _looks_like_network_mavlink_endpoint(port):
        blockers.append(f"serial_mavlink_endpoint_not_serial:{port}")
        return SerialMavlinkProbe(summary=summary, blockers=tuple(blockers))

    path = Path(port)
    if not path.exists():
        blockers.append(f"serial_port_missing:{port}")
        return SerialMavlinkProbe(summary=summary, blockers=tuple(blockers))
    if not os.access(path, os.R_OK | os.W_OK):
        blockers.append(f"serial_port_permission_denied:{port}")
        return SerialMavlinkProbe(summary=summary, blockers=tuple(blockers))

    try:
        import serial
    except ImportError:
        blockers.append("serial_dependency_missing:pyserial")
        return SerialMavlinkProbe(summary=summary, blockers=tuple(blockers))
    try:
        from pymavlink import mavutil
    except ImportError:
        blockers.append("serial_dependency_missing:pymavlink")
        return SerialMavlinkProbe(summary=summary, blockers=tuple(blockers))

    started = time.monotonic()
    try:
        with serial.Serial(  # type: ignore[attr-defined]
            port=port,
            baudrate=settings.baud,
            timeout=min(settings.connection_timeout_sec, 1.0),
            write_timeout=min(settings.connection_timeout_sec, 1.0),
        ):
            summary["serial_open_ok"] = True
            summary["serial_open_elapsed_sec"] = round(time.monotonic() - started, 3)
    except Exception as exc:  # pragma: no cover - exercised through unit-level injected probe.
        blockers.append(f"serial_open_failed:{exc}")
        return SerialMavlinkProbe(summary=summary, blockers=tuple(blockers))

    master = None
    try:
        master = mavutil.mavlink_connection(
            port,
            baud=settings.baud,
            autoreconnect=False,
            source_system=255,
            source_component=0,
        )
        heartbeat = master.wait_heartbeat(timeout=settings.heartbeat_timeout_sec)
        if heartbeat is None:
            if settings.require_autopilot_heartbeat:
                blockers.append("serial_mavlink_heartbeat_missing")
            return SerialMavlinkProbe(summary=summary, blockers=tuple(blockers))

        message_counts: dict[str, int] = {"HEARTBEAT": 1}
        summary["heartbeat_seen"] = True
        summary["system_id"] = int(getattr(heartbeat, "get_srcSystem", lambda: 0)())
        summary["component_id"] = int(getattr(heartbeat, "get_srcComponent", lambda: 0)())
        autopilot_value = int(getattr(heartbeat, "autopilot", 0))
        vehicle_type_value = int(getattr(heartbeat, "type", 0))
        summary["autopilot"] = _mavlink_enum_name(mavutil, "MAV_AUTOPILOT", autopilot_value)
        summary["vehicle_type"] = _mavlink_enum_name(mavutil, "MAV_TYPE", vehicle_type_value)
        summary["armed"] = bool(
            int(getattr(heartbeat, "base_mode", 0))
            & int(getattr(mavutil.mavlink, "MAV_MODE_FLAG_SAFETY_ARMED", 128))
        )
        summary["mode"] = _heartbeat_mode_name(mavutil, heartbeat)

        autopilot_normalized = _normalize_mavlink_enum_name(str(summary["autopilot"]), prefix="MAV_AUTOPILOT_")
        expected_autopilot = _normalize_mavlink_enum_name(settings.expected_autopilot, prefix="MAV_AUTOPILOT_")
        invalid_autopilot = _normalize_mavlink_enum_name("MAV_AUTOPILOT_INVALID", prefix="MAV_AUTOPILOT_")
        if autopilot_normalized == invalid_autopilot or (
            expected_autopilot and autopilot_normalized != expected_autopilot
        ):
            blockers.append("serial_mavlink_autopilot_invalid")
        if settings.require_not_armed and summary["armed"]:
            blockers.append("serial_mavlink_unexpected_armed")
        if settings.require_mode_observed and not summary["mode"]:
            blockers.append("serial_mavlink_mode_missing")

        _request_mavlink_telemetry_streams(master, mavutil)
        summary["telemetry_request_sent"] = True

        deadline = time.monotonic() + settings.telemetry_window_sec
        wanted_messages = set(settings.required_messages) | set(settings.optional_messages)
        while time.monotonic() < deadline:
            msg = master.recv_match(blocking=True, timeout=max(0.0, min(0.5, deadline - time.monotonic())))
            if msg is None:
                continue
            msg_type = str(msg.get_type()).upper()
            if msg_type == "BAD_DATA":
                continue
            if msg_type in wanted_messages:
                message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
            if all(message_counts.get(required, 0) > 0 for required in settings.required_messages):
                break

        if settings.require_system_status and message_counts.get("SYS_STATUS", 0) == 0:
            blockers.append("serial_mavlink_required_message_missing:SYS_STATUS")
        for required in settings.required_messages:
            if message_counts.get(required, 0) == 0:
                blocker = f"serial_mavlink_required_message_missing:{required}"
                if blocker not in blockers:
                    blockers.append(blocker)
        summary["message_counts"] = message_counts
    except Exception as exc:  # pragma: no cover - hardware-specific failure surface.
        blockers.append(f"serial_mavlink_probe_failed:{exc}")
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass

    return SerialMavlinkProbe(summary=summary, blockers=tuple(blockers))


def _request_mavlink_telemetry_streams(master: Any, mavutil: Any) -> None:
    target_system = int(getattr(master, "target_system", 0) or 0)
    target_component = int(getattr(master, "target_component", 0) or 0)
    for stream_id in (
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
    ):
        master.mav.request_data_stream_send(target_system, target_component, stream_id, 4, 1)


def _looks_like_network_mavlink_endpoint(value: str) -> bool:
    lowered = value.lower()
    return lowered.startswith(("tcp:", "udp:", "udpin:", "udpout:", "tcpin:", "tcpout:"))


def _mavlink_enum_name(mavutil: Any, enum_name: str, value: int) -> str:
    enum = getattr(mavutil.mavlink, "enums", {}).get(enum_name, {})
    item = enum.get(value)
    if item is None:
        return str(value)
    return str(getattr(item, "name", value))


def _normalize_mavlink_enum_name(value: str, *, prefix: str) -> str:
    normalized = value.strip().upper()
    if normalized.startswith(prefix):
        normalized = normalized[len(prefix) :]
    return normalized.replace("_", "").replace("-", "").lower()


def _heartbeat_mode_name(mavutil: Any, heartbeat: Any) -> str:
    for helper in ("mode_string_v10", "mode_string_v09"):
        fn = getattr(mavutil, helper, None)
        if fn is None:
            continue
        try:
            return str(fn(heartbeat))
        except Exception:
            continue
    return ""


@TaskRegistry.register
class RealPreflightDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "real-preflight-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check process+real runtime dependency and serial MAVLink preflight contract."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        task_config_path: str | Path | None = None,
        console: Console | None = None,
        prompt_install: bool = False,
        force_install: bool = False,
        soft_dependency_warnings: bool = False,
    ) -> int:
        console = console or Console()
        config = RunConfig.from_config(
            config_path=config_path,
            task_name="real-preflight-doctor",
            task_config_path=task_config_path,
        )
        artifact_dir = Path(
            os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_real_preflight_doctor/{config.run_id}")
        )
        config = RunConfig.from_config(
            config_path=config_path,
            task_name="real-preflight-doctor",
            task_config_path=task_config_path,
            run_id=config.run_id,
            artifact_dir=artifact_dir,
        )
        console.print("Checking real runtime preflight contract")
        serial_probe = _probe_serial_mavlink(config.orchestration.real_preflight.serial_mavlink)
        summary = _build_real_preflight_summary(
            config,
            serial_mavlink_probe=serial_probe,
        )
        summary["config_sources"] = config.config_sources_summary()
        if soft_dependency_warnings:
            _soften_installable_dependency_blockers(summary)
        artifact_dir.mkdir(parents=True, exist_ok=True)
        (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]Real preflight doctor rc={0 if summary['ok'] else 20}[/{color}]")
        _print_real_preflight_console_summary(console, summary=summary, summary_path=artifact_dir / "summary.json")
        missing_ros_packages = _missing_ros_packages(summary)
        if missing_ros_packages and (force_install or prompt_install):
            should_install = force_install or typer.confirm(
                "Install missing real-preflight ROS dependencies now?",
                default=False,
            )
            if should_install:
                install_result = _install_missing_real_preflight_dependencies(
                    missing_ros_packages,
                    ros_distro=config.orchestration.real_preflight.ros_distro,
                    console=console,
                )
                summary["dependency_install"] = install_result
                (artifact_dir / "summary.json").write_text(
                    json.dumps(summary, indent=2, sort_keys=True),
                    encoding="utf-8",
                )
                if not install_result["ok"]:
                    return 20
            else:
                console.print(
                    "[yellow]Skipped dependency install; run `just navlab-doctor --force` to install them.[/yellow]"
                )
        return 0 if summary["ok"] else 20


def _print_real_preflight_console_summary(
    console: Console,
    *,
    summary: dict[str, Any],
    summary_path: Path,
) -> None:
    real_preflight = summary.get("real_preflight", {})
    serial_mavlink = real_preflight.get("serial_mavlink", {})
    dependencies = real_preflight.get("dependencies", {})
    runtime_backend = summary.get("runtime_backend", "unknown")
    runtime_mode = summary.get("runtime_mode", "unknown")
    warnings = [str(item) for item in summary.get("warnings", [])]
    status = "WARNING" if summary.get("ok") and warnings else ("OK" if summary.get("ok") else "BLOCKED")
    border_style = "yellow" if summary.get("ok") and warnings else ("green" if summary.get("ok") else "red")
    table = Table.grid(padding=(0, 2))
    table.add_column(style="bold")
    table.add_column()
    table.add_row("Status", status)
    table.add_row("Runtime", f"{runtime_backend}+{runtime_mode}")
    table.add_row("ROS distro", str(summary.get("ros_distro", dependencies.get("ros_distro", "unknown"))))
    table.add_row("Serial", f"{serial_mavlink.get('port', '')} @ {serial_mavlink.get('baud', '')}")
    table.add_row(
        "Serial state",
        f"open={serial_mavlink.get('serial_open_ok')}, heartbeat={serial_mavlink.get('heartbeat_seen')}",
    )
    table.add_row(
        "FCU",
        (
            f"system={serial_mavlink.get('system_id')}, "
            f"component={serial_mavlink.get('component_id')}, "
            f"autopilot={serial_mavlink.get('autopilot') or 'unknown'}"
        ),
    )
    table.add_row("Mode", f"{serial_mavlink.get('mode') or 'unknown'}, armed={serial_mavlink.get('armed')}")
    table.add_row(
        "Deps",
        (
            f"cmd={_dependency_group_state(dependencies.get('required_command_groups', []))}, "
            f"ros={_dependency_map_state(dependencies.get('required_ros_packages', {}))}, "
            f"py={_dependency_map_state(dependencies.get('required_python_modules', {}))}"
        ),
    )
    table.add_row("Summary", str(summary_path))
    console.print(Panel(table, title="Real Preflight Doctor", border_style=border_style))

    blockers = [str(item) for item in summary.get("blockers", [])]
    if blockers:
        blocker_table = Table.grid()
        blocker_table.add_column()
        for blocker in blockers[:8]:
            blocker_table.add_row(f"- {blocker}")
        if len(blockers) > 8:
            blocker_table.add_row(f"- ... {len(blockers) - 8} more")
        console.print(Panel(blocker_table, title="Blockers", border_style="red"))

    if warnings:
        warning_table = Table.grid()
        warning_table.add_column()
        for warning in warnings[:8]:
            warning_table.add_row(f"- {warning}")
        if len(warnings) > 8:
            warning_table.add_row(f"- ... {len(warnings) - 8} more")
        console.print(Panel(warning_table, title="Warnings", border_style="yellow"))


def _dependency_group_state(groups: list[dict[str, Any]]) -> str:
    if not groups:
        return "none"
    present = sum(1 for item in groups if item.get("found"))
    return f"{present}/{len(groups)}"


def _dependency_map_state(items: dict[str, Any]) -> str:
    if not items:
        return "none"
    present = sum(1 for value in items.values() if value is True or (isinstance(value, dict) and value.get("present")))
    return f"{present}/{len(items)}"


def _installable_dependency_blockers(summary: dict[str, Any]) -> list[str]:
    warnings: list[str] = []
    missing_ros_packages = set(_missing_ros_packages(summary))
    for blocker in summary.get("blockers", []):
        text = str(blocker)
        if text == "required_command_missing:ros2" and missing_ros_packages:
            warnings.append(text)
        if text.startswith("required_ros_package_missing:"):
            package = text.split(":", 1)[1]
            if package in missing_ros_packages:
                warnings.append(text)
    return warnings


def _soften_installable_dependency_blockers(summary: dict[str, Any]) -> None:
    dependency_warnings = _installable_dependency_blockers(summary)
    hard_blockers = [
        str(blocker) for blocker in summary.get("blockers", []) if str(blocker) not in dependency_warnings
    ]
    if dependency_warnings and not hard_blockers:
        summary["preflight_blockers"] = list(summary.get("blockers", []))
        summary["warnings"] = dependency_warnings
        summary["blockers"] = []
        summary["blocked"] = False
        summary["ok"] = True


def _missing_ros_packages(summary: dict[str, Any]) -> list[str]:
    dependencies = summary.get("real_preflight", {}).get("dependencies", {})
    packages = dependencies.get("required_ros_packages", {})
    return [name for name, item in packages.items() if isinstance(item, dict) and not item.get("present")]


def _install_missing_real_preflight_dependencies(
    missing_ros_packages: list[str],
    *,
    ros_distro: str,
    console: Console,
) -> dict[str, Any]:
    apt_packages = [
        APT_ROS_PACKAGE_MAP[package].format(distro=ros_distro)
        for package in missing_ros_packages
        if package in APT_ROS_PACKAGE_MAP
    ]
    local_packages = [package for package in missing_ros_packages if package in LOCAL_ROS_PACKAGES]
    unknown_packages = [
        package
        for package in missing_ros_packages
        if package not in APT_ROS_PACKAGE_MAP and package not in LOCAL_ROS_PACKAGES
    ]
    if not (Path("/opt/ros") / ros_distro / "setup.bash").exists():
        apt_packages = [*(template.format(distro=ros_distro) for template in ROS_DISTRO_BASE_PACKAGES), *apt_packages]
    if local_packages:
        apt_packages.extend(template.format(distro=ros_distro) for template in LOCAL_ROS_BUILD_PACKAGE_TEMPLATES)
    apt_packages = list(dict.fromkeys(apt_packages))
    result: dict[str, Any] = {
        "ok": False,
        "ros_distro": ros_distro,
        "apt_packages": apt_packages,
        "local_packages": local_packages,
        "unknown_packages": unknown_packages,
        "commands": [],
        "errors": [],
    }
    try:
        if apt_packages:
            _run_install_command(
                [*_elevated_command("apt-get"), "install", "-y", "--no-install-recommends", *apt_packages],
                console=console,
                result=result,
            )
        if local_packages:
            if shutil.which("colcon") is None:
                _run_install_command(
                    [
                        *_elevated_command("apt-get"),
                        "install",
                        "-y",
                        "--no-install-recommends",
                        "python3-colcon-common-extensions",
                    ],
                    console=console,
                    result=result,
                )
            source_commands = [*_python_venv_deactivation_commands(), *_ros_source_commands(ros_distro)]
            build_command = " && ".join(
                [
                    *source_commands,
                    "rm -rf "
                    + " ".join(shlex.quote(str(Path("build") / package)) for package in LOCAL_ROS_BUILD_PACKAGES)
                    + " && /usr/bin/colcon build --symlink-install "
                    "--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "
                    "-DPYTHON_EXECUTABLE=/usr/bin/python3 "
                    "--base-paths "
                    + " ".join(shlex.quote(path) for path in LOCAL_ROS_PACKAGE_BASE_PATHS),
                ]
            )
            _run_install_command(["bash", "-lc", build_command], console=console, result=result)
        if unknown_packages:
            result["errors"].append(f"no install recipe for ROS packages: {', '.join(unknown_packages)}")
    except Exception as exc:  # noqa: BLE001 - install path should return a concise summary to the CLI.
        result["errors"].append(str(exc))
    result["ok"] = not result["errors"]
    if result["ok"]:
        console.print("[green]Dependency install completed. Re-run `just navlab-doctor` to verify.[/green]")
    else:
        console.print("[red]Dependency install failed:[/red] " + "; ".join(result["errors"]))
    return result


def _python_venv_deactivation_commands() -> list[str]:
    blocked_bins = {
        str(Path(sys.executable).resolve().parent),
        os.environ.get("CONDA_PREFIX", "") + "/bin" if os.environ.get("CONDA_PREFIX") else "",
        os.environ.get("VIRTUAL_ENV", "") + "/bin" if os.environ.get("VIRTUAL_ENV") else "",
    }
    grep_filters = " ".join(f"| grep -v -F {shlex.quote(path)}" for path in sorted(blocked_bins) if path)
    return [
        "unset VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH",
        f'export PATH="$(printf %s \"$PATH\" | tr : \"\\n\" {grep_filters} | paste -sd:)"',
        'export PATH="/usr/bin:/usr/sbin:/bin:/sbin:$PATH"',
    ]


def _run_install_command(command: list[str], *, console: Console, result: dict[str, Any]) -> None:
    result["commands"].append(command)
    console.print("[yellow]running:[/yellow] " + shlex.join(command))
    subprocess.run(command, check=True)


def _elevated_command(command: str) -> list[str]:
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        return [command]
    sudo = shutil.which("sudo")
    if sudo:
        return [sudo, command]
    raise RuntimeError(f"{command} needs root privileges or sudo")


def _ros_source_commands(distro: str | None = None) -> list[str]:
    resolved = distro or DEFAULT_REAL_ROS_DISTRO
    setup = Path("/opt/ros") / resolved / "setup.bash"
    return [f"source {shlex.quote(str(setup))}"] if setup.exists() else []
