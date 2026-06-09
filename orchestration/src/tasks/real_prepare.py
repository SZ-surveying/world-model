from __future__ import annotations

import ast
import fnmatch
import json
import os
import shlex
import shutil
import subprocess
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from src.config import RealPrepareServiceConfig, RunConfig
from src.runtime.errors import BackendConfigError, ServiceStartError
from src.runtime.process_backend import ProcessBackend
from src.runtime.specs import RuntimeHandle, ServiceSpec
from src.tasks.fcu_bridge import FcuBridgeModeSpec, get_fcu_bridge_mode

SIMULATION_TOKENS = ("gazebo", "sitl", "gazebo-sensor", "/scan_ideal", "/sim/x2/status")
GEOGRAPHICLIB_DATA_DIR = Path("build/geographiclib")
GEOGRAPHICLIB_GEOID = GEOGRAPHICLIB_DATA_DIR / "geoids/egm96-5.pgm"
ROS_HUMBLE_CARTOGRAPHER_OVERLAY = Path("build/ros_humble_cartographer_overlay/opt/ros/humble")


@dataclass(frozen=True, slots=True)
class TopicEvidence:
    type_name: str = ""
    fresh: bool = True
    frame_id: str = ""
    source_claim: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class RealTopicSnapshot:
    topics: Mapping[str, TopicEvidence]
    collected_at: str = ""
    error: str = ""


@dataclass(slots=True)
class RealPreparePhaseResult:
    return_code: int
    summary: dict[str, Any]
    backend: ProcessBackend
    handles: list[RuntimeHandle]


def run_real_prepare(
    *,
    task_name: str,
    config_path: str | Path | None = None,
    console: Console | None = None,
) -> int:
    return execute_real_prepare_phase(task_name=task_name, config_path=config_path, console=console).return_code


def execute_real_prepare_phase(
    *,
    task_name: str,
    config_path: str | Path | None = None,
    console: Console | None = None,
) -> RealPreparePhaseResult:
    config = RunConfig.from_config(
        config_path=config_path,
        task_name="real-prepare",
        artifact_dir=os.environ.get("ARTIFACT_DIR"),
        run_id=os.environ.get("RUN_ID"),
    )
    artifact_dir = Path(
        os.environ.get("ARTIFACT_DIR", f"{config.orchestration.real_prepare.summary_artifact_dir}/{config.run_id}")
    )
    log_dir = Path(config.orchestration.real_prepare.process_log_dir) / config.run_id
    backend = ProcessBackend(default_log_dir=log_dir, dry_run=config.orchestration.real_prepare.dry_run)
    handles: list[RuntimeHandle] = []
    summary = _build_real_prepare_summary(
        config,
        task_name=task_name,
        backend=backend,
        started_handles=handles,
        artifact_dir=artifact_dir,
        log_dir=log_dir,
    )
    summary_path = artifact_dir / "summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    _print_real_prepare_summary(console or Console(), summary=summary, summary_path=summary_path)
    if not summary["ok"]:
        _stop_handles(backend, handles)
    return RealPreparePhaseResult(
        return_code=0 if summary["ok"] else 20,
        summary=summary,
        backend=backend,
        handles=handles,
    )


def stop_real_prepare_phase(result: RealPreparePhaseResult) -> None:
    _stop_handles(result.backend, result.handles)


def run_real_task_doctor(
    *,
    task_name: str,
    config_path: str | Path | None = None,
    task_config_path: str | Path | None = None,
    console: Console | None = None,
) -> int:
    config = RunConfig.from_config(
        config_path=config_path,
        task_name=task_name,
        task_config_path=task_config_path,
        artifact_dir=os.environ.get("ARTIFACT_DIR"),
        run_id=os.environ.get("RUN_ID"),
    )
    artifact_dir = Path(
        os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_real_task_doctor/{config.run_id}/{task_name}")
    )
    summary = build_real_task_doctor_summary(task_name, config)
    summary_path = artifact_dir / "summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    _print_real_task_doctor_summary(console or Console(), summary=summary, summary_path=summary_path)
    return 0 if summary["ok"] else 20


def _build_real_prepare_summary(
    config: RunConfig,
    *,
    task_name: str,
    backend: ProcessBackend,
    started_handles: list[RuntimeHandle],
    artifact_dir: Path,
    log_dir: Path,
) -> dict[str, Any]:
    blockers: list[str] = []
    prepare = config.orchestration.real_prepare
    mode_spec, mode_blockers = _fcu_bridge_mode_selection(config)
    blockers.extend(mode_blockers)
    services = _prepare_services(config)
    blockers.extend(_validate_prepare_services(config, services))
    started_services: list[dict[str, Any]] = []

    if "mavros" in services and not prepare.dry_run:
        geoid_result = _ensure_geographiclib_geoid_data()
        if geoid_result.get("blocker"):
            blockers.append(str(geoid_result["blocker"]))

    if not blockers:
        for name, service in services.items():
            spec = _service_spec(name, service, config=config, log_dir=log_dir)
            try:
                handle = backend.start_service(spec)
            except (BackendConfigError, ServiceStartError, OSError) as exc:
                blockers.append(f"prepare_service_start_failed:{name}:{exc}")
                break
            started_handles.append(handle)
            started_services.append(_handle_summary(handle, service))

    router_probe = _probe_mavlink_router_endpoint(config)
    if router_probe.get("blocker"):
        blockers.append(str(router_probe["blocker"]))

    topic_snapshot = _wait_for_prepare_topic_snapshot(config, services, task_name=task_name)
    readiness = check_real_task_upstream_topics(task_name, config, topic_snapshot=topic_snapshot)
    if readiness["blocked"]:
        blockers.extend(f"prepare_{blocker}" for blocker in readiness["blockers"])

    blockers = list(dict.fromkeys(blockers))
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "task_name": task_name,
        "prepare_claim": "evaluated",
        "companion_claim": "not_started",
        "checked_at": _utc_now(),
        "artifact_dir": str(artifact_dir),
        "process_log_dir": str(log_dir),
        "dry_run": config.orchestration.real_prepare.dry_run,
        "fcu_bridge_mode": {
            "name": prepare.fcu_bridge_mode,
            "description": mode_spec.description if mode_spec else "",
            "required_topics": list(mode_spec.prepare_required_topics) if mode_spec else [],
        },
        "mavlink_router": {
            "serial": config.orchestration.real_prepare.mavlink_router_serial_port,
            "baud": config.orchestration.real_prepare.mavlink_router_baud,
            "local_endpoint": config.orchestration.real_prepare.mavlink_router_local_endpoint,
            "serial_provenance": _serial_provenance(config),
            "endpoint_probe": router_probe,
        },
        "geographiclib": _geographiclib_summary(),
        "started_services": started_services,
        "service_count": len(started_services),
        "readiness": readiness,
    }


def build_real_task_doctor_summary(
    task_name: str,
    config: RunConfig,
    *,
    topic_snapshot: RealTopicSnapshot | None = None,
) -> dict[str, Any]:
    upstream = check_real_task_upstream_topics(task_name, config, topic_snapshot=topic_snapshot)
    blockers = list(upstream["blockers"])
    task_specific = _check_task_specific_readiness(task_name, config, upstream)
    blockers.extend(task_specific["blockers"])
    blockers = list(dict.fromkeys(blockers))
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "task_name": task_name,
        "task_doctor_claim": "evaluated",
        "arm_claim": "not_evaluated",
        "takeoff_claim": "not_evaluated",
        "landing_claim": "not_evaluated",
        "companion_claim": "not_started",
        "checked_at": _utc_now(),
        "fcu_bridge_mode": config.orchestration.real_prepare.fcu_bridge_mode,
        "upstream": upstream,
        "task_specific": task_specific,
    }


def check_real_task_upstream_topics(
    task_name: str,
    config: RunConfig,
    *,
    topic_snapshot: RealTopicSnapshot | None = None,
) -> dict[str, Any]:
    required = _required_upstream_topics(task_name, config)
    snapshot = topic_snapshot or collect_ros_topic_snapshot(
        timeout_sec=config.orchestration.real_prepare.ros_topic_probe_timeout_sec,
        probe_topics=required,
    )
    expected_types = _expected_topic_types(config)
    forbidden_patterns = _forbidden_topic_patterns(config)
    topic_names = tuple(snapshot.topics)
    blockers: list[str] = []
    topic_results: dict[str, dict[str, Any]] = {}
    _, mode_blockers = _fcu_bridge_mode_selection(config)
    blockers.extend(mode_blockers)

    if snapshot.error:
        blockers.append(f"topic_probe_failed:{snapshot.error}")

    for topic in required:
        evidence = snapshot.topics.get(topic)
        expected_type = expected_types.get(topic, "")
        result = {
            "present": evidence is not None,
            "type": evidence.type_name if evidence else "",
            "expected_type": expected_type,
            "fresh": evidence.fresh if evidence else False,
            "frame_id": evidence.frame_id if evidence else "",
            "source_claim": evidence.source_claim if evidence else "",
            "metadata": dict(evidence.metadata) if evidence else {},
        }
        if evidence is None:
            blockers.append(f"required_topic_missing:{topic}")
        elif expected_type and evidence.type_name and evidence.type_name != expected_type:
            blockers.append(f"required_topic_type_mismatch:{topic}:{evidence.type_name}!={expected_type}")
        elif evidence.fresh is False:
            blockers.append(f"required_topic_stale:{topic}")
        topic_results[topic] = result

    forbidden_matches = sorted(
        topic for topic in topic_names for pattern in forbidden_patterns if fnmatch.fnmatch(topic, pattern)
    )
    for topic in forbidden_matches:
        blockers.append(f"forbidden_simulation_topic_present:{topic}")

    yaw_source = _check_external_nav_yaw_source(config, snapshot)
    if yaw_source["blocked"]:
        blockers.extend(yaw_source["blockers"])
    rtd5a = _check_real_slam_yaw_contract(config, snapshot)
    if rtd5a["blocked"]:
        blockers.extend(rtd5a["blockers"])

    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": list(dict.fromkeys(blockers)),
        "task_name": task_name,
        "checked_at": snapshot.collected_at or _utc_now(),
        "required_topics": topic_results,
        "yaw_source": yaw_source,
        "real_slam_yaw_contract": rtd5a,
        "forbidden_simulation_topics": forbidden_matches,
    }


def collect_ros_topic_snapshot(
    *,
    timeout_sec: float,
    probe_topics: tuple[str, ...] = (),
) -> RealTopicSnapshot:
    if not _ros2_available():
        return RealTopicSnapshot(topics={}, collected_at=_utc_now(), error="ros2_not_found")
    try:
        result = _run_ros2(["topic", "list", "-t"], timeout_sec=timeout_sec)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return RealTopicSnapshot(topics={}, collected_at=_utc_now(), error=str(exc))
    if result.returncode != 0:
        return RealTopicSnapshot(
            topics={},
            collected_at=_utc_now(),
            error=(result.stderr or result.stdout or f"rc={result.returncode}").strip(),
        )
    topics: dict[str, TopicEvidence] = {}
    for line in result.stdout.splitlines():
        topic, type_name = _parse_ros2_topic_list_line(line)
        if topic:
            topics[topic] = TopicEvidence(type_name=type_name, fresh=True)
    for topic in probe_topics:
        evidence = topics.get(topic)
        if evidence is None or not _topic_requires_sample_probe(topic):
            continue
        topics[topic] = _probe_topic_evidence(topic, evidence, timeout_sec=min(max(timeout_sec, 0.5), 2.0))
    return RealTopicSnapshot(topics=topics, collected_at=_utc_now())


def _wait_for_prepare_topic_snapshot(
    config: RunConfig,
    services: Mapping[str, RealPrepareServiceConfig],
    *,
    task_name: str,
) -> RealTopicSnapshot:
    prepare = config.orchestration.real_prepare
    required_topics = set(_required_upstream_topics(task_name, config))
    for service in services.values():
        required_topics.update(service.health_topics)
    timeout_sec = max(
        [prepare.ros_topic_probe_timeout_sec, *(service.startup_timeout_sec for service in services.values())]
    )
    deadline = time.monotonic() + timeout_sec
    probe_topics = tuple(sorted(required_topics))
    snapshot = collect_ros_topic_snapshot(
        timeout_sec=min(prepare.ros_topic_probe_timeout_sec, 2.0),
        probe_topics=(),
    )
    while time.monotonic() < deadline:
        if not snapshot.error and required_topics.issubset(snapshot.topics):
            probed_snapshot = collect_ros_topic_snapshot(
                timeout_sec=min(prepare.ros_topic_probe_timeout_sec, 2.0),
                probe_topics=probe_topics,
            )
            readiness = check_real_task_upstream_topics(task_name, config, topic_snapshot=probed_snapshot)
            if not readiness["blocked"]:
                return probed_snapshot
            soft_wait_blockers = {
                "external_nav_yaw_not_ready",
                "real_lidar_no_scan_data",
                "real_imu_no_data",
                "slam_status_not_ready",
                "external_nav_status_not_ready",
            }
            if not any(blocker in soft_wait_blockers for blocker in readiness["blockers"]):
                return probed_snapshot
            snapshot = probed_snapshot
        time.sleep(0.5)
        snapshot = collect_ros_topic_snapshot(
            timeout_sec=min(prepare.ros_topic_probe_timeout_sec, 2.0),
            probe_topics=(),
        )
    return collect_ros_topic_snapshot(
        timeout_sec=min(prepare.ros_topic_probe_timeout_sec, 2.0),
        probe_topics=probe_topics,
    )


def _prepare_services(config: RunConfig) -> dict[str, RealPrepareServiceConfig]:
    prepare = config.orchestration.real_prepare
    all_services = {
        "mavlink_router": prepare.mavlink_router,
        "navlab_mavlink_bridge": prepare.navlab_mavlink_bridge,
        "mavros": prepare.mavros,
        "lidar": prepare.lidar,
        "slam": prepare.slam,
        "rangefinder_bridge": prepare.rangefinder_bridge,
    }
    mode_spec, _ = _fcu_bridge_mode_selection(config)
    selected_names = mode_spec.prepare_service_names if mode_spec else tuple(all_services)
    return {name: all_services[name] for name in selected_names if name in all_services and all_services[name].enabled}


def _validate_prepare_services(
    config: RunConfig,
    services: Mapping[str, RealPrepareServiceConfig],
) -> tuple[str, ...]:
    blockers: list[str] = []
    serial_port = config.orchestration.real_prepare.mavlink_router_serial_port
    prepare = config.orchestration.real_prepare
    mode_spec, mode_blockers = _fcu_bridge_mode_selection(config)
    blockers.extend(mode_blockers)
    if mode_spec:
        for name in mode_spec.prepare_service_names:
            service = getattr(prepare, name, None)
            if service is None:
                blockers.append(f"prepare_fcu_bridge_service_unknown:{prepare.fcu_bridge_mode}:{name}")
            elif not service.enabled:
                blockers.append(f"prepare_fcu_bridge_service_disabled:{prepare.fcu_bridge_mode}:{name}")
    for name, service in services.items():
        if name == "companion":
            blockers.append("prepare_must_not_start_companion")
        if service.required and not service.command:
            blockers.append(f"prepare_service_command_missing:{name}")
        command_text = " ".join(service.command).lower()
        for token in SIMULATION_TOKENS:
            if token in command_text:
                blockers.append(f"prepare_service_uses_simulation_token:{name}:{token}")
        direct_serial = serial_port and any(serial_port in arg for arg in service.command)
        if name != "mavlink_router" and direct_serial and not service.direct_serial_access_allowed:
            blockers.append(f"prepare_service_direct_fcu_serial_forbidden:{name}:{serial_port}")
    if "mavlink_router" not in services:
        blockers.append("prepare_required_service_missing:mavlink_router")
    provenance = _serial_provenance(config)
    if not provenance["ok"]:
        blockers.append(str(provenance["blocker"]))
    return tuple(dict.fromkeys(blockers))


def _service_spec(
    name: str,
    service: RealPrepareServiceConfig,
    *,
    config: RunConfig,
    log_dir: Path,
) -> ServiceSpec:
    return ServiceSpec(
        name=name,
        command=_real_prepare_service_command(service.command, config=config),
        cwd=service.cwd or None,
        env=_service_env(name, service),
        required=service.required,
        log_path=log_dir / f"{name}.log",
    )


def _service_env(name: str, service: RealPrepareServiceConfig) -> dict[str, str]:
    env = dict(service.env)
    if name == "mavros" and GEOGRAPHICLIB_GEOID.exists():
        env.setdefault("GEOGRAPHICLIB_DATA", str(GEOGRAPHICLIB_DATA_DIR.resolve()))
    return env


def _ensure_geographiclib_geoid_data() -> dict[str, Any]:
    if Path("/usr/share/GeographicLib/geoids/egm96-5.pgm").exists() or GEOGRAPHICLIB_GEOID.exists():
        return {"ok": True, "source": "present"}
    tool = shutil.which("geographiclib-get-geoids")
    if not tool:
        return {"ok": False, "blocker": "prepare_geographiclib_geoid_tool_missing"}
    GEOGRAPHICLIB_DATA_DIR.mkdir(parents=True, exist_ok=True)
    try:
        result = subprocess.run(
            [tool, "-p", str(GEOGRAPHICLIB_DATA_DIR), "egm96-5"],
            check=False,
            capture_output=True,
            text=True,
            timeout=90.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "blocker": f"prepare_geographiclib_geoid_install_failed:{exc}"}
    if not GEOGRAPHICLIB_GEOID.exists():
        error = (result.stderr or result.stdout or f"rc={result.returncode}").strip()
        return {"ok": False, "blocker": f"prepare_geographiclib_geoid_missing:{error}"}
    return {"ok": True, "source": str(GEOGRAPHICLIB_GEOID)}


def _geographiclib_summary() -> dict[str, Any]:
    system_geoid = Path("/usr/share/GeographicLib/geoids/egm96-5.pgm")
    return {
        "geoid": "egm96-5",
        "system_path": str(system_geoid),
        "system_present": system_geoid.exists(),
        "local_path": str(GEOGRAPHICLIB_GEOID),
        "local_present": GEOGRAPHICLIB_GEOID.exists(),
        "env_data": str(GEOGRAPHICLIB_DATA_DIR.resolve()) if GEOGRAPHICLIB_GEOID.exists() else "",
    }


def _real_prepare_service_command(command: tuple[str, ...], *, config: RunConfig) -> tuple[str, ...]:
    if not command or command[0] != "ros2":
        return command
    source_commands = _real_prepare_ros_source_commands(config)
    if not source_commands:
        return command
    shell_command = " && ".join([*source_commands, shlex.join(command)])
    return ("bash", "-lc", shell_command)


def _real_prepare_ros_source_commands(config: RunConfig) -> list[str]:
    commands: list[str] = []
    ros_distro = _real_prepare_ros_distro(config)
    ros_setup = Path("/opt/ros") / ros_distro / "setup.bash"
    if ros_setup.exists():
        commands.append(f"source {shlex.quote(str(ros_setup))}")
    commands.extend(_local_ros_overlay_commands())
    commands.append(_cartographer_lua_overlay_command())
    local_setup = Path("install/setup.bash")
    if local_setup.exists():
        commands.append(f"source {shlex.quote(str(local_setup))}")
    return commands


def _cartographer_lua_overlay_command() -> str:
    candidates = [
        ROS_HUMBLE_CARTOGRAPHER_OVERLAY / "share/cartographer/configuration_files",
        Path("/opt/ros/humble/share/cartographer/configuration_files"),
        Path("/opt/ros/jazzy/share/cartographer/configuration_files"),
    ]
    target_dirs = [
        Path("install/navlab_cartographer_adapter/share/navlab_cartographer_adapter/config"),
    ]
    parts = []
    for source in candidates:
        for target in target_dirs:
            source_glob = shlex.quote(str(source.resolve())) + "/*.lua"
            parts.append(
                "[ ! -d "
                + shlex.quote(str(source))
                + " ] || [ ! -d "
                + shlex.quote(str(target))
                + " ] || ln -sf "
                + source_glob
                + " "
                + shlex.quote(str(target))
                + "/"
            )
    return " ; ".join(parts)


def _real_prepare_ros_distro(config: RunConfig) -> str:
    candidates = (
        config.orchestration.real_preflight.ros_distro.strip(),
        os.environ.get("ROS_DISTRO", "").strip(),
        "humble",
        "jazzy",
    )
    for distro in candidates:
        if distro and (Path("/opt/ros") / distro / "setup.bash").exists():
            return distro
    return next((distro for distro in candidates if distro), "humble")


def _handle_summary(handle: RuntimeHandle, service: RealPrepareServiceConfig) -> dict[str, Any]:
    return {
        "name": handle.service_name,
        "backend": handle.backend,
        "identifier": handle.identifier,
        "pid": handle.pid,
        "command": list(handle.command),
        "log_path": str(handle.log_path) if handle.log_path else "",
        "health_topics": list(service.health_topics),
        "shutdown_policy": service.shutdown_policy,
    }


def _probe_mavlink_router_endpoint(config: RunConfig) -> dict[str, Any]:
    prepare = config.orchestration.real_prepare
    summary: dict[str, Any] = {
        "endpoint": prepare.mavlink_router_local_endpoint,
        "heartbeat_seen": False,
        "skipped": "",
    }
    if prepare.dry_run:
        summary["skipped"] = "dry_run"
        return summary
    try:
        from pymavlink import mavutil
    except ImportError:
        summary["blocker"] = "mavlink_router_endpoint_probe_dependency_missing:pymavlink"
        return summary
    endpoint = _udp_in_endpoint(prepare.mavlink_router_local_endpoint)
    master = None
    try:
        master = mavutil.mavlink_connection(endpoint, autoreconnect=False, source_system=255)
        heartbeat = master.wait_heartbeat(timeout=5.0)
    except Exception as exc:  # pragma: no cover - hardware/network dependent.
        summary["blocker"] = f"mavlink_router_endpoint_probe_failed:{exc}"
        return summary
    finally:
        if master is not None:
            master.close()
    summary["heartbeat_seen"] = heartbeat is not None
    if heartbeat is None:
        summary["blocker"] = "mavlink_router_endpoint_no_heartbeat"
    return summary


def _serial_provenance(config: RunConfig) -> dict[str, Any]:
    prepare = config.orchestration.real_prepare
    serial = prepare.mavlink_router_serial_port.strip()
    command = prepare.mavlink_router.command
    if not serial:
        return {"ok": False, "blocker": "mavlink_router_serial_missing"}
    if any(prefix in serial.lower() for prefix in ("tcp:", "udp:", "udpin:", "udpout:")):
        return {"ok": False, "blocker": f"mavlink_router_serial_not_real_serial:{serial}"}
    command_has_serial = any(serial in arg for arg in command)
    if not command_has_serial:
        return {"ok": False, "blocker": f"mavlink_router_command_missing_serial_provenance:{serial}"}
    return {
        "ok": True,
        "serial": serial,
        "baud": prepare.mavlink_router_baud,
        "command_has_serial": command_has_serial,
    }


def _required_upstream_topics(task_name: str, config: RunConfig) -> tuple[str, ...]:
    prepare = config.orchestration.real_prepare
    mode_spec, _ = _fcu_bridge_mode_selection(config)
    topics = list(prepare.required_upstream_topics)
    if mode_spec:
        topics.extend(mode_spec.prepare_required_topics)
    topics.extend(
        (
            config.orchestration.slam_backend.slam_odom_topic,
            config.orchestration.slam_backend.slam_status_topic,
            prepare.fcu_bridge_state_topic,
            *prepare.external_nav_yaw_status_topics,
        )
    )
    if task_name == "scan-robustness":
        topics.append(config.orchestration.scan_stabilization.status_topic)
    return tuple(dict.fromkeys(topic for topic in topics if topic))


def _fcu_bridge_mode_selection(config: RunConfig) -> tuple[FcuBridgeModeSpec | None, tuple[str, ...]]:
    mode_name = config.orchestration.real_prepare.fcu_bridge_mode
    try:
        return get_fcu_bridge_mode(mode_name), ()
    except ValueError as exc:
        return None, (f"fcu_bridge_mode_unknown:{mode_name}:{exc}",)


def _expected_topic_types(config: RunConfig) -> dict[str, str]:
    return {
        "/scan": "sensor_msgs/msg/LaserScan",
        "/imu/data": "sensor_msgs/msg/Imu",
        "/imu": "sensor_msgs/msg/Imu",
        "/imu/status": "std_msgs/msg/String",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
        config.orchestration.slam_backend.slam_odom_topic: "nav_msgs/msg/Odometry",
        config.orchestration.slam_backend.slam_status_topic: "std_msgs/msg/String",
        config.orchestration.slam_backend.external_nav_status_topic: "std_msgs/msg/String",
        "/navlab/mavlink/status": "std_msgs/msg/String",
        "/mavlink_external_nav/status": "std_msgs/msg/String",
        config.orchestration.rangefinder_imu.rangefinder_range_topic: "sensor_msgs/msg/Range",
    }


def _check_real_slam_yaw_contract(config: RunConfig, snapshot: RealTopicSnapshot) -> dict[str, Any]:
    slam_status_topic = config.orchestration.slam_backend.slam_status_topic
    external_status_topic = config.orchestration.slam_backend.external_nav_status_topic
    checks = {
        "scan": _sample_check(snapshot, "/scan"),
        "imu_data": _sample_check(snapshot, "/imu/data"),
        "imu": _sample_check(snapshot, "/imu"),
        "slam_status": _status_check(snapshot, slam_status_topic),
        "external_nav_status": _status_check(snapshot, external_status_topic),
    }
    blockers: list[str] = []
    if not checks["scan"]["ok"]:
        blockers.append("real_lidar_no_scan_data")
    if not checks["imu_data"]["ok"] or not checks["imu"]["ok"]:
        blockers.append("real_imu_no_data")

    slam_metadata = checks["slam_status"]["metadata"]
    if not checks["slam_status"]["ready"]:
        blockers.append("slam_status_not_ready")
    else:
        scan_meta = slam_metadata.get("scan", {})
        imu_meta = slam_metadata.get("imu", {})
        if isinstance(scan_meta, Mapping) and not _metadata_bool(scan_meta.get("fresh")):
            blockers.append("real_lidar_no_scan_data")
        if isinstance(scan_meta, Mapping) and int(scan_meta.get("count") or 0) <= 0:
            blockers.append("real_lidar_no_scan_data")
        if isinstance(imu_meta, Mapping) and not _metadata_bool(imu_meta.get("fresh")):
            blockers.append("real_imu_no_data")
        if isinstance(imu_meta, Mapping) and int(imu_meta.get("count") or 0) <= 0:
            blockers.append("real_imu_no_data")

    external_metadata = checks["external_nav_status"]["metadata"]
    odom_meta = external_metadata.get("odom", {}) if isinstance(external_metadata, Mapping) else {}
    expected_odom_topic = config.orchestration.slam_backend.slam_odom_topic
    if not checks["external_nav_status"]["ready"]:
        blockers.append("external_nav_status_not_ready")
    if isinstance(odom_meta, Mapping):
        if odom_meta.get("input_topic") not in (None, "", expected_odom_topic):
            blockers.append(f"external_nav_input_topic_mismatch:{odom_meta.get('input_topic')}!={expected_odom_topic}")
        if "frame_ok" in odom_meta and not _metadata_bool(odom_meta.get("frame_ok")):
            blockers.append("external_nav_odom_frame_not_ok")
        if "rate_ok" in odom_meta and not _metadata_bool(odom_meta.get("rate_ok")):
            blockers.append("external_nav_odom_rate_not_ok")

    blockers = list(dict.fromkeys(blockers))
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "checks": checks,
        "expected_external_nav_input_topic": expected_odom_topic,
    }


def _sample_check(snapshot: RealTopicSnapshot, topic: str) -> dict[str, Any]:
    evidence = snapshot.topics.get(topic)
    metadata = dict(evidence.metadata) if evidence else {}
    sample_seen = _metadata_bool(metadata.get("sample_seen"))
    return {
        "ok": evidence is not None and evidence.fresh and sample_seen,
        "present": evidence is not None,
        "fresh": evidence.fresh if evidence else False,
        "type": evidence.type_name if evidence else "",
        "frame_id": evidence.frame_id if evidence else "",
        "metadata": metadata,
    }


def _status_check(snapshot: RealTopicSnapshot, topic: str) -> dict[str, Any]:
    evidence = snapshot.topics.get(topic)
    metadata = dict(evidence.metadata) if evidence else {}
    return {
        "ok": evidence is not None and evidence.fresh,
        "present": evidence is not None,
        "fresh": evidence.fresh if evidence else False,
        "ready": evidence is not None and evidence.fresh and _metadata_bool(metadata.get("ready")),
        "metadata": metadata,
    }


def _forbidden_topic_patterns(config: RunConfig) -> tuple[str, ...]:
    patterns = list(config.orchestration.real_prepare.forbidden_simulation_topics)
    try:
        from src.tasks.real_preflight import _load_runtime_selection

        runtime_selection = _load_runtime_selection(config)
        patterns.extend(runtime_selection.real_sources.forbidden_simulation_input_topics)
    except Exception:
        pass
    return tuple(dict.fromkeys(pattern for pattern in patterns if pattern))


def _topic_requires_sample_probe(topic: str) -> bool:
    return topic in {
        "/scan",
        "/imu/data",
        "/imu",
        "/imu/status",
        "/navlab/slam/status",
        "/external_nav/status",
        "/navlab/mavlink/status",
        "/mavlink_external_nav/status",
    }


def _probe_topic_evidence(topic: str, evidence: TopicEvidence, *, timeout_sec: float) -> TopicEvidence:
    command = ["topic", "echo", topic, "--once", "--spin-time", f"{timeout_sec:g}"]
    command.extend(_topic_probe_field_args(evidence.type_name))
    if _topic_uses_sensor_qos(evidence.type_name, topic):
        command.extend(["--qos-profile", "sensor_data"])
    try:
        result = _run_ros2(
            command,
            timeout_sec=max(timeout_sec + 6.0, 8.0),
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        metadata = {**dict(evidence.metadata), "sample_seen": False, "sample_error": str(exc)}
        return TopicEvidence(
            type_name=evidence.type_name,
            fresh=False,
            frame_id=evidence.frame_id,
            source_claim=evidence.source_claim,
            metadata=metadata,
        )
    stdout = result.stdout or ""
    metadata = {**dict(evidence.metadata), "sample_seen": result.returncode == 0 and bool(stdout.strip())}
    if result.returncode != 0:
        metadata["sample_error"] = (result.stderr or stdout or f"rc={result.returncode}").strip()
    frame_id = _yamlish_string_field(stdout, "frame_id") or evidence.frame_id
    payload = _json_payload_from_ros2_echo(stdout)
    if payload:
        metadata.update(payload)
    if evidence.type_name == "sensor_msgs/msg/LaserScan":
        metadata.setdefault("range_count", _count_yaml_sequence_items(stdout, "ranges"))
    return TopicEvidence(
        type_name=evidence.type_name,
        fresh=bool(metadata["sample_seen"]),
        frame_id=frame_id,
        source_claim=evidence.source_claim,
        metadata=metadata,
    )


def _topic_uses_sensor_qos(type_name: str, topic: str) -> bool:
    return type_name in {"sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Imu"} or topic in {
        "/scan",
        "/imu/data",
        "/imu",
    }


def _topic_probe_field_args(type_name: str) -> list[str]:
    if type_name in {"sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Imu"}:
        return ["--field", "header"]
    if type_name == "std_msgs/msg/String":
        return ["--field", "data"]
    return []


def _json_payload_from_ros2_echo(text: str) -> dict[str, Any]:
    string_payload = _std_msgs_string_payload(text)
    candidates = [string_payload, _strip_ros2_echo_document_separator(text)]
    for candidate in candidates:
        if not candidate:
            continue
        try:
            payload = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict):
            return payload
    return {}


def _strip_ros2_echo_document_separator(text: str) -> str:
    lines = []
    for line in text.splitlines():
        if line.strip() == "---":
            break
        lines.append(line)
    return "\n".join(lines).strip()


def _yamlish_string_field(text: str, field: str) -> str:
    prefix = f"{field}:"
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(prefix):
            return stripped[len(prefix) :].strip().strip("\"'")
    return ""


def _count_yaml_sequence_items(text: str, key: str) -> int:
    lines = text.splitlines()
    for index, line in enumerate(lines):
        if line.strip() != f"{key}:":
            continue
        count = 0
        for item in lines[index + 1 :]:
            stripped = item.strip()
            if not stripped:
                continue
            if stripped.startswith("- "):
                count += 1
                continue
            if not item.startswith((" ", "\t")):
                break
        return count
    return 0


def _ros2_available() -> bool:
    return shutil.which("ros2") is not None or any(
        (Path("/opt/ros") / distro / "setup.bash").exists()
        for distro in (os.environ.get("ROS_DISTRO", ""), "humble", "jazzy")
        if distro
    )


def _local_ros_overlay_commands() -> list[str]:
    prefix = ROS_HUMBLE_CARTOGRAPHER_OVERLAY
    sys_prefix = prefix.parents[2] / "usr" if len(prefix.parents) >= 3 else Path()
    commands: list[str] = []
    if (prefix / "share/ament_index").exists():
        resolved = shlex.quote(str(prefix.resolve()))
        commands.extend(
            [
                f'export AMENT_PREFIX_PATH="{resolved}:$AMENT_PREFIX_PATH"',
                f'export CMAKE_PREFIX_PATH="{resolved}:$CMAKE_PREFIX_PATH"',
                f'export PATH="{resolved}/bin:$PATH"',
                f'export PYTHONPATH="{resolved}/local/lib/python3.10/dist-packages:{resolved}/lib/python3.10/site-packages:$PYTHONPATH"',
                f'export LD_LIBRARY_PATH="{resolved}/lib:$LD_LIBRARY_PATH"',
            ]
        )
    if sys_prefix.exists():
        resolved_sys = shlex.quote(str(sys_prefix.resolve()))
        commands.append(
            f'export LD_LIBRARY_PATH="{resolved_sys}/lib:{resolved_sys}/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"'
        )
    return commands


def _run_ros2(args: list[str], *, timeout_sec: float) -> subprocess.CompletedProcess[str]:
    command = ["ros2", *args]
    source_commands = []
    for distro in (os.environ.get("ROS_DISTRO", ""), "humble", "jazzy"):
        setup = Path("/opt/ros") / distro / "setup.bash" if distro else None
        if setup and setup.exists():
            source_commands.append(f"source {shlex.quote(str(setup))}")
            break
    source_commands.extend(_local_ros_overlay_commands())
    local_setup = Path("install/setup.bash")
    if local_setup.exists():
        source_commands.append(f"source {shlex.quote(str(local_setup))}")
    if not source_commands and shutil.which("ros2"):
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    shell_command = " && ".join([*source_commands, shlex.join(command)])
    return subprocess.run(
        ["bash", "-lc", shell_command],
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_sec,
    )


def _check_external_nav_yaw_source(config: RunConfig, snapshot: RealTopicSnapshot) -> dict[str, Any]:
    prepare = config.orchestration.real_prepare
    topics = tuple(dict.fromkeys(topic for topic in prepare.external_nav_yaw_status_topics if topic))
    ready_fields = tuple(dict.fromkeys(field for field in prepare.external_nav_yaw_ready_fields if field))
    observed: list[dict[str, Any]] = []
    if not prepare.external_nav_yaw_required:
        return {
            "ok": True,
            "blocked": False,
            "blockers": [],
            "required": False,
            "accepted_source": "not_required",
            "observed": observed,
        }

    for topic in topics:
        evidence = snapshot.topics.get(topic)
        metadata = dict(evidence.metadata) if evidence else {}
        if evidence and not metadata:
            metadata = _probe_external_nav_yaw_metadata(topic, ready_fields, timeout_sec=3.0)
        ready_field = next((field for field in ready_fields if _metadata_bool(metadata.get(field))), "")
        row = {
            "topic": topic,
            "present": evidence is not None,
            "fresh": evidence.fresh if evidence else False,
            "ready_field": ready_field,
            "external_nav_yaw_ready": bool(ready_field),
            "metadata": metadata,
        }
        observed.append(row)
        if evidence is not None and evidence.fresh and ready_field:
            return {
                "ok": True,
                "blocked": False,
                "blockers": [],
                "required": True,
                "accepted_source": "external_nav_yaw_ready",
                "accepted_topic": topic,
                "accepted_ready_field": ready_field,
                "observed": observed,
            }

    return {
        "ok": False,
        "blocked": True,
        "blockers": ["external_nav_yaw_not_ready"],
        "required": True,
        "accepted_source": "",
        "observed": observed,
        "note": "indoor real tasks require ExternalNav/SLAM yaw readiness; compass/manual override are not accepted",
    }


def _probe_external_nav_yaw_metadata(
    topic: str,
    ready_fields: tuple[str, ...],
    *,
    timeout_sec: float,
) -> dict[str, Any]:
    if not _ros2_available():
        return {}
    try:
        result = _run_ros2(
            ["topic", "echo", topic, "--field", "data", "--once", "--spin-time", str(timeout_sec)],
            timeout_sec=timeout_sec + 1.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return {}
    if result.returncode != 0:
        try:
            result = _run_ros2(
                ["topic", "echo", topic, "--once", "--spin-time", str(timeout_sec)],
                timeout_sec=timeout_sec + 1.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            return {}
        if result.returncode != 0:
            return {}
    metadata: dict[str, Any] = {}
    string_payload = _std_msgs_string_payload(result.stdout) or result.stdout.strip()
    if string_payload:
        try:
            payload = json.loads(string_payload)
        except json.JSONDecodeError:
            payload = {}
        if isinstance(payload, dict):
            for ready_field in ready_fields:
                value = payload.get(ready_field)
                if value is not None:
                    metadata[ready_field] = value
    for ready_field in ready_fields:
        if _yamlish_bool_field(result.stdout, ready_field):
            metadata[ready_field] = True
    return metadata


def _std_msgs_string_payload(text: str) -> str:
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("data:"):
            continue
        raw = stripped[len("data:") :].strip()
        if not raw:
            return ""
        try:
            value = ast.literal_eval(raw)
        except (ValueError, SyntaxError):
            return raw.strip("\"'")
        return value if isinstance(value, str) else str(value)
    return ""


def _yamlish_bool_field(text: str, field: str) -> bool:
    prefix = f"{field}:"
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(prefix):
            return stripped[len(prefix) :].strip().lower() in {"1", "true", "yes", "on"}
    return False


def _metadata_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, int | float):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on", "ready"}
    return False


def _check_task_specific_readiness(task_name: str, config: RunConfig, upstream: Mapping[str, Any]) -> dict[str, Any]:
    blockers: list[str] = []
    landing_policy = config.orchestration.landing.policy_for_task(task_name)
    result: dict[str, Any] = {
        "landing_policy": landing_policy,
        "takeoff_alt_m": config.orchestration.fcu_controller.takeoff_alt_m,
    }
    if config.orchestration.fcu_controller.takeoff_alt_m <= 0:
        blockers.append("task_takeoff_altitude_invalid")

    status_topic = config.orchestration.fcu_controller.status_topic
    status = upstream.get("required_topics", {}).get(status_topic, {})
    metadata = status.get("metadata", {}) if isinstance(status, dict) else {}
    if isinstance(metadata, Mapping) and metadata.get("armed") is True:
        blockers.append("task_initial_fcu_armed")

    if task_name == "hover":
        if landing_policy != "land_in_place":
            blockers.append(f"hover_landing_policy_invalid:{landing_policy}")
    elif task_name == "exploration":
        result["home_source"] = config.orchestration.landing.home_source
        result["motion_distance_m"] = config.orchestration.motion_gate.motion_distance_m
        if landing_policy != "return_home_then_land":
            blockers.append(f"exploration_landing_policy_invalid:{landing_policy}")
        if not config.orchestration.landing.home_source:
            blockers.append("exploration_home_source_missing")
        if (
            config.orchestration.motion_gate.motion_distance_m <= 0
            or config.orchestration.motion_gate.motion_speed_mps <= 0
        ):
            blockers.append("exploration_bounded_movement_invalid")
    elif task_name == "scan-robustness":
        result["scan_stabilization_enabled"] = config.orchestration.scan_stabilization.enabled
        result["scan_stabilization_status_topic"] = config.orchestration.scan_stabilization.status_topic
        if landing_policy != "land_in_place":
            blockers.append(f"scan_robustness_landing_policy_invalid:{landing_policy}")
        if not config.orchestration.scan_stabilization.enabled:
            blockers.append("scan_robustness_stabilization_disabled")
    else:
        blockers.append(f"unsupported_real_task:{task_name}")

    return {"ok": not blockers, "blocked": bool(blockers), "blockers": blockers, **result}


def _parse_ros2_topic_list_line(line: str) -> tuple[str, str]:
    stripped = line.strip()
    if not stripped:
        return "", ""
    if "[" not in stripped or not stripped.endswith("]"):
        return stripped, ""
    topic, raw_type = stripped.rsplit("[", 1)
    return topic.strip(), raw_type[:-1].strip()


def _udp_in_endpoint(endpoint: str) -> str:
    if endpoint.startswith(("udp:", "udpin:", "udpout:", "tcp:")):
        return endpoint
    host, _, port = endpoint.partition(":")
    return f"udpin:{host or '0.0.0.0'}:{port or '14550'}"


def _stop_handles(backend: ProcessBackend, handles: list[RuntimeHandle]) -> None:
    for handle in reversed(handles):
        try:
            backend.stop(handle, timeout_sec=3.0)
        except Exception:
            pass


def _print_real_prepare_summary(console: Console, *, summary: Mapping[str, Any], summary_path: Path) -> None:
    router = summary.get("mavlink_router", {})
    table = Table(title="Real Prepare", show_header=True, header_style="bold cyan")
    table.add_column("Key")
    table.add_column("Value")
    table.add_row("ok", str(summary.get("ok")))
    table.add_row("task", str(summary.get("task_name")))
    table.add_row("dry_run", str(summary.get("dry_run")))
    bridge = summary.get("fcu_bridge_mode", {})
    bridge_name = bridge.get("name") if isinstance(bridge, Mapping) else bridge
    table.add_row("fcu_bridge", str(bridge_name))
    table.add_row("router_serial", str(router.get("serial")))
    table.add_row("router_endpoint", str(router.get("local_endpoint")))
    table.add_row("services", str(summary.get("service_count")))
    table.add_row("summary", str(summary_path))
    console.print(Panel(table, title="NavLab Real Prepare"))
    for blocker in summary.get("blockers", []):
        console.print(f"[red]blocker:[/red] {blocker}")


def _print_real_task_doctor_summary(console: Console, *, summary: Mapping[str, Any], summary_path: Path) -> None:
    table = Table(title="Real Task Doctor", show_header=True, header_style="bold cyan")
    table.add_column("Key")
    table.add_column("Value")
    table.add_row("ok", str(summary.get("ok")))
    table.add_row("task", str(summary.get("task_name")))
    table.add_row("summary", str(summary_path))
    table.add_row("arm", str(summary.get("arm_claim")))
    table.add_row("takeoff", str(summary.get("takeoff_claim")))
    console.print(Panel(table, title="NavLab Real Task Doctor"))
    for blocker in summary.get("blockers", []):
        console.print(f"[red]blocker:[/red] {blocker}")


def _utc_now() -> str:
    return datetime.now(UTC).isoformat().replace("+00:00", "Z")
