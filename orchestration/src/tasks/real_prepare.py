from __future__ import annotations

import fnmatch
import json
import os
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

REAL_PREPARE_AND_TASK_DOCTOR_TODO = "docs/scenarios/indoor/todos/real_prepare_and_task_doctor_todo.md"
SIMULATION_TOKENS = ("gazebo", "sitl", "gazebo-sensor", "/scan_ideal", "/sim/x2/status")


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
    services = _prepare_services(config)
    blockers.extend(_validate_prepare_services(config, services))
    started_services: list[dict[str, Any]] = []

    if not blockers:
        for name, service in services.items():
            spec = _service_spec(name, service, log_dir=log_dir)
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

    topic_snapshot = _wait_for_prepare_topic_snapshot(config, services)
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
        "todo": REAL_PREPARE_AND_TASK_DOCTOR_TODO,
        "checked_at": _utc_now(),
        "artifact_dir": str(artifact_dir),
        "process_log_dir": str(log_dir),
        "dry_run": config.orchestration.real_prepare.dry_run,
        "mavlink_router": {
            "serial": config.orchestration.real_prepare.mavlink_router_serial_port,
            "baud": config.orchestration.real_prepare.mavlink_router_baud,
            "local_endpoint": config.orchestration.real_prepare.mavlink_router_local_endpoint,
            "serial_provenance": _serial_provenance(config),
            "endpoint_probe": router_probe,
        },
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
        "todo": REAL_PREPARE_AND_TASK_DOCTOR_TODO,
        "checked_at": _utc_now(),
        "upstream": upstream,
        "task_specific": task_specific,
    }


def check_real_task_upstream_topics(
    task_name: str,
    config: RunConfig,
    *,
    topic_snapshot: RealTopicSnapshot | None = None,
) -> dict[str, Any]:
    snapshot = topic_snapshot or collect_ros_topic_snapshot(
        timeout_sec=config.orchestration.real_prepare.ros_topic_probe_timeout_sec
    )
    required = _required_upstream_topics(task_name, config)
    expected_types = _expected_topic_types(config)
    forbidden_patterns = _forbidden_topic_patterns(config)
    topic_names = tuple(snapshot.topics)
    blockers: list[str] = []
    topic_results: dict[str, dict[str, Any]] = {}

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

    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": list(dict.fromkeys(blockers)),
        "task_name": task_name,
        "checked_at": snapshot.collected_at or _utc_now(),
        "required_topics": topic_results,
        "yaw_source": yaw_source,
        "forbidden_simulation_topics": forbidden_matches,
    }


def collect_ros_topic_snapshot(*, timeout_sec: float) -> RealTopicSnapshot:
    if not shutil.which("ros2"):
        return RealTopicSnapshot(topics={}, collected_at=_utc_now(), error="ros2_not_found")
    try:
        result = subprocess.run(
            ["ros2", "topic", "list", "-t"],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
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
    return RealTopicSnapshot(topics=topics, collected_at=_utc_now())


def _wait_for_prepare_topic_snapshot(
    config: RunConfig,
    services: Mapping[str, RealPrepareServiceConfig],
) -> RealTopicSnapshot:
    prepare = config.orchestration.real_prepare
    required_topics = set(prepare.required_upstream_topics)
    for service in services.values():
        required_topics.update(service.health_topics)
    timeout_sec = max(
        [prepare.ros_topic_probe_timeout_sec, *(service.startup_timeout_sec for service in services.values())]
    )
    deadline = time.monotonic() + timeout_sec
    snapshot = collect_ros_topic_snapshot(timeout_sec=min(prepare.ros_topic_probe_timeout_sec, 2.0))
    while time.monotonic() < deadline:
        if not snapshot.error and required_topics.issubset(snapshot.topics):
            return snapshot
        time.sleep(0.5)
        snapshot = collect_ros_topic_snapshot(timeout_sec=min(prepare.ros_topic_probe_timeout_sec, 2.0))
    return snapshot


def _prepare_services(config: RunConfig) -> dict[str, RealPrepareServiceConfig]:
    prepare = config.orchestration.real_prepare
    services = {
        "mavlink_router": prepare.mavlink_router,
        "mavros": prepare.mavros,
        "lidar": prepare.lidar,
        "slam": prepare.slam,
        "rangefinder_bridge": prepare.rangefinder_bridge,
    }
    return {name: service for name, service in services.items() if service.enabled}


def _validate_prepare_services(
    config: RunConfig,
    services: Mapping[str, RealPrepareServiceConfig],
) -> tuple[str, ...]:
    blockers: list[str] = []
    serial_port = config.orchestration.real_prepare.mavlink_router_serial_port
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


def _service_spec(name: str, service: RealPrepareServiceConfig, *, log_dir: Path) -> ServiceSpec:
    return ServiceSpec(
        name=name,
        command=service.command,
        cwd=service.cwd or None,
        env=service.env,
        required=service.required,
        log_path=log_dir / f"{name}.log",
    )


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
    topics = list(prepare.required_upstream_topics)
    topics.extend(
        (
            config.orchestration.fcu_controller.status_topic,
            config.orchestration.fcu_controller.pose_topic,
            config.orchestration.fcu_controller.twist_topic,
            config.orchestration.slam_backend.slam_odom_topic,
            config.orchestration.slam_backend.slam_status_topic,
            prepare.fcu_bridge_state_topic,
            *prepare.external_nav_yaw_status_topics,
        )
    )
    if task_name == "scan-robustness":
        topics.append(config.orchestration.scan_stabilization.status_topic)
    return tuple(dict.fromkeys(topic for topic in topics if topic))


def _expected_topic_types(config: RunConfig) -> dict[str, str]:
    return {
        "/scan": "sensor_msgs/msg/LaserScan",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
        config.orchestration.slam_backend.slam_odom_topic: "nav_msgs/msg/Odometry",
        config.orchestration.rangefinder_imu.rangefinder_range_topic: "sensor_msgs/msg/Range",
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
            metadata = _probe_external_nav_yaw_metadata(topic, ready_fields, timeout_sec=1.5)
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


def _probe_external_nav_yaw_metadata(topic: str, ready_fields: tuple[str, ...], *, timeout_sec: float) -> dict[str, Any]:
    if not shutil.which("ros2"):
        return {}
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "--once", topic],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except (OSError, subprocess.TimeoutExpired):
        return {}
    if result.returncode != 0:
        return {}
    metadata: dict[str, Any] = {}
    for field in ready_fields:
        if _yamlish_bool_field(result.stdout, field):
            metadata[field] = True
    return metadata


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
