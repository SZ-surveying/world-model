from __future__ import annotations

import json
import os
import textwrap
import time
from collections.abc import Mapping
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from navlab.real.common.fcu_status import (
    FCU_STATUS_FIELDS,
    FCU_STATUS_PARAMETER_NAMES,
    FcuStatusField,
    arducopter_mode_name,
    source_set_uses_gps,
)
from src.configs.run_config import RunConfig
from src.workflows.real.prepare import (
    RealTopicSnapshot,
    check_real_task_upstream_topics,
    collect_ros_topic_snapshot,
    int_or_none,
    metadata_bool,
    metadata_bool_or_none,
    metadata_field_value,
    metadata_string_value,
    required_upstream_topics,
)

COMMON_DOCTOR_FIELDS = {
    "fcu_status_topic": "/navlab/mavlink/status",
    "bridge_state_topic": "/navlab/mavlink/status",
    "external_nav_status_topic": "/external_nav/status",
    "mavlink_external_nav_status_topic": "/mavlink_external_nav/status",
}

COMMON_DOCTOR_METADATA_FIELDS = FCU_STATUS_FIELDS


def run_real_common_doctor(
    *,
    config_path: str | Path | None = None,
    task_config_path: str | Path | None = None,
    task_name: str = "doctor",
    console: Console | None = None,
) -> int:
    console = console or Console()
    config = RunConfig.from_config(
        config_path=config_path,
        task_name=task_name,
        task_config_path=task_config_path,
        artifact_dir=os.environ.get("ARTIFACT_DIR"),
        run_id=os.environ.get("RUN_ID"),
    )
    artifact_dir = Path(
        os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_real_common_doctor/{config.run_id}/{task_name}")
    )
    summary = build_real_common_doctor_summary(
        config,
        task_name=task_name,
        topic_snapshot=wait_for_common_doctor_topic_snapshot(config, task_name=task_name),
    )
    summary_path = artifact_dir / "summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    rc = 0 if summary["ok"] else 20
    color = "green" if summary["ok"] else "red"
    console.print("\n\nChecking real common doctor contract")
    console.print(f"[{color}]Real common doctor rc={rc}[/{color}]")
    print_real_common_doctor_summary(console, summary=summary, summary_path=summary_path)
    return rc


def build_real_common_doctor_summary(
    config: RunConfig,
    *,
    task_name: str = "doctor",
    topic_snapshot: RealTopicSnapshot | None = None,
) -> dict[str, Any]:
    upstream = check_real_task_upstream_topics(task_name, config, topic_snapshot=topic_snapshot)
    blockers = list(upstream["blockers"])
    common = _check_common_readiness(config, upstream)
    blockers.extend(common["blockers"])
    blockers = list(dict.fromkeys(blockers))
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "task_name": task_name,
        "common_doctor_claim": "evaluated",
        "arm_claim": "not_evaluated",
        "takeoff_claim": "not_evaluated",
        "landing_claim": "not_evaluated",
        "companion_claim": "not_started",
        "checked_at": _utc_now(),
        "fcu_bridge_mode": config.orchestration.real_prepare.fcu_bridge_mode,
        "upstream": upstream,
        "common_state": common,
    }


def wait_for_common_doctor_topic_snapshot(config: RunConfig, *, task_name: str) -> RealTopicSnapshot:
    prepare = config.orchestration.real_prepare
    probe_topics = tuple(sorted(required_upstream_topics(task_name, config)))
    graph_probe_timeout_sec = max(prepare.ros_topic_probe_timeout_sec, 0.5)
    deadline = time.monotonic() + graph_probe_timeout_sec
    last_snapshot: RealTopicSnapshot | None = None
    while time.monotonic() < deadline:
        snapshot = collect_ros_topic_snapshot(
            timeout_sec=graph_probe_timeout_sec,
            probe_topics=probe_topics,
        )
        last_snapshot = snapshot
        status = snapshot.topics.get(prepare.fcu_bridge_state_topic)
        metadata = dict(status.metadata) if status else {}
        if _mavlink_status_has_common_metadata(metadata):
            return snapshot
        time.sleep(0.3)
    if last_snapshot is not None:
        return last_snapshot
    return collect_ros_topic_snapshot(
        timeout_sec=graph_probe_timeout_sec,
        probe_topics=probe_topics,
    )


def _mavlink_status_has_common_metadata(metadata: Mapping[str, Any]) -> bool:
    if not metadata:
        return False
    has_mode = bool(
        metadata_string_value(metadata, FcuStatusField("mode", ("mode", "mode_name", "flight_mode")))
        or metadata_field_value(metadata, FcuStatusField("mode_number", ("mode_number",))) is not None
    )
    parameters = metadata.get("parameters")
    if isinstance(parameters, Mapping):
        has_parameter = any(name in parameters for name in FCU_STATUS_PARAMETER_NAMES)
    else:
        has_parameter = any(name in metadata for name in FCU_STATUS_PARAMETER_NAMES)
    return has_mode and has_parameter


def _check_common_readiness(config: RunConfig, upstream: Mapping[str, Any]) -> dict[str, Any]:
    status_topic = config.orchestration.fcu_controller.status_topic
    status = upstream.get("required_topics", {}).get(status_topic, {})
    metadata = dict(status.get("metadata", {})) if isinstance(status, dict) else {}
    if not metadata:
        bridge_status_topic = config.orchestration.real_prepare.fcu_bridge_state_topic
        bridge_status = upstream.get("required_topics", {}).get(bridge_status_topic, {})
        metadata = dict(bridge_status.get("metadata", {})) if isinstance(bridge_status, dict) else {}
    return _common_fcu_external_nav_state(config, upstream, metadata)


def _common_fcu_external_nav_state(
    config: RunConfig,
    upstream: Mapping[str, Any],
    fcu_status_metadata: Mapping[str, Any],
) -> dict[str, Any]:
    prepare = config.orchestration.real_prepare
    topics = upstream.get("required_topics", {})
    bridge_status = topics.get(prepare.fcu_bridge_state_topic, {})
    mavlink_external_nav_status = topics.get(COMMON_DOCTOR_FIELDS["mavlink_external_nav_status_topic"], {})
    external_nav_status = topics.get(config.orchestration.slam_backend.external_nav_status_topic, {})

    bridge_metadata = dict(bridge_status.get("metadata", {})) if isinstance(bridge_status, dict) else {}
    mavlink_external_nav_metadata = (
        dict(mavlink_external_nav_status.get("metadata", {})) if isinstance(mavlink_external_nav_status, dict) else {}
    )
    external_nav_metadata = dict(external_nav_status.get("metadata", {})) if isinstance(external_nav_status, dict) else {}
    fcu_and_bridge_metadata = {**bridge_metadata, **dict(fcu_status_metadata)}

    extracted: dict[str, Any] = {}
    for field in COMMON_DOCTOR_METADATA_FIELDS:
        if field.name in {"local_position_valid"}:
            value = metadata_bool_or_none(fcu_and_bridge_metadata, field)
        elif field.name in {
            "gps_type",
            "gps1_type",
            "viso_type",
            "active_source_set",
            "configured_external_nav_source_set",
            "observed_ekf_source_set",
            "observed_ekf_source_set_text",
            "ek3_src1_posxy",
            "ek3_src1_velxy",
            "ek3_src1_velz",
            "ek3_src1_yaw",
            "ek3_src1_posz",
            "ek3_src2_posxy",
            "ek3_src2_velxy",
            "ek3_src2_velz",
            "ek3_src2_yaw",
            "ek3_src2_posz",
        }:
            value = metadata_string_value(fcu_status_metadata, field)
        else:
            value = metadata_field_value(fcu_status_metadata, field)
        extracted[field.name] = value

    external_nav_ros_ready = bool(
        metadata_bool(external_nav_metadata.get("ready"))
        or metadata_bool(external_nav_metadata.get("external_nav_yaw_ready"))
        or metadata_bool(mavlink_external_nav_metadata.get("ready"))
        or metadata_bool(mavlink_external_nav_metadata.get("external_nav_ready"))
    )
    legacy_source_set = str(extracted.get("active_source_set") or "")
    configured_external_nav_source_set = str(
        extracted.get("configured_external_nav_source_set") or legacy_source_set or "unknown"
    )
    observed_ekf_source_set = str(extracted.get("observed_ekf_source_set") or "not_observed")
    observed_ekf_source_set_text = str(extracted.get("observed_ekf_source_set_text") or "")
    local_position_valid = extracted.get("local_position_valid")
    ekf_source_set_switch = fcu_and_bridge_metadata.get("ekf_source_set_switch", {})
    if not isinstance(ekf_source_set_switch, Mapping):
        ekf_source_set_switch = {}
    mode = metadata_string_value(fcu_status_metadata, FcuStatusField("mode", ("mode", "mode_name", "flight_mode")))
    mode_number = int_or_none(metadata_field_value(fcu_status_metadata, FcuStatusField("mode_number", ("mode_number",))))
    if not mode and mode_number is not None:
        mode = arducopter_mode_name(mode_number)
    armed = metadata_bool_or_none(fcu_status_metadata, FcuStatusField("armed", ("armed",)))

    blockers: list[str] = []
    if configured_external_nav_source_set == "SRC2" and not external_nav_ros_ready:
        blockers.append("external_nav_or_gps_source_not_ready")
    gps_source_sets = []
    if source_set_uses_gps(fcu_status_metadata, prefix="EK3_SRC1"):
        gps_source_sets.append("SRC1")
    if source_set_uses_gps(fcu_status_metadata, prefix="EK3_SRC2"):
        gps_source_sets.append("SRC2")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": list(dict.fromkeys(blockers)),
        "gps_type": extracted.get("gps_type") or "unknown",
        "gps1_type": extracted.get("gps1_type") or "unknown",
        "viso_type": extracted.get("viso_type") or "unknown",
        "ek3_src1": {
            "posxy": extracted.get("ek3_src1_posxy") or "unknown",
            "velxy": extracted.get("ek3_src1_velxy") or "unknown",
            "velz": extracted.get("ek3_src1_velz") or "unknown",
            "yaw": extracted.get("ek3_src1_yaw") or "unknown",
            "posz": extracted.get("ek3_src1_posz") or "unknown",
        },
        "ek3_src2": {
            "posxy": extracted.get("ek3_src2_posxy") or "unknown",
            "velxy": extracted.get("ek3_src2_velxy") or "unknown",
            "velz": extracted.get("ek3_src2_velz") or "unknown",
            "yaw": extracted.get("ek3_src2_yaw") or "unknown",
            "posz": extracted.get("ek3_src2_posz") or "unknown",
        },
        "gps_source_sets": gps_source_sets,
        "active_source_set": observed_ekf_source_set,
        "configured_external_nav_source_set": configured_external_nav_source_set,
        "observed_ekf_source_set": observed_ekf_source_set,
        "observed_ekf_source_set_text": observed_ekf_source_set_text,
        "ekf_source_set_switch": dict(ekf_source_set_switch),
        "external_nav_ros_ready": external_nav_ros_ready,
        "local_position_valid": local_position_valid if local_position_valid is not None else "unknown",
        "mode": mode or "unknown",
        "armed": armed if armed is not None else "unknown",
        "bridge_metadata": bridge_metadata,
        "mavlink_external_nav_metadata": mavlink_external_nav_metadata,
        "external_nav_metadata": external_nav_metadata,
    }


def print_real_common_doctor_summary(console: Console, *, summary: Mapping[str, Any], summary_path: Path) -> None:
    common = summary.get("common_state", {})
    grid = _summary_grid()
    grid.add_row("Status", "OK" if summary.get("ok") else "BLOCKED")
    grid.add_row("Task", str(summary.get("task_name")))
    grid.add_row("Mode", str(common.get("mode", "unknown")))
    grid.add_row("GPS", f'{common.get("gps_type", "unknown")} / {common.get("gps1_type", "unknown")}')
    grid.add_row("VISO", str(common.get("viso_type", "unknown")))
    src1 = common.get("ek3_src1", {})
    src2 = common.get("ek3_src2", {})
    grid.add_row(
        "EK3 SRC1",
        f'posxy={src1.get("posxy", "unknown")}, velxy={src1.get("velxy", "unknown")}, '
        f'velz={src1.get("velz", "unknown")}, yaw={src1.get("yaw", "unknown")}, '
        f'posz={src1.get("posz", "unknown")}',
    )
    grid.add_row(
        "EK3 SRC2",
        f'posxy={src2.get("posxy", "unknown")}, velxy={src2.get("velxy", "unknown")}, '
        f'velz={src2.get("velz", "unknown")}, yaw={src2.get("yaw", "unknown")}, '
        f'posz={src2.get("posz", "unknown")}',
    )
    grid.add_row("ExternalNav config", str(common.get("configured_external_nav_source_set", "unknown")))
    grid.add_row("EKF active src", str(common.get("observed_ekf_source_set", "not_observed")))
    gps_source_sets = common.get("gps_source_sets", [])
    if isinstance(gps_source_sets, list) and gps_source_sets:
        grid.add_row("GPS EKF src", ", ".join(str(item) for item in gps_source_sets))
    switch = common.get("ekf_source_set_switch", {})
    if isinstance(switch, Mapping) and switch.get("enabled") is True:
        target = switch.get("target_source_set", "unknown")
        sent = switch.get("sent", "unknown")
        ack = switch.get("ack_result", "pending" if sent else "not_sent")
        grid.add_row("EKF switch", f"target={target}, sent={sent}, ack={ack}")
    grid.add_row("ExternalNav ROS", str(common.get("external_nav_ros_ready", "unknown")))
    grid.add_row("Local position", str(common.get("local_position_valid", "unknown")))
    grid.add_row("Summary", "")
    grid.add_row("", _wrapped_summary_path(console, summary_path))
    border_style = "green" if summary.get("ok") else "red"
    console.print(Panel(grid, title="NavLab Real Common Doctor", border_style=border_style))
    _print_blockers_panel(console, summary.get("blockers", []))


def _print_blockers_panel(console: Console, blockers: object) -> None:
    blocker_items = [str(item) for item in blockers] if isinstance(blockers, list | tuple) else []
    if not blocker_items:
        return
    grid = _summary_grid()
    for blocker in blocker_items[:8]:
        grid.add_row("", f"- {blocker}")
    if len(blocker_items) > 8:
        grid.add_row("", f"- ... {len(blocker_items) - 8} more")
    console.print(Panel(grid, title="Blockers", border_style="red"))


def _summary_grid() -> Table:
    grid = Table.grid(padding=(0, 2))
    grid.add_column(style="bold")
    grid.add_column()
    return grid


def _wrapped_summary_path(console: Console, summary_path: Path) -> str:
    width = max(32, min(100, console.width - 28))
    return "\n".join(textwrap.wrap(str(summary_path), width=width, break_long_words=True, break_on_hyphens=False))


def _utc_now() -> str:
    return datetime.now(UTC).isoformat().replace("+00:00", "Z")
