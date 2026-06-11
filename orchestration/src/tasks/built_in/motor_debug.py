from __future__ import annotations

import json
import os
import subprocess
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from navlab.real.companion.nodes.motor_debug import (
    REQUIRED_GUIDED_MODE_NAME,
    TASK_RESULT_SCHEMA_VERSION,
)
from src.configs.run_config import RunConfig
from src.configs.task_config import (
    load_task_config_data,
    optional_task_table,
    resolve_task_config_path,
    resolve_task_float,
    resolve_task_int,
)
from src.logger_utils import close_process_loggers, start_process_logger
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


@dataclass(frozen=True, slots=True)
class MotorDebugTaskConfig:
    task_name: str
    path: Path | None
    path_source: str
    motor_percent: float
    motor_percent_source: str
    motor_sec: float
    motor_sec_source: str
    motor_count: int
    motor_count_source: str

    @classmethod
    def from_file(
        cls,
        *,
        task_config_path: str | Path | None = None,
        cli_motor_percent: float | None = None,
        cli_motor_sec: float | None = None,
        cli_motor_count: int | None = None,
    ) -> MotorDebugTaskConfig:
        task_name = "motor-debug"
        path, path_source = resolve_task_config_path(task_name, task_config_path)
        data = load_task_config_data(task_name, task_config_path=task_config_path)
        task = optional_task_table(data, path)
        motor_percent, motor_percent_source = resolve_task_float(
            task,
            "motor_percent",
            cli_motor_percent,
            5.0,
        )
        motor_sec, motor_sec_source = resolve_task_float(
            task,
            "motor_sec",
            cli_motor_sec,
            5.0,
        )
        motor_count, motor_count_source = resolve_task_int(
            task,
            "motor_count",
            cli_motor_count,
            4,
        )
        return cls(
            task_name=task_name,
            path=path if path and path.is_file() else None,
            path_source=path_source,
            motor_percent=motor_percent,
            motor_percent_source=motor_percent_source,
            motor_sec=motor_sec,
            motor_sec_source=motor_sec_source,
            motor_count=motor_count,
            motor_count_source=motor_count_source,
        )

    def to_summary(self) -> dict[str, Any]:
        return {
            "motor_percent": {"value": self.motor_percent, "source": self.motor_percent_source},
            "motor_sec": {"value": self.motor_sec, "source": self.motor_sec_source},
            "motor_count": {"value": self.motor_count, "source": self.motor_count_source},
        }


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuiltInMotorDebugTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "motor-debug"
    TASK_DESCRIPTION: ClassVar[str] = "Run built-in real no-props motor idle spin without takeoff."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        task_config_path: str | Path | None = None,
        motor_percent: float | None = None,
        motor_sec: float | None = None,
        motor_count: int | None = None,
        console: Console | None = None,
    ) -> int:
        console = console or Console()
        task_config = MotorDebugTaskConfig.from_file(
            task_config_path=task_config_path,
            cli_motor_percent=motor_percent,
            cli_motor_sec=motor_sec,
            cli_motor_count=motor_count,
        )
        config = RunConfig.from_config(config_path=config_path, task_name="real-prepare")
        artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_real_motor_debug/{config.run_id}"))
        artifact_dir.mkdir(parents=True, exist_ok=True)
        summary_path = artifact_dir / "summary.json"
        companion_log = start_process_logger(
            process_name="companion",
            log_path=artifact_dir / "logs" / "companion.log",
        )
        companion_log.logger.info(
            "Starting real motor-debug companion process logger: motors={} spin_mode=armed_idle duration_sec={}",
            task_config.motor_count,
            task_config.motor_sec,
        )
        common_state = collect_motor_debug_common_state(config, process_logger=companion_log.logger)
        print_motor_debug_run_start(console, config=config, task_config=task_config, summary_path=summary_path)
        summary = run_motor_debug_sequence(
            config=config,
            summary_path=summary_path,
            motor_percent=task_config.motor_percent,
            motor_sec=task_config.motor_sec,
            motor_count=task_config.motor_count,
            process_logger=companion_log.logger,
        )
        if common_state:
            summary["common_state"] = common_state
        companion_log.logger.info("Real motor-debug completed: ok={} blockers={}", summary["ok"], summary["blockers"])
        summary.setdefault("schema_version", TASK_RESULT_SCHEMA_VERSION)
        summary["logs"] = close_process_loggers([companion_log])
        summary["task_config"] = task_config.to_summary()
        summary["artifact_dir"] = str(artifact_dir)
        summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        print_motor_debug_summary(console, summary=summary, summary_path=summary_path)
        return 0 if summary["ok"] else 20

    def build_real_task_doctor(
        self,
        *,
        config: object,
        upstream: Mapping[str, Any],
    ) -> Mapping[str, Any] | None:
        from src.configs.run_config import RunConfig
        from src.workflows.real.task_doctor import task_fcu_status_metadata

        if not isinstance(config, RunConfig):
            return None
        metadata = task_fcu_status_metadata(config, upstream)
        current_mode = str(
            metadata.get("mode") or metadata.get("mode_name") or metadata.get("flight_mode") or ""
        ).upper()
        return {
            "ok": True,
            "blocked": False,
            "blockers": [],
            "required_mode": "GUIDED",
            "guided_gate": "run_stage",
            "mode_switch_claim": "deferred_to_motor_debug_run",
            "current_fcu_mode": current_mode or "unknown",
            "guided_mode": "deferred_to_run",
        }


def collect_motor_debug_common_state(config: RunConfig, *, process_logger: Any | None = None) -> dict[str, Any]:
    try:
        from src.workflows.real.common_doctor import (
            build_real_common_doctor_summary,
            wait_for_common_doctor_topic_snapshot,
        )

        summary = build_real_common_doctor_summary(
            config,
            task_name="motor-debug",
            topic_snapshot=wait_for_common_doctor_topic_snapshot(config, task_name="motor-debug"),
        )
    except Exception as exc:  # pragma: no cover - real ROS graph dependent.
        if process_logger is not None:
            process_logger.warning("Failed to collect motor-debug common state: {}", exc)
        return {}
    common_state = summary.get("common_state", {})
    if isinstance(common_state, dict):
        return common_state
    return {}


def run_motor_debug_sequence(
    *,
    config: RunConfig,
    summary_path: Path,
    motor_percent: float,
    motor_sec: float,
    motor_count: int,
    process_logger: Any | None = None,
) -> dict[str, Any]:
    serial = config.orchestration.real_prepare.mavlink_router_serial_port
    baud = config.orchestration.real_prepare.mavlink_router_baud
    endpoint = mavlink_router_endpoint(config)
    command = motor_debug_runtime_command(
        serial=serial,
        baud=baud,
        endpoint=endpoint,
        motor_percent=motor_percent,
        motor_sec=motor_sec,
        motor_count=motor_count,
        summary_path=summary_path,
    )
    if process_logger is not None:
        process_logger.info("Starting motor-debug runtime command: {}", " ".join(command))
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if process_logger is not None:
        if result.stdout.strip():
            process_logger.info("motor-debug runtime stdout: {}", result.stdout.strip())
        if result.stderr.strip():
            process_logger.error("motor-debug runtime stderr: {}", result.stderr.strip())
        process_logger.info("motor-debug runtime exited rc={}", result.returncode)
    if summary_path.is_file():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if isinstance(summary, dict):
            return summary
    return {
        "schema_version": TASK_RESULT_SCHEMA_VERSION,
        "ok": False,
        "blocked": True,
        "blockers": [f"motor_debug_runtime_summary_missing:rc={result.returncode}"],
        "task": "motor-debug",
        "claim": "runtime_failed",
        "serial": serial,
        "connection_endpoint": endpoint,
        "baud": baud,
        "motor_percent": motor_percent,
        "motor_sec": motor_sec,
        "motor_count": motor_count,
        "required_mode": REQUIRED_GUIDED_MODE_NAME,
        "guided_mode": {},
        "shutdown_claim": "not_evaluated",
    }


def motor_debug_runtime_command(
    *,
    serial: str,
    baud: int | float,
    endpoint: str,
    motor_percent: float,
    motor_sec: float,
    motor_count: int,
    summary_path: Path,
) -> list[str]:
    return [
        sys.executable,
        "-m",
        "navlab.real.companion.nodes.motor_debug",
        "--serial",
        serial,
        "--baud",
        str(baud),
        "--endpoint",
        endpoint,
        "--motor-percent",
        str(motor_percent),
        "--motor-sec",
        str(motor_sec),
        "--motor-count",
        str(motor_count),
        "--summary-path",
        str(summary_path),
    ]


def mavlink_router_endpoint(config: RunConfig) -> str:
    endpoint = config.orchestration.real_prepare.mavlink_router_local_endpoint.strip()
    if endpoint.startswith(("udp:", "udpin:", "udpout:", "tcp:")):
        return endpoint
    host, _, port = endpoint.partition(":")
    return f"udpin:{host or '127.0.0.1'}:{port or '14550'}"


def print_motor_debug_run_start(console: Console, *, config: RunConfig, task_config: Any, summary_path: Path) -> None:
    prepare = config.orchestration.real_prepare
    grid = Table.grid(padding=(0, 2))
    grid.add_column(style="bold")
    grid.add_column()
    grid.add_row("Task", "motor-debug")
    grid.add_row("Serial", f"{prepare.mavlink_router_serial_port} @ {prepare.mavlink_router_baud}")
    grid.add_row("Endpoint", mavlink_router_endpoint(config))
    grid.add_row("Motors", str(task_config.motor_count))
    grid.add_row("Spin mode", "armed_idle")
    grid.add_row("Hold", f"{task_config.motor_sec} sec")
    grid.add_row("Required mode", REQUIRED_GUIDED_MODE_NAME)
    grid.add_row("Guided gate", "run_stage")
    grid.add_row("Arm command", "MAV_CMD_COMPONENT_ARM_DISARM param1=1 param2=0")
    grid.add_row("Disarm command", "MAV_CMD_COMPONENT_ARM_DISARM param1=0 param2=0")
    grid.add_row("Summary", str(summary_path))
    console.print(Panel(grid, title="NavLab Real Motor Debug Run", border_style="blue"))


def print_motor_debug_summary(console: Console, *, summary: dict[str, Any], summary_path: Path) -> None:
    grid = Table.grid(padding=(0, 2))
    grid.add_column(style="bold")
    grid.add_column()
    grid.add_row("Status", "OK" if summary.get("ok") else "BLOCKED")
    grid.add_row("Task", "motor-debug")
    grid.add_row("Serial", f"{summary.get('serial')} @ {summary.get('baud')}")
    grid.add_row("Endpoint", str(summary.get("connection_endpoint", "")))
    grid.add_row("Motors", str(summary.get("motor_count")))
    grid.add_row("Spin mode", str(summary.get("spin_mode")))
    grid.add_row("Hold", f"{summary.get('motor_sec')} sec")
    grid.add_row("Required mode", str(summary.get("required_mode")))
    grid.add_row("Guided mode", str(summary.get("guided_mode", {}).get("ok")))
    common_state = summary.get("common_state", {})
    if isinstance(common_state, Mapping):
        grid.add_row("ExternalNav config", str(common_state.get("configured_external_nav_source_set", "unknown")))
        grid.add_row("EKF active src", str(common_state.get("observed_ekf_source_set", "not_observed")))
        switch = common_state.get("ekf_source_set_switch", {})
        if isinstance(switch, Mapping) and switch.get("enabled") is True:
            target = switch.get("target_source_set", "unknown")
            sent = switch.get("sent", "unknown")
            ack = switch.get("ack_result", "pending" if sent else "not_sent")
            grid.add_row("EKF switch", f"target={target}, sent={sent}, ack={ack}")
        src2 = common_state.get("ek3_src2", {})
        if isinstance(src2, Mapping):
            grid.add_row(
                "EK3 SRC2",
                f'posxy={src2.get("posxy", "unknown")}, velxy={src2.get("velxy", "unknown")}, '
                f'velz={src2.get("velz", "unknown")}, yaw={src2.get("yaw", "unknown")}, '
                f'posz={src2.get("posz", "unknown")}',
            )
        gps_source_sets = common_state.get("gps_source_sets", [])
        if isinstance(gps_source_sets, list) and gps_source_sets:
            grid.add_row("GPS EKF src", ", ".join(str(item) for item in gps_source_sets))
    grid.add_row("Shutdown", str(summary.get("shutdown_claim")))
    grid.add_row("Logs", _format_log_count(summary.get("logs", [])))
    grid.add_row("Summary", str(summary_path))
    border_style = "green" if summary.get("ok") else "red"
    console.print(Panel(grid, title="NavLab Real Motor Debug", border_style=border_style))

    blockers = [str(item) for item in summary.get("blockers", [])]
    if blockers:
        blocker_grid = Table.grid()
        blocker_grid.add_column()
        for blocker in blockers[:8]:
            blocker_grid.add_row(f"- {blocker}")
        if len(blockers) > 8:
            blocker_grid.add_row(f"- ... {len(blockers) - 8} more")
        console.print(Panel(blocker_grid, title="Blockers", border_style="red"))

    logs = [item for item in summary.get("logs", []) if isinstance(item, dict)]
    if logs:
        log_grid = Table.grid(padding=(0, 2))
        log_grid.add_column(style="bold")
        log_grid.add_column()
        for item in logs:
            log_grid.add_row(
                str(item.get("process", "unknown")),
                f"{item.get('entries', 0)} entries, {item.get('bytes', 0)} bytes",
            )
            log_grid.add_row("", str(item.get("path", "")))
        console.print(Panel(log_grid, title="Logs", border_style="blue"))


def _format_log_count(logs: object) -> str:
    if not isinstance(logs, list):
        return "0 processes"
    count = len([item for item in logs if isinstance(item, dict)])
    return f"{count} process" if count == 1 else f"{count} processes"
