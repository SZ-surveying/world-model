from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from src.config import RunConfig, load_motor_debug_task_config
from src.logger_utils import close_process_loggers, start_process_logger
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry

MAX_MOTOR_DEBUG_SEC = 5.0
MAX_MOTOR_DEBUG_COUNT = 8
REQUIRED_GUIDED_MODE_NAME = "GUIDED"


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
        task_config = load_motor_debug_task_config(
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
        _print_motor_debug_run_start(console, config=config, task_config=task_config, summary_path=summary_path)
        summary = _run_motor_debug_sequence(
            config=config,
            motor_percent=task_config.motor_percent,
            motor_sec=task_config.motor_sec,
            motor_count=task_config.motor_count,
            process_logger=companion_log.logger,
        )
        companion_log.logger.info("Real motor-debug completed: ok={} blockers={}", summary["ok"], summary["blockers"])
        summary["logs"] = close_process_loggers([companion_log])
        summary["task_config"] = task_config.to_summary()
        summary["artifact_dir"] = str(artifact_dir)
        summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        _print_motor_debug_summary(console, summary=summary, summary_path=summary_path)
        return 0 if summary["ok"] else 20


def build_motor_debug_plan(
    *,
    motor_percent: float,
    motor_sec: float,
    motor_count: int,
) -> dict[str, Any]:
    blockers = _validate_motor_debug_limits(
        motor_percent=motor_percent,
        motor_sec=motor_sec,
        motor_count=motor_count,
    )
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "task": "motor-debug",
        "claim": "plan_only",
        "no_takeoff": True,
        "requires_no_props": True,
        "guided_mode_required": True,
        "required_mode": REQUIRED_GUIDED_MODE_NAME,
        "spin_mode": "armed_idle",
        "throttle_command_claim": "not_sent",
        "motor_percent": motor_percent,
        "motor_sec": motor_sec,
        "motor_count": motor_count,
        "steps": [
            {"step": "arm", "claim": "start_all_motors_at_fcu_armed_idle"},
            {"step": "hold", "duration_sec": motor_sec},
            {"step": "disarm", "claim": "stop_all_motors"},
        ],
        "shutdown": "send_disarm_after_idle_spin",
        "landing_claim": "not_evaluated_no_takeoff",
    }


def _run_motor_debug_sequence(
    *,
    config: RunConfig,
    motor_percent: float,
    motor_sec: float,
    motor_count: int,
    process_logger: Any | None = None,
) -> dict[str, Any]:
    plan = build_motor_debug_plan(motor_percent=motor_percent, motor_sec=motor_sec, motor_count=motor_count)
    serial = config.orchestration.real_prepare.mavlink_router_serial_port
    baud = config.orchestration.real_prepare.mavlink_router_baud
    endpoint = _mavlink_router_endpoint(config)
    summary: dict[str, Any] = {
        **plan,
        "claim": "evaluated",
        "serial": serial,
        "connection_endpoint": endpoint,
        "baud": baud,
        "arm_claim": "not_requested",
        "takeoff_claim": "not_evaluated",
        "landing_claim": "not_evaluated_no_takeoff",
        "guided_mode_claim": "not_evaluated",
        "shutdown_claim": "not_evaluated",
        "guided_mode": {},
        "acks": [],
    }
    if plan["blocked"]:
        if process_logger is not None:
            process_logger.warning("Motor-debug plan blocked before MAVLink connection: {}", plan["blockers"])
        return summary
    try:
        from pymavlink import mavutil
        from pymavlink.dialects.v20 import ardupilotmega as mavlink
    except ImportError as exc:
        summary["blockers"] = ["motor_debug_dependency_missing:pymavlink"]
        summary["blocked"] = True
        summary["ok"] = False
        summary["error"] = str(exc)
        if process_logger is not None:
            process_logger.error("Missing pymavlink dependency: {}", exc)
        return summary

    master = None
    try:
        if process_logger is not None:
            process_logger.info("Opening MAVLink connection: {} @ {}", endpoint, baud)
        master = mavutil.mavlink_connection(endpoint, baud=baud, autoreconnect=False, source_system=255)
        heartbeat = master.wait_heartbeat(timeout=10.0)
        if heartbeat is None:
            summary["blockers"] = ["motor_debug_heartbeat_missing"]
            summary["blocked"] = True
            summary["ok"] = False
            if process_logger is not None:
                process_logger.error("MAVLink heartbeat missing on {} @ {}", serial, baud)
            return summary
        target_system = master.target_system
        target_component = master.target_component
        summary["target_system"] = target_system
        summary["target_component"] = target_component
        if process_logger is not None:
            process_logger.info(
                "MAVLink heartbeat ok: target_system={} target_component={}",
                target_system,
                target_component,
            )
        guided_mode = _ensure_guided_mode(master, mode_name=REQUIRED_GUIDED_MODE_NAME, timeout_sec=5.0)
        summary["guided_mode"] = guided_mode
        summary["guided_mode_claim"] = "evaluated"
        if not guided_mode.get("ok"):
            summary["blockers"] = [str(guided_mode.get("blocker", "motor_debug_guided_mode_not_confirmed"))]
            summary["blocked"] = True
            summary["ok"] = False
            if process_logger is not None:
                process_logger.error("GUIDED mode was not confirmed: {}", summary["blockers"])
            return summary
        if process_logger is not None:
            process_logger.info("GUIDED mode confirmed: {}", guided_mode)
        arm_ack = _send_arm_disarm(master, target_system, target_component, mavlink, arm=True)
        summary["acks"].append({"action": "arm", **arm_ack})
        summary["arm_claim"] = "arm_command_sent"
        if not arm_ack.get("accepted"):
            summary["blockers"] = _command_rejection_blockers(
                prefix="motor_debug_arm_rejected",
                ack=arm_ack,
            )
            summary["blocked"] = True
            summary["ok"] = False
            if process_logger is not None:
                process_logger.error("Motor idle spin arm rejected: {}", arm_ack)
            summary["shutdown_claim"] = "not_evaluated_arm_rejected"
            return summary
        if process_logger is not None:
            process_logger.info("Motor idle spin armed; holding for {} seconds", motor_sec)
        time.sleep(max(motor_sec, 0.0))
        disarm_ack = _send_arm_disarm(master, target_system, target_component, mavlink, arm=False)
        summary["acks"].append({"action": "disarm", **disarm_ack})
        summary["shutdown_claim"] = "disarm_command_sent"
        if not disarm_ack.get("accepted"):
            summary["blockers"] = _command_rejection_blockers(
                prefix="motor_debug_disarm_rejected",
                ack=disarm_ack,
            )
            summary["blocked"] = True
            summary["ok"] = False
            if process_logger is not None:
                process_logger.error("Motor idle spin disarm rejected: {}", disarm_ack)
            return summary
        if process_logger is not None:
            process_logger.info("Motor idle spin disarmed: {}", disarm_ack)
        summary["ok"] = True
        summary["blocked"] = False
        summary["blockers"] = []
        return summary
    except Exception as exc:  # pragma: no cover - hardware dependent.
        summary["blockers"] = [f"motor_debug_failed:{exc}"]
        summary["blocked"] = True
        summary["ok"] = False
        if process_logger is not None:
            process_logger.exception("Motor-debug arm/hold/disarm flow failed: {}", exc)
        return summary
    finally:
        if master is not None:
            master.close()


def _validate_motor_debug_limits(*, motor_percent: float, motor_sec: float, motor_count: int) -> list[str]:
    blockers: list[str] = []
    if motor_sec <= 0.0 or motor_sec > MAX_MOTOR_DEBUG_SEC:
        blockers.append(f"motor_debug_duration_out_of_range:{motor_sec:g}:max={MAX_MOTOR_DEBUG_SEC:g}")
    if motor_count <= 0 or motor_count > MAX_MOTOR_DEBUG_COUNT:
        blockers.append(f"motor_debug_motor_count_out_of_range:{motor_count}:max={MAX_MOTOR_DEBUG_COUNT}")
    return blockers


def _mavlink_router_endpoint(config: RunConfig) -> str:
    endpoint = config.orchestration.real_prepare.mavlink_router_local_endpoint.strip()
    if endpoint.startswith(("udp:", "udpin:", "udpout:", "tcp:")):
        return endpoint
    host, _, port = endpoint.partition(":")
    return f"udpin:{host or '127.0.0.1'}:{port or '14550'}"


def _wait_command_ack(master: Any, command: int, *, timeout_sec: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    status_text: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=max(deadline - time.monotonic(), 0.0),
        )
        if msg is None:
            break
        if _message_type(msg) == "STATUSTEXT":
            status_text.append(_status_text_summary(msg))
            continue
        if _message_type(msg) != "COMMAND_ACK":
            continue
        if int(getattr(msg, "command", -1)) != int(command):
            continue
        result = int(getattr(msg, "result", -1))
        if result != 0:
            status_text.extend(_drain_status_text(master, timeout_sec=1.0))
        return {
            "command": command,
            "result": result,
            "result_name": _mav_result_name(result),
            "result_param2": getattr(msg, "result_param2", None),
            "accepted": result == 0,
            "status_text": status_text,
        }
    return {
        "command": command,
        "result": None,
        "result_name": "timeout",
        "accepted": False,
        "timeout": True,
        "status_text": status_text,
    }


def _send_arm_disarm(
    master: Any,
    target_system: int,
    target_component: int,
    mavlink: Any,
    *,
    arm: bool,
) -> dict[str, Any]:
    master.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm else 0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    return _wait_command_ack(master, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout_sec=3.0)


def _drain_status_text(master: Any, *, timeout_sec: float) -> list[dict[str, Any]]:
    deadline = time.monotonic() + timeout_sec
    status_text: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="STATUSTEXT",
            blocking=True,
            timeout=max(deadline - time.monotonic(), 0.0),
        )
        if msg is None:
            break
        status_text.append(_status_text_summary(msg))
    return status_text


def _command_rejection_blockers(*, prefix: str, ack: dict[str, Any]) -> list[str]:
    result_name = str(ack.get("result_name") or ack.get("result") or "unknown")
    blockers = [f"{prefix}:{result_name}"]
    for item in ack.get("status_text", []):
        if not isinstance(item, dict):
            continue
        text = str(item.get("text", "")).strip()
        if text:
            blockers.append(f"{prefix}_status:{text}")
    return blockers


def _message_type(msg: Any) -> str:
    get_type = getattr(msg, "get_type", None)
    if callable(get_type):
        return str(get_type())
    return str(getattr(msg, "_type", ""))


def _status_text_summary(msg: Any) -> dict[str, Any]:
    return {
        "severity": getattr(msg, "severity", None),
        "text": str(getattr(msg, "text", "")).strip().rstrip("\x00"),
    }


def _mav_result_name(result: int | None) -> str:
    names = {
        0: "MAV_RESULT_ACCEPTED",
        1: "MAV_RESULT_TEMPORARILY_REJECTED",
        2: "MAV_RESULT_DENIED",
        3: "MAV_RESULT_UNSUPPORTED",
        4: "MAV_RESULT_FAILED",
        5: "MAV_RESULT_IN_PROGRESS",
        7: "MAV_RESULT_COMMAND_LONG_ONLY",
        8: "MAV_RESULT_COMMAND_INT_ONLY",
    }
    return names.get(result, f"MAV_RESULT_UNKNOWN:{result}")


def _ensure_guided_mode(master: Any, *, mode_name: str, timeout_sec: float) -> dict[str, Any]:
    mapping = master.mode_mapping()
    mode_id = mapping.get(mode_name)
    result: dict[str, Any] = {
        "ok": False,
        "required_mode": mode_name,
        "mode_id": mode_id,
        "mode_mapping_has_mode": mode_id is not None,
        "set_mode_sent": False,
        "observed_mode_id": None,
    }
    if mode_id is None:
        result["blocker"] = f"motor_debug_required_mode_missing:{mode_name}"
        return result

    master.set_mode(mode_id)
    result["set_mode_sent"] = True
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=max(deadline - time.monotonic(), 0.0))
        if msg is None:
            break
        observed = int(getattr(msg, "custom_mode", -1))
        result["observed_mode_id"] = observed
        if observed == int(mode_id):
            result["ok"] = True
            return result
    result["blocker"] = f"motor_debug_required_mode_not_observed:{mode_name}"
    return result


def _print_motor_debug_run_start(console: Console, *, config: RunConfig, task_config: Any, summary_path: Path) -> None:
    prepare = config.orchestration.real_prepare
    grid = Table.grid(padding=(0, 2))
    grid.add_column(style="bold")
    grid.add_column()
    grid.add_row("Task", "motor-debug")
    grid.add_row("Serial", f"{prepare.mavlink_router_serial_port} @ {prepare.mavlink_router_baud}")
    grid.add_row("Endpoint", _mavlink_router_endpoint(config))
    grid.add_row("Motors", str(task_config.motor_count))
    grid.add_row("Spin mode", "armed_idle")
    grid.add_row("Hold", f"{task_config.motor_sec} sec")
    grid.add_row("Required mode", REQUIRED_GUIDED_MODE_NAME)
    grid.add_row("Guided gate", "run_stage")
    grid.add_row("Arm command", "MAV_CMD_COMPONENT_ARM_DISARM param1=1 param2=0")
    grid.add_row("Disarm command", "MAV_CMD_COMPONENT_ARM_DISARM param1=0 param2=0")
    grid.add_row("Summary", str(summary_path))
    console.print(Panel(grid, title="NavLab Real Motor Debug Run", border_style="blue"))


def _print_motor_debug_summary(console: Console, *, summary: dict[str, Any], summary_path: Path) -> None:
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
