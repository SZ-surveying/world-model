from __future__ import annotations

import json
import os
import textwrap
from collections.abc import Mapping
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from src.configs.run_config import RunConfig
from src.workflows.real.prepare import RealTopicSnapshot, check_real_task_upstream_topics
from src.workflows.real.task_specs import (
    build_real_task_doctor,
    real_altitude_hold_doctor,
    task_fcu_status_metadata,
)


def run_real_task_doctor(
    *,
    task_name: str,
    config_path: str | Path | None = None,
    task_config_path: str | Path | None = None,
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
        os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_real_task_doctor/{config.run_id}/{task_name}")
    )
    summary = build_real_task_doctor_summary(task_name, config)
    summary_path = artifact_dir / "summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    rc = 0 if summary["ok"] else 20
    color = "green" if summary["ok"] else "red"
    console.print("\n\nChecking real task doctor contract")
    console.print(f"[{color}]Real task doctor rc={rc}[/{color}]")
    print_real_task_doctor_summary(console, summary=summary, summary_path=summary_path)
    return rc


def build_real_task_doctor_summary(
    task_name: str,
    config: RunConfig,
    *,
    topic_snapshot: RealTopicSnapshot | None = None,
) -> dict[str, Any]:
    upstream = check_real_task_upstream_topics(task_name, config, topic_snapshot=topic_snapshot)
    blockers = list(upstream["blockers"])
    task_specific = _task_specific_doctor(task_name, config, upstream)
    blockers.extend(str(item) for item in task_specific.get("blockers", []))
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


def _task_specific_doctor(task_name: str, config: RunConfig, upstream: Mapping[str, Any]) -> dict[str, Any]:
    return build_real_task_doctor(task_name, config=config, upstream=upstream)


def print_real_task_doctor_summary(console: Console, *, summary: Mapping[str, Any], summary_path: Path) -> None:
    task_specific = summary.get("task_specific", {})
    grid = _summary_grid()
    grid.add_row("Status", "OK" if summary.get("ok") else "BLOCKED")
    grid.add_row("Task", str(summary.get("task_name")))
    if summary.get("task_name") == "motor-debug" and isinstance(task_specific, Mapping):
        grid.add_row("Required mode", str(task_specific.get("required_mode", "GUIDED")))
        grid.add_row("Current mode", str(task_specific.get("current_fcu_mode", "unknown")))
        grid.add_row("Guided gate", str(task_specific.get("guided_gate", "run_stage")))
        grid.add_row("Mode switch", str(task_specific.get("mode_switch_claim", "deferred_to_run")))
    else:
        grid.add_row("Arm", str(summary.get("arm_claim")))
        grid.add_row("Takeoff", str(summary.get("takeoff_claim")))
        if isinstance(task_specific, Mapping) and task_specific.get("skipped"):
            grid.add_row("Task doctor", f"skipped:{task_specific.get('reason', 'not_implemented')}")
    grid.add_row("Summary", "")
    grid.add_row("", _wrapped_summary_path(console, summary_path))
    border_style = "green" if summary.get("ok") else "red"
    title = "NavLab Real Motor Debug Doctor" if summary.get("task_name") == "motor-debug" else "NavLab Real Task Doctor"
    console.print(Panel(grid, title=title, border_style=border_style))
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
