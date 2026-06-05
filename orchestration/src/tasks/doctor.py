from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

from rich.console import Console

from src import host
from src.config import RunConfig
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class DoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check the NavLab companion image contents without running a mission."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_companion_doctor/{config.run_id}"))
        artifact_dir.mkdir(parents=True, exist_ok=True)
        console.print("[bold cyan]Checking NavLab companion image[/bold cyan]")
        rc = host._docker_run_runtime_command(
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
