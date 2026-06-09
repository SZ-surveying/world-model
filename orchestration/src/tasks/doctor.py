from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

from rich.console import Console

from src import host
from src.config import RunConfig
from src.project_config import DEFAULT_RUNTIME_BACKEND, DEFAULT_RUNTIME_MODE
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


def _runtime_backend_mode_from_env() -> tuple[str, str]:
    backend = os.environ.get("NAVLAB_RUNTIME_BACKEND", DEFAULT_RUNTIME_BACKEND).strip().lower()
    mode = os.environ.get("NAVLAB_RUNTIME_MODE", DEFAULT_RUNTIME_MODE).strip().lower()
    return backend, mode


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class DoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check the active runtime without running a mission."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        task_config_path: str | Path | None = None,
        console: Console | None = None,
    ) -> int:
        console = console or Console()
        backend, mode = _runtime_backend_mode_from_env()
        if backend == "process" and mode == "real":
            from src.tasks.real_preflight import RealPreflightDoctorTask

            console.print("[bold cyan]Checking process+real runtime preflight[/bold cyan]")
            return RealPreflightDoctorTask().run(
                config_path=config_path,
                task_config_path=task_config_path,
                console=console,
            )
        if backend != "docker" or mode != "simulation":
            console.print("[red]Unsupported runtime doctor combination:[/red] " f"{backend}+{mode}")
            return 20
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_companion_doctor/{config.run_id}"))
        artifact_dir.mkdir(parents=True, exist_ok=True)
        console.print("[bold cyan]Checking docker+simulation companion image[/bold cyan]")
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
