from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

from rich.console import Console

from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuiltInExplorationDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "exploration-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check built-in P8 movement/exploration prerequisites."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        from src.tasks.workflows.exploration import run_exploration_gate_doctor

        return run_exploration_gate_doctor(config_path=config_path, console=console)


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuiltInExplorationTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "exploration"
    TASK_DESCRIPTION: ClassVar[str] = "Run built-in P8 movement/exploration acceptance."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        duration_sec: float = 150.0,
        simulation_profile: str = "ideal",
        console: Console | None = None,
    ) -> int:
        from src.tasks.workflows.exploration import run_exploration_gate_acceptance

        return run_exploration_gate_acceptance(
            config_path=config_path,
            duration_sec=duration_sec,
            simulation_profile=simulation_profile,
            console=console,
        )
