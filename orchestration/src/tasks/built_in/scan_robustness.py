from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

from rich.console import Console

from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuiltInScanRobustnessDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "scan-robustness-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check built-in tilted-scan robustness prerequisites."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        from src.tasks.legacy.airframe_disturbance_gate import run_airframe_disturbance_gate_doctor

        return run_airframe_disturbance_gate_doctor(config_path=config_path, console=console)


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuiltInScanRobustnessTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "scan-robustness"
    TASK_DESCRIPTION: ClassVar[str] = "Run built-in P9/P12 tilted-scan robustness acceptance."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        duration_sec: float = 240.0,
        live_replay: bool = True,
        live_profiles: tuple[str, ...] = (),
        console: Console | None = None,
    ) -> int:
        from src.tasks.legacy.airframe_disturbance_gate import run_airframe_disturbance_gate_acceptance

        return run_airframe_disturbance_gate_acceptance(
            config_path=config_path,
            duration_sec=duration_sec,
            live_replay=live_replay,
            live_profiles=live_profiles,
            console=console,
        )
