from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

from rich.console import Console

from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuiltInExplorationDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "exploration-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check built-in P8 movement/exploration prerequisites."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        task_config_path: str | Path | None = None,
        console: Console | None = None,
    ) -> int:
        from src.tasks.workflows.exploration import run_exploration_gate_doctor

        return run_exploration_gate_doctor(config_path=config_path, task_config_path=task_config_path, console=console)


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuiltInExplorationTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "exploration"
    TASK_DESCRIPTION: ClassVar[str] = "Run built-in P8 movement/exploration acceptance."

    def build_real_task_doctor(
        self,
        *,
        config: object,
        upstream: Mapping[str, Any],
    ) -> Mapping[str, Any] | None:
        from src.configs.run_config import RunConfig
        from src.workflows.real.task_doctor import real_altitude_hold_doctor, task_fcu_status_metadata

        if not isinstance(config, RunConfig):
            return None
        blockers: list[str] = []
        landing_policy = config.orchestration.landing.policy_for_task("exploration")
        metadata = task_fcu_status_metadata(config, upstream)
        if config.orchestration.fcu_controller.takeoff_alt_m <= 0:
            blockers.append("task_takeoff_altitude_invalid")
        if metadata.get("armed") is True:
            blockers.append("task_initial_fcu_armed")
        altitude_hold = real_altitude_hold_doctor(config, upstream)
        if altitude_hold["blocked"]:
            blockers.extend(altitude_hold["blockers"])
        if landing_policy != "return_home_then_land":
            blockers.append(f"exploration_landing_policy_invalid:{landing_policy}")
        if not config.orchestration.landing.home_source:
            blockers.append("exploration_home_source_missing")
        if (
            config.orchestration.motion_gate.motion_distance_m <= 0
            or config.orchestration.motion_gate.motion_speed_mps <= 0
        ):
            blockers.append("exploration_bounded_movement_invalid")
        return {
            "ok": not blockers,
            "blocked": bool(blockers),
            "blockers": blockers,
            "landing_policy": landing_policy,
            "takeoff_alt_m": config.orchestration.fcu_controller.takeoff_alt_m,
            "altitude_hold": altitude_hold,
            "home_source": config.orchestration.landing.home_source,
            "motion_distance_m": config.orchestration.motion_gate.motion_distance_m,
        }

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        task_config_path: str | Path | None = None,
        duration_sec: float | None = None,
        simulation_profile: str | None = None,
        console: Console | None = None,
    ) -> int:
        from src.tasks.workflows.exploration import run_exploration_gate_acceptance

        return run_exploration_gate_acceptance(
            config_path=config_path,
            task_config_path=task_config_path,
            duration_sec=duration_sec,
            simulation_profile=simulation_profile,
            console=console,
        )
