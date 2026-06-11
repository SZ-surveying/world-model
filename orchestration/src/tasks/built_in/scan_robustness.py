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
class BuiltInScanRobustnessDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "scan-robustness-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check built-in tilted-scan robustness prerequisites."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        task_config_path: str | Path | None = None,
        console: Console | None = None,
    ) -> int:
        from src.tasks.workflows.scan_robustness import run_airframe_disturbance_gate_doctor

        return run_airframe_disturbance_gate_doctor(
            config_path=config_path,
            task_config_path=task_config_path,
            console=console,
        )


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuiltInScanRobustnessTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "scan-robustness"
    TASK_DESCRIPTION: ClassVar[str] = "Run built-in P9/P12 tilted-scan robustness acceptance."

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
        landing_policy = config.orchestration.landing.policy_for_task("scan-robustness")
        metadata = task_fcu_status_metadata(config, upstream)
        if config.orchestration.fcu_controller.takeoff_alt_m <= 0:
            blockers.append("task_takeoff_altitude_invalid")
        if metadata.get("armed") is True:
            blockers.append("task_initial_fcu_armed")
        altitude_hold = real_altitude_hold_doctor(config, upstream)
        if altitude_hold["blocked"]:
            blockers.extend(altitude_hold["blockers"])
        if landing_policy != "land_in_place":
            blockers.append(f"scan_robustness_landing_policy_invalid:{landing_policy}")
        if not config.orchestration.scan_stabilization.enabled:
            blockers.append("scan_robustness_stabilization_disabled")
        return {
            "ok": not blockers,
            "blocked": bool(blockers),
            "blockers": blockers,
            "landing_policy": landing_policy,
            "takeoff_alt_m": config.orchestration.fcu_controller.takeoff_alt_m,
            "altitude_hold": altitude_hold,
            "scan_stabilization_enabled": config.orchestration.scan_stabilization.enabled,
            "scan_stabilization_status_topic": config.orchestration.scan_stabilization.status_topic,
        }

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        task_config_path: str | Path | None = None,
        duration_sec: float | None = None,
        live_replay: bool | None = None,
        live_profiles: tuple[str, ...] | None = None,
        console: Console | None = None,
    ) -> int:
        from src.tasks.workflows.scan_robustness import run_airframe_disturbance_gate_acceptance

        return run_airframe_disturbance_gate_acceptance(
            config_path=config_path,
            task_config_path=task_config_path,
            duration_sec=duration_sec,
            live_replay=live_replay,
            live_profiles=live_profiles,
            console=console,
        )
