from __future__ import annotations

from pathlib import Path

import pytest
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


def test_orchestration_task_registry_contains_only_python_real_tasks() -> None:
    assert TaskRegistry.names() == ("motor-debug",)
    assert TaskRegistry.create("motor-debug").description
    for task_name in (
        "build",
        "doctor",
        "hover",
        "exploration",
        "scan-robustness",
        "hover-doctor",
        "exploration-doctor",
        "scan-robustness-doctor",
    ):
        with pytest.raises(ValueError, match=f"unknown orchestration task '{task_name}'"):
            TaskRegistry.create(task_name)


def test_orchestration_sources_do_not_import_retired_python_sim_tasks() -> None:
    retired_markers = (
        "src.tasks.built_in.hover",
        "src.tasks.built_in.exploration",
        "src.tasks.built_in.scan_robustness",
        "src.tasks.helpers",
        "src.tasks.workflows",
        "config.simulation.toml",
    )
    offenders: list[Path] = []
    for root in (Path("orchestration/src"),):
        for path in root.rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            source = path.read_text(encoding="utf-8")
            if any(marker in source for marker in retired_markers):
                offenders.append(path)

    assert offenders == []


def test_orchestration_task_registry_requires_task_name() -> None:
    class MissingNameTask(OrchestrationTask):
        TASK_DESCRIPTION = "Missing task name"

        def run(self, **kwargs: object) -> int:
            return 0

    with pytest.raises(ValueError, match="MissingNameTask must define TASK_NAME"):
        TaskRegistry.register(MissingNameTask)


def test_orchestration_task_registry_rejects_duplicate_task_name() -> None:
    class DuplicateMotorDebugTask(OrchestrationTask):
        TASK_NAME = "motor-debug"
        TASK_DESCRIPTION = "Duplicate motor debug task"

        def run(self, **kwargs: object) -> int:
            return 0

    with pytest.raises(ValueError, match="orchestration task 'motor-debug' is already registered"):
        TaskRegistry.register(DuplicateMotorDebugTask)
