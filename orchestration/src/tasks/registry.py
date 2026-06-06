from __future__ import annotations

from importlib import import_module
from typing import ClassVar

from src.tasks.base import OrchestrationTask

TaskType = type[OrchestrationTask]


class TaskRegistry:
    _tasks: ClassVar[dict[str, TaskType]] = {}
    _default_modules_loaded: ClassVar[bool] = False

    @classmethod
    def register(cls, task_cls: TaskType) -> TaskType:
        if not issubclass(task_cls, OrchestrationTask):
            raise TypeError(f"registered task must inherit OrchestrationTask, got {task_cls.__name__}")
        task_name = task_cls.TASK_NAME
        if not task_name:
            raise ValueError(f"{task_cls.__name__} must define TASK_NAME")
        normalized = cls._normalize_name(task_name)
        if normalized in cls._tasks:
            raise ValueError(f"orchestration task '{task_name}' is already registered")
        cls._tasks[normalized] = task_cls
        return task_cls

    @classmethod
    def create(cls, name: str) -> OrchestrationTask:
        cls._ensure_default_modules_loaded()
        normalized = cls._normalize_name(name)
        try:
            task_cls = cls._tasks[normalized]
        except KeyError as exc:
            available = ", ".join(cls.names()) or "<none>"
            raise ValueError(f"unknown orchestration task '{name}'. Available tasks: {available}") from exc
        return task_cls()

    @classmethod
    def names(cls) -> tuple[str, ...]:
        cls._ensure_default_modules_loaded()
        return tuple(sorted(cls._tasks))

    @classmethod
    def _ensure_default_modules_loaded(cls) -> None:
        if cls._default_modules_loaded:
            return
        cls._default_modules_loaded = True
        for module_name in (
            "src.tasks.build",
            "src.tasks.doctor",
            "src.tasks.acceptance",
            "src.tasks.hover",
            "src.tasks.hover_diagnostic",
            "src.tasks.hover_slam_diagnostic",
            "src.tasks.official_baseline",
            "src.tasks.official_maze_x2",
            "src.tasks.rangefinder_imu",
        ):
            import_module(module_name)

    @staticmethod
    def _normalize_name(name: str) -> str:
        normalized = name.strip()
        if not normalized:
            raise ValueError("task name cannot be empty")
        return normalized
