from __future__ import annotations

from src.tasks.acceptance import AcceptanceTask
from src.tasks.build import BuildTask
from src.tasks.doctor import DoctorTask
from src.tasks.hover import HoverAcceptanceTask
from src.tasks.hover_diagnostic import HoverDiagnosticTask
from src.tasks.hover_slam_diagnostic import HoverSlamDiagnosticTask
from src.tasks.registry import TaskRegistry

__all__ = [
    "AcceptanceTask",
    "BuildTask",
    "DoctorTask",
    "HoverAcceptanceTask",
    "HoverDiagnosticTask",
    "HoverSlamDiagnosticTask",
    "TaskRegistry",
]
