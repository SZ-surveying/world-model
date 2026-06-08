from __future__ import annotations

from src.tasks.acceptance import AcceptanceTask
from src.tasks.airframe_disturbance_gate import AirframeDisturbanceGateAcceptanceTask, AirframeDisturbanceGateDoctorTask
from src.tasks.build import BuildTask
from src.tasks.doctor import DoctorTask
from src.tasks.exploration_gate import ExplorationGateAcceptanceTask, ExplorationGateDoctorTask
from src.tasks.hover import HoverAcceptanceTask
from src.tasks.hover_diagnostic import HoverDiagnosticTask
from src.tasks.hover_slam_diagnostic import HoverSlamDiagnosticTask
from src.tasks.motion_gate import MotionGateAcceptanceTask, MotionGateDoctorTask
from src.tasks.registry import TaskRegistry

__all__ = [
    "AcceptanceTask",
    "AirframeDisturbanceGateAcceptanceTask",
    "AirframeDisturbanceGateDoctorTask",
    "BuildTask",
    "DoctorTask",
    "ExplorationGateAcceptanceTask",
    "ExplorationGateDoctorTask",
    "HoverAcceptanceTask",
    "HoverDiagnosticTask",
    "HoverSlamDiagnosticTask",
    "MotionGateAcceptanceTask",
    "MotionGateDoctorTask",
    "TaskRegistry",
]
