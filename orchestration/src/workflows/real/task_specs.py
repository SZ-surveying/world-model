from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any

from src.configs.run_config import RunConfig

RealTaskDoctorBuilder = Callable[[RunConfig, Mapping[str, Any]], Mapping[str, Any]]


def build_real_task_doctor(task_name: str, *, config: RunConfig, upstream: Mapping[str, Any]) -> dict[str, Any]:
    builder = _REAL_TASK_DOCTOR_BUILDERS.get(task_name.strip())
    if builder is None:
        return {
            "ok": True,
            "blocked": False,
            "blockers": [],
            "skipped": True,
            "reason": f"task_not_registered:unknown real task doctor spec '{task_name}'",
        }
    return dict(builder(config, upstream))


def task_fcu_status_metadata(config: RunConfig, upstream: Mapping[str, Any]) -> dict[str, Any]:
    status_topic = config.orchestration.fcu_controller.status_topic
    status = upstream.get("required_topics", {}).get(status_topic, {})
    metadata = dict(status.get("metadata", {})) if isinstance(status, dict) else {}
    if metadata:
        return metadata
    bridge_status_topic = config.orchestration.real_prepare.fcu_bridge_state_topic
    bridge_status = upstream.get("required_topics", {}).get(bridge_status_topic, {})
    return dict(bridge_status.get("metadata", {})) if isinstance(bridge_status, dict) else {}


def real_altitude_hold_doctor(config: RunConfig, upstream: Mapping[str, Any]) -> dict[str, Any]:
    from src.workflows.real.prepare import check_altitude_hold_mode

    return check_altitude_hold_mode(config, upstream, fcu_status_metadata=task_fcu_status_metadata(config, upstream))


def _build_hover_real_task_doctor(config: RunConfig, upstream: Mapping[str, Any]) -> Mapping[str, Any]:
    blockers: list[str] = []
    landing_policy = config.orchestration.landing.policy_for_task("hover")
    metadata = task_fcu_status_metadata(config, upstream)
    if config.orchestration.fcu_controller.takeoff_alt_m <= 0:
        blockers.append("task_takeoff_altitude_invalid")
    if metadata.get("armed") is True:
        blockers.append("task_initial_fcu_armed")
    altitude_hold = real_altitude_hold_doctor(config, upstream)
    if altitude_hold["blocked"]:
        blockers.extend(altitude_hold["blockers"])
    if landing_policy != "land_in_place":
        blockers.append(f"hover_landing_policy_invalid:{landing_policy}")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "landing_policy": landing_policy,
        "takeoff_alt_m": config.orchestration.fcu_controller.takeoff_alt_m,
        "altitude_hold": altitude_hold,
    }


def _build_exploration_real_task_doctor(config: RunConfig, upstream: Mapping[str, Any]) -> Mapping[str, Any]:
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


def _build_scan_robustness_real_task_doctor(config: RunConfig, upstream: Mapping[str, Any]) -> Mapping[str, Any]:
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


def _build_motor_debug_real_task_doctor(config: RunConfig, upstream: Mapping[str, Any]) -> Mapping[str, Any]:
    metadata = task_fcu_status_metadata(config, upstream)
    current_mode = str(metadata.get("mode") or metadata.get("mode_name") or metadata.get("flight_mode") or "").upper()
    return {
        "ok": True,
        "blocked": False,
        "blockers": [],
        "required_mode": "GUIDED",
        "guided_gate": "run_stage",
        "mode_switch_claim": "deferred_to_motor_debug_run",
        "current_fcu_mode": current_mode or "unknown",
        "guided_mode": "deferred_to_run",
    }


_REAL_TASK_DOCTOR_BUILDERS: dict[str, RealTaskDoctorBuilder] = {
    "hover": _build_hover_real_task_doctor,
    "exploration": _build_exploration_real_task_doctor,
    "scan-robustness": _build_scan_robustness_real_task_doctor,
    "motor-debug": _build_motor_debug_real_task_doctor,
}
