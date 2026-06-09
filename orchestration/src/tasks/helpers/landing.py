from __future__ import annotations

from copy import deepcopy
from typing import Any

from src import host
from src.config import RunConfig

LANDING_NOT_EVALUATED_BLOCKER = "landing_not_evaluated"
RETURN_HOME_REQUIRED_BLOCKER = "return_home_required_before_landing_not_satisfied"
SIMULATION_LANDING_REQUIRED_BLOCKER = "simulation_landing_acceptance_not_passed"
SUPPORTED_LANDING_POLICIES = ("land_in_place", "return_home_then_land")


def _landing_policy(config: RunConfig, task_name: str) -> str:
    policy = config.orchestration.landing.policy_for_task(task_name)
    if policy in SUPPORTED_LANDING_POLICIES:
        return policy
    return config.orchestration.landing.default_policy


def _acceptance_stage(config: RunConfig) -> str:
    return "real" if host._runtime_mode_name(config) == "real" else "simulation"


def _landing_config_summary(config: RunConfig, task_name: str) -> dict[str, Any]:
    landing = config.orchestration.landing
    policy = _landing_policy(config, task_name)
    return {
        "enabled": landing.enabled,
        "policy": policy,
        "landing_status_topic": landing.landing_status_topic,
        "landing_intent_topic": landing.landing_intent_topic,
        "home_source": landing.home_source,
        "home_radius_m": landing.home_radius_m,
        "pre_land_hold_sec": landing.pre_land_hold_sec,
        "max_return_home_duration_sec": landing.max_return_home_duration_sec,
        "max_landing_duration_sec": landing.max_landing_duration_sec,
        "max_descent_rate_mps": landing.max_descent_rate_mps,
        "touchdown_altitude_m": landing.touchdown_altitude_m,
        "touchdown_vertical_speed_mps": landing.touchdown_vertical_speed_mps,
        "require_disarm": landing.require_disarm,
        "require_motors_safe": landing.require_motors_safe,
        "uses_gazebo_truth_as_input": landing.uses_gazebo_truth_as_input,
    }


def _default_landing_summary(config: RunConfig, task_name: str) -> dict[str, Any]:
    landing = config.orchestration.landing
    policy = _landing_policy(config, task_name)
    return {
        "ok": False,
        "claim": "not_evaluated",
        "policy": policy,
        "state": "not_started",
        "return_home": {
            "required": policy == "return_home_then_land",
            "ok": False,
            "state": "not_started",
            "distance_to_home_m": None,
            "duration_sec": None,
        },
        "land_command_accepted": False,
        "landing_duration_sec": None,
        "landed_confirmed": False,
        "touchdown_confirmed": False,
        "disarmed": False,
        "motors_safe": False,
        "require_disarm": landing.require_disarm,
        "require_motors_safe": landing.require_motors_safe,
        "uses_gazebo_truth_as_input": landing.uses_gazebo_truth_as_input,
        "blockers": [LANDING_NOT_EVALUATED_BLOCKER],
    }


def build_landing_acceptance_summary(
    config: RunConfig,
    *,
    task_name: str,
    landing: dict[str, Any] | None = None,
    simulation_summary: dict[str, Any] | None = None,
) -> dict[str, Any]:
    stage = _acceptance_stage(config)
    landing_summary = deepcopy(landing) if landing is not None else _default_landing_summary(config, task_name)
    landing_summary.setdefault("policy", _landing_policy(config, task_name))
    landing_summary.setdefault("claim", "evaluated" if landing_summary.get("ok") else "not_evaluated")
    landing_summary.setdefault("uses_gazebo_truth_as_input", config.orchestration.landing.uses_gazebo_truth_as_input)
    landing_summary.setdefault("blockers", [])

    simulation_ok = bool(landing_summary.get("ok")) if stage == "simulation" else bool(
        (simulation_summary or {}).get("ok")
    )
    simulation_blockers = [] if simulation_ok else [SIMULATION_LANDING_REQUIRED_BLOCKER]
    real_ok = bool(landing_summary.get("ok")) and simulation_ok if stage == "real" else False

    real_state = "not_started" if stage == "simulation" else ("evaluated" if real_ok else "blocked")
    if stage == "real" and not simulation_ok:
        landing_summary.setdefault("blockers", []).append(SIMULATION_LANDING_REQUIRED_BLOCKER)
        landing_summary["ok"] = False

    return {
        "acceptance_stage": stage,
        "landing_claim": "evaluated" if landing_summary.get("ok") else "not_evaluated",
        "simulation_landing_claim": "evaluated" if simulation_ok else "not_evaluated",
        "real_landing_claim": "evaluated" if real_ok else "not_evaluated",
        "landing_config": _landing_config_summary(config, task_name),
        "landing": landing_summary,
        "simulation_landing_acceptance": {
            "ok": simulation_ok,
            "runtime_mode": "simulation",
            "blockers": simulation_blockers,
        },
        "real_landing_acceptance": {
            "ok": real_ok,
            "runtime_mode": "real",
            "state": real_state,
            "blockers": [] if (stage == "simulation" or real_ok) else landing_summary.get("blockers", []),
        },
    }


def apply_landing_gate(
    summary: dict[str, Any],
    config: RunConfig,
    *,
    task_name: str,
    landing: dict[str, Any] | None = None,
    simulation_summary: dict[str, Any] | None = None,
) -> dict[str, Any]:
    gated = dict(summary)
    landing_summary = build_landing_acceptance_summary(
        config,
        task_name=task_name,
        landing=landing,
        simulation_summary=simulation_summary,
    )
    gated.update(landing_summary)
    blockers = list(gated.get("blockers", []))
    landing_block = landing_summary["landing"]
    if not landing_block.get("ok"):
        blockers.extend(landing_block.get("blockers", []))
    return_home = landing_block.get("return_home", {})
    if (
        landing_block.get("policy") == "return_home_then_land" or return_home.get("required") is True
    ) and not return_home.get("ok"):
        blockers.append(RETURN_HOME_REQUIRED_BLOCKER)
    if landing_summary["acceptance_stage"] == "real" and not landing_summary["simulation_landing_acceptance"].get("ok"):
        blockers.append(SIMULATION_LANDING_REQUIRED_BLOCKER)
    if config.orchestration.landing.uses_gazebo_truth_as_input:
        blockers.append("landing_must_not_use_gazebo_truth_as_input")
    gated["blockers"] = sorted(set(str(blocker) for blocker in blockers))
    gated["blocked"] = bool(gated["blockers"])
    gated["ok"] = bool(gated.get("ok")) and not gated["blocked"]
    return gated
