from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from navlab.sim.companion.nodes.obstacle_mission import (
    DEFAULT_ORIGIN_ALT_M,
    DEFAULT_ORIGIN_LAT_DEG,
    DEFAULT_ORIGIN_LON_DEG,
    command_arm,
    command_takeoff,
    mode_number,
    send_gcs_heartbeat,
    send_local_position_yaw_setpoint,
    set_arming_check,
    set_ekf_origin,
    set_home_position,
    set_mode,
)
from navlab.sim.companion.runtime.status import DEFAULT_SIM_LOG_TOPIC, encode_sim_log

os.environ.setdefault("MAVLINK20", "1")
HOVER_DURATION_TOLERANCE_SEC = 0.25
DEFAULT_LANDING_DESCENT_RATE_MPS = 0.12
DEFAULT_LANDING_LAND_COMMAND_ALTITUDE_M = 0.18
DEFAULT_LANDING_MAX_POST_TOUCHDOWN_BOUNCE_M = 0.04
DEFAULT_LANDING_RANGEFINDER_OUTLIER_MIN_M = 0.45
DEFAULT_LANDING_RANGEFINDER_OUTLIER_MAX_NEIGHBOR_DT_SEC = 1.0
DEFAULT_LANDING_RANGEFINDER_MAX_ABOVE_LOCAL_M = 0.75
DEFAULT_LANDING_RANGEFINDER_LOCAL_CROSSCHECK_MAX_HEIGHT_M = 1.25
DEFAULT_LANDING_RANGEFINDER_MAX_RATE_MPS = 0.5
DEFAULT_LANDING_SLOWDOWN_ALTITUDE_M = 0.60
DEFAULT_LANDING_NEAR_GROUND_DESCENT_RATE_MPS = 0.01
LANDING_POLICY_AP_LAND_MODE_AFTER_HOVER = "ap_land_mode_after_hover"
LANDING_POLICY_GUIDED_DESCENT = "guided_descent"
LANDING_POLICY_LAND_IN_PLACE = "land_in_place"
ARDUCOPTER_LAND_MODE_NUMBER = 9
FCU_LAND_PARAM_NAMES = (
    "LAND_SPEED",
    "LAND_SPD_MS",
    "LAND_SPEED_HIGH",
    "LAND_SPD_HIGH_MS",
    "LAND_ALT_LOW_M",
    "SURFTRAK_TC",
    "SURFTRAK_GLDST",
    "SURFTRAK_GLSAM",
    "EK3_SRC1_POSZ",
    "EK3_RNG_USE_HGT",
)
LandingDescentSample = (
    tuple[float, float | None, float | None, float | None] | tuple[float, float | None, float | None, float | None, str]
)


@dataclass(frozen=True, slots=True)
class HoverInputs:
    external_nav_ready: bool
    mavlink_external_nav_ready: bool
    fcu_local_position_ready: bool
    imu_ready: bool
    slam_quality_good: bool
    slam_quality: str
    ready_elapsed_sec: float
    current_yaw_rad: float | None
    expected_mode_seen: bool
    armed_seen: bool
    airborne_seen: bool
    takeoff_ack_ok: bool
    airborne_elapsed_sec: float
    hover_elapsed_sec: float
    current_x: float | None
    current_y: float | None
    current_z_ned: float | None
    current_height_m: float | None
    external_nav_height_m: float | None
    rangefinder_relative_height_m: float | None
    target_z_ned: float | None
    slam_quality_loss_duration_sec: float = 0.0
    external_nav_loss_duration_sec: float = 0.0
    mavlink_external_nav_loss_duration_sec: float = 0.0
    fcu_local_position_loss_duration_sec: float = 0.0


@dataclass(frozen=True, slots=True)
class HoverRequirements:
    require_external_nav: bool = True
    require_fcu_external_nav: bool = True
    require_imu_status: bool = True
    external_nav_loss_grace_sec: float = 1.0


@dataclass(frozen=True, slots=True)
class HoverDecision:
    phase: str
    reason: str
    should_set_guided: bool = False
    should_arm: bool = False
    should_takeoff: bool = False
    terminal: bool = False


@dataclass(frozen=True, slots=True)
class HoverDriftSummary:
    sample_count: int
    duration_sec: float
    horizontal_span_m: float
    z_span_m: float
    horizontal_drift_m: float
    z_drift_m: float

    @property
    def ok(self) -> bool:
        return self.sample_count >= 2


HOVER_PHASE_TO_MISSION_FSM_STATE = {
    "wait_ready": "S1 wait_nav_ready",
    "guided": "S2 set_guided",
    "arm": "S3 arm",
    "takeoff": "S4 takeoff",
    "hover_settle": "S5 hover_settle",
    "hover_hold": "S6 hover_hold",
    "complete": "S7 pre_land_hold",
    "abort": "S_abort",
}

LANDING_STATE_TO_MISSION_FSM_STATE = {
    "not_started": "S7 pre_land_hold",
    "task_body_complete": "S7 pre_land_hold",
    "pre_land_hold": "S7 pre_land_hold",
    "guided_descent": "legacy_guided_descent_diagnostic",
    "land_command_sent": "S8 command_land",
    "descent_monitoring": "S9 land_mode_monitor",
    "touchdown_candidate": "S10 touchdown_monitor",
    "disarm_requested": "S11 disarm_monitor",
    "landing_complete": "S12 landing_complete",
}


def mission_fsm_state_for_hover_phase(phase: str) -> str:
    return HOVER_PHASE_TO_MISSION_FSM_STATE.get(phase, "S_abort")


def mission_fsm_state_for_landing_state(landing_state: str) -> str:
    return LANDING_STATE_TO_MISSION_FSM_STATE.get(landing_state, "S_abort")


def landing_policy_uses_ap_land_mode(policy: str) -> bool:
    return policy == LANDING_POLICY_AP_LAND_MODE_AFTER_HOVER


def should_use_guided_descent_before_land(
    *,
    landing_policy: str,
    land_command_sent: bool,
    touchdown_ready: bool,
) -> bool:
    return not landing_policy_uses_ap_land_mode(landing_policy) and not land_command_sent and not touchdown_ready


def should_command_land_this_tick(
    *,
    landing_policy: str,
    land_command_sent: bool,
    touchdown_ready: bool,
    command_due: bool,
) -> bool:
    if land_command_sent:
        return command_due
    if landing_policy_uses_ap_land_mode(landing_policy):
        return True
    return touchdown_ready


def should_send_disarm_after_touchdown(
    *,
    touchdown_confirmed: bool,
    disarmed: bool,
    require_disarm: bool,
    touchdown_confirmed_elapsed_sec: float | None,
    force_disarm_grace_sec: float,
) -> bool:
    if not require_disarm or disarmed or not touchdown_confirmed:
        return False
    if touchdown_confirmed_elapsed_sec is None:
        return False
    return touchdown_confirmed_elapsed_sec >= max(0.0, force_disarm_grace_sec)


def mavlink_param_id_to_str(value: object) -> str:
    if isinstance(value, bytes):
        return value.split(b"\x00", 1)[0].decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def fcu_land_params_report(params: dict[str, float]) -> dict[str, object]:
    return {
        "requested": list(FCU_LAND_PARAM_NAMES),
        "values": {name: params[name] for name in FCU_LAND_PARAM_NAMES if name in params},
        "missing": [name for name in FCU_LAND_PARAM_NAMES if name not in params],
        "ekf_posz_is_rangefinder": params.get("EK3_SRC1_POSZ") == 10,
        "ekf_rng_use_hgt_enabled": params.get("EK3_RNG_USE_HGT") not in (None, -1),
    }


def landing_controller_for_state(
    landing_state: str,
    *,
    landing_policy: str = LANDING_POLICY_GUIDED_DESCENT,
) -> str:
    if landing_state == "not_started":
        return "not_started"
    if landing_state in {"task_body_complete", "pre_land_hold"}:
        return "pending"
    if landing_policy_uses_ap_land_mode(landing_policy):
        return "ap_land_mode"
    return "guided_descent"


def landing_descent_profile_enforced(landing_policy: str) -> bool:
    return not landing_policy_uses_ap_land_mode(landing_policy)


def landing_handoff_confirmed(
    *,
    landing_policy: str,
    land_command_sent: bool,
    land_command_accepted: bool,
    land_mode_seen: bool,
) -> bool:
    if not landing_policy_uses_ap_land_mode(landing_policy):
        return True
    return bool(land_command_sent and (land_command_accepted or land_mode_seen))


def landing_acceptance_ok(
    *,
    landing_policy: str,
    land_command_sent: bool,
    land_command_accepted: bool,
    land_mode_seen: bool,
    touchdown_confirmed: bool,
    disarmed: bool,
    motors_safe: bool,
    require_disarm: bool,
    require_motors_safe: bool,
    descent_profile_ok: bool,
) -> bool:
    handoff_ok = landing_handoff_confirmed(
        landing_policy=landing_policy,
        land_command_sent=land_command_sent,
        land_command_accepted=land_command_accepted,
        land_mode_seen=land_mode_seen,
    )
    if not handoff_ok:
        return False
    descent_ok = True if not landing_descent_profile_enforced(landing_policy) else descent_profile_ok
    return bool(
        touchdown_confirmed
        and (disarmed if require_disarm else True)
        and (motors_safe if require_motors_safe else True)
        and descent_ok
    )


class MissionFsmRecorder:
    def __init__(
        self,
        *,
        started_at_monotonic: float,
        initial_state: str = "S0 wait_runtime",
        reason: str = "controller_started",
    ) -> None:
        self._started_at_monotonic = started_at_monotonic
        self._state = initial_state
        self._entered_at_sec = 0.0
        self._reason = reason
        self._guard: str | None = None
        self._blocker: str | None = None
        self._history: list[dict[str, object]] = []

    @property
    def state(self) -> str:
        return self._state

    @property
    def entered_at_sec(self) -> float:
        return self._entered_at_sec

    @property
    def last_transition_reason(self) -> str:
        return self._reason

    @property
    def blocker(self) -> str | None:
        return self._blocker

    def transition(
        self,
        *,
        now_monotonic: float,
        state: str,
        reason: str,
        guard: str | None = None,
        blocker: str | None = None,
    ) -> None:
        now_sec = max(0.0, now_monotonic - self._started_at_monotonic)
        if state == self._state:
            self._reason = reason
            self._guard = guard
            self._blocker = blocker
            return
        self._history.append(
            {
                "state": self._state,
                "entered_at_sec": self._entered_at_sec,
                "exited_at_sec": now_sec,
                "duration_sec": max(0.0, now_sec - self._entered_at_sec),
                "reason": self._reason,
                "guard": self._guard,
                "blocker": self._blocker,
            }
        )
        self._history = self._history[-80:]
        self._state = state
        self._entered_at_sec = now_sec
        self._reason = reason
        self._guard = guard
        self._blocker = blocker

    def snapshot(self, *, now_monotonic: float) -> dict[str, object]:
        now_sec = max(0.0, now_monotonic - self._started_at_monotonic)
        current = {
            "state": self._state,
            "entered_at_sec": self._entered_at_sec,
            "exited_at_sec": None,
            "duration_sec": max(0.0, now_sec - self._entered_at_sec),
            "reason": self._reason,
            "guard": self._guard,
            "blocker": self._blocker,
        }
        return {
            "state": self._state,
            "state_entered_at_sec": self._entered_at_sec,
            "last_transition_reason": self._reason,
            "blocker": self._blocker,
            "history": [*self._history, current],
        }


def classify_hover_drift(
    drift: HoverDriftSummary,
    *,
    max_horizontal_drift_m: float,
) -> str:
    if not drift.ok or max_horizontal_drift_m <= 0 or not math.isfinite(drift.horizontal_drift_m):
        return "unusable"
    ratio = drift.horizontal_drift_m / max_horizontal_drift_m
    if ratio <= 0.25:
        return "tight"
    if ratio <= 0.50:
        return "nominal"
    if ratio <= 1.0:
        return "marginal"
    return "unstable"


def summarize_hover_altitude_crosscheck(
    samples: list[dict[str, float | None]],
    *,
    target_alt_m: float,
    tolerance_m: float,
) -> dict[str, object]:
    latest = samples[-1] if samples else {}
    fcu_height = latest.get("fcu_local_height_m")
    external_height = latest.get("external_nav_height_m")
    rangefinder_height = latest.get("rangefinder_relative_height_m")

    def abs_diff(left: float | None, right: float | None) -> float | None:
        if left is None or right is None:
            return None
        return abs(float(left) - float(right))

    def target_error(value: float | None) -> float | None:
        return abs_diff(value, target_alt_m)

    diffs = {
        "fcu_vs_external_abs_m": abs_diff(fcu_height, external_height),
        "fcu_vs_rangefinder_abs_m": abs_diff(fcu_height, rangefinder_height),
        "external_vs_rangefinder_abs_m": abs_diff(external_height, rangefinder_height),
        "fcu_target_error_m": target_error(fcu_height),
        "external_target_error_m": target_error(external_height),
        "rangefinder_target_error_m": target_error(rangefinder_height),
    }
    missing = [
        name
        for name, value in {
            "fcu_local_height_m": fcu_height,
            "external_nav_height_m": external_height,
            "rangefinder_relative_height_m": rangefinder_height,
        }.items()
        if value is None
    ]
    over_tolerance = [name for name, value in diffs.items() if value is not None and float(value) > tolerance_m]
    ok = len(samples) >= 2 and not missing and not over_tolerance
    return {
        "ok": ok,
        "sample_count": len(samples),
        "target_alt_m": target_alt_m,
        "tolerance_m": tolerance_m,
        "sources": {
            "fcu_local_z_ned": latest.get("fcu_local_z_ned"),
            "fcu_local_height_m": fcu_height,
            "external_nav_height_m": external_height,
            "rangefinder_range_m": latest.get("rangefinder_range_m"),
            "rangefinder_relative_height_m": rangefinder_height,
        },
        "diffs": diffs,
        "missing_sources": missing,
        "over_tolerance": over_tolerance,
    }


def decide_hover(
    inputs: HoverInputs,
    *,
    requirements: HoverRequirements | None = None,
    preflight_ready_sec: float,
    hover_settle_sec: float,
    hover_hold_sec: float,
    takeoff_alt_m: float,
    hover_altitude_tolerance_m: float,
) -> HoverDecision:
    requirements = requirements or HoverRequirements()
    if inputs.airborne_seen and not inputs.armed_seen:
        return HoverDecision("abort", "disarmed_after_airborne", terminal=True)
    if (
        inputs.airborne_seen
        and requirements.require_external_nav
        and not inputs.slam_quality_good
        and inputs.slam_quality_loss_duration_sec >= requirements.external_nav_loss_grace_sec
    ):
        return HoverDecision("abort", "slam_quality_lost_after_airborne", terminal=True)
    if (
        inputs.airborne_seen
        and requirements.require_external_nav
        and not inputs.external_nav_ready
        and inputs.external_nav_loss_duration_sec >= requirements.external_nav_loss_grace_sec
    ):
        return HoverDecision("abort", "external_nav_lost_after_airborne", terminal=True)
    if inputs.airborne_seen and requirements.require_fcu_external_nav:
        if (
            not inputs.mavlink_external_nav_ready
            and inputs.mavlink_external_nav_loss_duration_sec >= requirements.external_nav_loss_grace_sec
        ):
            return HoverDecision("abort", "mavlink_external_nav_lost_after_airborne", terminal=True)
        if (
            not inputs.fcu_local_position_ready
            and inputs.fcu_local_position_loss_duration_sec >= requirements.external_nav_loss_grace_sec
        ):
            return HoverDecision("abort", "fcu_local_position_lost_after_airborne", terminal=True)
    if not inputs.airborne_seen and requirements.require_external_nav and not inputs.slam_quality_good:
        return HoverDecision("wait_ready", "waiting_for_slam_quality")
    if not inputs.airborne_seen and requirements.require_external_nav and not inputs.external_nav_ready:
        return HoverDecision("wait_ready", "waiting_for_external_nav_and_imu")
    if (
        not inputs.airborne_seen
        and requirements.require_fcu_external_nav
        and (not inputs.mavlink_external_nav_ready or not inputs.fcu_local_position_ready)
    ):
        return HoverDecision("wait_ready", "waiting_for_fcu_external_nav")
    if not inputs.airborne_seen and requirements.require_imu_status and not inputs.imu_ready:
        return HoverDecision("wait_ready", "waiting_for_external_nav_and_imu")
    if inputs.current_yaw_rad is None:
        return HoverDecision("wait_ready", "waiting_for_fcu_attitude")
    if inputs.ready_elapsed_sec < preflight_ready_sec:
        return HoverDecision("wait_ready", "waiting_for_stable_external_nav_and_imu")
    if inputs.airborne_seen and not inputs.expected_mode_seen:
        return HoverDecision("abort", "guided_mode_lost_after_airborne", terminal=True)
    if not inputs.expected_mode_seen:
        return HoverDecision("guided", "setting_guided", should_set_guided=True)
    if not inputs.armed_seen:
        return HoverDecision("arm", "arming_vehicle", should_arm=True)
    if not inputs.airborne_seen:
        return HoverDecision("takeoff", "taking_off", should_takeoff=True)
    if inputs.airborne_elapsed_sec < hover_settle_sec:
        return HoverDecision("hover_settle", "settling_before_position_hold")
    independent_height_ok = independent_takeoff_height_reached(
        inputs,
        takeoff_alt_m=takeoff_alt_m,
        hover_altitude_tolerance_m=hover_altitude_tolerance_m,
    )
    if not inputs.takeoff_ack_ok:
        if not independent_height_ok:
            return HoverDecision("hover_settle", "waiting_for_independent_takeoff_height")
        if inputs.hover_elapsed_sec < hover_hold_sec:
            return HoverDecision("hover_hold", "holding_position_with_independent_height")
        return HoverDecision("complete", "hover_complete", terminal=True)
    if not independent_height_ok:
        return HoverDecision("hover_settle", "waiting_for_independent_takeoff_height")
    target_z_ned = inputs.target_z_ned if inputs.target_z_ned is not None else -takeoff_alt_m
    if inputs.current_z_ned is not None:
        if abs(inputs.current_z_ned - target_z_ned) > hover_altitude_tolerance_m:
            return HoverDecision("hover_settle", "settling_until_target_altitude")
    elif inputs.current_height_m is not None:
        if abs(inputs.current_height_m - takeoff_alt_m) > hover_altitude_tolerance_m:
            return HoverDecision("hover_settle", "settling_until_target_altitude")
    else:
        return HoverDecision("hover_settle", "settling_until_target_altitude")
    if inputs.hover_elapsed_sec < hover_hold_sec:
        return HoverDecision("hover_hold", "holding_position")
    return HoverDecision("complete", "hover_complete", terminal=True)


def should_fail_fast_wait_ready(
    inputs: HoverInputs,
    decision: HoverDecision,
    *,
    mission_elapsed_sec: float,
    max_wait_ready_sec: float,
) -> bool:
    return (
        max_wait_ready_sec > 0.0
        and decision.phase == "wait_ready"
        and not inputs.armed_seen
        and not inputs.airborne_seen
        and mission_elapsed_sec >= max_wait_ready_sec
    )


def summarize_hover_drift(samples: list[tuple[float, float, float, float]]) -> HoverDriftSummary:
    if len(samples) < 2:
        return HoverDriftSummary(
            sample_count=len(samples),
            duration_sec=0.0,
            horizontal_span_m=math.inf,
            z_span_m=math.inf,
            horizontal_drift_m=math.inf,
            z_drift_m=math.inf,
        )
    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    zs = [sample[3] for sample in samples]
    start = samples[0]
    end = samples[-1]
    return HoverDriftSummary(
        sample_count=len(samples),
        duration_sec=max(0.0, end[0] - start[0]),
        horizontal_span_m=math.hypot(max(xs) - min(xs), max(ys) - min(ys)),
        z_span_m=max(zs) - min(zs),
        horizontal_drift_m=math.hypot(end[1] - start[1], end[2] - start[2]),
        z_drift_m=abs(end[3] - start[3]),
    )


def json_safe_number(value: float | int | None) -> float | int | None:
    if value is None:
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def hold_axis_or_current(hold_value: float | None, current_value: float | None) -> float:
    if hold_value is not None:
        return hold_value
    if current_value is not None:
        return current_value
    return 0.0


def hover_hold_setpoint_axes(
    *,
    hold_x: float | None,
    hold_y: float | None,
    current_x: float | None,
    current_y: float | None,
    current_z: float | None,
    target_z_ned: float | None,
) -> tuple[float, float, float]:
    return (
        hold_axis_or_current(hold_x, current_x),
        hold_axis_or_current(hold_y, current_y),
        hold_axis_or_current(target_z_ned, current_z),
    )


def landing_descent_target_z_ned(
    *,
    start_z_ned: float,
    ground_z_ned: float,
    elapsed_sec: float,
    descent_rate_mps: float,
    current_z_ned: float | None = None,
    setpoint_lookahead_sec: float | None = None,
) -> float:
    if descent_rate_mps <= 0 or not math.isfinite(descent_rate_mps):
        raise ValueError("descent_rate_mps must be positive and finite")
    target = start_z_ned + max(0.0, elapsed_sec) * descent_rate_mps
    target = min(target, ground_z_ned)
    if (
        current_z_ned is not None
        and math.isfinite(current_z_ned)
        and setpoint_lookahead_sec is not None
        and math.isfinite(setpoint_lookahead_sec)
        and setpoint_lookahead_sec > 0.0
    ):
        max_step_ahead = descent_rate_mps * setpoint_lookahead_sec
        target = min(target, min(current_z_ned + max_step_ahead, ground_z_ned))
    return target


def landing_effective_descent_rate_mps(
    *,
    nominal_descent_rate_mps: float,
    rangefinder_relative_height_m: float | None,
    slowdown_altitude_m: float,
    near_ground_descent_rate_mps: float,
) -> float:
    if nominal_descent_rate_mps <= 0 or not math.isfinite(nominal_descent_rate_mps):
        raise ValueError("nominal_descent_rate_mps must be positive and finite")
    effective_rate = nominal_descent_rate_mps
    range_height = _finite_float(rangefinder_relative_height_m)
    if (
        range_height is not None
        and range_height <= slowdown_altitude_m
        and near_ground_descent_rate_mps > 0.0
        and math.isfinite(near_ground_descent_rate_mps)
    ):
        effective_rate = min(effective_rate, near_ground_descent_rate_mps)
    return effective_rate


def landing_descent_height_m(z_ned: float | None, ground_z_ned: float | None) -> float | None:
    if z_ned is None or ground_z_ned is None:
        return None
    height = float(ground_z_ned) - float(z_ned)
    return max(0.0, height) if math.isfinite(height) else None


def _finite_float(value: float | None) -> float | None:
    if value is None:
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def landing_descent_evidence_height_and_source_m(
    *,
    current_range_m: float | None,
    ground_range_m: float | None,
    current_z_ned: float | None,
    ground_z_ned: float | None,
    rangefinder_max_above_local_m: float = DEFAULT_LANDING_RANGEFINDER_MAX_ABOVE_LOCAL_M,
    rangefinder_local_crosscheck_max_height_m: float = DEFAULT_LANDING_RANGEFINDER_LOCAL_CROSSCHECK_MAX_HEIGHT_M,
) -> tuple[float | None, str]:
    local_height = landing_descent_height_m(current_z_ned, ground_z_ned)
    current_range = _finite_float(current_range_m)
    ground_range = _finite_float(ground_range_m)
    if current_range is not None and ground_range is not None and current_range >= 0.0 and ground_range >= 0.0:
        range_height = max(0.0, current_range - ground_range)
        if (
            local_height is not None
            and local_height <= rangefinder_local_crosscheck_max_height_m
            and range_height > local_height + rangefinder_max_above_local_m
        ):
            return local_height, "fcu_local_z_after_rangefinder_high_outlier"
        return range_height, "rangefinder_relative_height"
    return local_height, "fcu_local_z_fallback"


def landing_descent_evidence_height_m(
    *,
    current_range_m: float | None,
    ground_range_m: float | None,
    current_z_ned: float | None,
    ground_z_ned: float | None,
) -> float | None:
    height, _source = landing_descent_evidence_height_and_source_m(
        current_range_m=current_range_m,
        ground_range_m=ground_range_m,
        current_z_ned=current_z_ned,
        ground_z_ned=ground_z_ned,
    )
    return height


def landing_touchdown_candidate(
    *,
    landed_state_on_ground: bool,
    current_range_m: float | None,
    current_z_ned: float | None,
    current_vz_mps: float | None,
    touchdown_altitude_m: float,
    touchdown_vertical_speed_mps: float,
) -> bool:
    if landed_state_on_ground:
        return True
    vz_ok = current_vz_mps is None or abs(current_vz_mps) <= touchdown_vertical_speed_mps
    if current_range_m is not None and math.isfinite(current_range_m):
        return bool(current_range_m <= touchdown_altitude_m and vz_ok)
    z_ok = current_z_ned is not None and current_z_ned >= -touchdown_altitude_m
    return bool(z_ok and vz_ok)


def _landing_descent_sample_fields(
    sample: LandingDescentSample,
) -> tuple[float, float | None, float | None, float | None, str]:
    t, height, range_m, vz = sample[:4]
    if len(sample) >= 5:
        source = str(sample[4])
    else:
        range_value = _finite_float(range_m)
        source = (
            "rangefinder_relative_height" if range_value is not None and range_value >= 0.0 else "fcu_local_z_fallback"
        )
    return float(t), height, range_m, vz, source


def summarize_landing_descent_profile(
    samples: Sequence[LandingDescentSample],
    *,
    max_descent_rate_mps: float,
    touchdown_altitude_m: float,
    max_post_touchdown_bounce_m: float = DEFAULT_LANDING_MAX_POST_TOUCHDOWN_BOUNCE_M,
    rangefinder_outlier_min_m: float = DEFAULT_LANDING_RANGEFINDER_OUTLIER_MIN_M,
    rangefinder_outlier_max_neighbor_dt_sec: float = DEFAULT_LANDING_RANGEFINDER_OUTLIER_MAX_NEIGHBOR_DT_SEC,
    rangefinder_max_rate_mps: float = DEFAULT_LANDING_RANGEFINDER_MAX_RATE_MPS,
) -> dict[str, object]:
    parsed_samples = [_landing_descent_sample_fields(sample) for sample in samples]
    source_counts: dict[str, int] = {}
    for _t, _height, _range_m, _vz, source in parsed_samples:
        source_counts[source] = source_counts.get(source, 0) + 1
    raw_heights = [
        (idx, float(t), float(height), None if _range_m is None else float(_range_m), source)
        for idx, (t, height, _range_m, _vz, source) in enumerate(parsed_samples)
        if height is not None and math.isfinite(float(height))
    ]
    outlier_indices: set[int] = set()
    outlier_details: list[dict[str, float | int]] = []
    outlier_jump_m = max(0.0, rangefinder_outlier_min_m)
    for prev_sample, sample, next_sample in zip(raw_heights, raw_heights[1:], raw_heights[2:]):
        prev_idx, prev_t, prev_height, _prev_range, _prev_source = prev_sample
        idx, t, height, range_m, source = sample
        next_idx, next_t, next_height, _next_range, _next_source = next_sample
        if source != "rangefinder_relative_height":
            continue
        if range_m is None or not math.isfinite(range_m):
            continue
        if t - prev_t > rangefinder_outlier_max_neighbor_dt_sec:
            continue
        if next_t - t > rangefinder_outlier_max_neighbor_dt_sec:
            continue
        # 只过滤孤立的向上尖峰；连续真实快速下降仍会保留并触发 speed gate。
        if (
            height - prev_height > outlier_jump_m
            and height - next_height > outlier_jump_m
            and abs(prev_height - next_height) <= outlier_jump_m
        ):
            outlier_indices.add(idx)
            outlier_details.append(
                {
                    "sample_index": idx,
                    "time_sec": t,
                    "height_m": height,
                    "raw_range_m": range_m,
                    "prev_height_m": prev_height,
                    "next_height_m": next_height,
                    "prev_sample_index": prev_idx,
                    "next_sample_index": next_idx,
                }
            )
    rate_outlier_details: list[dict[str, float | int | str]] = []
    while True:
        current_heights = [sample for sample in raw_heights if sample[0] not in outlier_indices]
        rejected: dict[str, float | int | str] | None = None
        for prev_sample, sample in zip(current_heights, current_heights[1:]):
            prev_idx, prev_t, prev_height, _prev_range, prev_source = prev_sample
            idx, t, height, _range_m, source = sample
            if prev_source != "rangefinder_relative_height" or source != "rangefinder_relative_height":
                continue
            dt = t - prev_t
            if dt <= 0:
                continue
            rate_mps = abs(height - prev_height) / dt
            if rate_mps <= rangefinder_max_rate_mps:
                continue
            rejected_idx, rejected_t, rejected_height = (prev_idx, prev_t, prev_height)
            kept_idx, kept_t, kept_height = (idx, t, height)
            if height > prev_height:
                rejected_idx, rejected_t, rejected_height = (idx, t, height)
                kept_idx, kept_t, kept_height = (prev_idx, prev_t, prev_height)
            rejected = {
                "sample_index": rejected_idx,
                "time_sec": rejected_t,
                "height_m": rejected_height,
                "neighbor_sample_index": kept_idx,
                "neighbor_time_sec": kept_t,
                "neighbor_height_m": kept_height,
                "abs_rate_mps": rate_mps,
                "height_source": source,
            }
            break
        if rejected is None:
            break
        outlier_indices.add(int(rejected["sample_index"]))
        rate_outlier_details.append(rejected)
    valid_height_records = [
        (t, height, source) for idx, t, height, _range_m, source in raw_heights if idx not in outlier_indices
    ]
    valid_heights = [(t, height) for t, height, _source in valid_height_records]
    valid_vz = [
        (t, None if height is None else float(height), float(vz))
        for idx, (t, height, _range_m, vz, _source) in enumerate(parsed_samples)
        if idx not in outlier_indices and vz is not None and math.isfinite(float(vz))
    ]
    touchdown_vz = [vz for _t, height, vz in valid_vz if height is not None and height <= touchdown_altitude_m]
    height_rates: list[tuple[float, float, float, float, float, str]] = []
    for (prev_t, prev_height, prev_source), (t, height, source) in zip(valid_height_records, valid_height_records[1:]):
        dt = t - prev_t
        if dt <= 0:
            continue
        if prev_source == source and prev_height > touchdown_altitude_m:
            height_rates.append((max(0.0, (prev_height - height) / dt), prev_t, t, prev_height, height, source))
    high_height_rate_windows = [
        (prev_t, t) for speed, prev_t, t, _prev_height, _height, _source in height_rates if speed > max_descent_rate_mps
    ]
    vertical_velocity_outliers: list[dict[str, float | str]] = []
    controlled_speeds = []
    for t, height, vz in valid_vz:
        if height is not None and height <= touchdown_altitude_m:
            continue
        if vz <= max_descent_rate_mps:
            controlled_speeds.append(
                {"source": "vertical_velocity", "speed_mps": vz, "time_sec": t, "height_m": height}
            )
            continue
        if not valid_heights or any(start - 0.10 <= t <= end + 0.10 for start, end in high_height_rate_windows):
            controlled_speeds.append(
                {"source": "vertical_velocity", "speed_mps": vz, "time_sec": t, "height_m": height}
            )
            continue
        vertical_velocity_outliers.append(
            {
                "source": "vertical_velocity_uncorroborated",
                "speed_mps": vz,
                "time_sec": t,
                "height_m": height,
            }
        )
    controlled_speeds.extend(
        {
            "source": "height_rate",
            "speed_mps": speed,
            "from_time_sec": prev_t,
            "to_time_sec": t,
            "from_height_m": prev_height,
            "to_height_m": height,
            "height_source": source,
        }
        for speed, prev_t, t, prev_height, height, source in height_rates
    )
    max_speed_entry = max(controlled_speeds, key=lambda entry: float(entry["speed_mps"])) if controlled_speeds else None
    max_downward_speed = float(max_speed_entry["speed_mps"]) if max_speed_entry is not None else None
    max_touchdown_downward_speed = max(touchdown_vz) if touchdown_vz else None
    first_touchdown = next(
        ((t, h, source) for t, h, source in valid_height_records if h <= touchdown_altitude_m),
        None,
    )
    post_touchdown_max_height = None
    post_touchdown_bounce = None
    if first_touchdown is not None:
        first_t, first_h, first_source = first_touchdown
        post_heights = [height for t, height, source in valid_height_records if t >= first_t and source == first_source]
        if post_heights:
            post_touchdown_max_height = max(post_heights)
            post_touchdown_bounce = max(0.0, post_touchdown_max_height - first_h)
    speed_ok = max_downward_speed is not None and max_downward_speed <= max_descent_rate_mps
    bounce_ok = post_touchdown_bounce is None or post_touchdown_bounce <= max_post_touchdown_bounce_m
    return {
        "ok": len(valid_heights) >= 2 and speed_ok and bounce_ok,
        "sample_count": len(samples),
        "raw_height_sample_count": len(raw_heights),
        "height_sample_count": len(valid_heights),
        "filtered_height_sample_count": len(outlier_indices),
        "rangefinder_raw_sample_count": sum(
            1
            for _t, _height, range_m, _vz, _source in parsed_samples
            if range_m is not None and math.isfinite(float(range_m)) and float(range_m) >= 0.0
        ),
        "fallback_height_sample_count": sum(
            1
            for _t, height, _range_m, _vz, source in parsed_samples
            if height is not None and source != "rangefinder_relative_height"
        ),
        "height_source_counts": source_counts,
        "duration_sec": 0.0 if len(samples) < 2 else max(0.0, samples[-1][0] - samples[0][0]),
        "start_height_m": valid_heights[0][1] if valid_heights else None,
        "end_height_m": valid_heights[-1][1] if valid_heights else None,
        "min_height_m": min((height for _t, height in valid_heights), default=None),
        "max_height_m": max((height for _t, height in valid_heights), default=None),
        "max_downward_speed_mps": max_downward_speed,
        "max_downward_speed_source": max_speed_entry,
        "max_touchdown_downward_speed_mps": max_touchdown_downward_speed,
        "max_descent_rate_mps": max_descent_rate_mps,
        "speed_ok": speed_ok,
        "height_source": "rangefinder_relative_height_preferred",
        "rangefinder_outlier_count": len(outlier_indices),
        "rangefinder_rate_outlier_count": len(rate_outlier_details),
        "vertical_velocity_outlier_count": len(vertical_velocity_outliers),
        "rangefinder_outlier_min_m": rangefinder_outlier_min_m,
        "rangefinder_outlier_max_neighbor_dt_sec": rangefinder_outlier_max_neighbor_dt_sec,
        "rangefinder_max_rate_mps": rangefinder_max_rate_mps,
        "rangefinder_outliers": outlier_details[:10],
        "rangefinder_rate_outliers": rate_outlier_details[:10],
        "vertical_velocity_outliers": vertical_velocity_outliers[:10],
        "touchdown_altitude_m": touchdown_altitude_m,
        "post_touchdown_max_height_m": post_touchdown_max_height,
        "post_touchdown_bounce_m": post_touchdown_bounce,
        "max_post_touchdown_bounce_m": max_post_touchdown_bounce_m,
        "bounce_ok": bounce_ok,
    }


def hold_yaw_or_current(hold_yaw_rad: float | None, current_yaw_rad: float | None) -> float:
    if hold_yaw_rad is not None:
        return hold_yaw_rad
    return current_yaw_rad if current_yaw_rad is not None else 0.0


def capture_hold_anchor(
    hold_x: float | None,
    hold_y: float | None,
    hold_yaw_rad: float | None,
    current_x: float | None,
    current_y: float | None,
    current_yaw_rad: float | None,
    *,
    refresh_yaw: bool = False,
) -> tuple[float | None, float | None, float]:
    yaw_anchor = current_yaw_rad if refresh_yaw and current_yaw_rad is not None else hold_yaw_rad
    if hold_x is not None and hold_y is not None:
        return hold_x, hold_y, hold_yaw_or_current(yaw_anchor, current_yaw_rad)
    if current_x is None or current_y is None:
        return hold_x, hold_y, hold_yaw_or_current(yaw_anchor, current_yaw_rad)
    yaw = current_yaw_rad if current_yaw_rad is not None else hold_yaw_or_current(yaw_anchor, None)
    return current_x, current_y, yaw


def should_send_position_hold_setpoint(
    *,
    send_position_setpoints: bool,
    inputs: HoverInputs,
    decision: HoverDecision,
) -> bool:
    return (
        send_position_setpoints
        and inputs.airborne_seen
        and inputs.armed_seen
        and decision.phase == "hover_hold"
        and not decision.terminal
    )


def command_ack_success(command_acks: list[dict[str, int]], command_id: int) -> bool:
    return any(ack.get("command") == command_id and ack.get("result") == 0 for ack in command_acks)


def command_ack_accepted(
    command_acks: list[dict[str, int]],
    command_id: int,
    accepted_command_ids: set[int] | None = None,
) -> bool:
    return command_id in (accepted_command_ids or set()) or command_ack_success(command_acks, command_id)


def command_ack_rejected(command_acks: list[dict[str, int]], command_id: int) -> bool:
    return any(ack.get("command") == command_id and ack.get("result") not in (0, None) for ack in command_acks)


def append_bounded_command_ack(
    command_acks: list[dict[str, int]],
    ack: dict[str, int],
    *,
    max_count: int = 240,
) -> None:
    command_acks.append(ack)
    del command_acks[: max(0, len(command_acks) - max_count)]


def statustext_indicates_crash(text: str) -> bool:
    return "Crash:" in text


def append_bounded_statustext(
    statustext: list[dict[str, int | str]],
    entry: dict[str, int | str],
    *,
    max_count: int = 120,
) -> None:
    statustext.append(entry)
    del statustext[: max(0, len(statustext) - max_count)]


def height_reaches_target(value_m: float | None, *, target_alt_m: float, tolerance_m: float) -> bool:
    if value_m is None:
        return False
    value = float(value_m)
    if not math.isfinite(value):
        return False
    return abs(value - target_alt_m) <= tolerance_m


def independent_takeoff_height_reached(
    inputs: HoverInputs,
    *,
    takeoff_alt_m: float,
    hover_altitude_tolerance_m: float,
) -> bool:
    return height_reaches_target(
        inputs.external_nav_height_m,
        target_alt_m=takeoff_alt_m,
        tolerance_m=hover_altitude_tolerance_m,
    ) and height_reaches_target(
        inputs.rangefinder_relative_height_m,
        target_alt_m=takeoff_alt_m,
        tolerance_m=hover_altitude_tolerance_m,
    )


def _request_hover_streams(connection, target_system: int, target_component: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    for message_id, hz in (
        (mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2.0),
        (mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0),
        (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0),
        (mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 4.0),
        (mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 4.0),
        (mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10.0),
        (mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 1.0),
        (mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1.0),
        (mavlink.MAVLINK_MSG_ID_STATUSTEXT, 2.0),
    ):
        connection.mav.command_long_send(
            target_system,
            target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            int(1_000_000.0 / hz),
            0,
            0,
            0,
            0,
            0,
        )


def _command_disarm(connection, target_system: int, target_component: int, *, force: bool = False) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    force_magic = 21196 if force else 0
    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        force_magic,
        0,
        0,
        0,
        0,
        0,
    )


def _command_land(connection, target_system: int, target_component: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def _request_param_read(connection, target_system: int, target_component: int, name: str) -> None:
    connection.mav.param_request_read_send(
        target_system,
        target_component,
        name.encode("ascii"),
        -1,
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run NavLab FCU-controlled hover mission via MAVLink setpoints.")
    parser.add_argument("--endpoint", default="tcp:sitl:5765")
    parser.add_argument("--duration-sec", type=float, default=90.0)
    parser.add_argument("--summary-file", default="")
    parser.add_argument("--mode", default="GUIDED")
    parser.add_argument("--takeoff-alt-m", type=float, default=0.45)
    parser.add_argument("--min-airborne-alt-m", type=float, default=0.10)
    parser.add_argument("--preflight-ready-sec", type=float, default=5.0)
    parser.add_argument("--max-wait-ready-sec", type=float, default=35.0)
    parser.add_argument("--hover-settle-sec", type=float, default=2.0)
    parser.add_argument("--hover-altitude-tolerance-m", type=float, default=0.18)
    parser.add_argument("--hover-hold-sec", type=float, default=20.0)
    parser.add_argument("--max-horizontal-drift-m", type=float, default=1.0)
    parser.add_argument("--max-altitude-drift-m", type=float, default=0.6)
    parser.add_argument("--origin-lat-deg", type=float, default=DEFAULT_ORIGIN_LAT_DEG)
    parser.add_argument("--origin-lon-deg", type=float, default=DEFAULT_ORIGIN_LON_DEG)
    parser.add_argument("--origin-alt-m", type=float, default=DEFAULT_ORIGIN_ALT_M)
    parser.add_argument("--source-system", type=int, default=255)
    parser.add_argument("--source-component", type=int, default=190)
    parser.add_argument("--status-topic", default="/navlab/hover/status")
    parser.add_argument("--landing-status-topic", default="/navlab/landing/status")
    parser.add_argument("--landing-intent-topic", default="/navlab/landing/intent")
    parser.add_argument("--sim-log-topic", default=DEFAULT_SIM_LOG_TOPIC)
    parser.add_argument("--external-nav-status-topic", default="/external_nav/status")
    parser.add_argument("--mavlink-external-nav-status-topic", default="/mavlink_external_nav/status")
    parser.add_argument("--imu-status-topic", default="/imu/status")
    parser.add_argument("--mavlink-status-topic", default="/navlab/mavlink/status")
    parser.add_argument("--status-timeout-sec", type=float, default=1.0)
    parser.add_argument("--external-nav-loss-grace-sec", type=float, default=1.0)
    parser.add_argument("--setpoint-rate-hz", type=float, default=5.0)
    parser.add_argument(
        "--landing-policy",
        choices=[
            LANDING_POLICY_AP_LAND_MODE_AFTER_HOVER,
            LANDING_POLICY_GUIDED_DESCENT,
            LANDING_POLICY_LAND_IN_PLACE,
        ],
        default=LANDING_POLICY_AP_LAND_MODE_AFTER_HOVER,
    )
    parser.add_argument("--pre-land-hold-sec", type=float, default=2.0)
    parser.add_argument("--max-landing-duration-sec", type=float, default=35.0)
    parser.add_argument("--landing-descent-rate-mps", type=float, default=DEFAULT_LANDING_DESCENT_RATE_MPS)
    parser.add_argument(
        "--landing-land-command-altitude-m",
        type=float,
        default=DEFAULT_LANDING_LAND_COMMAND_ALTITUDE_M,
    )
    parser.add_argument("--landing-setpoint-lookahead-sec", type=float, default=0.5)
    parser.add_argument("--landing-slowdown-altitude-m", type=float, default=DEFAULT_LANDING_SLOWDOWN_ALTITUDE_M)
    parser.add_argument(
        "--landing-near-ground-descent-rate-mps",
        type=float,
        default=DEFAULT_LANDING_NEAR_GROUND_DESCENT_RATE_MPS,
    )
    parser.add_argument("--max-landing-descent-rate-mps", type=float, default=0.25)
    parser.add_argument("--touchdown-altitude-m", type=float, default=0.12)
    parser.add_argument("--touchdown-vertical-speed-mps", type=float, default=0.08)
    parser.add_argument("--touchdown-confirm-sec", type=float, default=0.5)
    parser.add_argument("--force-disarm-grace-sec", type=float, default=3.0)
    parser.add_argument("--force-disarm-after-touchdown", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-disarm", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-motors-safe", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-external-nav", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-imu-status", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--send-position-setpoints", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--disable-arming-checks", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--force-arm", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--simulate-mode-arm", action=argparse.BooleanOptionalAction, default=False)
    # Compatibility with MissionNodeConfig argv; hover ignores these obstacle-demo fields.
    parser.add_argument("--forward-speed-mps", type=float, default=0.0)
    parser.add_argument("--avoid-forward-speed-mps", type=float, default=0.0)
    parser.add_argument("--obstacle-detect-distance-m", type=float, default=0.0)
    parser.add_argument("--obstacle-avoid-distance-m", type=float, default=0.0)
    parser.add_argument("--scan-yaw-deg", type=float, default=0.0)
    parser.add_argument("--scan-dwell-sec", type=float, default=0.0)
    parser.add_argument("--pass-x-m", type=float, default=0.0)
    parser.add_argument("--return-y-m", type=float, default=0.0)
    parser.add_argument("--final-hold-sec", type=float, default=0.0)
    parser.add_argument("--scan-features-topic", default="/scan_features")
    parser.add_argument("--pose-topic", default="/sim/uav_pose")
    parser.add_argument("--scan-timeout-sec", type=float, default=1.0)
    parser.add_argument("--setpoint-lookahead-sec", type=float, default=0.0)
    return parser


def run(argv: Sequence[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)

    try:
        import rclpy
        from pymavlink import mavutil
        from pymavlink.dialects.v20 import ardupilotmega as mavlink
        from rclpy.node import Node
        from std_msgs.msg import String
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "mavlink_hover_mission_controller requires ROS2 and pymavlink. Run it from the NavLab companion image."
        ) from exc

    class MavlinkHoverMissionController(Node):
        def __init__(self) -> None:
            super().__init__("mavlink_hover_mission_controller")
            self._connection = mavutil.mavlink_connection(
                args.endpoint,
                source_system=args.source_system,
                source_component=args.source_component,
                dialect="ardupilotmega",
            )
            self._status_pub = self.create_publisher(String, args.status_topic, 10)
            self._landing_status_pub = self.create_publisher(String, args.landing_status_topic, 10)
            self._landing_intent_pub = self.create_publisher(String, args.landing_intent_topic, 10)
            self._sim_log_pub = self.create_publisher(String, args.sim_log_topic, 10)
            self.create_subscription(String, args.external_nav_status_topic, self._handle_external_nav_status, 10)
            self.create_subscription(
                String,
                args.mavlink_external_nav_status_topic,
                self._handle_mavlink_external_nav_status,
                10,
            )
            self.create_subscription(String, args.imu_status_topic, self._handle_imu_status, 10)
            self.create_subscription(String, args.mavlink_status_topic, self._handle_mavlink_status, 10)
            self.mode_number = mode_number(args.mode)
            self._target_system: int | None = None
            self._target_component: int | None = None
            self._external_nav_ready = False
            self._last_external_status_monotonic = 0.0
            self._last_external_nav_state = ""
            self._external_nav_slam_quality = "bad"
            self._external_nav_slam_quality_good = False
            self._external_nav_slam_quality_reason = "missing_status"
            self._last_external_status_payload: dict[str, object] = {}
            self._external_nav_status_history: list[dict[str, object]] = []
            self._slam_quality_loss_started: float | None = None
            self._external_nav_loss_started: float | None = None
            self._mavlink_external_nav_ready = False
            self._fcu_local_position_ready = False
            self._last_mavlink_external_nav_status_payload: dict[str, object] = {}
            self._last_mavlink_external_nav_status_monotonic = 0.0
            self._mavlink_external_nav_status_history: list[dict[str, object]] = []
            self._mavlink_external_nav_loss_started: float | None = None
            self._fcu_local_position_loss_started: float | None = None
            self._imu_ready = False
            self._last_imu_status_monotonic = 0.0
            self._ready_started: float | None = None
            self._expected_mode_seen = False
            self._current_custom_mode: int | None = None
            self._armed_seen = False
            self._guided_seen_ever = False
            self._armed_seen_ever = False
            self._external_expected_mode_seen = False
            self._external_armed_seen = False
            self._airborne_seen = False
            self._airborne_started: float | None = None
            self._hover_started: float | None = None
            self._hold_x: float | None = None
            self._hold_y: float | None = None
            self._hold_yaw_rad = 0.0
            self._current_x: float | None = None
            self._current_y: float | None = None
            self._current_z: float | None = None
            self._ground_z_ned: float | None = None
            self._current_vz: float | None = None
            self._current_range_m: float | None = None
            self._ground_range_m: float | None = None
            self._current_yaw_rad: float | None = None
            self._next_request = 0.0
            self._next_heartbeat = 0.0
            self._next_origin_command = 0.0
            self._next_mode_command = 0.0
            self._next_arm_command = 0.0
            self._next_takeoff_command = 0.0
            self._next_setpoint = 0.0
            self._next_land_command = 0.0
            self._next_disarm_command = 0.0
            self._started = time.monotonic()
            self._mission_fsm = MissionFsmRecorder(started_at_monotonic=self._started)
            self._phases_seen: set[str] = set()
            self._phase_counts: dict[str, int] = {}
            self._status_history: list[dict[str, object]] = []
            self._setpoints_sent = 0
            self._message_counts: dict[str, int] = {}
            self._command_acks: list[dict[str, int]] = []
            self._accepted_command_ids: set[int] = set()
            self._statustext: list[dict[str, object]] = []
            self._crash_detected = False
            self._ekf_flags: list[int] = []
            self._sent_commands: dict[str, int] = {}
            self._gps_global_origin_seen = False
            self._home_position_seen = False
            self._hover_samples: list[tuple[float, float, float, float]] = []
            self._hover_altitude_samples: list[dict[str, float | None]] = []
            self._best_hover_samples: list[tuple[float, float, float, float]] = []
            self._best_hover_altitude_samples: list[dict[str, float | None]] = []
            self._hover_hold_segments_seen = 0
            self._landing_started: float | None = None
            self._hover_body_ok = False
            self._hover_body_reason = ""
            self._landing_state = "not_started"
            self._landed_state: int | None = None
            self._touchdown_confirmed = False
            self._touchdown_candidate_since: float | None = None
            self._land_command_sent = False
            self._land_command_sent_time: float | None = None
            self._fcu_land_param_requests_sent = False
            self._fcu_land_params: dict[str, float] = {}
            self._mode_before_land: int | None = None
            self._mode_after_land: int | None = None
            self._land_mode_seen = False
            self._land_mode_seen_elapsed_sec: float | None = None
            self._landed_state_timeline: list[dict[str, object]] = []
            self._force_disarm_used = False
            self._touchdown_confirmed_time: float | None = None
            self._frozen_hover_evidence: dict[str, object] = {}
            self._landing_descent_started: float | None = None
            self._landing_start_z_ned: float | None = None
            self._landing_descent_samples: list[LandingDescentSample] = []
            self._landing_blockers: list[str] = []
            self.create_timer(0.05, self._tick)
            self.get_logger().info(f"hover mission controller started endpoint={args.endpoint}")

        def _record_mission_fsm(
            self,
            now: float,
            state: str,
            reason: str,
            *,
            guard: str | None = None,
            blocker: str | None = None,
        ) -> None:
            self._mission_fsm.transition(
                now_monotonic=now,
                state=state,
                reason=reason,
                guard=guard,
                blocker=blocker,
            )

        def _mission_fsm_snapshot(self) -> dict[str, object]:
            return self._mission_fsm.snapshot(now_monotonic=time.monotonic())

        def _stop_vehicle(self) -> None:
            if self._target_system is None or self._target_component is None:
                return
            _command_disarm(self._connection, self._target_system, self._target_component)
            self._count_sent_command("disarm")

        def _request_fcu_land_params(self) -> None:
            if self._target_system is None or self._target_component is None or self._fcu_land_param_requests_sent:
                return
            for name in FCU_LAND_PARAM_NAMES:
                _request_param_read(self._connection, self._target_system, self._target_component, name)
            self._fcu_land_param_requests_sent = True

        def _handle_external_nav_status(self, msg: String) -> None:
            previous_ready = self._external_nav_ready
            previous_state = self._last_external_nav_state
            previous_quality = self._external_nav_slam_quality
            payload = self._parse_status_payload(msg.data)
            self._last_external_status_payload = payload
            self._external_nav_ready = payload.get("ready") is True
            self._last_external_nav_state = str(payload.get("state") or "")
            self._external_nav_slam_quality = str(payload.get("slam_quality") or "bad")
            self._external_nav_slam_quality_good = payload.get("slam_quality_good") is True
            self._external_nav_slam_quality_reason = str(payload.get("slam_quality_reason") or "")
            self._last_external_status_monotonic = time.monotonic()
            if (
                self._external_nav_ready != previous_ready
                or self._last_external_nav_state != previous_state
                or self._external_nav_slam_quality != previous_quality
            ):
                odom = payload.get("odom") if isinstance(payload.get("odom"), dict) else {}
                event = {
                    "elapsed_sec": round(self._last_external_status_monotonic - self._started, 3),
                    "ready": self._external_nav_ready,
                    "state": self._last_external_nav_state,
                    "slam_quality": self._external_nav_slam_quality,
                    "slam_quality_good": self._external_nav_slam_quality_good,
                    "slam_quality_reason": self._external_nav_slam_quality_reason,
                    "input_topic": odom.get("input_topic"),
                    "rate_hz": odom.get("rate_hz"),
                    "rate_ok": odom.get("rate_ok"),
                    "frame_ok": odom.get("frame_ok"),
                    "age_ms": odom.get("age_ms"),
                }
                self._external_nav_status_history.append(event)
                self._external_nav_status_history = self._external_nav_status_history[-40:]
                self.get_logger().info(
                    "external_nav status "
                    f"ready={self._external_nav_ready} "
                    f"state={self._last_external_nav_state} "
                    f"slam_quality={self._external_nav_slam_quality} "
                    f"slam_reason={self._external_nav_slam_quality_reason} "
                    f"input={event['input_topic']} "
                    f"rate_hz={event['rate_hz']} "
                    f"rate_ok={event['rate_ok']} "
                    f"frame_ok={event['frame_ok']}"
                )

        def _handle_imu_status(self, msg: String) -> None:
            self._imu_ready = self._status_ready(msg.data)
            self._last_imu_status_monotonic = time.monotonic()

        def _handle_mavlink_external_nav_status(self, msg: String) -> None:
            payload = self._parse_status_payload(msg.data)
            self._last_mavlink_external_nav_status_payload = payload
            self._mavlink_external_nav_ready = payload.get("ready") is True
            self._fcu_local_position_ready = payload.get("fcu_local_position_ready") is True
            now = time.monotonic()
            self._last_mavlink_external_nav_status_monotonic = now
            event = {
                "elapsed_sec": round(now - self._started, 3),
                "ready": self._mavlink_external_nav_ready,
                "state": payload.get("state"),
                "sent_count": payload.get("sent_count"),
                "local_position_count": payload.get("local_position_count"),
                "local_position_age_ms": payload.get("local_position_age_ms"),
                "fcu_local_position_ready": self._fcu_local_position_ready,
            }
            self._mavlink_external_nav_status_history.append(event)
            self._mavlink_external_nav_status_history = self._mavlink_external_nav_status_history[-40:]

        def _handle_mavlink_status(self, msg: String) -> None:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            self._external_expected_mode_seen = payload.get("mode_number") == self.mode_number
            self._external_armed_seen = payload.get("armed") is True

        @staticmethod
        def _status_ready(data: str) -> bool:
            return MavlinkHoverMissionController._parse_status_payload(data).get("ready") is True

        @staticmethod
        def _parse_status_payload(data: str) -> dict[str, object]:
            try:
                payload = json.loads(data)
            except json.JSONDecodeError:
                return {}
            return payload if isinstance(payload, dict) else {}

        def _tick(self) -> None:
            now = time.monotonic()
            if self._landing_started is not None:
                self._tick_landing(now)
                return
            if now - self._started >= args.duration_sec:
                if self._armed_seen or self._airborne_seen:
                    self._hover_body_ok = False
                    self._hover_body_reason = "duration_timeout"
                    self._start_landing(now)
                    return
                self._stop_vehicle()
                self.write_summary(ok=False, reason="duration_timeout", landing_ok=False)
                rclpy.try_shutdown()
                return
            self._drain_mavlink()
            if self._crash_detected:
                self._stop_vehicle()
                self.write_summary(ok=False, reason="crash_detected", landing_ok=False)
                rclpy.try_shutdown()
                return
            if now >= self._next_heartbeat:
                send_gcs_heartbeat(self._connection)
                self._next_heartbeat = now + 1.0
            if self._target_system is not None and self._target_component is not None and now >= self._next_request:
                _request_hover_streams(self._connection, self._target_system, self._target_component)
                if args.disable_arming_checks:
                    set_arming_check(self._connection, self._target_system, self._target_component, 0)
                self._next_request = now + 2.0
            if (
                self._target_system is not None
                and self._target_component is not None
                and now >= self._next_origin_command
            ):
                if not self._gps_global_origin_seen:
                    set_ekf_origin(
                        self._connection,
                        self._target_system,
                        args.origin_lat_deg,
                        args.origin_lon_deg,
                        args.origin_alt_m,
                    )
                    self._count_sent_command("set_gps_global_origin")
                if not self._home_position_seen:
                    set_home_position(
                        self._connection,
                        self._target_system,
                        self._target_component,
                        args.origin_lat_deg,
                        args.origin_lon_deg,
                        args.origin_alt_m,
                    )
                    self._count_sent_command("set_home_position")
                self._next_origin_command = now + 2.0

            inputs = self._build_inputs(now)
            decision = decide_hover(
                inputs,
                requirements=HoverRequirements(
                    require_external_nav=args.require_external_nav,
                    require_fcu_external_nav=args.require_external_nav,
                    require_imu_status=args.require_imu_status,
                    external_nav_loss_grace_sec=args.external_nav_loss_grace_sec,
                ),
                preflight_ready_sec=args.preflight_ready_sec,
                hover_settle_sec=args.hover_settle_sec,
                hover_hold_sec=args.hover_hold_sec,
                takeoff_alt_m=args.takeoff_alt_m,
                hover_altitude_tolerance_m=args.hover_altitude_tolerance_m,
            )
            self._phases_seen.add(decision.phase)
            self._phase_counts[decision.phase] = self._phase_counts.get(decision.phase, 0) + 1
            self._record_mission_fsm(
                now,
                mission_fsm_state_for_hover_phase(decision.phase),
                decision.reason,
                guard=decision.phase,
                blocker=decision.reason if decision.phase == "abort" else None,
            )
            if should_fail_fast_wait_ready(
                inputs,
                decision,
                mission_elapsed_sec=now - self._started,
                max_wait_ready_sec=args.max_wait_ready_sec,
            ):
                self._record_mission_fsm(
                    now,
                    mission_fsm_state_for_hover_phase(decision.phase),
                    "preflight_timeout",
                    guard=decision.phase,
                    blocker=decision.reason,
                )
                self._hover_body_ok = False
                self._hover_body_reason = "preflight_timeout"
                self._stop_vehicle()
                self.write_summary(ok=False, reason="preflight_timeout", landing_ok=False)
                rclpy.try_shutdown()
                return
            if decision.phase != "hover_hold" and self._hover_started is not None and not decision.terminal:
                self._remember_hover_segment()
                self._hover_started = None
                self._hover_samples.clear()
                self._hover_altitude_samples.clear()
            if decision.phase == "hover_hold" and self._hover_started is None:
                self._hover_started = now
                self._hover_hold_segments_seen += 1
                self._hover_samples.clear()
                self._hover_altitude_samples.clear()
                self._hold_x, self._hold_y, self._hold_yaw_rad = capture_hold_anchor(
                    self._hold_x,
                    self._hold_y,
                    self._hold_yaw_rad,
                    self._current_x,
                    self._current_y,
                    self._current_yaw_rad,
                    refresh_yaw=True,
                )
            if decision.phase == "hover_hold" and self._current_x is not None and self._current_y is not None:
                self._hover_samples.append((now, self._current_x, self._current_y, self._current_z or 0.0))
                self._record_hover_altitude_sample(now)

            if self._target_system is not None and self._target_component is not None:
                if decision.should_set_guided and now >= self._next_mode_command:
                    set_mode(self._connection, self._target_system, self.mode_number)
                    self._count_sent_command("set_mode_guided")
                    self._next_mode_command = now + 1.0
                if decision.should_arm and now >= self._next_arm_command:
                    command_arm(self._connection, self._target_system, self._target_component, args.force_arm)
                    self._count_sent_command("arm")
                    self._next_arm_command = now + 2.0
                if decision.should_takeoff and now >= self._next_takeoff_command:
                    command_takeoff(self._connection, self._target_system, self._target_component, args.takeoff_alt_m)
                    self._count_sent_command("takeoff")
                    self._next_takeoff_command = now + 2.0
                should_send_hold_setpoint = should_send_position_hold_setpoint(
                    send_position_setpoints=args.send_position_setpoints,
                    inputs=inputs,
                    decision=decision,
                )
                if should_send_hold_setpoint and now >= self._next_setpoint:
                    self._hold_x, self._hold_y, self._hold_yaw_rad = capture_hold_anchor(
                        self._hold_x,
                        self._hold_y,
                        self._hold_yaw_rad,
                        self._current_x,
                        self._current_y,
                        self._current_yaw_rad,
                    )
                    target_x, target_y, target_z = hover_hold_setpoint_axes(
                        hold_x=self._hold_x,
                        hold_y=self._hold_y,
                        current_x=self._current_x,
                        current_y=self._current_y,
                        current_z=self._current_z,
                        target_z_ned=self._target_z_ned(),
                    )
                    send_local_position_yaw_setpoint(
                        self._connection,
                        self._target_system,
                        self._target_component,
                        target_x,
                        target_y,
                        target_z,
                        hold_yaw_or_current(self._hold_yaw_rad, self._current_yaw_rad),
                    )
                    self._setpoints_sent += 1
                    self._count_sent_command("local_position_yaw_setpoint")
                    self._next_setpoint = now + (1.0 / args.setpoint_rate_hz)

            self._publish_status(decision, inputs)
            if decision.terminal:
                hover_samples, altitude_samples = self._select_hover_evidence()
                drift = summarize_hover_drift(hover_samples)
                altitude_crosscheck = summarize_hover_altitude_crosscheck(
                    altitude_samples,
                    target_alt_m=args.takeoff_alt_m,
                    tolerance_m=args.hover_altitude_tolerance_m,
                )
                self._hover_body_ok = (
                    drift.ok
                    and altitude_crosscheck["ok"] is True
                    and drift.duration_sec >= args.hover_hold_sec - HOVER_DURATION_TOLERANCE_SEC
                    and drift.horizontal_drift_m <= args.max_horizontal_drift_m
                    and drift.z_span_m <= args.max_altitude_drift_m
                    and self._message_counts.get("LOCAL_POSITION_NED", 0) > 0
                    and not self._crash_detected
                )
                self._hover_body_reason = "hover_complete" if self._hover_body_ok else "hover_unstable"
                self._start_landing(now)

        def _start_landing(self, now: float) -> None:
            self._landing_started = now
            self._landing_state = "task_body_complete"
            hover_samples, altitude_samples = self._select_hover_evidence()
            drift = summarize_hover_drift(hover_samples)
            altitude_crosscheck = summarize_hover_altitude_crosscheck(
                altitude_samples,
                target_alt_m=args.takeoff_alt_m,
                tolerance_m=args.hover_altitude_tolerance_m,
            )
            self._frozen_hover_evidence = {
                "takeoff_ack_ok": command_ack_accepted(
                    self._command_acks,
                    mavlink.MAV_CMD_NAV_TAKEOFF,
                    self._accepted_command_ids,
                ),
                "hover_altitude_crosscheck": altitude_crosscheck,
                "hover_drift": {
                    "sample_count": drift.sample_count,
                    "duration_sec": drift.duration_sec,
                    "horizontal_span_m": json_safe_number(drift.horizontal_span_m),
                    "z_span_m": json_safe_number(drift.z_span_m),
                    "horizontal_drift_m": json_safe_number(drift.horizontal_drift_m),
                    "z_drift_m": json_safe_number(drift.z_drift_m),
                },
                "hover_body_ok": self._hover_body_ok,
                "crash_detected": self._crash_detected,
            }
            self._record_mission_fsm(
                now,
                mission_fsm_state_for_landing_state(self._landing_state),
                self._hover_body_reason or "task_body_complete",
                guard=self._landing_state,
            )
            intent = String()
            intent.data = json.dumps(
                {
                    "source": "mavlink_hover_mission_controller",
                    "kind": "land_in_place",
                    "policy": args.landing_policy,
                    "reason": self._hover_body_reason,
                    "updated_ms": int(time.time() * 1000),
                },
                separators=(",", ":"),
                sort_keys=True,
            )
            self._landing_intent_pub.publish(intent)
            self._publish_landing_status()

        def _tick_landing(self, now: float) -> None:
            if self._target_system is None or self._target_component is None:
                self._landing_blockers.append("landing_target_system_missing")
                self._record_mission_fsm(
                    now,
                    "S_abort",
                    "landing_target_system_missing",
                    guard=self._landing_state,
                    blocker="landing_target_system_missing",
                )
                self.write_summary(ok=False, reason="landing_target_system_missing", landing_ok=False)
                rclpy.try_shutdown()
                return
            self._drain_mavlink()
            elapsed = 0.0 if self._landing_started is None else now - self._landing_started
            if elapsed > args.max_landing_duration_sec:
                self._landing_blockers.append("landing_timeout")
                self._record_mission_fsm(
                    now,
                    "S_abort",
                    "landing_timeout",
                    guard=self._landing_state,
                    blocker="landing_timeout",
                )
                self._stop_vehicle()
                self.write_summary(ok=False, reason="landing_timeout", landing_ok=False)
                rclpy.try_shutdown()
                return
            if elapsed < args.pre_land_hold_sec:
                self._landing_state = "pre_land_hold"
                self._send_hold_setpoint(now)
                self._publish_landing_status()
                return
            if self._landing_descent_started is None:
                self._landing_descent_started = now
                self._landing_start_z_ned = self._current_z if self._current_z is not None else -args.takeoff_alt_m
            self._record_landing_descent_sample(now)
            touchdown_ready = self._touchdown_candidate(now)
            if should_use_guided_descent_before_land(
                landing_policy=args.landing_policy,
                land_command_sent=self._land_command_sent,
                touchdown_ready=touchdown_ready,
            ):
                self._landing_state = "guided_descent"
                self._send_landing_descent_setpoint(now)
                self._publish_landing_status()
                return
            if should_command_land_this_tick(
                landing_policy=args.landing_policy,
                land_command_sent=self._land_command_sent,
                touchdown_ready=touchdown_ready,
                command_due=now >= self._next_land_command,
            ):
                self._landing_state = "land_command_sent"
                first_land_command = not self._land_command_sent
                if first_land_command:
                    self._land_command_sent_time = now
                    self._mode_before_land = self._current_custom_mode
                    self._request_fcu_land_params()
                _command_land(self._connection, self._target_system, self._target_component)
                self._count_sent_command("land")
                self._land_command_sent = True
                self._next_land_command = now + 2.0
                if first_land_command:
                    self._record_mission_fsm(
                        now,
                        mission_fsm_state_for_landing_state(self._landing_state),
                        "command_land",
                        guard=self._landing_state,
                    )
            if command_ack_rejected(self._command_acks, mavlink.MAV_CMD_NAV_LAND):
                self._landing_blockers.append("landing_command_rejected")
                self._record_mission_fsm(
                    now,
                    "S_abort",
                    "landing_command_rejected",
                    guard=self._landing_state,
                    blocker="landing_command_rejected",
                )
                self.write_summary(ok=False, reason="landing_command_rejected", landing_ok=False)
                rclpy.try_shutdown()
                return
            if touchdown_ready and not self._touchdown_confirmed:
                self._touchdown_confirmed_time = now
            self._touchdown_confirmed = self._touchdown_confirmed or touchdown_ready
            self._landing_state = "touchdown_candidate" if self._touchdown_confirmed else "descent_monitoring"
            disarmed = not self._armed_seen
            touchdown_confirmed_elapsed_sec = (
                None if self._touchdown_confirmed_time is None else now - self._touchdown_confirmed_time
            )
            if (
                should_send_disarm_after_touchdown(
                    touchdown_confirmed=self._touchdown_confirmed,
                    disarmed=disarmed,
                    require_disarm=args.require_disarm,
                    touchdown_confirmed_elapsed_sec=touchdown_confirmed_elapsed_sec,
                    force_disarm_grace_sec=args.force_disarm_grace_sec,
                )
                and now >= self._next_disarm_command
            ):
                self._landing_state = "disarm_requested"
                _command_disarm(
                    self._connection,
                    self._target_system,
                    self._target_component,
                    force=args.force_disarm_after_touchdown,
                )
                self._force_disarm_used = self._force_disarm_used or bool(args.force_disarm_after_touchdown)
                self._count_sent_command("disarm")
                self._next_disarm_command = now + 2.0
            motors_safe = disarmed if args.require_motors_safe else True
            descent_profile = self._landing_descent_profile()
            land_command_accepted = command_ack_accepted(
                self._command_acks,
                mavlink.MAV_CMD_NAV_LAND,
                self._accepted_command_ids,
            )
            landing_ok = landing_acceptance_ok(
                landing_policy=args.landing_policy,
                land_command_sent=self._land_command_sent,
                land_command_accepted=land_command_accepted,
                land_mode_seen=self._land_mode_seen,
                touchdown_confirmed=self._touchdown_confirmed,
                disarmed=disarmed,
                motors_safe=motors_safe,
                require_disarm=args.require_disarm,
                require_motors_safe=args.require_motors_safe,
                descent_profile_ok=descent_profile.get("ok") is True,
            )
            self._publish_landing_status()
            if landing_ok:
                self._landing_state = "landing_complete"
                self._record_mission_fsm(
                    now,
                    mission_fsm_state_for_landing_state(self._landing_state),
                    "landing_complete",
                    guard=self._landing_state,
                )
                final_ok = self._hover_body_ok and landing_ok
                if final_ok:
                    self._record_mission_fsm(
                        now,
                        "S13 task_success",
                        "task_success",
                        guard="task_success",
                    )
                self.write_summary(ok=final_ok, reason=self._hover_body_reason, landing_ok=True)
                rclpy.try_shutdown()

        def _send_hold_setpoint(self, now: float) -> None:
            if (
                not args.send_position_setpoints
                or self._target_system is None
                or self._target_component is None
                or now < self._next_setpoint
            ):
                return
            target_x, target_y, target_z = hover_hold_setpoint_axes(
                hold_x=self._hold_x,
                hold_y=self._hold_y,
                current_x=self._current_x,
                current_y=self._current_y,
                current_z=self._current_z,
                target_z_ned=-args.takeoff_alt_m,
            )
            send_local_position_yaw_setpoint(
                self._connection,
                self._target_system,
                self._target_component,
                target_x,
                target_y,
                target_z,
                hold_yaw_or_current(self._hold_yaw_rad, self._current_yaw_rad),
            )
            self._setpoints_sent += 1
            self._count_sent_command("local_position_yaw_setpoint")
            self._next_setpoint = now + (1.0 / args.setpoint_rate_hz)

        def _send_landing_descent_setpoint(self, now: float) -> None:
            if (
                not args.send_position_setpoints
                or self._target_system is None
                or self._target_component is None
                or now < self._next_setpoint
            ):
                return
            self._hold_x, self._hold_y, self._hold_yaw_rad = capture_hold_anchor(
                self._hold_x,
                self._hold_y,
                self._hold_yaw_rad,
                self._current_x,
                self._current_y,
                self._current_yaw_rad,
            )
            start_z = self._landing_start_z_ned if self._landing_start_z_ned is not None else -args.takeoff_alt_m
            ground_z = self._ground_z_ned if self._ground_z_ned is not None else 0.0
            descent_elapsed = 0.0 if self._landing_descent_started is None else now - self._landing_descent_started
            effective_descent_rate = landing_effective_descent_rate_mps(
                nominal_descent_rate_mps=args.landing_descent_rate_mps,
                rangefinder_relative_height_m=self._rangefinder_relative_height_m(),
                slowdown_altitude_m=args.landing_slowdown_altitude_m,
                near_ground_descent_rate_mps=args.landing_near_ground_descent_rate_mps,
            )
            target_z = landing_descent_target_z_ned(
                start_z_ned=start_z,
                ground_z_ned=ground_z,
                elapsed_sec=descent_elapsed,
                descent_rate_mps=effective_descent_rate,
                current_z_ned=self._current_z,
                setpoint_lookahead_sec=args.landing_setpoint_lookahead_sec,
            )
            send_local_position_yaw_setpoint(
                self._connection,
                self._target_system,
                self._target_component,
                hold_axis_or_current(self._hold_x, self._current_x),
                hold_axis_or_current(self._hold_y, self._current_y),
                target_z,
                hold_yaw_or_current(self._hold_yaw_rad, self._current_yaw_rad),
            )
            self._setpoints_sent += 1
            self._count_sent_command("local_position_yaw_setpoint")
            self._next_setpoint = now + (1.0 / args.setpoint_rate_hz)

        def _record_landing_descent_sample(self, now: float) -> None:
            evidence_height, evidence_source = landing_descent_evidence_height_and_source_m(
                current_range_m=self._current_range_m,
                ground_range_m=self._ground_range_m,
                current_z_ned=self._current_z,
                ground_z_ned=self._ground_z_ned,
            )
            self._landing_descent_samples.append(
                (
                    now,
                    evidence_height,
                    self._current_range_m,
                    self._current_vz,
                    evidence_source,
                )
            )
            self._landing_descent_samples = self._landing_descent_samples[-2000:]

        def _landing_descent_profile(self) -> dict[str, object]:
            return summarize_landing_descent_profile(
                self._landing_descent_samples,
                max_descent_rate_mps=args.max_landing_descent_rate_mps,
                touchdown_altitude_m=args.touchdown_altitude_m,
            )

        def _raw_touchdown_candidate(self) -> bool:
            touchdown_height, _touchdown_height_source = landing_descent_evidence_height_and_source_m(
                current_range_m=self._current_range_m,
                ground_range_m=self._ground_range_m,
                current_z_ned=self._current_z,
                ground_z_ned=self._ground_z_ned,
            )
            return landing_touchdown_candidate(
                landed_state_on_ground=self._landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND,
                current_range_m=touchdown_height,
                current_z_ned=self._current_z,
                current_vz_mps=self._current_vz,
                touchdown_altitude_m=args.touchdown_altitude_m,
                touchdown_vertical_speed_mps=args.touchdown_vertical_speed_mps,
            )

        def _touchdown_candidate(self, now: float) -> bool:
            if self._landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND:
                return True
            if not self._raw_touchdown_candidate():
                self._touchdown_candidate_since = None
                return False
            if self._touchdown_candidate_since is None:
                self._touchdown_candidate_since = now
            return now - self._touchdown_candidate_since >= max(0.0, args.touchdown_confirm_sec)

        def _drain_mavlink(self) -> None:
            while True:
                msg = self._connection.recv_match(blocking=False)
                if msg is None:
                    return
                msg_type = msg.get_type()
                self._message_counts[msg_type] = self._message_counts.get(msg_type, 0) + 1
                if msg_type == "HEARTBEAT" and int(msg.autopilot) != mavlink.MAV_AUTOPILOT_INVALID:
                    self._target_system = msg.get_srcSystem()
                    self._target_component = msg.get_srcComponent()
                    self._current_custom_mode = int(msg.custom_mode)
                    self._expected_mode_seen = self._current_custom_mode == self.mode_number
                    if self._land_command_sent:
                        if self._mode_after_land is None and self._current_custom_mode != self._mode_before_land:
                            self._mode_after_land = self._current_custom_mode
                        if self._current_custom_mode == ARDUCOPTER_LAND_MODE_NUMBER and not self._land_mode_seen:
                            self._land_mode_seen = True
                            if self._land_command_sent_time is not None:
                                self._land_mode_seen_elapsed_sec = max(
                                    0.0,
                                    time.monotonic() - self._land_command_sent_time,
                                )
                    self._armed_seen = bool(int(msg.base_mode) & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._guided_seen_ever = self._guided_seen_ever or self._expected_mode_seen
                    self._armed_seen_ever = self._armed_seen_ever or self._armed_seen
                elif msg_type == "COMMAND_ACK":
                    if int(msg.result) == 0:
                        self._accepted_command_ids.add(int(msg.command))
                    append_bounded_command_ack(
                        self._command_acks,
                        {"command": int(msg.command), "result": int(msg.result)},
                    )
                elif msg_type == "PARAM_VALUE":
                    name = mavlink_param_id_to_str(getattr(msg, "param_id", ""))
                    if name in FCU_LAND_PARAM_NAMES:
                        self._fcu_land_params[name] = float(msg.param_value)
                elif msg_type == "STATUSTEXT":
                    text = str(msg.text).rstrip("\x00")
                    if statustext_indicates_crash(text):
                        self._crash_detected = True
                    append_bounded_statustext(
                        self._statustext,
                        {"severity": int(msg.severity), "text": text},
                    )
                elif msg_type == "LOCAL_POSITION_NED":
                    self._current_x = float(msg.x)
                    self._current_y = float(msg.y)
                    self._current_z = float(msg.z)
                    self._current_vz = float(getattr(msg, "vz", 0.0))
                    if self._ground_z_ned is None:
                        self._ground_z_ned = self._current_z
                    if self._ground_z_ned - self._current_z >= args.min_airborne_alt_m:
                        self._airborne_seen = True
                elif msg_type == "ATTITUDE":
                    self._current_yaw_rad = float(msg.yaw)
                elif msg_type == "GLOBAL_POSITION_INT":
                    if float(msg.relative_alt) / 1000.0 >= args.min_airborne_alt_m:
                        self._airborne_seen = True
                elif msg_type == "EKF_STATUS_REPORT":
                    self._ekf_flags.append(int(msg.flags))
                elif msg_type == "GPS_GLOBAL_ORIGIN":
                    self._gps_global_origin_seen = True
                elif msg_type == "HOME_POSITION":
                    self._home_position_seen = True
                elif msg_type in {"DISTANCE_SENSOR", "RANGEFINDER"}:
                    if hasattr(msg, "current_distance"):
                        self._current_range_m = float(msg.current_distance) / 100.0
                    elif hasattr(msg, "distance"):
                        self._current_range_m = float(msg.distance)
                    if self._ground_range_m is None and self._current_range_m is not None:
                        self._ground_range_m = self._current_range_m
                elif msg_type == "EXTENDED_SYS_STATE":
                    self._landed_state = int(msg.landed_state)
                    self._landed_state_timeline.append(
                        {
                            "elapsed_sec": round(time.monotonic() - self._started, 3),
                            "landed_state": self._landed_state,
                        }
                    )
                    self._landed_state_timeline = self._landed_state_timeline[-80:]
                if self._airborne_seen and self._airborne_started is None:
                    self._airborne_started = time.monotonic()

        def _external_nav_height_m(self) -> float | None:
            height = self._last_external_status_payload.get("height")
            if not isinstance(height, dict):
                return None
            value = height.get("z")
            if not isinstance(value, int | float):
                return None
            value = float(value)
            return value if math.isfinite(value) else None

        def _fcu_local_height_m(self, z_ned: float | None = None) -> float | None:
            z = self._current_z if z_ned is None else z_ned
            if z is None:
                return None
            if self._ground_z_ned is not None:
                return float(self._ground_z_ned) - float(z)
            return -float(z)

        def _rangefinder_relative_height_m(self) -> float | None:
            if self._current_range_m is None or self._ground_range_m is None:
                return None
            return max(0.0, float(self._current_range_m) - float(self._ground_range_m))

        def _record_hover_altitude_sample(self, now: float) -> None:
            self._hover_altitude_samples.append(
                {
                    "elapsed_sec": now - self._started,
                    "fcu_local_z_ned": self._current_z,
                    "fcu_local_height_m": self._fcu_local_height_m(),
                    "external_nav_height_m": self._external_nav_height_m(),
                    "rangefinder_range_m": self._current_range_m,
                    "rangefinder_relative_height_m": self._rangefinder_relative_height_m(),
                }
            )

        def _remember_hover_segment(self) -> None:
            if not self._hover_samples:
                return
            current = summarize_hover_drift(self._hover_samples)
            best = summarize_hover_drift(self._best_hover_samples)
            if current.duration_sec >= best.duration_sec:
                self._best_hover_samples = list(self._hover_samples)
                self._best_hover_altitude_samples = list(self._hover_altitude_samples)

        def _select_hover_evidence(
            self,
        ) -> tuple[list[tuple[float, float, float, float]], list[dict[str, float | None]]]:
            candidates = []
            if self._best_hover_samples:
                candidates.append((self._best_hover_samples, self._best_hover_altitude_samples))
            if self._hover_samples:
                candidates.append((self._hover_samples, self._hover_altitude_samples))
            if not candidates:
                return self._hover_samples, self._hover_altitude_samples
            return max(candidates, key=lambda item: summarize_hover_drift(item[0]).duration_sec)

        def _build_inputs(self, now: float) -> HoverInputs:
            external_nav_fresh = (
                self._last_external_status_monotonic > 0.0
                and now - self._last_external_status_monotonic <= args.status_timeout_sec
            )
            mavlink_external_nav_fresh = (
                self._last_mavlink_external_nav_status_monotonic > 0.0
                and now - self._last_mavlink_external_nav_status_monotonic <= args.status_timeout_sec
            )
            imu_fresh = (
                self._last_imu_status_monotonic > 0.0
                and now - self._last_imu_status_monotonic <= args.status_timeout_sec
            )
            hover_elapsed = 0.0 if self._hover_started is None else now - self._hover_started
            airborne_elapsed = 0.0 if self._airborne_started is None else now - self._airborne_started
            takeoff_ack_ok = command_ack_accepted(
                self._command_acks,
                mavlink.MAV_CMD_NAV_TAKEOFF,
                self._accepted_command_ids,
            )
            external_ready = self._external_nav_ready and external_nav_fresh
            slam_quality_good = self._external_nav_slam_quality_good and external_nav_fresh
            mavlink_external_nav_ready = self._mavlink_external_nav_ready and mavlink_external_nav_fresh
            fcu_local_position_ready = self._fcu_local_position_ready and mavlink_external_nav_fresh
            imu_ready = self._imu_ready and imu_fresh
            slam_quality_loss_duration = self._loss_duration_sec(
                now,
                slam_quality_good or not self._airborne_seen,
                "_slam_quality_loss_started",
            )
            external_nav_loss_duration = self._loss_duration_sec(
                now,
                external_ready or not self._airborne_seen,
                "_external_nav_loss_started",
            )
            mavlink_external_nav_loss_duration = self._loss_duration_sec(
                now,
                mavlink_external_nav_ready or not self._airborne_seen,
                "_mavlink_external_nav_loss_started",
            )
            fcu_local_position_loss_duration = self._loss_duration_sec(
                now,
                fcu_local_position_ready or not self._airborne_seen,
                "_fcu_local_position_loss_started",
            )
            ready_for_preflight = (
                (external_ready or not args.require_external_nav)
                and (slam_quality_good or not args.require_external_nav)
                and (mavlink_external_nav_ready or not args.require_external_nav)
                and (fcu_local_position_ready or not args.require_external_nav)
                and (imu_ready or not args.require_imu_status)
            )
            if ready_for_preflight:
                if self._ready_started is None:
                    self._ready_started = now
            else:
                self._ready_started = None
            ready_elapsed = 0.0 if self._ready_started is None else now - self._ready_started
            return HoverInputs(
                external_nav_ready=external_ready,
                mavlink_external_nav_ready=mavlink_external_nav_ready,
                fcu_local_position_ready=fcu_local_position_ready,
                imu_ready=imu_ready,
                slam_quality_good=slam_quality_good,
                slam_quality=self._external_nav_slam_quality if external_nav_fresh else "stale",
                ready_elapsed_sec=ready_elapsed,
                current_yaw_rad=self._current_yaw_rad,
                expected_mode_seen=(
                    args.simulate_mode_arm or self._expected_mode_seen or self._external_expected_mode_seen
                ),
                armed_seen=args.simulate_mode_arm or self._armed_seen or self._external_armed_seen,
                airborne_seen=self._airborne_seen,
                takeoff_ack_ok=takeoff_ack_ok,
                airborne_elapsed_sec=airborne_elapsed,
                hover_elapsed_sec=hover_elapsed,
                current_x=self._current_x,
                current_y=self._current_y,
                current_z_ned=self._current_z,
                current_height_m=self._current_range_m,
                external_nav_height_m=self._external_nav_height_m(),
                rangefinder_relative_height_m=self._rangefinder_relative_height_m(),
                target_z_ned=self._target_z_ned(),
                slam_quality_loss_duration_sec=slam_quality_loss_duration,
                external_nav_loss_duration_sec=external_nav_loss_duration,
                mavlink_external_nav_loss_duration_sec=mavlink_external_nav_loss_duration,
                fcu_local_position_loss_duration_sec=fcu_local_position_loss_duration,
            )

        def _loss_duration_sec(self, now: float, ok: bool, attr_name: str) -> float:
            if ok:
                setattr(self, attr_name, None)
                return 0.0
            started = getattr(self, attr_name)
            if started is None:
                setattr(self, attr_name, now)
                return 0.0
            return max(0.0, now - float(started))

        def _target_z_ned(self) -> float:
            ground_z = self._ground_z_ned if self._ground_z_ned is not None else 0.0
            return ground_z - args.takeoff_alt_m

        def _publish_status(self, decision: HoverDecision, inputs: HoverInputs) -> None:
            fsm_snapshot = self._mission_fsm_snapshot()
            status_payload = {
                "phase": decision.phase,
                "reason": decision.reason,
                "mission_fsm_state": fsm_snapshot["state"],
                "mission_fsm_state_entered_at_sec": fsm_snapshot["state_entered_at_sec"],
                "mission_fsm_last_transition_reason": fsm_snapshot["last_transition_reason"],
                "mission_fsm_blocker": fsm_snapshot["blocker"],
                "mission_fsm_history": fsm_snapshot["history"],
                "external_nav_ready": inputs.external_nav_ready,
                "slam_quality": inputs.slam_quality,
                "slam_quality_good": inputs.slam_quality_good,
                "slam_quality_reason": self._external_nav_slam_quality_reason,
                "slam_quality_loss_duration_sec": inputs.slam_quality_loss_duration_sec,
                "external_nav_loss_duration_sec": inputs.external_nav_loss_duration_sec,
                "mavlink_external_nav_ready": inputs.mavlink_external_nav_ready,
                "mavlink_external_nav_loss_duration_sec": inputs.mavlink_external_nav_loss_duration_sec,
                "fcu_local_position_ready": inputs.fcu_local_position_ready,
                "fcu_local_position_loss_duration_sec": inputs.fcu_local_position_loss_duration_sec,
                "imu_ready": inputs.imu_ready,
                "ready_elapsed_sec": inputs.ready_elapsed_sec,
                "expected_mode_seen": inputs.expected_mode_seen,
                "armed_seen": inputs.armed_seen,
                "airborne_seen": inputs.airborne_seen,
                "takeoff_ack_ok": inputs.takeoff_ack_ok,
                "airborne_elapsed_sec": inputs.airborne_elapsed_sec,
                "hover_elapsed_sec": inputs.hover_elapsed_sec,
                "setpoints_sent_count": self._setpoints_sent,
                "local_position_count": self._message_counts.get("LOCAL_POSITION_NED", 0),
                "rangefinder_count": self._rangefinder_count(),
                "position": {
                    "x": inputs.current_x,
                    "y": inputs.current_y,
                    "z_ned": inputs.current_z_ned,
                    "height_m": inputs.current_height_m,
                    "external_nav_height_m": inputs.external_nav_height_m,
                    "rangefinder_relative_height_m": inputs.rangefinder_relative_height_m,
                    "target_z_ned": inputs.target_z_ned,
                    "yaw_rad": self._current_yaw_rad,
                    "hold_x": self._hold_x,
                    "hold_y": self._hold_y,
                    "hold_yaw_rad": self._hold_yaw_rad,
                },
            }
            self._status_history.append(status_payload)
            self._status_history = self._status_history[-80:]
            msg = String()
            msg.data = json.dumps(status_payload, separators=(",", ":"), sort_keys=True)
            self._status_pub.publish(msg)
            sim_log = String()
            sim_log.data = encode_sim_log(
                source="mavlink_hover_mission_controller",
                event=decision.reason,
                mission_state="complete" if decision.terminal else "running",
                phase=decision.phase,
                current_x=inputs.current_x,
                current_y=inputs.current_y,
                current_z_ned=inputs.current_z_ned,
                current_yaw_rad=self._current_yaw_rad,
                setpoints_sent_count=self._setpoints_sent,
            )
            self._sim_log_pub.publish(sim_log)

        def _landing_summary(self) -> dict[str, object]:
            fsm_snapshot = self._mission_fsm_snapshot()
            land_command_accepted = command_ack_accepted(
                self._command_acks,
                mavlink.MAV_CMD_NAV_LAND,
                self._accepted_command_ids,
            )
            disarmed = not self._armed_seen
            motors_safe = disarmed if args.require_motors_safe else True
            descent_profile = self._landing_descent_profile()
            landing_controller = landing_controller_for_state(
                self._landing_state,
                landing_policy=args.landing_policy,
            )
            descent_profile_enforced = landing_descent_profile_enforced(args.landing_policy)
            ok = landing_acceptance_ok(
                landing_policy=args.landing_policy,
                land_command_sent=self._land_command_sent,
                land_command_accepted=land_command_accepted,
                land_mode_seen=self._land_mode_seen,
                touchdown_confirmed=self._touchdown_confirmed,
                disarmed=disarmed,
                motors_safe=motors_safe,
                require_disarm=args.require_disarm,
                require_motors_safe=args.require_motors_safe,
                descent_profile_ok=descent_profile.get("ok") is True,
            )
            auto_disarm_by_land_mode = bool(
                landing_controller == "ap_land_mode" and disarmed and not self._force_disarm_used
            )
            blockers = list(dict.fromkeys(self._landing_blockers))
            if self._landing_started is not None:
                if not self._touchdown_confirmed:
                    blockers.append("touchdown_not_confirmed")
                if args.require_disarm and not disarmed:
                    blockers.append("disarm_not_confirmed")
                if args.require_motors_safe and not motors_safe:
                    blockers.append("motors_not_safe")
                if landing_policy_uses_ap_land_mode(args.landing_policy) and not landing_handoff_confirmed(
                    landing_policy=args.landing_policy,
                    land_command_sent=self._land_command_sent,
                    land_command_accepted=land_command_accepted,
                    land_mode_seen=self._land_mode_seen,
                ):
                    blockers.append("ap_land_mode_handoff_not_confirmed")
                if descent_profile_enforced and descent_profile.get("speed_ok") is not True:
                    blockers.append("landing_descent_too_fast")
                if descent_profile_enforced and descent_profile.get("bounce_ok") is not True:
                    blockers.append("landing_post_touchdown_bounce")
            else:
                blockers.append("landing_not_started")
            return {
                "ok": ok,
                "claim": "evaluated" if self._landing_started is not None else "not_evaluated",
                "policy": args.landing_policy,
                "state": self._landing_state,
                "landing_controller": landing_controller,
                "official_land_mode_descent_control": landing_policy_uses_ap_land_mode(args.landing_policy),
                "descent_profile_enforced": descent_profile_enforced,
                "frozen_hover_evidence": self._frozen_hover_evidence,
                "mission_fsm_state": fsm_snapshot["state"],
                "mission_fsm_state_entered_at_sec": fsm_snapshot["state_entered_at_sec"],
                "mission_fsm_last_transition_reason": fsm_snapshot["last_transition_reason"],
                "mission_fsm_blocker": fsm_snapshot["blocker"],
                "mission_fsm_history": fsm_snapshot["history"],
                "return_home": {
                    "required": False,
                    "ok": True,
                    "state": "not_required",
                    "distance_to_home_m": None,
                    "duration_sec": None,
                },
                "land_command_sent": self._land_command_sent,
                "land_command_sent_time_sec": None
                if self._land_command_sent_time is None
                else max(0.0, self._land_command_sent_time - self._started),
                "land_command_accepted": land_command_accepted,
                "mode_before_land": self._mode_before_land,
                "mode_after_land": self._mode_after_land,
                "land_mode_seen": self._land_mode_seen,
                "land_mode_seen_elapsed_sec": self._land_mode_seen_elapsed_sec,
                "landed_state_timeline": self._landed_state_timeline[-40:],
                "landing_duration_sec": None
                if self._landing_started is None
                else max(0.0, time.monotonic() - self._landing_started),
                "landed_confirmed": self._touchdown_confirmed,
                "touchdown_confirmed": self._touchdown_confirmed,
                "disarmed": disarmed,
                "motors_safe": motors_safe,
                "require_disarm": args.require_disarm,
                "require_motors_safe": args.require_motors_safe,
                "touchdown_confirm_sec": args.touchdown_confirm_sec,
                "touchdown_confirmed_time_sec": None
                if self._touchdown_confirmed_time is None
                else max(0.0, self._touchdown_confirmed_time - self._started),
                "force_disarm_grace_sec": args.force_disarm_grace_sec,
                "force_disarm_after_touchdown": args.force_disarm_after_touchdown,
                "force_disarm_used": self._force_disarm_used,
                "auto_disarm_by_land_mode": auto_disarm_by_land_mode,
                "landing_setpoint_lookahead_sec": args.landing_setpoint_lookahead_sec,
                "landing_slowdown_altitude_m": args.landing_slowdown_altitude_m,
                "landing_near_ground_descent_rate_mps": args.landing_near_ground_descent_rate_mps,
                "uses_gazebo_truth_as_input": False,
                "last_range_m": self._current_range_m,
                "last_rangefinder_relative_height_m": self._rangefinder_relative_height_m(),
                "last_z_ned": self._current_z,
                "last_vz_mps": self._current_vz,
                "landed_state": self._landed_state,
                "fcu_land_params": fcu_land_params_report(self._fcu_land_params),
                "descent_profile": descent_profile,
                "blockers": sorted(set(blockers)) if not ok else [],
            }

        def _publish_landing_status(self) -> None:
            self._record_mission_fsm(
                time.monotonic(),
                mission_fsm_state_for_landing_state(self._landing_state),
                self._landing_state,
                guard=self._landing_state,
            )
            msg = String()
            msg.data = json.dumps(self._landing_summary(), separators=(",", ":"), sort_keys=True)
            self._landing_status_pub.publish(msg)

        def _rangefinder_count(self) -> int:
            return self._message_counts.get("DISTANCE_SENSOR", 0) + self._message_counts.get("RANGEFINDER", 0)

        def _count_sent_command(self, name: str) -> None:
            self._sent_commands[name] = self._sent_commands.get(name, 0) + 1

        def write_summary(self, *, ok: bool, reason: str, landing_ok: bool) -> None:
            if not args.summary_file:
                return
            self._remember_hover_segment()
            hover_samples, altitude_samples = self._select_hover_evidence()
            drift = summarize_hover_drift(hover_samples)
            drift_quality = classify_hover_drift(drift, max_horizontal_drift_m=args.max_horizontal_drift_m)
            altitude_crosscheck = summarize_hover_altitude_crosscheck(
                altitude_samples,
                target_alt_m=args.takeoff_alt_m,
                tolerance_m=args.hover_altitude_tolerance_m,
            )
            hover_z_ned = hover_samples[-1][3] if hover_samples else self._current_z
            target_z_ned = self._target_z_ned()
            if hover_z_ned is not None:
                altitude_error_m = abs(float(hover_z_ned) - float(target_z_ned))
            elif self._current_range_m is not None:
                altitude_error_m = abs(float(self._current_range_m) - float(args.takeoff_alt_m))
            else:
                altitude_error_m = None
            landing_summary = self._landing_summary()
            external_nav_age_sec = -1.0
            if self._last_external_status_monotonic > 0.0:
                external_nav_age_sec = time.monotonic() - self._last_external_status_monotonic
            external_nav_ready = (
                self._external_nav_ready
                and self._last_external_status_monotonic > 0.0
                and external_nav_age_sec <= args.status_timeout_sec
            )
            mavlink_external_nav_age_sec = -1.0
            if self._last_mavlink_external_nav_status_monotonic > 0.0:
                mavlink_external_nav_age_sec = time.monotonic() - self._last_mavlink_external_nav_status_monotonic
            mavlink_external_nav_ready = (
                self._mavlink_external_nav_ready
                and self._last_mavlink_external_nav_status_monotonic > 0.0
                and mavlink_external_nav_age_sec <= args.status_timeout_sec
            )
            fcu_local_position_ready = self._fcu_local_position_ready and mavlink_external_nav_ready
            fsm_snapshot = self._mission_fsm_snapshot()
            summary = {
                "ok": ok,
                "reason": reason,
                "mission_fsm_state": fsm_snapshot["state"],
                "mission_fsm_state_entered_at_sec": fsm_snapshot["state_entered_at_sec"],
                "mission_fsm_last_transition_reason": fsm_snapshot["last_transition_reason"],
                "mission_fsm_blocker": fsm_snapshot["blocker"],
                "mission_fsm_history": fsm_snapshot["history"],
                "hover_body_ok": self._hover_body_ok,
                "landing_ok": landing_ok,
                "phases_seen": sorted(self._phases_seen),
                "phase_counts": dict(sorted(self._phase_counts.items())),
                "status_history": self._status_history[-40:],
                "mode": args.mode,
                "mode_number": self.mode_number,
                "guided_seen": self._guided_seen_ever or self._expected_mode_seen or self._external_expected_mode_seen,
                "armed_seen": self._armed_seen_ever or self._armed_seen or self._external_armed_seen,
                "airborne_seen": self._airborne_seen,
                "takeoff_ack_ok": command_ack_accepted(
                    self._command_acks,
                    mavlink.MAV_CMD_NAV_TAKEOFF,
                    self._accepted_command_ids,
                ),
                "arm_ack_ok": command_ack_accepted(
                    self._command_acks,
                    mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    self._accepted_command_ids,
                ),
                "crash_detected": self._crash_detected,
                "setpoints_sent_count": self._setpoints_sent,
                "local_position_count": self._message_counts.get("LOCAL_POSITION_NED", 0),
                "rangefinder_count": self._rangefinder_count(),
                "target_alt_m": args.takeoff_alt_m,
                "takeoff_alt_m": args.takeoff_alt_m,
                "ground_z_ned": self._ground_z_ned,
                "target_z_ned": target_z_ned,
                "current_z_ned": hover_z_ned,
                "current_height_m": self._current_range_m,
                "altitude_error_m": altitude_error_m,
                "hover_altitude_sources": altitude_crosscheck["sources"],
                "hover_altitude_crosscheck": altitude_crosscheck,
                "preflight_ready_sec": args.preflight_ready_sec,
                "max_wait_ready_sec": args.max_wait_ready_sec,
                "hover_settle_sec": args.hover_settle_sec,
                "hover_altitude_tolerance_m": args.hover_altitude_tolerance_m,
                "hover_hold_sec": args.hover_hold_sec,
                "hover_hold_duration_sec": drift.duration_sec,
                "hover_hold_segments_seen": self._hover_hold_segments_seen,
                "require_external_nav": args.require_external_nav,
                "external_nav_ready": external_nav_ready,
                "external_nav_status_age_sec": external_nav_age_sec,
                "external_nav_status_history": self._external_nav_status_history[-40:],
                "mavlink_external_nav_ready": mavlink_external_nav_ready,
                "fcu_local_position_ready": fcu_local_position_ready,
                "mavlink_external_nav_status_age_sec": mavlink_external_nav_age_sec,
                "mavlink_external_nav_status": self._last_mavlink_external_nav_status_payload,
                "mavlink_external_nav_status_history": self._mavlink_external_nav_status_history[-40:],
                "require_imu_status": args.require_imu_status,
                "send_position_setpoints": args.send_position_setpoints,
                "hover_drift": {
                    "sample_count": drift.sample_count,
                    "duration_sec": drift.duration_sec,
                    "horizontal_span_m": json_safe_number(drift.horizontal_span_m),
                    "z_span_m": json_safe_number(drift.z_span_m),
                    "horizontal_drift_m": json_safe_number(drift.horizontal_drift_m),
                    "z_drift_m": json_safe_number(drift.z_drift_m),
                    "max_horizontal_drift_m": args.max_horizontal_drift_m,
                    "max_altitude_drift_m": args.max_altitude_drift_m,
                    "duration_tolerance_sec": HOVER_DURATION_TOLERANCE_SEC,
                    "quality": drift_quality,
                    "gps_like": drift_quality == "tight"
                    and drift.horizontal_drift_m <= 0.05
                    and drift.z_span_m <= 0.05,
                    "horizontal_span_ok": drift.horizontal_span_m <= args.max_horizontal_drift_m,
                    "horizontal_drift_ok": drift.horizontal_drift_m <= args.max_horizontal_drift_m,
                    "z_span_ok": drift.z_span_m <= args.max_altitude_drift_m,
                    "duration_ok": drift.duration_sec >= args.hover_hold_sec - HOVER_DURATION_TOLERANCE_SEC,
                    "ok": (
                        drift.ok
                        and drift.duration_sec >= args.hover_hold_sec - HOVER_DURATION_TOLERANCE_SEC
                        and drift.horizontal_drift_m <= args.max_horizontal_drift_m
                        and drift.z_span_m <= args.max_altitude_drift_m
                    ),
                },
                "last_position": {"x": self._current_x, "y": self._current_y, "z_ned": self._current_z},
                "hold_position": {"x": self._hold_x, "y": self._hold_y},
                "last_yaw_rad": self._current_yaw_rad,
                "hold_yaw_rad": self._hold_yaw_rad,
                "message_counts": self._message_counts,
                "sent_commands": dict(sorted(self._sent_commands.items())),
                "accepted_command_ids": sorted(self._accepted_command_ids),
                "command_acks": self._command_acks[-60:],
                "statustext": self._statustext[-60:],
                "ekf_flags_seen": sorted(set(self._ekf_flags)),
                "gps_global_origin_seen": self._gps_global_origin_seen,
                "home_position_seen": self._home_position_seen,
                "land_command_accepted": landing_summary["land_command_accepted"],
                "touchdown_confirmed": landing_summary["touchdown_confirmed"],
                "disarmed": landing_summary["disarmed"],
                "motors_safe": landing_summary["motors_safe"],
                "landing": landing_summary,
            }
            path = Path(args.summary_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = path.with_name(path.name + ".tmp")
            tmp_path.write_text(json.dumps(summary, allow_nan=False, indent=2, sort_keys=True), encoding="utf-8")
            tmp_path.replace(path)

    rclpy.init(args=None)
    node = MavlinkHoverMissionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
