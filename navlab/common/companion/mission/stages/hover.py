"""Hover stage decision and setpoint helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass

from navlab.common.companion.mission.command_adapter import request_mission_command
from navlab.common.companion.mission.context import MissionContext
from navlab.common.companion.mission.fsm import mission_fsm_state_for_hover_phase
from navlab.common.companion.mission.pipeline import StageResult


@dataclass(frozen=True, slots=True)
class HoverInputs:
    """Sensor and FCU inputs consumed by the hover decision stage."""

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
    """Runtime gates that decide which readiness signals are mandatory."""

    require_external_nav: bool = True
    require_fcu_external_nav: bool = True
    require_imu_status: bool = True
    external_nav_loss_grace_sec: float = 1.0


@dataclass(frozen=True, slots=True)
class HoverHoldConfig:
    """Configuration for the executable hover-body stage."""

    preflight_ready_sec: float = 5.0
    hover_settle_sec: float = 2.0
    hover_hold_sec: float = 20.0
    takeoff_alt_m: float = 0.45
    hover_altitude_tolerance_m: float = 0.18
    send_position_setpoints: bool = True
    requirements: HoverRequirements = HoverRequirements()


@dataclass(frozen=True, slots=True)
class HoverDecision:
    """Decision emitted by one hover stage evaluation."""

    phase: str
    reason: str
    should_set_guided: bool = False
    should_arm: bool = False
    should_takeoff: bool = False
    terminal: bool = False


def hover_inputs_from_context(ctx: MissionContext) -> HoverInputs:
    """Project shared mission context into the hover decision input contract."""

    return HoverInputs(
        external_nav_ready=ctx.state.nav.external_nav_ready,
        mavlink_external_nav_ready=ctx.state.nav.mavlink_external_nav_ready,
        fcu_local_position_ready=ctx.state.nav.fcu_local_position_ready,
        imu_ready=ctx.state.nav.imu_ready,
        slam_quality_good=ctx.state.nav.slam_quality_good,
        slam_quality=ctx.state.nav.slam_quality,
        ready_elapsed_sec=ctx.state.nav.ready_elapsed_sec,
        current_yaw_rad=ctx.state.pose.yaw_rad,
        expected_mode_seen=ctx.state.fcu.expected_mode_seen,
        armed_seen=ctx.state.fcu.armed,
        airborne_seen=ctx.state.fcu.airborne,
        takeoff_ack_ok=ctx.state.fcu.takeoff_ack_ok,
        airborne_elapsed_sec=ctx.state.hover.airborne_elapsed_sec,
        hover_elapsed_sec=ctx.state.hover.hover_elapsed_sec,
        current_x=ctx.state.pose.x_m,
        current_y=ctx.state.pose.y_m,
        current_z_ned=ctx.state.pose.z_ned_m,
        current_height_m=ctx.state.pose.height_m,
        external_nav_height_m=ctx.state.pose.external_nav_height_m,
        rangefinder_relative_height_m=ctx.state.pose.rangefinder_relative_height_m,
        target_z_ned=ctx.state.pose.target_z_ned_m,
        slam_quality_loss_duration_sec=ctx.state.nav.slam_quality_loss_duration_sec,
        external_nav_loss_duration_sec=ctx.state.nav.external_nav_loss_duration_sec,
        mavlink_external_nav_loss_duration_sec=ctx.state.nav.mavlink_external_nav_loss_duration_sec,
        fcu_local_position_loss_duration_sec=ctx.state.nav.fcu_local_position_loss_duration_sec,
    )


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
    """Evaluate the current hover phase and requested side effects."""

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


class HoverHoldStage:
    """Execute the hover body decision and request hold setpoints when needed."""

    name = "hover_hold"

    def __init__(self, config: HoverHoldConfig) -> None:
        """Create the hover-body stage."""

        self._config = config

    def tick(self, ctx: MissionContext) -> StageResult:
        """Evaluate hover progress and request a hold setpoint through the adapter."""

        inputs = hover_inputs_from_context(ctx)
        decision = decide_hover(
            inputs,
            requirements=self._config.requirements,
            preflight_ready_sec=self._config.preflight_ready_sec,
            hover_settle_sec=self._config.hover_settle_sec,
            hover_hold_sec=self._config.hover_hold_sec,
            takeoff_alt_m=self._config.takeoff_alt_m,
            hover_altitude_tolerance_m=self._config.hover_altitude_tolerance_m,
        )
        should_send_hold_setpoint = should_send_position_hold_setpoint(
            send_position_setpoints=self._config.send_position_setpoints,
            inputs=inputs,
            decision=decision,
        )
        command_sent = False
        if should_send_hold_setpoint:
            command_sent = request_mission_command(ctx, "send_hold_setpoint")

        evidence = {
            "phase": decision.phase,
            "terminal": decision.terminal,
            "should_send_hold_setpoint": should_send_hold_setpoint,
            "command_sent": command_sent,
        }
        fsm_state = mission_fsm_state_for_hover_phase(decision.phase)
        if decision.phase == "abort":
            return StageResult.abort(decision.reason, fsm_state=fsm_state, blocker=decision.reason, evidence=evidence)
        if decision.terminal:
            return StageResult.complete(decision.reason, fsm_state=fsm_state, evidence=evidence)
        return StageResult.running(decision.reason, fsm_state=fsm_state, evidence=evidence)


def should_fail_fast_wait_ready(
    inputs: HoverInputs,
    decision: HoverDecision,
    *,
    mission_elapsed_sec: float,
    max_wait_ready_sec: float,
) -> bool:
    """Return whether wait-ready has exceeded its fail-fast deadline."""

    return (
        max_wait_ready_sec > 0.0
        and decision.phase == "wait_ready"
        and not inputs.armed_seen
        and not inputs.airborne_seen
        and mission_elapsed_sec >= max_wait_ready_sec
    )


def hold_axis_or_current(hold_value: float | None, current_value: float | None) -> float:
    """Use a held axis value when available, otherwise fall back to current."""

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
    """Build local-NED hold setpoint axes from anchors and current pose."""

    return (
        hold_axis_or_current(hold_x, current_x),
        hold_axis_or_current(hold_y, current_y),
        hold_axis_or_current(target_z_ned, current_z),
    )


def hold_yaw_or_current(hold_yaw_rad: float | None, current_yaw_rad: float | None) -> float:
    """Use held yaw when available, otherwise current yaw or zero."""

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
    """Capture or retain the position/yaw anchor for a hold segment."""

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
    """Return whether a position-hold setpoint should be sent this tick."""

    return (
        send_position_setpoints
        and inputs.airborne_seen
        and inputs.armed_seen
        and decision.phase == "hover_hold"
        and not decision.terminal
    )


def height_reaches_target(value_m: float | None, *, target_alt_m: float, tolerance_m: float) -> bool:
    """Return whether a height sample is finite and within target tolerance."""

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
    """Require external-nav and rangefinder height to agree with takeoff target."""

    return height_reaches_target(
        inputs.external_nav_height_m,
        target_alt_m=takeoff_alt_m,
        tolerance_m=hover_altitude_tolerance_m,
    ) and height_reaches_target(
        inputs.rangefinder_relative_height_m,
        target_alt_m=takeoff_alt_m,
        tolerance_m=hover_altitude_tolerance_m,
    )
