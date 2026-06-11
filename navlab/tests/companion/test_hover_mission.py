from __future__ import annotations

from math import isclose

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navlab.sim.companion.nodes.hover_mission import (
    HoverInputs,
    HoverRequirements,
    command_ack_success,
    decide_hover,
    summarize_hover_drift,
)


def _inputs(**overrides) -> HoverInputs:
    values = {
        "external_nav_ready": True,
        "imu_ready": True,
        "ready_elapsed_sec": 5.0,
        "current_yaw_rad": 0.0,
        "expected_mode_seen": True,
        "armed_seen": True,
        "airborne_seen": True,
        "takeoff_ack_ok": True,
        "airborne_elapsed_sec": 10.0,
        "hover_elapsed_sec": 20.0,
        "current_x": 0.0,
        "current_y": 0.0,
        "current_z_ned": -0.45,
    }
    values.update(overrides)
    return HoverInputs(**values)


def test_hover_decision_waits_then_guided_arm_takeoff_hold_complete() -> None:
    assert decide_hover(
        _inputs(external_nav_ready=False),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    ).phase == ("wait_ready")
    assert (
        decide_hover(
            _inputs(ready_elapsed_sec=4.9),
            preflight_ready_sec=5.0,
            hover_settle_sec=2.0,
            hover_hold_sec=20.0,
            takeoff_alt_m=0.45,
            hover_altitude_tolerance_m=0.18,
        ).reason
        == "waiting_for_stable_external_nav_and_imu"
    )
    assert (
        decide_hover(
            _inputs(current_yaw_rad=None),
            preflight_ready_sec=5.0,
            hover_settle_sec=2.0,
            hover_hold_sec=20.0,
            takeoff_alt_m=0.45,
            hover_altitude_tolerance_m=0.18,
        ).reason
        == "waiting_for_fcu_attitude"
    )

    guided = decide_hover(
        _inputs(expected_mode_seen=False, armed_seen=False, airborne_seen=False),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )
    arm = decide_hover(
        _inputs(armed_seen=False, airborne_seen=False),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )
    takeoff = decide_hover(
        _inputs(airborne_seen=False),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )
    takeoff_before_ack = decide_hover(
        _inputs(takeoff_ack_ok=False),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )
    settle = decide_hover(
        _inputs(airborne_elapsed_sec=1.9),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )
    altitude_settle = decide_hover(
        _inputs(current_z_ned=-0.2),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )
    hold = decide_hover(
        _inputs(hover_elapsed_sec=19.9),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )
    complete = decide_hover(
        _inputs(hover_elapsed_sec=20.0),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )

    assert guided.should_set_guided is True
    assert arm.should_arm is True
    assert takeoff.should_takeoff is True
    assert takeoff_before_ack.should_takeoff is True
    assert settle.phase == "hover_settle"
    assert altitude_settle.reason == "settling_until_target_altitude"
    assert hold.phase == "hover_hold"
    assert complete.terminal is True


def test_hover_decision_can_skip_external_nav_for_rangefinder_diagnostic() -> None:
    decision = decide_hover(
        _inputs(external_nav_ready=False, imu_ready=False, expected_mode_seen=False),
        requirements=HoverRequirements(require_external_nav=False, require_imu_status=False),
        preflight_ready_sec=5.0,
        hover_settle_sec=2.0,
        hover_hold_sec=20.0,
        takeoff_alt_m=0.45,
        hover_altitude_tolerance_m=0.18,
    )

    assert decision.phase == "guided"
    assert decision.should_set_guided is True


def test_hover_drift_summary_uses_span_and_endpoint_drift() -> None:
    summary = summarize_hover_drift(
        [
            (10.0, 0.0, 0.0, -0.45),
            (20.0, 0.1, -0.2, -0.43),
            (30.0, -0.1, 0.1, -0.47),
        ]
    )

    assert summary.sample_count == 3
    assert isclose(summary.duration_sec, 20.0)
    assert isclose(summary.horizontal_span_m, (0.2**2 + 0.3**2) ** 0.5)
    assert isclose(summary.z_span_m, 0.04)
    assert summary.ok is True


def test_command_ack_success_checks_mavlink_command_and_result() -> None:
    assert command_ack_success(
        [
            {"command": mavlink.MAV_CMD_NAV_TAKEOFF, "result": 4},
            {"command": mavlink.MAV_CMD_NAV_TAKEOFF, "result": 0},
        ],
        mavlink.MAV_CMD_NAV_TAKEOFF,
    )
    assert not command_ack_success(
        [{"command": mavlink.MAV_CMD_NAV_TAKEOFF, "result": 4}],
        mavlink.MAV_CMD_NAV_TAKEOFF,
    )
