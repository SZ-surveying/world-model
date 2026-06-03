from __future__ import annotations

import json

from lab_env.sim.nodes.mavlink_obstacle_mission_controller import (
    MissionConfig,
    MissionDecision,
    MissionInputs,
    decide_obstacle_mission,
    encode_mission_status,
)


def _ready_inputs(**overrides) -> MissionInputs:
    values = {
        "current_x": 0.0,
        "current_y": 0.0,
        "current_z_ned": -0.8,
        "front_min": 5.0,
        "external_nav_ready": True,
        "imu_ready": True,
        "scan_fresh": True,
        "expected_mode_seen": True,
        "armed_seen": True,
        "airborne_seen": True,
        "airborne_elapsed_sec": 10.0,
    }
    values.update(overrides)
    return MissionInputs(**values)


def test_decision_waits_for_inputs_before_mode_or_arm() -> None:
    decision = decide_obstacle_mission(
        _ready_inputs(external_nav_ready=False),
        MissionConfig(),
    )

    assert decision.phase == "wait_ready"
    assert decision.should_set_guided is False
    assert decision.should_arm is False


def test_decision_sets_guided_then_arms_then_takeoff() -> None:
    config = MissionConfig()

    guided = decide_obstacle_mission(
        _ready_inputs(expected_mode_seen=False, armed_seen=False, airborne_seen=False),
        config,
    )
    arm = decide_obstacle_mission(_ready_inputs(armed_seen=False, airborne_seen=False), config)
    takeoff = decide_obstacle_mission(_ready_inputs(airborne_seen=False), config)

    assert guided.should_set_guided is True
    assert arm.should_arm is True
    assert takeoff.should_takeoff is True


def test_decision_moves_forward_when_clear() -> None:
    decision = decide_obstacle_mission(_ready_inputs(current_x=1.0, front_min=5.0), MissionConfig())

    assert decision.phase == "forward"
    assert decision.vx_mps > 0.0
    assert decision.vy_mps == 0.0


def test_decision_avoids_left_when_obstacle_seen() -> None:
    decision = decide_obstacle_mission(_ready_inputs(current_x=3.2, current_y=0.2, front_min=1.5), MissionConfig())

    assert decision.phase == "avoid"
    assert decision.vx_mps > 0.0
    assert decision.vy_mps > 0.0


def test_decision_returns_to_track_after_passing_obstacle() -> None:
    decision = decide_obstacle_mission(_ready_inputs(current_x=6.7, current_y=2.0, front_min=5.0), MissionConfig())

    assert decision.phase == "return_track"
    assert decision.vy_mps < 0.0


def test_decision_passes_obstacle_after_lateral_offset() -> None:
    decision = decide_obstacle_mission(_ready_inputs(current_x=4.5, current_y=2.7, front_min=5.0), MissionConfig())

    assert decision.phase == "pass_obstacle"
    assert decision.vx_mps > 0.0
    assert decision.vy_mps == 0.0


def test_encode_mission_status_is_foxglove_friendly_json() -> None:
    decision = MissionDecision("avoid", "running", "avoid_left", 0.05, 0.15)
    payload = encode_mission_status(
        decision=decision,
        inputs=_ready_inputs(current_x=3.0, current_y=0.5, front_min=1.4),
        setpoints_sent_count=12,
        obstacle_detected=True,
    )

    data = json.loads(payload)
    assert data["phase"] == "avoid"
    assert data["mission_state"] == "running"
    assert data["cmd"]["vy_mps"] == 0.15
    assert data["obstacle_detected"] is True
    assert data["position"]["x"] == 3.0
