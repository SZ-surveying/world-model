from __future__ import annotations

import signal
from pathlib import Path

from navlab.sim import waypoints
from navlab.sim.waypoints import compute_auto_run_decision, load_straight_line_mission


def test_load_straight_line_mission_from_yaml(tmp_path: Path) -> None:
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        "\n".join(
            [
                "version: 1",
                "mode: straight_line",
                "frame_id: map",
                "forward_speed: 0.3",
                "position_tolerance: 0.1",
                "goal:",
                "  x: 3.0",
                "  y: 0.0",
                "  z: 0.0",
            ]
        ),
        encoding="utf-8",
    )

    mission = load_straight_line_mission(mission_file)

    assert mission.forward_speed == 0.3
    assert mission.position_tolerance == 0.1
    assert mission.start.x == 0.0
    assert mission.goal.x == 3.0


def test_load_straight_line_mission_supports_explicit_start_and_goal(tmp_path: Path) -> None:
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        "\n".join(
            [
                "version: 1",
                "start:",
                "  x: 0.5",
                "  y: 0.0",
                "  z: 0.0",
                "goal:",
                "  x: 2.0",
                "  y: 0.0",
                "  z: 0.0",
            ]
        ),
        encoding="utf-8",
    )

    mission = load_straight_line_mission(mission_file)

    assert mission.start.x == 0.5
    assert mission.goal.x == 2.0


def test_load_straight_line_mission_rejects_non_monotonic_x(tmp_path: Path) -> None:
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        "\n".join(
            [
                "version: 1",
                "start:",
                "  x: 1.5",
                "  y: 0.0",
                "  z: 0.0",
                "goal:",
                "  x: 1.0",
                "  y: 0.0",
                "  z: 0.0",
            ]
        ),
        encoding="utf-8",
    )

    try:
        load_straight_line_mission(mission_file)
    except ValueError as exc:
        assert "moves backward in x" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_compute_auto_run_decision_moves_until_target_then_stops() -> None:
    mission = load_straight_line_mission("docs/sim/examples/straight_line_demo.yaml")

    forward = compute_auto_run_decision(
        mission=mission,
        current_x=0.0,
        front_min=5.0,
        stop_distance=0.5,
        forward_speed=0.2,
        position_tolerance=0.05,
    )
    assert forward.state == "forward"
    assert forward.mission_state == "running"
    assert forward.linear_x == 0.2
    assert forward.active_waypoint_index == 0

    stopped = compute_auto_run_decision(
        mission=mission,
        current_x=2.05,
        front_min=5.0,
        stop_distance=0.5,
        forward_speed=0.2,
        position_tolerance=0.05,
    )
    assert stopped.state == "stop"
    assert stopped.mission_state == "complete"
    assert stopped.reason == "mission_complete"
    assert stopped.linear_x == 0.0


def test_compute_auto_run_decision_respects_stop_distance() -> None:
    mission = load_straight_line_mission("docs/sim/examples/straight_line_demo.yaml")

    decision = compute_auto_run_decision(
        mission=mission,
        current_x=0.0,
        front_min=0.45,
        stop_distance=0.5,
        forward_speed=0.2,
        position_tolerance=0.05,
    )

    assert decision.state == "stop"
    assert decision.mission_state == "blocked_by_stop_guard"
    assert decision.reason == "stop_distance_reached"
    assert decision.linear_x == 0.0


def test_compute_auto_run_decision_tolerates_float_noise_near_stop_distance() -> None:
    mission = load_straight_line_mission("docs/sim/examples/blocked_by_stop_guard.yaml")

    decision = compute_auto_run_decision(
        mission=mission,
        current_x=4.5,
        front_min=0.50003,
        stop_distance=0.5,
        forward_speed=0.5,
        position_tolerance=0.05,
    )

    assert decision.state == "stop"
    assert decision.mission_state == "blocked_by_stop_guard"
    assert decision.reason == "stop_distance_reached"


def test_blocked_mission_example_places_goal_behind_obstacle() -> None:
    mission = load_straight_line_mission("docs/sim/examples/blocked_by_stop_guard.yaml")

    assert mission.start.x == 0.0
    assert mission.goal.x == 6.0


def test_compute_auto_run_decision_reports_ready_before_inputs_arrive() -> None:
    mission = load_straight_line_mission("docs/sim/examples/straight_line_demo.yaml")

    decision = compute_auto_run_decision(
        mission=mission,
        current_x=None,
        front_min=None,
        stop_distance=0.5,
        forward_speed=0.2,
        position_tolerance=0.05,
    )

    assert decision.mission_state == "ready"
    assert decision.reason == "waiting_for_pose"


def test_stop_auto_rosbag_if_configured_sends_sigint_and_waits(monkeypatch) -> None:
    signals: list[tuple[int, int]] = []
    checks = {"remaining": 2}

    def fake_killpg(pid: int, sig: int) -> None:
        if sig == signal.SIGINT:
            signals.append((pid, sig))
            return
        if sig == 0 and checks["remaining"] > 0:
            checks["remaining"] -= 1
            return
        if sig == 0:
            raise ProcessLookupError
        raise AssertionError(f"unexpected signal {sig}")

    monkeypatch.setenv("SIM_AUTO_ROSBAG_PID", "4242")
    monkeypatch.setattr(waypoints.os, "killpg", fake_killpg)
    monkeypatch.setattr(waypoints.time, "sleep", lambda _seconds: None)

    waypoints.stop_auto_rosbag_if_configured(timeout_sec=0.1)

    assert signals == [(4242, signal.SIGINT)]
