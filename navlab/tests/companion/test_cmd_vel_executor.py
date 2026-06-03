from __future__ import annotations

from math import isclose

from navlab.common.perception.contract import DEFAULT_SCAN_CONTRACT
from navlab.common.perception.front_sector import ForwardStopStateMachine
from navlab.common.perception.scan_features import compute_scan_features
from navlab.companion.nodes.cmd_vel_executor import (
    PlanarPoseState,
    VelocityCommand,
    _build_set_pose_request,
    clamp_forward_speed_for_clearance,
    integrate_planar_pose,
)


def test_sim_top_level_exports_cover_common_helpers() -> None:
    assert DEFAULT_SCAN_CONTRACT.topic_name == "/scan"
    assert ForwardStopStateMachine(forward_speed=0.2, stop_distance=0.5).state == "forward"
    assert compute_scan_features([0.0] * DEFAULT_SCAN_CONTRACT.beam_count).front_min is None


def test_integrate_planar_pose_moves_forward_in_heading_direction() -> None:
    pose = PlanarPoseState(x=1.0, y=2.0, z=0.0, yaw=0.0)

    next_pose = integrate_planar_pose(pose, VelocityCommand(linear_x=0.5), 2.0)

    assert isclose(next_pose.x, 2.0)
    assert isclose(next_pose.y, 2.0)
    assert isclose(next_pose.yaw, 0.0)


def test_build_set_pose_request_contains_model_name_and_pose() -> None:
    request = _build_set_pose_request("uav_start_marker", PlanarPoseState(x=1.2, y=-0.5, z=0.0, yaw=0.0))

    assert 'name: "uav_start_marker"' in request
    assert "position { x: 1.2 y: -0.5 z: 0.0 }" in request
    assert "orientation {" in request


def test_clamp_forward_speed_for_clearance_limits_last_step_to_threshold() -> None:
    linear_x = clamp_forward_speed_for_clearance(
        2.0,
        dt=0.1,
        front_min=0.62,
        min_front_distance=0.5,
    )

    assert isclose(linear_x, 1.2)


def test_clamp_forward_speed_for_clearance_stops_forward_motion_when_already_too_close() -> None:
    linear_x = clamp_forward_speed_for_clearance(
        2.0,
        dt=0.1,
        front_min=0.45,
        min_front_distance=0.5,
    )

    assert linear_x == 0.0
