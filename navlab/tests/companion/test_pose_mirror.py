from __future__ import annotations

import json
from math import isclose, pi
from types import SimpleNamespace

from navlab.companion.nodes.pose_mirror import (
    MavlinkTelemetryStatus,
    NedPoseSample,
    PoseMirrorStatus,
    anchored_sim_stamp_nanoseconds,
    build_pose_stamped_fields,
    encode_mavlink_telemetry_status,
    encode_pose_mirror_status,
    filter_displayable_markers,
    marker_has_displayable_geometry,
    ned_to_gazebo_pose,
    next_monotonic_stamp_nanoseconds,
    stamp_fields_to_nanoseconds,
)


def test_ned_to_gazebo_pose_maps_forward_right_and_altitude() -> None:
    pose = ned_to_gazebo_pose(NedPoseSample(x_north_m=2.0, y_east_m=0.5, z_down_m=-0.8, yaw_rad=0.25))

    assert isclose(pose.x, 2.0)
    assert isclose(pose.y, 0.5)
    assert isclose(pose.z, 0.8)
    assert isclose(pose.yaw, 0.25)


def test_ned_to_gazebo_pose_keeps_landed_marker_above_floor() -> None:
    pose = ned_to_gazebo_pose(NedPoseSample(x_north_m=0.0, y_east_m=0.0, z_down_m=0.0), min_z_m=0.1)

    assert isclose(pose.z, 0.1)


def test_build_pose_stamped_fields_uses_yaw_quaternion() -> None:
    pose = ned_to_gazebo_pose(NedPoseSample(x_north_m=1.0, y_east_m=2.0, z_down_m=-0.5, yaw_rad=pi / 2))

    fields = build_pose_stamped_fields(pose)

    assert isclose(fields["x"], 1.0)
    assert isclose(fields["y"], 2.0)
    assert isclose(fields["z"], 0.5)
    assert isclose(fields["qz"], 2**0.5 / 2)
    assert isclose(fields["qw"], 2**0.5 / 2)


def test_anchored_sim_stamp_uses_scan_time_domain_and_stays_monotonic() -> None:
    anchor_ns = stamp_fields_to_nanoseconds(12, 100)
    candidate = anchored_sim_stamp_nanoseconds(
        anchor_stamp_ns=anchor_ns,
        anchor_monotonic=20.0,
        now_monotonic=20.25,
    )

    assert candidate == 12_250_000_100
    assert next_monotonic_stamp_nanoseconds(candidate_ns=99, previous_ns=100) == 101
    assert next_monotonic_stamp_nanoseconds(candidate_ns=200, previous_ns=100) == 200


def test_marker_filter_removes_empty_line_list_markers() -> None:
    empty_line_list = SimpleNamespace(type=5, action=0, points=[])
    odd_line_list = SimpleNamespace(type=5, action=0, points=[object()])
    valid_line_list = SimpleNamespace(type=5, action=0, points=[object(), object()])
    delete_marker = SimpleNamespace(type=5, action=2, points=[])

    assert marker_has_displayable_geometry(empty_line_list) is False
    assert marker_has_displayable_geometry(odd_line_list) is False
    assert marker_has_displayable_geometry(valid_line_list) is True
    assert marker_has_displayable_geometry(delete_marker) is True
    assert filter_displayable_markers([empty_line_list, odd_line_list, valid_line_list]) == [valid_line_list]


def test_encode_pose_mirror_status_is_json_with_expected_state() -> None:
    payload = encode_pose_mirror_status(
        PoseMirrorStatus(
            state="mirroring",
            local_position_present=True,
            local_position_age_ms=12.3456,
            set_pose_count=3,
            last_gazebo_x=1.0,
            last_gazebo_y=0.0,
            last_gazebo_z=0.8,
            last_yaw_rad=0.1,
            reason="set_pose_sent",
        )
    )

    data = json.loads(payload)
    assert data["state"] == "mirroring"
    assert data["local_position"]["present"] is True
    assert data["local_position"]["age_ms"] == 12.346
    assert data["set_pose_count"] == 3
    assert data["reason"] == "set_pose_sent"


def test_encode_mavlink_telemetry_status_is_json_with_message_counts() -> None:
    payload = encode_mavlink_telemetry_status(
        MavlinkTelemetryStatus(
            state="streaming",
            heartbeat_seen=True,
            target_system=1,
            target_component=1,
            armed=True,
            mode_number=4,
            local_position_present=True,
            local_position_age_ms=12.3456,
            ekf_flags=[831],
            command_acks=[{"command": 400, "result": 0}],
            statustext=[{"severity": 6, "text": "Mode GUIDED"}],
            message_counts={"HEARTBEAT": 2, "LOCAL_POSITION_NED": 5},
        )
    )

    data = json.loads(payload)
    assert data["state"] == "streaming"
    assert data["heartbeat_seen"] is True
    assert data["armed"] is True
    assert data["local_position"]["age_ms"] == 12.346
    assert data["ekf"]["flags_seen"] == [831]
    assert data["command_acks"][0]["command"] == 400
    assert data["statustext"][0]["text"] == "Mode GUIDED"
    assert data["message_counts"]["LOCAL_POSITION_NED"] == 5
