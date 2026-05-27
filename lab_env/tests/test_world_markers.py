from __future__ import annotations

from math import isclose

from lab_env.sim.nodes.world_marker_publisher import _apply_uav_pose, _build_local_marker_offsets
from lab_env.sim.perception.contract import DEFAULT_SCAN_CONTRACT
from lab_env.sim.world.world_markers import (
    MarkerPose,
    compute_forward_clearance,
    load_world_marker_specs,
    load_world_model_pose,
    load_world_obstacle_boxes,
    synthesize_planar_scan,
)


def test_world_marker_specs_cover_visible_models() -> None:
    specs = load_world_marker_specs("docker/worlds/uav_obstacle_5m.sdf")
    assert [spec.namespace for spec in specs] == [
        "uav_body_tail_marker",
        "uav_body_nose_marker",
        "obstacle_5m_ahead",
    ]


def test_uav_tail_marker_uses_flat_body_geometry_at_ground_origin() -> None:
    specs = load_world_marker_specs("docker/worlds/uav_obstacle_5m.sdf")
    tail = specs[0]
    assert tail.shape == "cube"
    assert tail.pose.x == -0.1
    assert tail.pose.z == 0.1
    assert abs(tail.scale.x - 0.4) < 1e-9
    assert tail.scale.y == 0.4
    assert tail.scale.z == 0.2
    assert tail.color.b == 0.85
    assert tail.pose.z - tail.scale.z / 2.0 == 0.0


def test_uav_nose_marker_uses_front_color_band() -> None:
    specs = load_world_marker_specs("docker/worlds/uav_obstacle_5m.sdf")
    nose = specs[1]
    assert nose.shape == "cube"
    assert nose.pose.x > specs[0].pose.x
    assert nose.pose.z == 0.1
    assert nose.scale.x == 0.2
    assert nose.scale.y == 0.4
    assert nose.scale.z == 0.2
    assert nose.color.r == 0.96
    assert nose.pose.z - nose.scale.z / 2.0 == 0.0


def test_obstacle_marker_faces_uav_with_4_by_2_face() -> None:
    specs = load_world_marker_specs("docker/worlds/uav_obstacle_5m.sdf")
    obstacle = specs[2]
    assert obstacle.shape == "cube"
    assert obstacle.pose.x == 5.5
    assert obstacle.scale.x == 1.0
    assert obstacle.scale.y == 4.0
    assert obstacle.scale.z == 2.0
    assert obstacle.color.a == 0.55


def test_uav_marker_specs_follow_runtime_pose() -> None:
    specs = load_world_marker_specs("docker/worlds/uav_obstacle_5m.sdf")
    root_pose = load_world_model_pose("docker/worlds/uav_obstacle_5m.sdf", "uav_start_marker")
    local_offsets = _build_local_marker_offsets(specs, root_pose=root_pose)

    moved_tail = _apply_uav_pose(
        specs[0],
        current_pose=MarkerPose(x=1.0, y=2.0, z=0.1, yaw=0.0),
        local_offsets=local_offsets,
    )
    moved_nose = _apply_uav_pose(
        specs[1],
        current_pose=MarkerPose(x=1.0, y=2.0, z=0.1, yaw=0.0),
        local_offsets=local_offsets,
    )

    assert isclose(moved_tail.pose.x, 0.9, abs_tol=1e-9)
    assert isclose(moved_tail.pose.y, 2.0, abs_tol=1e-9)
    assert isclose(moved_nose.pose.x, 1.2, abs_tol=1e-9)
    assert isclose(moved_nose.pose.y, 2.0, abs_tol=1e-9)


def test_world_obstacle_boxes_exclude_uav_models() -> None:
    boxes = load_world_obstacle_boxes("docker/worlds/uav_obstacle_5m.sdf")
    assert [box.name for box in boxes] == ["obstacle_5m_ahead"]


def test_synthesize_planar_scan_matches_expected_front_face_distance() -> None:
    boxes = load_world_obstacle_boxes("docker/worlds/uav_obstacle_5m.sdf")

    ranges = synthesize_planar_scan(
        boxes,
        origin_x=0.0,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
        contract=DEFAULT_SCAN_CONTRACT,
    )
    moved_ranges = synthesize_planar_scan(
        boxes,
        origin_x=2.0,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
        contract=DEFAULT_SCAN_CONTRACT,
    )

    center_index = DEFAULT_SCAN_CONTRACT.beam_count // 2
    assert isclose(ranges[center_index], 5.0, abs_tol=1e-6)
    assert isclose(moved_ranges[center_index], 3.0, abs_tol=1e-6)


def test_synthesize_planar_scan_clamps_inside_obstacle_to_range_min() -> None:
    boxes = load_world_obstacle_boxes("docker/worlds/uav_obstacle_5m.sdf")

    ranges = synthesize_planar_scan(
        boxes,
        origin_x=5.25,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
        contract=DEFAULT_SCAN_CONTRACT,
    )

    center_index = DEFAULT_SCAN_CONTRACT.beam_count // 2
    assert ranges[center_index] == DEFAULT_SCAN_CONTRACT.range_min


def test_compute_forward_clearance_matches_obstacle_front_face_distance() -> None:
    boxes = load_world_obstacle_boxes("docker/worlds/uav_obstacle_5m.sdf")

    clearance = compute_forward_clearance(
        boxes,
        origin_x=0.0,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
    )
    moved_clearance = compute_forward_clearance(
        boxes,
        origin_x=4.6,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
    )

    assert isclose(clearance or 0.0, 5.0, abs_tol=1e-6)
    assert isclose(moved_clearance or 0.0, 0.4, abs_tol=1e-6)
