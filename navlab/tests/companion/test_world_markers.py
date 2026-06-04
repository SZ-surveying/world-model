from __future__ import annotations

from math import isclose

from navlab.common.perception.contract import DEFAULT_SCAN_CONTRACT
from navlab.companion.nodes.world_markers import _apply_uav_pose, _build_local_marker_offsets
from navlab.sim.world.world_markers import (
    MarkerPose,
    compute_forward_clearance,
    load_world_marker_specs,
    load_world_model_pose,
    load_world_obstacle_boxes,
    synthesize_planar_scan,
)

FIGURE8_WORLD = "docker/worlds/navlab_iq_quad_figure8.sdf"


def _point_inside_box(x: float, y: float, box) -> bool:
    return (
        box.pose.x - box.scale.x / 2.0 <= x <= box.pose.x + box.scale.x / 2.0
        and box.pose.y - box.scale.y / 2.0 <= y <= box.pose.y + box.scale.y / 2.0
    )


def _assert_polyline_clear(points: list[tuple[float, float]]) -> None:
    boxes = load_world_obstacle_boxes(FIGURE8_WORLD)
    for start, end in zip(points[:-1], points[1:], strict=True):
        for step in range(41):
            ratio = step / 40.0
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            collisions = [box.name for box in boxes if _point_inside_box(x, y, box)]
            assert collisions == [], f"point {(x, y)} collides with {collisions}"


def test_world_marker_specs_cover_visible_models() -> None:
    specs = load_world_marker_specs(FIGURE8_WORLD)
    assert [spec.namespace for spec in specs] == [
        "navlab_iq_quad_body",
        "navlab_iq_quad_arm_a",
        "navlab_iq_quad_arm_b",
        "navlab_iq_quad_rotor_0_disc",
        "navlab_iq_quad_rotor_1_disc",
        "navlab_iq_quad_rotor_2_disc",
        "navlab_iq_quad_rotor_3_disc",
        "navlab_iq_quad_x2_lidar",
        "navlab_iq_quad_rotor_0_motor",
        "navlab_iq_quad_rotor_1_motor",
        "navlab_iq_quad_rotor_2_motor",
        "navlab_iq_quad_rotor_3_motor",
        "outer_left_north_wall",
        "outer_left_south_wall",
        "outer_left_west_wall",
        "outer_right_north_wall",
        "outer_right_south_wall",
        "outer_right_east_wall",
        "inner_left_island",
        "inner_right_island",
    ]


def test_uav_body_marker_is_self_contained_primitive_at_iq_quad_start() -> None:
    specs = load_world_marker_specs(FIGURE8_WORLD)
    body = specs[0]
    assert body.namespace == "navlab_iq_quad_body"
    assert body.shape == "sphere"
    assert body.pose.x == 0.0
    assert body.pose.z == 0.225
    assert body.scale.x == 0.34
    assert body.scale.y == 0.22
    assert body.scale.z == 0.12


def test_uav_replay_markers_do_not_require_external_mesh_fetch() -> None:
    specs = load_world_marker_specs(FIGURE8_WORLD)

    assert {spec.shape for spec in specs} <= {"arrow", "cube", "cylinder", "sphere"}
    assert all(not hasattr(spec, "mesh_resource") for spec in specs)
    assert all(not hasattr(spec, "mesh_file_path") for spec in specs)


def test_uav_rotor_and_lidar_markers_match_iq_quad_layout() -> None:
    specs = load_world_marker_specs(FIGURE8_WORLD)
    by_name = {spec.namespace: spec for spec in specs}
    rotor_0 = by_name["navlab_iq_quad_rotor_0_disc"]
    rotor_2 = by_name["navlab_iq_quad_rotor_2_disc"]
    lidar = by_name["navlab_iq_quad_x2_lidar"]

    assert rotor_0.shape == "cylinder"
    assert rotor_0.pose.x == 0.13
    assert rotor_0.pose.y == -0.22
    assert rotor_2.shape == "cylinder"
    assert lidar.shape == "cylinder"
    assert lidar.pose.x == 0.05
    assert isclose(lidar.pose.z, 0.3, abs_tol=1e-9)
    assert lidar.scale.x == 0.08


def test_figure8_world_has_two_shared_waist_rectangular_loops() -> None:
    specs = load_world_marker_specs(FIGURE8_WORLD)
    by_name = {spec.namespace: spec for spec in specs}

    assert by_name["outer_left_north_wall"].pose.x == -3.1
    assert by_name["outer_left_north_wall"].pose.y == 3.4
    assert by_name["outer_left_south_wall"].pose.y == -3.4
    assert by_name["outer_left_west_wall"].pose.x == -6.4
    assert by_name["outer_right_north_wall"].pose.x == 3.1
    assert by_name["outer_right_north_wall"].pose.y == 3.4
    assert by_name["outer_right_south_wall"].pose.y == -3.4
    assert by_name["outer_right_east_wall"].pose.x == 6.4

    left = by_name["inner_left_island"]
    right = by_name["inner_right_island"]
    assert left.shape == "cube"
    assert right.shape == "cube"
    assert left.pose.x == -right.pose.x
    assert left.pose.y == right.pose.y == 0.0
    assert left.scale.x == right.scale.x == 2.4
    assert left.scale.y == right.scale.y == 2.2
    assert left.scale.z == right.scale.z == 2.0
    assert left.color.a == right.color.a == 0.74


def test_figure8_origin_and_both_loops_are_navigable_corridors() -> None:
    boxes = load_world_obstacle_boxes(FIGURE8_WORLD)
    assert [box.name for box in boxes] == [
        "outer_left_north_wall",
        "outer_left_south_wall",
        "outer_left_west_wall",
        "outer_right_north_wall",
        "outer_right_south_wall",
        "outer_right_east_wall",
        "inner_left_island",
        "inner_right_island",
    ]
    assert [box.name for box in boxes if _point_inside_box(0.0, 0.0, box)] == []

    _assert_polyline_clear(
        [
            (0.0, 0.0),
            (-0.8, 1.8),
            (-3.1, 2.2),
            (-5.2, 1.2),
            (-5.2, -1.2),
            (-3.1, -2.2),
            (-0.8, -1.8),
            (0.0, 0.0),
        ]
    )
    _assert_polyline_clear(
        [
            (0.0, 0.0),
            (0.8, 1.8),
            (3.1, 2.2),
            (5.2, 1.2),
            (5.2, -1.2),
            (3.1, -2.2),
            (0.8, -1.8),
            (0.0, 0.0),
        ]
    )


def test_uav_marker_specs_follow_runtime_pose() -> None:
    specs = load_world_marker_specs(FIGURE8_WORLD)
    root_pose = load_world_model_pose(FIGURE8_WORLD, "navlab_iq_quad")
    local_offsets = _build_local_marker_offsets(specs, root_pose=root_pose)
    by_name = {spec.namespace: spec for spec in specs}

    moved_body = _apply_uav_pose(
        by_name["navlab_iq_quad_body"],
        current_pose=MarkerPose(x=1.0, y=2.0, z=0.2, yaw=0.0),
        local_offsets=local_offsets,
    )
    moved_rotor = _apply_uav_pose(
        by_name["navlab_iq_quad_rotor_0_disc"],
        current_pose=MarkerPose(x=1.0, y=2.0, z=0.2, yaw=0.0),
        local_offsets=local_offsets,
    )

    assert isclose(moved_body.pose.x, 1.0, abs_tol=1e-9)
    assert isclose(moved_body.pose.y, 2.0, abs_tol=1e-9)
    assert isclose(moved_rotor.pose.x, 1.13, abs_tol=1e-9)
    assert isclose(moved_rotor.pose.y, 1.78, abs_tol=1e-9)


def test_obstacle_marker_stays_static_when_uav_pose_changes() -> None:
    specs = load_world_marker_specs(FIGURE8_WORLD)
    root_pose = load_world_model_pose(FIGURE8_WORLD, "navlab_iq_quad")
    local_offsets = _build_local_marker_offsets(specs, root_pose=root_pose)
    by_name = {spec.namespace: spec for spec in specs}

    obstacle = by_name["inner_right_island"]
    resolved = _apply_uav_pose(
        obstacle,
        current_pose=MarkerPose(x=2.5, y=-0.7, z=1.0, yaw=1.2),
        local_offsets=local_offsets,
    )

    assert resolved is obstacle
    assert resolved.pose.x == 3.1
    assert resolved.pose.y == 0.0
    assert resolved.pose.z == obstacle.pose.z


def test_world_obstacle_boxes_exclude_uav_models() -> None:
    boxes = load_world_obstacle_boxes(FIGURE8_WORLD)
    assert [box.name for box in boxes] == [
        "outer_left_north_wall",
        "outer_left_south_wall",
        "outer_left_west_wall",
        "outer_right_north_wall",
        "outer_right_south_wall",
        "outer_right_east_wall",
        "inner_left_island",
        "inner_right_island",
    ]


def test_synthesize_planar_scan_matches_expected_front_face_distance() -> None:
    boxes = load_world_obstacle_boxes(FIGURE8_WORLD)

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
        origin_x=1.0,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
        contract=DEFAULT_SCAN_CONTRACT,
    )

    center_index = DEFAULT_SCAN_CONTRACT.beam_count // 2
    assert isclose(ranges[center_index], 1.9, abs_tol=1e-6)
    assert isclose(moved_ranges[center_index], 0.9, abs_tol=1e-6)


def test_synthesize_planar_scan_clamps_inside_obstacle_to_range_min() -> None:
    boxes = load_world_obstacle_boxes(FIGURE8_WORLD)

    ranges = synthesize_planar_scan(
        boxes,
        origin_x=3.0,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
        contract=DEFAULT_SCAN_CONTRACT,
    )

    center_index = DEFAULT_SCAN_CONTRACT.beam_count // 2
    assert ranges[center_index] == DEFAULT_SCAN_CONTRACT.range_min


def test_compute_forward_clearance_matches_obstacle_front_face_distance() -> None:
    boxes = load_world_obstacle_boxes(FIGURE8_WORLD)

    clearance = compute_forward_clearance(
        boxes,
        origin_x=0.0,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
    )
    moved_clearance = compute_forward_clearance(
        boxes,
        origin_x=1.8,
        origin_y=0.0,
        origin_z=0.1,
        yaw=0.0,
    )

    assert isclose(clearance or 0.0, 1.9, abs_tol=1e-6)
    assert isclose(moved_clearance or 0.0, 0.1, abs_tol=1e-6)
