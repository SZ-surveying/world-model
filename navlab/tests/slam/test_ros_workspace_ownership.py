from __future__ import annotations

from pathlib import Path


def test_slam_ros_packages_live_under_navlab_slam_without_src_wrapper() -> None:
    assert not Path("ros2_ws").exists()
    assert Path("navlab/slam/ros/localization/cartographer_indoor/package.xml").is_file()
    assert Path("navlab/slam/ros/bridges/external_nav_bridge/package.xml").is_file()
    assert Path("navlab/slam/ros/scenarios/indoor_bringup/package.xml").is_file()
    assert Path("navlab/slam/ros/sensors/imu_bridge/package.xml").is_file()


def test_companion_image_does_not_copy_slam_ros_workspace() -> None:
    dockerfile = Path("docker/Dockerfile.companion").read_text(encoding="utf-8")
    companion_stage = dockerfile.split("FROM remote-sitl-lab/ros-jazzy-base:latest AS navlab-companion", 1)[1].split(
        "FROM remote-sitl-lab/ros-jazzy-base:latest AS navlab-slam-cartographer",
        1,
    )[0]
    slam_stage = dockerfile.split("FROM remote-sitl-lab/ros-jazzy-base:latest AS navlab-slam-cartographer", 1)[1]

    assert "navlab/slam/ros" not in companion_stage
    assert "navlab/slam/ros" in slam_stage
    assert "--base-paths x3/src" in companion_stage
    assert "--base-paths navlab_slam_ros x3/src" in slam_stage
