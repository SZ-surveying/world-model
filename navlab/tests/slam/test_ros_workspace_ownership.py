from __future__ import annotations

from pathlib import Path


def test_slam_ros_packages_live_under_navlab_slam_without_src_wrapper() -> None:
    assert not Path("ros2_ws").exists()
    assert Path("navlab/slam/ros/localization/cartographer_indoor/package.xml").is_file()
    assert Path("navlab/slam/ros/bridges/external_nav_bridge/package.xml").is_file()
    assert Path("navlab/slam/ros/scenarios/indoor_bringup/package.xml").is_file()
    assert Path("navlab/slam/ros/sensors/imu_bridge/package.xml").is_file()


def test_companion_image_does_not_copy_slam_ros_workspace() -> None:
    companion_dockerfile = Path("docker/Dockerfile.companion").read_text(encoding="utf-8")
    slam_dockerfile = Path("docker/Dockerfile.slam").read_text(encoding="utf-8")

    assert "navlab-slam-cartographer" not in companion_dockerfile
    assert "navlab/slam/ros" not in companion_dockerfile
    assert "navlab/slam/ros" in slam_dockerfile
    assert "--base-paths x3/src" in companion_dockerfile
    assert "--base-paths navlab_slam_ros x3/src" in slam_dockerfile
