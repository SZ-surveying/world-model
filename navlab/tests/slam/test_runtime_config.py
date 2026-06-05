from __future__ import annotations

import pytest

from navlab.slam.backends import SlamBackend, SlamBackendRegistry
from navlab.slam.config import RuntimeConfig
from navlab.slam.runtime import build_command


def test_slam_runtime_config_loads_cartographer_contract() -> None:
    config = RuntimeConfig.load("navlab/config.toml")

    assert config.backend == "cartographer"
    assert config.scan_topic == "/scan"
    assert config.imu_topic == "/imu"
    assert config.cartographer_odometry_topic == "/odometry"
    assert config.odom_topic == "/odom"
    assert config.external_nav_input_odom_topic == "/odom"
    assert config.gazebo_truth_odom_topic == "/gazebo/truth/odom"
    assert config.uses_diagnostic_truth_for_external_nav is False


def test_cartographer_backend_generates_launch_command_from_runtime_config() -> None:
    command = build_command(config_path="navlab/config.toml")

    assert command[:4] == ["ros2", "launch", "navlab_slam_bringup", "navlab_slam_bringup.launch.py"]
    assert "launch_fake_odom:=false" in command
    assert "launch_cartographer_backend:=true" in command
    assert "publish_placeholder_odom:=false" in command
    assert "scan_topic:=/scan" in command
    assert "imu_topic:=/imu" in command
    assert "cartographer_odometry_topic:=/odometry" in command
    assert "odom_topic:=/odom" in command
    assert "slam_status_topic:=/navlab/slam/status" in command
    assert "imu_source_topic:=/ap/imu/experimental/data" in command
    assert "external_nav_input_odom_topic:=/odom" in command
    assert "external_nav_input_odom_topic:=/gazebo/truth/odom" not in command


def test_slam_backend_registry_exposes_cartographer() -> None:
    assert "cartographer" in SlamBackendRegistry.names()
    assert SlamBackendRegistry.create("cartographer").BACKEND_NAME == "cartographer"


def test_slam_backend_registry_requires_backend_name() -> None:
    class MissingNameBackend(SlamBackend):
        def command(self, config: RuntimeConfig) -> list[str]:
            return []

    with pytest.raises(ValueError, match="MissingNameBackend must define BACKEND_NAME"):
        SlamBackendRegistry.register(MissingNameBackend)


def test_slam_backend_registry_rejects_duplicate_backend_name() -> None:
    class DuplicateCartographerBackend(SlamBackend):
        BACKEND_NAME = "cartographer"

        def command(self, config: RuntimeConfig) -> list[str]:
            return []

    with pytest.raises(ValueError, match="SLAM backend 'cartographer' is already registered"):
        SlamBackendRegistry.register(DuplicateCartographerBackend)
