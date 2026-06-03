from __future__ import annotations

from pathlib import Path

from navlab.gazebo_sensor.runtime import (
    X2SensorLaunchConfig,
    build_emulator_command,
    build_scan_ideal_bridge_command,
    build_vendor_driver_command,
)


def _runtime_config() -> X2SensorLaunchConfig:
    return X2SensorLaunchConfig(
        scan_source="x2_virtual_serial",
        profile_path=Path("/workspace/profiles/x2-vendor-sim.yaml"),
        virtual_serial_link=Path("/tmp/navlab_x2"),
        scan_ideal_topic="/scan_ideal",
        scan_topic="/scan",
        status_topic="/sim/x2/status",
        scan_frequency_hz=7.0,
        sample_rate_hz=3000.0,
        range_min_m=0.1,
        range_max_m=8.0,
        static_range_m=1.5,
        auto_start=True,
        range_noise_stddev_m=0.02,
        dropout_rate=0.01,
        random_seed=123,
    )


def test_x2_sensor_runtime_bridges_gazebo_scan_ideal() -> None:
    command = build_scan_ideal_bridge_command(_runtime_config())

    assert command[:4] == ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
    assert "/scan_ideal@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" in command
    assert "override_frame_id:=laser_frame" in command


def test_x2_sensor_runtime_emulator_consumes_scan_ideal() -> None:
    command = build_emulator_command(_runtime_config())

    assert command == [command[0], "-m", "navlab.gazebo_sensor.cli"]
    assert "--scan-ideal-topic" not in command
    assert "--range-noise-stddev-m" not in command
    assert "--dropout-rate" not in command
    assert "--auto-start" not in command


def test_x2_sensor_runtime_vendor_driver_uses_profile() -> None:
    command = build_vendor_driver_command(_runtime_config())

    assert command == [
        "ros2",
        "run",
        "ydlidar_ros2_driver",
        "ydlidar_ros2_driver_node",
        "--ros-args",
        "--params-file",
        "/workspace/profiles/x2-vendor-sim.yaml",
    ]
