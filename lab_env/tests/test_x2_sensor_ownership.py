from __future__ import annotations

import re
from pathlib import Path

from lab_env.sim.sensors.x2.config import X2SensorRuntimeConfig


def test_x2_sensor_code_lives_under_sensor_package() -> None:
    assert Path("lab_env/sim/sensors/x2/protocol.py").is_file()
    assert Path("lab_env/sim/sensors/x2/config.py").is_file()
    assert Path("lab_env/sim/sensors/x2/emulator.py").is_file()
    assert Path("lab_env/sim/sensors/x2/cli.py").is_file()
    assert Path("lab_env/sim/sensors/x2/runtime.py").is_file()
    assert Path("lab_env/sim/sensors/x2/scan_source.py").is_file()
    assert not Path("lab_env/sim/perception/x2_protocol.py").exists()


def test_gazebo_sensor_docker_target_owns_vendor_driver_dependency() -> None:
    dockerfile = Path("docker/Dockerfile.companion").read_text(encoding="utf-8")

    assert "AS navlab-gazebo-sensor" in dockerfile
    assert "AS gazebo-sensor-python-builder" in dockerfile

    sensor_stage = dockerfile.split("FROM remote-sitl-lab/ros-jazzy-base:latest AS navlab-gazebo-sensor", 1)[1]
    earlier_stages = dockerfile.split("FROM remote-sitl-lab/ros-jazzy-base:latest AS navlab-gazebo-sensor", 1)[0]

    assert "third_party/YDLidar-SDK" in sensor_stage
    assert "third_party/ydlidar_ros2_driver" in sensor_stage
    assert "ros-jazzy-ros-gz-bridge" in sensor_stage
    assert "third_party/ydlidar_ros2_driver" not in earlier_stages


def test_vendor_driver_uses_jazzy_compatible_parameter_declarations() -> None:
    source = Path("third_party/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp").read_text(encoding="utf-8")

    assert not re.search(r'declare_parameter\("[^"]+"\)', source)


def test_x2_compose_environment_is_scoped_to_gazebo_sensor_service() -> None:
    compose = Path("compose/docker-compose.yaml").read_text(encoding="utf-8")
    before_sensor, sensor_and_after = compose.split("  gazebo-sensor:", 1)
    sensor_stage, after_sensor = sensor_and_after.split("  foxglove:", 1)

    assert "X2_VIRTUAL_SERIAL_LINK" in sensor_stage
    assert "X2_MODE" in sensor_stage
    assert "X2_ARTIFACT_DIR" in sensor_stage
    assert "X2_STATUS_TOPIC" in sensor_stage
    assert "X2_SCAN_IDEAL_TOPIC" in sensor_stage
    assert "X2_SCAN_SOURCE" in sensor_stage
    assert "start-gazebo-sensor.sh" in sensor_stage
    assert "X2_" not in before_sensor
    assert "X2_" not in after_sensor


def test_x2_sensor_runtime_config_reads_project_x2_protocol_section() -> None:
    config = X2SensorRuntimeConfig.load()

    assert config.virtual_serial_link.as_posix() == "/tmp/navlab_x2"
    assert config.status_topic == "/sim/x2/status"
    assert config.scan_topic == "/scan"
    assert config.scan_ideal_topic == "/scan_ideal"
    assert config.sample_rate_hz == 3000.0
    assert config.scan_frequency_hz == 7.0
    assert config.range_min_m == 0.1
    assert config.range_max_m == 8.0
    assert config.emulator_config().virtual_serial_link == config.virtual_serial_link
