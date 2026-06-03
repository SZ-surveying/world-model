from __future__ import annotations

from pathlib import Path

import pytest

from navlab.gazebo_sensor.config import load_x2_protocol_config


def test_x2_protocol_config_loads_vendor_defaults() -> None:
    config = load_x2_protocol_config()

    assert config.enabled.value is False
    assert config.profile.value == "profiles/x2-vendor-sim.yaml"
    assert config.virtual_serial_link.value == "/tmp/navlab_x2"
    assert config.scan_ideal_topic.value == "/scan_ideal"
    assert config.scan_topic.value == "/scan"
    assert config.status_topic.value == "/sim/x2/status"
    assert config.sample_rate_hz.value == 3000.0
    assert config.scan_frequency_hz.value == 7.0
    assert config.scan_frequency_min_hz.value == 4.0
    assert config.scan_frequency_max_hz.value == 8.0
    assert config.range_min_m.value == 0.1
    assert config.range_max_m.value == 8.0


def test_x2_vendor_profile_matches_protocol_baseline() -> None:
    yaml = pytest.importorskip("yaml")

    profile_path = Path("profiles/x2-vendor-sim.yaml")
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    params = profile["ydlidar_ros2_driver_node"]["ros__parameters"]

    assert params["port"] == "/tmp/navlab_x2"
    assert params["lidar_type"] == 1
    assert params["sample_rate"] == 3
    assert params["frequency"] == 7.0
    assert params["range_min"] == 0.1
    assert params["range_max"] == 8.0
    assert params["isSingleChannel"] is True
    assert params["intensity"] is False
