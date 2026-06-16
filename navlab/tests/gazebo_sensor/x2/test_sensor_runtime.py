from __future__ import annotations

import threading
import time
from pathlib import Path

from navlab.sim.gazebo_sensor.runtime import (
    X2SensorLaunchConfig,
    build_down_rangefinder_bridge_command,
    build_down_rangefinder_projection_command,
    build_emulator_command,
    build_scan_ideal_bridge_command,
    build_scan_integrity_filter_command,
    build_scan_stabilization_filter_command,
    build_scan_time_normalizer_command,
    build_vendor_driver_command,
    wait_for_virtual_serial_link,
)
from navlab.sim.gazebo_sensor.scan_time_normalizer import monotonic_scan_stamp_ns


def _runtime_config() -> X2SensorLaunchConfig:
    return X2SensorLaunchConfig(
        scan_source="x2_virtual_serial",
        profile_path=Path("/workspace/profiles/x2-vendor-sim.yaml"),
        virtual_serial_link=Path("/tmp/navlab_x2"),
        scan_ideal_topic="/scan_ideal",
        vendor_scan_topic="/navlab/x2/vendor_scan",
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
        down_rangefinder_enabled=True,
        down_rangefinder_scan_ideal_topic="/rangefinder/down/scan_ideal",
        down_rangefinder_frame_id="rangefinder_down_frame",
        scan_integrity_enabled=False,
        scan_stabilization_enabled=False,
        airframe_disturbance_enabled=False,
    )


def test_x2_sensor_runtime_bridges_gazebo_scan_ideal() -> None:
    command = build_scan_ideal_bridge_command(_runtime_config())

    assert command[:4] == ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
    assert "/scan_ideal@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" in command
    assert "override_frame_id:=laser_frame" in command


def test_x2_sensor_runtime_emulator_consumes_scan_ideal() -> None:
    command = build_emulator_command(_runtime_config())

    assert command == [command[0], "-m", "navlab.sim.gazebo_sensor.cli"]
    assert "--scan-ideal-topic" not in command
    assert "--range-noise-stddev-m" not in command
    assert "--dropout-rate" not in command
    assert "--auto-start" not in command


def test_down_rangefinder_runtime_bridges_gazebo_scan() -> None:
    command = build_down_rangefinder_bridge_command(_runtime_config())

    assert command[:4] == ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
    assert "/rangefinder/down/scan_ideal@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" in command
    assert "override_frame_id:=rangefinder_down_frame" in command


def test_down_rangefinder_projection_runs_in_gazebo_sensor_runtime() -> None:
    command = build_down_rangefinder_projection_command()

    assert command == [command[0], "-m", "navlab.sim.gazebo_sensor.range_projection"]


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
        "-p",
        "use_sim_time:=true",
        "-r",
        "scan:=/navlab/x2/vendor_scan",
    ]


def test_x2_sensor_runtime_normalizes_vendor_scan_time() -> None:
    command = build_scan_time_normalizer_command()

    assert command == [command[0], "-m", "navlab.sim.gazebo_sensor.scan_time_normalizer"]


def test_scan_time_normalizer_uses_monotonic_fallback_for_zero_stamp() -> None:
    first = monotonic_scan_stamp_ns(preferred_ns=0, fallback_elapsed_sec=0.25, previous_ns=None)
    second = monotonic_scan_stamp_ns(preferred_ns=0, fallback_elapsed_sec=0.20, previous_ns=first)

    assert first == 1_250_000_000
    assert second == first + 1


def test_scan_time_normalizer_keeps_preferred_stamp_when_valid() -> None:
    assert monotonic_scan_stamp_ns(preferred_ns=42, fallback_elapsed_sec=1.0, previous_ns=None) == 42
    assert monotonic_scan_stamp_ns(preferred_ns=40, fallback_elapsed_sec=1.0, previous_ns=42) == 43


def test_scan_time_normalizer_advances_by_scan_duration_when_clock_repeats() -> None:
    first = monotonic_scan_stamp_ns(preferred_ns=1_000_000_000, fallback_elapsed_sec=1.0, previous_ns=None)
    second = monotonic_scan_stamp_ns(
        preferred_ns=1_000_000_000,
        fallback_elapsed_sec=1.0,
        previous_ns=first,
        min_increment_ns=142_000_000,
    )

    assert second == first + 142_000_000


def test_x2_sensor_runtime_can_launch_scan_integrity_filter() -> None:
    command = build_scan_integrity_filter_command()

    assert command == [command[0], "-m", "navlab.sim.gazebo_sensor.scan_integrity"]


def test_x2_sensor_runtime_can_launch_scan_stabilization_filter() -> None:
    command = build_scan_stabilization_filter_command()

    assert command == [command[0], "-m", "navlab.sim.gazebo_sensor.scan_stabilization"]


def test_x2_sensor_runtime_waits_for_virtual_serial_link(tmp_path: Path) -> None:
    link = tmp_path / "navlab_x2"

    def create_link() -> None:
        time.sleep(0.05)
        link.write_text("ready", encoding="utf-8")

    thread = threading.Thread(target=create_link)
    thread.start()
    wait_for_virtual_serial_link(link, timeout_sec=1.0)
    thread.join(timeout=1.0)

    assert link.exists()


def test_x2_sensor_runtime_wait_for_virtual_serial_link_times_out(tmp_path: Path) -> None:
    missing = tmp_path / "missing"

    try:
        wait_for_virtual_serial_link(missing, timeout_sec=0.01)
    except TimeoutError as exc:
        assert str(missing) in str(exc)
    else:
        raise AssertionError("expected virtual serial wait to time out")
