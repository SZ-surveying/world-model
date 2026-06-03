from __future__ import annotations

from pathlib import Path

from lab_env.sim.sensors.x2.smoke import (
    X2DriverSmokeConfig,
    build_emulator_command,
    build_rosbag_record_command,
    build_vendor_driver_command,
)


def _smoke_config(tmp_path: Path) -> X2DriverSmokeConfig:
    return X2DriverSmokeConfig(
        artifact_dir=tmp_path / "artifacts",
        duration_sec=3.0,
        profile_path=Path("/workspace/profiles/x2-vendor-sim.yaml"),
        virtual_serial_link=Path("/tmp/navlab_x2"),
        status_topic="/sim/x2/status",
        scan_topic="/scan",
        scan_frequency_hz=7.0,
        sample_rate_hz=3000.0,
        range_min_m=0.1,
        range_max_m=8.0,
    )


def test_x2_driver_smoke_starts_emulator_with_static_ranges(tmp_path: Path) -> None:
    command = build_emulator_command(_smoke_config(tmp_path))

    assert command[:3] == [command[0], "-m", "lab_env.sim.sensors.x2.cli"]
    assert "--virtual-serial-link" in command
    assert "/tmp/navlab_x2" in command
    assert "--static-range-m" in command
    assert "1.5" in command
    assert "--auto-start" in command
    assert "--driver-smoke" not in command


def test_x2_driver_smoke_uses_vendor_driver_with_profile(tmp_path: Path) -> None:
    command = build_vendor_driver_command(_smoke_config(tmp_path))

    assert command == [
        "ros2",
        "run",
        "ydlidar_ros2_driver",
        "ydlidar_ros2_driver_node",
        "--ros-args",
        "--params-file",
        "/workspace/profiles/x2-vendor-sim.yaml",
    ]


def test_x2_driver_smoke_records_scan_and_status_topics(tmp_path: Path) -> None:
    config = _smoke_config(tmp_path)
    command = build_rosbag_record_command(config)

    assert command[:4] == ["ros2", "bag", "record", "-o"]
    assert str(config.artifact_dir / "rosbag") in command
    assert "/scan" in command
    assert "/sim/x2/status" in command
