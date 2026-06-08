from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from src.config import RunConfig
from src.tasks.legacy import airframe_disturbance_gate as p12


def test_p12_config_loads_airframe_disturbance_sections() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260608_000000")

    assert config.orchestration.airframe_disturbance.profile == "nominal_realistic"
    assert config.orchestration.airframe_disturbance.thrust_multipliers == (0.97, 1.03, 1.0, 0.98)
    assert config.orchestration.airframe_disturbance.esc_lag_ms == (20.0, 35.0, 25.0, 45.0)
    assert config.orchestration.airframe_disturbance.esc_lag_model == "first_order"
    assert config.orchestration.airframe_disturbance.imu_input_topic == "/navlab/imu/raw"
    assert config.orchestration.airframe_disturbance_gate.scan_contract == "p11_stabilized_scan"
    assert config.orchestration.airframe_disturbance_gate.fcu_status_topic == "/ap/v1/status"
    assert config.orchestration.airframe_disturbance_gate.fcu_status_mode_field == "mode"
    assert config.orchestration.airframe_disturbance_gate.fcu_mode_window_topic == "/navlab/exploration/status"
    assert config.orchestration.airframe_disturbance_gate.required_fcu_mode_name == "GUIDED"
    assert config.orchestration.airframe_disturbance_gate.required_fcu_mode_number == 4
    assert "vibration" in config.orchestration.airframe_disturbance_gate.required_profiles
    assert config.airframe_disturbance_gate_rosbag_profile == "profiles/navlab-airframe-disturbance-gate-rosbag-topics.txt"


def test_p12_doctor_summary_is_green_for_default_config(tmp_path: Path) -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260608_000000", artifact_dir=tmp_path)
    runtime = tmp_path / "p12_runtime.toml"
    p12._write_p12_runtime_config(config, runtime)

    summary = p12._build_p12_doctor_summary(config, runtime_config=runtime)

    assert summary["ok"] is True
    assert summary["p12_airframe_disturbance_doctor"]["motion_profile"] == "p9_representative_replay"
    assert summary["p12_airframe_disturbance_doctor"]["scan_contract"] == "p11_stabilized_scan"


def test_p12_config_validation_blocks_invalid_motor_array() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260608_000000")
    bad_airframe = replace(config.orchestration.airframe_disturbance, thrust_multipliers=(1.0, 1.0))
    bad = replace(config, orchestration=replace(config.orchestration, airframe_disturbance=bad_airframe))

    blockers = p12._validate_p12_config(bad)

    assert "airframe_disturbance_config_invalid: thrust_multipliers length must match motor_count" in blockers


def test_p12_profile_sweep_keeps_hard_bias_as_expected_fault() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260608_000000")

    summary = p12._build_p12_profile_sweep_summary(config)

    assert summary["profiles"]["clean"]["ok"] is True
    assert summary["profiles"]["nominal_realistic"]["ok"] is True
    assert summary["fault_profiles"]["invalid_config"]["ok"] is True
    assert summary["fault_profiles"]["hard_bias"]["expected_failure"] is True


def test_p12_fcu_mode_gate_accepts_guided_window() -> None:
    summary = p12._evaluate_fcu_mode_payloads(
        [
            (100, {"mode_number": 4, "state": "streaming", "armed": True}),
            (200, {"mode_number": "4", "state": "streaming", "armed": True}),
        ],
        required_mode_name="GUIDED",
        required_mode_number=4,
        mode_field="mode_number",
        status_topic="/ap/v1/status",
        window_topic="/navlab/exploration/status",
        window_start_ns=50,
        window_end_ns=250,
    )

    assert summary["ok"] is True
    assert summary["guided_count"] == 2
    assert summary["non_guided_count"] == 0
    assert summary["schema_invalid_count"] == 0


def test_p12_fcu_mode_gate_blocks_non_guided_window() -> None:
    summary = p12._evaluate_fcu_mode_payloads(
        [
            (100, {"mode_number": 4, "state": "streaming", "armed": True}),
            (200, {"mode_number": 6, "state": "streaming", "armed": True}),
        ],
        required_mode_name="GUIDED",
        required_mode_number=4,
        mode_field="mode_number",
        status_topic="/ap/v1/status",
        window_topic="/navlab/exploration/status",
        window_start_ns=50,
        window_end_ns=250,
    )

    assert summary["ok"] is False
    assert "fcu_mode_not_guided_during_disturbance_window" in summary["blockers"]
    assert summary["non_guided_samples"][0]["mode_value"] == 6


def test_p12_fcu_mode_gate_blocks_missing_mode_number() -> None:
    summary = p12._evaluate_fcu_mode_payloads(
        [(100, {"state": "streaming", "armed": True})],
        required_mode_name="GUIDED",
        required_mode_number=4,
        mode_field="mode_number",
        status_topic="/ap/v1/status",
        window_topic="/navlab/exploration/status",
        window_start_ns=50,
        window_end_ns=250,
    )

    assert summary["ok"] is False
    assert "fcu_mode_status_schema_invalid" in summary["blockers"]
    assert summary["schema_invalid_count"] == 1


def test_p12_sensor_config_enables_airframe_runtime_and_raw_imu_bridge(tmp_path: Path) -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260608_000000", artifact_dir=tmp_path)
    vendor = tmp_path / "vendor.yaml"
    vendor.write_text("x: y\n", encoding="utf-8")
    sensor_config = tmp_path / "sensor.toml"
    bridge = tmp_path / "bridge.yaml"
    profile = p12._profile_for_name(config, "vibration")

    p12._write_p12_bridge_override(bridge, imu_raw_topic=config.orchestration.airframe_disturbance.imu_input_topic)
    summary = p12._write_p12_sensor_config(config, sensor_config, vendor_profile=vendor, profile=profile)

    airframe = summary["data"]["gazebo_sensor"]["airframe_disturbance"]
    assert 'ros_topic_name: "navlab/imu/raw"' in bridge.read_text(encoding="utf-8")
    assert airframe["enabled"] is True
    assert airframe["profile"] == "vibration"
    assert airframe["imu_input_topic"] == "/navlab/imu/raw"
    assert airframe["imu_output_topic"] == "/imu"
    assert airframe["imu_vibration_enabled"] is True
