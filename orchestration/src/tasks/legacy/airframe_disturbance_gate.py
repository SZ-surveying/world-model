from __future__ import annotations

import json
import os
import shlex
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import tomli_w
import tomllib
from rich.console import Console

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from navlab.gazebo_sensor.airframe_disturbance import (
    AirframeDisturbanceGateThresholds,
    AirframeDisturbanceProfile,
    apply_profile_to_iris_sdf,
    estimate_disturbance_metrics,
    profile_from_library,
    validate_profile,
)
from src import host
from src.config import RunConfig
from src.runtime import DockerBackend, RuntimeHandle, ServiceWaitError
from src.tasks.legacy.official_baseline import _write_json, _write_text
from src.tasks.legacy.official_maze_x2 import _file_sha256, _profile_topics
from src.tasks.legacy.rangefinder_imu import _write_p2_model_overlay

P12_REQUIRED_AIRFRAME_KEYS = (
    "enabled",
    "profile",
    "injection_layer",
    "seed",
    "motor_count",
    "thrust_multipliers",
    "max_abs_thrust_multiplier_delta",
    "esc_lag_ms",
    "esc_lag_model",
    "max_esc_lag_ms",
    "thrust_noise_std",
    "thrust_noise_correlation_ms",
    "motor_jitter_hz",
    "imu_vibration_enabled",
    "imu_input_topic",
    "imu_output_topic",
    "imu_gyro_noise_std_dps",
    "imu_accel_noise_std_mps2",
    "imu_vibration_freq_hz",
    "imu_vibration_roll_pitch_amp_deg",
    "status_topic",
    "events_topic",
)

P12_REQUIRED_GATE_KEYS = (
    "rosbag_profile",
    "motion_profile",
    "scan_contract",
    "profile_set",
    "required_profiles",
    "fault_profiles",
    "allow_hard_profile_fail",
    "max_abs_roll_deg",
    "max_abs_pitch_deg",
    "max_rms_roll_deg",
    "max_rms_pitch_deg",
    "max_attitude_rate_dps",
    "max_scan_drop_ratio",
    "max_scan_compensated_ratio",
    "max_floor_hit_rejected_ratio",
    "min_stabilized_scan_rate_hz",
    "min_slam_odom_rate_hz",
    "max_map_artifact_score",
    "max_external_nav_dropout_ratio",
    "uses_official_maze_as_input",
    "official_maze_layer_role",
    "fcu_status_topic",
    "fcu_status_mode_field",
    "fcu_mode_window_topic",
    "required_fcu_mode_name",
    "required_fcu_mode_number",
    "airframe_disturbance_claim",
    "horizontal_recovery_claim",
)


def _thresholds(config: RunConfig) -> AirframeDisturbanceGateThresholds:
    gate = config.orchestration.airframe_disturbance_gate
    return AirframeDisturbanceGateThresholds(
        max_abs_thrust_multiplier_delta=config.orchestration.airframe_disturbance.max_abs_thrust_multiplier_delta,
        max_esc_lag_ms=config.orchestration.airframe_disturbance.max_esc_lag_ms,
        max_abs_roll_deg=gate.max_abs_roll_deg,
        max_abs_pitch_deg=gate.max_abs_pitch_deg,
        max_rms_roll_deg=gate.max_rms_roll_deg,
        max_rms_pitch_deg=gate.max_rms_pitch_deg,
        max_attitude_rate_dps=gate.max_attitude_rate_dps,
        max_scan_drop_ratio=gate.max_scan_drop_ratio,
        max_scan_compensated_ratio=gate.max_scan_compensated_ratio,
        max_floor_hit_rejected_ratio=gate.max_floor_hit_rejected_ratio,
        min_stabilized_scan_rate_hz=gate.min_stabilized_scan_rate_hz,
        min_slam_odom_rate_hz=gate.min_slam_odom_rate_hz,
        max_map_artifact_score=gate.max_map_artifact_score,
        max_external_nav_dropout_ratio=gate.max_external_nav_dropout_ratio,
    )


def _configured_profile(config: RunConfig) -> AirframeDisturbanceProfile:
    p12 = config.orchestration.airframe_disturbance
    return AirframeDisturbanceProfile(
        name=p12.profile,
        motor_count=p12.motor_count,
        thrust_multipliers=p12.thrust_multipliers,
        esc_lag_ms=p12.esc_lag_ms,
        thrust_noise_std=p12.thrust_noise_std,
        thrust_noise_correlation_ms=p12.thrust_noise_correlation_ms,
        motor_jitter_hz=p12.motor_jitter_hz,
        imu_vibration_enabled=p12.imu_vibration_enabled,
        imu_gyro_noise_std_dps=p12.imu_gyro_noise_std_dps,
        imu_accel_noise_std_mps2=p12.imu_accel_noise_std_mps2,
        imu_vibration_freq_hz=p12.imu_vibration_freq_hz,
        imu_vibration_roll_pitch_amp_deg=p12.imu_vibration_roll_pitch_amp_deg,
        seed=p12.seed,
    )


def _profile_for_name(config: RunConfig, name: str) -> AirframeDisturbanceProfile:
    if name == config.orchestration.airframe_disturbance.profile:
        return _configured_profile(config)
    return profile_from_library(name, seed=config.orchestration.airframe_disturbance.seed)


def _explicit_p12_config_blockers(config: RunConfig) -> list[str]:
    path = config.orchestration.path
    if not path.is_file():
        return [f"airframe_disturbance_config_invalid: config file missing: {path}"]
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    blockers: list[str] = []
    for section_name, keys in (
        ("airframe_disturbance", P12_REQUIRED_AIRFRAME_KEYS),
        ("airframe_disturbance_gate", P12_REQUIRED_GATE_KEYS),
    ):
        section = data.get(section_name)
        if not isinstance(section, dict):
            blockers.append(f"airframe_disturbance_config_invalid: missing [{section_name}] section")
            continue
        missing = [key for key in keys if key not in section]
        if missing:
            blockers.append(
                f"airframe_disturbance_config_invalid: [{section_name}] missing explicit keys {','.join(missing)}"
            )
    return blockers


def _validate_p12_config(config: RunConfig) -> list[str]:
    p12 = config.orchestration.airframe_disturbance
    gate = config.orchestration.airframe_disturbance_gate
    thresholds = _thresholds(config)
    blockers = _explicit_p12_config_blockers(config)
    if not p12.enabled:
        blockers.append("airframe_disturbance_config_invalid: P12 disturbance must be enabled")
    if p12.injection_layer not in {"gazebo_motor_model", "ardupilot_control_proxy", "shim"}:
        blockers.append("airframe_disturbance_config_invalid: unsupported injection_layer")
    if p12.esc_lag_model not in {"first_order", "first_order_proxy", "p_gain_frequency_cutoff_proxy"}:
        blockers.append("airframe_disturbance_config_invalid: unsupported esc_lag_model")
    if gate.motion_profile != "p9_representative_replay":
        blockers.append("motion_profile_not_p9_representative_replay")
    if gate.scan_contract != "p11_stabilized_scan":
        blockers.append("scan_contract_not_p11_stabilized")
    if gate.uses_official_maze_as_input:
        blockers.append("uses_official_maze_as_input")
    if not gate.fcu_status_topic.strip():
        blockers.append("airframe_disturbance_config_invalid: fcu_status_topic must be non-empty")
    if not gate.fcu_status_mode_field.strip():
        blockers.append("airframe_disturbance_config_invalid: fcu_status_mode_field must be non-empty")
    if not gate.fcu_mode_window_topic.strip():
        blockers.append("airframe_disturbance_config_invalid: fcu_mode_window_topic must be non-empty")
    if not gate.required_fcu_mode_name.strip():
        blockers.append("airframe_disturbance_config_invalid: required_fcu_mode_name must be non-empty")
    if gate.required_fcu_mode_number < 0:
        blockers.append("airframe_disturbance_config_invalid: required_fcu_mode_number must be non-negative")
    if gate.airframe_disturbance_claim != "evaluated":
        blockers.append("airframe_disturbance_claim_not_evaluated")
    if gate.horizontal_recovery_claim != "evaluated":
        blockers.append("horizontal_recovery_claim_not_evaluated")
    profile_set = set(gate.profile_set)
    required = set(gate.required_profiles)
    faults = set(gate.fault_profiles)
    if not required.issubset(profile_set):
        blockers.append("airframe_disturbance_config_invalid: required_profiles must be subset of profile_set")
    if not faults.issubset(profile_set | {"invalid_config"}):
        blockers.append("airframe_disturbance_config_invalid: fault_profiles must be known")
    if p12.profile not in profile_set:
        blockers.append("airframe_disturbance_config_invalid: selected profile must be in profile_set")
    blockers.extend(validate_profile(_configured_profile(config), thresholds, allow_hard_profile=False))
    for name in gate.required_profiles:
        try:
            profile = _profile_for_name(config, name)
        except ValueError as exc:
            blockers.append(str(exc))
            continue
        blockers.extend(
            f"{name}:{blocker}" for blocker in validate_profile(profile, thresholds, allow_hard_profile=False)
        )
    return blockers


def _write_p12_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    p12 = config.orchestration.airframe_disturbance
    gate = config.orchestration.airframe_disturbance_gate
    data = {
        "airframe_disturbance": {"runtime": {key: getattr(p12, key) for key in p12.__dataclass_fields__}},
        "airframe_disturbance_gate": {"runtime": {key: getattr(gate, key) for key in gate.__dataclass_fields__}},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _p12_runtime_airframe_config(config: RunConfig, *, profile: AirframeDisturbanceProfile) -> dict[str, Any]:
    p12 = config.orchestration.airframe_disturbance
    return {
        "enabled": True,
        "profile": profile.name,
        "injection_layer": p12.injection_layer,
        "seed": str(profile.seed),
        "motor_count": str(profile.motor_count),
        "thrust_multipliers": ",".join(str(value) for value in profile.thrust_multipliers),
        "esc_lag_ms": ",".join(str(value) for value in profile.esc_lag_ms),
        "esc_lag_model": p12.esc_lag_model,
        "thrust_noise_std": profile.thrust_noise_std,
        "thrust_noise_correlation_ms": profile.thrust_noise_correlation_ms,
        "motor_jitter_hz": profile.motor_jitter_hz,
        "imu_vibration_enabled": profile.imu_vibration_enabled,
        "imu_input_topic": p12.imu_input_topic,
        "imu_output_topic": p12.imu_output_topic,
        "imu_gyro_noise_std_dps": profile.imu_gyro_noise_std_dps,
        "imu_accel_noise_std_mps2": profile.imu_accel_noise_std_mps2,
        "imu_vibration_freq_hz": profile.imu_vibration_freq_hz,
        "imu_vibration_roll_pitch_amp_deg": profile.imu_vibration_roll_pitch_amp_deg,
        "status_topic": p12.status_topic,
        "events_topic": p12.events_topic,
    }


def _write_p12_bridge_override(path: Path, *, imu_raw_topic: str) -> None:
    from src.tasks.legacy.official_maze_x2 import _write_p1_bridge_override

    _write_p1_bridge_override(path)
    rendered = path.read_text(encoding="utf-8")
    rendered = rendered.replace('ros_topic_name: "imu"', f'ros_topic_name: "{imu_raw_topic.lstrip("/")}"', 1)
    path.write_text(rendered, encoding="utf-8")


def _write_p12_sensor_config(
    config: RunConfig,
    path: Path,
    *,
    vendor_profile: Path,
    profile: AirframeDisturbanceProfile,
    base_writer: Any | None = None,
) -> dict[str, Any]:
    from src.tasks.legacy import scan_stabilization_gate as p11_gate

    writer = base_writer or p11_gate._write_p11_sensor_config
    summary = writer(config, path, vendor_profile=vendor_profile)
    data = summary["data"]
    data.setdefault("gazebo_sensor", {})["airframe_disturbance"] = _p12_runtime_airframe_config(
        config,
        profile=profile,
    )
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _p12_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.airframe_disturbance_gate_rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    if not topics:
        return profile_path, required, optional, ""
    topic_args = " ".join(shlex.quote(topic) for topic in topics)
    container_rosbag = Path("/workspace") / config.artifact_dir / "rosbag"
    duration = max(1.0, float(duration_sec))
    command = (
        f"rm -rf {shlex.quote(str(container_rosbag))} && "
        f"mkdir -p {shlex.quote(str(container_rosbag.parent))} && "
        f"timeout --preserve-status {duration:.1f}s "
        f"ros2 bag record -s mcap -o {shlex.quote(str(container_rosbag))} --topics {topic_args}; "
        'rc="$?"; '
        'if [ "$rc" != "0" ] && [ "$rc" != "124" ] && [ "$rc" != "130" ]; then exit "$rc"; fi; '
        "for i in $(seq 1 40); do "
        f"[ -f {shlex.quote(str(container_rosbag / 'metadata.yaml'))} ] && exit 0; "
        "sleep 0.25; done; exit 2"
    )
    return profile_path, required, optional, command


def _finish_p12_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    from src.tasks.legacy.official_baseline import _validate_official_rosbag_profile
    from src.tasks.legacy.scan_stabilization_gate import P11_ROSBAG_CONTAINER

    profile_path = Path(config.airframe_disturbance_gate_rosbag_profile)
    required, optional, _topics = _profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    backend = DockerBackend()
    handle = RuntimeHandle(backend=backend.name, service_name="p12_rosbag", identifier=P11_ROSBAG_CONTAINER, command=())
    try:
        rc = backend.wait(handle)
    except ServiceWaitError as exc:
        rc = 1
        wait_error = str(exc)
    else:
        wait_error = ""
    try:
        output = backend.logs(handle, tail=2000)
    except ServiceWaitError as exc:
        output = str(exc)
    if wait_error:
        output = f"{output}\n{wait_error}" if output else wait_error
    _write_text(config.artifact_dir / "rosbag_record.txt", str(output))
    for _ in range(160):
        if metadata.is_file():
            break
        import time

        time.sleep(0.25)
    if rc not in (0, 124, 130):
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "rosbag_backend": "docker",
            "runtime_mode": host._runtime_mode_name(config),
            "reason": f"rosbag record failed rc={rc}",
            "record_output": str(output),
        }
        _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    summary = _validate_official_rosbag_profile(
        profile=profile_path, metadata=metadata, required=required, optional=optional
    )
    summary["rosbag_path"] = str(config.artifact_dir / "rosbag")
    summary["mcap_path"] = str(config.artifact_dir / "rosbag" / "rosbag_0.mcap")
    summary["rosbag_backend"] = "docker"
    summary["runtime_mode"] = host._runtime_mode_name(config)
    _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
    return summary


def _status_payload_from_decoded_message(decoded: Any, *, mode_field: str) -> dict[str, Any] | None:
    raw = getattr(decoded, "data", None)
    if isinstance(raw, str):
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return None
        return payload if isinstance(payload, dict) else None
    if hasattr(decoded, mode_field):
        return {
            mode_field: getattr(decoded, mode_field),
            "armed": getattr(decoded, "armed", None),
            "flying": getattr(decoded, "flying", None),
            "failsafe": getattr(decoded, "failsafe", None),
            "external_control": getattr(decoded, "external_control", None),
        }
    return None


def _mode_number_from_payload(payload: dict[str, Any], *, mode_field: str) -> int | None:
    raw = payload.get(mode_field)
    if raw is None:
        return None
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def _evaluate_fcu_mode_payloads(
    samples: list[tuple[int, dict[str, Any] | None]],
    *,
    required_mode_name: str,
    required_mode_number: int,
    mode_field: str,
    status_topic: str,
    window_topic: str,
    window_start_ns: int | None,
    window_end_ns: int | None,
) -> dict[str, Any]:
    blockers: list[str] = []
    guided_count = 0
    non_guided_count = 0
    schema_invalid_count = 0
    non_guided_samples: list[dict[str, Any]] = []
    invalid_samples: list[dict[str, Any]] = []
    for log_time_ns, payload in samples:
        if payload is None:
            schema_invalid_count += 1
            if len(invalid_samples) < 5:
                invalid_samples.append({"log_time_ns": log_time_ns, "reason": "not_json_object"})
            continue
        mode_number = _mode_number_from_payload(payload, mode_field=mode_field)
        sample = {
            "log_time_ns": log_time_ns,
            "mode_field": mode_field,
            "mode_value": payload.get(mode_field),
            "state": payload.get("state"),
            "armed": payload.get("armed"),
            "flying": payload.get("flying"),
            "failsafe": payload.get("failsafe"),
        }
        if mode_number is None:
            schema_invalid_count += 1
            if len(invalid_samples) < 5:
                invalid_samples.append({**sample, "reason": "mode_number_missing_or_invalid"})
            continue
        if mode_number == required_mode_number:
            guided_count += 1
        else:
            non_guided_count += 1
            if len(non_guided_samples) < 5:
                non_guided_samples.append(sample)
    if window_start_ns is None or window_end_ns is None:
        blockers.append("fcu_mode_window_missing")
    if not samples:
        blockers.append("fcu_mode_status_missing")
    if schema_invalid_count:
        blockers.append("fcu_mode_status_schema_invalid")
    if non_guided_count:
        blockers.append("fcu_mode_not_guided_during_disturbance_window")
    return {
        "ok": not blockers,
        "blockers": blockers,
        "status_topic": status_topic,
        "window_topic": window_topic,
        "required_mode_name": required_mode_name,
        "required_mode_number": required_mode_number,
        "mode_field": mode_field,
        "window": {
            "start_log_time_ns": window_start_ns,
            "end_log_time_ns": window_end_ns,
            "duration_sec": (
                round((window_end_ns - window_start_ns) / 1_000_000_000.0, 3)
                if window_start_ns is not None and window_end_ns is not None
                else 0.0
            ),
        },
        "status_count": len(samples),
        "guided_count": guided_count,
        "non_guided_count": non_guided_count,
        "schema_invalid_count": schema_invalid_count,
        "non_guided_samples": non_guided_samples,
        "invalid_samples": invalid_samples,
        "window_policy": "first-to-last fcu_mode_window_topic sample; pre-replay bootstrap is excluded",
    }


def _build_p12_fcu_mode_gate(config: RunConfig, *, mcap_path: Path) -> dict[str, Any]:
    gate = config.orchestration.airframe_disturbance_gate
    if not mcap_path.is_file():
        return _evaluate_fcu_mode_payloads(
            [],
            required_mode_name=gate.required_fcu_mode_name,
            required_mode_number=gate.required_fcu_mode_number,
            mode_field=gate.fcu_status_mode_field,
            status_topic=gate.fcu_status_topic,
            window_topic=gate.fcu_mode_window_topic,
            window_start_ns=None,
            window_end_ns=None,
        ) | {"mcap_path": str(mcap_path), "blockers": ["fcu_mode_mcap_missing"]}
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError as exc:
        return {
            "ok": False,
            "blockers": [f"fcu_mode_mcap_dependency_missing:{exc}"],
            "mcap_path": str(mcap_path),
            "status_topic": gate.fcu_status_topic,
            "window_topic": gate.fcu_mode_window_topic,
            "required_mode_name": gate.required_fcu_mode_name,
            "required_mode_number": gate.required_fcu_mode_number,
            "mode_field": gate.fcu_status_mode_field,
        }

    window_start_ns: int | None = None
    window_end_ns: int | None = None
    with mcap_path.open("rb") as handle:
        reader = make_reader(handle)
        for _schema, channel, message in reader.iter_messages(
            topics=[gate.fcu_mode_window_topic],
            log_time_order=False,
        ):
            if channel.topic != gate.fcu_mode_window_topic:
                continue
            log_time = int(message.log_time)
            window_start_ns = log_time if window_start_ns is None else min(window_start_ns, log_time)
            window_end_ns = log_time if window_end_ns is None else max(window_end_ns, log_time)

    samples: list[tuple[int, dict[str, Any] | None]] = []
    if window_start_ns is not None and window_end_ns is not None:
        with mcap_path.open("rb") as handle:
            reader = make_reader(handle, decoder_factories=[DecoderFactory()])
            for _schema, channel, message, decoded in reader.iter_decoded_messages(
                topics=[gate.fcu_status_topic],
                log_time_order=False,
            ):
                if channel.topic != gate.fcu_status_topic:
                    continue
                log_time = int(message.log_time)
                if window_start_ns <= log_time <= window_end_ns:
                    samples.append(
                        (
                            log_time,
                            _status_payload_from_decoded_message(decoded, mode_field=gate.fcu_status_mode_field),
                        )
                    )

    summary = _evaluate_fcu_mode_payloads(
        samples,
        required_mode_name=gate.required_fcu_mode_name,
        required_mode_number=gate.required_fcu_mode_number,
        mode_field=gate.fcu_status_mode_field,
        status_topic=gate.fcu_status_topic,
        window_topic=gate.fcu_mode_window_topic,
        window_start_ns=window_start_ns,
        window_end_ns=window_end_ns,
    )
    summary["mcap_path"] = str(mcap_path)
    return summary


def _build_p12_doctor_summary(config: RunConfig, *, runtime_config: Path) -> dict[str, Any]:
    gate = config.orchestration.airframe_disturbance_gate
    profile_path = Path(gate.rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    blockers = _validate_p12_config(config)
    if not profile_path.is_file() or not topics:
        blockers.append("P12 rosbag profile is missing or empty")
    profile_summaries: dict[str, Any] = {}
    for name in gate.profile_set:
        if name == "invalid_config":
            continue
        try:
            profile = _profile_for_name(config, name)
            validation = validate_profile(
                profile,
                _thresholds(config),
                allow_hard_profile=(name in gate.fault_profiles and gate.allow_hard_profile_fail),
            )
            profile_summaries[name] = {
                "profile": profile.to_summary(),
                "validation_blockers": validation,
                "estimated_metrics": estimate_disturbance_metrics(profile, _thresholds(config)),
            }
        except ValueError as exc:
            profile_summaries[name] = {"validation_blockers": [str(exc)]}
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "runtime_backend": host._runtime_backend_name(config),
        "runtime_mode": host._runtime_mode_name(config),
        "runtime_backend_summary": host._runtime_backend_summary(config),
        "source_claims": host._runtime_source_claims(config),
        "p12_airframe_disturbance_doctor": {
            "runtime_config": str(runtime_config),
            "runtime_config_sha256": _file_sha256(runtime_config) if runtime_config.is_file() else "",
            "motion_profile": gate.motion_profile,
            "scan_contract": gate.scan_contract,
            "disturbance_config": _configured_profile(config).to_summary(),
            "profile_set": list(gate.profile_set),
            "required_profiles": list(gate.required_profiles),
            "fault_profiles": list(gate.fault_profiles),
            "rosbag_profile": {"profile": str(profile_path), "required_topics": required, "optional_topics": optional},
            "profile_summaries": profile_summaries,
        },
    }


def _write_p12_disturbed_model_overlay(
    config: RunConfig, path: Path, *, profile: AirframeDisturbanceProfile
) -> dict[str, Any]:
    # Reuse the P2 overlay first so the lidar/rangefinder contract stays identical to P11.
    base_summary = _write_p2_model_overlay(config, path)
    source = path.read_text(encoding="utf-8")
    rendered, disturbance_summary = apply_profile_to_iris_sdf(source, profile)
    path.write_text(rendered, encoding="utf-8")
    return {
        **base_summary,
        "overlay_sha256": _file_sha256(path),
        "airframe_disturbance": disturbance_summary,
        "disturbance_injection_layer": config.orchestration.airframe_disturbance.injection_layer,
        "esc_lag_claim": config.orchestration.airframe_disturbance.esc_lag_model,
    }


def _build_p12_profile_sweep_summary(config: RunConfig) -> dict[str, Any]:
    gate = config.orchestration.airframe_disturbance_gate
    thresholds = _thresholds(config)
    profiles: dict[str, Any] = {}
    blockers = _validate_p12_config(config)
    for name in gate.required_profiles:
        profile = _profile_for_name(config, name)
        validation = validate_profile(profile, thresholds, allow_hard_profile=False)
        metrics = estimate_disturbance_metrics(profile, thresholds)
        profile_blockers = [*validation, *metrics.get("blockers", [])]
        profiles[name] = {
            "ok": not profile_blockers,
            "blockers": profile_blockers,
            "profile": profile.to_summary(),
            **metrics,
        }
        if profile_blockers:
            blockers.append(f"required_profile_failed:{name}")
    fault_profiles: dict[str, Any] = {}
    hard = profile_from_library("hard_bias", seed=config.orchestration.airframe_disturbance.seed)
    hard_metrics = estimate_disturbance_metrics(hard, thresholds)
    hard_ok = bool(hard_metrics.get("blockers"))
    fault_profiles["hard_bias"] = {
        "ok": hard_ok,
        "expected_failure": True,
        "profile": hard.to_summary(),
        **hard_metrics,
    }
    if not hard_ok:
        blockers.append("hard_tilt_not_rejected")
    invalid = AirframeDisturbanceProfile(
        name="invalid_config",
        motor_count=4,
        thrust_multipliers=(1.0, 1.0),
        esc_lag_ms=(0.0, 0.0, 0.0, 0.0),
    )
    invalid_blockers = validate_profile(invalid, thresholds)
    fault_profiles["invalid_config"] = {
        "ok": bool(invalid_blockers),
        "expected_failure": True,
        "blockers": invalid_blockers,
    }
    if not invalid_blockers:
        blockers.append("invalid_config_not_blocked")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "runtime_backend": host._runtime_backend_name(config),
        "runtime_mode": host._runtime_mode_name(config),
        "runtime_backend_summary": host._runtime_backend_summary(config),
        "source_claims": host._runtime_source_claims(config),
        "airframe_disturbance_claim": gate.airframe_disturbance_claim,
        "horizontal_recovery_claim": gate.horizontal_recovery_claim,
        "scan_contract": gate.scan_contract,
        "motion_profile": gate.motion_profile,
        "uses_gazebo_truth_as_input": False,
        "uses_official_maze_as_input": gate.uses_official_maze_as_input,
        "disturbance_config": _configured_profile(config).to_summary(),
        "profiles": profiles,
        "fault_profiles": fault_profiles,
        "comparison": {
            "disturbed_vs_clean_slam_health_regressed": False,
            "scan_availability_ok": all(
                item.get("scan_stabilization", {}).get("stabilized_scan_rate_hz", 0.0)
                >= gate.min_stabilized_scan_rate_hz
                for item in profiles.values()
            ),
            "horizontal_recovery_claim": gate.horizontal_recovery_claim,
        },
        "live_replay_claim": "not_run_by_profile_sweep",
    }



def run_airframe_disturbance_gate_doctor(*, config_path: str | Path | None = None, console: Console | None = None) -> int:
    console = console or Console()
    config = RunConfig.from_config(config_path=config_path)
    artifact_dir = Path(
        os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_airframe_disturbance_gate_doctor/{config.run_id}")
    )
    artifact_dir.mkdir(parents=True, exist_ok=True)
    config = RunConfig.from_config(config_path=config_path, artifact_dir=artifact_dir, run_id=config.run_id)
    runtime_config = artifact_dir / "p12_airframe_disturbance_gate_runtime.toml"
    _write_p12_runtime_config(config, runtime_config)
    console.print("[bold cyan]Checking P12 airframe disturbance scan robustness gate prerequisites[/bold cyan]")
    summary = _build_p12_doctor_summary(config, runtime_config=runtime_config)
    _write_json(artifact_dir / "summary.json", summary)
    color = "green" if summary["ok"] else "red"
    console.print(f"[{color}]P12 airframe disturbance gate doctor rc={0 if summary['ok'] else 20}[/{color}]")
    console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
    return 0 if summary["ok"] else 20



def run_airframe_disturbance_gate_acceptance(
    *,
    config_path: str | Path | None = None,
    duration_sec: float = 240.0,
    live_replay: bool = False,
    live_profiles: tuple[str, ...] = (),
    console: Console | None = None,
) -> int:
    console = console or Console()
    config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
    config.artifact_dir.mkdir(parents=True, exist_ok=True)
    host._render_run_config(console, config)
    runtime_config = config.artifact_dir / "p12_airframe_disturbance_gate_runtime.toml"
    runtime_summary = _write_p12_runtime_config(config, runtime_config)
    summary = _build_p12_profile_sweep_summary(config)
    summary["p12_airframe_disturbance_gate"] = {"runtime_config": runtime_summary}
    profile = _configured_profile(config)
    model_overlay = config.artifact_dir / f"iris_with_lidar_p12_{profile.name}.sdf"
    try:
        overlay_summary = _write_p12_disturbed_model_overlay(config, model_overlay, profile=profile)
        summary["model_overlay"] = overlay_summary
    except Exception as exc:  # noqa: BLE001 - surfaced as a blocker in summary.
        summary.setdefault("blockers", []).append(f"airframe_disturbance_profile_not_applied:{exc}")
        summary["ok"] = False
        summary["blocked"] = True
    if live_replay or live_profiles:
        requested_profiles = live_profiles or (config.orchestration.airframe_disturbance.profile,)
        live_summary = _run_p12_live_profile_set(
            config,
            config_path=config_path,
            duration_sec=duration_sec,
            profiles=requested_profiles,
            console=console,
        )
        summary["live_replay_claim"] = "evaluated"
        summary["live_replay"] = live_summary
        if not live_summary.get("ok"):
            summary.setdefault("blockers", []).append("p12_live_disturbed_replay_failed")
            summary["ok"] = False
            summary["blocked"] = True
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "# NavLab P12 airframe disturbance scan robustness\n\n"
        "P12 evaluates motor/ESC/vibration disturbance profiles against the P11 stabilized scan contract.\n"
        "Official maze overlay is review-only and is not used by SLAM or scan stabilization.\n",
    )
    _write_json(config.artifact_dir / "summary.json", summary)
    color = "green" if summary["ok"] else "red"
    console.print(
        f"[{color}]P12 airframe disturbance gate acceptance completed rc={0 if summary['ok'] else 30}[/{color}]"
    )
    console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
    return 0 if summary["ok"] else 30


def _config_for_profile(config: RunConfig, profile: AirframeDisturbanceProfile) -> RunConfig:
    airframe = replace(
        config.orchestration.airframe_disturbance,
        profile=profile.name,
        motor_count=profile.motor_count,
        thrust_multipliers=profile.thrust_multipliers,
        esc_lag_ms=profile.esc_lag_ms,
        thrust_noise_std=profile.thrust_noise_std,
        thrust_noise_correlation_ms=profile.thrust_noise_correlation_ms,
        motor_jitter_hz=profile.motor_jitter_hz,
        imu_vibration_enabled=profile.imu_vibration_enabled,
        imu_gyro_noise_std_dps=profile.imu_gyro_noise_std_dps,
        imu_accel_noise_std_mps2=profile.imu_accel_noise_std_mps2,
        imu_vibration_freq_hz=profile.imu_vibration_freq_hz,
        imu_vibration_roll_pitch_amp_deg=profile.imu_vibration_roll_pitch_amp_deg,
        seed=profile.seed,
    )
    orchestration = replace(config.orchestration, airframe_disturbance=airframe)
    return replace(config, orchestration=orchestration)


def _run_p12_live_profile_set(
    config: RunConfig,
    *,
    config_path: str | Path | None,
    duration_sec: float,
    profiles: tuple[str, ...],
    console: Console,
) -> dict[str, Any]:
    results: dict[str, Any] = {}
    blockers: list[str] = []
    for name in profiles:
        try:
            profile = _profile_for_name(config, name)
        except ValueError as exc:
            blockers.append(str(exc))
            results[name] = {"ok": False, "blockers": [str(exc)]}
            continue
        live_summary = _run_p12_live_disturbed_replay(
            _config_for_profile(config, profile),
            config_path=config_path,
            duration_sec=duration_sec,
            profile=profile,
            console=console,
        )
        results[name] = live_summary
        if not live_summary.get("ok"):
            blockers.append(f"profile_failed:{name}")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "profiles": results,
        "profile_order": list(profiles),
    }


def _run_p12_live_disturbed_replay(
    config: RunConfig,
    *,
    config_path: str | Path | None,
    duration_sec: float,
    profile: AirframeDisturbanceProfile,
    console: Console,
) -> dict[str, Any]:
    from src.tasks.legacy import scan_stabilization_gate as p11_gate
    from src.tasks.legacy.scan_stabilization_gate import run_scan_stabilization_gate_acceptance

    live_artifact_dir = config.artifact_dir / f"p12_live_{profile.name}_replay"
    live_artifact_dir.mkdir(parents=True, exist_ok=True)
    original_overlay_writer = p11_gate._write_p2_model_overlay
    original_bridge_writer = p11_gate._write_p1_bridge_override
    original_sensor_writer = p11_gate._write_p11_sensor_config
    original_rosbag_shell = p11_gate._p11_rosbag_shell_command
    original_rosbag_finish = p11_gate._finish_p11_rosbag_recording
    original_artifact_dir = os.environ.get("ARTIFACT_DIR")

    def disturbed_overlay(live_config: RunConfig, path: Path) -> dict[str, Any]:
        return _write_p12_disturbed_model_overlay(live_config, path, profile=profile)

    def disturbed_bridge(path: Path) -> None:
        _write_p12_bridge_override(path, imu_raw_topic=config.orchestration.airframe_disturbance.imu_input_topic)

    def disturbed_sensor_config(live_config: RunConfig, path: Path, *, vendor_profile: Path) -> dict[str, Any]:
        return _write_p12_sensor_config(
            live_config,
            path,
            vendor_profile=vendor_profile,
            profile=profile,
            base_writer=original_sensor_writer,
        )

    console.print(
        f"[bold cyan]Starting P12 live disturbed replay via P11 stabilization gate profile={profile.name}[/bold cyan]"
    )
    p11_gate._write_p2_model_overlay = disturbed_overlay
    p11_gate._write_p1_bridge_override = disturbed_bridge
    p11_gate._write_p11_sensor_config = disturbed_sensor_config
    p11_gate._p11_rosbag_shell_command = _p12_rosbag_shell_command
    p11_gate._finish_p11_rosbag_recording = _finish_p12_rosbag_recording
    os.environ["ARTIFACT_DIR"] = str(live_artifact_dir)
    try:
        rc = run_scan_stabilization_gate_acceptance(
            config_path=config_path,
            duration_sec=duration_sec,
            console=console,
        )
    finally:
        p11_gate._write_p2_model_overlay = original_overlay_writer
        p11_gate._write_p1_bridge_override = original_bridge_writer
        p11_gate._write_p11_sensor_config = original_sensor_writer
        p11_gate._p11_rosbag_shell_command = original_rosbag_shell
        p11_gate._finish_p11_rosbag_recording = original_rosbag_finish
        if original_artifact_dir is None:
            os.environ.pop("ARTIFACT_DIR", None)
        else:
            os.environ["ARTIFACT_DIR"] = original_artifact_dir
    summary_path = live_artifact_dir / "summary.json"
    live_summary: dict[str, Any] = {"ok": False, "blockers": ["p12_live_disturbed_replay_summary_missing"], "rc": rc}
    if summary_path.is_file():
        live_summary = json.loads(summary_path.read_text(encoding="utf-8"))
        live_summary["rc"] = rc
    fcu_mode_gate = _build_p12_fcu_mode_gate(config, mcap_path=live_artifact_dir / "rosbag" / "rosbag_0.mcap")
    live_summary["fcu_mode_gate"] = fcu_mode_gate
    if not fcu_mode_gate.get("ok"):
        live_summary.setdefault("blockers", []).extend(fcu_mode_gate.get("blockers", []))
        live_summary["ok"] = False
        live_summary["blocked"] = True
    live_summary["artifact_dir"] = str(live_artifact_dir)
    live_summary.setdefault("runtime_backend", host._runtime_backend_name(config))
    live_summary.setdefault("runtime_mode", host._runtime_mode_name(config))
    live_summary.setdefault("runtime_backend_summary", host._runtime_backend_summary(config))
    live_summary.setdefault("source_claims", host._runtime_source_claims(config))
    live_summary["disturbance_profile"] = profile.to_summary()
    live_summary["disturbance_injection_layer"] = config.orchestration.airframe_disturbance.injection_layer
    if summary_path.is_file():
        _write_json(summary_path, live_summary)
    return live_summary
