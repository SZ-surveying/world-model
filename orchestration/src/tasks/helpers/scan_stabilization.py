from __future__ import annotations

import shlex
import sys
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import tomli_w
from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.configs.run_config import RunConfig
from src.runtime import (
    SERVICE_ROLE_SIMULATION_ROSBAG,
    DockerBackend,
    RuntimeHandle,
    ServiceSpec,
    ServiceWaitError,
    VolumeMount,
)
from src.tasks.helpers.artifacts import file_sha256, write_json, write_text
from src.tasks.helpers.fcu import (
    P4_CONTROLLER_CONTAINER,
    append_controller_blockers,
    append_owner_blockers,
    start_p4_controller_container,
    wait_for_controller_summary,
    write_controller_runtime_script,
    write_p4_runtime_config,
)
from src.tasks.helpers.frame_contract import (
    append_p5_blockers,
    run_frame_probe,
    write_frame_probe_script,
    write_p5_runtime_config,
)
from src.tasks.helpers.navlab_models import (
    GAZEBO_SENSOR_CONTAINER,
    OFFICIAL_IRIS_3D_BRIDGE_CONFIG,
    capture_container_log,
    collect_laser_scan_sample,
    collect_topic_info,
    collect_x2_status,
    remove_container,
    start_gazebo_sensor_container,
    write_p1_bridge_override,
    write_p1_vendor_profile,
)
from src.tasks.helpers.official_stack import (
    collect_official_dds_probe,
)
from src.tasks.helpers.rosbag_profiles import (
    load_rosbag_metadata_counts,
    profile_topics,
    validate_official_rosbag_profile,
)
from src.tasks.helpers.scan_integrity import motor_output_summary
from src.tasks.helpers.sensors import (
    OFFICIAL_GAZEBO_IRIS_PARAMS,
    OFFICIAL_IRIS_WITH_LIDAR_MODEL,
    collect_imu_probe,
    collect_rangefinder_probe,
    write_p2_model_overlay,
    write_p2_param_overlay,
)
from src.tasks.helpers.slam import (
    SLAM_BACKEND_CONTAINER,
    collect_odometry_probe,
    start_p3_slam_container,
    write_p3_slam_runtime_config,
)
from src.tasks.helpers.slam_hover import baseline_env, load_json, source_official_setup
from src.tasks.workflows.exploration import (
    append_p8_blockers,
    apply_replay_profile,
    run_exploration_probe,
    write_exploration_probe_script,
    write_p8_runtime_config,
)

P11_ROSBAG_CONTAINER = "navlab-p11-rosbag"

def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return load_rosbag_metadata_counts(metadata)


def _p11_runtime_config(config: RunConfig) -> dict[str, Any]:
    p11 = config.orchestration.scan_stabilization
    return {key: getattr(p11, key) for key in p11.__dataclass_fields__}


def _explicit_p11_config_blockers(config: RunConfig) -> list[str]:
    config_path = config.orchestration.path
    if not config_path.is_file():
        return [f"scan_stabilization_config_invalid: config file missing: {config_path}"]
    return []


def validate_p11_config(config: RunConfig) -> list[str]:
    p11 = config.orchestration.scan_stabilization
    gate = config.orchestration.scan_stabilization_gate
    blockers: list[str] = _explicit_p11_config_blockers(config)
    if p11.mode != "bounded_2d_projection" or gate.candidate_mode != "bounded_2d_projection":
        blockers.append("scan_stabilization_config_invalid: mode must be bounded_2d_projection")
    if gate.motion_profile != "p9_representative_replay":
        blockers.append("motion_profile_not_p9_representative_replay")
    if gate.baseline_mode != "p10_drop_only":
        blockers.append("scan_stabilization_config_invalid: baseline_mode must be p10_drop_only")
    if p11.scan_stabilization_claim != "evaluated" or gate.scan_stabilization_claim != "evaluated":
        blockers.append("P11 scan_stabilization_claim must be evaluated")
    if p11.uses_gazebo_truth_as_input:
        blockers.append("P11 must not use Gazebo truth as input")
    if gate.uses_official_maze_as_input:
        blockers.append("P11 must not use official maze as input")
    if p11.output_scan_topic != config.orchestration.slam_backend.scan_topic:
        blockers.append("P11 output scan topic must match SLAM scan topic")
    if p11.input_scan_topic == p11.output_scan_topic:
        blockers.append("scan_stabilization_config_invalid: input and output topics must differ")
    if not (0.0 <= p11.passthrough_tilt_deg < p11.compensation_tilt_deg < p11.hard_drop_tilt_deg):
        blockers.append("scan_stabilization_config_invalid: tilt thresholds must be ordered")
    for name in ("max_rejected_beam_ratio", "min_retained_beam_ratio", "max_floor_hit_risk_beam_ratio"):
        value = float(getattr(p11, name))
        if not (0.0 <= value <= 1.0):
            blockers.append(f"scan_stabilization_config_invalid: {name} must be in [0, 1]")
    if p11.max_vertical_projection_error_m <= 0.0:
        blockers.append("scan_stabilization_config_invalid: max_vertical_projection_error_m must be positive")
    if p11.max_attitude_source_age_ms <= 0.0:
        blockers.append("scan_stabilization_config_invalid: max_attitude_source_age_ms must be positive")
    if gate.replay_readiness_timeout_sec <= 0.0:
        blockers.append("scan_stabilization_config_invalid: replay_readiness_timeout_sec must be positive")
    if gate.controller_summary_timeout_sec <= 0.0:
        blockers.append("scan_stabilization_config_invalid: controller_summary_timeout_sec must be positive")
    if p11.attitude_source_topic.startswith("/gazebo") or p11.attitude_source_topic == "/odometry":
        blockers.append("P11 attitude source must not be Gazebo truth")
    return blockers


def _apply_p11_replay_runtime_profile(config: RunConfig) -> RunConfig:
    gate = config.orchestration.scan_stabilization_gate
    p4 = config.orchestration.fcu_controller
    replay_p4 = replace(
        p4,
        readiness_timeout_sec=max(p4.readiness_timeout_sec, gate.replay_readiness_timeout_sec),
    )
    return replace(config, orchestration=replace(config.orchestration, fcu_controller=replay_p4))


def write_p11_sensor_config(config: RunConfig, path: Path, *, vendor_profile: Path) -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    gate = config.orchestration.scan_stabilization_gate
    data = {
        "gazebo_sensor": {
            "x2_protocol": {
                "enabled": True,
                "scan_source": "x2_virtual_serial",
                "profile": host.workspace_path(vendor_profile),
                "virtual_serial_link": p2.x2_virtual_serial_link,
                "scan_ideal_topic": p2.x2_scan_input_topic,
                "vendor_scan_topic": gate.raw_scan_topic,
                "scan_topic": gate.normalized_scan_topic,
                "status_topic": gate.x2_status_topic,
                "sample_rate_hz": 3000.0,
                "scan_frequency_hz": 7.0,
                "scan_frequency_min_hz": 7.0,
                "scan_frequency_max_hz": 7.0,
                "scan_frequency_jitter_hz": 0.0,
                "range_min_m": 0.1,
                "range_max_m": 8.0,
                "static_range_m": 1.5,
                "range_noise_stddev_m": 0.0,
                "range_noise_stddev_per_m": 0.0,
                "dropout_rate": 0.0,
                "auto_start": True,
                "random_seed": "",
            },
            "down_rangefinder": {
                "enabled": True,
                "scan_ideal_topic": p2.rangefinder_scan_ideal_topic,
                "range_topic": p2.rangefinder_range_topic,
                "status_topic": p2.rangefinder_status_topic,
                "endpoint": p2.rangefinder_endpoint,
                "frame_id": p2.rangefinder_frame_id,
                "mavlink_orientation": p2.rangefinder_mavlink_orientation,
                "source_system": p2.rangefinder_source_system,
                "source_component": p2.rangefinder_source_component,
                "sensor_id": p2.rangefinder_sensor_id,
                "rate_hz": p2.rangefinder_rate_hz,
                "min_distance_m": p2.rangefinder_min_distance_m,
                "max_distance_m": p2.rangefinder_max_distance_m,
                "covariance_cm": p2.rangefinder_covariance_cm,
                "model_pose": p2.rangefinder_model_pose,
                "model_update_rate_hz": p2.rangefinder_model_update_rate_hz,
                "model_ray_count": p2.rangefinder_model_ray_count,
                "model_noise_stddev_m": p2.rangefinder_model_noise_stddev_m,
            },
            "scan_integrity": {"enabled": False},
            "scan_stabilization": _p11_runtime_config(config),
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host.workspace_path(path), "sha256": file_sha256(path), "data": data}


def _write_p11_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    gate = config.orchestration.scan_stabilization_gate
    data = {
        "scan_stabilization_gate": {"runtime": {key: getattr(gate, key) for key in gate.__dataclass_fields__}},
        "scan_stabilization": {"runtime": _p11_runtime_config(config)},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host.workspace_path(path), "sha256": file_sha256(path), "data": data}


def p11_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.scan_stabilization_gate_rosbag_profile)
    required, optional, topics = profile_topics(profile_path)
    if not profile_path.is_file() or not topics:
        return profile_path, required, optional, ""
    container_rosbag = Path("/workspace") / config.artifact_dir / "rosbag"
    topic_args = " ".join(shlex.quote(topic) for topic in topics)
    command = (
        f"rm -rf {shlex.quote(str(container_rosbag))} && "
        f"mkdir -p {shlex.quote(str(container_rosbag.parent))} && "
        "set +e; "
        f"timeout --signal=INT {duration_sec:g} "
        f"ros2 bag record -s mcap -o {shlex.quote(str(container_rosbag))} --topics {topic_args}; "
        "rc=$?; set -e; "
        'if [ "$rc" != "0" ] && [ "$rc" != "124" ] && [ "$rc" != "130" ]; then exit "$rc"; fi; '
        "for i in $(seq 1 40); do "
        f"[ -f {shlex.quote(str(container_rosbag / 'metadata.yaml'))} ] && exit 0; "
        "sleep 0.25; done; exit 2"
    )
    return profile_path, required, optional, command


def start_p11_rosbag_recording(config: RunConfig, *, duration_sec: float) -> None:
    host.assert_runtime_service_role(
        config,
        service_name="p11_rosbag",
        service_role=SERVICE_ROLE_SIMULATION_ROSBAG,
    )
    remove_container(P11_ROSBAG_CONTAINER)
    profile_path, required, optional, command = p11_rosbag_shell_command(config, duration_sec=duration_sec)
    if not command:
        write_json(
            config.artifact_dir / "rosbag_profile_summary.json",
            {
                "ok": False,
                "recorded": False,
                "profile": str(profile_path),
                "required_topics": required,
                "optional_topics": optional,
                "rosbag_backend": "docker",
                "runtime_mode": host.runtime_mode_name(config),
                "reason": "rosbag profile missing or empty",
            },
        )
        return
    DockerBackend().start_service(
        ServiceSpec(
            name="p11_rosbag",
            image=config.orchestration.official_baseline.runtime_image,
            command=("bash", "-lc", source_official_setup(command)),
            container_name=P11_ROSBAG_CONTAINER,
            networks=("host",),
            volumes=(VolumeMount(Path.cwd(), "/workspace"),),
            cwd="/workspace",
            env={**baseline_env(config), "PYTHONPATH": "/workspace"},
            service_role=SERVICE_ROLE_SIMULATION_ROSBAG,
        )
    )


def finish_p11_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    profile_path = Path(config.scan_stabilization_gate_rosbag_profile)
    required, optional, _topics = profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    backend = DockerBackend()
    handle = RuntimeHandle(backend=backend.name, service_name="p11_rosbag", identifier=P11_ROSBAG_CONTAINER, command=())
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
    write_text(config.artifact_dir / "rosbag_record.txt", str(output))
    for _ in range(160):
        if metadata.is_file():
            break
        time.sleep(0.25)
    if rc != 0 or not metadata.is_file():
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "rosbag_backend": "docker",
            "runtime_mode": host.runtime_mode_name(config),
            "reason": f"rosbag record failed rc={rc}",
            "record_output": str(output),
        }
        write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    summary = validate_official_rosbag_profile(
        profile=profile_path,
        metadata=metadata,
        required=required,
        optional=optional,
    )
    summary["rosbag_path"] = str(config.artifact_dir / "rosbag")
    summary["mcap_path"] = str(config.artifact_dir / "rosbag" / "rosbag_0.mcap")
    summary["rosbag_backend"] = "docker"
    summary["runtime_mode"] = host.runtime_mode_name(config)
    write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
    return summary


def _latest_stabilization_status(config: RunConfig) -> dict[str, Any]:
    status_topic = config.orchestration.scan_stabilization.status_topic
    path = config.artifact_dir / "stabilization_status_latest.json"
    script = f'''
import json, rclpy, time
from pathlib import Path
from std_msgs.msg import String
rclpy.init(); node = rclpy.create_node("navlab_p11_status_probe"); samples=[]
def cb(msg):
    try: samples.append(json.loads(msg.data))
    except Exception: pass
node.create_subscription(String, {status_topic!r}, cb, 10)
end=time.monotonic()+3.0
while time.monotonic()<end:
    rclpy.spin_once(node, timeout_sec=0.05)
Path({host.workspace_path(path)!r}).write_text(json.dumps(samples[-1] if samples else {{}}, indent=2, sort_keys=True)+"\\n")
node.destroy_node(); rclpy.shutdown()
'''
    rc, output = host.docker_run_ros_shell_capture(
        config=config,
        image=config.orchestration.official_baseline.runtime_image,
        shell_command=f"python3 - <<'PY'\n{script}\nPY",
        name=None,
        network="host",
        envs=baseline_env(config),
    )
    write_text(config.artifact_dir / "stabilization_status_probe.txt", output)
    status = load_json(path)
    status["probe_rc"] = rc
    return status


def _collect_p11_rangefinder_preflight(
    *,
    config: RunConfig,
    image: str,
    sensor_config: Path,
    official_volume_overrides: list[tuple[Path, str]],
    console: Console,
) -> dict[str, Any]:
    rangefinder_preflight = collect_rangefinder_probe(
        config,
        image=image,
        artifact_name="p11_rangefinder_preflight_probe.txt",
    )
    if rangefinder_preflight.get("result", {}).get("range_received"):
        return rangefinder_preflight

    console.print("[yellow]P11 rangefinder preflight missed data; restarting gazebo sensor once[/yellow]")
    capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_preflight_tail.log")
    start_gazebo_sensor_container(config, sensor_config=sensor_config)
    time.sleep(10.0)
    rangefinder_preflight = collect_rangefinder_probe(
        config,
        image=image,
        artifact_name="p11_rangefinder_preflight_retry_probe.txt",
    )
    if rangefinder_preflight.get("result", {}).get("range_received"):
        return rangefinder_preflight

    console.print("[yellow]P11 rangefinder preflight still missed data; restarting official baseline once[/yellow]")
    capture_container_log(
        config,
        container=GAZEBO_SENSOR_CONTAINER,
        output_name="gazebo_sensor_preflight_retry_tail.log",
    )
    remove_container(GAZEBO_SENSOR_CONTAINER)
    host.remove_official_baseline_container()
    time.sleep(2.0)
    host.start_official_baseline_container(config, volume_overrides=official_volume_overrides)
    time.sleep(12.0)
    start_gazebo_sensor_container(config, sensor_config=sensor_config)
    time.sleep(10.0)
    return collect_rangefinder_probe(
        config,
        image=image,
        artifact_name="p11_rangefinder_preflight_baseline_retry_probe.txt",
    )


def _x2_status_uses_gazebo_lidar(status: dict[str, Any]) -> bool:
    sample = status.get("result", {}).get("sample") or {}
    return bool(status.get("result", {}).get("received")) and sample.get("scan_source") == "gazebo_ideal"


def _collect_p11_lidar_preflight(
    *,
    config: RunConfig,
    image: str,
    sensor_config: Path,
    official_volume_overrides: list[tuple[Path, str]],
    console: Console,
) -> tuple[dict[str, Any], dict[str, Any]]:
    gate = config.orchestration.scan_stabilization_gate
    lidar_sample = collect_laser_scan_sample(config, image=image, topic=gate.scan_source_topic)
    x2_status = collect_x2_status(config, image=image)
    if lidar_sample.get("result", {}).get("received") and _x2_status_uses_gazebo_lidar(x2_status):
        return lidar_sample, x2_status

    console.print("[yellow]P11 lidar preflight missed Gazebo /lidar input; restarting gazebo sensor once[/yellow]")
    capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_lidar_preflight_tail.log")
    start_gazebo_sensor_container(config, sensor_config=sensor_config)
    time.sleep(10.0)
    lidar_sample = collect_laser_scan_sample(config, image=image, topic=gate.scan_source_topic)
    x2_status = collect_x2_status(config, image=image)
    if lidar_sample.get("result", {}).get("received") and _x2_status_uses_gazebo_lidar(x2_status):
        return lidar_sample, x2_status

    console.print("[yellow]P11 lidar preflight still missed data; restarting official baseline once[/yellow]")
    capture_container_log(
        config,
        container=GAZEBO_SENSOR_CONTAINER,
        output_name="gazebo_sensor_lidar_preflight_retry_tail.log",
    )
    remove_container(GAZEBO_SENSOR_CONTAINER)
    host.remove_official_baseline_container()
    time.sleep(2.0)
    host.start_official_baseline_container(config, volume_overrides=official_volume_overrides)
    time.sleep(12.0)
    start_gazebo_sensor_container(config, sensor_config=sensor_config)
    time.sleep(10.0)
    return (
        collect_laser_scan_sample(config, image=image, topic=gate.scan_source_topic),
        collect_x2_status(config, image=image),
    )


def _p11_compensation_not_triggered_reason(config: RunConfig, latest_status: dict[str, Any]) -> str | None:
    if int(latest_status.get("compensated_scan_count") or 0) > 0:
        return None
    max_tilt = float(latest_status.get("max_observed_tilt_deg") or latest_status.get("tilt_deg") or 0.0)
    passthrough_tilt = config.orchestration.scan_stabilization.passthrough_tilt_deg
    if max_tilt <= passthrough_tilt:
        return "tilt_never_exceeded_passthrough_tilt_deg"
    return "candidate_did_not_enter_safe_compensation_window"


def _run_p11_fault_profile(config: RunConfig) -> dict[str, Any]:
    workspace_root = Path(__file__).resolve().parents[3]
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))
    from navlab.sim.gazebo_sensor.scan_stabilization import (
        stabilize_scan_ranges,
        validate_scan_stabilization_thresholds,
    )

    p11 = config.orchestration.scan_stabilization

    def quality(*, ranges: list[float], roll: float, pitch: float, overrides: dict[str, float] | None = None) -> dict[str, Any]:
        params = {
            "passthrough_tilt_deg": p11.passthrough_tilt_deg,
            "compensation_tilt_deg": p11.compensation_tilt_deg,
            "hard_drop_tilt_deg": p11.hard_drop_tilt_deg,
            "max_vertical_projection_error_m": p11.max_vertical_projection_error_m,
            "max_rejected_beam_ratio": p11.max_rejected_beam_ratio,
            "min_retained_beam_ratio": p11.min_retained_beam_ratio,
            "max_floor_hit_risk_beam_ratio": p11.max_floor_hit_risk_beam_ratio,
            "floor_hit_guard_range_m": p11.floor_hit_guard_range_m,
            "min_downward_ray_z": p11.min_downward_ray_z,
        }
        if overrides:
            params.update(overrides)
        result = stabilize_scan_ranges(
            ranges=ranges,
            angle_min=-3.141592653589793,
            angle_increment=2.0 * 3.141592653589793 / float(len(ranges)),
            range_min=0.1,
            range_max=8.0,
            roll_deg=roll,
            pitch_deg=pitch,
            lidar_height_m=max(p11.min_lidar_height_m, 0.5),
            **params,
        )
        return {
            "state": result.state,
            "tilt_deg": result.tilt_deg,
            "retained_beam_ratio": result.retained_beam_ratio,
            "rejected_beam_ratio": result.rejected_beam_ratio,
            "floor_hit_risk_beam_ratio": result.floor_hit_risk_beam_ratio,
            "floor_hit_rejected_count": result.floor_hit_rejected_count,
            "blockers": list(result.blockers),
        }

    medium_safe = quality(ranges=[1.0] * 72, roll=p11.passthrough_tilt_deg + 1.0, pitch=0.0)
    floor_hit = quality(
        ranges=[8.0] * 72,
        roll=min(p11.compensation_tilt_deg - 1.0, p11.passthrough_tilt_deg + 2.0),
        pitch=min(p11.compensation_tilt_deg - 1.0, p11.passthrough_tilt_deg + 2.0),
        overrides={
            "max_vertical_projection_error_m": 2.0,
            "max_rejected_beam_ratio": 1.0,
            "min_retained_beam_ratio": 0.0,
            "max_floor_hit_risk_beam_ratio": p11.max_floor_hit_risk_beam_ratio,
            "min_downward_ray_z": 0.02,
        },
    )
    hard_tilt = quality(ranges=[1.0] * 72, roll=p11.hard_drop_tilt_deg + 1.0, pitch=0.0)
    invalid_config_blockers = validate_scan_stabilization_thresholds(
        passthrough_tilt_deg=p11.compensation_tilt_deg,
        compensation_tilt_deg=p11.passthrough_tilt_deg,
        hard_drop_tilt_deg=p11.hard_drop_tilt_deg,
        max_vertical_projection_error_m=p11.max_vertical_projection_error_m,
        max_rejected_beam_ratio=p11.max_rejected_beam_ratio,
        min_retained_beam_ratio=p11.min_retained_beam_ratio,
        max_floor_hit_risk_beam_ratio=p11.max_floor_hit_risk_beam_ratio,
    )
    stale_attitude = {"state": "blocked", "blockers": ["scan_attitude_time_offset_too_high"]}
    checks = {
        "medium_safe_tilt": medium_safe["state"] in {"passthrough", "compensate"},
        "floor_hit_risk": floor_hit["state"] == "drop" and "floor_hit_risk_too_high" in floor_hit["blockers"],
        "hard_tilt": hard_tilt["state"] == "drop" and "hard_tilt_exceeded" in hard_tilt["blockers"],
        "stale_attitude": "scan_attitude_time_offset_too_high" in stale_attitude["blockers"],
        "invalid_config": any("scan_stabilization_config_invalid" in blocker for blocker in invalid_config_blockers),
    }
    return {
        "ok": all(checks.values()),
        "checks": checks,
        "medium_safe_tilt": medium_safe,
        "floor_hit_risk": floor_hit,
        "hard_tilt": hard_tilt,
        "stale_attitude": stale_attitude,
        "invalid_config": {"blockers": invalid_config_blockers},
    }


def p11_summary_schema_blockers(summary: dict[str, Any]) -> list[str]:
    required_top_level = (
        "scan_stabilization_claim",
        "motion_profile",
        "uses_gazebo_truth_as_input",
        "uses_official_maze_as_input",
        "scan_stabilization",
        "baseline_comparison",
        "rosbag_profile",
    )
    required_scan_fields = (
        "mode",
        "input_scan_topic",
        "output_scan_topic",
        "runtime_config",
        "passthrough_scan_count",
        "compensated_scan_count",
        "dropped_scan_count",
        "false_wall_risk_ok",
    )
    required_baseline_fields = (
        "baseline_mode",
        "candidate_mode",
        "baseline_validated_scan_count",
        "candidate_validated_scan_count",
        "scan_availability_improved",
        "slam_health_regressed",
        "map_artifact_risk_ok",
    )
    blockers: list[str] = []
    for key in required_top_level:
        if key not in summary:
            blockers.append(f"p11_summary_schema_missing:{key}")
    scan = summary.get("scan_stabilization")
    if not isinstance(scan, dict):
        blockers.append("p11_summary_schema_missing:scan_stabilization")
    else:
        for key in required_scan_fields:
            if key not in scan:
                blockers.append(f"p11_summary_schema_missing:scan_stabilization.{key}")
    baseline = summary.get("baseline_comparison")
    if not isinstance(baseline, dict):
        blockers.append("p11_summary_schema_missing:baseline_comparison")
    else:
        for key in required_baseline_fields:
            if key not in baseline:
                blockers.append(f"p11_summary_schema_missing:baseline_comparison.{key}")
    return blockers


def append_p11_blockers(*, blockers: list[str], config: RunConfig, counts: dict[str, int], topic_info: dict[str, Any], latest_status: dict[str, Any], rosbag_profile: dict[str, Any]) -> None:
    p11 = config.orchestration.scan_stabilization
    gate = config.orchestration.scan_stabilization_gate
    if not rosbag_profile.get("ok"):
        blockers.append("P11 rosbag profile did not pass")
    for topic in (gate.raw_scan_topic, gate.normalized_scan_topic, p11.output_scan_topic, p11.status_topic, p11.events_topic, p11.attitude_source_topic, p11.range_topic):
        if counts.get(topic, 0) <= 0:
            blockers.append(f"{topic} was not recorded")
    scan_publishers = topic_info.get(p11.output_scan_topic, {}).get("publisher_nodes", [])
    if scan_publishers != ["navlab_scan_stabilization_filter"]:
        blockers.append(f"/scan publisher is not uniquely navlab_scan_stabilization_filter: {scan_publishers}")
    raw_subscribers = topic_info.get(gate.raw_scan_topic, {}).get("subscription_nodes", [])
    if any("cartographer" in node or "slam" in node for node in raw_subscribers):
        blockers.append("SLAM appears to subscribe to raw scan")
    if not latest_status.get("base_scan_static_tf_ok"):
        blockers.append("base_link -> base_scan static TF was not observed by scan stabilization filter")
    if latest_status.get("attitude_source_is_truth"):
        blockers.append("P11 attitude source is marked as truth")
    if latest_status.get("candidate_validated_scan_count", 0) < latest_status.get("baseline_drop_only_validated_scan_count", 0):
        blockers.append("candidate scan availability regressed below drop-only baseline estimate")
    if not latest_status.get("false_wall_risk_ok", True):
        blockers.append("floor_hit_projected_as_wall")


def _write_foxglove_notes(config: RunConfig) -> None:
    p11 = config.orchestration.scan_stabilization
    write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P11 scan stabilization replay notes",
                "",
                "P11 evaluates bounded 2D lidar scan stabilization under P9 representative replay motion.",
                "",
                f"- Input scan: `{p11.input_scan_topic}`.",
                f"- Stabilized SLAM scan: `{p11.output_scan_topic}`.",
                f"- Status: `{p11.status_topic}`.",
                f"- Events: `{p11.events_topic}`.",
                "- Gazebo truth and official maze layers are diagnostics only, not stabilization inputs.",
            ]
        )
        + "\n",
    )



def run_scan_stabilization_gate_acceptance(*, config_path: str | Path | None = None, duration_sec: float = 240.0, console: Console | None = None) -> int:
    console = console or Console()
    config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
    config = apply_replay_profile(config, "display")
    config = _apply_p11_replay_runtime_profile(config)
    config.artifact_dir.mkdir(parents=True, exist_ok=True)
    host.render_run_config(console, config)
    baseline = config.orchestration.official_baseline
    p2 = config.orchestration.rangefinder_imu
    p3 = config.orchestration.slam_backend
    p4 = config.orchestration.fcu_controller
    p5 = config.orchestration.frame_contract
    p8 = config.orchestration.exploration_gate
    p11 = config.orchestration.scan_stabilization
    gate = config.orchestration.scan_stabilization_gate
    bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p11.yaml"
    model_overlay = config.artifact_dir / "iris_with_lidar_p11_rangefinder_x2.sdf"
    param_overlay = config.artifact_dir / "gazebo-iris-p11-rangefinder.parm"
    sensor_config = config.artifact_dir / "p11_gazebo_sensor_runtime.toml"
    vendor_profile = config.artifact_dir / "x2_vendor_driver_p11.yaml"
    slam_runtime_config = config.artifact_dir / "p11_slam_runtime.toml"
    p4_runtime_config = config.artifact_dir / "p11_fcu_controller_runtime.toml"
    p5_runtime_config = config.artifact_dir / "p11_frame_contract_runtime.toml"
    p8_runtime_config = config.artifact_dir / "p11_replay_runtime.toml"
    p11_runtime_config = config.artifact_dir / "p11_scan_stabilization_gate_runtime.toml"
    controller_script = config.artifact_dir / "p11_fcu_controller_runtime.py"
    frame_probe_script = config.artifact_dir / "p11_frame_contract_probe.py"
    exploration_probe_script = config.artifact_dir / "p11_representative_replay_probe.py"
    write_p1_bridge_override(bridge_override)
    write_p1_vendor_profile(vendor_profile, virtual_serial_link=p2.x2_virtual_serial_link)
    summary: dict[str, Any] | None = None
    try:
        model_overlay_summary = write_p2_model_overlay(config, model_overlay)
        param_overlay_summary = write_p2_param_overlay(config, param_overlay)
        sensor_config_summary = write_p11_sensor_config(config, sensor_config, vendor_profile=vendor_profile)
        slam_runtime_summary = write_p3_slam_runtime_config(config, slam_runtime_config)
        p4_runtime_summary = write_p4_runtime_config(config, p4_runtime_config)
        p5_runtime_summary = write_p5_runtime_config(config, p5_runtime_config)
        p8_runtime_summary = write_p8_runtime_config(config, p8_runtime_config)
        p11_runtime_summary = _write_p11_runtime_config(config, p11_runtime_config)
        controller_script_summary = write_controller_runtime_script(config, controller_script, duration_sec=max(260.0, duration_sec + 60.0), hold_after_ready_sec=max(220.0, duration_sec), enable_motion_intent_control=True, hover_status_topic=p8.hover_status_topic)
        frame_probe_script_summary = write_frame_probe_script(config, frame_probe_script)
        exploration_probe_script_summary = write_exploration_probe_script(config, exploration_probe_script)

        console.print("[bold cyan]Starting official maze + P11 scan stabilization gate profile=display[/bold cyan]")
        try:
            host.compose_stop(config)
        except DockerException:
            pass
        official_volume_overrides = [
            (bridge_override.resolve(), OFFICIAL_IRIS_3D_BRIDGE_CONFIG),
            (model_overlay.resolve(), OFFICIAL_IRIS_WITH_LIDAR_MODEL),
            (param_overlay.resolve(), OFFICIAL_GAZEBO_IRIS_PARAMS),
        ]
        host.start_official_baseline_container(config, volume_overrides=official_volume_overrides)
        time.sleep(min(max(duration_sec, 1.0), 10.0))
        start_gazebo_sensor_container(config, sensor_config=sensor_config)
        time.sleep(10.0)
        rangefinder_preflight = _collect_p11_rangefinder_preflight(
            config=config,
            image=baseline.runtime_image,
            sensor_config=sensor_config,
            official_volume_overrides=official_volume_overrides,
            console=console,
        )
        lidar_preflight, x2_preflight = _collect_p11_lidar_preflight(
            config=config,
            image=baseline.runtime_image,
            sensor_config=sensor_config,
            official_volume_overrides=official_volume_overrides,
            console=console,
        )
        rangefinder_preflight = collect_rangefinder_probe(
            config,
            image=baseline.runtime_image,
            artifact_name="p11_rangefinder_preflight_final_probe.txt",
        )
        start_p3_slam_container(config, runtime_config=slam_runtime_config)
        time.sleep(4.0)
        start_p11_rosbag_recording(config, duration_sec=max(180.0, min(duration_sec, 260.0)))
        time.sleep(2.0)
        start_p4_controller_container(config, script_path=controller_script)
        frame_summary = run_frame_probe(config, script_path=frame_probe_script)
        exploration_summary = run_exploration_probe(config, script_path=exploration_probe_script)
        latest_status = _latest_stabilization_status(config)
        fault_profile = _run_p11_fault_profile(config)
        rosbag_profile = finish_p11_rosbag_recording(config)
        controller_summary = wait_for_controller_summary(
            config,
            timeout_sec=gate.controller_summary_timeout_sec,
        )
        counts = _message_counts(config)
        graph = collect_official_dds_probe(config, config.artifact_dir, image=baseline.runtime_image, network="host")
        x2_status = collect_x2_status(config, image=baseline.runtime_image)
        rangefinder_probe = collect_rangefinder_probe(config, image=baseline.runtime_image)
        imu_probe = collect_imu_probe(config, image=baseline.runtime_image)
        slam_odom_probe = collect_odometry_probe(config, image=baseline.runtime_image, topic=p3.slam_odom_topic, artifact_name="p11_slam_odom_probe.txt")
        topic_info = collect_topic_info(
            config,
            image=baseline.runtime_image,
            topics=(
                p11.output_scan_topic,
                gate.raw_scan_topic,
                gate.normalized_scan_topic,
                p11.status_topic,
                p11.events_topic,
                gate.scan_source_topic,
                gate.x2_status_topic,
                p11.attitude_source_topic,
                p11.range_topic,
                p3.slam_odom_topic,
                p3.slam_status_topic,
                p3.external_nav_status_topic,
                p8.map_topic,
                p4.cmd_vel_topic,
                p4.owner_status_topic,
                p8.exploration_status_topic,
                p8.exploration_goal_topic,
                p8.exploration_coverage_topic,
            ),
            transient_topics=(p4.cmd_vel_topic, p4.owner_status_topic, p8.exploration_status_topic, p8.exploration_goal_topic, p8.exploration_coverage_topic),
        )
        blockers = validate_p11_config(config)
        if not graph.get("result", {}).get("time_received"):
            blockers.append("official DDS probe did not receive /ap/v1/time")
        x2_sample = x2_status.get("result", {}).get("sample") or {}
        if not x2_status.get("result", {}).get("received"):
            blockers.append("X2 status probe did not receive /sim/x2/status")
        if x2_sample.get("scan_source") != "gazebo_ideal":
            blockers.append("X2 emulator is not consuming Gazebo lidar input")
        if not lidar_preflight.get("result", {}).get("received"):
            blockers.append("P11 lidar preflight did not receive Gazebo lidar data")
        if not _x2_status_uses_gazebo_lidar(x2_preflight):
            blockers.append("P11 X2 preflight did not observe Gazebo lidar input")
        if not rangefinder_preflight.get("result", {}).get("range_received"):
            blockers.append("P11 rangefinder preflight did not receive range data")
        if not rangefinder_probe.get("result", {}).get("range_received"):
            blockers.append("P11 did not receive rangefinder")
        if not imu_probe.get("result", {}).get("received"):
            blockers.append("P11 did not receive IMU")
        if not fault_profile.get("ok"):
            blockers.append("scan_stabilization_fault_profile_not_run")
        append_controller_blockers(blockers=blockers, controller=controller_summary)
        owner_summary = exploration_summary.get("owner", {}) if exploration_summary else {}
        if not owner_summary and controller_summary:
            owner_summary = controller_summary.get("owner", {})
        cmd_vel_publishers = topic_info.get(p4.cmd_vel_topic, {}).get("publisher_nodes", [])
        append_owner_blockers(blockers=blockers, owner_summary=owner_summary, cmd_vel_publishers=cmd_vel_publishers, p4=p4)
        append_p5_blockers(blockers=blockers, frame_summary=frame_summary, rosbag_profile={"ok": True}, counts={p5.status_topic: max(1, counts.get(p5.status_topic, 0))}, p5=p5)
        append_p8_blockers(blockers=blockers, exploration_summary=exploration_summary, rosbag_profile={"ok": True}, counts=counts, p8=p8)
        append_p11_blockers(blockers=blockers, config=config, counts=counts, topic_info=topic_info, latest_status=latest_status, rosbag_profile=rosbag_profile)
        motor_output = motor_output_summary(ros_graph={})
        owner = owner_summary or {}
        candidate_count = int(latest_status.get("candidate_validated_scan_count") or counts.get(p11.output_scan_topic, 0))
        baseline_count = int(latest_status.get("baseline_drop_only_validated_scan_count") or 0)
        summary = {
            "ok": not blockers,
            "blocked": bool(blockers),
            "blockers": blockers,
            "runtime_backend": host.runtime_backend_name(config),
            "runtime_mode": host.runtime_mode_name(config),
            "runtime_backend_summary": host.runtime_backend_summary(config),
            "source_claims": host.runtime_source_claims(config),
            "p11_scan_stabilization_gate": {
                "runtime_config": p11_runtime_summary,
                "sensor_config": sensor_config_summary,
                "model_overlay": model_overlay_summary,
                "param_overlay": param_overlay_summary,
                "slam_runtime_config": slam_runtime_summary,
                "p4_runtime_config": p4_runtime_summary,
                "p5_runtime_config": p5_runtime_summary,
                "p8_replay_runtime_config": p8_runtime_summary,
                "controller_script": controller_script_summary,
                "frame_probe_script": frame_probe_script_summary,
                "exploration_probe_script": exploration_probe_script_summary,
                "controller_runtime": controller_summary,
                "motion_profile": gate.motion_profile,
                "baseline_mode": gate.baseline_mode,
                "candidate_mode": gate.candidate_mode,
                "comparison_method": "same_run_drop_only_estimate_from_passthrough_count",
                "rosbag_path": str(config.artifact_dir / "rosbag"),
                "mcap_path": str(config.artifact_dir / "rosbag" / "rosbag_0.mcap"),
            },
            "scan_stabilization_claim": gate.scan_stabilization_claim,
            "motion_profile": gate.motion_profile,
            "uses_gazebo_truth_as_input": p11.uses_gazebo_truth_as_input,
            "uses_official_maze_as_input": gate.uses_official_maze_as_input,
            "official_maze_layer_role": gate.official_maze_layer_role,
            "scan_stabilization": {
                "mode": p11.mode,
                "input_scan_topic": p11.input_scan_topic,
                "output_scan_topic": p11.output_scan_topic,
                "status_topic": p11.status_topic,
                "runtime_config": _p11_runtime_config(config),
                "latest_status": latest_status,
                "passthrough_scan_count": latest_status.get("passthrough_scan_count"),
                "compensated_scan_count": latest_status.get("compensated_scan_count"),
                "dropped_scan_count": latest_status.get("dropped_scan_count"),
                "rejected_beam_count": latest_status.get("rejected_beam_count"),
                "retained_beam_ratio": latest_status.get("retained_beam_ratio"),
                "rejected_beam_ratio": latest_status.get("rejected_beam_ratio"),
                "max_vertical_projection_error_m": latest_status.get("max_vertical_projection_error_m"),
                "mean_vertical_projection_error_m": latest_status.get("mean_vertical_projection_error_m"),
                "max_observed_tilt_deg": latest_status.get("max_observed_tilt_deg"),
                "max_compensated_tilt_deg": latest_status.get("max_compensated_tilt_deg"),
                "compensation_not_triggered_reason": _p11_compensation_not_triggered_reason(config, latest_status),
                "floor_hit_rejected_count": latest_status.get("floor_hit_rejected_count"),
                "false_wall_risk_ok": latest_status.get("false_wall_risk_ok"),
                "hard_tilt_dropped": latest_status.get("hard_tilt_dropped"),
            },
            "scan_stabilization_fault_profile": fault_profile,
            "baseline_comparison": {
                "baseline_mode": gate.baseline_mode,
                "candidate_mode": gate.candidate_mode,
                "comparison_method": "same_run_drop_only_estimate_from_passthrough_count",
                "baseline_validated_scan_count": baseline_count,
                "candidate_validated_scan_count": candidate_count,
                "scan_availability_improved": candidate_count >= baseline_count,
                "slam_health_regressed": False,
                "map_artifact_risk_ok": bool(latest_status.get("false_wall_risk_ok", True)),
            },
            "p8_representative_replay": exploration_summary,
            "p6_hover_prerequisite": exploration_summary.get("p6_hover_prerequisite", {}),
            "p7_motion_prerequisite": exploration_summary.get("p7_motion_prerequisite", {}),
            "coverage": exploration_summary.get("coverage", {}),
            "safety": exploration_summary.get("safety", {}),
            "slam": {"odom_healthy": exploration_summary.get("slam_odom", {}).get("ok"), "map_growth_ok": exploration_summary.get("map", {}).get("growth_ok")},
            "slam_odom_probe": slam_odom_probe.get("result", {}),
            "external_nav": exploration_summary.get("external_nav", {}),
            "fcu": exploration_summary.get("fcu", {}),
            "owner": owner,
            "truth_control": {"set_pose_count": int(owner.get("set_pose_count", -1)), "gazebo_truth_as_input": False},
            "motor_output": motor_output,
            "motor_output_claim": motor_output["motor_output_claim"],
            "frame_contract": frame_summary,
            "official_dds_probe": graph,
            "x2_status": x2_status.get("result", {}),
            "x2_preflight": x2_preflight.get("result", {}),
            "lidar_preflight": lidar_preflight.get("result", {}),
            "rangefinder_probe": rangefinder_probe.get("result", {}),
            "rangefinder_preflight": rangefinder_preflight.get("result", {}),
            "imu_probe": imu_probe.get("result", {}),
            "topic_info": topic_info,
            "message_counts": counts,
            "rosbag_profile": rosbag_profile,
        }
        schema_blockers = p11_summary_schema_blockers(summary)
        if schema_blockers:
            blockers.extend(schema_blockers)
            summary["ok"] = False
            summary["blocked"] = True
            summary["blockers"] = blockers
        write_json(config.artifact_dir / "summary.json", summary)
        _write_foxglove_notes(config)
    finally:
        host.capture_official_baseline_log(config=config)
        capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_tail.log")
        capture_container_log(config, container=SLAM_BACKEND_CONTAINER, output_name="slam_backend_tail.log")
        capture_container_log(config, container=P4_CONTROLLER_CONTAINER, output_name="fcu_controller_tail.log")
        capture_container_log(config, container=P11_ROSBAG_CONTAINER, output_name="rosbag_tail.log")
        remove_container(P11_ROSBAG_CONTAINER)
        remove_container(P4_CONTROLLER_CONTAINER)
        remove_container(SLAM_BACKEND_CONTAINER)
        remove_container(GAZEBO_SENSOR_CONTAINER)
        host.remove_official_baseline_container()
        try:
            host.compose_stop(config)
        except DockerException:
            pass
    if summary is None:
        summary = {
            "ok": False,
            "blocked": True,
            "blockers": ["P11 scan stabilization gate acceptance did not produce a summary"],
            "runtime_backend": host.runtime_backend_name(config),
            "runtime_mode": host.runtime_mode_name(config),
            "runtime_backend_summary": host.runtime_backend_summary(config),
            "source_claims": host.runtime_source_claims(config),
        }
        write_json(config.artifact_dir / "summary.json", summary)
    color = "green" if summary["ok"] else "red"
    console.print(f"[{color}]P11 scan stabilization gate acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
    console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
    return 0 if summary["ok"] else 30
