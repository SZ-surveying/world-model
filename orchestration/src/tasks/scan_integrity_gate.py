from __future__ import annotations

import json
import os
import shlex
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

import tomli_w
from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.config import RunConfig
from src.tasks.base import OrchestrationTask
from src.tasks.fcu_controller import (
    P4_CONTROLLER_CONTAINER,
    _append_controller_blockers,
    _append_owner_blockers,
    _start_p4_controller_container,
    _wait_for_controller_summary,
    _write_controller_runtime_script,
    _write_p4_runtime_config,
)
from src.tasks.frame_contract import _append_p5_blockers, _run_frame_probe, _write_frame_probe_script, _write_p5_runtime_config
from src.tasks.motion_gate import _build_p7_doctor_summary, _run_motion_probe, _write_motion_probe_script
from src.tasks.official_baseline import (
    _collect_official_dds_probe,
    _collect_ros_graph,
    _load_rosbag_metadata_counts,
    _validate_official_rosbag_profile,
    _write_json,
    _write_text,
)
from src.tasks.official_maze_x2 import (
    GAZEBO_SENSOR_CONTAINER,
    OFFICIAL_IRIS_3D_BRIDGE_CONFIG,
    _capture_container_log,
    _collect_topic_info,
    _collect_x2_status,
    _file_sha256,
    _profile_topics,
    _remove_container,
    _start_gazebo_sensor_container,
    _write_p1_bridge_override,
    _write_p1_vendor_profile,
)
from src.tasks.rangefinder_imu import (
    OFFICIAL_GAZEBO_IRIS_PARAMS,
    OFFICIAL_IRIS_WITH_LIDAR_MODEL,
    _collect_imu_probe,
    _collect_rangefinder_probe,
    _write_p2_model_overlay,
    _write_p2_param_overlay,
)
from src.tasks.registry import TaskRegistry
from src.tasks.slam_backend import (
    SLAM_BACKEND_CONTAINER,
    _append_slam_odom_quality_blockers,
    _collect_odometry_probe,
    _start_p3_slam_container,
    _write_p3_slam_runtime_config,
)
from src.tasks.slam_hover import _baseline_env, _build_p6_doctor_summary, _load_json, _source_official_setup

P10_ROSBAG_CONTAINER = "navlab-p10-rosbag"


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return _load_rosbag_metadata_counts(metadata)


def _write_p10_sensor_config(config: RunConfig, path: Path, *, vendor_profile: Path) -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    p10 = config.orchestration.scan_integrity_gate
    data = {
        "gazebo_sensor": {
            "x2_protocol": {
                "enabled": True,
                "scan_source": "x2_virtual_serial",
                "profile": host._workspace_path(vendor_profile),
                "virtual_serial_link": p2.x2_virtual_serial_link,
                "scan_ideal_topic": p2.x2_scan_input_topic,
                "vendor_scan_topic": p10.raw_scan_topic,
                "scan_topic": p10.normalized_scan_topic,
                "status_topic": p10.x2_status_topic,
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
            "scan_integrity": {
                "enabled": True,
                "input_scan_topic": p10.normalized_scan_topic,
                "output_scan_topic": p10.validated_scan_topic,
                "status_topic": p10.status_topic,
                "events_topic": p10.events_topic,
                "fault_injection_topic": p10.fault_injection_topic,
                "attitude_source_topic": p10.attitude_source_topic,
                "attitude_source_type": p10.attitude_source_type,
                "range_topic": p10.rangefinder_range_topic,
                "base_frame_id": p10.base_frame_id,
                "scan_frame_id": p10.scan_frame_id,
                "soft_tilt_deg": p10.soft_tilt_deg,
                "hard_tilt_deg": p10.hard_tilt_deg,
                "max_dropped_scan_ratio": p10.max_dropped_scan_ratio,
                "max_clipped_beam_ratio": p10.max_clipped_beam_ratio,
                "max_scan_attitude_time_offset_ms": p10.max_scan_attitude_time_offset_ms,
                "max_attitude_source_age_ms": p10.max_attitude_source_age_ms,
                "min_attitude_rate_hz": p10.min_attitude_rate_hz,
                "floor_hit_guard_range_m": p10.floor_hit_guard_range_m,
                "min_lidar_height_m": p10.min_lidar_height_m,
                "min_downward_ray_z": p10.min_downward_ray_z,
            },
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _write_p10_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    p10 = config.orchestration.scan_integrity_gate
    data = {"scan_integrity_gate": {"runtime": {key: getattr(p10, key) for key in p10.__dataclass_fields__}}}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _fault_probe_script(spec: dict[str, Any]) -> str:
    spec_json = json.dumps(spec, sort_keys=True)
    return f'''
from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from std_msgs.msg import String

SPEC = json.loads({spec_json!r})

class FaultProbe:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("navlab_p10_scan_integrity_fault_probe")
        self.pub = self.node.create_publisher(String, SPEC["fault_topic"], 10)
        self.samples = []
        self.node.create_subscription(String, SPEC["status_topic"], self._status_cb, 10)

    def _status_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {{"raw": msg.data}}
        payload["monotonic"] = time.monotonic()
        self.samples.append(payload)

    def _publish_fault(self, **payload) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.02)

    def _collect_window(self, duration: float) -> list:
        start = time.monotonic()
        while time.monotonic() - start < duration:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.02)
        return [item for item in self.samples if item.get("monotonic", 0.0) >= start]

    def run(self) -> dict:
        baseline = self._collect_window(float(SPEC["normal_window_sec"]))
        self._publish_fault(enabled=True, roll_bias_deg=SPEC["mild_roll_deg"], pitch_bias_deg=SPEC["mild_pitch_deg"])
        mild = self._collect_window(float(SPEC["fault_window_sec"]))
        self._publish_fault(enabled=True, roll_bias_deg=SPEC["hard_roll_deg"], pitch_bias_deg=SPEC["hard_pitch_deg"])
        hard = self._collect_window(float(SPEC["fault_window_sec"]))
        self._publish_fault(enabled=False, roll_bias_deg=0.0, pitch_bias_deg=0.0)
        reset = self._collect_window(float(SPEC["normal_window_sec"]))
        states = lambda items: [str(item.get("state")) for item in items]
        hard_rejected = any(state in {{"drop", "blocked"}} for state in states(hard))
        mild_survived = any(state in {{"accept", "warn", "clip"}} for state in states(mild))
        reset_ok = any(state in {{"accept", "warn", "clip"}} for state in states(reset))
        blockers = []
        if not baseline:
            blockers.append("fault_probe_missing_baseline_status")
        if not mild_survived:
            blockers.append("mild_tilt_not_observed")
        if not hard_rejected:
            blockers.append("hard_tilt_not_rejected")
        if not reset_ok:
            blockers.append("post_fault_reset_not_observed")
        summary = {{
            "ok": not blockers,
            "blockers": blockers,
            "fault_injection_mode": "attitude_bias_runtime",
            "baseline_states": states(baseline),
            "mild_states": states(mild),
            "hard_states": states(hard),
            "reset_states": states(reset),
            "mild_fault": {{"roll_bias_deg": SPEC["mild_roll_deg"], "pitch_bias_deg": SPEC["mild_pitch_deg"]}},
            "hard_fault": {{"roll_bias_deg": SPEC["hard_roll_deg"], "pitch_bias_deg": SPEC["hard_pitch_deg"]}},
            "hard_tilt_rejected": hard_rejected,
            "sample_count": len(self.samples),
            "latest": self.samples[-1] if self.samples else None,
        }}
        Path(SPEC["summary_file"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
        print(json.dumps(summary, sort_keys=True))
        self.node.destroy_node()
        rclpy.shutdown()
        return summary

raise SystemExit(0 if FaultProbe().run()["ok"] else 31)
'''


def _write_fault_probe_script(config: RunConfig, path: Path) -> dict[str, Any]:
    p10 = config.orchestration.scan_integrity_gate
    summary_file = config.artifact_dir / "scan_integrity_fault_summary.json"
    spec = {
        "summary_file": host._workspace_path(summary_file),
        "status_topic": p10.status_topic,
        "fault_topic": p10.fault_injection_topic,
        "normal_window_sec": p10.normal_window_sec,
        "fault_window_sec": p10.fault_window_sec,
        "mild_roll_deg": p10.mild_fault_roll_bias_deg,
        "mild_pitch_deg": p10.mild_fault_pitch_bias_deg,
        "hard_roll_deg": p10.hard_fault_roll_bias_deg,
        "hard_pitch_deg": p10.hard_fault_pitch_bias_deg,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    _write_text(path, _fault_probe_script(spec))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "summary_file": str(summary_file), "spec": spec}


def _run_fault_probe(config: RunConfig, *, script_path: Path) -> dict[str, Any]:
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.orchestration.official_baseline.runtime_image,
        shell_command=f"python3 {shlex.quote(host._workspace_path(script_path))}",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "scan_integrity_fault_probe.txt", output)
    summary = _load_json(config.artifact_dir / "scan_integrity_fault_summary.json")
    if not summary:
        summary = {"ok": False, "blockers": [f"fault probe failed rc={rc}"], "output": output}
    summary["rc"] = rc
    return summary


def _p10_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.scan_integrity_gate_rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
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
        f"for i in $(seq 1 40); do [ -f {shlex.quote(str(container_rosbag / 'metadata.yaml'))} ] && exit 0; sleep 0.25; done; exit 2"
    )
    return profile_path, required, optional, command


def _start_p10_rosbag_recording(config: RunConfig, *, duration_sec: float) -> None:
    _remove_container(P10_ROSBAG_CONTAINER)
    profile_path, required, optional, command = _p10_rosbag_shell_command(config, duration_sec=duration_sec)
    if not command:
        _write_json(config.artifact_dir / "rosbag_profile_summary.json", {"ok": False, "recorded": False, "profile": str(profile_path), "required_topics": required, "optional_topics": optional, "reason": "rosbag profile missing or empty"})
        return
    DockerClient().run(
        config.orchestration.official_baseline.runtime_image,
        ["bash", "-lc", _source_official_setup(command)],
        detach=True,
        name=P10_ROSBAG_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={**_baseline_env(config), "PYTHONPATH": "/workspace"},
    )


def _finish_p10_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    profile_path = Path(config.scan_integrity_gate_rosbag_profile)
    required, optional, _topics = _profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    try:
        rc = DockerClient().wait(P10_ROSBAG_CONTAINER)
    except DockerException as exc:
        rc = exc.return_code or 1
    try:
        output = DockerClient().logs(P10_ROSBAG_CONTAINER, tail=2000)
    except DockerException as exc:
        output = str(exc)
    _write_text(config.artifact_dir / "rosbag_record.txt", str(output))
    for _ in range(160):
        if metadata.is_file():
            break
        time.sleep(0.25)
    if rc != 0 or not metadata.is_file():
        summary = {"ok": False, "recorded": False, "profile": str(profile_path), "required_topics": required, "optional_topics": optional, "reason": f"rosbag record failed rc={rc}", "record_output": str(output)}
        _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    summary = _validate_official_rosbag_profile(profile=profile_path, metadata=metadata, required=required, optional=optional)
    summary["rosbag_path"] = str(config.artifact_dir / "rosbag")
    summary["mcap_path"] = str(config.artifact_dir / "rosbag" / "rosbag_0.mcap")
    _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
    return summary


def _build_p10_doctor_summary(config: RunConfig, *, runtime_config: Path, include_dependencies: bool = True) -> dict[str, Any]:
    p10 = config.orchestration.scan_integrity_gate
    p6_runtime = config.artifact_dir / "p10_doctor_p6_slam_hover_runtime.toml"
    p7_runtime = config.artifact_dir / "p10_doctor_p7_motion_gate_runtime.toml"
    p6_doctor = _build_p6_doctor_summary(config, runtime_config=p6_runtime) if include_dependencies else {"ok": True, "blockers": [], "skipped": "acceptance already launched P6 prerequisites"}
    p7_doctor = _build_p7_doctor_summary(config, runtime_config=p7_runtime, include_dependencies=False) if include_dependencies else {"ok": True, "blockers": [], "skipped": "acceptance already launched P7 prerequisites"}
    profile_path = Path(p10.rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    blockers = [str(item) for item in p6_doctor.get("blockers", [])]
    blockers.extend(str(item) for item in p7_doctor.get("blockers", []))
    if not profile_path.is_file() or not topics:
        blockers.append("P10 rosbag profile is missing or empty")
    if p10.uses_gazebo_truth_as_input:
        blockers.append("P10 must not use Gazebo truth as scan integrity input")
    if p10.validated_scan_topic != config.orchestration.slam_backend.scan_topic:
        blockers.append("P10 validated scan topic must match SLAM scan topic")
    if p10.raw_scan_topic == p10.validated_scan_topic or p10.normalized_scan_topic == p10.validated_scan_topic:
        blockers.append("P10 raw/normalized scan topics must not publish directly to /scan")
    if p10.max_attitude_source_age_ms <= 0.0:
        blockers.append("scan_integrity_config_invalid: max_attitude_source_age_ms must be positive")
    if p10.attitude_source_topic in {"/odometry", config.orchestration.slam_backend.truth_diagnostic_topic} or p10.attitude_source_topic.startswith("/gazebo"):
        blockers.append("P10 attitude source must not be Gazebo truth")
    if p10.scan_integrity_claim != "evaluated":
        blockers.append("P10 scan_integrity_claim must be evaluated")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "p10_scan_integrity_doctor": {
            "runtime_config": str(runtime_config),
            "runtime_config_sha256": _file_sha256(runtime_config) if runtime_config.is_file() else "",
            "dependency_checks_included": include_dependencies,
            "raw_scan_topic": p10.raw_scan_topic,
            "normalized_scan_topic": p10.normalized_scan_topic,
            "validated_scan_topic": p10.validated_scan_topic,
            "attitude_source_topic": p10.attitude_source_topic,
            "status_topic": p10.status_topic,
            "uses_gazebo_truth_as_input": p10.uses_gazebo_truth_as_input,
            "scan_integrity_claim": p10.scan_integrity_claim,
            "rosbag_profile": {"profile": str(profile_path), "required_topics": required, "optional_topics": optional},
        },
        "p6_slam_hover_doctor": p6_doctor,
        "p7_motion_gate_doctor": p7_doctor,
    }


def _latest_status_from_fault(fault_summary: dict[str, Any]) -> dict[str, Any]:
    latest = fault_summary.get("latest")
    return latest if isinstance(latest, dict) else {}


def _scan_attitude_quality(*, latest_status: dict[str, Any], ok: bool) -> dict[str, Any]:
    return {
        "ok": ok,
        "max_scan_tilt_deg": latest_status.get("max_scan_tilt_deg"),
        "tilt_filtered_scan_count": latest_status.get("tilt_filtered_scan_count"),
        "tilt_warning_count": latest_status.get("tilt_warning_count"),
        "dropped_scan_count": latest_status.get("dropped_scan_count"),
        "clipped_scan_count": latest_status.get("clipped_scan_count"),
        "hard_tilt_count": latest_status.get("hard_tilt_count"),
        "dropped_scan_ratio": latest_status.get("dropped_scan_ratio"),
        "max_clipped_beam_ratio": latest_status.get("clipped_beam_ratio"),
        "max_floor_hit_risk_beam_ratio": latest_status.get("floor_hit_risk_beam_ratio"),
    }


def _motor_output_summary(*, ros_graph: dict[str, Any]) -> dict[str, Any]:
    topics = ros_graph.get("ros2_topic_list", {}).get("lines", [])
    motor_keywords = ("motor", "servo", "actuator", "esc", "rpm", "pwm")
    excluded = {"/robot_description"}
    candidates = sorted(
        topic
        for topic in topics
        if topic not in excluded
        if any(keyword in topic.lower() for keyword in motor_keywords)
        and "support_motor" not in topic.lower()
    )
    if not candidates:
        return {
            "motor_output_claim": "not_available",
            "available": False,
            "candidate_topics": [],
            "motor_pwm_min": None,
            "motor_pwm_max": None,
            "motor_pwm_spread": None,
            "motor_rpm_min": None,
            "motor_rpm_max": None,
            "motor_rpm_spread": None,
            "motor_thrust_bias_estimate": None,
            "reason": "no motor/servo/actuator/ESC output topic is exposed in the ROS graph",
        }
    return {
        "motor_output_claim": "candidate_topics_present",
        "available": False,
        "candidate_topics": candidates,
        "motor_pwm_min": None,
        "motor_pwm_max": None,
        "motor_pwm_spread": None,
        "motor_rpm_min": None,
        "motor_rpm_max": None,
        "motor_rpm_spread": None,
        "motor_thrust_bias_estimate": None,
        "reason": "candidate topics exist, but P10.1 does not parse motor output message schemas yet",
    }


def _p10_normal_profile_ok(motion_summary: dict[str, Any]) -> bool:
    return bool(
        motion_summary
        and motion_summary.get("p6_hover_prerequisite", {}).get("ok")
        and motion_summary.get("clearance", {}).get("ok")
        and motion_summary.get("slam_odom", {}).get("ok")
        and motion_summary.get("external_nav", {}).get("ok")
        and motion_summary.get("fcu", {}).get("local_position_ok")
    )


def _append_p10_blockers(*, blockers: list[str], motion_summary: dict[str, Any], fault_summary: dict[str, Any], rosbag_profile: dict[str, Any], counts: dict[str, int], topic_info: dict[str, Any], p10: Any) -> None:
    if not motion_summary:
        blockers.append("P10 normal motion/scan summary is missing")
    else:
        if not motion_summary.get("p6_hover_prerequisite", {}).get("ok"):
            blockers.append("P10 P6 hover prerequisite did not pass")
        if not motion_summary.get("clearance", {}).get("ok"):
            blockers.append("P10 clearance gate did not pass")
        if not motion_summary.get("slam_odom", {}).get("ok"):
            blockers.append("P10 SLAM odom gate did not pass")
        if not motion_summary.get("external_nav", {}).get("ok"):
            blockers.append("P10 ExternalNav gate did not pass")
        if not motion_summary.get("fcu", {}).get("local_position_ok"):
            blockers.append("P10 FCU local position gate did not pass")
    if not fault_summary.get("ok"):
        blockers.extend(str(item) for item in fault_summary.get("blockers", []))
    if not rosbag_profile.get("ok"):
        blockers.append("P10 rosbag profile did not pass")
    for topic in (p10.raw_scan_topic, p10.normalized_scan_topic, p10.validated_scan_topic, p10.status_topic, p10.events_topic, p10.attitude_source_topic, p10.rangefinder_range_topic):
        if counts.get(topic, 0) <= 0:
            blockers.append(f"{topic} was not recorded")
    scan_publishers = topic_info.get(p10.validated_scan_topic, {}).get("publisher_nodes", [])
    if scan_publishers != ["navlab_scan_integrity_filter"]:
        blockers.append(f"/scan publisher is not uniquely navlab_scan_integrity_filter: {scan_publishers}")
    raw_subscribers = topic_info.get(p10.raw_scan_topic, {}).get("subscription_nodes", [])
    if any("cartographer" in node or "slam" in node for node in raw_subscribers):
        blockers.append("SLAM appears to subscribe to raw scan")
    latest = _latest_status_from_fault(fault_summary)
    if not latest.get("base_scan_static_tf_ok"):
        blockers.append("base_link -> base_scan static TF was not observed by scan integrity filter")
    if latest.get("attitude_source_is_truth"):
        blockers.append("P10 attitude source is marked as truth")
    if p10.uses_gazebo_truth_as_input:
        blockers.append("P10 is configured to use Gazebo truth as input")


def _write_foxglove_notes(config: RunConfig) -> None:
    p10 = config.orchestration.scan_integrity_gate
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P10 scan integrity replay notes",
                "",
                "P10 validates body-fixed 2D lidar scan integrity before real-machine flight.",
                "",
                "- Fixed frame: `map`.",
                f"- Raw scan: `{p10.raw_scan_topic}`.",
                f"- Normalized scan: `{p10.normalized_scan_topic}`.",
                f"- Validated SLAM scan: `{p10.validated_scan_topic}`.",
                f"- Scan integrity status: `{p10.status_topic}`.",
                f"- Attitude source: `{p10.attitude_source_topic}`.",
                "- Gazebo truth is diagnostic only and is not an integrity input.",
            ]
        )
        + "\n",
    )


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class ScanIntegrityGateDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "scan-integrity-gate-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check P10 body-fixed lidar scan integrity gate prerequisites."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_scan_integrity_gate_doctor/{config.run_id}"))
        artifact_dir.mkdir(parents=True, exist_ok=True)
        config = RunConfig.from_config(config_path=config_path, artifact_dir=artifact_dir, run_id=config.run_id)
        runtime_config = artifact_dir / "p10_scan_integrity_gate_runtime.toml"
        _write_p10_runtime_config(config, runtime_config)
        console.print("[bold cyan]Checking P10 body-fixed lidar scan integrity gate prerequisites[/bold cyan]")
        summary = _build_p10_doctor_summary(config, runtime_config=runtime_config, include_dependencies=False)
        _write_json(artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P10 scan integrity gate doctor rc={0 if summary['ok'] else 20}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 20


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class ScanIntegrityGateAcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "scan-integrity-gate-acceptance"
    TASK_DESCRIPTION: ClassVar[str] = "Run P10 body-fixed lidar scan integrity gate acceptance."

    def run(self, *, config_path: str | Path | None = None, duration_sec: float = 140.0, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
        config.artifact_dir.mkdir(parents=True, exist_ok=True)
        host._render_run_config(console, config)
        baseline = config.orchestration.official_baseline
        p2 = config.orchestration.rangefinder_imu
        p3 = config.orchestration.slam_backend
        p4 = config.orchestration.fcu_controller
        p5 = config.orchestration.frame_contract
        p10 = config.orchestration.scan_integrity_gate
        bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p10.yaml"
        model_overlay = config.artifact_dir / "iris_with_lidar_p10_rangefinder_x2.sdf"
        param_overlay = config.artifact_dir / "gazebo-iris-p10-rangefinder.parm"
        sensor_config = config.artifact_dir / "p10_gazebo_sensor_runtime.toml"
        vendor_profile = config.artifact_dir / "x2_vendor_driver_p10.yaml"
        slam_runtime_config = config.artifact_dir / "p10_slam_runtime.toml"
        p4_runtime_config = config.artifact_dir / "p10_fcu_controller_runtime.toml"
        p5_runtime_config = config.artifact_dir / "p10_frame_contract_runtime.toml"
        p10_runtime_config = config.artifact_dir / "p10_scan_integrity_gate_runtime.toml"
        controller_script = config.artifact_dir / "p10_fcu_controller_runtime.py"
        frame_probe_script = config.artifact_dir / "p10_frame_contract_probe.py"
        motion_probe_script = config.artifact_dir / "p10_motion_gate_probe.py"
        fault_probe_script = config.artifact_dir / "p10_scan_integrity_fault_probe.py"
        _write_p1_bridge_override(bridge_override)
        _write_p1_vendor_profile(vendor_profile, virtual_serial_link=p2.x2_virtual_serial_link)

        summary: dict[str, Any] | None = None
        try:
            model_overlay_summary = _write_p2_model_overlay(config, model_overlay)
            param_overlay_summary = _write_p2_param_overlay(config, param_overlay)
            sensor_config_summary = _write_p10_sensor_config(config, sensor_config, vendor_profile=vendor_profile)
            slam_runtime_summary = _write_p3_slam_runtime_config(config, slam_runtime_config)
            p4_runtime_summary = _write_p4_runtime_config(config, p4_runtime_config)
            p5_runtime_summary = _write_p5_runtime_config(config, p5_runtime_config)
            p10_runtime_summary = _write_p10_runtime_config(config, p10_runtime_config)
            controller_script_summary = _write_controller_runtime_script(config, controller_script, duration_sec=max(150.0, duration_sec + 60.0), hold_after_ready_sec=max(120.0, duration_sec), enable_motion_intent_control=True, hover_status_topic=config.orchestration.motion_gate.hover_status_topic)
            frame_probe_script_summary = _write_frame_probe_script(config, frame_probe_script)
            motion_probe_script_summary = _write_motion_probe_script(config, motion_probe_script)
            fault_probe_script_summary = _write_fault_probe_script(config, fault_probe_script)

            console.print("[bold cyan]Starting official maze + P10 scan integrity gate[/bold cyan]")
            try:
                host._compose_stop(config)
            except DockerException:
                pass
            official_volume_overrides = [
                (bridge_override.resolve(), OFFICIAL_IRIS_3D_BRIDGE_CONFIG),
                (model_overlay.resolve(), OFFICIAL_IRIS_WITH_LIDAR_MODEL),
                (param_overlay.resolve(), OFFICIAL_GAZEBO_IRIS_PARAMS),
            ]
            host._start_official_baseline_container(config, volume_overrides=official_volume_overrides)
            time.sleep(min(max(duration_sec, 1.0), 10.0))
            _start_gazebo_sensor_container(config, sensor_config=sensor_config)
            time.sleep(10.0)
            rangefinder_preflight = _collect_rangefinder_probe(config, image=baseline.runtime_image, artifact_name="p10_rangefinder_preflight_probe.txt")
            if not rangefinder_preflight.get("result", {}).get("range_received"):
                console.print("[yellow]P10 rangefinder preflight missed data; restarting gazebo sensor once[/yellow]")
                _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_preflight_tail.log")
                _start_gazebo_sensor_container(config, sensor_config=sensor_config)
                time.sleep(10.0)
                rangefinder_preflight = _collect_rangefinder_probe(config, image=baseline.runtime_image, artifact_name="p10_rangefinder_preflight_retry_probe.txt")
            _start_p3_slam_container(config, runtime_config=slam_runtime_config)
            time.sleep(4.0)
            _start_p10_rosbag_recording(config, duration_sec=max(120.0, min(duration_sec, 180.0)))
            time.sleep(2.0)
            _start_p4_controller_container(config, script_path=controller_script)
            frame_summary = _run_frame_probe(config, script_path=frame_probe_script)
            motion_summary = _run_motion_probe(config, script_path=motion_probe_script)
            fault_summary = _run_fault_probe(config, script_path=fault_probe_script)
            controller_summary = _wait_for_controller_summary(config, timeout_sec=30.0)
            rosbag_profile = _finish_p10_rosbag_recording(config)
            counts = _message_counts(config)

            graph = _collect_ros_graph(config, config.artifact_dir, image=baseline.runtime_image, network="host")
            probe = _collect_official_dds_probe(config, config.artifact_dir, image=baseline.runtime_image, network="host")
            x2_status = _collect_x2_status(config, image=baseline.runtime_image)
            rangefinder_probe = _collect_rangefinder_probe(config, image=baseline.runtime_image)
            imu_probe = _collect_imu_probe(config, image=baseline.runtime_image)
            slam_odom_probe = _collect_odometry_probe(config, image=baseline.runtime_image, topic=p3.slam_odom_topic, artifact_name="p10_slam_odom_probe.txt")
            topic_info = _collect_topic_info(
                config,
                image=baseline.runtime_image,
                topics=(
                    p10.validated_scan_topic,
                    p10.raw_scan_topic,
                    p10.normalized_scan_topic,
                    p10.status_topic,
                    p10.events_topic,
                    p10.scan_source_topic,
                    p10.x2_status_topic,
                    p10.attitude_source_topic,
                    p10.rangefinder_range_topic,
                    p3.slam_odom_topic,
                    p3.slam_status_topic,
                    p3.external_nav_status_topic,
                    p4.cmd_vel_topic,
                    p4.owner_status_topic,
                ),
                transient_topics=(p4.cmd_vel_topic, p4.owner_status_topic),
            )
            doctor = _build_p10_doctor_summary(config, runtime_config=p10_runtime_config, include_dependencies=False)
            blockers: list[str] = []
            if not doctor.get("ok"):
                blockers.extend(str(item) for item in doctor.get("blockers", []))
            if not probe.get("result", {}).get("time_received"):
                blockers.append("official DDS probe did not receive /ap/v1/time")
            x2_sample = x2_status.get("result", {}).get("sample") or {}
            if not x2_status.get("result", {}).get("received"):
                blockers.append("X2 status probe did not receive /sim/x2/status")
            if x2_sample.get("scan_source") != "gazebo_ideal":
                blockers.append("X2 emulator is not consuming Gazebo lidar input")
            if not rangefinder_preflight.get("result", {}).get("range_received"):
                blockers.append("P10 rangefinder preflight did not receive range data")
            if not rangefinder_probe.get("result", {}).get("range_received"):
                blockers.append("P10 did not receive rangefinder")
            if not imu_probe.get("result", {}).get("received"):
                blockers.append("P10 did not receive IMU")
            _append_slam_odom_quality_blockers(blockers=blockers, p3=p3, slam_odom_result=slam_odom_probe.get("result", {}))
            _append_controller_blockers(blockers=blockers, controller=controller_summary)
            owner_summary = motion_summary.get("owner", {}) if motion_summary else {}
            if not owner_summary and controller_summary:
                owner_summary = controller_summary.get("owner", {})
            cmd_vel_publishers = topic_info.get(p4.cmd_vel_topic, {}).get("publisher_nodes", [])
            _append_owner_blockers(blockers=blockers, owner_summary=owner_summary, cmd_vel_publishers=cmd_vel_publishers, p4=p4)
            _append_p5_blockers(blockers=blockers, frame_summary=frame_summary, rosbag_profile={"ok": True}, counts={p5.status_topic: max(1, counts.get(p5.status_topic, 0))}, p5=p5)
            _append_p10_blockers(blockers=blockers, motion_summary=motion_summary, fault_summary=fault_summary, rosbag_profile=rosbag_profile, counts=counts, topic_info=topic_info, p10=p10)
            latest_integrity = _latest_status_from_fault(fault_summary)
            normal_profile_ok = _p10_normal_profile_ok(motion_summary)
            flight_attitude_metrics = latest_integrity.get("flight_attitude_metrics", {})
            scan_attitude_quality = _scan_attitude_quality(latest_status=latest_integrity, ok=not blockers)
            motor_output = _motor_output_summary(ros_graph=graph)
            owner = owner_summary or {}
            summary = {
                "ok": not blockers,
                "blocked": bool(blockers),
                "blockers": blockers,
                "p10_scan_integrity_gate": {
                    "runtime_config": p10_runtime_summary,
                    "sensor_config": sensor_config_summary,
                    "model_overlay": model_overlay_summary,
                    "param_overlay": param_overlay_summary,
                    "slam_runtime_config": slam_runtime_summary,
                    "p4_runtime_config": p4_runtime_summary,
                    "p5_runtime_config": p5_runtime_summary,
                    "controller_script": controller_script_summary,
                    "frame_probe_script": frame_probe_script_summary,
                    "motion_probe_script": motion_probe_script_summary,
                    "fault_probe_script": fault_probe_script_summary,
                    "controller_runtime": controller_summary,
                    "external_nav_input_topic": p3.slam_odom_topic,
                    "uses_gazebo_truth_as_input": p10.uses_gazebo_truth_as_input,
                    "hover_claim": p10.hover_claim,
                    "motion_claim": p10.motion_claim,
                    "exploration_claim": p10.exploration_claim,
                    "scan_integrity_claim": p10.scan_integrity_claim,
                    "rosbag_path": str(config.artifact_dir / "rosbag"),
                    "mcap_path": str(config.artifact_dir / "rosbag" / "rosbag_0.mcap"),
                },
                "p6_hover_prerequisite": motion_summary.get("p6_hover_prerequisite", {}),
                "p7_motion_prerequisite": {
                    "ok": normal_profile_ok,
                    "source": "P10 reuses the P7 coordinator for small motion, but gates scan integrity on SLAM/ExternalNav/FCU/clearance health rather than P7 displacement thresholds",
                    "p7_displacement_gate_ok": bool(motion_summary.get("ok")),
                },
                "normal_profile": motion_summary,
                "fault_injection": fault_summary,
                "scan_integrity": {
                    "scan_owner": "navlab_scan_integrity_filter",
                    "raw_scan_topic": p10.raw_scan_topic,
                    "normalized_scan_topic": p10.normalized_scan_topic,
                    "validated_scan_topic": p10.validated_scan_topic,
                    "status_topic": p10.status_topic,
                    "attitude_source": p10.attitude_source_topic,
                    "attitude_source_type": p10.attitude_source_type,
                    "attitude_source_is_truth": False,
                    "base_scan_static_tf_ok": bool(latest_integrity.get("base_scan_static_tf_ok")),
                    "scan_source": "gazebo_ideal",
                    "normal_profile_ok": normal_profile_ok,
                    "fault_injection_ok": bool(fault_summary.get("ok")),
                    "fault_injection_mode": fault_summary.get("fault_injection_mode"),
                    "latest_status": latest_integrity,
                },
                "flight_attitude_metrics": flight_attitude_metrics,
                "scan_attitude_quality": scan_attitude_quality,
                "motor_output": motor_output,
                "motor_output_claim": motor_output["motor_output_claim"],
                "truth_control": {"set_pose_count": int(owner.get("set_pose_count", -1)), "gazebo_truth_as_input": False},
                "owner": owner,
                "frame_contract": frame_summary,
                "official_dds_probe": probe,
                "x2_status": x2_status.get("result", {}),
                "rangefinder_probe": rangefinder_probe.get("result", {}),
                "rangefinder_preflight": rangefinder_preflight.get("result", {}),
                "imu_probe": imu_probe.get("result", {}),
                "slam_odom": motion_summary.get("slam_odom", {}),
                "slam_odom_probe": slam_odom_probe.get("result", {}),
                "external_nav": motion_summary.get("external_nav", {}),
                "fcu": motion_summary.get("fcu", {}),
                "topic_info": topic_info,
                "ros_graph": graph,
                "message_counts": counts,
                "rosbag_profile": rosbag_profile,
                "hover_claim": p10.hover_claim,
                "motion_claim": p10.motion_claim,
                "exploration_claim": p10.exploration_claim,
                "scan_integrity_claim": p10.scan_integrity_claim,
                "uses_gazebo_truth_as_input": p10.uses_gazebo_truth_as_input,
            }
            _write_json(config.artifact_dir / "summary.json", summary)
            _write_foxglove_notes(config)
        finally:
            host._capture_official_baseline_log(config=config)
            _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_tail.log")
            _capture_container_log(config, container=SLAM_BACKEND_CONTAINER, output_name="slam_backend_tail.log")
            _capture_container_log(config, container=P4_CONTROLLER_CONTAINER, output_name="fcu_controller_tail.log")
            _capture_container_log(config, container=P10_ROSBAG_CONTAINER, output_name="rosbag_tail.log")
            _remove_container(P10_ROSBAG_CONTAINER)
            _remove_container(P4_CONTROLLER_CONTAINER)
            _remove_container(SLAM_BACKEND_CONTAINER)
            _remove_container(GAZEBO_SENSOR_CONTAINER)
            host._remove_official_baseline_container()
            try:
                host._compose_stop(config)
            except DockerException:
                pass
        if summary is None:
            summary = {"ok": False, "blocked": True, "blockers": ["P10 scan integrity gate acceptance did not produce a summary"]}
            _write_json(config.artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P10 scan integrity gate acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 30
