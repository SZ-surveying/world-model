from __future__ import annotations

import json
import math
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
from src.tasks.official_baseline import (
    _build_doctor_summary,
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
    _collect_laser_scan_sample,
    _collect_topic_info,
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
    _collect_json_status_sample,
    _collect_rangefinder_probe,
    _write_p2_model_overlay,
    _write_p2_param_overlay,
    _write_p2_sensor_config,
)
from src.tasks.registry import TaskRegistry

SLAM_BACKEND_CONTAINER = "navlab-p3-slam-backend"
P3_ROSBAG_CONTAINER = "navlab-p3-rosbag"


def _baseline_env(config: RunConfig) -> dict[str, str]:
    baseline = config.orchestration.official_baseline
    return {
        "DDS_ENABLE": baseline.dds_enable,
        "DDS_DOMAIN_ID": baseline.dds_domain_id,
        "ROS_DOMAIN_ID": baseline.dds_domain_id,
        "RMW_IMPLEMENTATION": baseline.rmw_implementation,
    }


def _write_p3_slam_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    p3 = config.orchestration.slam_backend
    data = {
        "slam": {
            "runtime": {
                "backend": p3.backend,
                "use_sim_time": True,
                "launch_package": p3.launch_package,
                "launch_file": p3.launch_file,
                "launch_fake_odom": False,
                "launch_cartographer_backend": True,
                "publish_placeholder_odom": False,
                "cartographer_configuration_basename": p3.cartographer_configuration_basename,
                "imu_source_mode": "topic",
                "imu_source_topic": p3.imu_topic,
                "imu_source_label": "official_gazebo_imu_bridge",
                "imu_min_input_rate_hz": "4.0",
                "require_imu_for_external_nav": False,
                "require_height_for_external_nav": False,
                "external_nav_input_odom_topic": p3.slam_odom_topic,
                "external_nav_output_topic": "/external_nav/odom",
                "external_nav_status_topic": p3.external_nav_status_topic,
                "scan_topic": p3.scan_topic,
                "imu_topic": p3.imu_topic,
                "cartographer_odometry_topic": p3.odometry_topic,
                "odom_topic": p3.slam_odom_topic,
                "slam_status_topic": p3.slam_status_topic,
                "map_frame_id": "map",
                "odom_frame_id": p3.odom_frame_id,
                "base_frame_id": p3.base_frame_id,
                "imu_frame_id": p3.imu_frame_id,
                "laser_frame_id": p3.laser_frame_id,
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _start_p3_slam_container(config: RunConfig, *, runtime_config: Path) -> None:
    _remove_container(SLAM_BACKEND_CONTAINER)
    p3 = config.orchestration.slam_backend
    workspace_config = host._workspace_path(runtime_config)
    launch_command = " ".join(
        shlex.quote(arg)
        for arg in [
            "python3",
            "-m",
            "navlab.slam.cli",
            "launch",
            "--config",
            workspace_config,
            "--backend",
            p3.backend,
        ]
    )
    DockerClient().run(
        config.slam_image,
        [
            "bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash && "
            "source /opt/navlab_ws/install/setup.bash && "
            f"exec {launch_command}",
        ],
        detach=True,
        name=SLAM_BACKEND_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        user=f"{os.getuid()}:{os.getgid()}",
        envs={
            "SESSION_ID": config.session_id,
            "ROS_DOMAIN_ID": config.orchestration.official_baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": config.orchestration.official_baseline.rmw_implementation,
            "PYTHONPATH": "/workspace",
            "NAVLAB_SLAM_RUNTIME_CONFIG": workspace_config,
        },
    )


def _p3_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.slam_backend_rosbag_profile)
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
        "rc=$?; "
        "set -e; "
        'if [ "$rc" != "0" ] && [ "$rc" != "124" ] && [ "$rc" != "130" ]; then exit "$rc"; fi; '
        f"for i in $(seq 1 40); do [ -f {shlex.quote(str(container_rosbag / 'metadata.yaml'))} ] && exit 0; "
        "sleep 0.25; done; exit 2"
    )
    return profile_path, required, optional, command


def _record_p3_rosbag(config: RunConfig, *, image: str, duration_sec: float) -> dict[str, Any]:
    profile_path, required, optional, command = _p3_rosbag_shell_command(config, duration_sec=duration_sec)
    if not command:
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "reason": "rosbag profile missing or empty",
        }
        _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=command,
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "rosbag_record.txt", output)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if rc != 0 or not metadata.is_file():
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "reason": f"rosbag record failed rc={rc}",
            "record_output": output,
        }
        _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    summary = _validate_official_rosbag_profile(
        profile=profile_path,
        metadata=metadata,
        required=required,
        optional=optional,
    )
    summary["rosbag_path"] = str(config.artifact_dir / "rosbag")
    summary["mcap_path"] = str(config.artifact_dir / "rosbag" / "rosbag_0.mcap")
    _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
    return summary


def _start_p3_rosbag_recording(config: RunConfig, *, image: str, duration_sec: float) -> None:
    _remove_container(P3_ROSBAG_CONTAINER)
    profile_path, required, optional, command = _p3_rosbag_shell_command(config, duration_sec=duration_sec)
    if not command:
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "reason": "rosbag profile missing or empty",
        }
        _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
        return
    shell_command = (
        "source /opt/ros/jazzy/setup.bash && "
        "if [ -f /opt/navlab_ws/install/setup.bash ]; then source /opt/navlab_ws/install/setup.bash; fi && "
        "if [ -f /opt/navlab_official_ws/install/setup.bash ]; then "
        "source /opt/navlab_official_ws/install/setup.bash; fi && "
        f"{command}"
    )
    DockerClient().run(
        image,
        ["bash", "-lc", shell_command],
        detach=True,
        name=P3_ROSBAG_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={
            "ROS_DOMAIN_ID": config.orchestration.official_baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": config.orchestration.official_baseline.rmw_implementation,
            "PYTHONPATH": "/workspace",
            **_baseline_env(config),
        },
    )


def _finish_p3_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    profile_path = Path(config.slam_backend_rosbag_profile)
    required, optional, _topics = _profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    try:
        rc = DockerClient().wait(P3_ROSBAG_CONTAINER)
    except DockerException as exc:
        rc = exc.return_code or 1
    try:
        output = DockerClient().logs(P3_ROSBAG_CONTAINER, tail=2000)
    except DockerException as exc:
        output = str(exc)
    _write_text(config.artifact_dir / "rosbag_record.txt", str(output))
    if rc != 0 or not metadata.is_file():
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "reason": f"rosbag record failed rc={rc}",
            "record_output": str(output),
        }
        _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    summary = _validate_official_rosbag_profile(
        profile=profile_path,
        metadata=metadata,
        required=required,
        optional=optional,
    )
    summary["rosbag_path"] = str(config.artifact_dir / "rosbag")
    summary["mcap_path"] = str(config.artifact_dir / "rosbag" / "rosbag_0.mcap")
    _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
    return summary


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return _load_rosbag_metadata_counts(metadata)


def _collect_odometry_probe(config: RunConfig, *, image: str, topic: str, artifact_name: str) -> dict[str, Any]:
    script = f"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

topic = {topic!r}
result = {{
    "topic": topic,
    "received": False,
    "message_count": 0,
    "max_jump_m": 0.0,
    "max_yaw_jump_rad": 0.0,
    "stationary_drift_m": 0.0,
}}

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))

rclpy.init()
node = rclpy.create_node("navlab_p3_odometry_probe_" + topic.strip("/").replace("/", "_").replace("-", "_"))
started = time.monotonic()
last_time = None
first_pos = None
prev_pos = None
prev_yaw = None

def callback(msg):
    global first_pos, prev_pos, prev_yaw, last_time
    result["received"] = True
    result["message_count"] += 1
    last_time = time.monotonic()
    pos = msg.pose.pose.position
    q = msg.pose.pose.orientation
    point = (float(pos.x), float(pos.y), float(pos.z))
    yaw = yaw_from_quat(q)
    if first_pos is None:
        first_pos = point
    if prev_pos is not None:
        jump = math.dist(point, prev_pos)
        result["max_jump_m"] = max(float(result["max_jump_m"]), jump)
        result["max_yaw_jump_rad"] = max(float(result["max_yaw_jump_rad"]), abs(angle_delta(yaw, prev_yaw)))
    result["stationary_drift_m"] = max(float(result["stationary_drift_m"]), math.dist(point, first_pos))
    prev_pos = point
    prev_yaw = yaw
    result["frame_id"] = msg.header.frame_id
    result["child_frame_id"] = msg.child_frame_id
    result["latest_position"] = {{"x": point[0], "y": point[1], "z": point[2]}}
    result["latest_yaw_rad"] = yaw

subscription = node.create_subscription(Odometry, topic, callback, qos_profile_sensor_data)
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
result["sample_window_sec"] = time.monotonic() - started
result["rate_hz"] = result["message_count"] / max(result["sample_window_sec"], 0.001)
result["latest_age_sec"] = None if last_time is None else time.monotonic() - last_time
node.destroy_subscription(subscription)
node.destroy_node()
rclpy.shutdown()
print(json.dumps(result, sort_keys=True))
"""
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=f"python3 - <<'PY'\n{script}\nPY",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / artifact_name, output)
    return {"rc": rc, "output": output, "result": _last_json_line(output) if rc == 0 else {}}


def _collect_tf_probe(config: RunConfig, *, image: str) -> dict[str, Any]:
    p3 = config.orchestration.slam_backend
    script = f"""
import json
import time

import rclpy
from tf2_msgs.msg import TFMessage

wanted = {{
    ("map", {p3.odom_frame_id!r}): "map_to_odom",
    ({p3.odom_frame_id!r}, {p3.base_frame_id!r}): "odom_to_base",
    ({p3.base_frame_id!r}, {p3.imu_frame_id!r}): "base_to_imu",
    ({p3.base_frame_id!r}, {p3.laser_frame_id!r}): "base_to_laser",
}}
result = {{"received_count": 0, "transforms": {{key: False for key in wanted.values()}}}}
rclpy.init()
node = rclpy.create_node("navlab_p3_tf_probe")

def callback(msg):
    result["received_count"] += len(msg.transforms)
    for transform in msg.transforms:
        key = (transform.header.frame_id, transform.child_frame_id)
        if key in wanted:
            result["transforms"][wanted[key]] = True

sub_tf = node.create_subscription(TFMessage, "/tf", callback, 10)
sub_static = node.create_subscription(TFMessage, "/tf_static", callback, 10)
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_subscription(sub_tf)
node.destroy_subscription(sub_static)
node.destroy_node()
rclpy.shutdown()
print(json.dumps(result, sort_keys=True))
"""
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=f"python3 - <<'PY'\n{script}\nPY",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "tf_probe.txt", output)
    return {"rc": rc, "output": output, "result": _last_json_line(output) if rc == 0 else {}}


def _collect_truth_error_probe(config: RunConfig, *, image: str) -> dict[str, Any]:
    p3 = config.orchestration.slam_backend
    script = f"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

slam_topic = {p3.slam_odom_topic!r}
truth_topic = {p3.truth_diagnostic_topic!r}
result = {{
    "slam_topic": slam_topic,
    "truth_diagnostic_topic": truth_topic,
    "slam_count": 0,
    "truth_count": 0,
    "paired_count": 0,
    "error_mean_m": None,
    "error_p95_m": None,
    "error_max_m": None,
}}
slam_samples = []
truth_samples = []

def point(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), float(p.z))

rclpy.init()
node = rclpy.create_node("navlab_p3_truth_error_probe")

def handle_slam(msg):
    result["slam_count"] += 1
    slam_samples.append((time.monotonic(), point(msg)))

def handle_truth(msg):
    result["truth_count"] += 1
    truth_samples.append((time.monotonic(), point(msg)))

slam_sub = node.create_subscription(Odometry, slam_topic, handle_slam, qos_profile_sensor_data)
truth_sub = node.create_subscription(Odometry, truth_topic, handle_truth, qos_profile_sensor_data)
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
errors = []
for stamp, slam_point in slam_samples:
    if not truth_samples:
        continue
    nearest_stamp, truth_point = min(truth_samples, key=lambda sample: abs(sample[0] - stamp))
    if abs(nearest_stamp - stamp) <= 0.25:
        errors.append(math.dist(slam_point, truth_point))
errors.sort()
result["paired_count"] = len(errors)
if errors:
    result["error_mean_m"] = sum(errors) / len(errors)
    result["error_max_m"] = errors[-1]
    result["error_p95_m"] = errors[min(len(errors) - 1, int(math.ceil(0.95 * len(errors))) - 1)]
node.destroy_subscription(slam_sub)
node.destroy_subscription(truth_sub)
node.destroy_node()
rclpy.shutdown()
print(json.dumps(result, sort_keys=True))
"""
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=f"python3 - <<'PY'\n{script}\nPY",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "truth_error_probe.txt", output)
    return {"rc": rc, "output": output, "result": _last_json_line(output) if rc == 0 else {}}


def _last_json_line(output: str) -> dict[str, Any]:
    for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        return value if isinstance(value, dict) else {}
    return {}


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _append_slam_odom_quality_blockers(*, blockers: list[str], p3: Any, slam_odom_result: dict[str, Any]) -> None:
    if not slam_odom_result.get("received"):
        blockers.append(f"P3 did not receive {p3.slam_odom_topic}")
    if slam_odom_result.get("frame_id") != p3.odom_frame_id:
        blockers.append(f"{p3.slam_odom_topic} frame_id is not {p3.odom_frame_id!r}")
    if slam_odom_result.get("child_frame_id") != p3.base_frame_id:
        blockers.append(f"{p3.slam_odom_topic} child_frame_id is not {p3.base_frame_id!r}")
    if float(slam_odom_result.get("rate_hz", 0.0) or 0.0) < p3.min_slam_odom_rate_hz:
        blockers.append("SLAM odom rate is below minimum")
    latest_age = _float_or_none(slam_odom_result.get("latest_age_sec"))
    if latest_age is None or latest_age > p3.max_latest_age_sec:
        blockers.append("SLAM odom latest age is too high")
    max_jump_m = _float_or_none(slam_odom_result.get("max_jump_m"))
    if max_jump_m is None or max_jump_m > p3.max_jump_m:
        blockers.append("SLAM odom jump exceeds threshold")
    max_yaw_jump_rad = _float_or_none(slam_odom_result.get("max_yaw_jump_rad"))
    if max_yaw_jump_rad is None or max_yaw_jump_rad > p3.max_yaw_jump_rad:
        blockers.append("SLAM odom yaw jump exceeds threshold")
    stationary_drift_m = _float_or_none(slam_odom_result.get("stationary_drift_m"))
    if stationary_drift_m is None or stationary_drift_m > p3.max_stationary_drift_m:
        blockers.append("SLAM odom stationary drift exceeds threshold")


def _slam_backend_print_command(config: RunConfig, *, runtime_config: Path) -> tuple[int, str]:
    p3 = config.orchestration.slam_backend
    command = " ".join(
        shlex.quote(arg)
        for arg in [
            "python3",
            "-m",
            "navlab.slam.cli",
            "print-command",
            "--config",
            host._workspace_path(runtime_config),
            "--backend",
            p3.backend,
        ]
    )
    return host._docker_run_ros_shell_capture(
        config=config,
        image=config.slam_image,
        shell_command=f"source /opt/navlab_ws/install/setup.bash && {command}",
        name=None,
        network=None,
        envs={"NAVLAB_SLAM_RUNTIME_CONFIG": host._workspace_path(runtime_config)},
    )


def _write_foxglove_notes(config: RunConfig) -> None:
    p3 = config.orchestration.slam_backend
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P3 SLAM backend replay notes",
                "",
                "P3 validates the SLAM backend as an observation pipeline. It does not validate hover, path tracking, or ExternalNav control.",
                "",
                "- Fixed frame: `map`.",
                f"- SLAM canonical odom: `{p3.slam_odom_topic}`.",
                f"- SLAM status: `{p3.slam_status_topic}`.",
                f"- Inputs: `{p3.scan_topic}`, `{p3.imu_topic}`, `{p3.odometry_topic}`, `/tf`, `/tf_static`.",
                "- Cartographer map topics: `/map`, `/submap_list`, `/trajectory_node_list`.",
                f"- Diagnostic truth comparison topic: `{p3.truth_diagnostic_topic}`.",
                "- Do not interpret this bag as a flight-complete acceptance. This phase only proves backend quality gates.",
            ]
        )
        + "\n",
    )


def _build_p3_doctor_summary(config: RunConfig, *, runtime_config: Path) -> dict[str, Any]:
    p3 = config.orchestration.slam_backend
    blockers: list[str] = []
    baseline_doctor = _build_doctor_summary(config)
    if not baseline_doctor.get("ok"):
        blockers.extend(str(item) for item in baseline_doctor.get("blockers", []))
    if p3.uses_gazebo_truth_as_input:
        blockers.append("P3 SLAM backend must not use Gazebo truth as the SLAM odom source")
    if p3.slam_odom_topic in {p3.truth_diagnostic_topic, p3.odometry_topic}:
        blockers.append("P3 canonical SLAM odom topic must be distinct from diagnostic truth / odometry input")
    rc, command = _slam_backend_print_command(config, runtime_config=runtime_config)
    if rc != 0:
        blockers.append(f"SLAM backend command could not be built rc={rc}")
    summary = {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "p3_slam_backend_doctor": {
            "backend": p3.backend,
            "slam_image": config.slam_image,
            "runtime_config": str(runtime_config),
            "runtime_config_sha256": _file_sha256(runtime_config) if runtime_config.is_file() else "",
            "command": command.strip(),
            "scan_topic": p3.scan_topic,
            "imu_topic": p3.imu_topic,
            "odometry_topic": p3.odometry_topic,
            "slam_odom_topic": p3.slam_odom_topic,
            "slam_status_topic": p3.slam_status_topic,
            "truth_diagnostic_topic": p3.truth_diagnostic_topic,
            "uses_gazebo_truth_as_input": p3.uses_gazebo_truth_as_input,
            "placeholder_odom_allowed": False,
            "fake_odom_allowed": False,
        },
        "official_baseline_doctor": baseline_doctor,
    }
    return summary


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class SlamBackendDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "slam-backend-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check P3 SLAM backend quality prerequisites."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(f"artifacts/ros/navlab_slam_backend_doctor/{config.run_id}")
        artifact_dir.mkdir(parents=True, exist_ok=True)
        runtime_config = artifact_dir / "p3_slam_runtime.toml"
        _write_p3_slam_runtime_config(config, runtime_config)
        console.print("[bold cyan]Checking P3 SLAM backend prerequisites[/bold cyan]")
        summary = _build_p3_doctor_summary(config, runtime_config=runtime_config)
        _write_json(artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P3 SLAM backend doctor rc={0 if summary['ok'] else 20}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 20


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class SlamBackendAcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "slam-backend-acceptance"
    TASK_DESCRIPTION: ClassVar[str] = "Run P3 SLAM backend quality acceptance."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        duration_sec: float = 90.0,
        console: Console | None = None,
    ) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
        config.artifact_dir.mkdir(parents=True, exist_ok=True)
        host._render_run_config(console, config)
        baseline = config.orchestration.official_baseline
        p2 = config.orchestration.rangefinder_imu
        p3 = config.orchestration.slam_backend
        bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p3.yaml"
        model_overlay = config.artifact_dir / "iris_with_lidar_p3_rangefinder_x2.sdf"
        param_overlay = config.artifact_dir / "gazebo-iris-p3-rangefinder.parm"
        sensor_config = config.artifact_dir / "p3_gazebo_sensor_runtime.toml"
        vendor_profile = config.artifact_dir / "x2_vendor_driver_p3.yaml"
        slam_runtime_config = config.artifact_dir / "p3_slam_runtime.toml"
        _write_p1_bridge_override(bridge_override)
        _write_p1_vendor_profile(vendor_profile, virtual_serial_link=p2.x2_virtual_serial_link)

        summary: dict[str, Any] | None = None
        try:
            model_overlay_summary = _write_p2_model_overlay(config, model_overlay)
            param_overlay_summary = _write_p2_param_overlay(config, param_overlay)
            sensor_config_summary = _write_p2_sensor_config(config, sensor_config, vendor_profile=vendor_profile)
            slam_runtime_summary = _write_p3_slam_runtime_config(config, slam_runtime_config)

            console.print("[bold cyan]Starting official maze + NavLab SLAM backend quality gate[/bold cyan]")
            try:
                host._compose_stop(config)
            except DockerException:
                pass
            host._start_official_baseline_container(
                config,
                volume_overrides=[
                    (bridge_override.resolve(), OFFICIAL_IRIS_3D_BRIDGE_CONFIG),
                    (model_overlay.resolve(), OFFICIAL_IRIS_WITH_LIDAR_MODEL),
                    (param_overlay.resolve(), OFFICIAL_GAZEBO_IRIS_PARAMS),
                ],
            )
            time.sleep(min(max(duration_sec, 1.0), 10.0))
            _start_gazebo_sensor_container(config, sensor_config=sensor_config)
            time.sleep(8.0)
            _start_p3_rosbag_recording(
                config,
                image=baseline.runtime_image,
                duration_sec=max(45.0, min(duration_sec, 90.0)),
            )
            time.sleep(2.0)
            _start_p3_slam_container(config, runtime_config=slam_runtime_config)
            time.sleep(min(max(duration_sec - 18.0, 10.0), 25.0))

            graph = _collect_ros_graph(
                config,
                config.artifact_dir,
                image=baseline.runtime_image,
                network="host",
            )
            probe = _collect_official_dds_probe(
                config,
                config.artifact_dir,
                image=baseline.runtime_image,
                network="host",
            )
            topic_info = _collect_topic_info(
                config,
                image=baseline.runtime_image,
                topics=(
                    p3.scan_topic,
                    p3.imu_topic,
                    p3.odometry_topic,
                    p3.slam_odom_topic,
                    p3.slam_status_topic,
                    p3.external_nav_status_topic,
                    p3.x2_vendor_scan_topic,
                    p3.x2_status_topic,
                    p3.rangefinder_range_topic,
                    p3.rangefinder_status_topic,
                    "/map",
                    "/submap_list",
                    "/trajectory_node_list",
                ),
            )
            x2_status = _collect_json_status_sample(
                config,
                image=baseline.runtime_image,
                topic=p3.x2_status_topic,
                artifact_name="x2_status_probe.txt",
                node_name="navlab_p3_x2_status_probe",
            )
            slam_status = _collect_json_status_sample(
                config,
                image=baseline.runtime_image,
                topic=p3.slam_status_topic,
                artifact_name="slam_status_probe.txt",
                node_name="navlab_p3_slam_status_probe",
            )
            external_nav_status = _collect_json_status_sample(
                config,
                image=baseline.runtime_image,
                topic=p3.external_nav_status_topic,
                artifact_name="external_nav_status_probe.txt",
                node_name="navlab_p3_external_nav_status_probe",
            )
            scan_sample = _collect_laser_scan_sample(config, image=baseline.runtime_image, topic=p3.scan_topic)
            imu_probe = _collect_imu_probe(config, image=baseline.runtime_image)
            rangefinder_probe = _collect_rangefinder_probe(config, image=baseline.runtime_image)
            slam_odom_probe = _collect_odometry_probe(
                config,
                image=baseline.runtime_image,
                topic=p3.slam_odom_topic,
                artifact_name="slam_odom_probe.txt",
            )
            truth_probe = _collect_odometry_probe(
                config,
                image=baseline.runtime_image,
                topic=p3.truth_diagnostic_topic,
                artifact_name="truth_odometry_probe.txt",
            )
            tf_probe = _collect_tf_probe(config, image=baseline.runtime_image)
            truth_error_probe = _collect_truth_error_probe(config, image=baseline.runtime_image)
            rosbag_profile = _finish_p3_rosbag_recording(config)
            counts = _message_counts(config)

            scan_publishers = topic_info.get(p3.scan_topic, {}).get("publisher_nodes", [])
            scan_subscribers = topic_info.get(p3.scan_topic, {}).get("subscription_nodes", [])
            vendor_scan_publishers = topic_info.get(p3.x2_vendor_scan_topic, {}).get("publisher_nodes", [])
            vendor_scan_subscribers = topic_info.get(p3.x2_vendor_scan_topic, {}).get("subscription_nodes", [])
            imu_publishers = topic_info.get(p3.imu_topic, {}).get("publisher_nodes", [])
            imu_subscribers = topic_info.get(p3.imu_topic, {}).get("subscription_nodes", [])
            odom_publishers = topic_info.get(p3.slam_odom_topic, {}).get("publisher_nodes", [])
            slam_status_sample = slam_status.get("result", {}).get("sample") or {}
            slam_odom_result = slam_odom_probe.get("result", {})
            truth_result = truth_probe.get("result", {})
            tf_result = tf_probe.get("result", {})
            truth_error_result = truth_error_probe.get("result", {})

            blockers: list[str] = []
            doctor = _build_p3_doctor_summary(config, runtime_config=slam_runtime_config)
            if not doctor.get("ok"):
                blockers.extend(str(item) for item in doctor.get("blockers", []))
            if not probe.get("result", {}).get("time_received"):
                blockers.append("official DDS probe did not receive /ap/v1/time")
            if "ydlidar_ros2_driver_node" not in vendor_scan_publishers:
                blockers.append(f"{p3.x2_vendor_scan_topic} is not published by ydlidar_ros2_driver_node")
            if "navlab_x2_scan_time_normalizer" not in scan_publishers:
                blockers.append(f"{p3.scan_topic} is not published by navlab_x2_scan_time_normalizer")
            if any("ros_gz_bridge" in publisher for publisher in scan_publishers):
                blockers.append("/scan is still published by ros_gz_bridge; X2 route is polluted")
            if "cartographer_node" not in scan_subscribers:
                blockers.append("cartographer_node is not subscribed to /scan")
            if "navlab_cartographer_adapter_node" not in scan_subscribers:
                blockers.append("navlab_cartographer_adapter_node is not subscribed to /scan")
            if "navlab_cartographer_adapter_node" not in odom_publishers:
                blockers.append(f"{p3.slam_odom_topic} is not published by navlab_cartographer_adapter_node")
            if not scan_sample.get("result", {}).get("received"):
                blockers.append(f"P3 did not receive {p3.scan_topic} LaserScan")
            if not imu_probe.get("result", {}).get("received"):
                blockers.append(f"P3 did not receive {p3.imu_topic} IMU")
            if float(imu_probe.get("result", {}).get("rate_hz", 0.0) or 0.0) < 4.0:
                blockers.append("P3 IMU rate is below minimum")
            rangefinder_result = rangefinder_probe.get("result", {})
            rangefinder_status = rangefinder_result.get("status_sample") or {}
            if not rangefinder_result.get("range_received"):
                blockers.append(f"P3 did not receive {p3.rangefinder_range_topic}")
            if rangefinder_status.get("state") != "sending":
                blockers.append("P3 rangefinder status is not sending")
            if slam_status_sample.get("state") != "publishing_tf_backed_odom":
                blockers.append("SLAM adapter is not publishing TF-backed odom")
            _append_slam_odom_quality_blockers(blockers=blockers, p3=p3, slam_odom_result=slam_odom_result)
            transforms = tf_result.get("transforms", {})
            if not transforms.get("odom_to_base"):
                blockers.append("TF missing odom -> base_link")
            if not transforms.get("base_to_imu"):
                blockers.append("TF missing base_link -> imu_link")
            if not transforms.get("base_to_laser"):
                blockers.append("TF missing base_link -> base_scan")
            if not rosbag_profile.get("ok"):
                blockers.append("P3 rosbag profile did not pass")
            for topic in (p3.scan_topic, p3.imu_topic, p3.odometry_topic, p3.slam_odom_topic, "/map"):
                if counts.get(topic, 0) <= 0:
                    blockers.append(f"{topic} was not recorded")

            quality = {
                "canonical_slam_odom_topic": p3.slam_odom_topic,
                "status_topic": p3.slam_status_topic,
                "backend": p3.backend,
                "slam_status": slam_status_sample,
                "slam_odom_probe": slam_odom_result,
                "truth_diagnostic_topic": p3.truth_diagnostic_topic,
                "truth_diagnostic_probe": truth_result,
                "truth_error_diagnostic": truth_error_result,
                "uses_gazebo_truth_as_input": p3.uses_gazebo_truth_as_input,
                "thresholds": {
                    "min_slam_odom_rate_hz": p3.min_slam_odom_rate_hz,
                    "max_latest_age_sec": p3.max_latest_age_sec,
                    "max_jump_m": p3.max_jump_m,
                    "max_yaw_jump_rad": p3.max_yaw_jump_rad,
                    "max_stationary_drift_m": p3.max_stationary_drift_m,
                },
            }
            slam_inputs = {
                "scan_topic": p3.scan_topic,
                "scan_publishers": scan_publishers,
                "scan_subscribers": scan_subscribers,
                "vendor_scan_topic": p3.x2_vendor_scan_topic,
                "vendor_scan_publishers": vendor_scan_publishers,
                "vendor_scan_subscribers": vendor_scan_subscribers,
                "scan_sample": scan_sample.get("result", {}),
                "imu_topic": p3.imu_topic,
                "imu_publishers": imu_publishers,
                "imu_subscribers": imu_subscribers,
                "imu_probe": imu_probe.get("result", {}),
                "odometry_topic": p3.odometry_topic,
                "odometry_publishers": topic_info.get(p3.odometry_topic, {}).get("publisher_nodes", []),
                "odometry_subscribers": topic_info.get(p3.odometry_topic, {}).get("subscription_nodes", []),
                "truth_diagnostic_topic": p3.truth_diagnostic_topic,
                "uses_gazebo_truth_as_input": p3.uses_gazebo_truth_as_input,
            }
            slam_outputs = {
                "slam_odom_topic": p3.slam_odom_topic,
                "slam_odom_publishers": odom_publishers,
                "slam_odom_probe": slam_odom_result,
                "slam_status_topic": p3.slam_status_topic,
                "slam_status": slam_status_sample,
                "map_topics": {
                    "/map": {"count": counts.get("/map", 0), **topic_info.get("/map", {})},
                    "/submap_list": {
                        "count": counts.get("/submap_list", 0),
                        **topic_info.get("/submap_list", {}),
                    },
                    "/trajectory_node_list": {
                        "count": counts.get("/trajectory_node_list", 0),
                        **topic_info.get("/trajectory_node_list", {}),
                    },
                },
            }
            p3_summary = {
                "world_source": config.orchestration.official_maze_x2.world_source,
                "vehicle_model_source": config.orchestration.official_maze_x2.vehicle_model_source,
                "model_overlay": model_overlay_summary,
                "param_overlay": param_overlay_summary,
                "sensor_config": sensor_config_summary,
                "slam_runtime_config": slam_runtime_summary,
                "slam_backend_doctor": doctor.get("p3_slam_backend_doctor", {}),
                "slam_launch_command": doctor.get("p3_slam_backend_doctor", {}).get("command", ""),
                "backend_image": config.slam_image,
                "backend_container": SLAM_BACKEND_CONTAINER,
                "gazebo_sensor_container": GAZEBO_SENSOR_CONTAINER,
                "official_container": host.OFFICIAL_BASELINE_CONTAINER,
                "scan_publishers": scan_publishers,
                "scan_subscribers": scan_subscribers,
                "vendor_scan_topic": p3.x2_vendor_scan_topic,
                "vendor_scan_publishers": vendor_scan_publishers,
                "vendor_scan_subscribers": vendor_scan_subscribers,
                "imu_publishers": imu_publishers,
                "imu_subscribers": imu_subscribers,
                "slam_odom_publishers": odom_publishers,
                "x2_status": x2_status.get("result", {}),
                "external_nav_status": external_nav_status.get("result", {}),
                "rangefinder": rangefinder_probe.get("result", {}),
                "imu": imu_probe.get("result", {}),
                "tf": tf_result,
                "message_counts": counts,
                "rosbag_path": str(config.artifact_dir / "rosbag"),
                "mcap_path": str(config.artifact_dir / "rosbag" / "rosbag_0.mcap"),
                "direct_set_pose": False,
                "hover_claim": "not_evaluated",
                "external_nav_claim": "not_evaluated",
            }
            summary = {
                "ok": not blockers,
                "blocked": bool(blockers),
                "blockers": blockers,
                "p3_slam_backend": p3_summary,
                "slam_inputs": slam_inputs,
                "slam_outputs": slam_outputs,
                "slam_quality": quality,
                "ros_graph": graph,
                "official_dds_probe": probe,
                "topic_info": topic_info,
                "rosbag_profile": rosbag_profile,
            }
            _write_json(config.artifact_dir / "summary.json", summary)
            _write_foxglove_notes(config)
        finally:
            host._capture_official_baseline_log(config=config)
            _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_tail.log")
            _capture_container_log(config, container=SLAM_BACKEND_CONTAINER, output_name="slam_backend_tail.log")
            _capture_container_log(config, container=P3_ROSBAG_CONTAINER, output_name="rosbag_tail.log")
            _remove_container(P3_ROSBAG_CONTAINER)
            _remove_container(SLAM_BACKEND_CONTAINER)
            _remove_container(GAZEBO_SENSOR_CONTAINER)
            host._remove_official_baseline_container()
            try:
                host._compose_stop(config)
            except DockerException:
                pass
        if summary is None:
            summary = {
                "ok": False,
                "blocked": True,
                "blockers": ["P3 SLAM backend acceptance did not produce a summary"],
            }
            _write_json(config.artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P3 SLAM backend acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 30
