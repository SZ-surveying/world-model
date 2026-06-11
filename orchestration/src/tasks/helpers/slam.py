from __future__ import annotations

import json
import os
import shlex
from pathlib import Path
from typing import Any

import tomli_w
from python_on_whales import DockerClient

from src import host
from src.configs.run_config import RunConfig
from src.tasks.helpers.artifacts import file_sha256, write_text
from src.tasks.helpers.navlab_models import (
    remove_container,
)
from src.tasks.helpers.official_stack import (
    build_doctor_summary,
)
from src.tasks.helpers.rosbag_profiles import load_rosbag_metadata_counts

SLAM_BACKEND_CONTAINER = "navlab-p3-slam-backend"
OFFICIAL_IRIS_LIDAR_Z_M = "0.075077"
P3_ROSBAG_CONTAINER = "navlab-p3-rosbag"


def _baseline_env(config: RunConfig) -> dict[str, str]:
    baseline = config.orchestration.official_baseline
    return {
        "DDS_ENABLE": baseline.dds_enable,
        "DDS_DOMAIN_ID": baseline.dds_domain_id,
        "ROS_DOMAIN_ID": baseline.dds_domain_id,
        "RMW_IMPLEMENTATION": baseline.rmw_implementation,
    }


def write_p3_slam_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
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
                "base_frame": p3.base_frame_id,
                "imu_frame": p3.imu_frame_id,
                "laser_frame": p3.laser_frame_id,
                "laser_x": "0",
                "laser_y": "0",
                "laser_z": OFFICIAL_IRIS_LIDAR_Z_M,
                "laser_roll": "0",
                "laser_pitch": "0",
                "laser_yaw": "0",
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host.workspace_path(path), "sha256": file_sha256(path), "data": data}


def start_p3_slam_container(config: RunConfig, *, runtime_config: Path) -> None:
    remove_container(SLAM_BACKEND_CONTAINER)
    p3 = config.orchestration.slam_backend
    workspace_config = host.workspace_path(runtime_config)
    launch_command = " ".join(
        shlex.quote(arg)
        for arg in [
            "python3",
            "-m",
            "navlab.common.slam.cli",
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


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return load_rosbag_metadata_counts(metadata)


def collect_odometry_probe(config: RunConfig, *, image: str, topic: str, artifact_name: str) -> dict[str, Any]:
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
    rc, output = host.docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=f"python3 - <<'PY'\n{script}\nPY",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    write_text(config.artifact_dir / artifact_name, output)
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


def append_slam_odom_quality_blockers(*, blockers: list[str], p3: Any, slam_odom_result: dict[str, Any]) -> None:
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


def slam_backend_print_command(config: RunConfig, *, runtime_config: Path) -> tuple[int, str]:
    p3 = config.orchestration.slam_backend
    command = " ".join(
        shlex.quote(arg)
        for arg in [
            "python3",
            "-m",
            "navlab.common.slam.cli",
            "print-command",
            "--config",
            host.workspace_path(runtime_config),
            "--backend",
            p3.backend,
        ]
    )
    return host.docker_run_ros_shell_capture(
        config=config,
        image=config.slam_image,
        shell_command=f"source /opt/navlab_ws/install/setup.bash && {command}",
        name=None,
        network=None,
        envs={"NAVLAB_SLAM_RUNTIME_CONFIG": host.workspace_path(runtime_config)},
    )


def _write_foxglove_notes(config: RunConfig) -> None:
    p3 = config.orchestration.slam_backend
    write_text(
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


def build_p3_doctor_summary(config: RunConfig, *, runtime_config: Path) -> dict[str, Any]:
    p3 = config.orchestration.slam_backend
    blockers: list[str] = []
    baseline_doctor = build_doctor_summary(config)
    if not baseline_doctor.get("ok"):
        blockers.extend(str(item) for item in baseline_doctor.get("blockers", []))
    if p3.uses_gazebo_truth_as_input:
        blockers.append("P3 SLAM backend must not use Gazebo truth as the SLAM odom source")
    if p3.slam_odom_topic in {p3.truth_diagnostic_topic, p3.odometry_topic}:
        blockers.append("P3 canonical SLAM odom topic must be distinct from diagnostic truth / odometry input")
    rc, command = slam_backend_print_command(config, runtime_config=runtime_config)
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
            "runtime_config_sha256": file_sha256(runtime_config) if runtime_config.is_file() else "",
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
