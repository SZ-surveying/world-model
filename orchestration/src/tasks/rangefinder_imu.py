from __future__ import annotations

import hashlib
import json
import shlex
import textwrap
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

import tomli_w
from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.config import RunConfig
from src.tasks.base import OrchestrationTask
from src.tasks.official_baseline import (
    _build_doctor_summary,
    _cartographer_dependency_summary,
    _collect_official_dds_probe,
    _collect_ros_graph,
    _load_rosbag_metadata_counts,
    _official_baseline_common,
    _official_ros_dependency_summary,
    _validate_official_rosbag_profile,
    _write_json,
    _write_text,
)
from src.tasks.official_maze_x2 import (
    CARTOGRAPHER_CONTAINER,
    GAZEBO_SENSOR_CONTAINER,
    OFFICIAL_IRIS_3D_BRIDGE_CONFIG,
    _capture_container_log,
    _collect_laser_scan_sample,
    _collect_topic_info,
    _file_sha256,
    _official_cartographer_config_summary,
    _profile_topics,
    _remove_container,
    _start_gazebo_sensor_container,
    _write_p1_bridge_override,
    _write_p1_vendor_profile,
)
from src.tasks.registry import TaskRegistry

OFFICIAL_IRIS_WITH_LIDAR_MODEL = (
    "/opt/navlab_official_ws/install/ardupilot_gz_description/share/"
    "ardupilot_gz_description/models/iris_with_lidar/model.sdf"
)
OFFICIAL_GAZEBO_IRIS_PARAMS = (
    "/opt/navlab_official_ws/install/ardupilot_sitl/share/"
    "ardupilot_sitl/config/default_params/gazebo-iris.parm"
)


def _docker_cat(config: RunConfig, *, image: str, path: str) -> str:
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=f"cat {shlex.quote(path)}",
        name=None,
        network=None,
    )
    if rc != 0:
        raise RuntimeError(f"failed to read {path} from {image}: {output.strip()}")
    return output


def _write_p2_model_overlay(config: RunConfig, path: Path) -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    source = _docker_cat(
        config,
        image=config.orchestration.official_baseline.runtime_image,
        path=OFFICIAL_IRIS_WITH_LIDAR_MODEL,
    )
    if "</model>" not in source:
        raise RuntimeError("official iris_with_lidar model does not contain a closing </model> tag")
    x2_lidar_model_source = "official_iris_with_lidar"
    if "model://lidar_3d" in source:
        source = source.replace("model://lidar_3d", "model://lidar_2d", 1)
        x2_lidar_model_source = "official_iris_with_lidar_2d_laserscan_overlay"
    overlay = textwrap.dedent(
        f"""\

            <!-- NavLab P2 overlay: down-facing rangefinder ray sensor.
                 The X2 vendor-driver input uses the official iris_with_lidar
                 /lidar sensor; do not add a second /lidar sensor here. -->
            <link name="{p2.rangefinder_frame_id}">
              <pose relative_to="base_link">{p2.rangefinder_model_pose}</pose>
              <inertial>
                <mass>0.02</mass>
                <inertia>
                  <ixx>0.00001</ixx>
                  <iyy>0.00001</iyy>
                  <izz>0.00001</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.02 0.02 0.01</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.02 0.02 0.01</size>
                  </box>
                </geometry>
              </visual>
              <sensor name="down_rangefinder_sensor" type="gpu_lidar">
                <gz_frame_id>{p2.rangefinder_frame_id}</gz_frame_id>
                <pose>0 0 0 0 0 0</pose>
                <topic>{p2.rangefinder_scan_ideal_topic.lstrip("/")}</topic>
                <always_on>true</always_on>
                <update_rate>{p2.rangefinder_model_update_rate_hz:g}</update_rate>
                <lidar>
                  <scan>
                    <horizontal>
                      <samples>{p2.rangefinder_model_ray_count}</samples>
                      <resolution>1</resolution>
                      <min_angle>-0.02</min_angle>
                      <max_angle>0.02</max_angle>
                    </horizontal>
                    <vertical>
                      <samples>1</samples>
                      <resolution>1</resolution>
                      <min_angle>0.0</min_angle>
                      <max_angle>0.0</max_angle>
                    </vertical>
                  </scan>
                  <range>
                    <min>{p2.rangefinder_min_distance_m:g}</min>
                    <max>{p2.rangefinder_max_distance_m:g}</max>
                    <resolution>0.01</resolution>
                  </range>
                </lidar>
                <visualize>false</visualize>
              </sensor>
            </link>

            <joint name="rangefinder_down_joint" type="fixed">
              <parent>base_link</parent>
              <child>{p2.rangefinder_frame_id}</child>
            </joint>
        """
    )
    rendered = source.replace("</model>", overlay + "\n  </model>", 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(rendered, encoding="utf-8")
    return {
        "source_path": OFFICIAL_IRIS_WITH_LIDAR_MODEL,
        "overlay_path": str(path),
        "overlay_sha256": _file_sha256(path),
        "model_overlay_source": p2.model_overlay_source,
        "x2_sensor_topic": p2.x2_scan_input_topic,
        "x2_sensor_frame_id": "base_scan",
        "x2_sensor_source": x2_lidar_model_source,
        "sensor_topic": p2.rangefinder_scan_ideal_topic,
        "sensor_type": "gpu_lidar",
        "frame_id": p2.rangefinder_frame_id,
        "pose": p2.rangefinder_model_pose,
        "update_rate_hz": p2.rangefinder_model_update_rate_hz,
        "ray_count": p2.rangefinder_model_ray_count,
        "noise_stddev_m": p2.rangefinder_model_noise_stddev_m,
    }


def _write_p2_param_overlay(config: RunConfig, path: Path) -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    source = _docker_cat(
        config,
        image=config.orchestration.official_baseline.runtime_image,
        path=OFFICIAL_GAZEBO_IRIS_PARAMS,
    )
    min_cm = int(round(p2.rangefinder_min_distance_m * 100.0))
    max_cm = int(round(p2.rangefinder_max_distance_m * 100.0))
    orientation = 25 if p2.rangefinder_mavlink_orientation == "MAV_SENSOR_ROTATION_PITCH_270" else 25
    overlay = textwrap.dedent(
        f"""\

        # NavLab P2 MAVLink down rangefinder overlay.
        RNGFND1_TYPE 10
        RNGFND1_ORIENT {orientation}
        RNGFND1_MIN_CM {min_cm}
        RNGFND1_MAX_CM {max_cm}
        RNGFND1_GNDCLEAR 10
        """
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(source.rstrip() + "\n" + overlay, encoding="utf-8")
    return {
        "source_path": OFFICIAL_GAZEBO_IRIS_PARAMS,
        "overlay_path": str(path),
        "overlay_sha256": _file_sha256(path),
        "parameters": {
            "RNGFND1_TYPE": 10,
            "RNGFND1_ORIENT": orientation,
            "RNGFND1_MIN_CM": min_cm,
            "RNGFND1_MAX_CM": max_cm,
            "RNGFND1_GNDCLEAR": 10,
        },
    }


def _write_p2_sensor_config(config: RunConfig, path: Path, *, vendor_profile: Path) -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "gazebo_sensor": {
            "x2_protocol": {
                "enabled": True,
                "scan_source": "x2_virtual_serial",
                "profile": host._workspace_path(vendor_profile),
                "virtual_serial_link": p2.x2_virtual_serial_link,
                "scan_ideal_topic": p2.x2_scan_input_topic,
                "vendor_scan_topic": "/navlab/x2/vendor_scan",
                "scan_topic": p2.x2_scan_topic,
                "status_topic": p2.x2_status_topic,
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
        }
    }
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {
        "path": str(path),
        "sha256": _file_sha256(path),
        "data": data,
    }


def _start_p2_cartographer_container(config: RunConfig) -> None:
    _remove_container(CARTOGRAPHER_CONTAINER)
    baseline = config.orchestration.official_baseline
    launch_command = f"{config.orchestration.rangefinder_imu.cartographer_launch} rviz:=false"
    command = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /opt/navlab_official_ws/install/setup.bash && "
        "export NAVLAB_OFFICIAL_SDF_ROOTS="
        "/opt/navlab_official_ws/install/ardupilot_gazebo/share:"
        "/opt/navlab_official_ws/install/ardupilot_gz_description/share && "
        "export SDF_PATH=${NAVLAB_OFFICIAL_SDF_ROOTS}:${SDF_PATH:-} && "
        "export GZ_SIM_RESOURCE_PATH=${NAVLAB_OFFICIAL_SDF_ROOTS}:${GZ_SIM_RESOURCE_PATH:-} && "
        f"exec {launch_command}"
    )
    from python_on_whales import DockerClient

    DockerClient().run(
        baseline.runtime_image,
        ["bash", "-lc", command],
        detach=True,
        name=CARTOGRAPHER_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={
            "SESSION_ID": config.session_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "PYTHONPATH": "/workspace",
        },
    )


def _record_rosbag(config: RunConfig, *, image: str, duration_sec: float) -> dict[str, Any]:
    profile_path = Path(config.rangefinder_imu_rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    if not profile_path.is_file() or not topics:
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
    baseline = config.orchestration.official_baseline
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=command,
        name=None,
        network="host",
        envs={
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
        },
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
    _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
    return summary


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return _load_rosbag_metadata_counts(metadata)


def _collect_rangefinder_probe(config: RunConfig, *, image: str, artifact_name: str = "rangefinder_probe.txt") -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    baseline = config.orchestration.official_baseline
    script = f"""
import json
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String

range_topic = {p2.rangefinder_range_topic!r}
status_topic = {p2.rangefinder_status_topic!r}
result = {{
    "range_topic": range_topic,
    "status_topic": status_topic,
    "range_received": False,
    "range_count": 0,
    "status_received": False,
    "status_sample": None,
}}

rclpy.init()
node = rclpy.create_node("navlab_p2_rangefinder_probe")
started = time.monotonic()
last_range_time = None
last_status_time = None

def handle_range(msg):
    global last_range_time
    result["range_received"] = True
    result["range_count"] += 1
    last_range_time = time.monotonic()
    result["frame_id"] = msg.header.frame_id
    result["latest_distance_m"] = float(msg.range)
    result["min_range_m"] = float(msg.min_range)
    result["max_range_m"] = float(msg.max_range)

def handle_status(msg):
    global last_status_time
    result["status_received"] = True
    last_status_time = time.monotonic()
    try:
        result["status_sample"] = json.loads(msg.data)
    except json.JSONDecodeError:
        result["status_sample"] = {{"raw": msg.data}}

range_sub = node.create_subscription(Range, range_topic, handle_range, qos_profile_sensor_data)
status_sub = node.create_subscription(String, status_topic, handle_status, 10)
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
result["sample_window_sec"] = time.monotonic() - started
result["range_rate_hz"] = result["range_count"] / max(result["sample_window_sec"], 0.001)
result["latest_input_age_sec"] = None if last_range_time is None else time.monotonic() - last_range_time
result["latest_status_age_sec"] = None if last_status_time is None else time.monotonic() - last_status_time
node.destroy_subscription(range_sub)
node.destroy_subscription(status_sub)
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
        envs={
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
        },
    )
    _write_text(config.artifact_dir / artifact_name, output)
    parsed: dict[str, Any] = {}
    if rc == 0:
        for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
            try:
                parsed = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    return {"rc": rc, "output": output, "result": parsed}


def _collect_json_status_sample(
    config: RunConfig,
    *,
    image: str,
    topic: str,
    artifact_name: str,
    node_name: str,
) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    script = f"""
import json
import threading
import time

import rclpy
from std_msgs.msg import String

topic = {topic!r}
result = {{"topic": topic, "received": False, "sample": None}}
rclpy.init()
node = rclpy.create_node({node_name!r})
event = threading.Event()

def callback(msg):
    result["received"] = True
    try:
        result["sample"] = json.loads(msg.data)
    except json.JSONDecodeError:
        result["sample"] = {{"raw": msg.data}}
    event.set()

subscription = node.create_subscription(String, topic, callback, 10)
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline and not event.is_set():
    rclpy.spin_once(node, timeout_sec=0.1)
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
        envs={
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
        },
    )
    _write_text(config.artifact_dir / artifact_name, output)
    parsed: dict[str, Any] = {}
    if rc == 0:
        for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
            try:
                parsed = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    return {"rc": rc, "output": output, "result": parsed}


def _collect_imu_probe(config: RunConfig, *, image: str) -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    baseline = config.orchestration.official_baseline
    script = f"""
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

topic = {p2.imu_output_topic!r}
result = {{
    "topic": topic,
    "source_route": {p2.imu_source_route!r},
    "source_topic": {p2.imu_source_topic!r},
    "received": False,
    "message_count": 0,
    "synthetic_fallback_enabled": {p2.synthetic_fallback_enabled!r},
}}
rclpy.init()
node = rclpy.create_node("navlab_p2_imu_probe")
started = time.monotonic()
last_msg_time = None

def callback(msg):
    global last_msg_time
    result["received"] = True
    result["message_count"] += 1
    result["frame_id"] = msg.header.frame_id
    result["orientation_w"] = float(msg.orientation.w)
    result["angular_velocity_norm"] = float(
        (msg.angular_velocity.x ** 2 + msg.angular_velocity.y ** 2 + msg.angular_velocity.z ** 2) ** 0.5
    )
    result["linear_acceleration_norm"] = float(
        (msg.linear_acceleration.x ** 2 + msg.linear_acceleration.y ** 2 + msg.linear_acceleration.z ** 2) ** 0.5
    )
    last_msg_time = time.monotonic()

subscription = node.create_subscription(Imu, topic, callback, qos_profile_sensor_data)
deadline = time.monotonic() + 8.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
result["sample_window_sec"] = time.monotonic() - started
result["rate_hz"] = result["message_count"] / max(result["sample_window_sec"], 0.001)
result["latest_age_sec"] = None if last_msg_time is None else time.monotonic() - last_msg_time
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
        envs={
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
        },
    )
    _write_text(config.artifact_dir / "imu_probe.txt", output)
    parsed: dict[str, Any] = {}
    if rc == 0:
        for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
            try:
                parsed = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    return {"rc": rc, "output": output, "result": parsed}


def _collect_fcu_rangefinder_probe(config: RunConfig) -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    script = f"""
import json
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

endpoint = {p2.rangefinder_fcu_probe_endpoint!r}
result = {{
    "endpoint": endpoint,
    "connected": False,
    "heartbeat": False,
    "distance_sensor_count": 0,
    "latest_distance_m": None,
    "evidence": "",
}}
try:
    master = mavutil.mavlink_connection(endpoint, source_system=192, source_component=191, dialect="ardupilotmega")
    heartbeat = master.wait_heartbeat(timeout=8)
    result["connected"] = True
    result["heartbeat"] = heartbeat is not None
    target_system = master.target_system or 1
    target_component = master.target_component or 1
    master.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        100000,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=0.5)
        if msg is None:
            continue
        result["distance_sensor_count"] += 1
        result["latest_distance_m"] = float(msg.current_distance) * 0.01
        result["latest_orientation"] = int(msg.orientation)
        result["latest_sensor_id"] = int(msg.id)
    if result["distance_sensor_count"] > 0:
        result["evidence"] = "received DISTANCE_SENSOR from FCU telemetry"
except Exception as exc:
    result["error"] = str(exc)
print(json.dumps(result, sort_keys=True))
"""
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.gazebo_sensor_image,
        shell_command=f"/opt/gazebo-sensor-venv/bin/python - <<'PY'\n{script}\nPY",
        name=None,
        network="host",
        envs={
            "ROS_DOMAIN_ID": config.orchestration.official_baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": config.orchestration.official_baseline.rmw_implementation,
        },
    )
    _write_text(config.artifact_dir / "rangefinder_fcu_probe.txt", output)
    parsed: dict[str, Any] = {}
    if rc == 0:
        for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
            try:
                parsed = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    return {"rc": rc, "output": output, "result": parsed}


def _write_foxglove_notes(config: RunConfig) -> None:
    p2 = config.orchestration.rangefinder_imu
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P2 rangefinder + IMU replay notes",
                "",
                "P2 keeps the official ArduPilot iris_maze world/model control path and adds only a down rangefinder sensor overlay.",
                "",
                "- Fixed frame: `map` when Cartographer is present; otherwise inspect `odom`/`base_link`.",
                f"- X2 scan: `{p2.x2_scan_topic}` from the vendor driver; input Gazebo lidar is `{p2.x2_scan_input_topic}`.",
                f"- Down rangefinder ideal scan: `{p2.rangefinder_scan_ideal_topic}`.",
                f"- Down rangefinder ROS Range: `{p2.rangefinder_range_topic}`.",
                f"- Down rangefinder status: `{p2.rangefinder_status_topic}`.",
                f"- IMU topic: `{p2.imu_output_topic}` with frame `{p2.imu_frame_id}`.",
                "- Altitude control and hover are intentionally not evaluated in P2.",
                "- If the summary is blocked only on FCU receive evidence, sensor-side Gazebo and MAVLink sending may still be healthy.",
            ]
        )
        + "\n",
    )


def _build_p2_doctor_summary(config: RunConfig) -> dict[str, Any]:
    p2 = config.orchestration.rangefinder_imu
    blockers: list[str] = []
    baseline_doctor = _build_doctor_summary(config)
    if not baseline_doctor.get("ok"):
        blockers.extend(str(item) for item in baseline_doctor.get("blockers", []))
    try:
        _docker_cat(config, image=config.orchestration.official_baseline.runtime_image, path=OFFICIAL_IRIS_WITH_LIDAR_MODEL)
    except RuntimeError as exc:
        blockers.append(str(exc))
    try:
        _docker_cat(config, image=config.orchestration.official_baseline.runtime_image, path=OFFICIAL_GAZEBO_IRIS_PARAMS)
    except RuntimeError as exc:
        blockers.append(str(exc))
    if p2.altitude_control_claim != "not_evaluated":
        blockers.append("P2 must not claim altitude control")
    if p2.hover_claim != "not_evaluated":
        blockers.append("P2 must not claim hover completion")
    if p2.synthetic_fallback_enabled:
        blockers.append("P2 IMU synthetic fallback must be disabled")
    summary = {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "p2_rangefinder_imu_doctor": {
            "official_model_path": OFFICIAL_IRIS_WITH_LIDAR_MODEL,
            "official_param_path": OFFICIAL_GAZEBO_IRIS_PARAMS,
            "world_source": p2.world_source,
            "vehicle_model_source": p2.vehicle_model_source,
            "model_overlay_source": p2.model_overlay_source,
            "rangefinder_endpoint": p2.rangefinder_endpoint,
            "rangefinder_orientation": p2.rangefinder_mavlink_orientation,
            "imu_source_route": p2.imu_source_route,
            "imu_output_topic": p2.imu_output_topic,
            "altitude_control_claim": p2.altitude_control_claim,
            "hover_claim": p2.hover_claim,
            "direct_set_pose": False,
        },
        "official_baseline_doctor": baseline_doctor,
    }
    return summary


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class RangefinderImuDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "rangefinder-imu-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check P2 official maze rangefinder/IMU mechanism prerequisites."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(f"artifacts/ros/navlab_rangefinder_imu_doctor/{config.run_id}")
        artifact_dir.mkdir(parents=True, exist_ok=True)
        console.print("[bold cyan]Checking P2 rangefinder/IMU prerequisites[/bold cyan]")
        summary = _build_p2_doctor_summary(config)
        _write_json(artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P2 rangefinder/IMU doctor rc={0 if summary['ok'] else 20}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 20


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class RangefinderImuAcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "rangefinder-imu-acceptance"
    TASK_DESCRIPTION: ClassVar[str] = "Run P2 official maze rangefinder/IMU mechanism acceptance."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        duration_sec: float = 60.0,
        console: Console | None = None,
    ) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
        config.artifact_dir.mkdir(parents=True, exist_ok=True)
        host._render_run_config(console, config)
        baseline = config.orchestration.official_baseline
        p2 = config.orchestration.rangefinder_imu
        bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p2.yaml"
        model_overlay = config.artifact_dir / "iris_with_lidar_p2_rangefinder.sdf"
        param_overlay = config.artifact_dir / "gazebo-iris-p2-rangefinder.parm"
        sensor_config = config.artifact_dir / "p2_gazebo_sensor_runtime.toml"
        vendor_profile = config.artifact_dir / "x2_vendor_driver_p2.yaml"
        _write_p1_bridge_override(bridge_override)
        _write_p1_vendor_profile(vendor_profile, virtual_serial_link=p2.x2_virtual_serial_link)

        summary: dict[str, Any] | None = None
        try:
            model_overlay_summary = _write_p2_model_overlay(config, model_overlay)
            param_overlay_summary = _write_p2_param_overlay(config, param_overlay)
            sensor_config_summary = _write_p2_sensor_config(config, sensor_config, vendor_profile=vendor_profile)

            console.print("[bold cyan]Starting official iris_maze with P2 rangefinder/IMU probes[/bold cyan]")
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
            _start_p2_cartographer_container(config)
            time.sleep(min(max(duration_sec - 18.0, 5.0), 15.0))

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
                    p2.x2_scan_input_topic,
                    "/navlab/x2/vendor_scan",
                    p2.x2_scan_topic,
                    p2.x2_status_topic,
                    p2.rangefinder_scan_ideal_topic,
                    p2.rangefinder_range_topic,
                    p2.rangefinder_status_topic,
                    p2.imu_output_topic,
                    "/map",
                ),
            )
            x2_status = _collect_json_status_sample(
                config,
                image=baseline.runtime_image,
                topic=p2.x2_status_topic,
                artifact_name="x2_status_probe.txt",
                node_name="navlab_p2_x2_status_probe",
            )
            scan_sample = _collect_laser_scan_sample(config, image=baseline.runtime_image, topic=p2.x2_scan_topic)
            scan_input_sample = _collect_laser_scan_sample(
                config,
                image=baseline.runtime_image,
                topic=p2.x2_scan_input_topic,
            )
            rangefinder_probe = _collect_rangefinder_probe(config, image=baseline.runtime_image)
            imu_probe = _collect_imu_probe(config, image=baseline.runtime_image)
            fcu_rangefinder_probe = _collect_fcu_rangefinder_probe(config)
            rosbag_profile = _record_rosbag(
                config,
                image=baseline.runtime_image,
                duration_sec=max(10.0, min(duration_sec / 3.0, 20.0)),
            )
            counts = _message_counts(config)
            scan_publishers = topic_info.get(p2.x2_scan_topic, {}).get("publisher_nodes", [])
            scan_subscribers = topic_info.get(p2.x2_scan_topic, {}).get("subscription_nodes", [])
            vendor_scan_publishers = topic_info.get("/navlab/x2/vendor_scan", {}).get("publisher_nodes", [])
            vendor_scan_subscribers = topic_info.get("/navlab/x2/vendor_scan", {}).get("subscription_nodes", [])
            rangefinder_publishers = topic_info.get(p2.rangefinder_range_topic, {}).get("publisher_nodes", [])
            rangefinder_status_publishers = topic_info.get(p2.rangefinder_status_topic, {}).get("publisher_nodes", [])
            imu_publishers = topic_info.get(p2.imu_output_topic, {}).get("publisher_nodes", [])
            x2_sample = x2_status.get("result", {}).get("sample") or {}
            range_result = rangefinder_probe.get("result", {})
            range_status = range_result.get("status_sample") or {}
            imu_result = imu_probe.get("result", {})
            fcu_result = fcu_rangefinder_probe.get("result", {})
            official_cartographer_config = _official_cartographer_config_summary(config, image=baseline.runtime_image)
            rangefinder_fcu_received = bool(fcu_result.get("distance_sensor_count", 0) > 0)
            rangefinder = {
                "source_owner": "gazebo_sensor",
                "scan_ideal_topic": p2.rangefinder_scan_ideal_topic,
                "range_topic": p2.rangefinder_range_topic,
                "status_topic": p2.rangefinder_status_topic,
                "frame_id": p2.rangefinder_frame_id,
                "mavlink_message": "DISTANCE_SENSOR",
                "mavlink_orientation": p2.rangefinder_mavlink_orientation,
                "endpoint": p2.rangefinder_endpoint,
                "fcu_probe_endpoint": p2.rangefinder_fcu_probe_endpoint,
                "input_count": range_status.get("input_count", range_result.get("range_count", 0)),
                "sent_count": range_status.get("sent_count", 0),
                "latest_distance_m": range_status.get("latest_distance_m", range_result.get("latest_distance_m")),
                "latest_input_age_sec": range_status.get("latest_input_age_sec", range_result.get("latest_input_age_sec")),
                "latest_sent_age_sec": range_status.get("latest_sent_age_sec"),
                "fcu_received": rangefinder_fcu_received,
                "fcu_receive_evidence": fcu_result.get("evidence", ""),
                "fcu_probe": fcu_result,
                "status_sample": range_status,
                "range_probe": range_result,
                "publishers": rangefinder_publishers,
                "status_publishers": rangefinder_status_publishers,
                "config": {
                    "sensor_id": p2.rangefinder_sensor_id,
                    "source_system": p2.rangefinder_source_system,
                    "source_component": p2.rangefinder_source_component,
                    "min_distance_m": p2.rangefinder_min_distance_m,
                    "max_distance_m": p2.rangefinder_max_distance_m,
                    "covariance_cm": p2.rangefinder_covariance_cm,
                },
                "model_overlay": model_overlay_summary,
                "param_overlay": param_overlay_summary,
            }
            imu = {
                "source_route": p2.imu_source_route,
                "source_topic": p2.imu_source_topic,
                "output_topic": p2.imu_output_topic,
                "status_topic": p2.imu_status_topic,
                "frame_id": imu_result.get("frame_id", p2.imu_frame_id),
                "expected_frame_id": p2.imu_frame_id,
                "message_count": imu_result.get("message_count", 0),
                "rate_hz": imu_result.get("rate_hz", 0.0),
                "latest_age_sec": imu_result.get("latest_age_sec"),
                "synthetic_fallback_enabled": p2.synthetic_fallback_enabled,
                "probe": imu_result,
                "publishers": imu_publishers,
                "min_rate_hz": p2.imu_min_rate_hz,
            }
            p2_rangefinder_imu = {
                **_official_baseline_common(config),
                **_cartographer_dependency_summary(config),
                **_official_ros_dependency_summary(config),
                **official_cartographer_config,
                "world_source": p2.world_source,
                "vehicle_model_source": p2.vehicle_model_source,
                "model_overlay_source": p2.model_overlay_source,
                "official_launch_command": baseline.gazebo_launch,
                "official_bridge_config_override": str(bridge_override),
                "x2_runtime_config": str(sensor_config),
                "x2_runtime_config_sha256": sensor_config_summary["sha256"],
                "x2_vendor_profile": str(vendor_profile),
                "x2_vendor_profile_sha256": _file_sha256(vendor_profile),
                "cartographer_launch": p2.cartographer_launch,
                "altitude_control_claim": p2.altitude_control_claim,
                "hover_claim": p2.hover_claim,
                "direct_set_pose": False,
                "set_pose_count": 0,
                "gazebo_lidar_topic": p2.gazebo_lidar_topic,
                "x2_scan_input_topic": p2.x2_scan_input_topic,
                "x2_vendor_scan_topic": "/navlab/x2/vendor_scan",
                "x2_scan_topic": p2.x2_scan_topic,
                "x2_status_topic": p2.x2_status_topic,
                "scan_publishers": scan_publishers,
                "scan_subscribers": scan_subscribers,
                "vendor_scan_publishers": vendor_scan_publishers,
                "vendor_scan_subscribers": vendor_scan_subscribers,
                "scan_sample": scan_sample.get("result", {}),
                "scan_input_sample": scan_input_sample.get("result", {}),
                "x2_status": x2_status.get("result", {}),
                "rangefinder": rangefinder,
                "imu": imu,
                "message_counts": counts,
                "official_dds_probe": probe.get("result", {}),
                "topic_info": topic_info,
                "official_container": host.OFFICIAL_BASELINE_CONTAINER,
                "gazebo_sensor_container": GAZEBO_SENSOR_CONTAINER,
                "cartographer_container": CARTOGRAPHER_CONTAINER,
            }
            blockers: list[str] = []
            doctor = _build_p2_doctor_summary(config)
            if not doctor.get("ok"):
                blockers.extend(str(item) for item in doctor.get("blockers", []))
            if p2.world_source != "official_iris_maze":
                blockers.append(f"world_source={p2.world_source!r} is not official_iris_maze")
            if p2.altitude_control_claim != "not_evaluated":
                blockers.append("P2 must not claim altitude control")
            if p2.hover_claim != "not_evaluated":
                blockers.append("P2 must not claim hover completion")
            if p2.synthetic_fallback_enabled:
                blockers.append("P2 IMU synthetic fallback must be disabled")
            if "ydlidar_ros2_driver_node" not in vendor_scan_publishers:
                blockers.append("/navlab/x2/vendor_scan is not published by ydlidar_ros2_driver_node")
            if "navlab_x2_scan_time_normalizer" not in scan_publishers:
                blockers.append("/scan is not published by navlab_x2_scan_time_normalizer")
            if any("ros_gz_bridge" in publisher for publisher in scan_publishers):
                blockers.append("/scan is still published by ros_gz_bridge; X2 route is polluted")
            if "cartographer_node" not in scan_subscribers:
                blockers.append("cartographer_node is not subscribed to /scan")
            if not probe.get("result", {}).get("time_received"):
                blockers.append("official DDS probe did not receive /ap/v1/time")
            if not x2_status.get("result", {}).get("received"):
                blockers.append("X2 status probe did not receive /sim/x2/status")
            if x2_sample.get("scan_source") != "gazebo_ideal":
                blockers.append("X2 emulator is not consuming Gazebo lidar input")
            scan_sample_result = scan_sample.get("result", {})
            if not scan_sample_result.get("received"):
                blockers.append("P2 did not receive a /scan LaserScan sample")
            if counts.get(p2.x2_scan_topic, 0) <= 0:
                blockers.append(f"{p2.x2_scan_topic} was not recorded")
            if counts.get(p2.rangefinder_scan_ideal_topic, 0) <= 0:
                blockers.append(f"{p2.rangefinder_scan_ideal_topic} was not recorded")
            if counts.get(p2.rangefinder_range_topic, 0) <= 0:
                blockers.append(f"{p2.rangefinder_range_topic} was not recorded")
            if counts.get(p2.rangefinder_status_topic, 0) <= 0:
                blockers.append(f"{p2.rangefinder_status_topic} was not recorded")
            if counts.get(p2.imu_output_topic, 0) <= 0:
                blockers.append(f"{p2.imu_output_topic} was not recorded")
            if not range_result.get("range_received"):
                blockers.append(f"P2 did not receive {p2.rangefinder_range_topic}")
            if range_result.get("frame_id") != p2.rangefinder_frame_id:
                blockers.append(
                    f"{p2.rangefinder_range_topic} frame_id={range_result.get('frame_id')!r} "
                    f"is not {p2.rangefinder_frame_id!r}"
                )
            latest_distance = rangefinder.get("latest_distance_m")
            if latest_distance is None or not (
                p2.rangefinder_min_distance_m <= float(latest_distance) <= p2.rangefinder_max_distance_m
            ):
                blockers.append("latest rangefinder distance is missing or outside configured min/max")
            if range_status.get("state") != "sending":
                blockers.append("rangefinder status is not sending")
            if int(range_status.get("sent_count", 0) or 0) <= 0:
                blockers.append("rangefinder MAVLink DISTANCE_SENSOR sent_count is zero")
            if not rangefinder_fcu_received:
                blockers.append("FCU telemetry did not provide DISTANCE_SENSOR receive evidence")
            if not imu_result.get("received"):
                blockers.append(f"P2 did not receive IMU topic {p2.imu_output_topic}")
            if imu_result.get("frame_id") != p2.imu_frame_id:
                blockers.append(f"IMU frame_id={imu_result.get('frame_id')!r} is not {p2.imu_frame_id!r}")
            if float(imu_result.get("rate_hz", 0.0) or 0.0) < p2.imu_min_rate_hz:
                blockers.append("IMU rate is below P2 minimum")
            if not rosbag_profile.get("ok"):
                blockers.append("P2 rosbag profile did not pass")
            summary = {
                "ok": not blockers,
                "blocked": bool(blockers),
                "blockers": blockers,
                "p2_rangefinder_imu": p2_rangefinder_imu,
                "rangefinder": rangefinder,
                "imu": imu,
                "ros_graph": graph,
                "official_dds_probe": probe,
                "rosbag_profile": rosbag_profile,
            }
            _write_json(config.artifact_dir / "summary.json", summary)
            _write_foxglove_notes(config)
        finally:
            host._capture_official_baseline_log(config=config)
            _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_tail.log")
            _capture_container_log(config, container=CARTOGRAPHER_CONTAINER, output_name="cartographer_tail.log")
            _remove_container(CARTOGRAPHER_CONTAINER)
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
                "blockers": ["P2 rangefinder/IMU acceptance did not produce a summary"],
            }
            _write_json(config.artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P2 rangefinder/IMU acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 30
