from __future__ import annotations

import json
import shlex
import textwrap
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import tomli_w
from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.config import RunConfig
from src.runtime import SERVICE_ROLE_OFFICIAL_BASELINE, SERVICE_ROLE_SDF_OVERLAY
from src.tasks.helpers.official_stack import (
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
from src.tasks.helpers.navlab_models import (
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
    host._assert_runtime_service_role(
        config,
        service_name="p2_model_overlay",
        service_role=SERVICE_ROLE_SDF_OVERLAY,
    )
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
    host._assert_runtime_service_role(
        config,
        service_name="p2_param_overlay",
        service_role=SERVICE_ROLE_SDF_OVERLAY,
    )
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
    host._assert_runtime_service_role(
        config,
        service_name="p2_cartographer",
        service_role=SERVICE_ROLE_OFFICIAL_BASELINE,
    )
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




