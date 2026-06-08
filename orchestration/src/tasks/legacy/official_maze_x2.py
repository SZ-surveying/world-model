from __future__ import annotations

import hashlib
import json
import shlex
import textwrap
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import tomli_w
from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.config import RunConfig
from src.runtime import SERVICE_ROLE_GAZEBO_SENSOR, SERVICE_ROLE_OFFICIAL_BASELINE
from src.tasks.legacy.official_baseline import (
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

GAZEBO_SENSOR_CONTAINER = "navlab-official-maze-x2-sensor"
CARTOGRAPHER_CONTAINER = "navlab-official-maze-x2-cartographer"
OFFICIAL_IRIS_3D_BRIDGE_CONFIG = (
    "/opt/navlab_official_ws/install/ardupilot_gz_bringup/share/"
    "ardupilot_gz_bringup/config/iris_3Dlidar_bridge.yaml"
)


def _remove_container(name: str) -> None:
    try:
        DockerClient().remove(name, force=True)
    except DockerException:
        pass


def _workspace_path(config: RunConfig, path: Path) -> str:
    try:
        return host._workspace_path(path)
    except Exception:
        return str(Path("/workspace") / path)


def _write_p1_bridge_override(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        textwrap.dedent(
            """\
            ---
            - ros_topic_name: "clock"
              gz_topic_name: "/clock"
              ros_type_name: "rosgraph_msgs/msg/Clock"
              gz_type_name: "gz.msgs.Clock"
              direction: GZ_TO_ROS
            - ros_topic_name: "joint_states"
              gz_topic_name: "/world/{{ world_name }}/model/{{ robot_name }}/joint_state"
              ros_type_name: "sensor_msgs/msg/JointState"
              gz_type_name: "gz.msgs.Model"
              direction: GZ_TO_ROS
            - ros_topic_name: "odometry"
              gz_topic_name: "/model/{{ robot_name }}/odometry"
              ros_type_name: "nav_msgs/msg/Odometry"
              gz_type_name: "gz.msgs.Odometry"
              direction: GZ_TO_ROS
            - ros_topic_name: "gz/tf"
              gz_topic_name: "/model/{{ robot_name }}/pose"
              ros_type_name: "tf2_msgs/msg/TFMessage"
              gz_type_name: "gz.msgs.Pose_V"
              direction: GZ_TO_ROS
            - ros_topic_name: "gz/tf_static"
              gz_topic_name: "/model/{{ robot_name }}/pose_static"
              ros_type_name: "tf2_msgs/msg/TFMessage"
              gz_type_name: "gz.msgs.Pose_V"
              direction: GZ_TO_ROS
            - ros_topic_name: "imu"
              gz_topic_name: "/world/{{ world_name }}/model/{{ robot_name }}/link/imu_link/sensor/imu_sensor/imu"
              ros_type_name: "sensor_msgs/msg/Imu"
              gz_type_name: "gz.msgs.IMU"
              direction: GZ_TO_ROS
            - ros_topic_name: "battery"
              gz_topic_name: "/model/{{ robot_name }}/battery/linear_battery/state"
              ros_type_name: "sensor_msgs/msg/BatteryState"
              gz_type_name: "gz.msgs.BatteryState"
              direction: GZ_TO_ROS
            - ros_topic_name: "cloud_in"
              gz_topic_name: "/lidar/points"
              ros_type_name: "sensor_msgs/msg/PointCloud2"
              gz_type_name: "gz.msgs.PointCloudPacked"
              direction: GZ_TO_ROS
            """
        ),
        encoding="utf-8",
    )


def _write_p1_vendor_profile(path: Path, *, virtual_serial_link: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        textwrap.dedent(
            f"""\
            ydlidar_ros2_driver_node:
              ros__parameters:
                use_sim_time: true
                port: {virtual_serial_link}
                frame_id: base_scan
                ignore_array: ""
                baudrate: 115200
                lidar_type: 1
                device_type: 0
                sample_rate: 3
                abnormal_check_count: 4
                fixed_resolution: true
                reversion: false
                inverted: false
                auto_reconnect: true
                isSingleChannel: true
                intensity: false
                support_motor_dtr: true
                angle_max: 180.0
                angle_min: -180.0
                range_max: 8.0
                range_min: 0.1
                frequency: 7.0
            """
        ),
        encoding="utf-8",
    )


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_p1_sensor_config(config: RunConfig, path: Path, *, vendor_profile: Path) -> None:
    maze_x2 = config.orchestration.official_maze_x2
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "gazebo_sensor": {
            "x2_protocol": {
                "enabled": True,
                "scan_source": "x2_virtual_serial",
                "profile": _workspace_path(config, vendor_profile),
                "virtual_serial_link": maze_x2.x2_virtual_serial_link,
                "scan_ideal_topic": maze_x2.x2_scan_input_topic,
                "vendor_scan_topic": "/navlab/x2/vendor_scan",
                "scan_topic": maze_x2.x2_scan_topic,
                "status_topic": maze_x2.x2_status_topic,
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
                "enabled": False,
            },
        }
    }
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))


def _start_gazebo_sensor_container(config: RunConfig, *, sensor_config: Path) -> None:
    host._assert_runtime_service_role(
        config,
        service_name="gazebo_sensor",
        service_role=SERVICE_ROLE_GAZEBO_SENSOR,
    )
    _remove_container(GAZEBO_SENSOR_CONTAINER)
    baseline = config.orchestration.official_baseline
    log_path = config.artifact_dir / "gazebo_sensor_runtime.log"
    command = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /opt/navlab_sensor_ws/install/setup.bash && "
        f"exec /opt/gazebo-sensor-venv/bin/python -m navlab.gazebo_sensor.cli --runtime "
        f"--log-file {shlex.quote(_workspace_path(config, log_path))}"
    )
    DockerClient().run(
        config.gazebo_sensor_image,
        ["bash", "-lc", command],
        detach=True,
        name=GAZEBO_SENSOR_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={
            "SESSION_ID": config.session_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
            "PYTHONPATH": "/workspace",
            "NAVLAB_CONFIG": _workspace_path(config, sensor_config),
        },
    )


def _start_cartographer_container(config: RunConfig) -> None:
    host._assert_runtime_service_role(
        config,
        service_name="official_maze_cartographer",
        service_role=SERVICE_ROLE_OFFICIAL_BASELINE,
    )
    _remove_container(CARTOGRAPHER_CONTAINER)
    baseline = config.orchestration.official_baseline
    launch_command = f"{config.orchestration.official_maze_x2.cartographer_launch} rviz:=false"
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


def _capture_container_log(config: RunConfig, *, container: str, output_name: str, tail: int = 2000) -> None:
    try:
        output = DockerClient().logs(container, tail=tail)
    except DockerException as exc:
        output = str(exc)
    _write_text(config.artifact_dir / output_name, str(output))


def _profile_topics(profile_path: Path) -> tuple[list[str], list[str], list[str]]:
    required: list[str] = []
    optional: list[str] = []
    if profile_path.is_file():
        for raw_line in profile_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(maxsplit=1)
            if len(parts) != 2:
                continue
            kind, topic = parts
            if kind == "required":
                required.append(topic)
            elif kind == "optional":
                optional.append(topic)
    return required, optional, [*required, *optional]


def _record_rosbag(config: RunConfig, *, image: str, duration_sec: float) -> dict[str, Any]:
    profile_path = Path(config.official_maze_x2_rosbag_profile)
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


def _collect_topic_info(
    config: RunConfig,
    *,
    image: str,
    topics: tuple[str, ...],
    transient_topics: tuple[str, ...] = (),
) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    result: dict[str, Any] = {}
    transient = set(transient_topics)
    for topic in topics:
        artifact = config.artifact_dir / f"topic_info_{topic.strip('/').replace('/', '_') or 'root'}.txt"
        if topic in transient:
            output = f"skipped transient topic info after run: {topic}\n"
            _write_text(artifact, output)
            result[topic] = {
                "rc": 0,
                "output": output,
                "publisher_nodes": [],
                "subscription_nodes": [],
                "skipped": True,
                "reason": "transient_topic_gone_after_run",
            }
            continue
        rc, output = host._docker_run_ros_shell_capture(
            config=config,
            image=image,
            shell_command=f"timeout --signal=INT 8s ros2 topic info -v {shlex.quote(topic)}",
            name=None,
            network="host",
            envs={
                "DDS_ENABLE": baseline.dds_enable,
                "DDS_DOMAIN_ID": baseline.dds_domain_id,
                "ROS_DOMAIN_ID": baseline.dds_domain_id,
                "RMW_IMPLEMENTATION": baseline.rmw_implementation,
            },
        )
        _write_text(artifact, output)
        result[topic] = {
            "rc": rc,
            "output": output,
            "publisher_nodes": _publisher_nodes_from_topic_info(output),
            "subscription_nodes": _subscription_nodes_from_topic_info(output),
        }
    return result


def _publisher_nodes_from_topic_info(output: str) -> list[str]:
    nodes: list[str] = []
    in_publishers = False
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("Publisher count:"):
            in_publishers = True
            continue
        if stripped.startswith("Subscription count:"):
            in_publishers = False
            continue
        if in_publishers and stripped.startswith("Node name:"):
            nodes.append(stripped.split(":", 1)[1].strip())
    return nodes


def _subscription_nodes_from_topic_info(output: str) -> list[str]:
    nodes: list[str] = []
    in_subscriptions = False
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("Subscription count:"):
            in_subscriptions = True
            continue
        if in_subscriptions and stripped.startswith("Node name:"):
            nodes.append(stripped.split(":", 1)[1].strip())
    return nodes


def _collect_x2_status(config: RunConfig, *, image: str) -> dict[str, Any]:
    topic = config.orchestration.official_maze_x2.x2_status_topic
    baseline = config.orchestration.official_baseline
    script = f"""
import json
import threading
import time

import rclpy
from std_msgs.msg import String

result = {{"topic": {topic!r}, "received": False, "sample": None}}
rclpy.init()
node = rclpy.create_node("navlab_p1_x2_status_probe")
event = threading.Event()

def callback(msg):
    result["received"] = True
    try:
        result["sample"] = json.loads(msg.data)
    except json.JSONDecodeError:
        result["sample"] = {{"raw": msg.data}}
    event.set()

subscription = node.create_subscription(String, {topic!r}, callback, 10)
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
    _write_text(config.artifact_dir / "x2_status_probe.txt", output)
    parsed: dict[str, Any] = {}
    if rc == 0:
        for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
            try:
                parsed = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    return {"rc": rc, "output": output, "result": parsed}


def _official_cartographer_config_summary(config: RunConfig, *, image: str) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    script = r"""
import hashlib
import json
import re
from pathlib import Path

path = Path(
    "/opt/navlab_official_ws/install/ardupilot_cartographer/share/"
    "ardupilot_cartographer/config/cartographer.lua"
)
if not path.is_file():
    print(json.dumps({"cartographer_config_present": False, "cartographer_config_path": str(path)}, sort_keys=True))
    raise SystemExit(0)
content = path.read_text(encoding="utf-8")

def lua_string(key):
    match = re.search(rf"\b{re.escape(key)}\s*=\s*\"([^\"]+)\"", content)
    return match.group(1) if match else None

def lua_bool(key):
    match = re.search(rf"\b{re.escape(key)}\s*=\s*(true|false)", content)
    return None if not match else match.group(1) == "true"

print(json.dumps({
    "cartographer_config_present": True,
    "cartographer_config_path": str(path),
    "cartographer_config_hash": hashlib.sha256(content.encode("utf-8")).hexdigest(),
    "cartographer_uses_odometry": lua_bool("use_odometry"),
    "tracking_frame": lua_string("tracking_frame"),
    "published_frame": lua_string("published_frame"),
    "odom_frame": lua_string("odom_frame"),
}, sort_keys=True))
"""
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=f"python3 - <<'PY'\n{script}\nPY",
        name=None,
        network=None,
        envs={
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
        },
    )
    parsed: dict[str, Any] = {}
    if rc == 0:
        for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
            try:
                parsed = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    if parsed:
        return parsed
    return {
        "cartographer_config_present": False,
        "cartographer_config_error": output.strip() or f"official cartographer config probe failed rc={rc}",
    }


def _collect_laser_scan_sample(config: RunConfig, *, image: str, topic: str) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    script = f"""
import json
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

result = {{"topic": {topic!r}, "received": False}}
rclpy.init()
node = rclpy.create_node("navlab_p1_laserscan_probe")
event = threading.Event()

def callback(msg):
    finite_ranges = [float(value) for value in msg.ranges if value == value]
    result.update({{
        "received": True,
        "frame_id": msg.header.frame_id,
        "angle_min": float(msg.angle_min),
        "angle_max": float(msg.angle_max),
        "angle_increment": float(msg.angle_increment),
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
        "range_count": len(msg.ranges),
        "finite_range_count": len(finite_ranges),
    }})
    event.set()

subscription = node.create_subscription(LaserScan, {topic!r}, callback, qos_profile_sensor_data)
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
    _write_text(config.artifact_dir / f"laserscan_sample_{topic.strip('/').replace('/', '_')}.txt", output)
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
    maze_x2 = config.orchestration.official_maze_x2
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P1 official maze + X2 replay notes",
                "",
                "P1 keeps the official ArduPilot iris_maze world/model and replaces only the lidar data path.",
                "",
                "- Fixed frame: `map` when Cartographer is present; otherwise inspect `odom`/`base_link`.",
                f"- Gazebo lidar input: `{maze_x2.gazebo_lidar_topic}` bridged to `{maze_x2.x2_scan_input_topic}`.",
                f"- Completion scan topic: `{maze_x2.x2_scan_topic}` from `navlab_x2_scan_time_normalizer`.",
                "- Raw vendor scan topic: `/navlab/x2/vendor_scan` from `ydlidar_ros2_driver_node`.",
                f"- X2 status topic: `{maze_x2.x2_status_topic}`.",
                "- Altitude control and hover are intentionally not evaluated in P1.",
                "- If `/scan` has a `ros_gz_bridge` publisher, the run is polluted and should fail.",
            ]
        )
        + "\n",
    )


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return _load_rosbag_metadata_counts(metadata)


