from __future__ import annotations

import json
import shlex
import textwrap
from pathlib import Path
from typing import Any

from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException

from src import host
from src.configs.run_config import RunConfig
from src.runtime import SERVICE_ROLE_GAZEBO_SENSOR
from src.tasks.helpers.artifacts import write_text
from src.tasks.helpers.rosbag_profiles import load_rosbag_metadata_counts

GAZEBO_SENSOR_CONTAINER = "navlab-official-maze-x2-sensor"
CARTOGRAPHER_CONTAINER = "navlab-official-maze-x2-cartographer"
OFFICIAL_IRIS_3D_BRIDGE_CONFIG = (
    "/opt/navlab_official_ws/install/ardupilot_gz_bringup/share/"
    "ardupilot_gz_bringup/config/iris_3Dlidar_bridge.yaml"
)


def remove_container(name: str) -> None:
    try:
        DockerClient().remove(name, force=True)
    except DockerException:
        pass


def workspace_path(config: RunConfig, path: Path) -> str:
    try:
        return host.workspace_path(path)
    except Exception:
        return str(Path("/workspace") / path)


def write_p1_bridge_override(path: Path) -> None:
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


def write_p1_vendor_profile(path: Path, *, virtual_serial_link: str) -> None:
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


def start_gazebo_sensor_container(config: RunConfig, *, sensor_config: Path) -> None:
    host.assert_runtime_service_role(
        config,
        service_name="gazebo_sensor",
        service_role=SERVICE_ROLE_GAZEBO_SENSOR,
    )
    remove_container(GAZEBO_SENSOR_CONTAINER)
    baseline = config.orchestration.official_baseline
    log_path = config.artifact_dir / "gazebo_sensor_runtime.log"
    command = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /opt/navlab_sensor_ws/install/setup.bash && "
        f"exec /opt/gazebo-sensor-venv/bin/python -m navlab.sim.gazebo_sensor.cli --runtime "
        f"--log-file {shlex.quote(workspace_path(config, log_path))}"
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
            "NAVLAB_CONFIG": workspace_path(config, sensor_config),
        },
    )


def capture_container_log(config: RunConfig, *, container: str, output_name: str, tail: int = 2000) -> None:
    try:
        output = DockerClient().logs(container, tail=tail)
    except DockerException as exc:
        output = str(exc)
    write_text(config.artifact_dir / output_name, str(output))


def collect_topic_info(
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
            write_text(artifact, output)
            result[topic] = {
                "rc": 0,
                "output": output,
                "publisher_nodes": [],
                "subscription_nodes": [],
                "skipped": True,
                "reason": "transient_topic_gone_after_run",
            }
            continue
        rc, output = host.docker_run_ros_shell_capture(
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
        write_text(artifact, output)
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


def collect_x2_status(config: RunConfig, *, image: str) -> dict[str, Any]:
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
    rc, output = host.docker_run_ros_shell_capture(
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
    write_text(config.artifact_dir / "x2_status_probe.txt", output)
    parsed: dict[str, Any] = {}
    if rc == 0:
        for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
            try:
                parsed = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    return {"rc": rc, "output": output, "result": parsed}


def collect_laser_scan_sample(config: RunConfig, *, image: str, topic: str) -> dict[str, Any]:
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
    rc, output = host.docker_run_ros_shell_capture(
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
    write_text(config.artifact_dir / f"laserscan_sample_{topic.strip('/').replace('/', '_')}.txt", output)
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
    write_text(
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
    return load_rosbag_metadata_counts(metadata)
