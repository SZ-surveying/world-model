from __future__ import annotations

import hashlib
import json
import os
import shlex
import textwrap
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
from src.tasks.registry import TaskRegistry

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


def _collect_topic_info(config: RunConfig, *, image: str, topics: tuple[str, ...]) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    result: dict[str, Any] = {}
    for topic in topics:
        rc, output = host._docker_run_ros_shell_capture(
            config=config,
            image=image,
            shell_command=f"ros2 topic info -v {shlex.quote(topic)}",
            name=None,
            network="host",
            envs={
                "DDS_ENABLE": baseline.dds_enable,
                "DDS_DOMAIN_ID": baseline.dds_domain_id,
                "ROS_DOMAIN_ID": baseline.dds_domain_id,
                "RMW_IMPLEMENTATION": baseline.rmw_implementation,
            },
        )
        _write_text(config.artifact_dir / f"topic_info_{topic.strip('/').replace('/', '_') or 'root'}.txt", output)
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


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class OfficialMazeX2AcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "official-maze-x2-acceptance"
    TASK_DESCRIPTION: ClassVar[str] = "Run official iris_maze with NavLab X2 virtual serial lidar acceptance."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        duration_sec: float = 45.0,
        console: Console | None = None,
    ) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
        config.artifact_dir.mkdir(parents=True, exist_ok=True)
        host._render_run_config(console, config)
        baseline = config.orchestration.official_baseline
        maze_x2 = config.orchestration.official_maze_x2
        bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p1.yaml"
        sensor_config = config.artifact_dir / "p1_gazebo_sensor_runtime.toml"
        vendor_profile = config.artifact_dir / "x2_vendor_driver_p1.yaml"
        _write_p1_bridge_override(bridge_override)
        _write_p1_vendor_profile(vendor_profile, virtual_serial_link=maze_x2.x2_virtual_serial_link)
        _write_p1_sensor_config(config, sensor_config, vendor_profile=vendor_profile)

        summary: dict[str, Any] | None = None
        try:
            console.print("[bold cyan]Starting official iris_maze with NavLab X2 lidar route[/bold cyan]")
            try:
                host._compose_stop(config)
            except DockerException:
                pass
            host._start_official_baseline_container(
                config,
                volume_overrides=[(bridge_override.resolve(), OFFICIAL_IRIS_3D_BRIDGE_CONFIG)],
            )
            time.sleep(min(max(duration_sec, 1.0), 10.0))
            _start_gazebo_sensor_container(config, sensor_config=sensor_config)
            time.sleep(8.0)
            _start_cartographer_container(config)
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
                    maze_x2.x2_scan_input_topic,
                    "/navlab/x2/vendor_scan",
                    maze_x2.x2_scan_topic,
                    maze_x2.x2_status_topic,
                    "/map",
                ),
            )
            x2_status = _collect_x2_status(config, image=baseline.runtime_image)
            scan_sample = _collect_laser_scan_sample(
                config,
                image=baseline.runtime_image,
                topic=maze_x2.x2_scan_topic,
            )
            scan_input_sample = _collect_laser_scan_sample(
                config,
                image=baseline.runtime_image,
                topic=maze_x2.x2_scan_input_topic,
            )
            rosbag_profile = _record_rosbag(
                config,
                image=baseline.runtime_image,
                duration_sec=max(8.0, min(duration_sec / 3.0, 18.0)),
            )
            counts = _message_counts(config)
            scan_publishers = topic_info.get(maze_x2.x2_scan_topic, {}).get("publisher_nodes", [])
            scan_subscribers = topic_info.get(maze_x2.x2_scan_topic, {}).get("subscription_nodes", [])
            vendor_scan_publishers = topic_info.get("/navlab/x2/vendor_scan", {}).get("publisher_nodes", [])
            vendor_scan_subscribers = topic_info.get("/navlab/x2/vendor_scan", {}).get("subscription_nodes", [])
            x2_sample = x2_status.get("result", {}).get("sample") or {}
            official_cartographer_config = _official_cartographer_config_summary(
                config,
                image=baseline.runtime_image,
            )
            latest_age = x2_sample.get("latest_scan_ideal_age_sec")
            try:
                latest_age_ok = latest_age is not None and float(latest_age) <= 2.0
            except (TypeError, ValueError):
                latest_age_ok = False
            official_maze_x2 = {
                **_official_baseline_common(config),
                **_cartographer_dependency_summary(config),
                **_official_ros_dependency_summary(config),
                **official_cartographer_config,
                "world_source": maze_x2.world_source,
                "vehicle_model_source": maze_x2.vehicle_model_source,
                "official_launch_command": baseline.gazebo_launch,
                "official_bridge_config_override": str(bridge_override),
                "official_bridge_scan_removed": True,
                "lidar_route": "navlab_x2_vendor_driver",
                "gazebo_lidar_topic": maze_x2.gazebo_lidar_topic,
                "x2_scan_input_topic": maze_x2.x2_scan_input_topic,
                "x2_vendor_scan_topic": "/navlab/x2/vendor_scan",
                "x2_scan_topic": maze_x2.x2_scan_topic,
                "x2_status_topic": maze_x2.x2_status_topic,
                "x2_vendor_profile": str(vendor_profile),
                "x2_vendor_profile_sha256": _file_sha256(vendor_profile),
                "x2_runtime_config": str(sensor_config),
                "x2_runtime_config_sha256": _file_sha256(sensor_config),
                "cartographer_launch": maze_x2.cartographer_launch,
                "altitude_control_claim": maze_x2.altitude_control_claim,
                "hover_claim": maze_x2.hover_claim,
                "direct_set_pose": False,
                "set_pose_count": 0,
                "scan_publishers": scan_publishers,
                "scan_subscribers": scan_subscribers,
                "vendor_scan_publishers": vendor_scan_publishers,
                "vendor_scan_subscribers": vendor_scan_subscribers,
                "scan_sample": scan_sample.get("result", {}),
                "scan_input_sample": scan_input_sample.get("result", {}),
                "x2_status": x2_status.get("result", {}),
                "message_counts": counts,
                "official_dds_probe": probe.get("result", {}),
                "topic_info": topic_info,
                "official_container": host.OFFICIAL_BASELINE_CONTAINER,
                "gazebo_sensor_container": GAZEBO_SENSOR_CONTAINER,
                "cartographer_container": CARTOGRAPHER_CONTAINER,
            }
            cartographer = {
                "launch": maze_x2.cartographer_launch,
                "scan_topic": maze_x2.x2_scan_topic,
                "scan_subscribers": scan_subscribers,
                "map_count": counts.get("/map", 0),
                "submap_list_count": counts.get("/submap_list", 0),
                "trajectory_node_list_count": counts.get("/trajectory_node_list", 0),
                "config": official_cartographer_config,
            }
            blockers: list[str] = []
            doctor = _build_doctor_summary(config)
            if not doctor.get("ok"):
                blockers.extend(str(item) for item in doctor.get("blockers", []))
            if maze_x2.world_source != "official_iris_maze":
                blockers.append(f"world_source={maze_x2.world_source!r} is not official_iris_maze")
            if maze_x2.altitude_control_claim != "not_evaluated":
                blockers.append("P1 must not claim altitude control")
            if maze_x2.hover_claim != "not_evaluated":
                blockers.append("P1 must not claim hover completion")
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
            if not probe.get("result", {}).get("prearm_service_available"):
                blockers.append("official DDS probe did not find /ap/v1/prearm_check service")
            if not x2_status.get("result", {}).get("received"):
                blockers.append("X2 status probe did not receive /sim/x2/status")
            scan_sample_result = scan_sample.get("result", {})
            if not scan_sample_result.get("received"):
                blockers.append("P1 did not receive a /scan LaserScan sample")
            elif scan_sample_result.get("frame_id") not in {"base_scan", "laser_frame"}:
                blockers.append(f"/scan frame_id={scan_sample_result.get('frame_id')!r} is not an accepted P1 frame")
            if x2_sample.get("scan_source") != "gazebo_ideal":
                blockers.append("X2 emulator is not consuming Gazebo lidar input")
            if not latest_age_ok:
                blockers.append("X2 Gazebo lidar input is stale")
            if counts.get(maze_x2.x2_scan_input_topic, 0) <= 0:
                blockers.append(f"{maze_x2.x2_scan_input_topic} was not recorded")
            if counts.get(maze_x2.x2_scan_topic, 0) <= 0:
                blockers.append(f"{maze_x2.x2_scan_topic} was not recorded")
            if not rosbag_profile.get("ok"):
                blockers.append("P1 rosbag profile did not pass")
            summary = {
                "ok": not blockers,
                "blocked": bool(blockers),
                "blockers": blockers,
                "official_maze_x2": official_maze_x2,
                "p1_maze_x2": official_maze_x2,
                "cartographer": cartographer,
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
                "blockers": ["official maze X2 acceptance did not produce a summary"],
            }
            _write_json(config.artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]Official maze X2 acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 30
