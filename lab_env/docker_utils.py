from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from python_on_whales import DockerClient, docker
from python_on_whales.exceptions import DockerException

from lab_env.config import (
    ComposeConfig,
    FastLioConfig,
    FoxgloveConfig,
    GazeboConfig,
    RosbagConfig,
    RouterConfig,
    all_services,
    load_compose_config,
    load_fast_lio_config,
    load_foxglove_config,
    load_gazebo_config,
    load_rosbag_config,
    load_router_config,
    load_runtime_config,
    services_for_profile,
)


@dataclass(slots=True)
class GazeboStatus:
    container_name: str
    pid: int | None
    world_path: str
    uptime: str
    cpu_percent: float
    memory_used: str
    memory_limit: str
    gz_topic_count: int
    gz_service_count: int


@dataclass(slots=True)
class FoxgloveStatus:
    container_name: str
    pid: int | None
    port: str
    uptime: str
    cpu_percent: float
    memory_used: str
    memory_limit: str


@dataclass(slots=True)
class RosbagStatus:
    container_name: str
    pid: int | None
    session_id: str
    topic_file: str
    output_dir: str
    uptime: str
    cpu_percent: float
    memory_used: str
    memory_limit: str
    topic_count: int


@dataclass(slots=True)
class FastLioStatus:
    container_name: str
    state: str
    pid: int | None
    exit_code: int | None
    config_path: str
    config_exists: bool | None
    rviz_enabled: str
    package_prefix: str
    uptime: str
    cpu_percent: float | None
    memory_used: str
    memory_limit: str
    ros_node_count: int | None
    ros_topic_count: int | None
    fastlio_node_present: bool | None
    input_points_present: bool | None
    input_imu_present: bool | None
    output_path_present: bool | None
    output_cloud_registered_present: bool | None
    input_points_stamp: str | None
    input_imu_stamp: str | None
    output_cloud_registered_rate_hz: float | None
    points_publisher_count: int | None
    points_subscriber_count: int | None
    points_type: str | None
    imu_publisher_count: int | None
    imu_subscriber_count: int | None
    imu_type: str | None
    path_publisher_count: int | None
    path_subscriber_count: int | None
    path_type: str | None
    cloud_registered_publisher_count: int | None
    cloud_registered_subscriber_count: int | None
    cloud_registered_type: str | None
    fastlio_subscriptions: list[str] | None
    fastlio_publishers: list[str] | None


@dataclass(slots=True)
class ComposeServiceState:
    service: str
    container_name: str | None
    state: str
    health: str | None
    exit_code: int | None
    pid: int | None
    uptime: str
    cpu_percent: float | None
    memory_used: str
    memory_limit: str


@dataclass(slots=True)
class SitlStatus:
    container_name: str
    pid: int | None
    session_id: str
    upstream_endpoint: str
    router_only: str
    uptime: str
    cpu_percent: float | None
    memory_used: str
    memory_limit: str


@dataclass(slots=True)
class MavlinkRouterStatus:
    container_name: str
    pid: int | None
    session_id: str
    listen: str
    tcp_port: str
    downstream_endpoints: str
    uptime: str
    cpu_percent: float | None
    memory_used: str
    memory_limit: str


@dataclass(slots=True)
class RosbagPlayStatus:
    container_name: str
    pid: int | None
    session_id: str
    play_bag_dir: str
    play_args: str
    uptime: str
    cpu_percent: float | None
    memory_used: str
    memory_limit: str


@dataclass(slots=True)
class ServiceSummary:
    service: str
    level: str
    detail: str


def _format_uptime(started_at: datetime | None) -> str:
    if started_at is None:
        return "<unknown>"
    delta = datetime.now(UTC) - started_at.astimezone(UTC)
    total_seconds = max(int(delta.total_seconds()), 0)
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours:
        return f"{hours}h {minutes}m {seconds}s"
    if minutes:
        return f"{minutes}m {seconds}s"
    return f"{seconds}s"


def _env_value(env_list: list[str] | None, key: str) -> str | None:
    if not env_list:
        return None
    prefix = f"{key}="
    for item in env_list:
        if item.startswith(prefix):
            return item[len(prefix) :]
    return None


def _compose_client(
    compose_config: ComposeConfig | None = None,
    *,
    profile: str | None = None,
) -> DockerClient:
    runtime = load_runtime_config()
    compose = compose_config or load_compose_config(runtime)
    env_file = runtime.lab_root / ".env"
    return DockerClient(
        compose_files=[compose.compose_file],
        compose_profiles=([profile] if profile else []),
        compose_project_name=compose.project_name.value,
        compose_project_directory=runtime.lab_root,
        compose_env_file=(env_file if env_file.is_file() else None),
    )


def compose_up(profile: str) -> None:
    compose = load_compose_config(load_runtime_config())
    _compose_client(compose, profile=profile).compose.up(detach=True)


def compose_down(*, remove_orphans: bool = False) -> None:
    compose = load_compose_config(load_runtime_config())
    _compose_client(compose).compose.down(remove_orphans=remove_orphans)


def default_profile_services() -> tuple[str, ...]:
    runtime = load_runtime_config()
    compose = load_compose_config(runtime)
    return services_for_selected_profile(compose.default_profile.value)


def services_for_selected_profile(profile: str) -> tuple[str, ...]:
    services = services_for_profile(profile)
    if not services:
        raise ValueError(f"unknown compose profile: {profile}")
    return services


def resolve_service_names(*, profile: str | None, service: str) -> tuple[str, ...]:
    if profile is None:
        if service != "all":
            raise ValueError("service selection requires --profile; without it doctor checks the default profile")
        return default_profile_services()

    profile_services = services_for_selected_profile(profile)
    if service == "all":
        return profile_services
    if service not in all_services():
        raise ValueError(f"unknown service: {service}")
    if service not in profile_services:
        raise ValueError(f"service {service!r} is not part of compose profile {profile!r}")
    return (service,)


def _service_container(service: str):
    compose = load_compose_config(load_runtime_config())
    containers = _compose_client(compose).compose.ps(services=[service], all=True)
    if not containers:
        return None
    if len(containers) > 1:
        raise ValueError(f"compose service has multiple containers: {service}")
    return containers[0]


def _stats_snapshot(container_name: str, *, running: bool) -> tuple[float | None, str, str]:
    if not running:
        return None, "<not running>", "<not running>"
    stats = docker.container.stats(container_name)[0]
    return (
        stats.cpu_percentage,
        stats.memory_used.human_readable(decimal=True),
        stats.memory_limit.human_readable(decimal=True),
    )


def get_compose_service_state(service: str) -> ComposeServiceState:
    container = _service_container(service)
    if container is None:
        return ComposeServiceState(
            service=service,
            container_name=None,
            state="missing",
            health=None,
            exit_code=None,
            pid=None,
            uptime="<not created>",
            cpu_percent=None,
            memory_used="<not created>",
            memory_limit="<not created>",
        )

    is_running = bool(container.state.running)
    cpu_percent, memory_used, memory_limit = _stats_snapshot(container.name, running=is_running)
    health = container.state.health.status if container.state.health else None
    return ComposeServiceState(
        service=service,
        container_name=container.name,
        state=container.state.status or "<unknown>",
        health=health,
        exit_code=container.state.exit_code,
        pid=container.state.pid,
        uptime=_format_uptime(container.state.started_at) if is_running else "<not running>",
        cpu_percent=cpu_percent,
        memory_used=memory_used,
        memory_limit=memory_limit,
    )


def summarize_service(service: str) -> ServiceSummary:
    status = get_compose_service_state(service)
    detail_parts = [
        f"container={status.container_name or '<not created>'}",
        f"state={status.state}",
    ]
    if status.health:
        detail_parts.append(f"health={status.health}")
    if status.exit_code is not None and status.state != "running":
        detail_parts.append(f"exit={status.exit_code}")

    if status.state == "running" and status.health in (None, "healthy"):
        level = "ok"
    elif status.state == "running" and status.health == "starting":
        level = "warn"
    else:
        level = "fail"
    return ServiceSummary(service=service, level=level, detail=" | ".join(detail_parts))


def _gazebo_count(container_name: str, command: str) -> int:
    result = docker.container.execute(
        container_name,
        [
            "bash",
            "-lc",
            (
                "set -o pipefail; "
                "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 || true; "
                f"{command} | sed '/^$/d' | wc -l"
            ),
        ],
    )
    return int(str(result).strip())


def _rosbag_topic_count(container_name: str, topic_file: str) -> int:
    result = docker.container.execute(
        container_name,
        [
            "bash",
            "-lc",
            (f"set -o pipefail; grep -vE '^\\s*(#|$)' {topic_file!r} 2>/dev/null | sed '/^$/d' | wc -l"),
        ],
    )
    return int(str(result).strip())


def _container_output(container_name: str, command: str) -> str:
    result = docker.container.execute(container_name, ["bash", "-lc", command])
    return str(result).strip()


def _resolve_container_name(name_or_service: str) -> str:
    try:
        docker.container.inspect(name_or_service)
        return name_or_service
    except DockerException:
        pass

    container = _service_container(name_or_service)
    if container is None:
        raise ValueError(f"container or compose service not found: {name_or_service}")
    return container.name


def _fast_lio_ros_prefix(container_name: str) -> str:
    command = (
        "source /usr/local/bin/source-ros.sh; "
        "source_ros_setup /opt/ros/jazzy/setup.bash; "
        "if [ -f /workspace/ros_ws/install/setup.bash ]; then "
        "source_ros_setup /workspace/ros_ws/install/setup.bash; "
        "fi; "
        "ros2 pkg prefix fast_lio 2>/dev/null || true"
    )
    return _container_output(container_name, command) or "<missing>"


def _path_exists(container_name: str, path: str) -> bool:
    output = _container_output(
        container_name,
        f"test -f {path!r} && echo yes || echo no",
    )
    return output == "yes"


def _ros_list_count(container_name: str, command: str) -> int:
    result = _container_output(
        container_name,
        (
            "source /usr/local/bin/source-ros.sh; "
            "source_ros_setup /opt/ros/jazzy/setup.bash; "
            "if [ -f /workspace/ros_ws/install/setup.bash ]; then "
            "source_ros_setup /workspace/ros_ws/install/setup.bash; "
            "fi; "
            f"{command} 2>/dev/null | sed '/^$/d' | wc -l"
        ),
    )
    return int(result)


def _ros_output_contains(container_name: str, command: str, pattern: str) -> bool:
    output = _container_output(
        container_name,
        (
            "source /usr/local/bin/source-ros.sh; "
            "source_ros_setup /opt/ros/jazzy/setup.bash; "
            "if [ -f /workspace/ros_ws/install/setup.bash ]; then "
            "source_ros_setup /workspace/ros_ws/install/setup.bash; "
            "fi; "
            f"{command} 2>/dev/null || true"
        ),
    )
    return pattern in output.splitlines()


def _ros_topic_present(container_name: str, topic_name: str) -> bool:
    return _ros_output_contains(container_name, "ros2 topic list", topic_name)


def _ros_python_probe(container_name: str, code: str) -> dict[str, object]:
    escaped = code.replace("\\", "\\\\").replace('"', '\\"')
    output = _container_output(
        container_name,
        (
            "source /usr/local/bin/source-ros.sh; "
            "source_ros_setup /opt/ros/jazzy/setup.bash; "
            "if [ -f /workspace/ros_ws/install/setup.bash ]; then "
            "source_ros_setup /workspace/ros_ws/install/setup.bash; "
            "fi; "
            f'python3 -c "{escaped}"'
        ),
    )
    if not output:
        return {}
    return json.loads(output)


def _ros_topic_header_stamp(
    container_name: str,
    topic_name: str,
    message_type: str,
) -> str | None:
    code = (
        "import json, time, rclpy\n"
        f"from sensor_msgs.msg import {message_type}\n"
        "from rclpy.node import Node\n"
        "rclpy.init(args=None)\n"
        "node = Node('status_probe_stamp')\n"
        "holder = {}\n"
        f"node.create_subscription({message_type}, {topic_name!r}, "
        "lambda msg: holder.setdefault('stamp', (msg.header.stamp.sec, msg.header.stamp.nanosec)), 10)\n"
        "end = time.time() + 3.0\n"
        "while time.time() < end and 'stamp' not in holder:\n"
        "    rclpy.spin_once(node, timeout_sec=0.2)\n"
        "print(json.dumps({'stamp': holder.get('stamp')}))\n"
        "node.destroy_node()\n"
        "rclpy.shutdown()\n"
    )
    probe = _ros_python_probe(container_name, code)
    stamp = probe.get("stamp")
    if not isinstance(stamp, list) or len(stamp) != 2:
        return None
    sec, nanosec = stamp
    return f"{sec}.{str(nanosec).zfill(9)}"


def _ros_topic_rate_hz(container_name: str, topic_name: str) -> float | None:
    code = (
        "import json, time, rclpy\n"
        "from sensor_msgs.msg import PointCloud2\n"
        "from rclpy.node import Node\n"
        "rclpy.init(args=None)\n"
        "node = Node('status_probe_rate')\n"
        "holder = {'count': 0, 'first': None, 'last': None}\n"
        "def cb(_msg):\n"
        "    now = time.time()\n"
        "    holder['count'] += 1\n"
        "    holder['first'] = holder['first'] or now\n"
        "    holder['last'] = now\n"
        f"node.create_subscription(PointCloud2, {topic_name!r}, cb, 10)\n"
        "end = time.time() + 3.0\n"
        "while time.time() < end:\n"
        "    rclpy.spin_once(node, timeout_sec=0.2)\n"
        "rate = None\n"
        "if holder['count'] >= 2 and holder['first'] is not None and holder['last'] is not None and holder['last'] > holder['first']:\n"
        "    rate = (holder['count'] - 1) / (holder['last'] - holder['first'])\n"
        "print(json.dumps({'count': holder['count'], 'rate_hz': rate}))\n"
        "node.destroy_node()\n"
        "rclpy.shutdown()\n"
    )
    probe = _ros_python_probe(container_name, code)
    rate_hz = probe.get("rate_hz")
    if isinstance(rate_hz, int | float):
        return float(rate_hz)
    return None


def _ros_graph_snapshot(container_name: str) -> dict[str, object]:
    code = (
        "import json, time, rclpy\n"
        "from rclpy.node import Node\n"
        "rclpy.init(args=None)\n"
        "node = Node('status_probe_graph')\n"
        "deadline = time.time() + 2.0\n"
        "topics = {}\n"
        "laser_present = False\n"
        "while time.time() < deadline:\n"
        "    topic_pairs = node.get_topic_names_and_types()\n"
        "    topics = {name: types for name, types in topic_pairs}\n"
        "    nodes = node.get_node_names_and_namespaces()\n"
        "    laser_present = any(name == 'laser_mapping' and namespace == '/' for name, namespace in nodes)\n"
        "    if topics and laser_present:\n"
        "        break\n"
        "    rclpy.spin_once(node, timeout_sec=0.2)\n"
        "def normalize(entries):\n"
        "    return [\n"
        "        {'topic': topic, 'types': list(types)}\n"
        "        for topic, types in sorted(entries, key=lambda item: item[0])\n"
        "    ]\n"
        "result = {\n"
        "    'laser_mapping_present': laser_present,\n"
        "    'topics': {},\n"
        "    'subscriptions': [],\n"
        "    'publishers': [],\n"
        "}\n"
        "for topic_name in ('/points', '/imu', '/path', '/cloud_registered'):\n"
        "    result['topics'][topic_name] = {\n"
        "        'types': list(topics.get(topic_name, [])),\n"
        "        'publisher_count': node.count_publishers(topic_name),\n"
        "        'subscriber_count': node.count_subscribers(topic_name),\n"
        "    }\n"
        "if laser_present:\n"
        "    result['subscriptions'] = normalize(\n"
        "        node.get_subscriber_names_and_types_by_node('laser_mapping', '/')\n"
        "    )\n"
        "    result['publishers'] = normalize(\n"
        "        node.get_publisher_names_and_types_by_node('laser_mapping', '/')\n"
        "    )\n"
        "print(json.dumps(result))\n"
        "node.destroy_node()\n"
        "rclpy.shutdown()\n"
    )
    return _ros_python_probe(container_name, code)


def _first_topic_type(topic_info: dict[str, object] | None) -> str | None:
    if not isinstance(topic_info, dict):
        return None
    types = topic_info.get("types")
    if not isinstance(types, list) or not types:
        return None
    first = types[0]
    return first if isinstance(first, str) else None


def _topic_count(topic_info: dict[str, object] | None, key: str) -> int | None:
    if not isinstance(topic_info, dict):
        return None
    value = topic_info.get(key)
    return value if isinstance(value, int) else None


def _format_topic_bindings(entries: object) -> list[str] | None:
    if not isinstance(entries, list):
        return None

    formatted: list[str] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        topic = entry.get("topic")
        types = entry.get("types")
        if not isinstance(topic, str):
            continue
        if not isinstance(types, list):
            types = []
        type_label = ", ".join(item for item in types if isinstance(item, str))
        formatted.append(f"{topic} ({type_label or '<unknown>'})")
    return formatted


def get_gazebo_status(gazebo: GazeboConfig) -> GazeboStatus:
    container_name = _resolve_container_name(gazebo.container_name.value)
    container = docker.container.inspect(container_name)
    stats = docker.container.stats(container_name)[0]

    world_path = _env_value(container.config.env, "WORLD") or gazebo.world.value

    return GazeboStatus(
        container_name=container_name,
        pid=container.state.pid,
        world_path=world_path,
        uptime=_format_uptime(container.state.started_at),
        cpu_percent=stats.cpu_percentage,
        memory_used=stats.memory_used.human_readable(decimal=True),
        memory_limit=stats.memory_limit.human_readable(decimal=True),
        gz_topic_count=_gazebo_count(container_name, "gz topic -l"),
        gz_service_count=_gazebo_count(container_name, "gz service -l"),
    )


def get_foxglove_status(foxglove: FoxgloveConfig) -> FoxgloveStatus:
    container_name = _resolve_container_name(foxglove.container_name.value)
    container = docker.container.inspect(container_name)
    stats = docker.container.stats(container_name)[0]

    port = _env_value(container.config.env, "FOXGLOVE_PORT") or foxglove.port.value

    return FoxgloveStatus(
        container_name=container_name,
        pid=container.state.pid,
        port=port,
        uptime=_format_uptime(container.state.started_at),
        cpu_percent=stats.cpu_percentage,
        memory_used=stats.memory_used.human_readable(decimal=True),
        memory_limit=stats.memory_limit.human_readable(decimal=True),
    )


def get_rosbag_status(rosbag: RosbagConfig) -> RosbagStatus:
    container_name = _resolve_container_name(rosbag.container_name.value)
    container = docker.container.inspect(container_name)
    stats = docker.container.stats(container_name)[0]

    session_id = _env_value(container.config.env, "SESSION_ID") or "manual"
    topic_file = _env_value(container.config.env, "TOPIC_FILE") or rosbag.topic_file.value
    output_dir = _env_value(container.config.env, "OUTPUT_DIR") or f"/artifacts/ros/{session_id}/rosbag"

    return RosbagStatus(
        container_name=container_name,
        pid=container.state.pid,
        session_id=session_id,
        topic_file=topic_file,
        output_dir=output_dir,
        uptime=_format_uptime(container.state.started_at),
        cpu_percent=stats.cpu_percentage,
        memory_used=stats.memory_used.human_readable(decimal=True),
        memory_limit=stats.memory_limit.human_readable(decimal=True),
        topic_count=_rosbag_topic_count(container_name, topic_file),
    )


def get_rosbag_play_status() -> RosbagPlayStatus:
    container_name = _resolve_container_name("rosbag-play")
    container = docker.container.inspect(container_name)
    is_running = bool(container.state.running)
    cpu_percent, memory_used, memory_limit = _stats_snapshot(container_name, running=is_running)

    return RosbagPlayStatus(
        container_name=container_name,
        pid=container.state.pid,
        session_id=_env_value(container.config.env, "SESSION_ID") or "manual",
        play_bag_dir=_env_value(container.config.env, "PLAY_BAG_DIR") or "/artifacts/ros/manual/rosbag",
        play_args=_env_value(container.config.env, "PLAY_ARGS") or "--loop",
        uptime=_format_uptime(container.state.started_at) if is_running else "<not running>",
        cpu_percent=cpu_percent,
        memory_used=memory_used,
        memory_limit=memory_limit,
    )


def get_sitl_status() -> SitlStatus:
    container_name = _resolve_container_name("sitl")
    container = docker.container.inspect(container_name)
    is_running = bool(container.state.running)
    cpu_percent, memory_used, memory_limit = _stats_snapshot(container_name, running=is_running)

    return SitlStatus(
        container_name=container_name,
        pid=container.state.pid,
        session_id=_env_value(container.config.env, "SESSION_ID") or "manual",
        upstream_endpoint=_env_value(container.config.env, "SITL_UPSTREAM_ENDPOINT") or "mavlink-router:14550",
        router_only=_env_value(container.config.env, "SITL_ROUTER_ONLY") or "false",
        uptime=_format_uptime(container.state.started_at) if is_running else "<not running>",
        cpu_percent=cpu_percent,
        memory_used=memory_used,
        memory_limit=memory_limit,
    )


def get_mavlink_router_status(router: RouterConfig) -> MavlinkRouterStatus:
    container_name = _resolve_container_name("mavlink-router")
    container = docker.container.inspect(container_name)
    is_running = bool(container.state.running)
    cpu_percent, memory_used, memory_limit = _stats_snapshot(container_name, running=is_running)

    downstream_endpoints = _env_value(container.config.env, "ROUTER_DOWNSTREAM_ENDPOINTS") or ", ".join(
        endpoint.endpoint for endpoint in router.endpoints
    )

    return MavlinkRouterStatus(
        container_name=container_name,
        pid=container.state.pid,
        session_id=_env_value(container.config.env, "SESSION_ID") or "manual",
        listen=_env_value(container.config.env, "ROUTER_LISTEN") or router.listen.value,
        tcp_port=_env_value(container.config.env, "ROUTER_TCP_PORT") or router.tcp_port.value,
        downstream_endpoints=downstream_endpoints or "<none>",
        uptime=_format_uptime(container.state.started_at) if is_running else "<not running>",
        cpu_percent=cpu_percent,
        memory_used=memory_used,
        memory_limit=memory_limit,
    )


def get_fast_lio_status(fast_lio: FastLioConfig) -> FastLioStatus:
    container_name = _resolve_container_name(fast_lio.container_name.value)
    container = docker.container.inspect(container_name)
    is_running = bool(container.state.running)
    stats = docker.container.stats(container_name)[0] if is_running else None

    config_path = _env_value(container.config.env, "FAST_LIO_CONFIG") or fast_lio.config_path.value
    rviz_enabled = _env_value(container.config.env, "FAST_LIO_RVIZ") or "false"
    config_exists: bool | None = None
    package_prefix = "<container not running>"
    ros_node_count: int | None = None
    ros_topic_count: int | None = None
    fastlio_node_present: bool | None = None
    input_points_present: bool | None = None
    input_imu_present: bool | None = None
    output_path_present: bool | None = None
    output_cloud_registered_present: bool | None = None
    input_points_stamp: str | None = None
    input_imu_stamp: str | None = None
    output_cloud_registered_rate_hz: float | None = None
    points_publisher_count: int | None = None
    points_subscriber_count: int | None = None
    points_type: str | None = None
    imu_publisher_count: int | None = None
    imu_subscriber_count: int | None = None
    imu_type: str | None = None
    path_publisher_count: int | None = None
    path_subscriber_count: int | None = None
    path_type: str | None = None
    cloud_registered_publisher_count: int | None = None
    cloud_registered_subscriber_count: int | None = None
    cloud_registered_type: str | None = None
    fastlio_subscriptions: list[str] | None = None
    fastlio_publishers: list[str] | None = None

    if is_running:
        try:
            config_exists = _path_exists(container_name, config_path)
            package_prefix = _fast_lio_ros_prefix(container_name)
            ros_node_count = _ros_list_count(container_name, "ros2 node list")
            ros_topic_count = _ros_list_count(container_name, "ros2 topic list")
            fastlio_node_present = _ros_output_contains(
                container_name,
                "ros2 node list",
                "/laser_mapping",
            )
            input_points_present = _ros_topic_present(container_name, "/points")
            input_imu_present = _ros_topic_present(container_name, "/imu")
            output_path_present = _ros_topic_present(container_name, "/path")
            output_cloud_registered_present = _ros_topic_present(
                container_name,
                "/cloud_registered",
            )
            graph_snapshot = _ros_graph_snapshot(container_name)
            topics = graph_snapshot.get("topics")
            topics_map = topics if isinstance(topics, dict) else {}
            points_info = topics_map.get("/points") if isinstance(topics_map.get("/points"), dict) else None
            imu_info = topics_map.get("/imu") if isinstance(topics_map.get("/imu"), dict) else None
            path_info = topics_map.get("/path") if isinstance(topics_map.get("/path"), dict) else None
            cloud_registered_info = (
                topics_map.get("/cloud_registered") if isinstance(topics_map.get("/cloud_registered"), dict) else None
            )
            points_publisher_count = _topic_count(points_info, "publisher_count")
            points_subscriber_count = _topic_count(points_info, "subscriber_count")
            points_type = _first_topic_type(points_info)
            imu_publisher_count = _topic_count(imu_info, "publisher_count")
            imu_subscriber_count = _topic_count(imu_info, "subscriber_count")
            imu_type = _first_topic_type(imu_info)
            path_publisher_count = _topic_count(path_info, "publisher_count")
            path_subscriber_count = _topic_count(path_info, "subscriber_count")
            path_type = _first_topic_type(path_info)
            cloud_registered_publisher_count = _topic_count(
                cloud_registered_info,
                "publisher_count",
            )
            cloud_registered_subscriber_count = _topic_count(
                cloud_registered_info,
                "subscriber_count",
            )
            cloud_registered_type = _first_topic_type(cloud_registered_info)
            fastlio_subscriptions = _format_topic_bindings(graph_snapshot.get("subscriptions"))
            fastlio_publishers = _format_topic_bindings(graph_snapshot.get("publishers"))
            input_points_stamp = _ros_topic_header_stamp(
                container_name,
                "/points",
                "PointCloud2",
            )
            input_imu_stamp = _ros_topic_header_stamp(
                container_name,
                "/imu",
                "Imu",
            )
            output_cloud_registered_rate_hz = _ros_topic_rate_hz(
                container_name,
                "/cloud_registered",
            )
        except DockerException:
            config_exists = None
            package_prefix = "<status unavailable>"
            ros_node_count = None
            ros_topic_count = None
            fastlio_node_present = None
            input_points_present = None
            input_imu_present = None
            output_path_present = None
            output_cloud_registered_present = None
            input_points_stamp = None
            input_imu_stamp = None
            output_cloud_registered_rate_hz = None
            points_publisher_count = None
            points_subscriber_count = None
            points_type = None
            imu_publisher_count = None
            imu_subscriber_count = None
            imu_type = None
            path_publisher_count = None
            path_subscriber_count = None
            path_type = None
            cloud_registered_publisher_count = None
            cloud_registered_subscriber_count = None
            cloud_registered_type = None
            fastlio_subscriptions = None
            fastlio_publishers = None

    return FastLioStatus(
        container_name=container_name,
        state=container.state.status or "<unknown>",
        pid=container.state.pid,
        exit_code=container.state.exit_code,
        config_path=config_path,
        config_exists=config_exists,
        rviz_enabled=rviz_enabled,
        package_prefix=package_prefix,
        uptime=(_format_uptime(container.state.started_at) if is_running else "<not running>"),
        cpu_percent=stats.cpu_percentage if stats else None,
        memory_used=(stats.memory_used.human_readable(decimal=True) if stats else "<not running>"),
        memory_limit=(stats.memory_limit.human_readable(decimal=True) if stats else "<not running>"),
        ros_node_count=ros_node_count,
        ros_topic_count=ros_topic_count,
        fastlio_node_present=fastlio_node_present,
        input_points_present=input_points_present,
        input_imu_present=input_imu_present,
        output_path_present=output_path_present,
        output_cloud_registered_present=output_cloud_registered_present,
        input_points_stamp=input_points_stamp,
        input_imu_stamp=input_imu_stamp,
        output_cloud_registered_rate_hz=output_cloud_registered_rate_hz,
        points_publisher_count=points_publisher_count,
        points_subscriber_count=points_subscriber_count,
        points_type=points_type,
        imu_publisher_count=imu_publisher_count,
        imu_subscriber_count=imu_subscriber_count,
        imu_type=imu_type,
        path_publisher_count=path_publisher_count,
        path_subscriber_count=path_subscriber_count,
        path_type=path_type,
        cloud_registered_publisher_count=cloud_registered_publisher_count,
        cloud_registered_subscriber_count=cloud_registered_subscriber_count,
        cloud_registered_type=cloud_registered_type,
        fastlio_subscriptions=fastlio_subscriptions,
        fastlio_publishers=fastlio_publishers,
    )
