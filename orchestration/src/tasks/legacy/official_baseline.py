from __future__ import annotations

import hashlib
import json
import os
import re
import shlex
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.config import RunConfig

CARTOGRAPHER_CONFIG_PATH = Path(
    "navlab/slam/ros/localization/navlab_cartographer_adapter/config/navlab_cartographer_2d.lua"
)
KNOWN_EXTERNAL_NAV_ROUTES = ("official_dds", "mavlink_fallback", "diagnostic_only", "unknown")
OFFICIAL_REFERENCES = {
    "ros2": "https://ardupilot.org/dev/docs/ros2.html",
    "sitl": "https://ardupilot.org/dev/docs/ros2-sitl.html",
    "gazebo": "https://ardupilot.org/dev/docs/ros2-gazebo.html",
    "cartographer": "https://ardupilot.org/dev/docs/ros2-cartographer-slam.html",
    "ardupilot_ros": "https://github.com/ArduPilot/ardupilot_ros",
    "ardupilot_gz": "https://github.com/ArduPilot/ardupilot_gz",
}


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _extract_lua_string(content: str, key: str) -> str | None:
    match = re.search(rf"\b{re.escape(key)}\s*=\s*\"([^\"]+)\"", content)
    return match.group(1) if match else None


def _extract_lua_bool(content: str, key: str) -> bool | None:
    match = re.search(rf"\b{re.escape(key)}\s*=\s*(true|false)", content)
    if not match:
        return None
    return match.group(1) == "true"


def _cartographer_config_summary() -> dict[str, Any]:
    path = CARTOGRAPHER_CONFIG_PATH
    if not path.is_file():
        return {
            "cartographer_config_present": False,
            "cartographer_config_path": str(path),
        }
    content = path.read_text(encoding="utf-8")
    return {
        "cartographer_config_present": True,
        "cartographer_config_path": str(path),
        "cartographer_config_hash": hashlib.sha256(content.encode("utf-8")).hexdigest(),
        "cartographer_uses_odometry": _extract_lua_bool(content, "use_odometry"),
        "tracking_frame": _extract_lua_string(content, "tracking_frame"),
        "published_frame": _extract_lua_string(content, "published_frame"),
        "odom_frame": _extract_lua_string(content, "odom_frame"),
    }


def _cartographer_dependency_summary(config: RunConfig) -> dict[str, Any]:
    prefix_rc, prefix_output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.slam_image,
        shell_command="timeout --signal=INT 8s ros2 pkg prefix cartographer_ros",
        name=None,
    )
    executables_rc, executables_output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.slam_image,
        shell_command="timeout --signal=INT 8s ros2 pkg executables cartographer_ros",
        name=None,
    )
    executables = [line.strip() for line in executables_output.splitlines() if line.strip()]
    return {
        "slam_image": config.slam_image,
        "cartographer_ros_present": prefix_rc == 0,
        "cartographer_ros_prefix": prefix_output.strip() if prefix_rc == 0 else "",
        "cartographer_ros_prefix_error": "" if prefix_rc == 0 else prefix_output.strip(),
        "cartographer_executables": executables if executables_rc == 0 else [],
        "cartographer_executables_error": "" if executables_rc == 0 else executables_output.strip(),
        "cartographer_node_present": any(line.endswith(" cartographer_node") for line in executables),
        "cartographer_occupancy_grid_node_present": any(
            line.endswith(" cartographer_occupancy_grid_node") for line in executables
        ),
    }


def _official_baseline_common(config: RunConfig) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    return {
        "official_references": OFFICIAL_REFERENCES,
        "dds_enable": baseline.dds_enable,
        "dds_domain_id": baseline.dds_domain_id,
        "official_ros_domain_id": baseline.dds_domain_id,
        "rmw_implementation": baseline.rmw_implementation,
        "navlab_ros_domain_id": config.ros_domain_id,
        "expected_ap_node": baseline.expected_ap_node,
        "required_ap_topics": list(baseline.required_ap_topics),
        "runtime_image": baseline.runtime_image,
        "required_ros_packages": list(baseline.required_ros_packages),
        "micro_ros_agent_binaries": list(baseline.micro_ros_agent_binaries),
        "sitl_launch": baseline.sitl_launch,
        "gazebo_launch": baseline.gazebo_launch,
        "cartographer_launch": baseline.cartographer_launch,
        "gazebo_bringup_mode": baseline.gazebo_bringup_mode,
        "external_nav_route": baseline.external_nav_route,
        "known_external_nav_routes": list(KNOWN_EXTERNAL_NAV_ROUTES),
        "gazebo_truth_route": "diagnostic_only",
        "direct_set_pose": False,
    }


def _route_is_official(route: str) -> bool:
    return route == "official_dds"


def _official_ros_dependency_summary(config: RunConfig) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    image_rc, _image_output = host._docker_run_ros_shell_capture(
        config=config,
        image=baseline.runtime_image,
        shell_command="true",
        name=None,
    )
    if image_rc != 0:
        return {
            "official_runtime_image": baseline.runtime_image,
            "official_runtime_image_available": False,
            "official_runtime_image_error": f"runtime image {baseline.runtime_image} is not runnable rc={image_rc}",
            "official_ros_packages": {
                package: {
                    "present": False,
                    "prefix": "",
                    "error": f"runtime image {baseline.runtime_image} is not runnable",
                }
                for package in baseline.required_ros_packages
            },
            "missing_official_ros_packages": list(baseline.required_ros_packages),
            "micro_ros_agent_binaries_status": {
                binary: {
                    "present": False,
                    "path": "",
                    "error": f"runtime image {baseline.runtime_image} is not runnable",
                }
                for binary in baseline.micro_ros_agent_binaries
            },
            "micro_ros_agent_available": False,
            "present_micro_ros_agent_binaries": [],
            "official_sitl_launch": baseline.sitl_launch,
            "official_gazebo_launch": baseline.gazebo_launch,
            "official_cartographer_launch": baseline.cartographer_launch,
            "official_dependency_sources": OFFICIAL_REFERENCES,
        }
    packages: dict[str, dict[str, Any]] = {}
    for package in baseline.required_ros_packages:
        rc, output = host._docker_run_ros_shell_capture(
            config=config,
            image=baseline.runtime_image,
            shell_command=f"timeout --signal=INT 8s ros2 pkg prefix {shlex.quote(package)}",
            name=None,
        )
        packages[package] = {
            "present": rc == 0,
            "prefix": output.strip() if rc == 0 else "",
            "error": "" if rc == 0 else f"ros2 pkg prefix {package} failed rc={rc}",
        }
    binaries: dict[str, dict[str, Any]] = {}
    for binary in baseline.micro_ros_agent_binaries:
        rc, output = host._docker_run_ros_shell_capture(
            config=config,
            image=baseline.runtime_image,
            shell_command=(
                f"command -v {shlex.quote(binary)} || "
                "timeout --signal=INT 8s ros2 pkg executables micro_ros_agent | "
                f"awk '{{print $2}}' | grep -Fx {shlex.quote(binary)}"
            ),
            name=None,
        )
        binaries[binary] = {
            "present": rc == 0,
            "path": output.strip() if rc == 0 else "",
            "error": "" if rc == 0 else f"micro_ros_agent executable {binary} lookup failed rc={rc}",
        }
    missing_packages = [name for name, item in packages.items() if not item["present"]]
    present_agent_binaries = [name for name, item in binaries.items() if item["present"]]
    return {
        "official_runtime_image": baseline.runtime_image,
        "official_runtime_image_available": True,
        "official_runtime_image_error": "",
        "official_ros_packages": packages,
        "missing_official_ros_packages": missing_packages,
        "micro_ros_agent_binaries_status": binaries,
        "micro_ros_agent_available": bool(present_agent_binaries),
        "present_micro_ros_agent_binaries": present_agent_binaries,
        "official_sitl_launch": baseline.sitl_launch,
        "official_gazebo_launch": baseline.gazebo_launch,
        "official_cartographer_launch": baseline.cartographer_launch,
        "official_dependency_sources": OFFICIAL_REFERENCES,
    }


def _build_doctor_summary(config: RunConfig) -> dict[str, Any]:
    official_baseline = {
        **_official_baseline_common(config),
        **_cartographer_dependency_summary(config),
        **_official_ros_dependency_summary(config),
        **_cartographer_config_summary(),
    }
    blockers: list[str] = []
    if not official_baseline["cartographer_ros_present"]:
        blockers.append("cartographer_ros is not present in the configured SLAM image")
    if not official_baseline["cartographer_node_present"]:
        blockers.append("cartographer_node is not present in cartographer_ros executables")
    if not official_baseline["cartographer_occupancy_grid_node_present"]:
        blockers.append("cartographer_occupancy_grid_node is not present in cartographer_ros executables")
    if not official_baseline.get("cartographer_config_present"):
        blockers.append("NavLab Cartographer Lua config is missing")
    if official_baseline["direct_set_pose"]:
        blockers.append("direct Gazebo set-pose behavior is not allowed in P0 official baseline")
    if not official_baseline["official_runtime_image_available"]:
        blockers.append(official_baseline["official_runtime_image_error"])
    if official_baseline["missing_official_ros_packages"]:
        blockers.append(
            "official ROS packages are missing from the configured runtime image: "
            f"{official_baseline['missing_official_ros_packages']}"
        )
    if not official_baseline["micro_ros_agent_available"]:
        blockers.append("Micro-XRCE-DDS / micro-ROS-Agent executable is missing from the configured runtime image")
    ok = not blockers
    return {
        "ok": ok,
        "blocked": not ok,
        "blockers": blockers,
        "official_baseline": official_baseline,
    }


def _collect_ros_graph(config: RunConfig, artifact_dir: Path, *, image: str, network: str | None) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    commands = {
        "ros2_node_list": "ros2 node list",
        "ros2_topic_list": "ros2 topic list --include-hidden-topics",
    }
    collected: dict[str, Any] = {}
    for key, command in commands.items():
        rc, output = host._docker_run_ros_shell_capture(
            config=config,
            image=image,
            shell_command=command,
            name=None,
            network=network,
            envs={
                "DDS_ENABLE": config.orchestration.official_baseline.dds_enable,
                "DDS_DOMAIN_ID": config.orchestration.official_baseline.dds_domain_id,
                "ROS_DOMAIN_ID": config.orchestration.official_baseline.dds_domain_id,
                "RMW_IMPLEMENTATION": baseline.rmw_implementation,
            },
        )
        _write_text(artifact_dir / f"{key}.txt", output)
        collected[key] = {
            "rc": rc,
            "output": output,
            "lines": [line.strip() for line in output.splitlines() if line.strip()],
        }
    expected_ap_node = config.orchestration.official_baseline.expected_ap_node
    node_lines = collected.get("ros2_node_list", {}).get("lines", [])
    if expected_ap_node in node_lines:
        rc, output = host._docker_run_ros_shell_capture(
            config=config,
            image=image,
            shell_command=f"ros2 node info {expected_ap_node}",
            name=None,
            network=network,
            envs={
                "DDS_ENABLE": config.orchestration.official_baseline.dds_enable,
                "DDS_DOMAIN_ID": config.orchestration.official_baseline.dds_domain_id,
                "ROS_DOMAIN_ID": config.orchestration.official_baseline.dds_domain_id,
                "RMW_IMPLEMENTATION": baseline.rmw_implementation,
            },
        )
    else:
        rc = 0
        output = f"skipped node info; {expected_ap_node} is not present in ros2 node list\n"
    _write_text(artifact_dir / "ap_node_info.txt", output)
    collected["ap_node_info"] = {
        "rc": rc,
        "output": output,
        "lines": [line.strip() for line in output.splitlines() if line.strip()],
    }
    return collected


def _collect_official_dds_probe(
    config: RunConfig,
    artifact_dir: Path,
    *,
    image: str,
    network: str | None,
) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    probe_script = r"""
import json
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger

topic = "/ap/v1/time"
service = "/ap/v1/prearm_check"
result = {
    "time_topic": topic,
    "prearm_service": service,
    "time_received": False,
    "time_sec": None,
    "time_nanosec": None,
    "prearm_service_available": False,
    "prearm_success": None,
}

rclpy.init()
node = rclpy.create_node("navlab_official_dds_probe")
event = threading.Event()

def callback(msg: Time) -> None:
    result["time_received"] = True
    result["time_sec"] = int(msg.sec)
    result["time_nanosec"] = int(msg.nanosec)
    event.set()

subscription = node.create_subscription(Time, topic, callback, 1)
client = node.create_client(Trigger, service)
deadline = time.monotonic() + 12.0
while time.monotonic() < deadline and not event.is_set():
    rclpy.spin_once(node, timeout_sec=0.1)

result["prearm_service_available"] = bool(client.wait_for_service(timeout_sec=3.0))
if result["prearm_service_available"]:
    future = client.call_async(Trigger.Request())
    call_deadline = time.monotonic() + 5.0
    while time.monotonic() < call_deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if future.done() and future.result() is not None:
        result["prearm_success"] = bool(future.result().success)

node.destroy_subscription(subscription)
node.destroy_node()
rclpy.shutdown()
print(json.dumps(result, sort_keys=True))
"""
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=image,
        shell_command=f"python3 - <<'PY'\n{probe_script}\nPY",
        name=None,
        network=network,
        envs={
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
        },
    )
    _write_text(artifact_dir / "official_dds_probe.txt", output)
    parsed: dict[str, Any] = {}
    if rc == 0:
        for line in reversed([line.strip() for line in output.splitlines() if line.strip()]):
            try:
                parsed = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    return {
        "rc": rc,
        "output": output,
        "result": parsed,
    }


def _rosbag_profile_summary(config: RunConfig) -> dict[str, Any]:
    profile_path = Path(config.official_baseline_rosbag_profile)
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
    return {
        "ok": profile_path.is_file(),
        "profile": str(profile_path),
        "required_topics": required,
        "optional_topics": optional,
        "recorded": False,
        "reason": "P0 graph acceptance records ROS graph artifacts; rosbag capture is a later official bringup step.",
    }


def _official_rosbag_topics(config: RunConfig) -> tuple[list[str], list[str], list[str]]:
    profile_summary = _rosbag_profile_summary(config)
    required = list(profile_summary["required_topics"])
    optional = list(profile_summary["optional_topics"])
    return required, optional, [*required, *optional]


def _load_rosbag_metadata_counts(metadata: Path) -> dict[str, int]:
    content = metadata.read_text(encoding="utf-8")
    counts: dict[str, int] = {}
    topic_matches = list(re.finditer(r"name: (/[^\n]+)", content))
    for index, match in enumerate(topic_matches):
        topic = match.group(1).strip()
        end = topic_matches[index + 1].start() if index + 1 < len(topic_matches) else len(content)
        block = content[match.end() : end]
        count_match = re.search(r"message_count:\s*(\d+)", block)
        counts[topic] = int(count_match.group(1)) if count_match else 0
    return counts


def _validate_official_rosbag_profile(
    *,
    profile: Path,
    metadata: Path,
    required: list[str],
    optional: list[str],
) -> dict[str, Any]:
    counts = _load_rosbag_metadata_counts(metadata)
    missing_required = [topic for topic in required if topic not in counts]
    zero_count_required = [topic for topic in required if counts.get(topic, 0) <= 0]
    present_optional = [topic for topic in optional if counts.get(topic, 0) > 0]
    missing_optional = [topic for topic in optional if topic not in counts]
    return {
        "ok": not missing_required and not zero_count_required,
        "recorded": True,
        "profile": str(profile),
        "metadata": str(metadata),
        "required_topics": required,
        "optional_topics": optional,
        "present_topics": sorted(counts),
        "message_counts": counts,
        "missing_required_topics": missing_required,
        "zero_count_required_topics": zero_count_required,
        "present_optional_topics": present_optional,
        "missing_optional_topics": missing_optional,
    }


def _record_official_rosbag(
    config: RunConfig,
    artifact_dir: Path,
    *,
    image: str,
    network: str | None,
    duration_sec: float = 8.0,
) -> dict[str, Any]:
    profile_path = Path(config.official_baseline_rosbag_profile)
    required, optional, topics = _official_rosbag_topics(config)
    if not profile_path.is_file():
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "reason": "rosbag profile does not exist",
        }
        _write_json(artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    if not topics:
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "reason": "rosbag profile has no topics",
        }
        _write_json(artifact_dir / "rosbag_profile_summary.json", summary)
        return summary

    container_rosbag = Path("/workspace") / artifact_dir / "rosbag"
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
        network=network,
        envs={
            "DDS_ENABLE": baseline.dds_enable,
            "DDS_DOMAIN_ID": baseline.dds_domain_id,
            "ROS_DOMAIN_ID": baseline.dds_domain_id,
            "RMW_IMPLEMENTATION": baseline.rmw_implementation,
        },
    )
    _write_text(artifact_dir / "rosbag_record.txt", output)
    metadata = artifact_dir / "rosbag" / "metadata.yaml"
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
        _write_json(artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    summary = _validate_official_rosbag_profile(
        profile=profile_path,
        metadata=metadata,
        required=required,
        optional=optional,
    )
    _write_json(artifact_dir / "rosbag_profile_summary.json", summary)
    return summary


def _write_foxglove_notes(artifact_dir: Path) -> None:
    _write_text(
        artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P0 official baseline Foxglove notes",
                "",
                "P0 is an official baseline visibility check, not a hover completion gate.",
                "",
                "- Expected fixed frame once the official DDS path is available: `map` "
                "or the official `/ap` frame tree.",
                "- Required graph evidence is recorded in `ros2_node_list.txt`, "
                "`ros2_topic_list.txt`, and `ap_node_info.txt`.",
                "- MAVLink fallback and Gazebo truth diagnostics do not count as official P0 completion.",
                "- A rosbag profile is present for the future official bringup capture, "
                "but this graph gate does not claim rosbag replay success.",
            ]
        )
        + "\n",
    )



def run_official_baseline_doctor(*, config_path: str | Path | None = None, console: Console | None = None) -> int:
    console = console or Console()
    config = RunConfig.from_config(config_path=config_path)
    artifact_dir = Path(
        os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_official_baseline_doctor/{config.run_id}")
    )
    artifact_dir.mkdir(parents=True, exist_ok=True)
    console.print("[bold cyan]Checking official baseline prerequisites[/bold cyan]")
    summary = _build_doctor_summary(config)
    _write_json(artifact_dir / "summary.json", summary)
    color = "green" if summary["ok"] else "red"
    console.print(f"[{color}]Official baseline doctor rc={0 if summary['ok'] else 20}[/{color}]")
    console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
    return 0 if summary["ok"] else 20



def run_official_baseline_acceptance(
    *,
    config_path: str | Path | None = None,
    duration_sec: float = 30.0,
    console: Console | None = None,
) -> int:
    console = console or Console()
    config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
    config.artifact_dir.mkdir(parents=True, exist_ok=True)
    host._render_run_config(console, config)
    summary: dict[str, Any] | None = None
    try:
        console.print("[bold cyan]Starting SITL + Gazebo stack for official baseline graph check[/bold cyan]")
        try:
            host._compose_stop(config)
        except DockerException:
            pass
        host._start_official_baseline_container(config)
        time.sleep(min(max(duration_sec, 1.0), 10.0))
        baseline = config.orchestration.official_baseline
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
        rosbag_profile = _record_official_rosbag(
            config,
            config.artifact_dir,
            image=baseline.runtime_image,
            network="host",
        )
        node_lines = graph["ros2_node_list"]["lines"]
        topic_lines = graph["ros2_topic_list"]["lines"]
        probe_result = probe["result"]
        graph_ap_topics = [topic for topic in topic_lines if topic.startswith("/ap/")]
        probe_confirmed_ap_topics = []
        if probe_result.get("time_received") and probe_result.get("time_topic"):
            probe_confirmed_ap_topics.append(str(probe_result["time_topic"]))
        ap_topics = sorted(set(graph_ap_topics + probe_confirmed_ap_topics))
        missing_required_ap_topics = [
            topic for topic in baseline.required_ap_topics if topic not in ap_topics
        ]
        official_baseline = {
            **_official_baseline_common(config),
            **_cartographer_dependency_summary(config),
            **_official_ros_dependency_summary(config),
            **_cartographer_config_summary(),
            "ap_node_present": baseline.expected_ap_node in node_lines,
            "graph_ap_topics": graph_ap_topics,
            "probe_confirmed_ap_topics": probe_confirmed_ap_topics,
            "ap_topics": ap_topics,
            "missing_required_ap_topics": missing_required_ap_topics,
            "ap_node_info_rc": graph["ap_node_info"]["rc"],
            "official_dds_probe_rc": probe["rc"],
            "official_dds_probe": probe_result,
            "official_dds_time_received": bool(probe_result.get("time_received")),
            "official_dds_prearm_service_available": bool(
                probe_result.get("prearm_service_available")
            ),
            "official_dds_prearm_success": probe_result.get("prearm_success"),
            "process_status": host._compose_ps_status(config),
            "official_container": host.OFFICIAL_BASELINE_CONTAINER,
        }
        blockers: list[str] = []
        if not official_baseline["cartographer_ros_present"]:
            blockers.append("cartographer_ros is not present in the configured SLAM image")
        if not official_baseline["official_runtime_image_available"]:
            blockers.append(official_baseline["official_runtime_image_error"])
        if official_baseline["missing_official_ros_packages"]:
            blockers.append(
                "official ROS packages are missing from the configured runtime image: "
                f"{official_baseline['missing_official_ros_packages']}"
            )
        if not official_baseline["micro_ros_agent_available"]:
            blockers.append(
                "Micro-XRCE-DDS / micro-ROS-Agent executable is missing from the configured runtime image"
            )
        if official_baseline["direct_set_pose"]:
            blockers.append("direct Gazebo set-pose behavior is not allowed in P0 official baseline")
        if not official_baseline["official_dds_time_received"]:
            blockers.append("official DDS probe did not receive /ap/v1/time")
        if not official_baseline["official_dds_prearm_service_available"]:
            blockers.append("official DDS probe did not find /ap/v1/prearm_check service")
        if not rosbag_profile.get("ok"):
            blockers.append("official baseline rosbag profile did not pass")
        if not _route_is_official(baseline.external_nav_route):
            blockers.append(f"external_nav_route={baseline.external_nav_route!r} is not official_dds")
        if baseline.gazebo_bringup_mode != "official_gz_bringup":
            blockers.append(
                f"gazebo_bringup_mode={baseline.gazebo_bringup_mode!r} is not official_gz_bringup"
            )
        summary = {
            "ok": not blockers,
            "blocked": bool(blockers),
            "blockers": blockers,
            "official_baseline": official_baseline,
            "rosbag_profile": rosbag_profile,
        }
        _write_json(config.artifact_dir / "summary.json", summary)
        _write_foxglove_notes(config.artifact_dir)
    finally:
        host._capture_official_baseline_log(config=config)
        host._remove_official_baseline_container()
        try:
            host._compose_stop(config)
        except DockerException:
            pass
    if summary is None:
        summary = {
            "ok": False,
            "blocked": True,
            "blockers": ["official baseline acceptance did not produce a summary"],
        }
        _write_json(config.artifact_dir / "summary.json", summary)
    color = "green" if summary["ok"] else "red"
    console.print(f"[{color}]Official baseline acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
    console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
    return 0 if summary["ok"] else 30
