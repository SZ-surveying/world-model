from __future__ import annotations

import hashlib
import json
import re
import shlex
from pathlib import Path
from typing import Any

from src import host
from src.configs.run_config import RunConfig
from src.tasks.helpers.artifacts import write_text

CARTOGRAPHER_CONFIG_PATH = Path(
    "navlab/common/slam/ros/localization/navlab_cartographer_adapter/config/navlab_cartographer_2d.lua"
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
    prefix_rc, prefix_output = host.docker_run_ros_shell_capture(
        config=config,
        image=config.slam_image,
        shell_command="timeout --signal=INT 8s ros2 pkg prefix cartographer_ros",
        name=None,
    )
    executables_rc, executables_output = host.docker_run_ros_shell_capture(
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


def _official_ros_dependency_summary(config: RunConfig) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    image_rc, _image_output = host.docker_run_ros_shell_capture(
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
        rc, output = host.docker_run_ros_shell_capture(
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
        rc, output = host.docker_run_ros_shell_capture(
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


def build_doctor_summary(config: RunConfig) -> dict[str, Any]:
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


def collect_ros_graph(config: RunConfig, artifact_dir: Path, *, image: str, network: str | None) -> dict[str, Any]:
    baseline = config.orchestration.official_baseline
    commands = {
        "ros2_node_list": "ros2 node list",
        "ros2_topic_list": "ros2 topic list --include-hidden-topics",
    }
    collected: dict[str, Any] = {}
    for key, command in commands.items():
        rc, output = host.docker_run_ros_shell_capture(
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
        write_text(artifact_dir / f"{key}.txt", output)
        collected[key] = {
            "rc": rc,
            "output": output,
            "lines": [line.strip() for line in output.splitlines() if line.strip()],
        }
    expected_ap_node = config.orchestration.official_baseline.expected_ap_node
    node_lines = collected.get("ros2_node_list", {}).get("lines", [])
    if expected_ap_node in node_lines:
        rc, output = host.docker_run_ros_shell_capture(
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
    write_text(artifact_dir / "ap_node_info.txt", output)
    collected["ap_node_info"] = {
        "rc": rc,
        "output": output,
        "lines": [line.strip() for line in output.splitlines() if line.strip()],
    }
    return collected


def collect_official_dds_probe(
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
    rc, output = host.docker_run_ros_shell_capture(
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
    write_text(artifact_dir / "official_dds_probe.txt", output)
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


def _write_foxglove_notes(artifact_dir: Path) -> None:
    write_text(
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
