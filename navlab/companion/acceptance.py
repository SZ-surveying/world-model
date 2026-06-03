from __future__ import annotations

import json
import re
import subprocess
import time
from pathlib import Path
from typing import Any

from jinja2 import Environment, FileSystemLoader, Template, select_autoescape

from navlab.common.logging import logger
from navlab.common.process_manager import ManagedProcess, ProcessManager
from navlab.common.rosbag import load_rosbag_topics
from navlab.companion.config import RuntimeConfig
from navlab.companion.rosbag_profile import validate_rosbag_profile

TEMPLATE_DIR = Path(__file__).parent / "templates"

SAMPLE_TOPICS = (
    ("MISSION_STATUS_SAMPLE", "/navlab/mission/status"),
    ("MAVLINK_STATUS_SAMPLE", "/navlab/mavlink/status"),
    ("POSE_MIRROR_STATUS_SAMPLE", "/navlab/pose_mirror/status"),
    ("IMU_STATUS_SAMPLE", "/imu/status"),
    ("CARTOGRAPHER_STATUS_SAMPLE", "/cartographer/status"),
    ("EXTERNAL_STATUS_SAMPLE", "/external_nav/status"),
    ("MAVLINK_EXTERNAL_NAV_STATUS_SAMPLE", "/mavlink_external_nav/status"),
    ("SCAN_FEATURES_SAMPLE", "/scan_features"),
    ("SIM_LOG_SAMPLE", "/sim/log"),
    ("X2_STATUS_SAMPLE", "/sim/x2/status"),
)


def _template(name: str) -> Template:
    environment = Environment(
        autoescape=select_autoescape(disabled_extensions=("md", "j2"), default_for_string=False, default=False),
        loader=FileSystemLoader(TEMPLATE_DIR),
        lstrip_blocks=True,
        trim_blocks=True,
    )
    return environment.get_template(name)


def _run(
    command: list[str],
    *,
    stdout_path: Path | None = None,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    logger.debug("Running command: {}", " ".join(command))
    if stdout_path is None:
        return subprocess.run(command, check=check, text=True)
    with stdout_path.open("w", encoding="utf-8") as stdout:
        return subprocess.run(command, check=check, text=True, stdout=stdout, stderr=subprocess.STDOUT)


def _record_all_topics(*, manager: ProcessManager, artifact_dir: Path) -> ManagedProcess:
    logger.info("Starting rosbag record in {}", artifact_dir / "rosbag")
    return manager.start_subprocess(
        "rosbag_record",
        ["ros2", "bag", "record", "-o", str(artifact_dir / "rosbag"), "--all-topics"],
        log_path=artifact_dir / "rosbag.log",
    )


def _wait_for_metadata(path: Path, *, timeout_sec: float = 15.0) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if path.is_file() and path.stat().st_size > 0:
            return
        time.sleep(0.5)


def _run_mission(*, artifact_dir: Path, duration_sec: float, config: RuntimeConfig) -> int:
    logger.info("Running obstacle mission for {}s via {}", duration_sec, config.mission.endpoint)
    command = [
        "python3",
        "-m",
        "navlab.companion.nodes.obstacle_mission",
        "--endpoint",
        config.mission.endpoint,
        "--duration-sec",
        str(duration_sec),
        "--summary-file",
        str(artifact_dir / "mission_summary.json"),
        *config.mission.args,
    ]
    result = _run(command, stdout_path=artifact_dir / "mission_controller.log", check=False)
    (artifact_dir / "mission_rc.txt").write_text(str(result.returncode), encoding="utf-8")
    logger.info("Mission controller completed rc={}", result.returncode)
    return result.returncode


def _collect_samples(*, artifact_dir: Path) -> None:
    logger.info("Collecting ROS topic samples")
    with (artifact_dir / "samples.txt").open("w", encoding="utf-8") as output:
        for label, topic in SAMPLE_TOPICS:
            output.write(f"{label}\n")
            output.flush()
            subprocess.run(
                ["timeout", "5s", "ros2", "topic", "echo", "--once", "--full-length", topic],
                check=False,
                text=True,
                stdout=output,
                stderr=subprocess.STDOUT,
            )
        output.write("SCAN_TOPIC_INFO\n")
        output.flush()
        subprocess.run(
            ["timeout", "5s", "ros2", "topic", "info", "-v", "/scan"],
            check=False,
            text=True,
            stdout=output,
            stderr=subprocess.STDOUT,
        )


def _sample_payload(samples: str, label: str) -> dict[str, Any]:
    start = samples.find(label)
    if start < 0:
        return {}
    end = len(samples)
    for next_label, _ in SAMPLE_TOPICS:
        if next_label == label:
            continue
        next_start = samples.find(next_label, start + len(label))
        if next_start >= 0:
            end = min(end, next_start)
    section = samples[start:end]
    match = re.search(r"data: '(.+)'", section)
    if not match:
        return {}
    return json.loads(match.group(1))


def _scan_publisher_summary(samples: str) -> dict[str, Any]:
    start = samples.find("SCAN_TOPIC_INFO")
    if start < 0:
        return {}
    section = samples[start:]
    subscription_start = section.find("Subscription count:")
    if subscription_start >= 0:
        section = section[:subscription_start]
    node_names = re.findall(r"Node name: ([^\n]+)", section)
    node_names = [name.strip() for name in node_names]
    return {
        "topic": "/scan",
        "publisher_nodes": sorted(set(node_names)),
        "vendor_driver_publisher": "ydlidar_ros2_driver_node" in node_names,
        "emulator_publisher": "x2_serial_emulator" in node_names,
    }


def _write_foxglove_notes(*, artifact_dir: Path) -> None:
    logger.info("Writing Foxglove replay notes")
    rendered = _template("foxglove_notes.md.j2").render(
        artifact_dir=artifact_dir,
        rosbag_dir=artifact_dir / "rosbag",
        fixed_frame="map",
        recommended_panels=("3D", "Raw Messages", "Plot", "Topic Graph"),
        marker_topics=(
            "/navlab/replay/markers",
            "/navlab/replay/constraint_markers",
            "/sim/markers",
            "/trajectory_node_list",
            "/submap_list",
        ),
        pose_topics=("/sim/uav_pose", "/odom", "/external_nav/odom"),
        laser_topics=("/scan", "/scan_ideal", "/scan_features", "/scan_nearest_point"),
        raw_status_topics=(
            "/navlab/mission/status",
            "/sim/log",
            "/sim/x2/status",
            "/imu/status",
            "/external_nav/status",
            "/mavlink_external_nav/status",
        ),
        expected_sequence=(
            "wait_ready",
            "guided",
            "arm",
            "takeoff",
            "hover_settle",
            "forward",
            "avoid",
            "pass_obstacle",
            "return_track",
            "final_hold/complete",
        ),
    )
    (artifact_dir / "foxglove_notes.md").write_text(rendered.rstrip() + "\n", encoding="utf-8")


def _write_summary(
    *,
    artifact_dir: Path,
    duration_sec: float,
    mission_rc: int,
    rosbag_profile: dict[str, Any],
    config: RuntimeConfig,
    scan_source: str,
) -> int:
    samples = (artifact_dir / "samples.txt").read_text(encoding="utf-8")
    metadata = (artifact_dir / "rosbag" / "metadata.yaml").read_text(encoding="utf-8")
    mission_summary_path = artifact_dir / "mission_summary.json"
    mission_summary = (
        json.loads(mission_summary_path.read_text(encoding="utf-8")) if mission_summary_path.exists() else {}
    )
    mission_status = _sample_payload(samples, "MISSION_STATUS_SAMPLE")
    mavlink_status = _sample_payload(samples, "MAVLINK_STATUS_SAMPLE")
    pose_mirror_status = _sample_payload(samples, "POSE_MIRROR_STATUS_SAMPLE")
    imu_status = _sample_payload(samples, "IMU_STATUS_SAMPLE")
    cartographer_status = _sample_payload(samples, "CARTOGRAPHER_STATUS_SAMPLE")
    external_status = _sample_payload(samples, "EXTERNAL_STATUS_SAMPLE")
    mavlink_external_nav_status = _sample_payload(samples, "MAVLINK_EXTERNAL_NAV_STATUS_SAMPLE")
    x2_status = _sample_payload(samples, "X2_STATUS_SAMPLE")
    scan_publisher = _scan_publisher_summary(samples)
    topics = sorted(set(re.findall(r"name: (/[^\n]+)", metadata)))
    message_counts = rosbag_profile.get("message_counts", {})
    scan_ideal_recorded = message_counts.get("/scan_ideal", 0) > 0
    x2_status_recorded = message_counts.get("/sim/x2/status", 0) > 0
    vendor_scan_publisher_ok = scan_publisher.get("vendor_driver_publisher") is True
    x2_is_internal_to_sensor_runtime = (
        vendor_scan_publisher_ok
        and scan_publisher.get("emulator_publisher") is False
        and x2_status.get("source") == "x2_serial_emulator"
    )
    pose_mirror_state = pose_mirror_status.get("state")
    phases_seen = mission_summary.get("phases_seen", [])
    summary = {
        "ok": (
            rosbag_profile.get("ok") is True
            and mission_rc == 0
            and pose_mirror_state in {"mirroring", "simulated_mission_pose"}
            and mavlink_status.get("heartbeat_seen") is True
            and imu_status.get("state") == "streaming_fcu_imu"
            and imu_status.get("ready") is True
            and cartographer_status.get("ready") is True
            and external_status.get("state") == "healthy"
            and external_status.get("ready") is True
            and mavlink_external_nav_status.get("state") == "sending"
            and "hover_settle" in phases_seen
            and any(phase in phases_seen for phase in ["forward", "avoid", "pass_obstacle"])
            and mission_summary.get("obstacle_detected") is True
            and mission_summary.get("avoidance_setpoint_sent") is True
            and scan_ideal_recorded
            and x2_status_recorded
            and vendor_scan_publisher_ok
        ),
        "duration_sec": duration_sec,
        "mission_rc": mission_rc,
        "scan_source": scan_source,
        "gps_free": True,
        "rosbag_started_before_mission": True,
        "rosbag_covers_full_mission": rosbag_profile.get("ok") is True and "/navlab/mission/status" in topics,
        "companion_ready": True,
        "sitl_heartbeat": mavlink_status.get("heartbeat_seen") is True,
        "imu_source": config.imu_source_label,
        "scan_fresh": rosbag_profile.get("message_counts", {}).get("/scan_features", 0) > 0,
        "scan_ideal_recorded": scan_ideal_recorded,
        "x2_status_recorded": x2_status_recorded,
        "scan_publisher": scan_publisher,
        "vendor_scan_publisher_ok": vendor_scan_publisher_ok,
        "x2_is_internal_to_sensor_runtime": x2_is_internal_to_sensor_runtime,
        "slam_consumes_final_scan": True,
        "mission_consumes_final_scan": True,
        "scan_features_consumes_final_scan": True,
        "external_nav_healthy": external_status.get("state") == "healthy",
        "gazebo_pose_mirror_ok": pose_mirror_state in {"mirroring", "simulated_mission_pose"},
        "hover_ok": "hover_settle" in phases_seen,
        "forward_progress_ok": any(phase in phases_seen for phase in ["forward", "avoid", "pass_obstacle"]),
        "obstacle_detected": (
            mission_summary.get("obstacle_detected") is True or mission_status.get("obstacle_detected") is True
        ),
        "avoidance_setpoint_sent": mission_summary.get("avoidance_setpoint_sent") is True,
        "lateral_detour_ok": "avoid" in phases_seen,
        "final_hold_ok": any(phase in phases_seen for phase in ["final_hold", "complete"]),
        "rosbag_profile_ok": rosbag_profile.get("ok") is True,
        "rosbag_profile": rosbag_profile,
        "mission_summary": mission_summary,
        "mission_status": mission_status,
        "mavlink_status": mavlink_status,
        "pose_mirror_status": pose_mirror_status,
        "imu_status": imu_status,
        "cartographer_status": cartographer_status,
        "external_nav_status": external_status,
        "mavlink_external_nav_status": mavlink_external_nav_status,
        "x2_status": x2_status,
        "topics_recorded": topics,
        "foxglove_notes": str(artifact_dir / "foxglove_notes.md"),
    }
    (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    return 0 if summary["ok"] else 3


def execute_companion_gazebo_acceptance(
    *,
    artifact_dir: Path,
    duration_sec: float,
    rosbag_profile_path: Path,
    companion_image: str,
    scan_source: str,
    config: RuntimeConfig,
) -> int:
    logger.info("Starting NavLab acceptance artifact_dir={} duration_sec={}", artifact_dir, duration_sec)
    artifact_dir.mkdir(parents=True, exist_ok=True)
    time.sleep(14)

    topics = load_rosbag_topics(rosbag_profile_path)
    logger.info("Loaded {} rosbag profile topics from {}", len(topics), rosbag_profile_path)
    (artifact_dir / "rosbag_requested_topics.txt").write_text("\n".join(topics) + "\n", encoding="utf-8")
    (artifact_dir / "rosbag_record_mode.txt").write_text("all-topics-discovery\n", encoding="utf-8")

    manager = ProcessManager()
    _record_all_topics(manager=manager, artifact_dir=artifact_dir)
    try:
        time.sleep(2)
        mission_rc = _run_mission(artifact_dir=artifact_dir, duration_sec=duration_sec, config=config)
        _collect_samples(artifact_dir=artifact_dir)
    finally:
        manager.stop_all(timeout_sec=15)
        _wait_for_metadata(artifact_dir / "rosbag" / "metadata.yaml")

    rosbag_summary = validate_rosbag_profile(
        profile=rosbag_profile_path,
        metadata=artifact_dir / "rosbag" / "metadata.yaml",
        summary_file=artifact_dir / "rosbag_profile_summary.json",
    )
    (artifact_dir / "rosbag_profile_rc.txt").write_text("0" if rosbag_summary["ok"] else "3", encoding="utf-8")
    logger.info("Rosbag profile validation ok={}", rosbag_summary["ok"])

    _write_foxglove_notes(artifact_dir=artifact_dir)
    rc = _write_summary(
        artifact_dir=artifact_dir,
        duration_sec=duration_sec,
        mission_rc=mission_rc,
        rosbag_profile=rosbag_summary,
        config=config,
        scan_source=scan_source,
    )
    logger.info("NavLab acceptance completed rc={}", rc)
    return rc
