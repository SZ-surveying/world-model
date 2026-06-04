from __future__ import annotations

import json
from pathlib import Path

from navlab.companion.acceptance import _scan_publisher_summary, _write_summary
from navlab.companion.config import NodeConfig, RuntimeConfig


def _metadata_for_counts(counts: dict[str, int]) -> str:
    topics = []
    for topic, count in counts.items():
        topics.append(
            {
                "topic_metadata": {
                    "name": topic,
                    "type": "std_msgs/msg/String",
                    "serialization_format": "cdr",
                    "offered_qos_profiles": "",
                },
                "message_count": count,
            }
        )
    return "rosbag2_bagfile_information:\n  topics_with_message_count:\n" + "\n".join(
        [
            "    - topic_metadata:\n"
            f"        name: {item['topic_metadata']['name']}\n"
            f"        type: {item['topic_metadata']['type']}\n"
            "        serialization_format: cdr\n"
            "        offered_qos_profiles: ''\n"
            f"      message_count: {item['message_count']}"
            for item in topics
        ]
    )


def _runtime_config(*, pose_args: tuple[str, ...] = ()) -> RuntimeConfig:
    return RuntimeConfig(
        path=Path("navlab/config.toml"),
        imu_source_label="fcu_mavlink_navlab",
        world_markers=NodeConfig(autostart=True, args=()),
        scan_features=NodeConfig(autostart=True, args=()),
        gazebo_truth_bridge=NodeConfig(autostart=False),
        gazebo_truth_odom=NodeConfig(autostart=False),
        pose_mirror=NodeConfig(autostart=True, endpoint="tcp:mavlink-router:5760", args=pose_args),
        imu_bridge=NodeConfig(autostart=False),
        external_nav_sender=NodeConfig(autostart=True, endpoint="tcp:mavlink-router:5760", args=()),
        mission=NodeConfig(autostart=False, endpoint="tcp:mavlink-router:5760", args=()),
    )


def _samples(*, pose_set_count: int = 0, pose_reason: str = "mavlink_local_position_observed") -> str:
    def section(label: str, payload: dict[str, object]) -> str:
        return f"{label}\ndata: '{json.dumps(payload, separators=(',', ':'))}'\n"

    return "".join(
        [
            section("MISSION_STATUS_SAMPLE", {"obstacle_detected": True}),
            section("MAVLINK_STATUS_SAMPLE", {"heartbeat_seen": True}),
            section(
                "POSE_MIRROR_STATUS_SAMPLE",
                {
                    "state": "mirroring",
                    "set_pose_count": pose_set_count,
                    "reason": pose_reason,
                },
            ),
            section("IMU_STATUS_SAMPLE", {"state": "streaming_fcu_imu", "ready": True}),
            section("CARTOGRAPHER_STATUS_SAMPLE", {"ready": True}),
            section("EXTERNAL_STATUS_SAMPLE", {"state": "healthy", "ready": True}),
            section("MAVLINK_EXTERNAL_NAV_STATUS_SAMPLE", {"state": "sending"}),
            section("SCAN_FEATURES_SAMPLE", {"front_clearance_m": 5.0}),
            section("SIM_LOG_SAMPLE", {"event": "ok"}),
            section(
                "X2_STATUS_SAMPLE",
                {
                    "source": "x2_serial_emulator",
                    "latest_scan_ideal_age_sec": 0.05,
                },
            ),
            "SCAN_TOPIC_INFO\n"
            "Type: sensor_msgs/msg/LaserScan\n"
            "Publisher count: 1\n"
            "Node name: ydlidar_ros2_driver_node\n"
            "Node namespace: /\n"
            "Topic type: sensor_msgs/msg/LaserScan\n"
            "Subscription count: 1\n"
            "Node name: scan_features_publisher\n"
            "Node namespace: /\n",
        ]
    )


def _write_acceptance_inputs(tmp_path: Path) -> dict[str, object]:
    (tmp_path / "rosbag").mkdir()
    counts = {
        "/scan": 10,
        "/scan_ideal": 12,
        "/sim/x2/status": 3,
        "/scan_features": 2,
        "/navlab/mission/status": 4,
    }
    (tmp_path / "rosbag" / "metadata.yaml").write_text(_metadata_for_counts(counts), encoding="utf-8")
    (tmp_path / "mission_summary.json").write_text(
        json.dumps(
            {
                "phases_seen": [
                    "wait_ready",
                    "hover_settle",
                    "forward",
                    "scan_left",
                    "scan_right",
                    "avoid",
                    "final_hold",
                ],
                "obstacle_detected": True,
                "avoidance_setpoint_sent": True,
                "scan_left_seen": True,
                "scan_right_seen": True,
            }
        ),
        encoding="utf-8",
    )
    rosbag_profile = {"ok": True, "message_counts": counts}
    return rosbag_profile


def test_scan_publisher_summary_reads_only_publishers() -> None:
    samples = """SCAN_TOPIC_INFO
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Node name: ydlidar_ros2_driver_node
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Subscription count: 1
Node name: scan_features_publisher
Node namespace: /
"""

    summary = _scan_publisher_summary(samples)

    assert summary["publisher_nodes"] == ["ydlidar_ros2_driver_node"]
    assert summary["vendor_driver_publisher"] is True
    assert summary["emulator_publisher"] is False


def test_write_summary_requires_x2_lidar_and_observation_mode(tmp_path: Path) -> None:
    rosbag_profile = _write_acceptance_inputs(tmp_path)
    (tmp_path / "samples.txt").write_text(_samples(), encoding="utf-8")

    rc = _write_summary(
        artifact_dir=tmp_path,
        duration_sec=90.0,
        mission_rc=0,
        rosbag_profile=rosbag_profile,
        config=_runtime_config(),
        scan_source="x2_virtual_serial_vendor_driver",
    )

    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert rc == 0
    assert summary["ok"] is True
    assert summary["lidar_chain"]["scan_count"] == 10
    assert summary["lidar_chain"]["scan_ideal_count"] == 12
    assert summary["lidar_chain"]["x2_status_count"] == 3
    assert summary["x2_status"]["source"] == "x2_serial_emulator"
    assert summary["observation_mode"]["pose_mirror_observation_only"] is True


def test_write_summary_fails_when_pose_mirror_can_set_gazebo_pose(tmp_path: Path) -> None:
    rosbag_profile = _write_acceptance_inputs(tmp_path)
    (tmp_path / "samples.txt").write_text(_samples(pose_set_count=2, pose_reason="set_pose_sent"), encoding="utf-8")

    rc = _write_summary(
        artifact_dir=tmp_path,
        duration_sec=90.0,
        mission_rc=0,
        rosbag_profile=rosbag_profile,
        config=_runtime_config(pose_args=("--set-gazebo-pose",)),
        scan_source="x2_virtual_serial_vendor_driver",
    )

    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert rc == 3
    assert summary["ok"] is False
    assert summary["observation_mode"]["pose_mirror_set_pose_disabled"] is False
