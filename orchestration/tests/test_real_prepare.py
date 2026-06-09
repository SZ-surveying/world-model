from __future__ import annotations

from pathlib import Path

from src.config import RunConfig
from src.runtime.process_backend import ProcessBackend
from src.tasks import real_prepare
from src.tasks.real_prepare import RealTopicSnapshot, TopicEvidence


def _complete_hover_snapshot() -> RealTopicSnapshot:
    return RealTopicSnapshot(
        topics={
            "/scan": TopicEvidence(
                type_name="sensor_msgs/msg/LaserScan",
                frame_id="laser_frame",
                metadata={"sample_seen": True, "range_count": 360},
            ),
            "/imu/data": TopicEvidence(
                type_name="sensor_msgs/msg/Imu",
                frame_id="imu_link",
                metadata={"sample_seen": True},
            ),
            "/imu": TopicEvidence(
                type_name="sensor_msgs/msg/Imu",
                frame_id="imu_link",
                metadata={"sample_seen": True},
            ),
            "/imu/status": TopicEvidence(type_name="std_msgs/msg/String", metadata={"ready": True}),
            "/tf": TopicEvidence(type_name="tf2_msgs/msg/TFMessage"),
            "/tf_static": TopicEvidence(type_name="tf2_msgs/msg/TFMessage"),
            "/slam/odom": TopicEvidence(type_name="nav_msgs/msg/Odometry"),
            "/navlab/slam/status": TopicEvidence(
                type_name="std_msgs/msg/String",
                metadata={
                    "ready": True,
                    "external_nav_yaw_ready": True,
                    "scan": {"fresh": True, "count": 10},
                    "imu": {"fresh": True, "count": 10},
                    "tf": {"fresh": True, "count": 10},
                },
            ),
            "/navlab/mavlink/status": TopicEvidence(type_name="std_msgs/msg/String"),
            "/navlab/fcu/local_position_pose": TopicEvidence(),
            "/mavlink_external_nav/status": TopicEvidence(type_name="std_msgs/msg/String"),
            "/external_nav/status": TopicEvidence(
                type_name="std_msgs/msg/String",
                metadata={
                    "ready": True,
                    "external_nav_yaw_ready": True,
                    "odom": {"input_topic": "/slam/odom", "frame_ok": True, "rate_ok": True},
                },
            ),
        },
        collected_at="2026-06-09T00:00:00Z",
    )


def test_real_prepare_config_parser_loads_router_serial_and_services() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    prepare = config.orchestration.real_prepare

    assert prepare.mavlink_router_serial_port == "/dev/ttyUSB1"
    assert prepare.mavlink_router_baud == 115200
    assert prepare.mavlink_router_local_endpoint == "127.0.0.1:14550"
    assert prepare.fcu_bridge_mode == "navlab_mavlink"
    assert prepare.mavlink_router.command[-1] == "/dev/ttyUSB1:115200"
    assert prepare.navlab_mavlink_bridge.enabled is True
    assert "orchestration/src/tasks/fcu_bridge/navlab_mavlink_bridge.py" in prepare.navlab_mavlink_bridge.command
    assert "/navlab/mavlink/status" in prepare.navlab_mavlink_bridge.health_topics
    assert "/imu/data" in prepare.navlab_mavlink_bridge.health_topics
    assert "/imu/status" in prepare.navlab_mavlink_bridge.health_topics
    assert prepare.mavros.enabled is False
    assert prepare.mavros.command[3] == "apm.launch"
    assert prepare.mavros.command[-1] == "fcu_url:=udp://@127.0.0.1:14550"
    assert prepare.lidar.health_topics == ("/scan",)
    assert "launch_cartographer_backend:=true" in prepare.slam.command
    assert "publish_placeholder_odom:=false" in prepare.slam.command
    assert "cartographer_odometry_topic:=/odometry" in prepare.slam.command
    assert "scan_topic:=/scan" in prepare.slam.command
    assert "imu_topic:=/imu" in prepare.slam.command
    assert "odom_topic:=/slam/odom" in prepare.slam.command
    assert "external_nav_input_odom_topic:=/slam/odom" in prepare.slam.command
    assert "require_imu_for_external_nav:=false" in prepare.slam.command
    assert prepare.slam.health_topics == ("/imu", "/slam/odom", "/navlab/slam/status", "/external_nav/status")
    assert prepare.rangefinder_bridge.enabled is False
    assert prepare.external_nav_yaw_required is True
    assert prepare.external_nav_yaw_status_topics == ("/external_nav/status", "/navlab/slam/status")
    assert "external_nav_yaw_ready" in prepare.external_nav_yaw_ready_fields
    assert "ready" in prepare.external_nav_yaw_ready_fields
    assert "/ap/v1/status" not in prepare.required_upstream_topics
    assert "/mavros/state" not in prepare.required_upstream_topics


def test_real_prepare_fcu_bridge_registry_selects_navlab_mavlink_topics() -> None:
    from src.tasks.fcu_bridge import get_fcu_bridge_mode

    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    mode = get_fcu_bridge_mode(config.orchestration.real_prepare.fcu_bridge_mode)
    services = real_prepare._prepare_services(config)
    required_topics = real_prepare._required_upstream_topics("hover", config)

    assert mode.name == "navlab_mavlink"
    assert set(services) == {"mavlink_router", "navlab_mavlink_bridge", "lidar", "slam"}
    assert "/navlab/mavlink/status" in required_topics
    assert "/navlab/fcu/local_position_pose" in required_topics
    assert "/mavlink_external_nav/status" in required_topics
    assert "/imu/data" in required_topics
    assert "/imu" in required_topics
    assert "/external_nav/status" in required_topics
    assert "/mavros/state" not in required_topics
    assert not any(topic.startswith("/ap/v1/") for topic in required_topics)


def test_real_prepare_ros_services_source_local_install(monkeypatch, tmp_path: Path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    service = config.orchestration.real_prepare.slam
    existing = {Path("/opt/ros/humble/setup.bash"), Path("install/setup.bash")}

    monkeypatch.setattr(real_prepare.Path, "exists", lambda self: self in existing)

    spec = real_prepare._service_spec("slam", service, config=config, log_dir=tmp_path)

    assert spec.command[:2] == ("bash", "-lc")
    assert "source /opt/ros/humble/setup.bash" in spec.command[2]
    assert "source install/setup.bash" in spec.command[2]
    assert "ros2 launch navlab_slam_bringup" in spec.command[2]


def test_real_prepare_keeps_non_ros_service_command(tmp_path: Path) -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    service = config.orchestration.real_prepare.mavlink_router

    spec = real_prepare._service_spec("mavlink_router", service, config=config, log_dir=tmp_path)

    assert spec.command == service.command


def test_real_prepare_serial_provenance_requires_router_command_serial() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")

    provenance = real_prepare._serial_provenance(config)

    assert provenance["ok"] is True
    assert provenance["serial"] == "/dev/ttyUSB1"
    assert provenance["command_has_serial"] is True


def test_real_prepare_blocks_mavros_direct_serial_access(tmp_path: Path) -> None:
    task_config = tmp_path / "real_prepare.toml"
    task_config.write_text(
        """
[real_prepare]
mavlink_router_serial_port = "/dev/ttyUSB1"

[real_prepare.mavros]
command = ["ros2", "launch", "mavros", "apm.launch.py", "fcu_url:=/dev/ttyUSB1:115200"]
direct_serial_access_allowed = false
""".strip(),
        encoding="utf-8",
    )
    config = RunConfig.from_config(
        config_path="orchestration/config.real.toml",
        task_name="real-prepare",
        task_config_path=task_config,
    )

    services = real_prepare._prepare_services(config)
    services["mavros"] = config.orchestration.real_prepare.mavros
    blockers = real_prepare._validate_prepare_services(config, services)

    assert "prepare_service_direct_fcu_serial_forbidden:mavros:/dev/ttyUSB1" in blockers


def test_real_prepare_blocks_simulation_service_tokens(tmp_path: Path) -> None:
    task_config = tmp_path / "real_prepare.toml"
    task_config.write_text(
        """
[real_prepare.lidar]
command = ["ros2", "launch", "gazebo_sensor", "scan.launch.py"]
""".strip(),
        encoding="utf-8",
    )
    config = RunConfig.from_config(
        config_path="orchestration/config.real.toml",
        task_name="real-prepare",
        task_config_path=task_config,
    )

    blockers = real_prepare._validate_prepare_services(config, real_prepare._prepare_services(config))

    assert "prepare_service_uses_simulation_token:lidar:gazebo" in blockers


def test_real_prepare_summary_dry_run_starts_only_auxiliary_services(
    monkeypatch,  # noqa: ANN001
    tmp_path: Path,
) -> None:
    task_config = tmp_path / "real_prepare.toml"
    task_config.write_text("[real_prepare]\ndry_run = true\n", encoding="utf-8")
    config = RunConfig.from_config(
        config_path="orchestration/config.real.toml",
        task_name="real-prepare",
        task_config_path=task_config,
        run_id="20260609_000000",
        artifact_dir=tmp_path,
    )
    monkeypatch.setattr(
        real_prepare,
        "collect_ros_topic_snapshot",
        lambda **_kwargs: _complete_hover_snapshot(),
    )
    handles = []

    summary = real_prepare._build_real_prepare_summary(
        config,
        task_name="hover",
        backend=ProcessBackend(default_log_dir=tmp_path / "logs", dry_run=True),
        started_handles=handles,
        artifact_dir=tmp_path,
        log_dir=tmp_path / "logs",
    )

    assert summary["ok"] is True
    assert summary["prepare_claim"] == "evaluated"
    assert summary["companion_claim"] == "not_started"
    assert "companion" not in {service["name"] for service in summary["started_services"]}
    assert summary["mavlink_router"]["serial_provenance"]["ok"] is True


def test_task_doctor_helper_blocks_missing_stale_wrong_type_and_forbidden_topics() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")

    missing_scan = RealTopicSnapshot(
        topics={k: v for k, v in _complete_hover_snapshot().topics.items() if k != "/scan"}
    )
    result = real_prepare.check_real_task_upstream_topics("hover", config, topic_snapshot=missing_scan)
    assert "required_topic_missing:/scan" in result["blockers"]

    stale_scan_topics = dict(_complete_hover_snapshot().topics)
    stale_scan_topics["/scan"] = TopicEvidence(type_name="sensor_msgs/msg/LaserScan", fresh=False)
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=stale_scan_topics),
    )
    assert "required_topic_stale:/scan" in result["blockers"]

    wrong_type_topics = dict(_complete_hover_snapshot().topics)
    wrong_type_topics["/scan"] = TopicEvidence(type_name="sensor_msgs/msg/PointCloud2")
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=wrong_type_topics),
    )
    assert "required_topic_type_mismatch:/scan:sensor_msgs/msg/PointCloud2!=sensor_msgs/msg/LaserScan" in result[
        "blockers"
    ]

    forbidden_topics = dict(_complete_hover_snapshot().topics)
    forbidden_topics["/sim/x2/status"] = TopicEvidence()
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=forbidden_topics),
    )
    assert "forbidden_simulation_topic_present:/sim/x2/status" in result["blockers"]


def test_task_doctor_requires_external_nav_yaw_ready() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")
    topics = dict(_complete_hover_snapshot().topics)
    topics["/external_nav/status"] = TopicEvidence(metadata={"external_nav_yaw_ready": False})
    topics["/navlab/slam/status"] = TopicEvidence(metadata={"yaw_ready": False})

    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=topics),
    )

    assert "external_nav_yaw_not_ready" in result["blockers"]
    assert result["yaw_source"]["accepted_source"] == ""


def test_external_nav_yaw_metadata_parses_std_msgs_json_payload() -> None:
    payload = """data: '{"state":"healthy","ready":true,"odom":{"rate_ok":true}}'\n---\n"""

    assert real_prepare._std_msgs_string_payload(payload) == '{"state":"healthy","ready":true,"odom":{"rate_ok":true}}'
    assert real_prepare._metadata_bool({"ready": True}.get("ready")) is True


def test_task_doctor_external_nav_yaw_ready_ignores_uncalibrated_compass() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")
    topics = dict(_complete_hover_snapshot().topics)
    topics["/external_nav/status"] = TopicEvidence(
        metadata={
            "external_nav_yaw_ready": True,
            "compass_calibrated": False,
        }
    )

    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=topics),
    )

    assert result["yaw_source"]["ok"] is True
    assert result["yaw_source"]["accepted_source"] == "external_nav_yaw_ready"
    assert "external_nav_yaw_not_ready" not in result["blockers"]


def test_task_doctor_does_not_accept_compass_or_manual_override_as_yaw_source() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")
    topics = dict(_complete_hover_snapshot().topics)
    topics["/external_nav/status"] = TopicEvidence(
        metadata={
            "external_nav_yaw_ready": False,
            "compass_calibrated": True,
            "manual_override_acknowledged": True,
        }
    )
    topics["/navlab/slam/status"] = TopicEvidence(metadata={"external_nav_yaw_ready": False})

    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=topics),
    )

    assert "external_nav_yaw_not_ready" in result["blockers"]
    assert result["yaw_source"]["note"].startswith("indoor real tasks require ExternalNav")


def test_hover_task_doctor_passes_without_arm_takeoff_claims() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")

    summary = real_prepare.build_real_task_doctor_summary(
        "hover",
        config,
        topic_snapshot=_complete_hover_snapshot(),
    )

    assert summary["ok"] is True
    assert summary["arm_claim"] == "not_evaluated"
    assert summary["takeoff_claim"] == "not_evaluated"
    assert summary["task_specific"]["landing_policy"] == "land_in_place"
    assert summary["upstream"]["yaw_source"]["accepted_source"] == "external_nav_yaw_ready"
