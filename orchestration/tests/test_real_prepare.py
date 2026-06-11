from __future__ import annotations

from pathlib import Path

from rich.console import Console
from src.configs.run_config import RunConfig
from src.runtime.process_backend import ProcessBackend
from src.workflows.real import common_doctor as real_common_doctor
from src.workflows.real import prepare as real_prepare
from src.workflows.real import task_doctor as real_task_doctor
from src.workflows.real.prepare import RealTopicSnapshot, TopicEvidence


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
            "/slam/odom": TopicEvidence(type_name="nav_msgs/msg/Odometry", frame_id="odom"),
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
            "/navlab/mavlink/status": TopicEvidence(type_name="std_msgs/msg/String", metadata={"mode": "STABILIZE"}),
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
            "/rangefinder/down/range": TopicEvidence(
                type_name="sensor_msgs/msg/Range",
                frame_id="rangefinder_down_frame",
                metadata={"sample_seen": True, "range": 0.45, "min_range": 0.1, "max_range": 12.0},
            ),
            "/rangefinder/down/status": TopicEvidence(
                type_name="std_msgs/msg/String",
                metadata={
                    "ready": True,
                    "source": "real_fcu_distance_sensor",
                    "current_distance_m": 0.45,
                    "min_distance_m": 0.1,
                    "max_distance_m": 12.0,
                    "orientation": 25,
                    "fresh": True,
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
    assert "navlab.real.companion.nodes.mavlink_bridge" in prepare.navlab_mavlink_bridge.command
    assert "--auto-ekf-source-set" in prepare.navlab_mavlink_bridge.command
    assert "/navlab/mavlink/status" in prepare.navlab_mavlink_bridge.health_topics
    assert "/imu/data" in prepare.navlab_mavlink_bridge.health_topics
    assert "/imu/status" in prepare.navlab_mavlink_bridge.health_topics
    assert prepare.mavros.enabled is False
    assert prepare.mavros.command[3] == "apm.launch"
    assert prepare.mavros.command[-1] == "fcu_url:=udp://@127.0.0.1:14550"
    assert prepare.lidar.health_topics == ("/scan",)
    assert "navlab.real.companion.nodes.ydlidar_x2_scan" in " ".join(prepare.lidar.command)
    assert "launch_cartographer_backend:=true" in prepare.slam.command
    assert "publish_placeholder_odom:=false" in prepare.slam.command
    assert "cartographer_odometry_topic:=/odometry" in prepare.slam.command
    assert "scan_topic:=/scan" in prepare.slam.command
    assert "imu_topic:=/imu" in prepare.slam.command
    assert "odom_topic:=/slam/odom" in prepare.slam.command
    assert "external_nav_input_odom_topic:=/slam/odom" in prepare.slam.command
    assert "require_imu_for_external_nav:=false" in prepare.slam.command
    assert prepare.slam.health_topics == ("/imu", "/slam/odom", "/navlab/slam/status", "/external_nav/status")
    assert prepare.rangefinder_bridge.enabled is True
    assert prepare.rangefinder_bridge.required is True
    assert "navlab.real.companion.nodes.rangefinder_bridge" in " ".join(prepare.rangefinder_bridge.command)
    assert prepare.rangefinder_bridge.health_topics == ("/rangefinder/down/range", "/rangefinder/down/status")
    assert prepare.height_rangefinder_required is True
    assert prepare.altitude_hold_mode == "fcu_rangefinder_guided"
    assert prepare.altitude_hold_requires_rangefinder is True
    assert prepare.altitude_hold_allows_indoor_no_gps is True
    assert prepare.altitude_hold_allowed_initial_modes == ("STABILIZE", "ALT_HOLD", "GUIDED")
    assert prepare.external_nav_yaw_required is True
    assert prepare.external_nav_yaw_status_topics == ("/external_nav/status", "/navlab/slam/status")
    assert "external_nav_yaw_ready" in prepare.external_nav_yaw_ready_fields
    assert "ready" in prepare.external_nav_yaw_ready_fields
    assert "/ap/v1/status" not in prepare.required_upstream_topics
    assert "/mavros/state" not in prepare.required_upstream_topics


def test_real_prepare_topic_graph_probe_uses_configured_timeout(monkeypatch) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    timeouts: list[float] = []

    def fake_collect_ros_topic_snapshot(*, timeout_sec: float, probe_topics=()):  # noqa: ANN001
        timeouts.append(timeout_sec)
        return _complete_hover_snapshot()

    monkeypatch.setattr(real_prepare, "collect_ros_topic_snapshot", fake_collect_ros_topic_snapshot)
    monkeypatch.setattr(real_prepare, "_probe_mavlink_router_endpoint", lambda _config: {"ok": True})

    result = real_prepare.build_real_prepare_summary(
        config,
        task_name="hover",
        backend=ProcessBackend(default_log_dir=config.artifact_dir / "logs", dry_run=True),
        started_handles=[],
        artifact_dir=config.artifact_dir,
        log_dir=config.artifact_dir / "logs",
    )

    assert result["ok"] is True
    assert timeouts
    assert min(timeouts) == config.orchestration.real_prepare.ros_topic_probe_timeout_sec


def test_real_common_doctor_topic_graph_probe_uses_configured_timeout(monkeypatch) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    timeouts: list[float] = []

    def fake_collect_ros_topic_snapshot(*, timeout_sec: float, probe_topics=()):  # noqa: ANN001
        timeouts.append(timeout_sec)
        return _complete_hover_snapshot()

    monkeypatch.setattr(real_common_doctor, "collect_ros_topic_snapshot", fake_collect_ros_topic_snapshot)

    snapshot = real_common_doctor.wait_for_common_doctor_topic_snapshot(config, task_name="hover")

    assert snapshot.topics
    assert timeouts
    assert set(timeouts) == {config.orchestration.real_prepare.ros_topic_probe_timeout_sec}


def test_real_prepare_fcu_bridge_registry_selects_navlab_mavlink_topics() -> None:
    from src.tasks.fcu_bridge import get_fcu_bridge_mode

    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    mode = get_fcu_bridge_mode(config.orchestration.real_prepare.fcu_bridge_mode)
    services = real_prepare.prepare_services(config)
    required_topics = real_prepare.required_upstream_topics("hover", config)

    assert mode.name == "navlab_mavlink"
    assert set(services) == {"mavlink_router", "navlab_mavlink_bridge", "lidar", "slam", "rangefinder_bridge"}
    assert "/navlab/mavlink/status" in required_topics
    assert "/navlab/fcu/local_position_pose" in required_topics
    assert "/mavlink_external_nav/status" in required_topics
    assert "/rangefinder/down/range" in required_topics
    assert "/rangefinder/down/status" in required_topics
    assert "/imu/data" in required_topics
    assert "/imu" in required_topics
    assert "/external_nav/status" in required_topics
    assert "/mavros/state" not in required_topics
    assert not any(topic.startswith("/ap/v1/") for topic in required_topics)


def test_real_prepare_motor_debug_skips_rangefinder_contract() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    services = real_prepare.prepare_services(config, task_name="motor-debug")
    required_topics = real_prepare.required_upstream_topics("motor-debug", config)

    assert set(services) == {"mavlink_router", "navlab_mavlink_bridge", "lidar", "slam"}
    assert "/rangefinder/down/range" not in required_topics
    assert "/rangefinder/down/status" not in required_topics
    assert "/navlab/mavlink/status" in required_topics
    assert "/external_nav/status" in required_topics


def test_real_prepare_ros_services_source_local_install(monkeypatch, tmp_path: Path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    service = config.orchestration.real_prepare.slam
    existing = {Path("/opt/ros/humble/setup.bash"), Path("install/setup.bash")}

    monkeypatch.setattr(real_prepare.Path, "exists", lambda self: self in existing)

    spec = real_prepare.service_spec("slam", service, config=config, log_dir=tmp_path)

    assert spec.command[:2] == ("bash", "-lc")
    assert "source /opt/ros/humble/setup.bash" in spec.command[2]
    assert "source install/setup.bash" in spec.command[2]
    assert "ros2 launch navlab_slam_bringup" in spec.command[2]


def test_real_prepare_keeps_non_ros_service_command(tmp_path: Path) -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    service = config.orchestration.real_prepare.mavlink_router

    spec = real_prepare.service_spec("mavlink_router", service, config=config, log_dir=tmp_path)

    assert spec.command == service.command


def test_real_prepare_serial_provenance_requires_router_command_serial() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")

    provenance = real_prepare.serial_provenance(config)

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

    services = real_prepare.prepare_services(config)
    services["mavros"] = config.orchestration.real_prepare.mavros
    blockers = real_prepare.validate_prepare_services(config, services)

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

    blockers = real_prepare.validate_prepare_services(config, real_prepare.prepare_services(config))

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

    summary = real_prepare.build_real_prepare_summary(
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


def test_task_doctor_helper_checks_scan_and_slam_odom_frames() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")

    wrong_scan_frame = dict(_complete_hover_snapshot().topics)
    wrong_scan_frame["/scan"] = TopicEvidence(
        type_name="sensor_msgs/msg/LaserScan",
        frame_id="base_scan",
        metadata={"sample_seen": True, "range_count": 360},
    )
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=wrong_scan_frame),
    )
    assert "required_topic_frame_mismatch:/scan:base_scan!=laser_frame" in result["blockers"]
    assert result["required_topics"]["/scan"]["expected_frame_id"] == "laser_frame"

    wrong_odom_frame = dict(_complete_hover_snapshot().topics)
    wrong_odom_frame["/slam/odom"] = TopicEvidence(type_name="nav_msgs/msg/Odometry", frame_id="map")
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=wrong_odom_frame),
    )
    assert "required_topic_frame_mismatch:/slam/odom:map!=odom" in result["blockers"]
    assert result["required_topics"]["/slam/odom"]["expected_frame_id"] == "odom"


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


def test_task_doctor_requires_real_rangefinder_contract() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")

    missing_topics = {
        k: v for k, v in _complete_hover_snapshot().topics.items() if not k.startswith("/rangefinder/down/")
    }
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=missing_topics),
    )
    assert "required_topic_missing:/rangefinder/down/range" in result["blockers"]
    assert "required_topic_missing:/rangefinder/down/status" in result["blockers"]
    assert "rangefinder_down_no_data" in result["blockers"]

    status_not_ready = dict(_complete_hover_snapshot().topics)
    status_not_ready["/rangefinder/down/status"] = TopicEvidence(
        type_name="std_msgs/msg/String",
        metadata={"ready": False, "blocker": "rangefinder_down_distance_invalid"},
    )
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=status_not_ready),
    )
    assert "rangefinder_down_distance_invalid" in result["blockers"]

    wrong_source = dict(_complete_hover_snapshot().topics)
    wrong_source["/rangefinder/down/status"] = TopicEvidence(
        type_name="std_msgs/msg/String",
        metadata={
            "ready": True,
            "source": "simulation_rangefinder",
            "current_distance_m": 0.45,
            "min_distance_m": 0.1,
            "max_distance_m": 12.0,
            "orientation": 25,
        },
    )
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=wrong_source),
    )
    assert "rangefinder_down_source_forbidden" in result["blockers"]

    wrong_orientation = dict(_complete_hover_snapshot().topics)
    wrong_orientation["/rangefinder/down/status"] = TopicEvidence(
        type_name="std_msgs/msg/String",
        metadata={
            "ready": True,
            "source": "real_fcu_distance_sensor",
            "current_distance_m": 0.45,
            "min_distance_m": 0.1,
            "max_distance_m": 12.0,
            "orientation": 0,
        },
    )
    result = real_prepare.check_real_task_upstream_topics(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=wrong_orientation),
    )
    assert "rangefinder_down_orientation_invalid" in result["blockers"]


def test_task_doctor_blocks_altitude_hold_when_rangefinder_not_ready() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")
    topics = dict(_complete_hover_snapshot().topics)
    topics["/rangefinder/down/range"] = TopicEvidence(
        type_name="sensor_msgs/msg/Range",
        metadata={"sample_seen": True, "range": 0.0, "min_range": 0.1, "max_range": 12.0},
    )
    topics["/rangefinder/down/status"] = TopicEvidence(
        type_name="std_msgs/msg/String",
        metadata={
            "ready": False,
            "source": "real_fcu_distance_sensor",
            "current_distance_m": 0.0,
            "min_distance_m": 0.1,
            "max_distance_m": 12.0,
            "orientation": 25,
            "blocker": "rangefinder_down_distance_invalid",
        },
    )

    summary = real_task_doctor.build_real_task_doctor_summary(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=topics),
    )

    assert summary["ok"] is False
    assert "rangefinder_down_distance_invalid" in summary["blockers"]
    assert "altitude_hold_mode_not_ready" in summary["blockers"]
    assert summary["task_specific"]["altitude_hold"]["rangefinder_ready"] is False


def test_task_doctor_blocks_altitude_hold_on_disallowed_initial_fcu_mode() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="hover")
    topics = dict(_complete_hover_snapshot().topics)
    topics["/navlab/mavlink/status"] = TopicEvidence(type_name="std_msgs/msg/String", metadata={"mode": "AUTO"})

    summary = real_task_doctor.build_real_task_doctor_summary(
        "hover",
        config,
        topic_snapshot=RealTopicSnapshot(topics=topics),
    )

    assert summary["ok"] is False
    assert "altitude_hold_mode_not_ready" in summary["blockers"]
    assert summary["task_specific"]["altitude_hold"]["current_fcu_mode"] == "AUTO"


def test_common_real_doctor_uses_shared_fcu_external_nav_state() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="doctor")
    topics = dict(_complete_hover_snapshot().topics)
    topics["/navlab/mavlink/status"] = TopicEvidence(
        type_name="std_msgs/msg/String",
        metadata={
            "mode": "STABILIZE",
            "armed": False,
            "active_source_set": "SRC2",
            "GPS_TYPE": 0,
            "GPS1_TYPE": 0,
            "VISO_TYPE": 1,
            "EK3_SRC1_POSXY": 6,
            "EK3_SRC1_VELXY": 6,
            "EK3_SRC1_YAW": 6,
            "EK3_SRC1_POSZ": 1,
            "EK3_SRC2_POSXY": 6,
            "EK3_SRC2_VELXY": 6,
            "EK3_SRC2_YAW": 6,
            "EK3_SRC2_POSZ": 1,
            "external_nav_seen_by_fcu": True,
            "local_position_valid": True,
            "ekf_origin_set": True,
        },
    )
    topics["/mavlink_external_nav/status"] = TopicEvidence(type_name="std_msgs/msg/String", metadata={"ready": True})
    topics["/external_nav/status"] = TopicEvidence(type_name="std_msgs/msg/String", metadata={"ready": True})

    summary = real_common_doctor.build_real_common_doctor_summary(
        config,
        topic_snapshot=RealTopicSnapshot(topics=topics),
    )

    assert summary["ok"] is True
    assert summary["task_name"] == "doctor"
    assert summary["common_state"]["configured_external_nav_source_set"] == "SRC2"
    assert summary["common_state"]["observed_ekf_source_set"] == "not_observed"
    assert summary["common_state"]["external_nav_ros_ready"] is True
    assert "external_nav_seen_by_fcu" not in summary["common_state"]
    assert "rc_input" not in summary["common_state"]
    assert "ekf_origin_home" not in summary["common_state"]
    assert "landing_policy" not in summary["common_state"]
    assert not any("unsupported_real_task" in blocker for blocker in summary["blockers"])


def test_common_real_doctor_reads_navlab_mavlink_status_parameters() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="doctor")
    topics = dict(_complete_hover_snapshot().topics)
    topics["/navlab/mavlink/status"] = TopicEvidence(
        type_name="std_msgs/msg/String",
        metadata={
            "mode_number": 0,
            "armed": False,
            "local_position_valid": True,
            "external_nav_seen_by_fcu": True,
            "parameters": {
                "GPS_TYPE": 0,
                "GPS1_TYPE": 0,
                "VISO_TYPE": 1,
                "EK3_SRC1_POSXY": 6,
                "EK3_SRC1_VELXY": 6,
                "EK3_SRC1_YAW": 6,
                "EK3_SRC1_POSZ": 1,
                "EK3_SRC2_POSXY": 0,
                "EK3_SRC2_VELXY": 0,
                "EK3_SRC2_YAW": 0,
                "EK3_SRC2_POSZ": 1,
            },
            "active_source_set": "SRC1",
        },
    )
    topics["/mavlink_external_nav/status"] = TopicEvidence(type_name="std_msgs/msg/String", metadata={"ready": True})
    topics["/external_nav/status"] = TopicEvidence(type_name="std_msgs/msg/String", metadata={"ready": True})

    summary = real_common_doctor.build_real_common_doctor_summary(
        config,
        topic_snapshot=RealTopicSnapshot(topics=topics),
    )

    assert summary["ok"] is True
    assert summary["common_state"]["mode"] == "STABILIZE"
    assert summary["common_state"]["gps_type"] == "0"
    assert summary["common_state"]["gps1_type"] == "0"
    assert summary["common_state"]["viso_type"] == "1"
    assert summary["common_state"]["ek3_src1"]["posxy"] == "6"
    assert summary["common_state"]["ek3_src1"]["velxy"] == "6"
    assert summary["common_state"]["ek3_src1"]["yaw"] == "6"
    assert summary["common_state"]["configured_external_nav_source_set"] == "SRC1"


def test_common_real_doctor_does_not_block_on_external_nav_fcu_inference() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="doctor")
    topics = dict(_complete_hover_snapshot().topics)
    topics["/navlab/mavlink/status"] = TopicEvidence(
        type_name="std_msgs/msg/String",
        metadata={
            "mode": "STABILIZE",
            "armed": False,
            "configured_external_nav_source_set": "SRC2",
            "observed_ekf_source_set": "not_observed",
            "GPS_TYPE": 0,
            "GPS1_TYPE": 0,
            "VISO_TYPE": 1,
            "EK3_SRC1_POSXY": 6,
            "EK3_SRC1_VELXY": 6,
            "EK3_SRC1_YAW": 6,
            "EK3_SRC2_POSXY": 6,
            "EK3_SRC2_VELXY": 6,
            "EK3_SRC2_YAW": 6,
            "local_position_valid": False,
            "ekf_origin_set": False,
        },
    )
    topics["/mavlink_external_nav/status"] = TopicEvidence(type_name="std_msgs/msg/String", metadata={"ready": True})
    topics["/external_nav/status"] = TopicEvidence(type_name="std_msgs/msg/String", metadata={"ready": True})

    summary = real_common_doctor.build_real_common_doctor_summary(
        config,
        topic_snapshot=RealTopicSnapshot(topics=topics),
    )

    assert summary["ok"] is True
    assert "ekf_source_requires_externalnav:SRC2" not in summary["blockers"]
    assert "external_nav_not_seen_by_fcu" not in summary["blockers"]


def test_real_prepare_and_common_doctor_panels_do_not_use_nested_tables(tmp_path: Path) -> None:
    console = Console(record=True, width=78)
    prepare_summary = {
        "ok": True,
        "task_name": "doctor",
        "dry_run": False,
        "fcu_bridge_mode": {"name": "navlab_mavlink"},
        "mavlink_router": {"serial": "/dev/ttyUSB1", "local_endpoint": "127.0.0.1:14550"},
        "service_count": 5,
        "blockers": [],
    }
    common_summary = {
        "ok": True,
        "task_name": "doctor",
        "common_state": {
            "mode": "STABILIZE",
            "armed": False,
            "gps_type": "0",
            "gps1_type": "0",
            "viso_type": "1",
            "ek3_src1": {"posxy": "6", "velxy": "6", "yaw": "6", "posz": "1"},
            "ek3_src2": {"posxy": "6", "velxy": "6", "yaw": "6", "posz": "1"},
            "active_source_set": "not_observed",
            "configured_external_nav_source_set": "SRC2",
            "observed_ekf_source_set": "not_observed",
            "external_nav_ros_ready": True,
            "local_position_valid": "unknown",
        },
        "blockers": [],
    }
    task_summary = {
        "ok": True,
        "task_name": "doctor",
        "arm_claim": "not_evaluated",
        "takeoff_claim": "not_evaluated",
        "blockers": [],
    }

    real_prepare.print_real_prepare_summary(console, summary=prepare_summary, summary_path=tmp_path / "summary.json")
    real_common_doctor.print_real_common_doctor_summary(console, summary=common_summary, summary_path=tmp_path / "summary.json")
    real_task_doctor.print_real_task_doctor_summary(console, summary=task_summary, summary_path=tmp_path / "summary.json")
    output = console.export_text()

    assert "┏" not in output
    assert "Summary" in output
    assert "…" not in output
    assert "summary.json" in output
    assert "Armed" not in output
    assert "ExternalNav FCU" not in output
    assert "EKF origin/home" not in output
    assert "RC input" not in output


def test_motor_debug_task_doctor_panel_shows_guided_gate_without_nested_tables(tmp_path: Path) -> None:
    console = Console(record=True, width=90)
    summary = {
        "ok": True,
        "task_name": "motor-debug",
        "arm_claim": "not_evaluated",
        "takeoff_claim": "not_evaluated",
        "task_specific": {
            "required_mode": "GUIDED",
            "current_fcu_mode": "STABILIZE",
            "guided_gate": "run_stage",
            "mode_switch_claim": "deferred_to_motor_debug_run",
        },
        "blockers": [],
    }

    real_task_doctor.print_real_task_doctor_summary(console, summary=summary, summary_path=tmp_path / "summary.json")
    output = console.export_text()

    assert "NavLab Real Motor Debug Doctor" in output
    assert "Required mode" in output
    assert "Current mode" in output
    assert "Guided gate" in output
    assert "Mode switch" in output
    assert "motor_debug_guided_mode_not_confirmed" not in output
    assert "┏" not in output
    assert "┃" not in output


def test_motor_debug_task_doctor_does_not_require_current_guided_mode(monkeypatch, tmp_path: Path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    upstream = {
        "blockers": [],
        "required_topics": {
            config.orchestration.fcu_controller.status_topic: {
                "metadata": {
                    "mode": "STABILIZE",
                    "GPS_TYPE": "0",
                    "GPS1_TYPE": "0",
                    "VISO_TYPE": "1",
                    "EK3_SRC1_POSXY": "6",
                    "EK3_SRC1_VELXY": "6",
                    "EK3_SRC1_YAW": "6",
                    "EK3_SRC1_POSZ": "1",
                    "active_source_set": "SRC1",
                    "external_nav_seen_by_fcu": True,
                    "local_position_valid": True,
                    "ekf_origin_home": True,
                    "armed": False,
                }
            },
            config.orchestration.slam_backend.external_nav_status_topic: {
                "metadata": {"ready": True}
            },
        }
    }

    monkeypatch.setattr(real_task_doctor, "check_real_task_upstream_topics", lambda *_args, **_kwargs: upstream)

    summary = real_task_doctor.build_real_task_doctor_summary("motor-debug", config)

    assert summary["ok"] is True
    assert "motor_debug_guided_mode_not_confirmed" not in summary["blockers"]
    assert summary["task_specific"]["current_fcu_mode"] == "STABILIZE"
    assert summary["task_specific"]["guided_gate"] == "run_stage"


def test_external_nav_yaw_metadata_parses_std_msgs_json_payload() -> None:
    payload = """data: '{"state":"healthy","ready":true,"odom":{"rate_ok":true}}'\n---\n"""

    assert real_prepare.std_msgs_string_payload(payload) == '{"state":"healthy","ready":true,"odom":{"rate_ok":true}}'
    assert real_prepare.metadata_bool({"ready": True}.get("ready")) is True


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

    summary = real_task_doctor.build_real_task_doctor_summary(
        "hover",
        config,
        topic_snapshot=_complete_hover_snapshot(),
    )

    assert summary["ok"] is True
    assert summary["arm_claim"] == "not_evaluated"
    assert summary["takeoff_claim"] == "not_evaluated"
    assert summary["task_specific"]["landing_policy"] == "land_in_place"
    assert summary["upstream"]["yaw_source"]["accepted_source"] == "external_nav_yaw_ready"


def test_real_task_doctor_skips_task_specific_when_task_has_no_hook() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="doctor")

    summary = real_task_doctor.build_real_task_doctor_summary(
        "doctor",
        config,
        topic_snapshot=_complete_hover_snapshot(),
    )

    assert summary["ok"] is True
    assert summary["task_specific"]["skipped"] is True
    assert summary["task_specific"]["reason"].startswith("task_not_registered:")
    assert not any("unsupported_real_task" in blocker for blocker in summary["blockers"])
