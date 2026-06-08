from __future__ import annotations

import io
import json
import os
import subprocess
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from rich.console import Console
from src import host
from src.config import (
    NAVLAB_SERVICES,
    NAVLAB_STOP_SERVICES,
    OrchestrationConfig,
    RunConfig,
)
from src.project_config import load_navlab_images_config, load_runtime_config, resolve_navlab_image_tag
from src.runtime import ServiceSpec
from src.tasks import build as build_task_module
from src.tasks.workflows import exploration as exploration_gate_task_module
from src.tasks.helpers import fcu as fcu_controller_task_module
from src.tasks.helpers import frame_contract as frame_contract_task_module
from src.tasks import hover as hover_task_module
from src.tasks.helpers import motion as motion_gate_task_module
from src.tasks.helpers import official_stack as official_baseline_task_module
from src.tasks.helpers import navlab_models as official_maze_x2_task_module
from src.tasks.helpers import sensors as rangefinder_imu_task_module
from src.tasks.helpers import scan_integrity as scan_integrity_gate_task_module
from src.tasks.helpers import scan_stabilization as scan_stabilization_gate_task_module
from src.tasks.helpers import slam as slam_backend_task_module
from src.tasks.helpers import slam_hover as slam_hover_task_module
from src.tasks.base import OrchestrationTask
from src.tasks.build import BuildTask
from src.tasks.hover import HoverAcceptanceTask
from src.tasks.helpers.official_stack import run_official_baseline_acceptance, run_official_baseline_doctor
from src.tasks.registry import TaskRegistry

from navlab.companion.config import (
    ExternalNavSenderConfig,
    GazeboTruthBridgeConfig,
    GazeboTruthOdomConfig,
    ImuBridgeConfig,
    MissionNodeConfig,
    PoseMirrorConfig,
    RuntimeConfig,
    ScanFeaturesConfig,
    WorldMarkersConfig,
)
from navlab.companion.runtime import CompanionLauncher


def test_navlab_runtime_config_loads_companion_nodes_from_toml() -> None:
    config = RuntimeConfig.load("navlab/config.toml")
    world_argv = config.world_markers.argv()
    pose_argv = config.pose_mirror.argv()
    odom_argv = config.gazebo_truth_odom.argv()
    external_nav_argv = config.external_nav_sender.argv()
    mission_argv = config.mission.argv()

    assert config.imu_source_label == "fcu_mavlink_navlab"
    assert config.world_markers.autostart is True
    assert "/sim/markers" in world_argv
    assert "--frame-id" in world_argv
    assert "navlab_world" in world_argv
    assert config.scan_features.autostart is True
    assert config.pose_mirror.autostart is True
    assert config.pose_mirror.endpoint == "tcp:mavlink-router:5760"
    assert "--pose-frame-id" in pose_argv
    assert "navlab_world" in pose_argv
    assert "--map-frame-id" in pose_argv
    assert "--odom-frame-id" in pose_argv
    assert "--sensor-base-frame-id" in pose_argv
    assert "--replay-base-frame-id" in pose_argv
    assert "--replay-base-parent-frame-id" in pose_argv
    assert "--laser-frame-id" in pose_argv
    replay_base_flag = pose_argv.index("--replay-base-frame-id")
    replay_parent_flag = pose_argv.index("--replay-base-parent-frame-id")
    assert pose_argv[replay_base_flag + 1] == "navlab_replay_base_link"
    assert pose_argv[replay_parent_flag + 1] == "navlab_world"
    assert "--simulate-pose-from-mission-status" not in pose_argv
    assert "--set-gazebo-pose" not in pose_argv
    assert "--world-name" not in pose_argv
    assert "--model-name" not in pose_argv
    assert config.gazebo_truth_bridge.autostart is True
    assert config.gazebo_truth_odom.autostart is True
    assert "/gazebo/truth/odom" in odom_argv
    index_flag = odom_argv.index("--transform-index")
    assert odom_argv[index_flag + 1] == "0"
    assert external_nav_argv == [
        "--endpoint",
        "tcp:127.0.0.1:5762",
        "--odom-topic",
        "/external_nav/odom",
        "--status-topic",
        "/mavlink_external_nav/status",
        "--rate-hz",
        "20.0",
        "--quality",
        "100",
        "--reset-counter",
        "0",
        "--source-system",
        "191",
        "--use-fcu-roll-pitch",
        "--local-position-pose-topic",
        "/navlab/fcu/local_position_pose",
    ]
    assert config.external_nav_sender.endpoint == "tcp:127.0.0.1:5762"
    assert config.mission.endpoint == "tcp:127.0.0.1:5763"
    assert "--simulate-mode-arm" not in mission_argv
    assert "--obstacle-detect-distance-m" in mission_argv
    assert "--obstacle-avoid-distance-m" in mission_argv
    assert "--pass-x-m" in mission_argv
    assert "--return-y-m" in mission_argv
    pass_x_flag = mission_argv.index("--pass-x-m")
    assert mission_argv[pass_x_flag + 1] == "1.25"


def test_navlab_compose_env_contains_only_compose_level_config() -> None:
    config = OrchestrationConfig.load()
    image_config = load_navlab_images_config(load_runtime_config())
    env = config.compose_env()

    assert "NAVLAB_CONFIG" not in env
    assert "NAVLAB_RUNTIME_CONFIG" not in env
    assert env["NAVLAB_COMPANION_IMAGE"] == image_config.companion.image()
    assert env["NAVLAB_SLAM_IMAGE"] == image_config.slam.image()
    assert env["NAVLAB_GAZEBO_SENSOR_IMAGE"] == image_config.gazebo_sensor.image()
    assert env["X2_MODE"] == "runtime"
    assert "X2_SCAN_SOURCE" not in env
    assert "X2_SCAN_IDEAL_TOPIC" not in env
    assert "X2_SCAN_TOPIC" not in env
    assert "X2_STATUS_TOPIC" not in env
    assert "X2_VIRTUAL_SERIAL_LINK" not in env
    assert env["SITL_IMAGE"] == "remote-sitl-lab/ardupilot-sitl:stage1-f10500ae45aa"
    assert env["SITL_MODEL"] == "JSON"
    assert env["GAZEBO_WORLD"] == "/workspace/worlds/navlab_iq_quad_figure8.sdf"
    assert "NAVLAB_POSE_MIRROR_EXTRA_ARGS" not in env
    assert "NAVLAB_MISSION_EXTRA_ARGS" not in env
    assert "SIM_UP_MODE" not in env
    assert "SIM_AUTO_ROSBAG_ENABLED" not in env
    assert config.foxglove_upload.enabled is True
    assert config.foxglove_upload.token_env == "FOXGLOVE_API_TOKEN"
    assert config.foxglove_upload.device_name == "navlab_companion_sitl_gazebo"
    assert config.slam.autostart is True
    assert config.slam.backend == "cartographer"
    assert config.slam.runtime_config == "/workspace/navlab/config.toml"
    assert config.sensor.scan_source == "x2_virtual_serial"
    assert config.sensor.acceptance_scan_source == "x2_virtual_serial_vendor_driver"
    assert config.official_baseline.rosbag_profile == "profiles/navlab-official-baseline-rosbag-topics.txt"
    assert config.official_baseline.dds_enable == "1"
    assert config.official_baseline.dds_domain_id == "0"
    assert config.official_baseline.rmw_implementation == "rmw_cyclonedds_cpp"
    assert config.official_baseline.expected_ap_node == "/ap"
    assert config.official_baseline.required_ap_topics == ("/ap/v1/time",)
    assert config.official_baseline.runtime_image == "world-model/navlab-official-baseline:latest"
    assert config.official_baseline.required_ros_packages == (
        "ardupilot_sitl",
        "ardupilot_msgs",
        "ardupilot_dds_tests",
        "micro_ros_agent",
        "ardupilot_gz_bringup",
        "ardupilot_gz_application",
        "ardupilot_gazebo",
        "ardupilot_gz_gazebo",
        "ardupilot_sitl_models",
        "ardupilot_cartographer",
    )
    assert config.official_baseline.micro_ros_agent_binaries == ("MicroXRCEAgent", "micro_ros_agent")
    assert config.official_baseline.sitl_launch == "ros2 launch ardupilot_sitl sitl_dds_udp.launch.py"
    assert config.official_baseline.gazebo_launch == "ros2 launch ardupilot_gz_bringup iris_maze.launch.py"
    assert config.official_baseline.cartographer_launch == "ros2 launch ardupilot_cartographer cartographer.launch.py"
    assert config.official_baseline.gazebo_bringup_mode == "official_gz_bringup"
    assert config.official_baseline.external_nav_route == "official_dds"
    assert config.official_maze_x2.rosbag_profile == "profiles/navlab-official-maze-x2-rosbag-topics.txt"
    assert config.official_maze_x2.world_source == "official_iris_maze"
    assert config.official_maze_x2.vehicle_model_source == "official_iris_with_lidar"
    assert config.official_maze_x2.gazebo_lidar_topic == "/lidar"
    assert config.official_maze_x2.x2_scan_input_topic == "/lidar"
    assert config.official_maze_x2.x2_scan_topic == "/scan"
    assert config.official_maze_x2.x2_status_topic == "/sim/x2/status"
    assert config.official_maze_x2.altitude_control_claim == "not_evaluated"
    assert config.official_maze_x2.hover_claim == "not_evaluated"
    assert config.rangefinder_imu.rosbag_profile == "profiles/navlab-rangefinder-imu-rosbag-topics.txt"
    assert config.rangefinder_imu.world_source == "official_iris_maze"
    assert config.rangefinder_imu.vehicle_model_source == "official_iris_with_lidar"
    assert config.rangefinder_imu.model_overlay_source == "official_iris_with_lidar_plus_down_rangefinder"
    assert config.rangefinder_imu.gazebo_lidar_topic == "/lidar"
    assert config.rangefinder_imu.x2_scan_input_topic == "/lidar"
    assert config.rangefinder_imu.x2_scan_topic == "/scan"
    assert config.rangefinder_imu.x2_status_topic == "/sim/x2/status"
    assert config.rangefinder_imu.rangefinder_scan_ideal_topic == "/rangefinder/down/scan_ideal"
    assert config.rangefinder_imu.rangefinder_range_topic == "/rangefinder/down/range"
    assert config.rangefinder_imu.rangefinder_status_topic == "/rangefinder/down/status"
    assert config.rangefinder_imu.rangefinder_frame_id == "rangefinder_down_frame"
    assert config.rangefinder_imu.rangefinder_endpoint == "udpin:0.0.0.0:14550"
    assert config.rangefinder_imu.rangefinder_fcu_probe_endpoint == "udpin:0.0.0.0:14551"
    assert config.rangefinder_imu.rangefinder_mavlink_orientation == "MAV_SENSOR_ROTATION_PITCH_270"
    assert config.rangefinder_imu.rangefinder_rate_hz == 20.0
    assert config.rangefinder_imu.rangefinder_min_distance_m == 0.05
    assert config.rangefinder_imu.rangefinder_max_distance_m == 6.0
    assert config.rangefinder_imu.imu_source_route == "official_gazebo_imu_bridge"
    assert config.rangefinder_imu.imu_output_topic == "/imu"
    assert config.rangefinder_imu.imu_frame_id == "imu_link"
    assert config.rangefinder_imu.synthetic_fallback_enabled is False
    assert config.rangefinder_imu.altitude_control_claim == "not_evaluated"
    assert config.rangefinder_imu.hover_claim == "not_evaluated"
    assert config.slam_backend.rosbag_profile == "profiles/navlab-slam-backend-rosbag-topics.txt"
    assert config.slam_backend.backend == "cartographer"
    assert config.slam_backend.launch_package == "navlab_slam_bringup"
    assert config.slam_backend.scan_topic == "/scan"
    assert config.slam_backend.x2_vendor_scan_topic == "/navlab/x2/vendor_scan"
    assert config.slam_backend.imu_topic == "/imu"
    assert config.slam_backend.odometry_topic == "/odometry"
    assert config.slam_backend.slam_odom_topic == "/slam/odom"
    assert config.slam_backend.slam_status_topic == "/navlab/slam/status"
    assert config.slam_backend.uses_gazebo_truth_as_input is False
    assert config.fcu_controller.rosbag_profile == "profiles/navlab-fcu-controller-rosbag-topics.txt"
    assert config.fcu_controller.control_route == "mavlink_bootstrap_plus_dds_cmd_vel"
    assert config.fcu_controller.mavlink_bootstrap_endpoint == "udp:127.0.0.1:14550"
    assert config.fcu_controller.mavlink_bootstrap_source_system == 246
    assert config.fcu_controller.mavlink_bootstrap_source_component == 190
    assert config.fcu_controller.owner_name == "navlab_fcu_controller"
    assert config.fcu_controller.owner_id == "navlab-p4-fcu-controller"
    assert config.fcu_controller.fcu_state_topic == "/navlab/fcu/state"
    assert config.fcu_controller.controller_status_topic == "/navlab/fcu/controller/status"
    assert config.fcu_controller.setpoint_intent_topic == "/navlab/fcu/setpoint/intent"
    assert config.fcu_controller.setpoint_output_topic == "/navlab/fcu/setpoint/output"
    assert config.fcu_controller.owner_status_topic == "/navlab/fcu/owner/status"
    assert config.fcu_controller.prearm_service == "/ap/v1/prearm_check"
    assert config.fcu_controller.mode_switch_service == "/ap/v1/mode_switch"
    assert config.fcu_controller.arm_service == "/ap/v1/arm_motors"
    assert config.fcu_controller.takeoff_service == "/ap/v1/experimental/takeoff"
    assert config.fcu_controller.cmd_vel_topic == "/ap/v1/cmd_vel"
    assert config.fcu_controller.guided_mode == 4
    assert config.fcu_controller.takeoff_alt_m == 0.5
    assert config.fcu_controller.require_slam_backend is True
    assert config.fcu_controller.hover_claim == "not_evaluated"
    assert config.fcu_controller.exploration_claim == "not_evaluated"
    assert config.frame_contract.rosbag_profile == "profiles/navlab-frame-contract-rosbag-topics.txt"
    assert config.frame_contract.required_frames == (
        "map",
        "odom",
        "base_link",
        "imu_link",
        "base_scan",
        "rangefinder_down_frame",
    )
    assert config.frame_contract.map_frame_id == "map"
    assert config.frame_contract.odom_frame_id == "odom"
    assert config.frame_contract.base_frame_id == "base_link"
    assert config.frame_contract.imu_frame_id == "imu_link"
    assert config.frame_contract.laser_frame_id == "base_scan"
    assert config.frame_contract.rangefinder_frame_id == "rangefinder_down_frame"
    assert config.frame_contract.scan_topic == "/scan"
    assert config.frame_contract.imu_topic == "/imu"
    assert config.frame_contract.rangefinder_range_topic == "/rangefinder/down/range"
    assert config.frame_contract.rangefinder_status_topic == "/rangefinder/down/status"
    assert config.frame_contract.fcu_pose_topic == "/ap/v1/pose/filtered"
    assert config.frame_contract.fcu_twist_topic == "/ap/v1/twist/filtered"
    assert config.frame_contract.cmd_vel_topic == "/ap/v1/cmd_vel"
    assert config.frame_contract.slam_odom_topic == "/slam/odom"
    assert config.frame_contract.truth_diagnostic_topic == "/odometry"
    assert config.frame_contract.status_topic == "/navlab/frame_contract/status"
    assert config.frame_contract.require_motion_direction_check is False
    assert config.frame_contract.uses_gazebo_truth_as_input is False
    assert config.slam_hover.rosbag_profile == "profiles/navlab-slam-hover-rosbag-topics.txt"
    assert config.slam_hover.slam_odom_topic == "/slam/odom"
    assert config.slam_hover.external_nav_status_topic == "/external_nav/status"
    assert config.slam_hover.fcu_pose_topic == "/ap/v1/pose/filtered"
    assert config.slam_hover.cmd_vel_topic == "/ap/v1/cmd_vel"
    assert config.slam_hover.rangefinder_range_topic == "/rangefinder/down/range"
    assert config.slam_hover.imu_topic == "/imu"
    assert config.slam_hover.truth_diagnostic_topic == "/odometry"
    assert config.slam_hover.hover_status_topic == "/navlab/hover/status"
    assert config.slam_hover.vehicle_marker_topic == "/navlab/vehicle/markers"
    assert config.slam_hover.vehicle_marker_pose_topic == "/ap/v1/pose/filtered"
    assert config.slam_hover.vehicle_marker_frame_id == ""
    assert config.slam_hover.record_visualization_markers is False
    assert config.slam_hover.vehicle_marker_rate_hz == 10.0
    assert config.slam_hover.hover_window_sec == 18.0
    assert config.slam_hover.max_hover_horizontal_drift_m == 0.35
    assert config.slam_hover.min_external_nav_rate_hz == 5.0
    assert config.slam_hover.uses_gazebo_truth_as_input is False
    assert config.slam_hover.hover_claim == "evaluated"
    assert config.slam_hover.exploration_claim == "not_evaluated"
    assert config.motion_gate.rosbag_profile == "profiles/navlab-motion-gate-rosbag-topics.txt"
    assert config.motion_gate.slam_odom_topic == "/slam/odom"
    assert config.motion_gate.external_nav_status_topic == "/external_nav/status"
    assert config.motion_gate.fcu_pose_topic == "/ap/v1/pose/filtered"
    assert config.motion_gate.cmd_vel_topic == "/ap/v1/cmd_vel"
    assert config.motion_gate.scan_topic == "/scan"
    assert config.motion_gate.motion_status_topic == "/navlab/motion/status"
    assert config.motion_gate.motion_distance_m == 0.40
    assert config.motion_gate.motion_speed_mps == 0.12
    assert config.motion_gate.yaw_scan_rad == 0.50
    assert config.motion_gate.yaw_window_sec == 4.0
    assert config.motion_gate.min_clearance_m == 0.35
    assert config.motion_gate.uses_gazebo_truth_as_input is False
    assert config.motion_gate.hover_claim == "evaluated"
    assert config.motion_gate.motion_claim == "evaluated"
    assert config.motion_gate.exploration_claim == "not_evaluated"
    assert config.scan_integrity_gate.rosbag_profile == "profiles/navlab-scan-integrity-gate-rosbag-topics.txt"
    assert config.scan_integrity_gate.raw_scan_topic == "/navlab/x2/scan_raw"
    assert config.scan_integrity_gate.normalized_scan_topic == "/navlab/x2/scan_normalized"
    assert config.scan_integrity_gate.validated_scan_topic == "/scan"
    assert config.scan_integrity_gate.status_topic == "/navlab/scan_integrity/status"
    assert config.scan_integrity_gate.attitude_source_topic == "/imu"
    assert config.scan_integrity_gate.attitude_source_type == "imu"
    assert config.scan_integrity_gate.max_attitude_source_age_ms == 250.0
    assert config.scan_integrity_gate.hard_tilt_deg == 6.0
    assert config.scan_integrity_gate.uses_gazebo_truth_as_input is False
    assert config.scan_integrity_gate.scan_integrity_claim == "evaluated"
    assert config.scan_stabilization.enabled is True
    assert config.scan_stabilization.mode == "bounded_2d_projection"
    assert config.scan_stabilization.input_scan_topic == "/navlab/x2/scan_normalized"
    assert config.scan_stabilization.output_scan_topic == "/scan"
    assert config.scan_stabilization.status_topic == "/navlab/scan_stabilization/status"
    assert config.scan_stabilization.passthrough_tilt_deg == 3.0
    assert config.scan_stabilization.compensation_tilt_deg == 8.0
    assert config.scan_stabilization.hard_drop_tilt_deg == 10.0
    assert config.scan_stabilization.max_attitude_source_age_ms == 250.0
    assert config.scan_stabilization.uses_gazebo_truth_as_input is False
    assert config.scan_stabilization_gate.rosbag_profile == "profiles/navlab-scan-stabilization-gate-rosbag-topics.txt"
    assert config.scan_stabilization_gate.motion_profile == "p9_representative_replay"
    assert config.scan_stabilization_gate.baseline_mode == "p10_drop_only"
    assert config.scan_stabilization_gate.candidate_mode == "bounded_2d_projection"
    assert config.scan_stabilization_gate.replay_readiness_timeout_sec == 90.0
    assert config.scan_stabilization_gate.controller_summary_timeout_sec == 45.0


def test_navlab_compose_environment_uses_run_scoped_session_id(monkeypatch) -> None:
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        duration_sec=45.0,
        run_id="20260603_000000",
    )

    with host._compose_environment(config):
        assert os.environ["SESSION_ID"] == "navlab_companion_sitl_gazebo/20260603_000000"


def test_navlab_images_config_drives_default_run_images() -> None:
    image_config = load_navlab_images_config(load_runtime_config())
    orchestration = OrchestrationConfig.load("orchestration/config.toml")

    assert image_config.companion.dockerfile.value == "docker/Dockerfile.companion"
    assert image_config.companion.context.value == "."
    assert image_config.companion.target.value == "navlab-companion"
    assert image_config.companion.repository.value == "world-model/navlab-companion"
    assert image_config.companion.tag_strategy.value == "latest"
    assert image_config.companion.image() == "world-model/navlab-companion:latest"
    assert image_config.slam.dockerfile.value == "docker/Dockerfile.slam"
    assert image_config.slam.target.value == "navlab-slam-cartographer"
    assert image_config.slam.repository.value == "world-model/navlab-slam-cartographer"
    assert image_config.slam.image() == "world-model/navlab-slam-cartographer:latest"
    assert image_config.gazebo_sensor.dockerfile.value == "docker/Dockerfile.gazebo-sensor"
    assert image_config.gazebo_sensor.target.value == "navlab-gazebo-sensor"
    assert image_config.gazebo_sensor.repository.value == "world-model/navlab-gazebo-sensor"
    assert image_config.gazebo_sensor.image() == "world-model/navlab-gazebo-sensor:latest"
    assert image_config.official_baseline.dockerfile.value == "docker/Dockerfile.official-baseline"
    assert image_config.official_baseline.target.value == "navlab-official-baseline"
    assert image_config.official_baseline.repository.value == "world-model/navlab-official-baseline"
    assert image_config.official_baseline.image() == "world-model/navlab-official-baseline:latest"
    assert orchestration.companion_image == image_config.companion.image()
    assert orchestration.slam.image == image_config.slam.image()
    assert orchestration.sensor.image == image_config.gazebo_sensor.image()
    assert orchestration.official_baseline.runtime_image == image_config.official_baseline.image()


def test_navlab_image_tag_cli_override_wins_over_strategy() -> None:
    image_config = load_navlab_images_config(load_runtime_config())

    assert resolve_navlab_image_tag("latest") == "latest"
    assert image_config.companion.image(cli_tag="manual-tag") == "world-model/navlab-companion:manual-tag"


def test_navlab_compose_services_do_not_include_sim_runtime() -> None:
    assert "sim-runtime" not in NAVLAB_SERVICES
    assert "sim-runtime" not in NAVLAB_STOP_SERVICES
    assert "scan-bridge" not in NAVLAB_SERVICES
    assert "foxglove" not in NAVLAB_SERVICES
    assert "rosbag-play" not in NAVLAB_SERVICES
    assert "gazebo-sensor" in NAVLAB_SERVICES


def test_navlab_run_config_is_derived_from_config() -> None:
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        duration_sec=45.0,
        run_id="20260603_000000",
    )

    assert config.duration_sec == 45.0
    assert config.run_id == "20260603_000000"
    assert config.artifact_dir.as_posix() == "artifacts/ros/navlab_companion_sitl_gazebo/20260603_000000"
    assert config.rangefinder_imu_rosbag_profile == "profiles/navlab-rangefinder-imu-rosbag-topics.txt"
    assert config.slam_backend_rosbag_profile == "profiles/navlab-slam-backend-rosbag-topics.txt"
    assert config.frame_contract_rosbag_profile == "profiles/navlab-frame-contract-rosbag-topics.txt"
    assert config.slam_hover_rosbag_profile == "profiles/navlab-slam-hover-rosbag-topics.txt"


def test_p2_rangefinder_imu_rosbag_profile_contains_required_topics() -> None:
    profile = Path("profiles/navlab-rangefinder-imu-rosbag-topics.txt")
    lines = profile.read_text(encoding="utf-8").splitlines()
    required = {
        line.split(maxsplit=1)[1]
        for line in lines
        if line.strip().startswith("required ") and len(line.split(maxsplit=1)) == 2
    }
    optional = {
        line.split(maxsplit=1)[1]
        for line in lines
        if line.strip().startswith("optional ") and len(line.split(maxsplit=1)) == 2
    }

    assert {
        "/clock",
        "/tf",
        "/tf_static",
        "/ap/v1/time",
        "/lidar",
        "/scan",
        "/sim/x2/status",
        "/rangefinder/down/scan_ideal",
        "/rangefinder/down/range",
        "/rangefinder/down/status",
        "/imu",
    }.issubset(required)
    assert "/navlab/x2/scan_ideal" not in required
    assert required.isdisjoint(optional)


def test_p2_model_overlay_keeps_official_lidar_and_adds_down_rangefinder(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(run_id="test", artifact_dir=tmp_path)
    source = (
        """<sdf version="1.9"><model name="iris_with_lidar">"""
        """<include><uri>model://lidar_3d</uri></include><link name="base_link"/></model></sdf>"""
    )
    monkeypatch.setattr(rangefinder_imu_task_module, "_docker_cat", lambda *_args, **_kwargs: source)

    summary = rangefinder_imu_task_module._write_p2_model_overlay(config, tmp_path / "model.sdf")
    rendered = (tmp_path / "model.sdf").read_text(encoding="utf-8")

    assert "navlab_x2_lidar_frame" not in rendered
    assert "navlab_x2_ideal_sensor" not in rendered
    assert "model://lidar_3d" not in rendered
    assert "model://lidar_2d" in rendered
    assert '<sensor name="down_rangefinder_sensor" type="gpu_lidar">' in rendered
    assert "<always_on>true</always_on>" in rendered
    assert "<topic>rangefinder/down/scan_ideal</topic>" in rendered
    assert summary["x2_sensor_source"] == "official_iris_with_lidar_2d_laserscan_overlay"
    assert summary["sensor_type"] == "gpu_lidar"


def test_p3_slam_backend_rosbag_profile_contains_required_topics() -> None:
    profile = Path("profiles/navlab-slam-backend-rosbag-topics.txt")
    lines = profile.read_text(encoding="utf-8").splitlines()
    required = {
        line.split(maxsplit=1)[1]
        for line in lines
        if line.strip().startswith("required ") and len(line.split(maxsplit=1)) == 2
    }

    assert {
        "/clock",
        "/tf",
        "/tf_static",
        "/lidar",
        "/scan",
        "/sim/x2/status",
        "/imu",
        "/odometry",
        "/slam/odom",
        "/map",
        "/submap_list",
        "/trajectory_node_list",
        "/sim/x2/status",
        "/rangefinder/down/range",
        "/rangefinder/down/status",
    }.issubset(required)
    assert "optional /navlab/x2/vendor_scan" in lines


def test_p3_slam_runtime_config_writes_canonical_backend_contract(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    runtime_config = tmp_path / "p3_slam_runtime.toml"

    summary = slam_backend_task_module._write_p3_slam_runtime_config(config, runtime_config)

    assert runtime_config.is_file()
    runtime = summary["data"]["slam"]["runtime"]
    assert runtime["backend"] == "cartographer"
    assert runtime["use_sim_time"] is True
    assert runtime["launch_fake_odom"] is False
    assert runtime["publish_placeholder_odom"] is False
    assert runtime["scan_topic"] == "/scan"
    assert runtime["imu_topic"] == "/imu"
    assert runtime["cartographer_odometry_topic"] == "/odometry"
    assert runtime["odom_topic"] == "/slam/odom"
    assert runtime["external_nav_input_odom_topic"] == "/slam/odom"
    assert runtime["base_frame"] == "base_link"
    assert runtime["imu_frame"] == "imu_link"
    assert runtime["laser_frame"] == "base_scan"
    assert runtime["laser_z"] == "0.075077"


def test_p3_doctor_blocks_gazebo_truth_as_slam_input(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    orchestration = replace(
        config.orchestration,
        slam_backend=replace(config.orchestration.slam_backend, uses_gazebo_truth_as_input=True),
    )
    unsafe_config = replace(config, orchestration=orchestration)
    runtime_config = tmp_path / "p3_slam_runtime.toml"
    slam_backend_task_module._write_p3_slam_runtime_config(unsafe_config, runtime_config)

    monkeypatch.setattr(slam_backend_task_module, "_build_doctor_summary", lambda _config: {"ok": True, "blockers": []})
    monkeypatch.setattr(
        slam_backend_task_module,
        "_slam_backend_print_command",
        lambda _config, runtime_config: (0, "ros2 launch navlab_slam_bringup navlab_slam_bringup.launch.py"),
    )

    summary = slam_backend_task_module._build_p3_doctor_summary(unsafe_config, runtime_config=runtime_config)

    assert summary["ok"] is False
    assert summary["p3_slam_backend_doctor"]["command"].startswith(
        "ros2 launch navlab_slam_bringup navlab_slam_bringup.launch.py"
    )
    assert "P3 SLAM backend must not use Gazebo truth as the SLAM odom source" in summary["blockers"]


def test_p3_slam_odom_quality_blocks_missing_output() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    blockers: list[str] = []

    slam_backend_task_module._append_slam_odom_quality_blockers(
        blockers=blockers,
        p3=config.orchestration.slam_backend,
        slam_odom_result={},
    )

    assert f"P3 did not receive {config.orchestration.slam_backend.slam_odom_topic}" in blockers
    assert "SLAM odom rate is below minimum" in blockers
    assert "SLAM odom latest age is too high" in blockers


def test_p3_slam_odom_quality_blocks_unstable_output() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    p3 = config.orchestration.slam_backend
    blockers: list[str] = []

    slam_backend_task_module._append_slam_odom_quality_blockers(
        blockers=blockers,
        p3=p3,
        slam_odom_result={
            "received": True,
            "frame_id": p3.odom_frame_id,
            "child_frame_id": p3.base_frame_id,
            "rate_hz": p3.min_slam_odom_rate_hz,
            "latest_age_sec": p3.max_latest_age_sec + 1.0,
            "max_jump_m": p3.max_jump_m + 1.0,
            "max_yaw_jump_rad": p3.max_yaw_jump_rad + 1.0,
            "stationary_drift_m": p3.max_stationary_drift_m + 1.0,
        },
    )

    assert "SLAM odom latest age is too high" in blockers
    assert "SLAM odom jump exceeds threshold" in blockers
    assert "SLAM odom yaw jump exceeds threshold" in blockers
    assert "SLAM odom stationary drift exceeds threshold" in blockers


def test_p4_fcu_controller_rosbag_profile_contains_required_topics() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    profile = Path(config.fcu_controller_rosbag_profile)
    content = profile.read_text(encoding="utf-8")

    for topic in (
        "/ap/v1/time",
        "/ap/v1/pose/filtered",
        "/ap/v1/twist/filtered",
        "/rangefinder/down/range",
        "/rangefinder/down/status",
        "/imu",
        "/navlab/fcu/state",
        "/navlab/fcu/controller/status",
        "/navlab/fcu/setpoint/intent",
        "/navlab/fcu/setpoint/output",
        "/navlab/fcu/owner/status",
    ):
        assert f"required {topic}" in content


def test_p4_runtime_config_writes_controller_contract(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    runtime_config = tmp_path / "p4_fcu_controller_runtime.toml"

    summary = fcu_controller_task_module._write_p4_runtime_config(config, runtime_config)

    assert runtime_config.is_file()
    runtime = summary["data"]["fcu_controller"]["runtime"]
    assert runtime["control_route"] == "mavlink_bootstrap_plus_dds_cmd_vel"
    assert runtime["mavlink_bootstrap_endpoint"] == "udp:127.0.0.1:14550"
    assert runtime["owner_name"] == "navlab_fcu_controller"
    assert runtime["cmd_vel_topic"] == "/ap/v1/cmd_vel"
    assert runtime["prearm_service"] == "/ap/v1/prearm_check"
    assert runtime["mode_switch_service"] == "/ap/v1/mode_switch"
    assert runtime["arm_service"] == "/ap/v1/arm_motors"
    assert runtime["takeoff_service"] == "/ap/v1/experimental/takeoff"
    assert runtime["hover_claim"] == "not_evaluated"
    assert runtime["exploration_claim"] == "not_evaluated"


def test_p4_doctor_blocks_mavlink_fallback(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    orchestration = replace(
        config.orchestration,
        fcu_controller=replace(config.orchestration.fcu_controller, control_route="mavlink_fallback"),
    )
    unsafe_config = replace(config, orchestration=orchestration)
    runtime_config = tmp_path / "p4_fcu_controller_runtime.toml"
    fcu_controller_task_module._write_p4_runtime_config(unsafe_config, runtime_config)

    monkeypatch.setattr(fcu_controller_task_module, "_build_doctor_summary", lambda _config: {"ok": True, "blockers": []})
    monkeypatch.setattr(
        host,
        "_docker_run_ros_shell_capture",
        lambda **_kwargs: (0, "interface ok"),
    )

    summary = fcu_controller_task_module._build_p4_doctor_summary(unsafe_config, runtime_config=runtime_config)

    assert summary["ok"] is False
    assert "control_route='mavlink_fallback' is not supported" in summary["blockers"]


def test_p4_doctor_allows_mavlink_bootstrap_route(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    runtime_config = tmp_path / "p4_fcu_controller_runtime.toml"
    fcu_controller_task_module._write_p4_runtime_config(config, runtime_config)

    monkeypatch.setattr(fcu_controller_task_module, "_build_doctor_summary", lambda _config: {"ok": True, "blockers": []})
    monkeypatch.setattr(
        host,
        "_docker_run_ros_shell_capture",
        lambda **_kwargs: (0, "ok"),
    )

    summary = fcu_controller_task_module._build_p4_doctor_summary(config, runtime_config=runtime_config)

    assert summary["ok"] is True
    assert summary["p4_fcu_controller_doctor"]["official_control_claim"] is False
    assert summary["p4_fcu_controller_doctor"]["mavlink_bootstrap_claim"] is True


def test_ros_shell_capture_filters_cyclonedds_type_hash_noise() -> None:
    command = host._with_ros_shell_stderr_filter("ros2 topic list")

    assert "ros2 topic list" in command
    assert "grep -v -E" in command
    assert "Failed to parse type hash" in command


def test_ros_shell_stderr_filter_preserves_heredoc_terminator() -> None:
    command = host._with_ros_shell_stderr_filter("python3 - <<'PY'\nprint('ok')\nPY")

    result = subprocess.run(["bash", "-lc", command], check=False, text=True, capture_output=True)

    assert result.returncode == 0
    assert result.stdout.strip() == "ok"
    assert result.stderr.strip() == ""


def test_collect_topic_info_skips_transient_topics(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        artifact_dir=tmp_path,
        run_id="20260603_000000",
    )
    calls: list[str] = []

    def fake_run_ros_shell_capture(**kwargs):  # noqa: ANN001
        calls.append(kwargs["shell_command"])
        return 0, "Publisher count: 0\nSubscription count: 0\n"

    monkeypatch.setattr(host, "_docker_run_ros_shell_capture", fake_run_ros_shell_capture)

    result = official_maze_x2_task_module._collect_topic_info(
        config,
        image="test-image",
        topics=("/navlab/exploration/coverage", "/scan"),
        transient_topics=("/navlab/exploration/coverage",),
    )

    assert calls == ["timeout --signal=INT 8s ros2 topic info -v /scan"]
    assert result["/navlab/exploration/coverage"]["skipped"] is True
    assert result["/navlab/exploration/coverage"]["reason"] == "transient_topic_gone_after_run"
    assert (tmp_path / "topic_info_navlab_exploration_coverage.txt").read_text(encoding="utf-8").startswith(
        "skipped transient topic info after run"
    )


def test_p4_owner_blockers_detect_competing_publishers() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    blockers: list[str] = []

    fcu_controller_task_module._append_owner_blockers(
        blockers=blockers,
        owner_summary={"unique": True, "set_pose_count": 0},
        cmd_vel_publishers=["navlab_fcu_controller", "mission_controller"],
        p4=config.orchestration.fcu_controller,
    )

    assert "movement output has competing publishers: ['mission_controller']" in blockers


def test_p4_owner_blockers_detect_direct_set_pose() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    blockers: list[str] = []

    fcu_controller_task_module._append_owner_blockers(
        blockers=blockers,
        owner_summary={"unique": True, "set_pose_count": 1},
        cmd_vel_publishers=["navlab_fcu_controller"],
        p4=config.orchestration.fcu_controller,
    )

    assert "direct set pose count is non-zero" in blockers


def test_p4_controller_blockers_detect_pre_ready_output_and_direct_pose() -> None:
    blockers: list[str] = []

    fcu_controller_task_module._append_controller_blockers(
        blockers=blockers,
        controller={
            "ok": True,
            "command_results": {
                "prearm_check": {"success": True},
                "set_guided": {"success": True},
                "arm": {"success": True},
                "takeoff": {"success": True},
            },
            "counts": {"pose": 1, "rejected_before_ready": 0, "output": 0},
        },
    )

    assert "controller did not reject a pre-ready movement intent" in blockers
    assert "controller did not publish setpoint output diagnostics" in blockers


def test_p5_frame_contract_rosbag_profile_contains_required_topics() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    profile = Path(config.frame_contract_rosbag_profile)
    content = profile.read_text(encoding="utf-8")

    for topic in (
        "/tf",
        "/tf_static",
        "/ap/v1/time",
        "/ap/v1/pose/filtered",
        "/ap/v1/twist/filtered",
        "/ap/v1/status",
        "/ap/v1/cmd_vel",
        "/scan",
        "/imu",
        "/rangefinder/down/range",
        "/rangefinder/down/status",
        "/slam/odom",
        "/navlab/slam/status",
        "/navlab/fcu/state",
        "/navlab/fcu/controller/status",
        "/navlab/fcu/setpoint/output",
        "/navlab/fcu/owner/status",
        "/navlab/frame_contract/status",
    ):
        assert f"required {topic}" in content


def test_p5_runtime_config_writes_frame_contract(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    runtime_config = tmp_path / "p5_frame_contract_runtime.toml"

    summary = frame_contract_task_module._write_p5_runtime_config(config, runtime_config)

    assert runtime_config.is_file()
    runtime = summary["data"]["frame_contract"]["runtime"]
    assert runtime["required_frames"] == [
        "map",
        "odom",
        "base_link",
        "imu_link",
        "base_scan",
        "rangefinder_down_frame",
    ]
    assert runtime["map_frame_id"] == "map"
    assert runtime["odom_frame_id"] == "odom"
    assert runtime["base_frame_id"] == "base_link"
    assert runtime["scan_topic"] == "/scan"
    assert runtime["slam_odom_topic"] == "/slam/odom"
    assert runtime["status_topic"] == "/navlab/frame_contract/status"
    assert runtime["uses_gazebo_truth_as_input"] is False
    assert runtime["hover_claim"] == "not_evaluated"
    assert runtime["exploration_claim"] == "not_evaluated"


def test_p5_doctor_blocks_gazebo_truth_as_input(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    orchestration = replace(
        config.orchestration,
        frame_contract=replace(config.orchestration.frame_contract, uses_gazebo_truth_as_input=True),
    )
    unsafe_config = replace(config, orchestration=orchestration, artifact_dir=tmp_path)
    runtime_config = tmp_path / "p5_frame_contract_runtime.toml"
    frame_contract_task_module._write_p5_runtime_config(unsafe_config, runtime_config)

    monkeypatch.setattr(frame_contract_task_module, "_build_doctor_summary", lambda _config: {"ok": True, "blockers": []})
    monkeypatch.setattr(
        frame_contract_task_module,
        "_build_p3_doctor_summary",
        lambda _config, runtime_config: {"ok": True, "blockers": []},
    )
    monkeypatch.setattr(
        frame_contract_task_module,
        "_build_p4_doctor_summary",
        lambda _config, runtime_config: {"ok": True, "blockers": []},
    )

    summary = frame_contract_task_module._build_p5_doctor_summary(unsafe_config, runtime_config=runtime_config)

    assert summary["ok"] is False
    assert "P5 must not use Gazebo truth as a control/planning/ExternalNav input" in summary["blockers"]


def test_p5_blockers_detect_failed_frame_contract() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    blockers: list[str] = []

    frame_contract_task_module._append_p5_blockers(
        blockers=blockers,
        frame_summary={
            "ok": False,
            "blockers": ["required frames are missing"],
            "tf": {"ok": False},
            "scan": {"ok": False},
            "imu": {"ok": False},
            "rangefinder": {"ok": False},
        },
        rosbag_profile={"ok": False},
        counts={},
        p5=config.orchestration.frame_contract,
    )

    assert "required frames are missing" in blockers
    assert "P5 TF contract did not pass" in blockers
    assert "P5 scan contract did not pass" in blockers
    assert "P5 IMU contract did not pass" in blockers
    assert "P5 rangefinder contract did not pass" in blockers
    assert "P5 rosbag profile did not pass" in blockers
    assert "/navlab/frame_contract/status was not recorded" in blockers


def test_p6_slam_hover_rosbag_profile_contains_required_topics() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    profile = Path(config.slam_hover_rosbag_profile)
    content = profile.read_text(encoding="utf-8")

    for topic in (
        "/clock",
        "/tf",
        "/tf_static",
        "/lidar",
        "/scan",
        "/sim/x2/status",
        "/imu",
        "/rangefinder/down/range",
        "/rangefinder/down/status",
        "/slam/odom",
        "/navlab/slam/status",
        "/external_nav/status",
        "/ap/v1/time",
        "/ap/v1/pose/filtered",
        "/ap/v1/twist/filtered",
        "/ap/v1/status",
        "/ap/v1/cmd_vel",
        "/navlab/fcu/state",
        "/navlab/fcu/controller/status",
        "/navlab/fcu/setpoint/intent",
        "/navlab/fcu/setpoint/output",
        "/navlab/fcu/owner/status",
        "/navlab/hover/status",
    ):
        assert f"required {topic}" in content
    assert "required /navlab/vehicle/markers" not in content


def test_p6_runtime_config_writes_slam_hover_contract(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    runtime_config = tmp_path / "p6_slam_hover_runtime.toml"

    summary = slam_hover_task_module._write_p6_runtime_config(config, runtime_config)

    assert runtime_config.is_file()
    runtime = summary["data"]["slam_hover"]["runtime"]
    assert runtime["slam_odom_topic"] == "/slam/odom"
    assert runtime["external_nav_status_topic"] == "/external_nav/status"
    assert runtime["fcu_pose_topic"] == "/ap/v1/pose/filtered"
    assert runtime["hover_status_topic"] == "/navlab/hover/status"
    assert runtime["vehicle_marker_topic"] == "/navlab/vehicle/markers"
    assert runtime["vehicle_marker_pose_topic"] == "/ap/v1/pose/filtered"
    assert runtime["vehicle_marker_frame_id"] == ""
    assert runtime["vehicle_marker_rate_hz"] == 10.0
    assert runtime["record_visualization_markers"] is False
    assert runtime["uses_gazebo_truth_as_input"] is False
    assert runtime["hover_claim"] == "evaluated"
    assert runtime["exploration_claim"] == "not_evaluated"


def test_p6_effective_rosbag_profile_adds_vehicle_markers_only_when_enabled(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        run_id="20260603_000000",
        artifact_dir=tmp_path / "disabled",
    )

    disabled_profile, disabled_required, _, disabled_topics = slam_hover_task_module._write_p6_effective_rosbag_profile(
        config
    )

    assert disabled_profile.is_file()
    assert config.orchestration.slam_hover.vehicle_marker_topic not in disabled_required
    assert config.orchestration.slam_hover.vehicle_marker_topic not in disabled_topics

    enabled_orchestration = replace(
        config.orchestration,
        slam_hover=replace(config.orchestration.slam_hover, record_visualization_markers=True),
    )
    enabled_config = replace(config, orchestration=enabled_orchestration, artifact_dir=tmp_path / "enabled")

    enabled_profile, enabled_required, _, enabled_topics = slam_hover_task_module._write_p6_effective_rosbag_profile(
        enabled_config
    )

    assert enabled_profile.is_file()
    assert enabled_config.orchestration.slam_hover.vehicle_marker_topic in enabled_required
    assert enabled_config.orchestration.slam_hover.vehicle_marker_topic in enabled_topics
    assert "record_visualization_markers: true" in enabled_profile.read_text(encoding="utf-8")


def test_p6_doctor_blocks_gazebo_truth_as_input(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    orchestration = replace(
        config.orchestration,
        slam_hover=replace(config.orchestration.slam_hover, uses_gazebo_truth_as_input=True),
    )
    unsafe_config = replace(config, orchestration=orchestration, artifact_dir=tmp_path)
    runtime_config = tmp_path / "p6_slam_hover_runtime.toml"
    slam_hover_task_module._write_p6_runtime_config(unsafe_config, runtime_config)

    monkeypatch.setattr(slam_hover_task_module, "_build_doctor_summary", lambda _config: {"ok": True, "blockers": []})
    monkeypatch.setattr(
        slam_hover_task_module,
        "_build_p3_doctor_summary",
        lambda _config, runtime_config: {"ok": True, "blockers": []},
    )
    monkeypatch.setattr(
        slam_hover_task_module,
        "_build_p4_doctor_summary",
        lambda _config, runtime_config: {"ok": True, "blockers": []},
    )
    monkeypatch.setattr(
        slam_hover_task_module,
        "_build_p5_doctor_summary",
        lambda _config, runtime_config: {"ok": True, "blockers": []},
    )

    summary = slam_hover_task_module._build_p6_doctor_summary(unsafe_config, runtime_config=runtime_config)

    assert summary["ok"] is False
    assert "P6 must not use Gazebo truth as a control/planning/SLAM/ExternalNav input" in summary["blockers"]


def test_p6_blockers_detect_failed_hover_gate() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    blockers: list[str] = []

    slam_hover_task_module._append_p6_blockers(
        blockers=blockers,
        hover_summary={
            "ok": False,
            "blockers": ["P6 hover drift gate did not pass"],
            "slam_odom": {"ok": False},
            "external_nav": {"ok": False},
            "fcu": {"local_position_ok": False},
            "hover": {"ok": False},
        },
        rosbag_profile={"ok": False},
        counts={},
        p6=config.orchestration.slam_hover,
    )

    assert "P6 hover drift gate did not pass" in blockers
    assert "P6 SLAM odom gate did not pass" in blockers
    assert "P6 ExternalNav gate did not pass" in blockers
    assert "P6 FCU local position gate did not pass" in blockers
    assert "P6 rosbag profile did not pass" in blockers
    assert "/navlab/hover/status was not recorded" in blockers


def test_p6_blockers_require_vehicle_marker_only_when_recording_enabled() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    enabled_orchestration = replace(
        config.orchestration,
        slam_hover=replace(config.orchestration.slam_hover, record_visualization_markers=True),
    )
    enabled_config = replace(config, orchestration=enabled_orchestration)
    blockers: list[str] = []

    slam_hover_task_module._append_p6_blockers(
        blockers=blockers,
        hover_summary={
            "ok": True,
            "slam_odom": {"ok": True},
            "external_nav": {"ok": True},
            "fcu": {"local_position_ok": True},
            "hover": {"ok": True},
        },
        rosbag_profile={"ok": True},
        counts={config.orchestration.slam_hover.hover_status_topic: 1},
        p6=enabled_config.orchestration.slam_hover,
    )

    assert "/navlab/vehicle/markers was not recorded" in blockers


def test_p7_motion_gate_rosbag_profile_contains_required_topics() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    profile = Path(config.motion_gate_rosbag_profile)
    content = profile.read_text(encoding="utf-8")

    for topic in (
        "/clock",
        "/tf",
        "/tf_static",
        "/scan",
        "/imu",
        "/rangefinder/down/range",
        "/rangefinder/down/status",
        "/slam/odom",
        "/navlab/slam/status",
        "/external_nav/status",
        "/ap/v1/time",
        "/ap/v1/pose/filtered",
        "/ap/v1/twist/filtered",
        "/ap/v1/status",
        "/ap/v1/cmd_vel",
        "/navlab/fcu/state",
        "/navlab/fcu/controller/status",
        "/navlab/fcu/setpoint/intent",
        "/navlab/fcu/setpoint/output",
        "/navlab/fcu/owner/status",
        "/navlab/hover/status",
        "/navlab/motion/status",
    ):
        assert f"required {topic}" in content


def test_p7_runtime_config_writes_motion_gate_contract(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    runtime_config = tmp_path / "p7_motion_gate_runtime.toml"

    summary = motion_gate_task_module._write_p7_runtime_config(config, runtime_config)

    assert runtime_config.is_file()
    runtime = summary["data"]["motion_gate"]["runtime"]
    assert runtime["slam_odom_topic"] == "/slam/odom"
    assert runtime["external_nav_status_topic"] == "/external_nav/status"
    assert runtime["fcu_pose_topic"] == "/ap/v1/pose/filtered"
    assert runtime["cmd_vel_topic"] == "/ap/v1/cmd_vel"
    assert runtime["scan_topic"] == "/scan"
    assert runtime["motion_status_topic"] == "/navlab/motion/status"
    assert runtime["motion_distance_m"] == 0.40
    assert runtime["motion_speed_mps"] == 0.12
    assert runtime["yaw_scan_rad"] == 0.50
    assert runtime["uses_gazebo_truth_as_input"] is False
    assert runtime["hover_claim"] == "evaluated"
    assert runtime["motion_claim"] == "evaluated"
    assert runtime["exploration_claim"] == "not_evaluated"


def test_p7_motion_coordinator_does_not_publish_cmd_vel(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        artifact_dir=tmp_path,
        run_id="20260603_000000",
    )
    script_path = tmp_path / "p7_motion_gate_probe.py"

    motion_gate_task_module._write_motion_probe_script(config, script_path)
    script = script_path.read_text(encoding="utf-8")

    assert 'rclpy.create_node("navlab_p7_motion_gate_coordinator")' in script
    assert 'create_publisher(TwistStamped, SPEC["cmd_vel_topic"]' not in script
    assert 'create_subscription(TwistStamped, SPEC["cmd_vel_topic"]' in script


def test_p7_controller_script_can_consume_motion_intents(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        artifact_dir=tmp_path,
        run_id="20260603_000000",
    )
    script_path = tmp_path / "p7_fcu_controller_runtime.py"

    summary = fcu_controller_task_module._write_controller_runtime_script(
        config,
        script_path,
        duration_sec=120.0,
        hold_after_ready_sec=90.0,
        enable_motion_intent_control=True,
        hover_status_topic=config.orchestration.motion_gate.hover_status_topic,
    )
    script = script_path.read_text(encoding="utf-8")

    assert summary["spec"]["enable_motion_intent_control"] is True
    assert summary["spec"]["hover_status_topic"] == "/navlab/hover/status"
    assert 'self.node.create_subscription(String, SPEC["setpoint_intent_topic"], self._motion_intent_cb, 10)' in script
    assert 'self.cmd_vel_pub = self.node.create_publisher(TwistStamped, SPEC["cmd_vel_topic"], 10)' in script


def test_p7_doctor_blocks_gazebo_truth_as_input(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    orchestration = replace(
        config.orchestration,
        motion_gate=replace(config.orchestration.motion_gate, uses_gazebo_truth_as_input=True),
    )
    unsafe_config = replace(config, orchestration=orchestration, artifact_dir=tmp_path)
    runtime_config = tmp_path / "p7_motion_gate_runtime.toml"
    motion_gate_task_module._write_p7_runtime_config(unsafe_config, runtime_config)

    monkeypatch.setattr(
        motion_gate_task_module,
        "_build_p6_doctor_summary",
        lambda _config, runtime_config: {"ok": True, "blockers": []},
    )

    summary = motion_gate_task_module._build_p7_doctor_summary(unsafe_config, runtime_config=runtime_config)

    assert summary["ok"] is False
    assert "P7 must not use Gazebo truth as a control/planning/SLAM/ExternalNav input" in summary["blockers"]


def test_p7_blockers_detect_failed_motion_gate() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    blockers: list[str] = []

    motion_gate_task_module._append_p7_blockers(
        blockers=blockers,
        motion_summary={
            "ok": False,
            "blockers": ["P7 forward displacement below threshold"],
            "motion_actions": {
                "forward": {"ok": False},
                "back": {"ok": False},
                "yaw_scan": {"ok": False},
            },
            "clearance": {"ok": False},
            "slam_odom": {"ok": False},
            "external_nav": {"ok": False},
            "fcu": {"local_position_ok": False},
        },
        rosbag_profile={"ok": False},
        counts={},
        p7=config.orchestration.motion_gate,
    )

    assert "P7 forward displacement below threshold" in blockers
    assert "P7 forward motion gate did not pass" in blockers
    assert "P7 back motion gate did not pass" in blockers
    assert "P7 yaw scan gate did not pass" in blockers
    assert "P7 clearance gate did not pass" in blockers
    assert "P7 SLAM odom gate did not pass" in blockers
    assert "P7 ExternalNav gate did not pass" in blockers
    assert "P7 FCU local position gate did not pass" in blockers
    assert "P7 rosbag profile did not pass" in blockers
    assert "/navlab/motion/status was not recorded" in blockers


def test_p8_exploration_gate_config_loads() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")

    assert config.exploration_gate_rosbag_profile == "profiles/navlab-exploration-gate-rosbag-topics.txt"
    assert config.orchestration.exploration_gate.strategy == "frontier_lite"
    assert config.orchestration.exploration_gate.slam_odom_topic == "/slam/odom"
    assert config.orchestration.exploration_gate.map_topic == "/map"
    assert config.orchestration.exploration_gate.cmd_vel_topic == "/ap/v1/cmd_vel"
    assert config.orchestration.exploration_gate.exploration_status_topic == "/navlab/exploration/status"
    assert config.orchestration.exploration_gate.exploration_goal_topic == "/navlab/exploration/goal"
    assert config.orchestration.exploration_gate.exploration_coverage_topic == "/navlab/exploration/coverage"
    assert config.orchestration.exploration_gate.min_accepted_goals == 3
    assert config.orchestration.exploration_gate.min_clearance_m == 0.35
    assert config.orchestration.exploration_gate.uses_gazebo_truth_as_input is False
    assert config.orchestration.exploration_gate.hover_claim == "evaluated"
    assert config.orchestration.exploration_gate.motion_claim == "evaluated"
    assert config.orchestration.exploration_gate.exploration_claim == "evaluated"


def test_p8_replay_profile_extends_motion_without_relaxing_safety_contract() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")

    replay_config = exploration_gate_task_module._apply_replay_profile(config, "conservative")
    base = config.orchestration.exploration_gate
    replay_p8 = replay_config.orchestration.exploration_gate

    assert replay_p8.strategy == "frontier_lite_replay_conservative"
    assert replay_p8.exploration_window_sec == 50.0
    assert replay_p8.forward_probe_window_sec == 4.0
    assert replay_p8.stop_hold_window_sec == 2.5
    assert replay_p8.motion_speed_mps == 0.18
    assert replay_p8.min_accepted_goals == 5
    assert replay_p8.min_path_length_m == 2.5
    assert replay_p8.min_clearance_m == base.min_clearance_m
    assert replay_p8.max_stop_drift_m == base.max_stop_drift_m
    assert replay_p8.owner_status_topic == base.owner_status_topic
    assert replay_p8.setpoint_intent_topic == base.setpoint_intent_topic
    assert replay_p8.setpoint_output_topic == base.setpoint_output_topic
    assert replay_p8.uses_gazebo_truth_as_input is False
    assert replay_p8.hover_claim == "evaluated"
    assert replay_p8.motion_claim == "evaluated"
    assert replay_p8.exploration_claim == "evaluated"


def test_p9_display_replay_profile_goes_farther_without_relaxing_safety_contract() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")

    replay_config = exploration_gate_task_module._apply_replay_profile(config, "display")
    base = config.orchestration.exploration_gate
    replay_p8 = replay_config.orchestration.exploration_gate

    assert replay_p8.strategy == "frontier_lite_replay_display"
    assert replay_p8.exploration_window_sec == 90.0
    assert replay_p8.forward_probe_window_sec == 5.0
    assert replay_p8.stop_hold_window_sec == 2.0
    assert replay_p8.final_hold_window_sec == 12.0
    assert replay_p8.motion_speed_mps == 0.25
    assert replay_p8.min_accepted_goals == 7
    assert replay_p8.min_path_length_m == 5.0
    assert replay_p8.min_clearance_m == base.min_clearance_m
    assert replay_p8.max_stop_drift_m == base.max_stop_drift_m
    assert replay_p8.owner_status_topic == base.owner_status_topic
    assert replay_p8.setpoint_intent_topic == base.setpoint_intent_topic
    assert replay_p8.setpoint_output_topic == base.setpoint_output_topic
    assert replay_p8.uses_gazebo_truth_as_input is False
    assert replay_p8.hover_claim == "evaluated"
    assert replay_p8.motion_claim == "evaluated"
    assert replay_p8.exploration_claim == "evaluated"


def test_p8_replay_slam_health_check_ignores_stationary_probe_metrics() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    p3 = config.orchestration.slam_backend
    blockers: list[str] = []

    exploration_gate_task_module._append_replay_slam_health_blockers(
        blockers=blockers,
        p3=p3,
        slam_odom_result={
            "received": True,
            "frame_id": p3.odom_frame_id,
            "child_frame_id": p3.base_frame_id,
            "rate_hz": p3.min_slam_odom_rate_hz,
            "latest_age_sec": 0.0,
            "max_jump_m": p3.max_jump_m + 10.0,
            "stationary_drift_m": p3.max_stationary_drift_m + 10.0,
        },
    )

    assert blockers == []


def test_p8_exploration_gate_rosbag_profile_contains_required_topics() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    profile = Path(config.exploration_gate_rosbag_profile)
    content = profile.read_text(encoding="utf-8")

    for topic in (
        "/clock",
        "/tf",
        "/tf_static",
        "/lidar",
        "/scan",
        "/sim/x2/status",
        "/imu",
        "/rangefinder/down/range",
        "/rangefinder/down/status",
        "/slam/odom",
        "/navlab/slam/status",
        "/external_nav/status",
        "/map",
        "/submap_list",
        "/trajectory_node_list",
        "/ap/v1/time",
        "/ap/v1/pose/filtered",
        "/ap/v1/twist/filtered",
        "/ap/v1/status",
        "/ap/v1/cmd_vel",
        "/navlab/fcu/state",
        "/navlab/fcu/controller/status",
        "/navlab/fcu/setpoint/intent",
        "/navlab/fcu/setpoint/output",
        "/navlab/fcu/owner/status",
        "/navlab/hover/status",
        "/navlab/motion/status",
        "/navlab/exploration/status",
        "/navlab/exploration/goal",
        "/navlab/exploration/coverage",
    ):
        assert f"required {topic}" in content


def test_p8_runtime_config_writes_exploration_gate_contract(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    runtime_config = tmp_path / "p8_exploration_gate_runtime.toml"

    summary = exploration_gate_task_module._write_p8_runtime_config(config, runtime_config)

    assert runtime_config.is_file()
    runtime = summary["data"]["exploration_gate"]["runtime"]
    assert runtime["strategy"] == "frontier_lite"
    assert runtime["slam_odom_topic"] == "/slam/odom"
    assert runtime["map_topic"] == "/map"
    assert runtime["cmd_vel_topic"] == "/ap/v1/cmd_vel"
    assert runtime["exploration_status_topic"] == "/navlab/exploration/status"
    assert runtime["min_accepted_goals"] == 3
    assert runtime["uses_gazebo_truth_as_input"] is False
    assert runtime["hover_claim"] == "evaluated"
    assert runtime["motion_claim"] == "evaluated"
    assert runtime["exploration_claim"] == "evaluated"


def test_p8_exploration_coordinator_does_not_publish_cmd_vel(tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        artifact_dir=tmp_path,
        run_id="20260603_000000",
    )
    script_path = tmp_path / "p8_exploration_gate_probe.py"

    exploration_gate_task_module._write_exploration_probe_script(config, script_path)
    script = script_path.read_text(encoding="utf-8")

    assert 'rclpy.create_node("navlab_p8_exploration_coordinator")' in script
    assert 'create_publisher(TwistStamped, SPEC["cmd_vel_topic"]' not in script
    assert 'create_subscription(TwistStamped, SPEC["cmd_vel_topic"]' in script
    assert 'self.intent_pub = self.node.create_publisher(String, SPEC["setpoint_intent_topic"], 10)' in script


def test_p8_doctor_blocks_gazebo_truth_as_input(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    orchestration = replace(
        config.orchestration,
        exploration_gate=replace(config.orchestration.exploration_gate, uses_gazebo_truth_as_input=True),
    )
    unsafe_config = replace(config, orchestration=orchestration, artifact_dir=tmp_path)
    runtime_config = tmp_path / "p8_exploration_gate_runtime.toml"
    exploration_gate_task_module._write_p8_runtime_config(unsafe_config, runtime_config)

    monkeypatch.setattr(
        exploration_gate_task_module,
        "_build_p6_doctor_summary",
        lambda _config, runtime_config: {"ok": True, "blockers": []},
    )
    monkeypatch.setattr(
        exploration_gate_task_module,
        "_build_p7_doctor_summary",
        lambda _config, runtime_config, include_dependencies=False: {"ok": True, "blockers": []},
    )

    summary = exploration_gate_task_module._build_p8_doctor_summary(unsafe_config, runtime_config=runtime_config)

    assert summary["ok"] is False
    assert "P8 must not use Gazebo truth as a control/planning/SLAM/ExternalNav input" in summary["blockers"]


def test_p8_blockers_detect_failed_exploration_gate() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", run_id="20260603_000000")
    blockers: list[str] = []

    exploration_gate_task_module._append_p8_blockers(
        blockers=blockers,
        exploration_summary={
            "ok": False,
            "blockers": ["P8 coverage/progress below threshold"],
            "p6_hover_prerequisite": {"ok": False},
            "p7_motion_prerequisite": {"ok": False},
            "p8_exploration": {"ok": False},
            "coverage": {"ok": False},
            "safety": {"ok": False},
            "collision": {"detected": True},
            "stuck": {"blocked": True},
            "slam_odom": {"ok": False},
            "external_nav": {"ok": False},
            "fcu": {"local_position_ok": False},
        },
        rosbag_profile={"ok": False},
        counts={},
        p8=config.orchestration.exploration_gate,
    )

    assert "P8 coverage/progress below threshold" in blockers
    assert "P8 P6 hover prerequisite did not pass" in blockers
    assert "P8 P7 motion prerequisite did not pass" in blockers
    assert "P8 exploration gate did not pass" in blockers
    assert "P8 coverage/progress gate did not pass" in blockers
    assert "P8 safety gate did not pass" in blockers
    assert "P8 collision diagnostic triggered" in blockers
    assert "P8 stuck gate did not pass" in blockers
    assert "P8 SLAM odom gate did not pass" in blockers
    assert "P8 ExternalNav gate did not pass" in blockers
    assert "P8 FCU local position gate did not pass" in blockers
    assert "P8 rosbag profile did not pass" in blockers
    assert "/navlab/exploration/status was not recorded" in blockers
    assert "/navlab/exploration/goal was not recorded" in blockers
    assert "/navlab/exploration/coverage was not recorded" in blockers


def test_p10_scan_attitude_quality_summary_fields() -> None:
    quality = scan_integrity_gate_task_module._scan_attitude_quality(
        latest_status={
            "max_scan_tilt_deg": 8.0,
            "tilt_filtered_scan_count": 12,
            "tilt_warning_count": 3,
            "dropped_scan_count": 10,
            "clipped_scan_count": 2,
        },
        ok=True,
    )

    assert quality["ok"] is True
    assert quality["max_scan_tilt_deg"] == 8.0
    assert quality["tilt_filtered_scan_count"] == 12
    assert quality["tilt_warning_count"] == 3


def test_p10_motor_output_summary_marks_missing_topics_not_available() -> None:
    summary = scan_integrity_gate_task_module._motor_output_summary(
        ros_graph={"ros2_topic_list": {"lines": ["/ap/v1/time", "/ap/v1/pose/filtered", "/ap/v1/rc"]}}
    )

    assert summary["motor_output_claim"] == "not_available"
    assert summary["available"] is False
    assert summary["motor_pwm_spread"] is None


def test_p10_motor_output_summary_reports_candidate_topics() -> None:
    summary = scan_integrity_gate_task_module._motor_output_summary(
        ros_graph={"ros2_topic_list": {"lines": ["/ap/v1/esc_status", "/actuator_outputs"]}}
    )

    assert summary["motor_output_claim"] == "candidate_topics_present"
    assert summary["candidate_topics"] == ["/actuator_outputs", "/ap/v1/esc_status"]


def test_p10_normal_profile_ok_ignores_displacement_only_failures() -> None:
    summary = {
        "ok": False,
        "blockers": ["P7 forward displacement below threshold"],
        "p6_hover_prerequisite": {"ok": True},
        "clearance": {"ok": True},
        "slam_odom": {"ok": True},
        "external_nav": {"ok": True},
        "fcu": {"local_position_ok": True},
    }

    assert scan_integrity_gate_task_module._p10_normal_profile_ok(summary) is True


def test_p11_scan_stabilization_config_validation_accepts_defaults() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", duration_sec=10.0)

    assert scan_stabilization_gate_task_module._validate_p11_config(config) == []


def test_p11_scan_stabilization_config_validation_rejects_p8_motion_profile() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", duration_sec=10.0)
    gate = replace(config.orchestration.scan_stabilization_gate, motion_profile="p8_slow_exploration")
    orchestration = replace(config.orchestration, scan_stabilization_gate=gate)
    invalid = replace(config, orchestration=orchestration)

    assert "motion_profile_not_p9_representative_replay" in scan_stabilization_gate_task_module._validate_p11_config(invalid)


def test_p11_summary_schema_parser_accepts_required_shape() -> None:
    summary = {
        "scan_stabilization_claim": "evaluated",
        "motion_profile": "p9_representative_replay",
        "uses_gazebo_truth_as_input": False,
        "uses_official_maze_as_input": False,
        "scan_stabilization": {
            "mode": "bounded_2d_projection",
            "input_scan_topic": "/navlab/x2/scan_normalized",
            "output_scan_topic": "/scan",
            "runtime_config": {},
            "passthrough_scan_count": 10,
            "compensated_scan_count": 0,
            "dropped_scan_count": 0,
            "false_wall_risk_ok": True,
        },
        "baseline_comparison": {
            "baseline_mode": "p10_drop_only",
            "candidate_mode": "bounded_2d_projection",
            "baseline_validated_scan_count": 10,
            "candidate_validated_scan_count": 10,
            "scan_availability_improved": True,
            "slam_health_regressed": False,
            "map_artifact_risk_ok": True,
        },
        "rosbag_profile": {"ok": True},
    }

    assert scan_stabilization_gate_task_module._p11_summary_schema_blockers(summary) == []


def test_p11_summary_schema_parser_reports_missing_fields() -> None:
    blockers = scan_stabilization_gate_task_module._p11_summary_schema_blockers(
        {"scan_stabilization": {}, "baseline_comparison": {}}
    )

    assert "p11_summary_schema_missing:scan_stabilization_claim" in blockers
    assert "p11_summary_schema_missing:scan_stabilization.mode" in blockers
    assert "p11_summary_schema_missing:baseline_comparison.map_artifact_risk_ok" in blockers


def test_p11_blockers_detect_candidate_scan_availability_regression() -> None:
    config = RunConfig.from_config(config_path="orchestration/config.toml", duration_sec=10.0)
    blockers: list[str] = []
    scan = config.orchestration.scan_stabilization
    gate = config.orchestration.scan_stabilization_gate
    counts = {
        gate.raw_scan_topic: 10,
        gate.normalized_scan_topic: 10,
        scan.output_scan_topic: 9,
        scan.status_topic: 10,
        scan.events_topic: 1,
        scan.attitude_source_topic: 10,
        scan.range_topic: 10,
    }
    topic_info = {
        scan.output_scan_topic: {"publisher_nodes": ["navlab_scan_stabilization_filter"]},
        gate.raw_scan_topic: {"subscription_nodes": ["navlab_x2_scan_time_normalizer"]},
    }

    scan_stabilization_gate_task_module._append_p11_blockers(
        blockers=blockers,
        config=config,
        counts=counts,
        topic_info=topic_info,
        latest_status={
            "base_scan_static_tf_ok": True,
            "attitude_source_is_truth": False,
            "candidate_validated_scan_count": 8,
            "baseline_drop_only_validated_scan_count": 9,
            "false_wall_risk_ok": True,
        },
        rosbag_profile={"ok": True},
    )

    assert "candidate scan availability regressed below drop-only baseline estimate" in blockers


def test_orchestration_task_registry_contains_navlab_workflows() -> None:
    assert TaskRegistry.names() == (
        "build",
        "doctor",
        "exploration",
        "exploration-doctor",
        "hover",
        "real-preflight-doctor",
        "scan-robustness",
        "scan-robustness-doctor",
    )
    assert TaskRegistry.create("build").description
    assert TaskRegistry.create("doctor").description
    assert TaskRegistry.create("exploration-doctor").description
    assert TaskRegistry.create("exploration").description
    assert TaskRegistry.create("hover").description
    assert TaskRegistry.create("real-preflight-doctor").description
    assert TaskRegistry.create("scan-robustness-doctor").description
    assert TaskRegistry.create("scan-robustness").description
    with pytest.raises(ValueError, match="unknown orchestration task 'scan-stabilization-gate-acceptance'"):
        TaskRegistry.create("scan-stabilization-gate-acceptance")


def test_orchestration_sources_do_not_import_legacy_tasks() -> None:
    source_roots = (Path("orchestration/src"), Path("orchestration/tests"))
    legacy_import_markers = ("src.tasks." + "legacy", "from ." + "legacy")
    offenders: list[Path] = []
    for root in source_roots:
        for path in root.rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            source = path.read_text(encoding="utf-8")
            if any(marker in source for marker in legacy_import_markers):
                offenders.append(path)

    assert offenders == []


def test_legacy_task_helper_package_has_no_business_modules() -> None:
    legacy_dir = Path("orchestration/src/tasks/legacy")
    if not legacy_dir.exists():
        return
    business_files = [path for path in legacy_dir.glob("*.py") if path.name != "__init__.py"]
    assert business_files == []


def test_orchestration_task_registry_requires_task_name() -> None:
    class MissingNameTask(OrchestrationTask):
        TASK_DESCRIPTION = "Missing task name"

        def run(self, **kwargs: object) -> int:
            return 0

    with pytest.raises(ValueError, match="MissingNameTask must define TASK_NAME"):
        TaskRegistry.register(MissingNameTask)


def test_orchestration_task_registry_rejects_duplicate_task_name() -> None:
    class DuplicateHoverTask(OrchestrationTask):
        TASK_NAME = "hover"
        TASK_DESCRIPTION = "Duplicate hover task"

        def run(self, **kwargs: object) -> int:
            return 0

    with pytest.raises(ValueError, match="orchestration task 'hover' is already registered"):
        TaskRegistry.register(DuplicateHoverTask)


def test_navlab_official_baseline_doctor_writes_cartographer_summary(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("ARTIFACT_DIR", str(tmp_path))

    def fake_run_ros_shell_capture(**kwargs):  # noqa: ANN001
        command = kwargs["shell_command"]
        if command == "true":
            return 0, ""
        if command == "timeout --signal=INT 8s ros2 pkg prefix cartographer_ros":
            return 0, "/opt/ros/jazzy\n"
        if command == "timeout --signal=INT 8s ros2 pkg executables cartographer_ros":
            return 0, (
                "cartographer_ros cartographer_node\n"
                "cartographer_ros cartographer_occupancy_grid_node\n"
            )
        if command.startswith("timeout --signal=INT 8s ros2 pkg prefix ardupilot_"):
            return 0, "/opt/navlab_official_ws/install\n"
        if command == "timeout --signal=INT 8s ros2 pkg prefix micro_ros_agent":
            return 0, "/opt/navlab_official_ws/install\n"
        if "MicroXRCEAgent" in command:
            return 0, f"/usr/bin/{command.rsplit(maxsplit=1)[-1]}\n"
        return 1, "unexpected command"

    monkeypatch.setattr(host, "_docker_run_ros_shell_capture", fake_run_ros_shell_capture)

    rc = run_official_baseline_doctor(console=Console(file=io.StringIO()))

    assert rc == 0
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    official = summary["official_baseline"]
    assert summary["ok"] is True
    assert official["cartographer_ros_present"] is True
    assert official["cartographer_node_present"] is True
    assert official["cartographer_occupancy_grid_node_present"] is True
    assert official["cartographer_config_hash"]
    assert official["cartographer_uses_odometry"] is True
    assert official["tracking_frame"] == "imu_link"
    assert official["published_frame"] == "base_link"
    assert official["odom_frame"] == "odom"
    assert official["official_runtime_image"] == "world-model/navlab-official-baseline:latest"
    assert official["official_runtime_image_available"] is True
    assert official["missing_official_ros_packages"] == []
    assert official["micro_ros_agent_available"] is True
    assert official["official_sitl_launch"] == "ros2 launch ardupilot_sitl sitl_dds_udp.launch.py"
    assert official["official_gazebo_launch"] == "ros2 launch ardupilot_gz_bringup iris_maze.launch.py"
    assert official["official_cartographer_launch"] == "ros2 launch ardupilot_cartographer cartographer.launch.py"


def test_navlab_official_baseline_acceptance_blocks_mavlink_fallback_without_ap_graph(
    monkeypatch,
    tmp_path,
) -> None:
    monkeypatch.setenv("ARTIFACT_DIR", str(tmp_path))
    monkeypatch.setattr(host, "_render_run_config", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(host, "_compose_up", lambda _config: None)
    monkeypatch.setattr(host, "_compose_stop", lambda _config: None)
    monkeypatch.setattr(host, "_capture_stack_logs", lambda **_kwargs: None)
    monkeypatch.setattr(host, "_gazebo_network_namespace", lambda: "container:gazebo-id")
    monkeypatch.setattr(
        official_baseline_task_module,
        "_record_official_rosbag",
        lambda *_args, **_kwargs: {
            "ok": True,
            "recorded": True,
            "required_topics": ["/ap/v1/time"],
            "optional_topics": [],
        },
    )
    monkeypatch.setattr(
        host,
        "_compose_ps_status",
        lambda _config: [
            {"name": "navlab-gazebo-1", "image": "remote-sitl-lab/gazebo-headless:latest", "running": True},
            {"name": "navlab-sitl-1", "image": "remote-sitl-lab/ardupilot-sitl:stage1", "running": True},
        ],
    )
    monkeypatch.setattr(official_baseline_task_module.time, "sleep", lambda _sec: None)

    def fake_run_ros_shell_capture(**kwargs):  # noqa: ANN001
        command = kwargs["shell_command"]
        if command == "true":
            return 0, ""
        if command == "timeout --signal=INT 8s ros2 pkg prefix cartographer_ros":
            return 0, "/opt/ros/jazzy\n"
        if command == "timeout --signal=INT 8s ros2 pkg executables cartographer_ros":
            return 0, (
                "cartographer_ros cartographer_node\n"
                "cartographer_ros cartographer_occupancy_grid_node\n"
            )
        if command.startswith("timeout --signal=INT 8s ros2 pkg prefix ardupilot_"):
            return 0, "/opt/navlab_official_ws/install\n"
        if command == "timeout --signal=INT 8s ros2 pkg prefix micro_ros_agent":
            return 0, "/opt/navlab_official_ws/install\n"
        if "MicroXRCEAgent" in command:
            return 0, f"/usr/bin/{command.rsplit(maxsplit=1)[-1]}\n"
        if command == "ros2 node list":
            return 0, "/rosout\n"
        if command == "ros2 topic list --include-hidden-topics":
            return 0, "/clock\n/tf\n/tf_static\n"
        if command == "ros2 node info /ap":
            return 1, "Unable to find node /ap\n"
        if "navlab_official_dds_probe" in command:
            return 0, (
                '{"prearm_service": "/ap/v1/prearm_check", '
                '"prearm_service_available": false, "prearm_success": null, '
                '"time_nanosec": null, "time_received": false, "time_sec": null, '
                '"time_topic": "/ap/v1/time"}\n'
            )
        return 1, "unexpected command"

    monkeypatch.setattr(host, "_docker_run_ros_shell_capture", fake_run_ros_shell_capture)

    rc = run_official_baseline_acceptance(duration_sec=1.0, console=Console(file=io.StringIO()))

    assert rc == 30
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    official = summary["official_baseline"]
    assert summary["ok"] is False
    assert summary["blocked"] is True
    assert official["ap_node_present"] is False
    assert official["ap_topics"] == []
    assert official["external_nav_route"] == "official_dds"
    assert official["gazebo_truth_route"] == "diagnostic_only"
    assert official["missing_official_ros_packages"] == []
    assert official["micro_ros_agent_available"] is True
    assert official["process_status"][0]["name"] == "navlab-gazebo-1"
    assert "official DDS probe did not receive /ap/v1/time" in summary["blockers"]
    assert "official DDS probe did not find /ap/v1/prearm_check service" in summary["blockers"]
    assert summary["rosbag_profile"]["ok"] is True
    assert "/ap/v1/time" in summary["rosbag_profile"]["required_topics"]


def test_navlab_official_baseline_only_official_dds_route_can_pass() -> None:
    assert official_baseline_task_module._route_is_official("official_dds") is True
    assert official_baseline_task_module._route_is_official("mavlink_fallback") is False
    assert official_baseline_task_module._route_is_official("diagnostic_only") is False
    assert official_baseline_task_module._route_is_official("unknown") is False


def test_companion_launcher_autostarts_sim_marker_and_scan_nodes(monkeypatch) -> None:
    config = RuntimeConfig(
        path=Path("profile.toml"),
        imu_source_label="fcu",
        world_markers=WorldMarkersConfig(autostart=True, topic="/sim/markers"),
        scan_features=ScanFeaturesConfig(autostart=True, features_topic="/scan_features"),
        gazebo_truth_bridge=GazeboTruthBridgeConfig(autostart=False),
        gazebo_truth_odom=GazeboTruthOdomConfig(autostart=False),
        pose_mirror=PoseMirrorConfig(autostart=False),
        imu_bridge=ImuBridgeConfig(autostart=False),
        external_nav_sender=ExternalNavSenderConfig(autostart=False),
        mission=MissionNodeConfig(autostart=False),
    )
    started: list[tuple[str, list[str]]] = []

    def fake_start_function(self: CompanionLauncher, name, target, argv):  # noqa: ANN001
        started.append((name, argv))

    monkeypatch.setattr(CompanionLauncher, "_start_function", fake_start_function)

    CompanionLauncher(config)._start_configured_processes()

    assert started[0][0] == "world_marker_publisher"
    assert "--topic" in started[0][1]
    assert "/sim/markers" in started[0][1]
    assert started[1][0] == "scan_features_publisher"
    assert "--features-topic" in started[1][1]
    assert "/scan_features" in started[1][1]


def test_navlab_orchestration_starts_slam_container(monkeypatch) -> None:
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        duration_sec=45.0,
        run_id="20260603_000000",
    )
    services: list[ServiceSpec] = []

    class FakeDockerBackend:
        def start_service(self, spec: ServiceSpec) -> None:
            services.append(spec)

        def remove_container(self, *_args, **_kwargs) -> None:  # noqa: ANN002, ANN003
            return None

    monkeypatch.setattr(host, "DockerBackend", FakeDockerBackend)
    monkeypatch.setattr(host, "_gazebo_network_namespace", lambda: "container:gazebo-id")
    monkeypatch.setattr(host, "_remove_slam_container", lambda: None)

    host._start_slam_container(config)

    assert len(services) == 1
    service = services[0]
    assert service.image == "world-model/navlab-slam-cartographer:latest"
    assert service.container_name == "navlab-slam"
    assert service.networks == ("container:gazebo-id",)
    command = " ".join(service.command)
    assert "python3 -m navlab.slam.cli launch" in command
    assert "--config /workspace/navlab/config.toml" in command
    assert "--backend cartographer" in command
    assert "/gazebo/truth/odom" not in command
    assert service.env["NAVLAB_SLAM_RUNTIME_CONFIG"] == "/workspace/navlab/config.toml"


def test_navlab_official_baseline_uses_artifact_sitl_workdir(monkeypatch, tmp_path) -> None:
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        duration_sec=45.0,
        run_id="20260603_000000",
        artifact_dir=tmp_path,
    )
    services: list[ServiceSpec] = []

    class FakeDockerBackend:
        def start_service(self, spec: ServiceSpec) -> None:
            services.append(spec)

        def remove_container(self, *_args, **_kwargs) -> None:  # noqa: ANN002, ANN003
            return None

    monkeypatch.setattr(host, "DockerBackend", FakeDockerBackend)
    monkeypatch.setattr(host, "_workspace_path", lambda path: f"/workspace/{Path(path).name}")

    host._start_official_baseline_container(config)

    assert (tmp_path / "sitl_work").is_dir()
    assert len(services) == 1
    service = services[0]
    assert service.image == "world-model/navlab-official-baseline:latest"
    assert service.container_name == "navlab-official-baseline"
    assert service.cwd == "/workspace/sitl_work"


def test_navlab_orchestration_starts_companion_with_runtime_config(monkeypatch) -> None:
    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
        duration_sec=45.0,
        run_id="20260603_000000",
    )
    services: list[ServiceSpec] = []

    class FakeDockerBackend:
        def start_service(self, spec: ServiceSpec) -> None:
            services.append(spec)

        def remove_container(self, *_args, **_kwargs) -> None:  # noqa: ANN002, ANN003
            return None

    monkeypatch.setattr(host, "DockerBackend", FakeDockerBackend)
    monkeypatch.setattr(host, "_gazebo_network_namespace", lambda: "container:gazebo-id")

    host._start_companion_container(config)

    assert len(services) == 1
    service = services[0]
    assert service.image == "world-model/navlab-companion:latest"
    assert service.container_name == "navlab-companion"
    assert service.env["NAVLAB_RUNTIME_CONFIG"] == "/workspace/navlab/config.toml"
    assert "NAVLAB_CONFIG" not in service.env


def test_navlab_hover_acceptance_invokes_hover_runtime_cli(monkeypatch, tmp_path) -> None:
    calls: list[dict[str, object]] = []
    captured_logs: list[dict[str, object]] = []

    monkeypatch.setenv("ARTIFACT_DIR", str(tmp_path))
    monkeypatch.setattr(host, "_render_run_config", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(host, "_compose_up", lambda _config: None)
    monkeypatch.setattr(host, "_start_slam_container", lambda _config: None)
    monkeypatch.setattr(host, "_start_companion_container", lambda _config: None)
    monkeypatch.setattr(host, "_capture_stack_logs", lambda **_kwargs: None)
    monkeypatch.setattr(host, "_capture_compose_service_log", lambda **kwargs: captured_logs.append(kwargs))
    monkeypatch.setattr(hover_task_module, "finalize_navlab_artifact", lambda **_kwargs: None)
    monkeypatch.setattr(host, "_remove_companion_container", lambda: None)
    monkeypatch.setattr(host, "_remove_slam_container", lambda: None)
    monkeypatch.setattr(host, "_compose_stop", lambda _config: None)
    monkeypatch.setattr(
        hover_task_module,
        "upload_acceptance_rosbag",
        lambda _config: SimpleNamespace(ok=False, state="skipped", reason="test"),
    )

    def fake_exec_runtime_command(**kwargs):  # noqa: ANN001
        calls.append(kwargs)
        return 0

    monkeypatch.setattr(host, "_docker_exec_runtime_command", fake_exec_runtime_command)

    rc = HoverAcceptanceTask().run(duration_sec=12.0, console=Console(file=io.StringIO()))

    assert rc == 0
    assert len(calls) == 1
    assert calls[0]["module"] == "navlab.companion.acceptance_cli"
    args = calls[0]["args"]
    assert args[0] == "execute-hover-acceptance"
    assert "--artifact-dir" in args
    assert captured_logs[0]["service"] == "sitl"
    assert captured_logs[0]["output_path"] == tmp_path / "sitl.log"


def test_navlab_image_build_uses_global_image_config(monkeypatch) -> None:
    builds: list[dict[str, object]] = []

    class FakeDockerClient:
        def build(self, context_path, **kwargs):  # noqa: ANN001
            builds.append({"context_path": context_path, **kwargs})
            return iter(["build log"])

    monkeypatch.setattr(build_task_module, "DockerClient", FakeDockerClient)
    console = Console(file=io.StringIO(), width=120)

    rc = BuildTask().run(kind="companion", tag="cli-tag", console=console)

    assert rc == 0
    assert len(builds) == 1
    build = builds[0]
    assert Path(build["context_path"]).resolve() == Path.cwd().resolve()
    assert Path(build["file"]).resolve() == (Path.cwd() / "docker/Dockerfile.companion").resolve()
    assert build["target"] == "navlab-companion"
    assert build["tags"] == "world-model/navlab-companion:cli-tag"
    assert build["stream_logs"] is True


def test_navlab_image_build_supports_slam(monkeypatch) -> None:
    builds: list[dict[str, object]] = []

    class FakeDockerClient:
        def build(self, context_path, **kwargs):  # noqa: ANN001
            builds.append({"context_path": context_path, **kwargs})
            return iter(["build log"])

    monkeypatch.setattr(build_task_module, "DockerClient", FakeDockerClient)
    console = Console(file=io.StringIO(), width=120)

    rc = BuildTask().run(kind="slam", tag="cli-tag", console=console)

    assert rc == 0
    assert len(builds) == 1
    build = builds[0]
    assert Path(build["file"]).resolve() == (Path.cwd() / "docker/Dockerfile.slam").resolve()
    assert build["target"] == "navlab-slam-cartographer"
    assert build["tags"] == "world-model/navlab-slam-cartographer:cli-tag"


def test_navlab_image_build_supports_gazebo_sensor(monkeypatch) -> None:
    builds: list[dict[str, object]] = []

    class FakeDockerClient:
        def build(self, context_path, **kwargs):  # noqa: ANN001
            builds.append({"context_path": context_path, **kwargs})
            return iter(["build log"])

    monkeypatch.setattr(build_task_module, "DockerClient", FakeDockerClient)
    console = Console(file=io.StringIO(), width=120)

    rc = BuildTask().run(kind="gazebo-sensor", tag="cli-tag", console=console)

    assert rc == 0
    assert len(builds) == 1
    build = builds[0]
    assert Path(build["file"]).resolve() == (Path.cwd() / "docker/Dockerfile.gazebo-sensor").resolve()
    assert build["target"] == "navlab-gazebo-sensor"
    assert build["tags"] == "world-model/navlab-gazebo-sensor:cli-tag"


def test_navlab_image_build_supports_official_baseline(monkeypatch) -> None:
    builds: list[dict[str, object]] = []

    class FakeDockerClient:
        def build(self, context_path, **kwargs):  # noqa: ANN001
            builds.append({"context_path": context_path, **kwargs})
            return iter(["build log"])

    monkeypatch.setattr(build_task_module, "DockerClient", FakeDockerClient)
    console = Console(file=io.StringIO(), width=120)

    rc = BuildTask().run(kind="official-baseline", tag="cli-tag", console=console)

    assert rc == 0
    assert len(builds) == 1
    build = builds[0]
    assert Path(build["file"]).resolve() == (Path.cwd() / "docker/Dockerfile.official-baseline").resolve()
    assert build["target"] == "navlab-official-baseline"
    assert build["tags"] == "world-model/navlab-official-baseline:cli-tag"
