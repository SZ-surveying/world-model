from __future__ import annotations

import os
import shlex
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import tomllib

from src.project_config import load_navlab_images_config, load_runtime_config

DEFAULT_CONFIG = Path("orchestration/config.toml")
NAVLAB_PROFILES = ("base_env", "x2_sensor")
NAVLAB_SERVICES = (
    "gazebo",
    "gazebo-sensor",
    "mavlink-router",
    "sitl",
)
NAVLAB_STOP_SERVICES = (
    "gazebo-sensor",
    "gazebo",
    "sitl",
    "mavlink-router",
)


@dataclass(frozen=True, slots=True)
class FoxgloveUploadConfig:
    enabled: bool
    api_url: str
    token_env: str
    project_id: str
    device_id: str
    device_name: str
    key_prefix: str
    filename_prefix: str


@dataclass(frozen=True, slots=True)
class SlamContainerConfig:
    autostart: bool
    image: str
    backend: str
    runtime_config: str


@dataclass(frozen=True, slots=True)
class SensorContainerConfig:
    scan_source: str
    image: str

    @property
    def acceptance_scan_source(self) -> str:
        if self.scan_source == "x2_virtual_serial":
            return "x2_virtual_serial_vendor_driver"
        return self.scan_source


@dataclass(frozen=True, slots=True)
class OfficialBaselineConfig:
    rosbag_profile: str
    dds_enable: str
    dds_domain_id: str
    rmw_implementation: str
    expected_ap_node: str
    required_ap_topics: tuple[str, ...]
    runtime_image: str
    required_ros_packages: tuple[str, ...]
    micro_ros_agent_binaries: tuple[str, ...]
    sitl_launch: str
    gazebo_launch: str
    cartographer_launch: str
    gazebo_bringup_mode: str
    external_nav_route: str


@dataclass(frozen=True, slots=True)
class OfficialMazeX2Config:
    rosbag_profile: str
    world_source: str
    vehicle_model_source: str
    gazebo_lidar_topic: str
    x2_scan_input_topic: str
    x2_scan_topic: str
    x2_status_topic: str
    x2_virtual_serial_link: str
    altitude_control_claim: str
    hover_claim: str
    cartographer_launch: str


@dataclass(frozen=True, slots=True)
class RangefinderImuConfig:
    rosbag_profile: str
    world_source: str
    vehicle_model_source: str
    model_overlay_source: str
    gazebo_lidar_topic: str
    x2_scan_input_topic: str
    x2_scan_topic: str
    x2_status_topic: str
    x2_virtual_serial_link: str
    rangefinder_scan_ideal_topic: str
    rangefinder_range_topic: str
    rangefinder_status_topic: str
    rangefinder_frame_id: str
    rangefinder_model_pose: str
    rangefinder_model_update_rate_hz: float
    rangefinder_model_ray_count: str
    rangefinder_model_noise_stddev_m: float
    rangefinder_endpoint: str
    rangefinder_fcu_probe_endpoint: str
    rangefinder_mavlink_orientation: str
    rangefinder_source_system: str
    rangefinder_source_component: str
    rangefinder_sensor_id: str
    rangefinder_rate_hz: float
    rangefinder_min_distance_m: float
    rangefinder_max_distance_m: float
    rangefinder_covariance_cm: str
    imu_source_route: str
    imu_source_topic: str
    imu_output_topic: str
    imu_status_topic: str
    imu_frame_id: str
    imu_min_rate_hz: float
    synthetic_fallback_enabled: bool
    altitude_control_claim: str
    hover_claim: str
    cartographer_launch: str


@dataclass(frozen=True, slots=True)
class SlamBackendQualityConfig:
    rosbag_profile: str
    backend: str
    launch_package: str
    launch_file: str
    cartographer_configuration_basename: str
    scan_topic: str
    imu_topic: str
    odometry_topic: str
    slam_odom_topic: str
    slam_status_topic: str
    external_nav_status_topic: str
    x2_scan_input_topic: str
    x2_vendor_scan_topic: str
    x2_scan_topic: str
    x2_status_topic: str
    rangefinder_range_topic: str
    rangefinder_status_topic: str
    imu_frame_id: str
    laser_frame_id: str
    odom_frame_id: str
    base_frame_id: str
    min_slam_odom_rate_hz: float
    max_latest_age_sec: float
    max_jump_m: float
    max_yaw_jump_rad: float
    max_stationary_drift_m: float
    truth_diagnostic_topic: str
    uses_gazebo_truth_as_input: bool


@dataclass(frozen=True, slots=True)
class FcuControllerConfig:
    rosbag_profile: str
    control_route: str
    mavlink_bootstrap_endpoint: str
    mavlink_bootstrap_source_system: int
    mavlink_bootstrap_source_component: int
    owner_name: str
    owner_id: str
    fcu_state_topic: str
    controller_status_topic: str
    setpoint_intent_topic: str
    setpoint_output_topic: str
    owner_status_topic: str
    time_topic: str
    prearm_service: str
    mode_switch_service: str
    arm_service: str
    takeoff_service: str
    cmd_vel_topic: str
    pose_topic: str
    twist_topic: str
    status_topic: str
    rangefinder_range_topic: str
    rangefinder_status_topic: str
    imu_topic: str
    slam_odom_topic: str
    slam_status_topic: str
    guided_mode: int
    takeoff_alt_m: float
    readiness_timeout_sec: float
    hold_after_ready_sec: float
    require_slam_backend: bool
    hover_claim: str
    exploration_claim: str


@dataclass(frozen=True, slots=True)
class FrameContractConfig:
    rosbag_profile: str
    required_frames: tuple[str, ...]
    map_frame_id: str
    odom_frame_id: str
    base_frame_id: str
    imu_frame_id: str
    laser_frame_id: str
    rangefinder_frame_id: str
    scan_topic: str
    imu_topic: str
    rangefinder_range_topic: str
    rangefinder_status_topic: str
    fcu_pose_topic: str
    fcu_twist_topic: str
    fcu_status_topic: str
    cmd_vel_topic: str
    slam_odom_topic: str
    slam_status_topic: str
    truth_diagnostic_topic: str
    controller_status_topic: str
    setpoint_output_topic: str
    owner_status_topic: str
    status_topic: str
    max_dynamic_tf_age_sec: float
    min_scan_valid_ratio: float
    max_rangefinder_height_error_m: float
    max_direction_error_rad: float
    probe_duration_sec: float
    require_motion_direction_check: bool
    hover_claim: str
    exploration_claim: str
    uses_gazebo_truth_as_input: bool


@dataclass(frozen=True, slots=True)
class OrchestrationConfig:
    path: Path
    session_id: str
    ros_domain_id: str
    gazebo_world: str
    rosbag_profile: str
    companion_image: str
    sitl_image: str
    sitl_model: str
    sitl_speedup: str
    sitl_instance: str
    sitl_home: str
    sitl_upstream_endpoint: str
    sitl_router_only: str
    sitl_extra_args: tuple[str, ...]
    router_image: str
    router_downstream_endpoints: str
    router_listen: str
    router_tcp_port: str
    sensor: SensorContainerConfig
    slam: SlamContainerConfig
    official_baseline: OfficialBaselineConfig
    official_maze_x2: OfficialMazeX2Config
    rangefinder_imu: RangefinderImuConfig
    slam_backend: SlamBackendQualityConfig
    fcu_controller: FcuControllerConfig
    frame_contract: FrameContractConfig
    foxglove_upload: FoxgloveUploadConfig

    @classmethod
    def load(cls, path: str | Path | None = None) -> OrchestrationConfig:
        config_path = resolve_config_path(path)
        data = tomllib.loads(config_path.read_text(encoding="utf-8"))
        runtime_config = load_runtime_config()
        image_config = load_navlab_images_config(runtime_config)
        sitl = _section(data, "sitl")
        router = _section(data, "router")
        sensor = _section(data, "sensor")
        slam = _section(data, "slam")
        official_baseline = _section(data, "official_baseline")
        official_maze_x2 = _section(data, "official_maze_x2")
        rangefinder_imu = _section(data, "rangefinder_imu")
        slam_backend = _section(data, "slam_backend")
        fcu_controller = _section(data, "fcu_controller")
        frame_contract = _section(data, "frame_contract")
        foxglove_upload = _section(data, "foxglove_upload")
        ros_domain_id = _as_str(data.get("ros_domain_id"), "85")
        return cls(
            path=config_path,
            session_id=_as_str(data.get("session_id"), "navlab_companion_sitl_gazebo"),
            ros_domain_id=ros_domain_id,
            gazebo_world=_as_str(data.get("gazebo_world"), "/workspace/worlds/navlab_iq_quad_figure8.sdf"),
            rosbag_profile=_as_str(data.get("rosbag_profile"), "profiles/navlab-rosbag-topics.txt"),
            companion_image=_as_str(
                data.get("companion_image"),
                image_config.companion.image(cwd=runtime_config.lab_root),
            ),
            sitl_image=_as_str(sitl.get("image"), "remote-sitl-lab/ardupilot-sitl:stage1-f10500ae45aa"),
            sitl_model=_as_str(sitl.get("model"), "JSON"),
            sitl_speedup=_as_str(sitl.get("speedup"), "1"),
            sitl_instance=_as_str(sitl.get("instance"), "0"),
            sitl_home=_as_str(sitl.get("home"), ""),
            sitl_upstream_endpoint=_as_str(sitl.get("upstream_endpoint"), "mavlink-router:14550"),
            sitl_router_only=_as_str(sitl.get("router_only"), "false"),
            sitl_extra_args=_as_args(sitl.get("extra_args")),
            router_image=_as_str(router.get("image"), "remote-sitl-lab/mavlink-router:stage1-4ee567d97525"),
            router_downstream_endpoints=_as_str(router.get("downstream_endpoints"), "127.0.0.1:14552"),
            router_listen=_as_str(router.get("listen"), "0.0.0.0:14550"),
            router_tcp_port=_as_str(router.get("tcp_port"), "5760"),
            sensor=SensorContainerConfig(
                scan_source=_as_scan_source(sensor.get("scan_source")),
                image=_as_str(sensor.get("image"), image_config.gazebo_sensor.image(cwd=runtime_config.lab_root)),
            ),
            slam=SlamContainerConfig(
                autostart=_as_bool(slam.get("autostart"), True),
                image=_as_str(slam.get("image"), image_config.slam.image(cwd=runtime_config.lab_root)),
                backend=_as_str(slam.get("backend"), "cartographer"),
                runtime_config=_as_str(slam.get("runtime_config"), "/workspace/navlab/config.toml"),
            ),
            official_baseline=OfficialBaselineConfig(
                rosbag_profile=_as_str(
                    official_baseline.get("rosbag_profile"),
                    "profiles/navlab-official-baseline-rosbag-topics.txt",
                ),
                dds_enable=_as_str(official_baseline.get("dds_enable"), "1"),
                dds_domain_id=_as_str(official_baseline.get("dds_domain_id"), ros_domain_id),
                rmw_implementation=_as_str(official_baseline.get("rmw_implementation"), "rmw_cyclonedds_cpp"),
                expected_ap_node=_as_str(official_baseline.get("expected_ap_node"), "/ap"),
                required_ap_topics=_as_str_tuple(official_baseline.get("required_ap_topics"), ("/ap/v1/time",)),
                runtime_image=_as_str(
                    official_baseline.get("runtime_image"),
                    image_config.official_baseline.image(cwd=runtime_config.lab_root),
                ),
                required_ros_packages=_as_str_tuple(
                    official_baseline.get("required_ros_packages"),
                    (
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
                    ),
                ),
                micro_ros_agent_binaries=_as_str_tuple(
                    official_baseline.get("micro_ros_agent_binaries"),
                    ("MicroXRCEAgent", "micro_ros_agent"),
                ),
                sitl_launch=_as_str(
                    official_baseline.get("sitl_launch"),
                    "ros2 launch ardupilot_sitl sitl_dds_udp.launch.py",
                ),
                gazebo_launch=_as_str(
                    official_baseline.get("gazebo_launch"),
                    "ros2 launch ardupilot_gz_bringup iris_maze.launch.py",
                ),
                cartographer_launch=_as_str(
                    official_baseline.get("cartographer_launch"),
                    "ros2 launch ardupilot_cartographer cartographer.launch.py",
                ),
                gazebo_bringup_mode=_as_str(official_baseline.get("gazebo_bringup_mode"), "navlab_custom_bringup"),
                external_nav_route=_as_str(official_baseline.get("external_nav_route"), "mavlink_fallback"),
            ),
            official_maze_x2=OfficialMazeX2Config(
                rosbag_profile=_as_str(
                    official_maze_x2.get("rosbag_profile"),
                    "profiles/navlab-official-maze-x2-rosbag-topics.txt",
                ),
                world_source=_as_str(official_maze_x2.get("world_source"), "official_iris_maze"),
                vehicle_model_source=_as_str(
                    official_maze_x2.get("vehicle_model_source"),
                    "official_iris_with_lidar",
                ),
                gazebo_lidar_topic=_as_str(official_maze_x2.get("gazebo_lidar_topic"), "/lidar"),
                x2_scan_input_topic=_as_str(official_maze_x2.get("x2_scan_input_topic"), "/lidar"),
                x2_scan_topic=_as_str(official_maze_x2.get("x2_scan_topic"), "/scan"),
                x2_status_topic=_as_str(official_maze_x2.get("x2_status_topic"), "/sim/x2/status"),
                x2_virtual_serial_link=_as_str(
                    official_maze_x2.get("x2_virtual_serial_link"),
                    "/tmp/navlab_p1_x2",
                ),
                altitude_control_claim=_as_str(
                    official_maze_x2.get("altitude_control_claim"),
                    "not_evaluated",
                ),
                hover_claim=_as_str(official_maze_x2.get("hover_claim"), "not_evaluated"),
                cartographer_launch=_as_str(
                    official_maze_x2.get("cartographer_launch"),
                    official_baseline.get("cartographer_launch")
                    or "ros2 launch ardupilot_cartographer cartographer.launch.py",
                ),
            ),
            rangefinder_imu=RangefinderImuConfig(
                rosbag_profile=_as_str(
                    rangefinder_imu.get("rosbag_profile"),
                    "profiles/navlab-rangefinder-imu-rosbag-topics.txt",
                ),
                world_source=_as_str(rangefinder_imu.get("world_source"), "official_iris_maze"),
                vehicle_model_source=_as_str(
                    rangefinder_imu.get("vehicle_model_source"),
                    "official_iris_with_lidar",
                ),
                model_overlay_source=_as_str(
                    rangefinder_imu.get("model_overlay_source"),
                    "official_iris_with_lidar_plus_down_rangefinder",
                ),
                gazebo_lidar_topic=_as_str(rangefinder_imu.get("gazebo_lidar_topic"), "/lidar"),
                x2_scan_input_topic=_as_str(rangefinder_imu.get("x2_scan_input_topic"), "/lidar"),
                x2_scan_topic=_as_str(rangefinder_imu.get("x2_scan_topic"), "/scan"),
                x2_status_topic=_as_str(rangefinder_imu.get("x2_status_topic"), "/sim/x2/status"),
                x2_virtual_serial_link=_as_str(
                    rangefinder_imu.get("x2_virtual_serial_link"),
                    "/tmp/navlab_p2_x2",
                ),
                rangefinder_scan_ideal_topic=_as_str(
                    rangefinder_imu.get("rangefinder_scan_ideal_topic"),
                    "/rangefinder/down/scan_ideal",
                ),
                rangefinder_range_topic=_as_str(
                    rangefinder_imu.get("rangefinder_range_topic"),
                    "/rangefinder/down/range",
                ),
                rangefinder_status_topic=_as_str(
                    rangefinder_imu.get("rangefinder_status_topic"),
                    "/rangefinder/down/status",
                ),
                rangefinder_frame_id=_as_str(
                    rangefinder_imu.get("rangefinder_frame_id"),
                    "rangefinder_down_frame",
                ),
                rangefinder_model_pose=_as_str(
                    rangefinder_imu.get("rangefinder_model_pose"),
                    "0 0 -0.02 0 1.5707963267948966 0",
                ),
                rangefinder_model_update_rate_hz=_as_float(
                    rangefinder_imu.get("rangefinder_model_update_rate_hz"),
                    20.0,
                ),
                rangefinder_model_ray_count=_as_str(rangefinder_imu.get("rangefinder_model_ray_count"), "1"),
                rangefinder_model_noise_stddev_m=_as_float(
                    rangefinder_imu.get("rangefinder_model_noise_stddev_m"),
                    0.0,
                ),
                rangefinder_endpoint=_as_str(rangefinder_imu.get("rangefinder_endpoint"), "tcp:127.0.0.1:5760"),
                rangefinder_fcu_probe_endpoint=_as_str(
                    rangefinder_imu.get("rangefinder_fcu_probe_endpoint"),
                    "udpin:0.0.0.0:14551",
                ),
                rangefinder_mavlink_orientation=_as_str(
                    rangefinder_imu.get("rangefinder_mavlink_orientation"),
                    "MAV_SENSOR_ROTATION_PITCH_270",
                ),
                rangefinder_source_system=_as_str(rangefinder_imu.get("rangefinder_source_system"), "1"),
                rangefinder_source_component=_as_str(rangefinder_imu.get("rangefinder_source_component"), "158"),
                rangefinder_sensor_id=_as_str(rangefinder_imu.get("rangefinder_sensor_id"), "1"),
                rangefinder_rate_hz=_as_float(rangefinder_imu.get("rangefinder_rate_hz"), 20.0),
                rangefinder_min_distance_m=_as_float(rangefinder_imu.get("rangefinder_min_distance_m"), 0.05),
                rangefinder_max_distance_m=_as_float(rangefinder_imu.get("rangefinder_max_distance_m"), 6.0),
                rangefinder_covariance_cm=_as_str(rangefinder_imu.get("rangefinder_covariance_cm"), "2"),
                imu_source_route=_as_str(rangefinder_imu.get("imu_source_route"), "official_gazebo_imu_bridge"),
                imu_source_topic=_as_str(rangefinder_imu.get("imu_source_topic"), "/imu"),
                imu_output_topic=_as_str(rangefinder_imu.get("imu_output_topic"), "/imu"),
                imu_status_topic=_as_str(rangefinder_imu.get("imu_status_topic"), "/imu/status"),
                imu_frame_id=_as_str(rangefinder_imu.get("imu_frame_id"), "imu_link"),
                imu_min_rate_hz=_as_float(rangefinder_imu.get("imu_min_rate_hz"), 4.0),
                synthetic_fallback_enabled=_as_bool(rangefinder_imu.get("synthetic_fallback_enabled"), False),
                altitude_control_claim=_as_str(
                    rangefinder_imu.get("altitude_control_claim"),
                    "not_evaluated",
                ),
                hover_claim=_as_str(rangefinder_imu.get("hover_claim"), "not_evaluated"),
                cartographer_launch=_as_str(
                    rangefinder_imu.get("cartographer_launch"),
                    official_baseline.get("cartographer_launch")
                    or "ros2 launch ardupilot_cartographer cartographer.launch.py",
                ),
            ),
            slam_backend=SlamBackendQualityConfig(
                rosbag_profile=_as_str(
                    slam_backend.get("rosbag_profile"),
                    "profiles/navlab-slam-backend-rosbag-topics.txt",
                ),
                backend=_as_str(slam_backend.get("backend"), "cartographer"),
                launch_package=_as_str(slam_backend.get("launch_package"), "navlab_slam_bringup"),
                launch_file=_as_str(slam_backend.get("launch_file"), "navlab_slam_bringup.launch.py"),
                cartographer_configuration_basename=_as_str(
                    slam_backend.get("cartographer_configuration_basename"),
                    "navlab_cartographer_2d.lua",
                ),
                scan_topic=_as_str(slam_backend.get("scan_topic"), "/scan"),
                imu_topic=_as_str(slam_backend.get("imu_topic"), "/imu"),
                odometry_topic=_as_str(slam_backend.get("odometry_topic"), "/odometry"),
                slam_odom_topic=_as_str(slam_backend.get("slam_odom_topic"), "/slam/odom"),
                slam_status_topic=_as_str(slam_backend.get("slam_status_topic"), "/navlab/slam/status"),
                external_nav_status_topic=_as_str(
                    slam_backend.get("external_nav_status_topic"),
                    "/external_nav/status",
                ),
                x2_scan_input_topic=_as_str(slam_backend.get("x2_scan_input_topic"), "/navlab/x2/scan_ideal"),
                x2_vendor_scan_topic=_as_str(slam_backend.get("x2_vendor_scan_topic"), "/navlab/x2/vendor_scan"),
                x2_scan_topic=_as_str(slam_backend.get("x2_scan_topic"), "/scan"),
                x2_status_topic=_as_str(slam_backend.get("x2_status_topic"), "/sim/x2/status"),
                rangefinder_range_topic=_as_str(
                    slam_backend.get("rangefinder_range_topic"),
                    "/rangefinder/down/range",
                ),
                rangefinder_status_topic=_as_str(
                    slam_backend.get("rangefinder_status_topic"),
                    "/rangefinder/down/status",
                ),
                imu_frame_id=_as_str(slam_backend.get("imu_frame_id"), "imu_link"),
                laser_frame_id=_as_str(slam_backend.get("laser_frame_id"), "base_scan"),
                odom_frame_id=_as_str(slam_backend.get("odom_frame_id"), "odom"),
                base_frame_id=_as_str(slam_backend.get("base_frame_id"), "base_link"),
                min_slam_odom_rate_hz=_as_float(slam_backend.get("min_slam_odom_rate_hz"), 1.0),
                max_latest_age_sec=_as_float(slam_backend.get("max_latest_age_sec"), 1.0),
                max_jump_m=_as_float(slam_backend.get("max_jump_m"), 2.0),
                max_yaw_jump_rad=_as_float(slam_backend.get("max_yaw_jump_rad"), 1.0),
                max_stationary_drift_m=_as_float(slam_backend.get("max_stationary_drift_m"), 2.0),
                truth_diagnostic_topic=_as_str(slam_backend.get("truth_diagnostic_topic"), "/odometry"),
                uses_gazebo_truth_as_input=_as_bool(slam_backend.get("uses_gazebo_truth_as_input"), False),
            ),
            fcu_controller=FcuControllerConfig(
                rosbag_profile=_as_str(
                    fcu_controller.get("rosbag_profile"),
                    "profiles/navlab-fcu-controller-rosbag-topics.txt",
                ),
                control_route=_as_str(
                    fcu_controller.get("control_route"),
                    "mavlink_bootstrap_plus_dds_cmd_vel",
                ),
                mavlink_bootstrap_endpoint=_as_str(
                    fcu_controller.get("mavlink_bootstrap_endpoint"),
                    "udp:127.0.0.1:14550",
                ),
                mavlink_bootstrap_source_system=int(fcu_controller.get("mavlink_bootstrap_source_system", 246)),
                mavlink_bootstrap_source_component=int(
                    fcu_controller.get("mavlink_bootstrap_source_component", 190)
                ),
                owner_name=_as_str(fcu_controller.get("owner_name"), "navlab_fcu_controller"),
                owner_id=_as_str(fcu_controller.get("owner_id"), "navlab-p4-fcu-controller"),
                fcu_state_topic=_as_str(fcu_controller.get("fcu_state_topic"), "/navlab/fcu/state"),
                controller_status_topic=_as_str(
                    fcu_controller.get("controller_status_topic"),
                    "/navlab/fcu/controller/status",
                ),
                setpoint_intent_topic=_as_str(
                    fcu_controller.get("setpoint_intent_topic"),
                    "/navlab/fcu/setpoint/intent",
                ),
                setpoint_output_topic=_as_str(
                    fcu_controller.get("setpoint_output_topic"),
                    "/navlab/fcu/setpoint/output",
                ),
                owner_status_topic=_as_str(fcu_controller.get("owner_status_topic"), "/navlab/fcu/owner/status"),
                time_topic=_as_str(fcu_controller.get("time_topic"), "/ap/v1/time"),
                prearm_service=_as_str(fcu_controller.get("prearm_service"), "/ap/v1/prearm_check"),
                mode_switch_service=_as_str(fcu_controller.get("mode_switch_service"), "/ap/v1/mode_switch"),
                arm_service=_as_str(fcu_controller.get("arm_service"), "/ap/v1/arm_motors"),
                takeoff_service=_as_str(fcu_controller.get("takeoff_service"), "/ap/v1/experimental/takeoff"),
                cmd_vel_topic=_as_str(fcu_controller.get("cmd_vel_topic"), "/ap/v1/cmd_vel"),
                pose_topic=_as_str(fcu_controller.get("pose_topic"), "/ap/v1/pose/filtered"),
                twist_topic=_as_str(fcu_controller.get("twist_topic"), "/ap/v1/twist/filtered"),
                status_topic=_as_str(fcu_controller.get("status_topic"), "/ap/v1/status"),
                rangefinder_range_topic=_as_str(
                    fcu_controller.get("rangefinder_range_topic"),
                    "/rangefinder/down/range",
                ),
                rangefinder_status_topic=_as_str(
                    fcu_controller.get("rangefinder_status_topic"),
                    "/rangefinder/down/status",
                ),
                imu_topic=_as_str(fcu_controller.get("imu_topic"), "/imu"),
                slam_odom_topic=_as_str(fcu_controller.get("slam_odom_topic"), "/slam/odom"),
                slam_status_topic=_as_str(fcu_controller.get("slam_status_topic"), "/navlab/slam/status"),
                guided_mode=int(_as_float(fcu_controller.get("guided_mode"), 4.0)),
                takeoff_alt_m=_as_float(fcu_controller.get("takeoff_alt_m"), 0.5),
                readiness_timeout_sec=_as_float(fcu_controller.get("readiness_timeout_sec"), 45.0),
                hold_after_ready_sec=_as_float(fcu_controller.get("hold_after_ready_sec"), 8.0),
                require_slam_backend=_as_bool(fcu_controller.get("require_slam_backend"), True),
                hover_claim=_as_str(fcu_controller.get("hover_claim"), "not_evaluated"),
                exploration_claim=_as_str(fcu_controller.get("exploration_claim"), "not_evaluated"),
            ),
            frame_contract=FrameContractConfig(
                rosbag_profile=_as_str(
                    frame_contract.get("rosbag_profile"),
                    "profiles/navlab-frame-contract-rosbag-topics.txt",
                ),
                required_frames=_as_str_tuple(
                    frame_contract.get("required_frames"),
                    ("map", "odom", "base_link", "imu_link", "base_scan", "rangefinder_down_frame"),
                ),
                map_frame_id=_as_str(frame_contract.get("map_frame_id"), "map"),
                odom_frame_id=_as_str(frame_contract.get("odom_frame_id"), "odom"),
                base_frame_id=_as_str(frame_contract.get("base_frame_id"), "base_link"),
                imu_frame_id=_as_str(frame_contract.get("imu_frame_id"), "imu_link"),
                laser_frame_id=_as_str(frame_contract.get("laser_frame_id"), "base_scan"),
                rangefinder_frame_id=_as_str(frame_contract.get("rangefinder_frame_id"), "rangefinder_down_frame"),
                scan_topic=_as_str(frame_contract.get("scan_topic"), "/scan"),
                imu_topic=_as_str(frame_contract.get("imu_topic"), "/imu"),
                rangefinder_range_topic=_as_str(frame_contract.get("rangefinder_range_topic"), "/rangefinder/down/range"),
                rangefinder_status_topic=_as_str(
                    frame_contract.get("rangefinder_status_topic"),
                    "/rangefinder/down/status",
                ),
                fcu_pose_topic=_as_str(frame_contract.get("fcu_pose_topic"), "/ap/v1/pose/filtered"),
                fcu_twist_topic=_as_str(frame_contract.get("fcu_twist_topic"), "/ap/v1/twist/filtered"),
                fcu_status_topic=_as_str(frame_contract.get("fcu_status_topic"), "/ap/v1/status"),
                cmd_vel_topic=_as_str(frame_contract.get("cmd_vel_topic"), "/ap/v1/cmd_vel"),
                slam_odom_topic=_as_str(frame_contract.get("slam_odom_topic"), "/slam/odom"),
                slam_status_topic=_as_str(frame_contract.get("slam_status_topic"), "/navlab/slam/status"),
                truth_diagnostic_topic=_as_str(frame_contract.get("truth_diagnostic_topic"), "/odometry"),
                controller_status_topic=_as_str(
                    frame_contract.get("controller_status_topic"),
                    "/navlab/fcu/controller/status",
                ),
                setpoint_output_topic=_as_str(
                    frame_contract.get("setpoint_output_topic"),
                    "/navlab/fcu/setpoint/output",
                ),
                owner_status_topic=_as_str(frame_contract.get("owner_status_topic"), "/navlab/fcu/owner/status"),
                status_topic=_as_str(frame_contract.get("status_topic"), "/navlab/frame_contract/status"),
                max_dynamic_tf_age_sec=_as_float(frame_contract.get("max_dynamic_tf_age_sec"), 3.0),
                min_scan_valid_ratio=_as_float(frame_contract.get("min_scan_valid_ratio"), 0.05),
                max_rangefinder_height_error_m=_as_float(
                    frame_contract.get("max_rangefinder_height_error_m"),
                    0.35,
                ),
                max_direction_error_rad=_as_float(frame_contract.get("max_direction_error_rad"), 0.8),
                probe_duration_sec=_as_float(frame_contract.get("probe_duration_sec"), 16.0),
                require_motion_direction_check=_as_bool(
                    frame_contract.get("require_motion_direction_check"),
                    False,
                ),
                hover_claim=_as_str(frame_contract.get("hover_claim"), "not_evaluated"),
                exploration_claim=_as_str(frame_contract.get("exploration_claim"), "not_evaluated"),
                uses_gazebo_truth_as_input=_as_bool(frame_contract.get("uses_gazebo_truth_as_input"), False),
            ),
            foxglove_upload=FoxgloveUploadConfig(
                enabled=_as_bool(foxglove_upload.get("enabled"), True),
                api_url=_as_str(foxglove_upload.get("api_url"), "https://api.foxglove.dev/v1"),
                token_env=_as_str(foxglove_upload.get("token_env"), "FOXGLOVE_API_TOKEN"),
                project_id=_as_str(foxglove_upload.get("project_id"), ""),
                device_id=_as_str(foxglove_upload.get("device_id"), ""),
                device_name=_as_str(foxglove_upload.get("device_name"), "navlab_companion_sitl_gazebo"),
                key_prefix=_as_str(foxglove_upload.get("key_prefix"), "navlab"),
                filename_prefix=_as_str(foxglove_upload.get("filename_prefix"), "navlab"),
            ),
        )

    def compose_env(self) -> dict[str, str]:
        return {
            "SESSION_ID": self.session_id,
            "SITL_IMAGE": self.sitl_image,
            "ROUTER_IMAGE": self.router_image,
            "ROS_DOMAIN_ID": self.ros_domain_id,
            "ROUTER_DOWNSTREAM_ENDPOINTS": self.router_downstream_endpoints,
            "ROUTER_LISTEN": self.router_listen,
            "ROUTER_TCP_PORT": self.router_tcp_port,
            "SITL_UPSTREAM_ENDPOINT": self.sitl_upstream_endpoint,
            "SITL_ROUTER_ONLY": self.sitl_router_only,
            "SITL_MODEL": self.sitl_model,
            "SITL_SPEEDUP": self.sitl_speedup,
            "SITL_INSTANCE": self.sitl_instance,
            "SITL_HOME": self.sitl_home,
            "SITL_EXTRA_ARGS": shlex.join(self.sitl_extra_args),
            "GAZEBO_WORLD": self.gazebo_world,
            "NAVLAB_COMPANION_IMAGE": self.companion_image,
            "NAVLAB_SLAM_IMAGE": self.slam.image,
            "NAVLAB_GAZEBO_SENSOR_IMAGE": self.sensor.image,
            "X2_MODE": "runtime",
        }


@dataclass(frozen=True, slots=True)
class RunConfig:
    orchestration: OrchestrationConfig
    duration_sec: float = 90.0
    run_id: str = ""
    artifact_dir: Path = Path()

    @property
    def ros_domain_id(self) -> str:
        return self.orchestration.ros_domain_id

    @property
    def session_id(self) -> str:
        return self.orchestration.session_id

    @property
    def companion_image(self) -> str:
        return self.orchestration.companion_image

    @property
    def slam_image(self) -> str:
        return self.orchestration.slam.image

    @property
    def gazebo_sensor_image(self) -> str:
        return self.orchestration.sensor.image

    @property
    def scan_source(self) -> str:
        return self.orchestration.sensor.acceptance_scan_source

    @property
    def rosbag_profile(self) -> str:
        return self.orchestration.rosbag_profile

    @property
    def official_baseline_rosbag_profile(self) -> str:
        return self.orchestration.official_baseline.rosbag_profile

    @property
    def official_maze_x2_rosbag_profile(self) -> str:
        return self.orchestration.official_maze_x2.rosbag_profile

    @property
    def rangefinder_imu_rosbag_profile(self) -> str:
        return self.orchestration.rangefinder_imu.rosbag_profile

    @property
    def slam_backend_rosbag_profile(self) -> str:
        return self.orchestration.slam_backend.rosbag_profile

    @property
    def fcu_controller_rosbag_profile(self) -> str:
        return self.orchestration.fcu_controller.rosbag_profile

    @property
    def frame_contract_rosbag_profile(self) -> str:
        return self.orchestration.frame_contract.rosbag_profile

    @classmethod
    def from_config(
        cls,
        *,
        config_path: str | Path | None = None,
        duration_sec: float = 90.0,
        artifact_dir: str | Path | None = None,
        run_id: str | None = None,
    ) -> RunConfig:
        orchestration = OrchestrationConfig.load(config_path)
        final_run_id = run_id or os.environ.get("RUN_ID") or datetime.now().strftime("%Y%m%d_%H%M%S")
        final_artifact_dir = Path(
            artifact_dir or os.environ.get("ARTIFACT_DIR") or f"artifacts/ros/{orchestration.session_id}/{final_run_id}"
        )
        return cls(
            orchestration=orchestration,
            duration_sec=duration_sec,
            run_id=final_run_id,
            artifact_dir=final_artifact_dir,
        )


def resolve_config_path(path: str | Path | None = None) -> Path:
    raw = path or os.environ.get("NAVLAB_ORCHESTRATION_CONFIG") or DEFAULT_CONFIG
    return Path(raw).expanduser()


def _as_str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    return str(value)


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).lower() in {"1", "true", "yes", "on"}


def _as_float(value: Any, default: float) -> float:
    if value is None:
        return default
    return float(value)


def _as_args(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return tuple(shlex.split(value))
    if isinstance(value, list | tuple):
        return tuple(str(item) for item in value)
    raise TypeError(f"expected args list or string, got {type(value).__name__}")


def _as_scan_source(value: Any) -> str:
    scan_source = _as_str(value, "x2_virtual_serial")
    if scan_source not in {"gazebo_ideal", "x2_virtual_serial"}:
        raise ValueError("orchestration.sensor.scan_source must be gazebo_ideal or x2_virtual_serial")
    return scan_source


def _as_str_tuple(value: Any, default: tuple[str, ...] = ()) -> tuple[str, ...]:
    if value is None:
        return default
    if isinstance(value, str):
        return (value,)
    if isinstance(value, list | tuple):
        return tuple(str(item) for item in value)
    raise TypeError(f"expected string list, got {type(value).__name__}")


def _section(data: dict[str, Any], *keys: str) -> dict[str, Any]:
    value: Any = data
    for key in keys:
        value = value.get(key, {}) if isinstance(value, dict) else {}
    if not isinstance(value, dict):
        return {}
    return value
