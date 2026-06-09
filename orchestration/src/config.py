from __future__ import annotations

import os
import shlex
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import tomllib

from src.project_config import (
    DEFAULT_RUNTIME_MODE,
    PROJECT_PATH,
    REPO_PATH,
    load_navlab_images_config,
    load_runtime_config,
)

DEFAULT_ORCHESTRATION_CONFIG_DIR = PROJECT_PATH
DEFAULT_TASK_CONFIG_DIR = PROJECT_PATH / "configs"
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

TASK_CONFIG_NAMES = {
    "hover": "hover",
    "exploration": "exploration",
    "exploration-doctor": "exploration",
    "scan-robustness": "scan_robustness",
    "scan-robustness-doctor": "scan_robustness",
    "real-preflight-doctor": "real_preflight",
    "real-prepare": "real_prepare",
}


@dataclass(frozen=True, slots=True)
class TaskInvocationConfig:
    task_name: str
    path: Path | None
    path_source: str
    duration_sec: float
    duration_source: str
    simulation_profile: str
    simulation_profile_source: str
    live_replay: bool
    live_replay_source: str
    live_profiles: tuple[str, ...]
    live_profiles_source: str

    def to_summary(self) -> dict[str, Any]:
        return {
            "duration_sec": {"value": self.duration_sec, "source": self.duration_source},
            "simulation_profile": {
                "value": self.simulation_profile,
                "source": self.simulation_profile_source,
            },
            "live_replay": {"value": self.live_replay, "source": self.live_replay_source},
            "live_profiles": {"value": list(self.live_profiles), "source": self.live_profiles_source},
        }


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
class SerialMavlinkConfig:
    enabled: bool
    port: str
    baud: int
    connection_timeout_sec: float
    heartbeat_timeout_sec: float
    telemetry_window_sec: float
    require_autopilot_heartbeat: bool
    require_system_status: bool
    require_not_armed: bool
    require_mode_observed: bool
    expected_autopilot: str
    required_messages: tuple[str, ...]
    optional_messages: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class RealPreflightConfig:
    valid_for_sec: float
    ros_distro: str
    serial_mavlink: SerialMavlinkConfig
    dependencies: RealPreflightDependencyConfig


@dataclass(frozen=True, slots=True)
class RealPreflightDependencyConfig:
    required_command_groups: tuple[tuple[str, ...], ...]
    required_ros_packages: tuple[str, ...]
    required_python_modules: tuple[str, ...]
    required_process_services: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class RealPrepareServiceConfig:
    enabled: bool
    required: bool
    command: tuple[str, ...]
    cwd: str
    env: dict[str, str]
    startup_timeout_sec: float
    health_topics: tuple[str, ...]
    shutdown_policy: str
    direct_serial_access_allowed: bool


@dataclass(frozen=True, slots=True)
class RealPrepareConfig:
    dry_run: bool
    process_log_dir: str
    summary_artifact_dir: str
    ros_topic_probe_timeout_sec: float
    topic_freshness_window_sec: float
    external_nav_yaw_required: bool
    external_nav_yaw_status_topics: tuple[str, ...]
    external_nav_yaw_ready_fields: tuple[str, ...]
    mavlink_router_serial_port: str
    mavlink_router_baud: int
    mavlink_router_local_endpoint: str
    fcu_bridge_state_topic: str
    required_upstream_topics: tuple[str, ...]
    forbidden_simulation_topics: tuple[str, ...]
    mavlink_router: RealPrepareServiceConfig
    mavros: RealPrepareServiceConfig
    lidar: RealPrepareServiceConfig
    slam: RealPrepareServiceConfig
    rangefinder_bridge: RealPrepareServiceConfig


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
class SlamHoverConfig:
    rosbag_profile: str
    slam_odom_topic: str
    slam_status_topic: str
    external_nav_status_topic: str
    fcu_pose_topic: str
    fcu_twist_topic: str
    fcu_status_topic: str
    cmd_vel_topic: str
    rangefinder_range_topic: str
    rangefinder_status_topic: str
    imu_topic: str
    truth_diagnostic_topic: str
    controller_status_topic: str
    setpoint_intent_topic: str
    setpoint_output_topic: str
    owner_status_topic: str
    hover_status_topic: str
    vehicle_marker_topic: str
    vehicle_marker_pose_topic: str
    vehicle_marker_frame_id: str
    vehicle_marker_rate_hz: float
    record_visualization_markers: bool
    settle_window_sec: float
    hover_window_sec: float
    final_hold_window_sec: float
    max_hover_horizontal_drift_m: float
    max_hover_altitude_error_m: float
    max_hover_yaw_drift_rad: float
    max_stop_drift_m: float
    min_slam_odom_rate_hz: float
    min_external_nav_rate_hz: float
    min_fcu_local_position_rate_hz: float
    max_latest_age_sec: float
    uses_gazebo_truth_as_input: bool
    hover_claim: str
    exploration_claim: str


@dataclass(frozen=True, slots=True)
class MotionGateConfig:
    rosbag_profile: str
    slam_odom_topic: str
    slam_status_topic: str
    external_nav_status_topic: str
    fcu_pose_topic: str
    fcu_twist_topic: str
    fcu_status_topic: str
    cmd_vel_topic: str
    rangefinder_range_topic: str
    rangefinder_status_topic: str
    imu_topic: str
    scan_topic: str
    truth_diagnostic_topic: str
    controller_status_topic: str
    setpoint_intent_topic: str
    setpoint_output_topic: str
    owner_status_topic: str
    hover_status_topic: str
    motion_status_topic: str
    settle_window_sec: float
    forward_window_sec: float
    back_window_sec: float
    yaw_window_sec: float
    stop_hold_window_sec: float
    final_hold_window_sec: float
    motion_distance_m: float
    motion_speed_mps: float
    yaw_scan_rad: float
    yaw_rate_radps: float
    min_forward_displacement_m: float
    max_forward_displacement_m: float
    min_back_displacement_m: float
    max_back_displacement_m: float
    min_yaw_delta_rad: float
    max_yaw_delta_rad: float
    max_lateral_error_m: float
    max_motion_altitude_error_m: float
    max_stop_drift_m: float
    min_clearance_m: float
    min_slam_odom_rate_hz: float
    min_external_nav_rate_hz: float
    min_fcu_local_position_rate_hz: float
    max_latest_age_sec: float
    uses_gazebo_truth_as_input: bool
    hover_claim: str
    motion_claim: str
    exploration_claim: str


@dataclass(frozen=True, slots=True)
class ExplorationGateConfig:
    rosbag_profile: str
    strategy: str
    slam_odom_topic: str
    slam_status_topic: str
    external_nav_status_topic: str
    map_topic: str
    submap_list_topic: str
    trajectory_node_list_topic: str
    fcu_pose_topic: str
    fcu_twist_topic: str
    fcu_status_topic: str
    cmd_vel_topic: str
    rangefinder_range_topic: str
    rangefinder_status_topic: str
    imu_topic: str
    scan_topic: str
    truth_diagnostic_topic: str
    controller_status_topic: str
    setpoint_intent_topic: str
    setpoint_output_topic: str
    owner_status_topic: str
    hover_status_topic: str
    motion_status_topic: str
    exploration_status_topic: str
    exploration_goal_topic: str
    exploration_coverage_topic: str
    exploration_frontiers_topic: str
    exploration_path_topic: str
    exploration_markers_topic: str
    settle_window_sec: float
    exploration_window_sec: float
    forward_probe_window_sec: float
    yaw_scan_window_sec: float
    stop_hold_window_sec: float
    final_hold_window_sec: float
    motion_speed_mps: float
    yaw_rate_radps: float
    min_accepted_goals: int
    min_path_length_m: float
    min_known_cell_growth: int
    max_stop_drift_m: float
    min_clearance_m: float
    stuck_timeout_sec: float
    min_slam_odom_rate_hz: float
    min_external_nav_rate_hz: float
    min_fcu_local_position_rate_hz: float
    max_latest_age_sec: float
    uses_gazebo_truth_as_input: bool
    hover_claim: str
    motion_claim: str
    exploration_claim: str


@dataclass(frozen=True, slots=True)
class ScanIntegrityGateConfig:
    rosbag_profile: str
    raw_scan_topic: str
    normalized_scan_topic: str
    validated_scan_topic: str
    status_topic: str
    events_topic: str
    fault_injection_topic: str
    attitude_source_topic: str
    attitude_source_type: str
    rangefinder_range_topic: str
    imu_topic: str
    fcu_pose_topic: str
    scan_source_topic: str
    x2_status_topic: str
    base_frame_id: str
    scan_frame_id: str
    soft_tilt_deg: float
    hard_tilt_deg: float
    max_dropped_scan_ratio: float
    max_clipped_beam_ratio: float
    max_scan_attitude_time_offset_ms: float
    max_attitude_source_age_ms: float
    min_attitude_rate_hz: float
    floor_hit_guard_range_m: float
    min_lidar_height_m: float
    min_downward_ray_z: float
    mild_fault_roll_bias_deg: float
    mild_fault_pitch_bias_deg: float
    hard_fault_roll_bias_deg: float
    hard_fault_pitch_bias_deg: float
    normal_window_sec: float
    fault_window_sec: float
    uses_gazebo_truth_as_input: bool
    hover_claim: str
    motion_claim: str
    exploration_claim: str
    scan_integrity_claim: str


@dataclass(frozen=True, slots=True)
class ScanStabilizationConfig:
    enabled: bool
    mode: str
    input_scan_topic: str
    output_scan_topic: str
    status_topic: str
    events_topic: str
    debug_scan_topic: str
    fault_injection_topic: str
    attitude_source_topic: str
    attitude_source_type: str
    range_topic: str
    base_frame_id: str
    scan_frame_id: str
    passthrough_tilt_deg: float
    compensation_tilt_deg: float
    hard_drop_tilt_deg: float
    max_vertical_projection_error_m: float
    max_rejected_beam_ratio: float
    min_retained_beam_ratio: float
    max_floor_hit_risk_beam_ratio: float
    floor_hit_guard_range_m: float
    min_lidar_height_m: float
    min_downward_ray_z: float
    max_scan_attitude_time_offset_ms: float
    max_attitude_source_age_ms: float
    min_attitude_rate_hz: float
    min_stabilized_scan_rate_hz: float
    publish_debug_scan: bool
    uses_gazebo_truth_as_input: bool
    scan_stabilization_claim: str


@dataclass(frozen=True, slots=True)
class ScanStabilizationGateConfig:
    rosbag_profile: str
    motion_profile: str
    baseline_mode: str
    candidate_mode: str
    raw_scan_topic: str
    normalized_scan_topic: str
    validated_scan_topic: str
    scan_source_topic: str
    x2_status_topic: str
    imu_topic: str
    fcu_pose_topic: str
    uses_official_maze_as_input: bool
    official_maze_layer_role: str
    hover_claim: str
    motion_claim: str
    exploration_claim: str
    scan_stabilization_claim: str
    replay_readiness_timeout_sec: float
    controller_summary_timeout_sec: float


@dataclass(frozen=True, slots=True)
class AirframeDisturbanceConfig:
    enabled: bool
    profile: str
    injection_layer: str
    seed: int
    motor_count: int
    thrust_multipliers: tuple[float, ...]
    max_abs_thrust_multiplier_delta: float
    esc_lag_ms: tuple[float, ...]
    esc_lag_model: str
    max_esc_lag_ms: float
    thrust_noise_std: float
    thrust_noise_correlation_ms: float
    motor_jitter_hz: float
    imu_vibration_enabled: bool
    imu_input_topic: str
    imu_output_topic: str
    imu_gyro_noise_std_dps: float
    imu_accel_noise_std_mps2: float
    imu_vibration_freq_hz: float
    imu_vibration_roll_pitch_amp_deg: float
    status_topic: str
    events_topic: str


@dataclass(frozen=True, slots=True)
class AirframeDisturbanceGateConfig:
    rosbag_profile: str
    motion_profile: str
    scan_contract: str
    profile_set: tuple[str, ...]
    required_profiles: tuple[str, ...]
    fault_profiles: tuple[str, ...]
    allow_hard_profile_fail: bool
    max_abs_roll_deg: float
    max_abs_pitch_deg: float
    max_rms_roll_deg: float
    max_rms_pitch_deg: float
    max_attitude_rate_dps: float
    max_scan_drop_ratio: float
    max_scan_compensated_ratio: float
    max_floor_hit_rejected_ratio: float
    min_stabilized_scan_rate_hz: float
    min_slam_odom_rate_hz: float
    max_map_artifact_score: float
    max_external_nav_dropout_ratio: float
    uses_official_maze_as_input: bool
    official_maze_layer_role: str
    fcu_status_topic: str
    fcu_status_mode_field: str
    fcu_mode_window_topic: str
    required_fcu_mode_name: str
    required_fcu_mode_number: int
    airframe_disturbance_claim: str
    horizontal_recovery_claim: str


@dataclass(frozen=True, slots=True)
class LandingConfig:
    enabled: bool
    default_policy: str
    hover_policy: str
    exploration_policy: str
    scan_robustness_policy: str
    landing_status_topic: str
    landing_intent_topic: str
    home_source: str
    home_radius_m: float
    pre_land_hold_sec: float
    max_return_home_duration_sec: float
    max_landing_duration_sec: float
    max_descent_rate_mps: float
    touchdown_altitude_m: float
    touchdown_vertical_speed_mps: float
    require_disarm: bool
    require_motors_safe: bool
    uses_gazebo_truth_as_input: bool

    def policy_for_task(self, task_name: str) -> str:
        if task_name == "hover":
            return self.hover_policy
        if task_name == "exploration":
            return self.exploration_policy
        if task_name == "scan-robustness":
            return self.scan_robustness_policy
        return self.default_policy


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
    slam_hover: SlamHoverConfig
    motion_gate: MotionGateConfig
    exploration_gate: ExplorationGateConfig
    scan_integrity_gate: ScanIntegrityGateConfig
    scan_stabilization: ScanStabilizationConfig
    scan_stabilization_gate: ScanStabilizationGateConfig
    airframe_disturbance: AirframeDisturbanceConfig
    airframe_disturbance_gate: AirframeDisturbanceGateConfig
    landing: LandingConfig
    real_preflight: RealPreflightConfig
    real_prepare: RealPrepareConfig
    foxglove_upload: FoxgloveUploadConfig
    orchestration_config_source: str = "mode default"
    task_name: str | None = None
    task_config_path: Path | None = None
    task_config_source: str = "none"

    @classmethod
    def load(
        cls,
        path: str | Path | None = None,
        *,
        task_name: str | None = None,
        task_config_path: str | Path | None = None,
    ) -> OrchestrationConfig:
        config_path = resolve_config_path(path)
        task_path, task_source = resolve_task_config_path(task_name, task_config_path)
        data = tomllib.loads(config_path.read_text(encoding="utf-8"))
        task_data = load_task_config_data(task_name, task_config_path=task_config_path)
        if task_data:
            data = _deep_merge_dicts(data, task_data)
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
        slam_hover = _section(data, "slam_hover")
        motion_gate = _section(data, "motion_gate")
        exploration_gate = _section(data, "exploration_gate")
        scan_integrity_gate = _section(data, "scan_integrity_gate")
        scan_stabilization = _section(data, "scan_stabilization")
        scan_stabilization_gate = _section(data, "scan_stabilization_gate")
        airframe_disturbance = _section(data, "airframe_disturbance")
        airframe_disturbance_gate = _section(data, "airframe_disturbance_gate")
        landing = _section(data, "landing")
        real_preflight = _section(data, "real_preflight")
        serial_mavlink = _section(data, "serial_mavlink")
        real_preflight_dependencies = _section(real_preflight, "dependencies")
        real_prepare = _section(data, "real_prepare")
        foxglove_upload = _section(data, "foxglove_upload")
        task = _optional_task_table(data, task_path)
        ros_domain_id = _as_str(data.get("ros_domain_id"), "85")
        serial_port = _as_str(serial_mavlink.get("port"), "/dev/ttyACM0")
        serial_baud = int(_as_float(serial_mavlink.get("baud"), 115200.0))
        router_endpoint = _as_str(real_prepare.get("mavlink_router_local_endpoint"), "127.0.0.1:14550")
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
            slam_hover=SlamHoverConfig(
                rosbag_profile=_as_str(
                    slam_hover.get("rosbag_profile"),
                    "profiles/navlab-slam-hover-rosbag-topics.txt",
                ),
                slam_odom_topic=_as_str(slam_hover.get("slam_odom_topic"), "/slam/odom"),
                slam_status_topic=_as_str(slam_hover.get("slam_status_topic"), "/navlab/slam/status"),
                external_nav_status_topic=_as_str(slam_hover.get("external_nav_status_topic"), "/external_nav/status"),
                fcu_pose_topic=_as_str(slam_hover.get("fcu_pose_topic"), "/ap/v1/pose/filtered"),
                fcu_twist_topic=_as_str(slam_hover.get("fcu_twist_topic"), "/ap/v1/twist/filtered"),
                fcu_status_topic=_as_str(slam_hover.get("fcu_status_topic"), "/ap/v1/status"),
                cmd_vel_topic=_as_str(slam_hover.get("cmd_vel_topic"), "/ap/v1/cmd_vel"),
                rangefinder_range_topic=_as_str(
                    slam_hover.get("rangefinder_range_topic"),
                    "/rangefinder/down/range",
                ),
                rangefinder_status_topic=_as_str(
                    slam_hover.get("rangefinder_status_topic"),
                    "/rangefinder/down/status",
                ),
                imu_topic=_as_str(slam_hover.get("imu_topic"), "/imu"),
                truth_diagnostic_topic=_as_str(slam_hover.get("truth_diagnostic_topic"), "/odometry"),
                controller_status_topic=_as_str(
                    slam_hover.get("controller_status_topic"),
                    "/navlab/fcu/controller/status",
                ),
                setpoint_intent_topic=_as_str(
                    slam_hover.get("setpoint_intent_topic"),
                    "/navlab/fcu/setpoint/intent",
                ),
                setpoint_output_topic=_as_str(
                    slam_hover.get("setpoint_output_topic"),
                    "/navlab/fcu/setpoint/output",
                ),
                owner_status_topic=_as_str(slam_hover.get("owner_status_topic"), "/navlab/fcu/owner/status"),
                hover_status_topic=_as_str(slam_hover.get("hover_status_topic"), "/navlab/hover/status"),
                vehicle_marker_topic=_as_str(slam_hover.get("vehicle_marker_topic"), "/navlab/vehicle/markers"),
                vehicle_marker_pose_topic=_as_str(
                    slam_hover.get("vehicle_marker_pose_topic"),
                    "/ap/v1/pose/filtered",
                ),
                vehicle_marker_frame_id=_as_str(slam_hover.get("vehicle_marker_frame_id"), ""),
                vehicle_marker_rate_hz=_as_float(slam_hover.get("vehicle_marker_rate_hz"), 10.0),
                record_visualization_markers=_as_bool(slam_hover.get("record_visualization_markers"), False),
                settle_window_sec=_as_float(slam_hover.get("settle_window_sec"), 8.0),
                hover_window_sec=_as_float(slam_hover.get("hover_window_sec"), 18.0),
                final_hold_window_sec=_as_float(slam_hover.get("final_hold_window_sec"), 5.0),
                max_hover_horizontal_drift_m=_as_float(
                    slam_hover.get("max_hover_horizontal_drift_m"),
                    0.35,
                ),
                max_hover_altitude_error_m=_as_float(slam_hover.get("max_hover_altitude_error_m"), 0.30),
                max_hover_yaw_drift_rad=_as_float(slam_hover.get("max_hover_yaw_drift_rad"), 0.45),
                max_stop_drift_m=_as_float(slam_hover.get("max_stop_drift_m"), 0.25),
                min_slam_odom_rate_hz=_as_float(slam_hover.get("min_slam_odom_rate_hz"), 1.0),
                min_external_nav_rate_hz=_as_float(slam_hover.get("min_external_nav_rate_hz"), 5.0),
                min_fcu_local_position_rate_hz=_as_float(
                    slam_hover.get("min_fcu_local_position_rate_hz"),
                    2.0,
                ),
                max_latest_age_sec=_as_float(slam_hover.get("max_latest_age_sec"), 1.5),
                uses_gazebo_truth_as_input=_as_bool(slam_hover.get("uses_gazebo_truth_as_input"), False),
                hover_claim=_as_str(slam_hover.get("hover_claim"), "evaluated"),
                exploration_claim=_as_str(slam_hover.get("exploration_claim"), "not_evaluated"),
            ),
            motion_gate=MotionGateConfig(
                rosbag_profile=_as_str(
                    motion_gate.get("rosbag_profile"),
                    "profiles/navlab-motion-gate-rosbag-topics.txt",
                ),
                slam_odom_topic=_as_str(motion_gate.get("slam_odom_topic"), "/slam/odom"),
                slam_status_topic=_as_str(motion_gate.get("slam_status_topic"), "/navlab/slam/status"),
                external_nav_status_topic=_as_str(
                    motion_gate.get("external_nav_status_topic"),
                    "/external_nav/status",
                ),
                fcu_pose_topic=_as_str(motion_gate.get("fcu_pose_topic"), "/ap/v1/pose/filtered"),
                fcu_twist_topic=_as_str(motion_gate.get("fcu_twist_topic"), "/ap/v1/twist/filtered"),
                fcu_status_topic=_as_str(motion_gate.get("fcu_status_topic"), "/ap/v1/status"),
                cmd_vel_topic=_as_str(motion_gate.get("cmd_vel_topic"), "/ap/v1/cmd_vel"),
                rangefinder_range_topic=_as_str(
                    motion_gate.get("rangefinder_range_topic"),
                    "/rangefinder/down/range",
                ),
                rangefinder_status_topic=_as_str(
                    motion_gate.get("rangefinder_status_topic"),
                    "/rangefinder/down/status",
                ),
                imu_topic=_as_str(motion_gate.get("imu_topic"), "/imu"),
                scan_topic=_as_str(motion_gate.get("scan_topic"), "/scan"),
                truth_diagnostic_topic=_as_str(motion_gate.get("truth_diagnostic_topic"), "/odometry"),
                controller_status_topic=_as_str(
                    motion_gate.get("controller_status_topic"),
                    "/navlab/fcu/controller/status",
                ),
                setpoint_intent_topic=_as_str(
                    motion_gate.get("setpoint_intent_topic"),
                    "/navlab/fcu/setpoint/intent",
                ),
                setpoint_output_topic=_as_str(
                    motion_gate.get("setpoint_output_topic"),
                    "/navlab/fcu/setpoint/output",
                ),
                owner_status_topic=_as_str(motion_gate.get("owner_status_topic"), "/navlab/fcu/owner/status"),
                hover_status_topic=_as_str(motion_gate.get("hover_status_topic"), "/navlab/hover/status"),
                motion_status_topic=_as_str(motion_gate.get("motion_status_topic"), "/navlab/motion/status"),
                settle_window_sec=_as_float(motion_gate.get("settle_window_sec"), 4.0),
                forward_window_sec=_as_float(motion_gate.get("forward_window_sec"), 4.0),
                back_window_sec=_as_float(motion_gate.get("back_window_sec"), 4.0),
                yaw_window_sec=_as_float(motion_gate.get("yaw_window_sec"), 3.0),
                stop_hold_window_sec=_as_float(motion_gate.get("stop_hold_window_sec"), 5.0),
                final_hold_window_sec=_as_float(motion_gate.get("final_hold_window_sec"), 8.0),
                motion_distance_m=_as_float(motion_gate.get("motion_distance_m"), 0.40),
                motion_speed_mps=_as_float(motion_gate.get("motion_speed_mps"), 0.12),
                yaw_scan_rad=_as_float(motion_gate.get("yaw_scan_rad"), 0.50),
                yaw_rate_radps=_as_float(motion_gate.get("yaw_rate_radps"), 0.20),
                min_forward_displacement_m=_as_float(
                    motion_gate.get("min_forward_displacement_m"),
                    0.20,
                ),
                max_forward_displacement_m=_as_float(
                    motion_gate.get("max_forward_displacement_m"),
                    0.80,
                ),
                min_back_displacement_m=_as_float(motion_gate.get("min_back_displacement_m"), 0.20),
                max_back_displacement_m=_as_float(motion_gate.get("max_back_displacement_m"), 0.80),
                min_yaw_delta_rad=_as_float(motion_gate.get("min_yaw_delta_rad"), 0.25),
                max_yaw_delta_rad=_as_float(motion_gate.get("max_yaw_delta_rad"), 0.90),
                max_lateral_error_m=_as_float(motion_gate.get("max_lateral_error_m"), 0.30),
                max_motion_altitude_error_m=_as_float(
                    motion_gate.get("max_motion_altitude_error_m"),
                    0.30,
                ),
                max_stop_drift_m=_as_float(motion_gate.get("max_stop_drift_m"), 0.25),
                min_clearance_m=_as_float(motion_gate.get("min_clearance_m"), 0.35),
                min_slam_odom_rate_hz=_as_float(motion_gate.get("min_slam_odom_rate_hz"), 1.0),
                min_external_nav_rate_hz=_as_float(motion_gate.get("min_external_nav_rate_hz"), 5.0),
                min_fcu_local_position_rate_hz=_as_float(
                    motion_gate.get("min_fcu_local_position_rate_hz"),
                    2.0,
                ),
                max_latest_age_sec=_as_float(motion_gate.get("max_latest_age_sec"), 1.5),
                uses_gazebo_truth_as_input=_as_bool(motion_gate.get("uses_gazebo_truth_as_input"), False),
                hover_claim=_as_str(motion_gate.get("hover_claim"), "evaluated"),
                motion_claim=_as_str(motion_gate.get("motion_claim"), "evaluated"),
                exploration_claim=_as_str(motion_gate.get("exploration_claim"), "not_evaluated"),
            ),
            exploration_gate=ExplorationGateConfig(
                rosbag_profile=_as_str(
                    exploration_gate.get("rosbag_profile"),
                    "profiles/navlab-exploration-gate-rosbag-topics.txt",
                ),
                strategy=_as_str(exploration_gate.get("strategy"), "frontier_lite"),
                slam_odom_topic=_as_str(exploration_gate.get("slam_odom_topic"), "/slam/odom"),
                slam_status_topic=_as_str(exploration_gate.get("slam_status_topic"), "/navlab/slam/status"),
                external_nav_status_topic=_as_str(
                    exploration_gate.get("external_nav_status_topic"),
                    "/external_nav/status",
                ),
                map_topic=_as_str(exploration_gate.get("map_topic"), "/map"),
                submap_list_topic=_as_str(exploration_gate.get("submap_list_topic"), "/submap_list"),
                trajectory_node_list_topic=_as_str(
                    exploration_gate.get("trajectory_node_list_topic"),
                    "/trajectory_node_list",
                ),
                fcu_pose_topic=_as_str(exploration_gate.get("fcu_pose_topic"), "/ap/v1/pose/filtered"),
                fcu_twist_topic=_as_str(exploration_gate.get("fcu_twist_topic"), "/ap/v1/twist/filtered"),
                fcu_status_topic=_as_str(exploration_gate.get("fcu_status_topic"), "/ap/v1/status"),
                cmd_vel_topic=_as_str(exploration_gate.get("cmd_vel_topic"), "/ap/v1/cmd_vel"),
                rangefinder_range_topic=_as_str(
                    exploration_gate.get("rangefinder_range_topic"),
                    "/rangefinder/down/range",
                ),
                rangefinder_status_topic=_as_str(
                    exploration_gate.get("rangefinder_status_topic"),
                    "/rangefinder/down/status",
                ),
                imu_topic=_as_str(exploration_gate.get("imu_topic"), "/imu"),
                scan_topic=_as_str(exploration_gate.get("scan_topic"), "/scan"),
                truth_diagnostic_topic=_as_str(exploration_gate.get("truth_diagnostic_topic"), "/odometry"),
                controller_status_topic=_as_str(
                    exploration_gate.get("controller_status_topic"),
                    "/navlab/fcu/controller/status",
                ),
                setpoint_intent_topic=_as_str(
                    exploration_gate.get("setpoint_intent_topic"),
                    "/navlab/fcu/setpoint/intent",
                ),
                setpoint_output_topic=_as_str(
                    exploration_gate.get("setpoint_output_topic"),
                    "/navlab/fcu/setpoint/output",
                ),
                owner_status_topic=_as_str(exploration_gate.get("owner_status_topic"), "/navlab/fcu/owner/status"),
                hover_status_topic=_as_str(exploration_gate.get("hover_status_topic"), "/navlab/hover/status"),
                motion_status_topic=_as_str(exploration_gate.get("motion_status_topic"), "/navlab/motion/status"),
                exploration_status_topic=_as_str(
                    exploration_gate.get("exploration_status_topic"),
                    "/navlab/exploration/status",
                ),
                exploration_goal_topic=_as_str(
                    exploration_gate.get("exploration_goal_topic"),
                    "/navlab/exploration/goal",
                ),
                exploration_coverage_topic=_as_str(
                    exploration_gate.get("exploration_coverage_topic"),
                    "/navlab/exploration/coverage",
                ),
                exploration_frontiers_topic=_as_str(
                    exploration_gate.get("exploration_frontiers_topic"),
                    "/navlab/exploration/frontiers",
                ),
                exploration_path_topic=_as_str(
                    exploration_gate.get("exploration_path_topic"),
                    "/navlab/exploration/path",
                ),
                exploration_markers_topic=_as_str(
                    exploration_gate.get("exploration_markers_topic"),
                    "/navlab/exploration/markers",
                ),
                settle_window_sec=_as_float(exploration_gate.get("settle_window_sec"), 4.0),
                exploration_window_sec=_as_float(exploration_gate.get("exploration_window_sec"), 26.0),
                forward_probe_window_sec=_as_float(exploration_gate.get("forward_probe_window_sec"), 3.0),
                yaw_scan_window_sec=_as_float(exploration_gate.get("yaw_scan_window_sec"), 3.0),
                stop_hold_window_sec=_as_float(exploration_gate.get("stop_hold_window_sec"), 4.0),
                final_hold_window_sec=_as_float(exploration_gate.get("final_hold_window_sec"), 8.0),
                motion_speed_mps=_as_float(exploration_gate.get("motion_speed_mps"), 0.10),
                yaw_rate_radps=_as_float(exploration_gate.get("yaw_rate_radps"), 0.18),
                min_accepted_goals=int(_as_float(exploration_gate.get("min_accepted_goals"), 3.0)),
                min_path_length_m=_as_float(exploration_gate.get("min_path_length_m"), 0.35),
                min_known_cell_growth=int(_as_float(exploration_gate.get("min_known_cell_growth"), 0.0)),
                max_stop_drift_m=_as_float(exploration_gate.get("max_stop_drift_m"), 0.30),
                min_clearance_m=_as_float(exploration_gate.get("min_clearance_m"), 0.35),
                stuck_timeout_sec=_as_float(exploration_gate.get("stuck_timeout_sec"), 8.0),
                min_slam_odom_rate_hz=_as_float(exploration_gate.get("min_slam_odom_rate_hz"), 1.0),
                min_external_nav_rate_hz=_as_float(exploration_gate.get("min_external_nav_rate_hz"), 5.0),
                min_fcu_local_position_rate_hz=_as_float(
                    exploration_gate.get("min_fcu_local_position_rate_hz"),
                    1.5,
                ),
                max_latest_age_sec=_as_float(exploration_gate.get("max_latest_age_sec"), 1.5),
                uses_gazebo_truth_as_input=_as_bool(exploration_gate.get("uses_gazebo_truth_as_input"), False),
                hover_claim=_as_str(exploration_gate.get("hover_claim"), "evaluated"),
                motion_claim=_as_str(exploration_gate.get("motion_claim"), "evaluated"),
                exploration_claim=_as_str(exploration_gate.get("exploration_claim"), "evaluated"),
            ),
            scan_integrity_gate=ScanIntegrityGateConfig(
                rosbag_profile=_as_str(
                    scan_integrity_gate.get("rosbag_profile"),
                    "profiles/navlab-scan-integrity-gate-rosbag-topics.txt",
                ),
                raw_scan_topic=_as_str(scan_integrity_gate.get("raw_scan_topic"), "/navlab/x2/scan_raw"),
                normalized_scan_topic=_as_str(
                    scan_integrity_gate.get("normalized_scan_topic"),
                    "/navlab/x2/scan_normalized",
                ),
                validated_scan_topic=_as_str(scan_integrity_gate.get("validated_scan_topic"), "/scan"),
                status_topic=_as_str(scan_integrity_gate.get("status_topic"), "/navlab/scan_integrity/status"),
                events_topic=_as_str(scan_integrity_gate.get("events_topic"), "/navlab/scan_integrity/events"),
                fault_injection_topic=_as_str(
                    scan_integrity_gate.get("fault_injection_topic"),
                    "/navlab/scan_integrity/fault_injection",
                ),
                attitude_source_topic=_as_str(scan_integrity_gate.get("attitude_source_topic"), "/imu"),
                attitude_source_type=_as_str(scan_integrity_gate.get("attitude_source_type"), "imu"),
                rangefinder_range_topic=_as_str(
                    scan_integrity_gate.get("rangefinder_range_topic"),
                    "/rangefinder/down/range",
                ),
                imu_topic=_as_str(scan_integrity_gate.get("imu_topic"), "/imu"),
                fcu_pose_topic=_as_str(scan_integrity_gate.get("fcu_pose_topic"), "/ap/v1/pose/filtered"),
                scan_source_topic=_as_str(scan_integrity_gate.get("scan_source_topic"), "/lidar"),
                x2_status_topic=_as_str(scan_integrity_gate.get("x2_status_topic"), "/sim/x2/status"),
                base_frame_id=_as_str(scan_integrity_gate.get("base_frame_id"), "base_link"),
                scan_frame_id=_as_str(scan_integrity_gate.get("scan_frame_id"), "base_scan"),
                soft_tilt_deg=_as_float(scan_integrity_gate.get("soft_tilt_deg"), 3.0),
                hard_tilt_deg=_as_float(scan_integrity_gate.get("hard_tilt_deg"), 6.0),
                max_dropped_scan_ratio=_as_float(scan_integrity_gate.get("max_dropped_scan_ratio"), 0.05),
                max_clipped_beam_ratio=_as_float(scan_integrity_gate.get("max_clipped_beam_ratio"), 0.20),
                max_scan_attitude_time_offset_ms=_as_float(
                    scan_integrity_gate.get("max_scan_attitude_time_offset_ms"),
                    50.0,
                ),
                max_attitude_source_age_ms=_as_float(
                    scan_integrity_gate.get("max_attitude_source_age_ms"),
                    250.0,
                ),
                min_attitude_rate_hz=_as_float(scan_integrity_gate.get("min_attitude_rate_hz"), 20.0),
                floor_hit_guard_range_m=_as_float(scan_integrity_gate.get("floor_hit_guard_range_m"), 8.0),
                min_lidar_height_m=_as_float(scan_integrity_gate.get("min_lidar_height_m"), 0.25),
                min_downward_ray_z=_as_float(scan_integrity_gate.get("min_downward_ray_z"), 0.05),
                mild_fault_roll_bias_deg=_as_float(scan_integrity_gate.get("mild_fault_roll_bias_deg"), 2.0),
                mild_fault_pitch_bias_deg=_as_float(scan_integrity_gate.get("mild_fault_pitch_bias_deg"), 0.0),
                hard_fault_roll_bias_deg=_as_float(scan_integrity_gate.get("hard_fault_roll_bias_deg"), 8.0),
                hard_fault_pitch_bias_deg=_as_float(scan_integrity_gate.get("hard_fault_pitch_bias_deg"), 0.0),
                normal_window_sec=_as_float(scan_integrity_gate.get("normal_window_sec"), 4.0),
                fault_window_sec=_as_float(scan_integrity_gate.get("fault_window_sec"), 4.0),
                uses_gazebo_truth_as_input=_as_bool(
                    scan_integrity_gate.get("uses_gazebo_truth_as_input"),
                    False,
                ),
                hover_claim=_as_str(scan_integrity_gate.get("hover_claim"), "evaluated"),
                motion_claim=_as_str(scan_integrity_gate.get("motion_claim"), "evaluated"),
                exploration_claim=_as_str(scan_integrity_gate.get("exploration_claim"), "not_evaluated"),
                scan_integrity_claim=_as_str(scan_integrity_gate.get("scan_integrity_claim"), "evaluated"),
            ),
            scan_stabilization=ScanStabilizationConfig(
                enabled=_as_bool(scan_stabilization.get("enabled"), True),
                mode=_as_str(scan_stabilization.get("mode"), "bounded_2d_projection"),
                input_scan_topic=_as_str(scan_stabilization.get("input_scan_topic"), "/navlab/x2/scan_normalized"),
                output_scan_topic=_as_str(scan_stabilization.get("output_scan_topic"), "/scan"),
                status_topic=_as_str(scan_stabilization.get("status_topic"), "/navlab/scan_stabilization/status"),
                events_topic=_as_str(scan_stabilization.get("events_topic"), "/navlab/scan_stabilization/events"),
                debug_scan_topic=_as_str(
                    scan_stabilization.get("debug_scan_topic"),
                    "/navlab/scan_stabilization/debug_scan",
                ),
                fault_injection_topic=_as_str(
                    scan_stabilization.get("fault_injection_topic"),
                    "/navlab/scan_stabilization/fault_injection",
                ),
                attitude_source_topic=_as_str(scan_stabilization.get("attitude_source_topic"), "/imu"),
                attitude_source_type=_as_str(scan_stabilization.get("attitude_source_type"), "imu"),
                range_topic=_as_str(scan_stabilization.get("range_topic"), "/rangefinder/down/range"),
                base_frame_id=_as_str(scan_stabilization.get("base_frame_id"), "base_link"),
                scan_frame_id=_as_str(scan_stabilization.get("scan_frame_id"), "base_scan"),
                passthrough_tilt_deg=_as_float(scan_stabilization.get("passthrough_tilt_deg"), 3.0),
                compensation_tilt_deg=_as_float(scan_stabilization.get("compensation_tilt_deg"), 8.0),
                hard_drop_tilt_deg=_as_float(scan_stabilization.get("hard_drop_tilt_deg"), 10.0),
                max_vertical_projection_error_m=_as_float(
                    scan_stabilization.get("max_vertical_projection_error_m"),
                    0.15,
                ),
                max_rejected_beam_ratio=_as_float(scan_stabilization.get("max_rejected_beam_ratio"), 0.35),
                min_retained_beam_ratio=_as_float(scan_stabilization.get("min_retained_beam_ratio"), 0.55),
                max_floor_hit_risk_beam_ratio=_as_float(
                    scan_stabilization.get("max_floor_hit_risk_beam_ratio"),
                    0.05,
                ),
                floor_hit_guard_range_m=_as_float(scan_stabilization.get("floor_hit_guard_range_m"), 8.0),
                min_lidar_height_m=_as_float(scan_stabilization.get("min_lidar_height_m"), 0.25),
                min_downward_ray_z=_as_float(scan_stabilization.get("min_downward_ray_z"), 0.05),
                max_scan_attitude_time_offset_ms=_as_float(
                    scan_stabilization.get("max_scan_attitude_time_offset_ms"),
                    50.0,
                ),
                max_attitude_source_age_ms=_as_float(
                    scan_stabilization.get("max_attitude_source_age_ms"),
                    250.0,
                ),
                min_attitude_rate_hz=_as_float(scan_stabilization.get("min_attitude_rate_hz"), 20.0),
                min_stabilized_scan_rate_hz=_as_float(
                    scan_stabilization.get("min_stabilized_scan_rate_hz"),
                    5.0,
                ),
                publish_debug_scan=_as_bool(scan_stabilization.get("publish_debug_scan"), False),
                uses_gazebo_truth_as_input=_as_bool(scan_stabilization.get("uses_gazebo_truth_as_input"), False),
                scan_stabilization_claim=_as_str(
                    scan_stabilization.get("scan_stabilization_claim"),
                    "evaluated",
                ),
            ),
            scan_stabilization_gate=ScanStabilizationGateConfig(
                rosbag_profile=_as_str(
                    scan_stabilization_gate.get("rosbag_profile"),
                    "profiles/navlab-scan-stabilization-gate-rosbag-topics.txt",
                ),
                motion_profile=_as_str(scan_stabilization_gate.get("motion_profile"), "p9_representative_replay"),
                baseline_mode=_as_str(scan_stabilization_gate.get("baseline_mode"), "p10_drop_only"),
                candidate_mode=_as_str(
                    scan_stabilization_gate.get("candidate_mode"),
                    "bounded_2d_projection",
                ),
                raw_scan_topic=_as_str(scan_stabilization_gate.get("raw_scan_topic"), "/navlab/x2/scan_raw"),
                normalized_scan_topic=_as_str(
                    scan_stabilization_gate.get("normalized_scan_topic"),
                    "/navlab/x2/scan_normalized",
                ),
                validated_scan_topic=_as_str(scan_stabilization_gate.get("validated_scan_topic"), "/scan"),
                scan_source_topic=_as_str(scan_stabilization_gate.get("scan_source_topic"), "/lidar"),
                x2_status_topic=_as_str(scan_stabilization_gate.get("x2_status_topic"), "/sim/x2/status"),
                imu_topic=_as_str(scan_stabilization_gate.get("imu_topic"), "/imu"),
                fcu_pose_topic=_as_str(scan_stabilization_gate.get("fcu_pose_topic"), "/ap/v1/pose/filtered"),
                uses_official_maze_as_input=_as_bool(
                    scan_stabilization_gate.get("uses_official_maze_as_input"),
                    False,
                ),
                official_maze_layer_role=_as_str(
                    scan_stabilization_gate.get("official_maze_layer_role"),
                    "visualization_only",
                ),
                hover_claim=_as_str(scan_stabilization_gate.get("hover_claim"), "evaluated"),
                motion_claim=_as_str(scan_stabilization_gate.get("motion_claim"), "evaluated"),
                exploration_claim=_as_str(scan_stabilization_gate.get("exploration_claim"), "evaluated"),
                scan_stabilization_claim=_as_str(
                    scan_stabilization_gate.get("scan_stabilization_claim"),
                    "evaluated",
                ),
                replay_readiness_timeout_sec=_as_float(
                    scan_stabilization_gate.get("replay_readiness_timeout_sec"),
                    90.0,
                ),
                controller_summary_timeout_sec=_as_float(
                    scan_stabilization_gate.get("controller_summary_timeout_sec"),
                    45.0,
                ),
            ),
            airframe_disturbance=AirframeDisturbanceConfig(
                enabled=_as_bool(airframe_disturbance.get("enabled"), True),
                profile=_as_str(airframe_disturbance.get("profile"), "nominal_realistic"),
                injection_layer=_as_str(airframe_disturbance.get("injection_layer"), "gazebo_motor_model"),
                seed=int(_as_float(airframe_disturbance.get("seed"), 12012.0)),
                motor_count=int(_as_float(airframe_disturbance.get("motor_count"), 4.0)),
                thrust_multipliers=_as_float_tuple(
                    airframe_disturbance.get("thrust_multipliers"),
                    (0.97, 1.03, 1.0, 0.98),
                ),
                max_abs_thrust_multiplier_delta=_as_float(
                    airframe_disturbance.get("max_abs_thrust_multiplier_delta"),
                    0.20,
                ),
                esc_lag_ms=_as_float_tuple(
                    airframe_disturbance.get("esc_lag_ms"),
                    (20.0, 35.0, 25.0, 45.0),
                ),
                esc_lag_model=_as_str(airframe_disturbance.get("esc_lag_model"), "first_order"),
                max_esc_lag_ms=_as_float(airframe_disturbance.get("max_esc_lag_ms"), 120.0),
                thrust_noise_std=_as_float(airframe_disturbance.get("thrust_noise_std"), 0.015),
                thrust_noise_correlation_ms=_as_float(
                    airframe_disturbance.get("thrust_noise_correlation_ms"),
                    80.0,
                ),
                motor_jitter_hz=_as_float(airframe_disturbance.get("motor_jitter_hz"), 35.0),
                imu_vibration_enabled=_as_bool(airframe_disturbance.get("imu_vibration_enabled"), True),
                imu_input_topic=_as_str(airframe_disturbance.get("imu_input_topic"), "/navlab/imu/raw"),
                imu_output_topic=_as_str(airframe_disturbance.get("imu_output_topic"), "/imu"),
                imu_gyro_noise_std_dps=_as_float(airframe_disturbance.get("imu_gyro_noise_std_dps"), 0.8),
                imu_accel_noise_std_mps2=_as_float(airframe_disturbance.get("imu_accel_noise_std_mps2"), 0.15),
                imu_vibration_freq_hz=_as_float(airframe_disturbance.get("imu_vibration_freq_hz"), 80.0),
                imu_vibration_roll_pitch_amp_deg=_as_float(
                    airframe_disturbance.get("imu_vibration_roll_pitch_amp_deg"),
                    0.4,
                ),
                status_topic=_as_str(
                    airframe_disturbance.get("status_topic"),
                    "/navlab/airframe_disturbance/status",
                ),
                events_topic=_as_str(
                    airframe_disturbance.get("events_topic"),
                    "/navlab/airframe_disturbance/events",
                ),
            ),
            airframe_disturbance_gate=AirframeDisturbanceGateConfig(
                rosbag_profile=_as_str(
                    airframe_disturbance_gate.get("rosbag_profile"),
                    "profiles/navlab-airframe-disturbance-gate-rosbag-topics.txt",
                ),
                motion_profile=_as_str(airframe_disturbance_gate.get("motion_profile"), "p9_representative_replay"),
                scan_contract=_as_str(airframe_disturbance_gate.get("scan_contract"), "p11_stabilized_scan"),
                profile_set=_as_str_tuple(
                    airframe_disturbance_gate.get("profile_set"),
                    ("clean", "mild_bias", "nominal_realistic", "hard_bias", "esc_lag", "vibration"),
                ),
                required_profiles=_as_str_tuple(
                    airframe_disturbance_gate.get("required_profiles"),
                    ("clean", "mild_bias", "nominal_realistic", "esc_lag", "vibration"),
                ),
                fault_profiles=_as_str_tuple(
                    airframe_disturbance_gate.get("fault_profiles"),
                    ("hard_bias", "invalid_config"),
                ),
                allow_hard_profile_fail=_as_bool(airframe_disturbance_gate.get("allow_hard_profile_fail"), True),
                max_abs_roll_deg=_as_float(airframe_disturbance_gate.get("max_abs_roll_deg"), 8.0),
                max_abs_pitch_deg=_as_float(airframe_disturbance_gate.get("max_abs_pitch_deg"), 8.0),
                max_rms_roll_deg=_as_float(airframe_disturbance_gate.get("max_rms_roll_deg"), 3.0),
                max_rms_pitch_deg=_as_float(airframe_disturbance_gate.get("max_rms_pitch_deg"), 3.0),
                max_attitude_rate_dps=_as_float(airframe_disturbance_gate.get("max_attitude_rate_dps"), 120.0),
                max_scan_drop_ratio=_as_float(airframe_disturbance_gate.get("max_scan_drop_ratio"), 0.20),
                max_scan_compensated_ratio=_as_float(
                    airframe_disturbance_gate.get("max_scan_compensated_ratio"),
                    0.80,
                ),
                max_floor_hit_rejected_ratio=_as_float(
                    airframe_disturbance_gate.get("max_floor_hit_rejected_ratio"),
                    0.05,
                ),
                min_stabilized_scan_rate_hz=_as_float(
                    airframe_disturbance_gate.get("min_stabilized_scan_rate_hz"),
                    5.0,
                ),
                min_slam_odom_rate_hz=_as_float(airframe_disturbance_gate.get("min_slam_odom_rate_hz"), 10.0),
                max_map_artifact_score=_as_float(airframe_disturbance_gate.get("max_map_artifact_score"), 0.15),
                max_external_nav_dropout_ratio=_as_float(
                    airframe_disturbance_gate.get("max_external_nav_dropout_ratio"),
                    0.05,
                ),
                uses_official_maze_as_input=_as_bool(
                    airframe_disturbance_gate.get("uses_official_maze_as_input"),
                    False,
                ),
                official_maze_layer_role=_as_str(
                    airframe_disturbance_gate.get("official_maze_layer_role"),
                    "visualization_only",
                ),
                fcu_status_topic=_as_str(airframe_disturbance_gate.get("fcu_status_topic"), "/ap/v1/status"),
                fcu_status_mode_field=_as_str(airframe_disturbance_gate.get("fcu_status_mode_field"), "mode"),
                fcu_mode_window_topic=_as_str(
                    airframe_disturbance_gate.get("fcu_mode_window_topic"),
                    "/navlab/exploration/status",
                ),
                required_fcu_mode_name=_as_str(airframe_disturbance_gate.get("required_fcu_mode_name"), "GUIDED"),
                required_fcu_mode_number=int(
                    _as_float(airframe_disturbance_gate.get("required_fcu_mode_number"), 4.0)
                ),
                airframe_disturbance_claim=_as_str(
                    airframe_disturbance_gate.get("airframe_disturbance_claim"),
                    "evaluated",
                ),
                horizontal_recovery_claim=_as_str(
                    airframe_disturbance_gate.get("horizontal_recovery_claim"),
                    "evaluated",
                ),
            ),
            landing=LandingConfig(
                enabled=_as_bool(landing.get("enabled"), True),
                default_policy=_as_str(landing.get("default_policy"), "land_in_place"),
                hover_policy=_as_str(landing.get("hover_policy"), "land_in_place"),
                exploration_policy=_as_str(landing.get("exploration_policy"), "return_home_then_land"),
                scan_robustness_policy=_as_str(landing.get("scan_robustness_policy"), "land_in_place"),
                landing_status_topic=_as_str(landing.get("landing_status_topic"), "/navlab/landing/status"),
                landing_intent_topic=_as_str(landing.get("landing_intent_topic"), "/navlab/landing/intent"),
                home_source=_as_str(landing.get("home_source"), "post_takeoff_hover_pose"),
                home_radius_m=_as_float(landing.get("home_radius_m"), 0.35),
                pre_land_hold_sec=_as_float(landing.get("pre_land_hold_sec"), 2.0),
                max_return_home_duration_sec=_as_float(landing.get("max_return_home_duration_sec"), 45.0),
                max_landing_duration_sec=_as_float(landing.get("max_landing_duration_sec"), 35.0),
                max_descent_rate_mps=_as_float(landing.get("max_descent_rate_mps"), 0.6),
                touchdown_altitude_m=_as_float(landing.get("touchdown_altitude_m"), 0.12),
                touchdown_vertical_speed_mps=_as_float(landing.get("touchdown_vertical_speed_mps"), 0.08),
                require_disarm=_as_bool(landing.get("require_disarm"), True),
                require_motors_safe=_as_bool(landing.get("require_motors_safe"), True),
                uses_gazebo_truth_as_input=_as_bool(landing.get("uses_gazebo_truth_as_input"), False),
            ),
            real_preflight=RealPreflightConfig(
                valid_for_sec=_as_float(
                    real_preflight.get("valid_for_sec", task.get("valid_for_sec")),
                    300.0,
                ),
                ros_distro=_as_str(real_preflight.get("ros_distro"), "jazzy"),
                serial_mavlink=SerialMavlinkConfig(
                    enabled=_as_bool(serial_mavlink.get("enabled"), False),
                    port=_as_str(serial_mavlink.get("port"), "/dev/ttyACM0"),
                    baud=int(_as_float(serial_mavlink.get("baud"), 115200.0)),
                    connection_timeout_sec=_as_float(serial_mavlink.get("connection_timeout_sec"), 3.0),
                    heartbeat_timeout_sec=_as_float(serial_mavlink.get("heartbeat_timeout_sec"), 5.0),
                    telemetry_window_sec=_as_float(serial_mavlink.get("telemetry_window_sec"), 8.0),
                    require_autopilot_heartbeat=_as_bool(
                        serial_mavlink.get("require_autopilot_heartbeat"),
                        True,
                    ),
                    require_system_status=_as_bool(serial_mavlink.get("require_system_status"), True),
                    require_not_armed=_as_bool(serial_mavlink.get("require_not_armed"), True),
                    require_mode_observed=_as_bool(serial_mavlink.get("require_mode_observed"), True),
                    expected_autopilot=_as_str(serial_mavlink.get("expected_autopilot"), "ardupilotmega"),
                    required_messages=tuple(
                        item.upper()
                        for item in _as_str_tuple(
                            serial_mavlink.get("required_messages"),
                            ("HEARTBEAT", "SYS_STATUS", "ATTITUDE"),
                        )
                    ),
                    optional_messages=tuple(
                        item.upper()
                        for item in _as_str_tuple(
                            serial_mavlink.get("optional_messages"),
                            (
                                "LOCAL_POSITION_NED",
                                "GLOBAL_POSITION_INT",
                                "RANGEFINDER",
                                "DISTANCE_SENSOR",
                                "HIGHRES_IMU",
                                "RAW_IMU",
                                "SCALED_IMU",
                            ),
                        )
                    ),
                ),
                dependencies=RealPreflightDependencyConfig(
                    required_command_groups=_as_str_group_tuple(
                        real_preflight_dependencies.get("required_command_groups"),
                        (
                            ("mavlink-routerd", "mavlink-router"),
                            ("ros2",),
                        ),
                    ),
                    required_ros_packages=_as_str_tuple(
                        real_preflight_dependencies.get("required_ros_packages"),
                        (
                            "mavros",
                            "mavros_msgs",
                            "navlab_slam_bringup",
                            "navlab_cartographer_adapter",
                            "navlab_external_nav_bridge",
                            "navlab_slam_imu_bridge",
                        ),
                    ),
                    required_python_modules=_as_str_tuple(
                        real_preflight_dependencies.get("required_python_modules"),
                        ("navlab.companion.cli", "navlab.slam.cli"),
                    ),
                    required_process_services=_as_str_tuple(
                        real_preflight_dependencies.get("required_process_services"),
                        (),
                    ),
                ),
            ),
            real_prepare=RealPrepareConfig(
                dry_run=_as_bool(real_prepare.get("dry_run"), False),
                process_log_dir=_as_str(
                    real_prepare.get("process_log_dir"),
                    "artifacts/runtime_logs/real_prepare",
                ),
                summary_artifact_dir=_as_str(
                    real_prepare.get("summary_artifact_dir"),
                    "artifacts/ros/navlab_real_prepare",
                ),
                ros_topic_probe_timeout_sec=_as_float(real_prepare.get("ros_topic_probe_timeout_sec"), 5.0),
                topic_freshness_window_sec=_as_float(real_prepare.get("topic_freshness_window_sec"), 2.0),
                external_nav_yaw_required=_as_bool(real_prepare.get("external_nav_yaw_required"), True),
                external_nav_yaw_status_topics=_as_str_tuple(
                    real_prepare.get("external_nav_yaw_status_topics"),
                    (
                        _as_str(slam_backend.get("external_nav_status_topic"), "/external_nav/status"),
                        _as_str(slam_backend.get("slam_status_topic"), "/navlab/slam/status"),
                    ),
                ),
                external_nav_yaw_ready_fields=_as_str_tuple(
                    real_prepare.get("external_nav_yaw_ready_fields"),
                    (
                        "external_nav_yaw_ready",
                        "yaw_ready",
                        "orientation_ready",
                    ),
                ),
                mavlink_router_serial_port=_as_str(
                    real_prepare.get("mavlink_router_serial_port"),
                    serial_port,
                ),
                mavlink_router_baud=int(
                    _as_float(real_prepare.get("mavlink_router_baud"), float(serial_baud)),
                ),
                mavlink_router_local_endpoint=router_endpoint,
                fcu_bridge_state_topic=_as_str(real_prepare.get("fcu_bridge_state_topic"), "/mavros/state"),
                required_upstream_topics=_as_str_tuple(
                    real_prepare.get("required_upstream_topics"),
                    (
                        "/scan",
                        "/tf",
                        "/tf_static",
                        "/slam/odom",
                        "/navlab/slam/status",
                        "/ap/v1/status",
                        "/ap/v1/pose/filtered",
                        "/ap/v1/twist/filtered",
                        "/mavros/state",
                    ),
                ),
                forbidden_simulation_topics=_as_str_tuple(
                    real_prepare.get("forbidden_simulation_topics"),
                    (
                        "/gazebo/*",
                        "/scan_ideal",
                        "/sim/x2/status",
                        "/rangefinder/down/scan_ideal",
                    ),
                ),
                mavlink_router=_real_prepare_service_from_config(
                    real_prepare,
                    "mavlink_router",
                    default_command=(
                        "mavlink-routerd",
                        "-e",
                        router_endpoint,
                        f"{serial_port}:{serial_baud}",
                    ),
                    default_health_topics=(),
                    default_shutdown_policy="stop_on_wrapper_exit",
                ),
                mavros=_real_prepare_service_from_config(
                    real_prepare,
                    "mavros",
                    default_command=(
                        "ros2",
                        "launch",
                        "mavros",
                        "apm.launch.py",
                        f"fcu_url:=udp://@{router_endpoint}",
                    ),
                    default_health_topics=("/mavros/state", "/ap/v1/status"),
                    default_shutdown_policy="stop_on_wrapper_exit",
                ),
                lidar=_real_prepare_service_from_config(
                    real_prepare,
                    "lidar",
                    default_command=("ros2", "launch", "ydlidar_ros2_driver", "ydlidar_launch.py"),
                    default_health_topics=("/scan",),
                    default_shutdown_policy="stop_on_wrapper_exit",
                ),
                slam=_real_prepare_service_from_config(
                    real_prepare,
                    "slam",
                    default_command=(
                        "ros2",
                        "launch",
                        "navlab_slam_bringup",
                        "cartographer.launch.py",
                        "use_sim_time:=false",
                    ),
                    default_health_topics=("/slam/odom", "/navlab/slam/status"),
                    default_shutdown_policy="stop_on_wrapper_exit",
                ),
                rangefinder_bridge=_real_prepare_service_from_config(
                    real_prepare,
                    "rangefinder_bridge",
                    default_enabled=False,
                    default_required=False,
                    default_command=(),
                    default_health_topics=("/rangefinder/down/range", "/rangefinder/down/status"),
                    default_shutdown_policy="stop_on_wrapper_exit",
                ),
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
            orchestration_config_source="CLI --orchestration-config" if path else "mode default",
            task_name=task_name,
            task_config_path=task_path if task_path and task_path.is_file() else None,
            task_config_source=task_source,
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

    @property
    def slam_hover_rosbag_profile(self) -> str:
        return self.orchestration.slam_hover.rosbag_profile

    @property
    def motion_gate_rosbag_profile(self) -> str:
        return self.orchestration.motion_gate.rosbag_profile

    @property
    def exploration_gate_rosbag_profile(self) -> str:
        return self.orchestration.exploration_gate.rosbag_profile

    @property
    def scan_integrity_gate_rosbag_profile(self) -> str:
        return self.orchestration.scan_integrity_gate.rosbag_profile

    @property
    def scan_stabilization_gate_rosbag_profile(self) -> str:
        return self.orchestration.scan_stabilization_gate.rosbag_profile

    @property
    def airframe_disturbance_gate_rosbag_profile(self) -> str:
        return self.orchestration.airframe_disturbance_gate.rosbag_profile

    def config_sources_summary(self) -> dict[str, Any]:
        task_config = {
            "task_name": self.orchestration.task_name,
            "path": str(self.orchestration.task_config_path) if self.orchestration.task_config_path else None,
            "source": self.orchestration.task_config_source,
        }
        return {
            "orchestration_config": {
                "path": str(self.orchestration.path),
                "source": self.orchestration.orchestration_config_source,
            },
            "task_config": task_config,
            "runtime": {
                "backend": os.environ.get("NAVLAB_RUNTIME_BACKEND") or "docker",
                "backend_source": "NAVLAB_RUNTIME_BACKEND" if os.environ.get("NAVLAB_RUNTIME_BACKEND") else "default",
                "mode": os.environ.get("NAVLAB_RUNTIME_MODE") or DEFAULT_RUNTIME_MODE,
                "mode_source": "NAVLAB_RUNTIME_MODE" if os.environ.get("NAVLAB_RUNTIME_MODE") else "default",
            },
        }

    @classmethod
    def from_config(
        cls,
        *,
        config_path: str | Path | None = None,
        task_name: str | None = None,
        task_config_path: str | Path | None = None,
        duration_sec: float = 90.0,
        artifact_dir: str | Path | None = None,
        run_id: str | None = None,
    ) -> RunConfig:
        orchestration = OrchestrationConfig.load(
            config_path,
            task_name=task_name,
            task_config_path=task_config_path,
        )
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
    if path:
        return Path(path).expanduser()
    return default_orchestration_config_path()


def default_orchestration_config_path() -> Path:
    return DEFAULT_ORCHESTRATION_CONFIG_DIR / f"config.{_runtime_mode_for_default_config()}.toml"


def _runtime_mode_for_default_config() -> str:
    raw_mode = os.environ.get("NAVLAB_RUNTIME_MODE")
    if raw_mode in (None, ""):
        return DEFAULT_RUNTIME_MODE
    mode = raw_mode.strip().lower()
    if mode not in {"simulation", "real"}:
        raise ValueError(f"Invalid orchestration runtime mode '{mode}': expected simulation or real")
    return mode


def default_task_config_path(task_name: str) -> Path:
    return DEFAULT_TASK_CONFIG_DIR / f"{TASK_CONFIG_NAMES.get(task_name, task_name.replace('-', '_'))}.toml"


def resolve_task_config_path(task_name: str | None, path: str | Path | None = None) -> tuple[Path | None, str]:
    if path:
        return Path(path).expanduser(), "CLI --task-config"
    if not task_name:
        return None, "none"
    return default_task_config_path(task_name), "task default"


def load_task_config_data(task_name: str | None, *, task_config_path: str | Path | None = None) -> dict[str, Any]:
    path, source = resolve_task_config_path(task_name, task_config_path)
    if path is None:
        return {}
    if not path.is_file():
        if source == "CLI --task-config":
            raise FileNotFoundError(f"Task config does not exist: {path}")
        return {}
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Invalid task config in {path}")
    return data


def load_task_invocation_config(
    task_name: str,
    *,
    task_config_path: str | Path | None = None,
    cli_duration_sec: float | None = None,
    default_duration_sec: float = 90.0,
    cli_simulation_profile: str | None = None,
    default_simulation_profile: str = "ideal",
    cli_live_replay: bool | None = None,
    default_live_replay: bool = True,
    cli_live_profiles: tuple[str, ...] | None = None,
    default_live_profiles: tuple[str, ...] = (),
) -> TaskInvocationConfig:
    path, path_source = resolve_task_config_path(task_name, task_config_path)
    data = load_task_config_data(task_name, task_config_path=task_config_path)
    task = _optional_task_table(data, path)
    duration_sec, duration_source = _resolve_task_float(
        task,
        "duration_sec",
        cli_duration_sec,
        default_duration_sec,
    )
    simulation_profile, simulation_profile_source = _resolve_task_str(
        task,
        "simulation_profile",
        cli_simulation_profile,
        default_simulation_profile,
    )
    live_replay, live_replay_source = _resolve_task_bool(
        task,
        "live",
        cli_live_replay,
        default_live_replay,
    )
    live_profiles, live_profiles_source = _resolve_task_str_tuple(
        task,
        "live_profiles",
        cli_live_profiles,
        default_live_profiles,
    )
    return TaskInvocationConfig(
        task_name=task_name,
        path=path if path and path.is_file() else None,
        path_source=path_source,
        duration_sec=duration_sec,
        duration_source=duration_source,
        simulation_profile=simulation_profile,
        simulation_profile_source=simulation_profile_source,
        live_replay=live_replay,
        live_replay_source=live_replay_source,
        live_profiles=live_profiles,
        live_profiles_source=live_profiles_source,
    )


def _deep_merge_dicts(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge_dicts(merged[key], value)
        else:
            merged[key] = value
    return merged


def _optional_task_table(data: dict[str, Any], path: Path | None) -> dict[str, Any]:
    raw_task = data.get("task", {})
    if raw_task is None:
        raw_task = {}
    if not isinstance(raw_task, dict):
        location = str(path) if path else "task config"
        raise ValueError(f"Invalid [task] section in {location}")
    return raw_task


def _resolve_task_float(
    task: dict[str, Any],
    key: str,
    cli_value: float | None,
    default: float,
) -> tuple[float, str]:
    if cli_value is not None:
        return float(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return float(task[key]), "task config"
    return default, "default"


def _resolve_task_str(
    task: dict[str, Any],
    key: str,
    cli_value: str | None,
    default: str,
) -> tuple[str, str]:
    if cli_value not in (None, ""):
        return str(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return str(task[key]), "task config"
    return default, "default"


def _resolve_task_bool(
    task: dict[str, Any],
    key: str,
    cli_value: bool | None,
    default: bool,
) -> tuple[bool, str]:
    if cli_value is not None:
        return bool(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return _as_bool(task[key], default), "task config"
    return default, "default"


def _resolve_task_str_tuple(
    task: dict[str, Any],
    key: str,
    cli_value: tuple[str, ...] | None,
    default: tuple[str, ...],
) -> tuple[tuple[str, ...], str]:
    if cli_value is not None:
        return tuple(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return _as_str_tuple(task[key], default), "task config"
    return default, "default"


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


def _as_float_tuple(value: Any, default: tuple[float, ...] = ()) -> tuple[float, ...]:
    if value is None:
        return default
    if isinstance(value, int | float):
        return (float(value),)
    if isinstance(value, str):
        if not value.strip():
            return default
        return tuple(float(item.strip()) for item in value.split(",") if item.strip())
    if isinstance(value, list | tuple):
        return tuple(float(item) for item in value)
    raise TypeError(f"expected float list, got {type(value).__name__}")


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


def _as_str_group_tuple(value: Any, default: tuple[tuple[str, ...], ...] = ()) -> tuple[tuple[str, ...], ...]:
    if value is None:
        return default
    if isinstance(value, str):
        return ((value,),)
    if isinstance(value, list | tuple):
        groups: list[tuple[str, ...]] = []
        for item in value:
            if isinstance(item, str):
                groups.append((item,))
            elif isinstance(item, list | tuple):
                group = tuple(str(group_item) for group_item in item if str(group_item))
                if group:
                    groups.append(group)
            else:
                raise TypeError(f"expected string command group, got {type(item).__name__}")
        return tuple(groups)
    raise TypeError(f"expected string group list, got {type(value).__name__}")


def _real_prepare_service_from_config(
    real_prepare: dict[str, Any],
    name: str,
    *,
    default_command: tuple[str, ...],
    default_health_topics: tuple[str, ...],
    default_shutdown_policy: str,
    default_enabled: bool = True,
    default_required: bool = True,
) -> RealPrepareServiceConfig:
    service = _section(real_prepare, name)
    raw_env = service.get("env", {})
    if raw_env is None:
        raw_env = {}
    if not isinstance(raw_env, dict) or not all(isinstance(key, str) and isinstance(value, str) for key, value in raw_env.items()):
        raise ValueError(f"Invalid [real_prepare.{name}.env] section: expected string table")
    return RealPrepareServiceConfig(
        enabled=_as_bool(service.get("enabled"), default_enabled),
        required=_as_bool(service.get("required"), default_required),
        command=_as_args(service.get("command")) or default_command,
        cwd=_as_str(service.get("cwd"), ""),
        env=dict(raw_env),
        startup_timeout_sec=_as_float(service.get("startup_timeout_sec"), 8.0),
        health_topics=_as_str_tuple(service.get("health_topics"), default_health_topics),
        shutdown_policy=_as_str(service.get("shutdown_policy"), default_shutdown_policy),
        direct_serial_access_allowed=_as_bool(service.get("direct_serial_access_allowed"), False),
    )


def _section(data: dict[str, Any], *keys: str) -> dict[str, Any]:
    value: Any = data
    for key in keys:
        value = value.get(key, {}) if isinstance(value, dict) else {}
    if not isinstance(value, dict):
        return {}
    return value
