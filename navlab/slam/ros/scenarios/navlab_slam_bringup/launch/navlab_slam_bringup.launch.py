from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_bridge_params = PathJoinSubstitution(
        [FindPackageShare("navlab_slam_imu_bridge"), "config", "navlab_slam_imu_bridge.params.yaml"]
    )
    cartographer_params = PathJoinSubstitution(
        [
            FindPackageShare("navlab_cartographer_adapter"),
            "config",
            "navlab_cartographer_adapter.params.yaml",
        ]
    )
    cartographer_config_dir = PathJoinSubstitution(
        [FindPackageShare("navlab_cartographer_adapter"), "config"]
    )
    external_nav_params = PathJoinSubstitution(
        [
            FindPackageShare("navlab_external_nav_bridge"),
            "config",
            "navlab_external_nav_bridge.params.yaml",
        ]
    )
    fake_odom_params = PathJoinSubstitution(
        [
            FindPackageShare("navlab_fake_odom"),
            "config",
            "navlab_fake_odom.params.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo/ROS simulation clock for all SLAM runtime nodes",
            ),
            DeclareLaunchArgument(
                "imu_source_mode",
                default_value="topic",
                description="imu_bridge source mode: topic or placeholder",
            ),
            DeclareLaunchArgument(
                "imu_source_topic",
                default_value="/ap/imu/experimental/data",
                description="Upstream FCU IMU topic for imu_bridge",
            ),
            DeclareLaunchArgument(
                "imu_source_label",
                default_value="ardupilot_dds",
                description="Human-readable IMU source label",
            ),
            DeclareLaunchArgument(
                "imu_min_input_rate_hz",
                default_value="4.0",
                description="Minimum accepted upstream IMU rate for imu_bridge readiness",
            ),
            DeclareLaunchArgument(
                "publish_placeholder_odom",
                default_value="false",
                description="Publish placeholder odometry for smoke tests only",
            ),
            DeclareLaunchArgument(
                "launch_fake_odom",
                default_value="false",
                description="Launch navlab_fake_odom as the odometry source",
            ),
            DeclareLaunchArgument(
                "fake_odom_mode",
                default_value="static",
                description="navlab_fake_odom mode: static, line, or yaw",
            ),
            DeclareLaunchArgument(
                "launch_cartographer_backend",
                default_value="false",
                description="Launch cartographer_ros backend nodes",
            ),
            DeclareLaunchArgument(
                "cartographer_configuration_basename",
                default_value="navlab_cartographer_2d.lua",
                description="Cartographer Lua configuration file",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan topic consumed by the SLAM backend",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu",
                description="Normalized IMU topic consumed by the SLAM backend",
            ),
            DeclareLaunchArgument(
                "cartographer_odometry_topic",
                default_value="/odometry",
                description="Official ArduPilot Cartographer odometry input remapped from /odom",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Odometry topic emitted by the SLAM adapter",
            ),
            DeclareLaunchArgument(
                "slam_status_topic",
                default_value="/navlab/slam/status",
                description="Structured SLAM backend status topic",
            ),
            DeclareLaunchArgument(
                "external_nav_status_topic",
                default_value="/external_nav/status",
                description="Structured ExternalNav bridge status topic",
            ),
            DeclareLaunchArgument(
                "laser_frame",
                default_value="laser_frame",
                description="LiDAR frame used by static TF publisher",
            ),
            DeclareLaunchArgument("laser_x", default_value="0", description="LiDAR static TF x offset"),
            DeclareLaunchArgument("laser_y", default_value="0", description="LiDAR static TF y offset"),
            DeclareLaunchArgument("laser_z", default_value="0", description="LiDAR static TF z offset"),
            DeclareLaunchArgument("laser_roll", default_value="0", description="LiDAR static TF roll"),
            DeclareLaunchArgument("laser_pitch", default_value="0", description="LiDAR static TF pitch"),
            DeclareLaunchArgument("laser_yaw", default_value="0", description="LiDAR static TF yaw"),
            DeclareLaunchArgument(
                "imu_frame",
                default_value="imu_link",
                description="IMU frame used by static TF publisher",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Base frame used by static TF publishers",
            ),
            DeclareLaunchArgument(
                "require_imu_for_external_nav",
                default_value="false",
                description="Require fresh /imu/data before external nav output",
            ),
            DeclareLaunchArgument(
                "external_nav_input_odom_topic",
                default_value="/odom",
                description="Input odometry topic consumed by external_nav_bridge",
            ),
            DeclareLaunchArgument(
                "require_height_for_external_nav",
                default_value="false",
                description="Require fresh /height/estimate before external nav output",
            ),
            Node(
                package="navlab_slam_imu_bridge",
                executable="navlab_slam_imu_bridge_node",
                name="navlab_slam_imu_bridge_node",
                output="screen",
                parameters=[
                    imu_bridge_params,
                    {
                        "source_mode": LaunchConfiguration("imu_source_mode"),
                        "source_topic": LaunchConfiguration("imu_source_topic"),
                        "source_label": LaunchConfiguration("imu_source_label"),
                        "min_input_rate_hz": LaunchConfiguration("imu_min_input_rate_hz"),
                        "output_topic": LaunchConfiguration("imu_topic"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_static_tf",
                output="screen",
                arguments=[
                    "--x",
                    LaunchConfiguration("laser_x"),
                    "--y",
                    LaunchConfiguration("laser_y"),
                    "--z",
                    LaunchConfiguration("laser_z"),
                    "--roll",
                    LaunchConfiguration("laser_roll"),
                    "--pitch",
                    LaunchConfiguration("laser_pitch"),
                    "--yaw",
                    LaunchConfiguration("laser_yaw"),
                    "--frame-id",
                    LaunchConfiguration("base_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("laser_frame"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_static_tf",
                output="screen",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    LaunchConfiguration("base_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("imu_frame"),
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("launch_fake_odom")),
                package="navlab_fake_odom",
                executable="navlab_fake_odom_node",
                name="navlab_fake_odom_node",
                output="screen",
                parameters=[
                    fake_odom_params,
                    {
                        "mode": LaunchConfiguration("fake_odom_mode"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                condition=UnlessCondition(LaunchConfiguration("launch_fake_odom")),
                package="navlab_cartographer_adapter",
                executable="navlab_cartographer_adapter_node",
                name="navlab_cartographer_adapter_node",
                output="screen",
                parameters=[
                    cartographer_params,
                    {
                        "publish_placeholder_odom": LaunchConfiguration("publish_placeholder_odom"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "status_topic": LaunchConfiguration("slam_status_topic"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("launch_cartographer_backend")),
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                arguments=[
                    "-configuration_directory",
                    cartographer_config_dir,
                    "-configuration_basename",
                    LaunchConfiguration("cartographer_configuration_basename"),
                ],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[
                    ("scan", LaunchConfiguration("scan_topic")),
                    ("imu", LaunchConfiguration("imu_topic")),
                    ("/odom", LaunchConfiguration("cartographer_odometry_topic")),
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("launch_cartographer_backend")),
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                arguments=["-resolution", "0.05"],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="navlab_external_nav_bridge",
                executable="navlab_external_nav_bridge_node",
                name="navlab_external_nav_bridge_node",
                output="screen",
                parameters=[
                    external_nav_params,
                    {
                        "input_odom_topic": LaunchConfiguration(
                            "external_nav_input_odom_topic"
                        ),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "status_topic": LaunchConfiguration("external_nav_status_topic"),
                        "require_imu_for_output": LaunchConfiguration(
                            "require_imu_for_external_nav"
                        ),
                        "require_height_for_output": LaunchConfiguration(
                            "require_height_for_external_nav"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
        ]
    )
