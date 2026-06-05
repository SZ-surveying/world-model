from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("navlab_cartographer_adapter")
    params_file = PathJoinSubstitution(
        [pkg_share, "config", "navlab_cartographer_adapter.params.yaml"]
    )
    config_dir = PathJoinSubstitution([pkg_share, "config"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_cartographer_backend",
                default_value="false",
                description="Launch cartographer_ros backend nodes",
            ),
            DeclareLaunchArgument(
                "publish_placeholder_odom",
                default_value="false",
                description="Fallback placeholder odometry for smoke tests only",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
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
                description="IMU topic consumed by the SLAM backend",
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
                "status_topic",
                default_value="/navlab/slam/status",
                description="Structured SLAM status topic",
            ),
            DeclareLaunchArgument(
                "laser_frame",
                default_value="laser_frame",
                description="LiDAR frame used by static TF publisher",
            ),
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
                "resolution",
                default_value="0.05",
                description="Occupancy grid resolution",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_static_tf",
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
                package="navlab_cartographer_adapter",
                executable="navlab_cartographer_adapter_node",
                name="navlab_cartographer_adapter_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "publish_placeholder_odom": LaunchConfiguration(
                            "publish_placeholder_odom"
                        ),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
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
                    config_dir,
                    "-configuration_basename",
                    LaunchConfiguration("configuration_basename"),
                ],
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
                arguments=[
                    "-resolution",
                    LaunchConfiguration("resolution"),
                ],
            ),
        ]
    )
