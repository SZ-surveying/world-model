from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("cartographer_indoor")
    params_file = PathJoinSubstitution(
        [pkg_share, "config", "cartographer_indoor.params.yaml"]
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
                description="Fallback placeholder /odom for smoke tests only",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="cartographer_indoor_2d.lua",
                description="Cartographer Lua configuration file",
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
                package="cartographer_indoor",
                executable="cartographer_indoor_node",
                name="cartographer_indoor_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "publish_placeholder_odom": LaunchConfiguration(
                            "publish_placeholder_odom"
                        ),
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
                remappings=[("scan", "/scan"), ("imu", "/imu/data")],
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
