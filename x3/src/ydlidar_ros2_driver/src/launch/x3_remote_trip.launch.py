#!/usr/bin/python3

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _split_topics(value):
    return [topic.strip() for topic in value.split(",") if topic.strip()]


def _launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory("ydlidar_ros2_driver")
    params_file = LaunchConfiguration("params_file").perform(context)
    foxglove_address = LaunchConfiguration("foxglove_address").perform(context)
    foxglove_port = LaunchConfiguration("foxglove_port").perform(context)
    bag_storage = LaunchConfiguration("bag_storage").perform(context)
    bag_root = os.path.expanduser(LaunchConfiguration("bag_root").perform(context))
    with_features = _as_bool(LaunchConfiguration("with_features").perform(context))
    record_all = _as_bool(LaunchConfiguration("record_all").perform(context))
    extra_topics = _split_topics(LaunchConfiguration("extra_topics").perform(context))

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = os.path.join(bag_root, f"x3_trip_{run_id}")
    os.makedirs(bag_root, exist_ok=True)

    actions = [
        LogInfo(msg=f"Foxglove websocket: ws://<robot-ip>:{foxglove_port}"),
        LogInfo(msg=f"rosbag output: {bag_path} (storage: {bag_storage})"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share_dir, "launch", "ydlidar_launch.py")),
            launch_arguments={"params_file": params_file}.items(),
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "address": foxglove_address,
                    "port": int(foxglove_port),
                }
            ],
        ),
    ]

    if with_features:
        actions.append(
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_scan_features",
                name="ydlidar_ros2_scan_features",
                output="screen",
                emulate_tty=True,
            )
        )

    if record_all:
        bag_cmd = ["ros2", "bag", "record", "-s", bag_storage, "-a", "-o", bag_path]
    else:
        topics = ["/scan", "/point_cloud", "/tf", "/tf_static", "/rosout"]
        if with_features:
            topics.extend(["/scan_features", "/scan_nearest_point"])
        topics.extend(extra_topics)
        bag_cmd = ["ros2", "bag", "record", "-s", bag_storage, "-o", bag_path] + topics

    actions.append(ExecuteProcess(cmd=bag_cmd, output="screen"))
    return actions


def generate_launch_description():
    share_dir = get_package_share_directory("ydlidar_ros2_driver")
    default_params = os.path.join(share_dir, "params", "X2.yaml")
    default_bag_root = os.path.join(os.path.expanduser("~"), "workspace", "world-model", "x3", "bags")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the lidar parameter file.",
            ),
            DeclareLaunchArgument(
                "foxglove_address",
                default_value="0.0.0.0",
                description="Foxglove bridge bind address.",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="Foxglove bridge port.",
            ),
            DeclareLaunchArgument(
                "bag_storage",
                default_value="mcap",
                description="rosbag2 storage backend, defaults to mcap.",
            ),
            DeclareLaunchArgument(
                "bag_root",
                default_value=default_bag_root,
                description="Directory where rosbag2 output folders are created.",
            ),
            DeclareLaunchArgument(
                "with_features",
                default_value="true",
                description="Whether to start the /scan_features node.",
            ),
            DeclareLaunchArgument(
                "record_all",
                default_value="false",
                description="Whether to record all ROS topics.",
            ),
            DeclareLaunchArgument(
                "extra_topics",
                default_value="",
                description="Comma-separated extra topics to record.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
