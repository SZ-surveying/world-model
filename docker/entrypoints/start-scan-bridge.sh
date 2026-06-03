#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

CONFIG_FILE="${CONFIG_FILE:-/workspace/docker/ros_gz_bridge/scan_bridge.yaml}"

echo "Starting ros_gz_bridge for /scan_ideal using gz.msgs.LaserScan -> sensor_msgs/msg/LaserScan"
exec ros2 run ros_gz_bridge parameter_bridge /scan_ideal@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -p override_frame_id:=laser_frame
