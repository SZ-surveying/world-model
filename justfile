set shell := ["bash", "-lc"]

x3_ws := "/home/admin/workspace/world-model/x3"
pkg := "ydlidar_ros2_driver"

default:
    @just --list

# Build the YDLidar ROS2 package.
x3-build:
    cd {{x3_ws}} && ../scripts/build-ydlidar.sh --no-clean

# Launch the lidar driver with the current default setup.
x3-launch:
    cd {{x3_ws}} && \
    source install/setup.bash && \
    ros2 launch {{pkg}} ydlidar_launch.py

# Convert /scan into compact structured features for downstream nodes.
x3-features:
    cd {{x3_ws}} && \
    source install/setup.bash && \
    ros2 run {{pkg}} ydlidar_ros2_driver_scan_features

# Echo compact scan features.
x3-echo-features:
    cd {{x3_ws}} && \
    source install/setup.bash && \
    ros2 topic echo /scan_features
