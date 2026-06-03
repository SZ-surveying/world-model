set dotenv-load := true
set dotenv-filename := ".env"

set shell := ["bash", "-euo", "pipefail", "-c"]

x3_ws := "/home/admin/workspace/world-model/x3"
pkg := "ydlidar_ros2_driver"
lab_env_cmd := "uv run --project lab_env --no-sync --group host python -m lab_env.main"

default:
    @just --list

# Build the NavLab companion image.
navlab-companion-image-build *args='':
    {{lab_env_cmd}} navlab build companion {{args}}

# Build the NavLab Cartographer SLAM image.
navlab-slam-image-build *args='':
    {{lab_env_cmd}} navlab build slam {{args}}

# Build all NavLab images.
navlab-images-build *args='':
    {{lab_env_cmd}} navlab build all {{args}}

# Check the NavLab companion image contents without running a flight mission.
navlab-doctor *args='':
    {{lab_env_cmd}} navlab doctor {{args}}

# Run NavLab companion + SITL + Gazebo obstacle acceptance with rosbag/Foxglove artifacts.
navlab-acceptance duration_sec='90' *args='':
    {{lab_env_cmd}} navlab acceptance {{duration_sec}} {{args}}

# Build the YDLidar ROS2 package.
x3-build:
    cd {{x3_ws}} && ../scripts/build-ydlidar.sh --no-clean

# Launch the lidar driver with the current default setup.
x3-launch:
    cd {{x3_ws}} && \
    source install/setup.bash && \
    ros2 launch {{pkg}} ydlidar_launch.py

# Run lidar + Foxglove bridge + rosbag2 recording in one command.
x3-trip *launch_args='':
    cd {{x3_ws}} && \
    source install/setup.bash && \
    ros2 launch {{pkg}} x3_remote_trip.launch.py {{launch_args}}
