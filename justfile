set dotenv-load := true
set dotenv-filename := ".env"

set shell := ["bash", "-euo", "pipefail", "-c"]

x3_ws := "/home/admin/workspace/world-model/x3"
pkg := "ydlidar_ros2_driver"
orchestration_cmd := "uv run --project orchestration python orchestration/main.py"

default:
    @just --list

# Build the NavLab companion image.
navlab-companion-image-build *args='':
    {{orchestration_cmd}} build companion {{args}}

# Build the NavLab Cartographer SLAM image.
navlab-slam-image-build *args='':
    {{orchestration_cmd}} build slam {{args}}

# Build the NavLab Gazebo/sensor image with YDLidar vendor driver support.
navlab-gazebo-sensor-image-build *args='':
    {{orchestration_cmd}} build gazebo-sensor {{args}}

# Build the NavLab official ArduPilot ROS2/DDS baseline image.
navlab-official-baseline-image-build *args='':
    {{orchestration_cmd}} build official-baseline {{args}}

# Run the X2 vendor-driver smoke in the Gazebo/sensor runtime.
x2-driver-smoke duration_sec='15':
    X2_MODE=driver-smoke \
    X2_SMOKE_DURATION_SEC={{duration_sec}} \
    X2_ARTIFACT_DIR=/artifacts/ros/x2_driver_smoke/manual \
    docker compose --file compose/docker-compose.yaml --project-directory . --profile x2_sensor \
      up --abort-on-container-exit --exit-code-from gazebo-sensor gazebo-sensor

# Build all NavLab images.
navlab-images-build *args='':
    {{orchestration_cmd}} build all {{args}}

# Check the NavLab companion image contents without running a flight mission.
navlab-doctor *args='':
    {{orchestration_cmd}} doctor {{args}}

# Check official ArduPilot ROS2/DDS and Cartographer baseline prerequisites.
navlab-official-baseline-doctor *args='':
    {{orchestration_cmd}} official-baseline-doctor {{args}}

# Run the official ArduPilot ROS2/DDS baseline graph acceptance.
navlab-official-baseline-acceptance duration_sec='30' *args='':
    {{orchestration_cmd}} official-baseline-acceptance {{duration_sec}} {{args}}

# Run official iris_maze with NavLab X2 virtual serial + vendor /scan acceptance.
navlab-official-maze-x2-acceptance duration_sec='45' *args='':
    {{orchestration_cmd}} official-maze-x2-acceptance {{duration_sec}} {{args}}

# Check P2 official maze rangefinder/IMU mechanism prerequisites.
navlab-rangefinder-imu-doctor *args='':
    {{orchestration_cmd}} rangefinder-imu-doctor {{args}}

# Run P2 official maze down rangefinder + IMU mechanism acceptance.
navlab-rangefinder-imu-acceptance duration_sec='60' *args='':
    {{orchestration_cmd}} rangefinder-imu-acceptance {{duration_sec}} {{args}}

# Run NavLab companion + SITL + Gazebo obstacle acceptance with rosbag/Foxglove artifacts.
navlab-acceptance duration_sec='90' *args='':
    {{orchestration_cmd}} acceptance {{duration_sec}} {{args}}

# Run NavLab FCU GUIDED/arm/takeoff/hover acceptance with rosbag/Foxglove artifacts.
navlab-hover-acceptance duration_sec='90' *args='':
    {{orchestration_cmd}} hover {{duration_sec}} {{args}}

# Run NavLab rangefinder altitude hover diagnostic without SLAM horizontal control.
navlab-hover-diagnostic duration_sec='90' *args='':
    {{orchestration_cmd}} hover-diagnostic {{duration_sec}} {{args}}

# Run NavLab SLAM ExternalNav hover diagnostic without horizontal setpoints.
navlab-hover-slam-diagnostic duration_sec='90' *args='':
    {{orchestration_cmd}} hover-slam-diagnostic {{duration_sec}} {{args}}

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
