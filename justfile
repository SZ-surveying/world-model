set dotenv-load := true
set dotenv-filename := ".env"

set shell := ["bash", "-euo", "pipefail", "-c"]

x3_ws := "/home/admin/workspace/world-model/x3"
pkg := "ydlidar_ros2_driver"
lab_env_cmd := "uv run --project lab_env --no-sync --group host python -m lab_env.main"

default:
    @just --list

# Start the default lab environment profile.
up:
    {{lab_env_cmd}} up

# Check the default lab environment profile.
doctor:
    {{lab_env_cmd}} doctor

# Stop the lab environment.
down:
    {{lab_env_cmd}} down

# Subscribe to /scan on the host. Requires a sourced ROS2 Python environment.
sim-scan-consumer *args='':
    python3 -m lab_env.sim.nodes.front_sector_consumer {{args}}

# Start Gazebo + scan bridge + runtime helper container for P1 validation.
sim-p1-up *args='':
    {{lab_env_cmd}} sim up {{args}}

# Start the sim stack in explicit manual mode for Foxglove teleop.
sim-p1-manual *args='':
    {{lab_env_cmd}} sim up --mode manual {{args}}

# Start the sim stack in explicit auto mode with a waypoint file.
sim-p1-auto waypoint_file *args='':
    {{lab_env_cmd}} sim up --mode auto --waypoint-file {{waypoint_file}} {{args}}

# Stop Gazebo + scan bridge + runtime helper container.
sim-p1-down:
    {{lab_env_cmd}} sim down

# Run the existing front-sector consumer against Gazebo-backed /scan.
sim-p1-consumer *args='':
    {{lab_env_cmd}} sim consumer {{args}}

# Run the front-sector consumer with rosbag recording enabled.
sim-p1-consumer-record *args='':
    {{lab_env_cmd}} sim consumer-record {{args}}

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
