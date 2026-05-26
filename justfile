set dotenv-load := true
set dotenv-filename := ".env"

set shell := ["bash", "-euo", "pipefail", "-c"]

x3_ws := "/home/admin/workspace/world-model/x3"
pkg := "ydlidar_ros2_driver"
compose_cmd := "docker compose -f compose/docker-compose.yaml --project-directory ."

default:
    @just --list

# Start the default lab environment profile.
up:
    uv run --no-sync --group lab_env python -m lab_env.main up

# Check the default lab environment profile.
doctor:
    uv run --no-sync --group lab_env python -m lab_env.main doctor

# Stop the lab environment.
down:
    uv run --no-sync --group lab_env python -m lab_env.main down

# Subscribe to /scan on the host. Requires a sourced ROS2 Python environment.
sim-scan-consumer *args='':
    python3 -m lab_env.sim.front_sector_consumer {{args}}

# Start Gazebo + scan bridge + runtime helper container for P1 validation.
sim-p1-up:
    {{compose_cmd}} --profile base_env --profile sim_p1 up -d gazebo scan-bridge sim-runtime

# Stop Gazebo + scan bridge + runtime helper container.
sim-p1-down:
    {{compose_cmd}} --profile base_env --profile sim_p1 down

# Run the existing front-sector consumer against Gazebo-backed /scan.
sim-p1-consumer *args='':
    {{compose_cmd}} exec sim-runtime bash -lc "source /opt/ros/jazzy/setup.bash && python3 -m lab_env.sim.front_sector_consumer {{args}}"

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
