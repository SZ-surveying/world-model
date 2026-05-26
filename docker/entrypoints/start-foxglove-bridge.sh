#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

PORT="8765"
echo "Starting foxglove_bridge on port ${PORT}"
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${PORT}
