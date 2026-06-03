#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/navlab_ws/install/setup.bash ]]; then
  source /opt/navlab_ws/install/setup.bash
fi
set -u

PORT="8765"
echo "Starting foxglove_bridge on port ${PORT}"
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${PORT}
