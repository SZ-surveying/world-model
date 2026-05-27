#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

CONFIG_FILE="${CONFIG_FILE:-/workspace/docker/ros_gz_bridge/scan_bridge.yaml}"
SIM_CMD_VEL_EXECUTOR_AUTOSTART="${SIM_CMD_VEL_EXECUTOR_AUTOSTART:-true}"
executor_pid=""

cleanup() {
  if [[ -n "${executor_pid}" ]]; then
    kill -TERM "${executor_pid}" 2>/dev/null || true
    wait "${executor_pid}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if [[ "${SIM_CMD_VEL_EXECUTOR_AUTOSTART,,}" =~ ^(1|true|yes|on)$ ]]; then
  echo "Starting lab_env.sim.nodes.cmd_vel_executor:run"
  python3 -c 'import importlib; raise SystemExit(importlib.import_module("lab_env.sim.nodes.cmd_vel_executor").run())' \
    >/tmp/cmd_vel_executor.log 2>&1 &
  executor_pid="$!"
fi

echo "Starting ros_gz_bridge for /scan using gz.msgs.LaserScan -> sensor_msgs/msg/LaserScan"
exec ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -p override_frame_id:=laser_frame
