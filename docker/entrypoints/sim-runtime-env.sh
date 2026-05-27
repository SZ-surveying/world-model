#!/usr/bin/env bash
set -euo pipefail

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

overlay_root="/workspace/.sim_ros2_overlay"
overlay_source="/workspace/x3/src/ydlidar_interfaces"
overlay_setup="${overlay_root}/install/setup.bash"

needs_overlay_build() {
  [[ ! -f "${overlay_setup}" ]] && return 0
  find "${overlay_source}" \
    \( -name '*.msg' -o -name 'package.xml' -o -name 'CMakeLists.txt' \) \
    -type f \
    -newer "${overlay_setup}" \
    -print \
    -quit | grep -q .
}

build_overlay() {
  local build_log
  build_log="/tmp/sim_ros2_overlay_build.log"
  mkdir -p "${overlay_root}"
  echo "Building ydlidar_interfaces overlay from ${overlay_source}"
  if ! colcon --log-base "${overlay_root}/log" build \
    --base-paths "${overlay_source}" \
    --build-base "${overlay_root}/build" \
    --install-base "${overlay_root}/install" \
    --packages-select ydlidar_interfaces \
    >"${build_log}" 2>&1; then
    cat "${build_log}" >&2
    return 1
  fi
}

source_ros_setup /opt/ros/jazzy/setup.bash

if [[ -d "${overlay_source}" ]]; then
  if needs_overlay_build; then
    build_overlay
  fi
  if [[ -f "${overlay_setup}" ]]; then
    source_ros_setup "${overlay_setup}"
  fi
fi

exec "$@"
