#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

PLAY_BAG_DIR="${PLAY_BAG_DIR:-/artifacts/ros/manual/rosbag}"
PLAY_ARGS="${PLAY_ARGS:---loop}"

if [[ ! -f "${PLAY_BAG_DIR}/metadata.yaml" ]]; then
  echo "rosbag play input missing: expected ${PLAY_BAG_DIR}/metadata.yaml" >&2
  exit 2
fi

echo "Playing rosbag from ${PLAY_BAG_DIR}"
if [[ -n "${PLAY_ARGS}" ]]; then
  echo "Extra play args: ${PLAY_ARGS}"
fi

read -r -a EXTRA_ARGS <<< "${PLAY_ARGS}"
exec ros2 bag play "${PLAY_BAG_DIR}" "${EXTRA_ARGS[@]}"
