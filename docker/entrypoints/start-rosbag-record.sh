#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

SESSION_ID="${SESSION_ID:-manual}"
TOPIC_FILE="${TOPIC_FILE:-/workspace/profiles/rosbag-topics.txt}"
OUTPUT_DIR="${OUTPUT_DIR:-/artifacts/ros/${SESSION_ID}/rosbag}"

if [[ ! -f "${TOPIC_FILE}" ]]; then
  echo "rosbag topic file missing: ${TOPIC_FILE}" >&2
  exit 2
fi

mkdir -p "$(dirname "${OUTPUT_DIR}")"
mapfile -t TOPICS < <(grep -vE '^\s*(#|$)' "${TOPIC_FILE}")

if [[ "${#TOPICS[@]}" -eq 0 ]]; then
  echo "no rosbag topics configured in ${TOPIC_FILE}" >&2
  exit 2
fi

if [[ -e "${OUTPUT_DIR}" ]]; then
  echo "Removing existing rosbag output directory ${OUTPUT_DIR}"
  rm -rf "${OUTPUT_DIR}"
fi

echo "Recording rosbag to ${OUTPUT_DIR}"
printf '  %s\n' "${TOPICS[@]}"
exec ros2 bag record -o "${OUTPUT_DIR}" --topics "${TOPICS[@]}"
