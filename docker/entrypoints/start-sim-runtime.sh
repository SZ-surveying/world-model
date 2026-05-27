#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

SIM_MARKERS_AUTOSTART="${SIM_MARKERS_AUTOSTART:-true}"
SIM_SCAN_FEATURES_AUTOSTART="${SIM_SCAN_FEATURES_AUTOSTART:-true}"
SIM_UP_MODE="${SIM_UP_MODE:-manual}"
SIM_AUTO_WAYPOINT_FILE="${SIM_AUTO_WAYPOINT_FILE:-}"
SIM_AUTO_ROSBAG_ENABLED="${SIM_AUTO_ROSBAG_ENABLED:-true}"
SIM_AUTO_ROSBAG_LABEL="${SIM_AUTO_ROSBAG_LABEL:-auto_waypoint_follower}"
SIM_AUTO_RUN_ID="${SIM_AUTO_RUN_ID:-}"
SIM_AUTO_ARTIFACT_DIR="${SIM_AUTO_ARTIFACT_DIR:-}"
SIM_AUTO_ROSBAG_TOPIC_FILE="${SIM_AUTO_ROSBAG_TOPIC_FILE:-/workspace/profiles/sim-rosbag-topics.txt}"
runtime_env="/usr/local/bin/sim-runtime-env.sh"
pids=()
group_signal_pids=()
last_background_pid=""

cleanup() {
  for pid in "${group_signal_pids[@]}"; do
    kill -INT -- "-${pid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${group_signal_pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

trap cleanup EXIT INT TERM

start_background_python() {
  local name="$1"
  local target="$2"
  local rosbag_enabled="${3:-false}"
  local rosbag_label="${4:-$1}"
  local log_file="/tmp/${name}.log"
  local code
  code="from lab_env.sim.runtime import build_rosbag_options, invoke_python_target; "
  code+="raise SystemExit(invoke_python_target("
  code+="${target@Q}, rosbag_options=build_rosbag_options("
  code+="enabled=${rosbag_enabled@Q}.lower() in ('1', 'true', 'yes', 'on'), "
  code+="label=${rosbag_label@Q})))"
  echo "Starting ${target}"
  bash "${runtime_env}" python3 -c "${code}" >"${log_file}" 2>&1 &
  last_background_pid="$!"
  pids+=("${last_background_pid}")
}

start_background_rosbag() {
  local name="$1"
  local label="$2"
  local topic_file="$3"
  local log_file="/tmp/${name}.log"
  local session_id="${SESSION_ID:-manual}"
  local run_id
  local output_dir
  local rosbag_uri
  run_id="${SIM_AUTO_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  output_dir="${SIM_AUTO_ARTIFACT_DIR:-/workspace/artifacts/ros/${session_id}/${label}/${run_id}}"
  rosbag_uri="${output_dir}/rosbag"
  mkdir -p "${output_dir}"

  mapfile -t topics < <(grep -Ev '^[[:space:]]*(#|$)' "${topic_file}")
  if [[ "${#topics[@]}" -eq 0 ]]; then
    echo "rosbag topic file has no topics: ${topic_file}" >&2
    return 2
  fi

  echo "Starting rosbag record to ${rosbag_uri}"
  setsid bash "${runtime_env}" ros2 bag record -o "${rosbag_uri}" --topics "${topics[@]}" >"${log_file}" 2>&1 &
  local launcher_pid="$!"
  local process_pid=""
  local process_group_pid
  for _ in $(seq 1 50); do
    process_pid="$(
      ps -eo pid=,args= | awk '/\/opt\/ros\/jazzy\/bin\/ros2 bag record -o / && index($0, "'"${rosbag_uri}"'") {print $1; exit}'
    )"
    if [[ -n "${process_pid}" ]]; then
      break
    fi
    sleep 0.1
  done
  process_pid="${process_pid:-${launcher_pid}}"
  process_group_pid="$(ps -o pgid= -p "${process_pid}" | tr -d '[:space:]')"
  last_background_pid="${process_group_pid:-${process_pid}}"
  pids+=("${process_pid}")
  group_signal_pids+=("${last_background_pid}")
}

if [[ "${SIM_SCAN_FEATURES_AUTOSTART,,}" =~ ^(1|true|yes|on)$ ]]; then
  start_background_python "scan_features_publisher" "lab_env.sim.nodes.scan_features_publisher:run"
fi

if [[ "${SIM_MARKERS_AUTOSTART,,}" =~ ^(1|true|yes|on)$ ]]; then
  start_background_python "world_marker_publisher" "lab_env.sim.nodes.world_marker_publisher:run"
fi

if [[ "${SIM_UP_MODE}" == "auto" && -n "${SIM_AUTO_WAYPOINT_FILE}" ]]; then
  rosbag_pid=""
  if [[ "${SIM_AUTO_ROSBAG_ENABLED,,}" =~ ^(1|true|yes|on)$ ]]; then
    start_background_rosbag "auto_waypoint_rosbag" "${SIM_AUTO_ROSBAG_LABEL}" "${SIM_AUTO_ROSBAG_TOPIC_FILE}"
    rosbag_pid="${last_background_pid}"
    export SIM_AUTO_ROSBAG_PID="${rosbag_pid}"
  fi

  start_background_python "waypoint_follower" "lab_env.sim.nodes.waypoint_follower:run"
  unset SIM_AUTO_ROSBAG_PID || true
fi

echo "Starting sim-runtime idle shell"
sleep infinity &
wait $!
