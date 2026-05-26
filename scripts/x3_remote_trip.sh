#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
X3_WS="${X3_WS:-${REPO_ROOT}/x3}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
X3_SETUP="${X3_SETUP:-${X3_WS}/install/setup.bash}"
PARAMS_FILE_DEFAULT="${X3_WS}/src/ydlidar_ros2_driver/params/X2.yaml"
BAG_ROOT_DEFAULT="${X3_WS}/bags"
RUNTIME_ROOT_DEFAULT="${X3_WS}/log/runtime"
DEFAULT_BAG_STORAGE="${DEFAULT_BAG_STORAGE:-mcap}"

PARAMS_FILE="${PARAMS_FILE_DEFAULT}"
BAG_ROOT="${BAG_ROOT_DEFAULT}"
RUNTIME_ROOT="${RUNTIME_ROOT_DEFAULT}"
BAG_STORAGE="${DEFAULT_BAG_STORAGE}"
FOXGLOVE_ADDRESS="${FOXGLOVE_ADDRESS:-0.0.0.0}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
WITH_FEATURES=0
RECORD_ALL=0
DRY_RUN=0
EXTRA_TOPICS=()

usage() {
  cat <<EOF
用法:
  $(basename "$0") [选项]

作用:
  1. 启动 YDLIDAR 驱动
  2. 启动 Foxglove Bridge，方便远程连接
  3. 以 MCAP 为默认格式启动 rosbag2 录制

默认录制的话题:
  /scan /point_cloud /tf /tf_static /rosout

选项:
  --params-file PATH      指定雷达参数文件，默认: ${PARAMS_FILE_DEFAULT}
  --bag-root DIR          rosbag2 输出根目录，默认: ${BAG_ROOT_DEFAULT}
  --bag-storage ID        rosbag2 存储后端，默认: ${DEFAULT_BAG_STORAGE}
  --runtime-root DIR      运行日志根目录，默认: ${RUNTIME_ROOT_DEFAULT}
  --foxglove-address IP   Foxglove Bridge 监听地址，默认: ${FOXGLOVE_ADDRESS}
  --foxglove-port PORT    Foxglove Bridge 端口，默认: ${FOXGLOVE_PORT}
  --with-features         额外启动 /scan_features 节点，并把特征话题一起录制
  --topic TOPIC           额外录制一个话题，可重复传入
  --record-all            录制所有 ROS 话题
  --dry-run               只打印将要执行的命令，不真正启动
  -h, --help              显示帮助

Foxglove 远程连接示例:
  ws://<机器人IP>:${FOXGLOVE_PORT}
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --params-file)
      PARAMS_FILE="$2"
      shift 2
      ;;
    --bag-root)
      BAG_ROOT="$2"
      shift 2
      ;;
    --bag-storage)
      BAG_STORAGE="$2"
      shift 2
      ;;
    --runtime-root)
      RUNTIME_ROOT="$2"
      shift 2
      ;;
    --foxglove-address)
      FOXGLOVE_ADDRESS="$2"
      shift 2
      ;;
    --foxglove-port)
      FOXGLOVE_PORT="$2"
      shift 2
      ;;
    --with-features)
      WITH_FEATURES=1
      shift
      ;;
    --topic)
      EXTRA_TOPICS+=("$2")
      shift 2
      ;;
    --record-all)
      RECORD_ALL=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "找不到 ROS 环境脚本: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${X3_SETUP}" ]]; then
  echo "找不到 x3 工作空间环境脚本: ${X3_SETUP}" >&2
  echo "如果还没编译，先执行: cd ${X3_WS} && colcon build --symlink-install" >&2
  exit 1
fi

if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "找不到参数文件: ${PARAMS_FILE}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
source "${X3_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "当前环境里没有 ros2 命令，请检查 ROS 2 安装。" >&2
  exit 1
fi

if ! ros2 pkg prefix ydlidar_ros2_driver >/dev/null 2>&1; then
  echo "找不到包 ydlidar_ros2_driver，请先确认 x3 工作空间已经成功构建。" >&2
  exit 1
fi

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "找不到包 foxglove_bridge，请先安装 ROS 2 的 foxglove_bridge。" >&2
  exit 1
fi

if ! ros2 bag record --help 2>/dev/null | grep -q -- "${BAG_STORAGE}"; then
  echo "当前 rosbag2 环境里找不到存储后端: ${BAG_STORAGE}" >&2
  echo "如果要使用 MCAP，请先安装 ros-humble-rosbag2-storage-mcap。" >&2
  exit 1
fi

detect_ip() {
  local ip_addr=""
  if command -v ip >/dev/null 2>&1; then
    ip_addr="$(ip route get 1.1.1.1 2>/dev/null | awk '/src/ {for (i = 1; i <= NF; ++i) if ($i == "src") {print $(i + 1); exit}}')"
  fi
  if [[ -z "${ip_addr}" ]] && command -v hostname >/dev/null 2>&1; then
    ip_addr="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi
  printf '%s' "${ip_addr}"
}

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUNTIME_DIR="${RUNTIME_ROOT}/${RUN_ID}"
BAG_PREFIX="${BAG_ROOT}/x3_trip_${RUN_ID}"
mkdir -p "${RUNTIME_DIR}" "${BAG_ROOT}"

TOPICS=(/scan /point_cloud /tf /tf_static /rosout)
if [[ ${WITH_FEATURES} -eq 1 ]]; then
  TOPICS+=(/scan_features /scan_nearest_point)
fi
if [[ ${#EXTRA_TOPICS[@]} -gt 0 ]]; then
  TOPICS+=("${EXTRA_TOPICS[@]}")
fi

LIDAR_CMD=(ros2 launch ydlidar_ros2_driver ydlidar_launch.py "params_file:=${PARAMS_FILE}")
FOXGLOVE_CMD=(ros2 launch foxglove_bridge foxglove_bridge_launch.xml "address:=${FOXGLOVE_ADDRESS}" "port:=${FOXGLOVE_PORT}")
FEATURE_CMD=(ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_scan_features)
if [[ ${RECORD_ALL} -eq 1 ]]; then
  BAG_CMD=(ros2 bag record -s "${BAG_STORAGE}" -a -o "${BAG_PREFIX}")
else
  BAG_CMD=(ros2 bag record -s "${BAG_STORAGE}" -o "${BAG_PREFIX}" "${TOPICS[@]}")
fi

LOCAL_IP="$(detect_ip)"

echo "运行 ID: ${RUN_ID}"
echo "参数文件: ${PARAMS_FILE}"
echo "日志目录: ${RUNTIME_DIR}"
echo "rosbag 输出: ${BAG_PREFIX}"
echo "rosbag 存储后端: ${BAG_STORAGE}"
if [[ -n "${LOCAL_IP}" ]]; then
  echo "Foxglove 连接地址: ws://${LOCAL_IP}:${FOXGLOVE_PORT}"
else
  echo "Foxglove 连接地址: ws://<本机IP>:${FOXGLOVE_PORT}"
fi
echo

print_cmd() {
  local name="$1"
  shift
  printf '[%s]\n' "${name}"
  printf '  ' 
  printf '%q ' "$@"
  printf '\n'
}

print_cmd "lidar" "${LIDAR_CMD[@]}"
print_cmd "foxglove" "${FOXGLOVE_CMD[@]}"
if [[ ${WITH_FEATURES} -eq 1 ]]; then
  print_cmd "scan_features" "${FEATURE_CMD[@]}"
fi
print_cmd "rosbag" "${BAG_CMD[@]}"
echo

if [[ ${DRY_RUN} -eq 1 ]]; then
  exit 0
fi

declare -A PIDS=()
START_ORDER=()

cleanup() {
  trap - EXIT
  for name in "${START_ORDER[@]}"; do
    local pid="${PIDS[$name]:-}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for name in "${START_ORDER[@]}"; do
    local pid="${PIDS[$name]:-}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}

trap cleanup EXIT
trap 'exit 130' INT TERM

start_process() {
  local name="$1"
  shift
  local log_file="${RUNTIME_DIR}/${name}.log"
  echo "启动 ${name}，日志: ${log_file}"
  "$@" >"${log_file}" 2>&1 &
  local pid=$!
  PIDS["${name}"]="${pid}"
  START_ORDER+=("${name}")
}

start_process lidar "${LIDAR_CMD[@]}"
sleep 2
start_process foxglove "${FOXGLOVE_CMD[@]}"

if [[ ${WITH_FEATURES} -eq 1 ]]; then
  sleep 1
  start_process scan_features "${FEATURE_CMD[@]}"
fi

sleep 1
start_process rosbag "${BAG_CMD[@]}"

echo
echo "全部已启动，按 Ctrl-C 结束录制。"
echo

while true; do
  sleep 2
  for name in "${START_ORDER[@]}"; do
    pid="${PIDS[$name]}"
    if ! kill -0 "${pid}" 2>/dev/null; then
      set +e
      wait "${pid}"
      status=$?
      set -e
      echo "${name} 提前退出，返回码: ${status}" >&2
      exit "${status}"
    fi
  done
done
