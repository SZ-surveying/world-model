#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${WS_DIR:-${REPO_ROOT}/x3}"
PKG_NAME="${PKG_NAME:-ydlidar_ros2_driver}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
PYTHON_BIN="${PYTHON_BIN:-}"
CLEAN_BUILD="${CLEAN_BUILD:-1}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--clean|--no-clean] [--help]

Build the ${PKG_NAME} package in the ROS workspace.

Options:
  --clean     Remove the package build/install outputs before building (default)
  --no-clean  Keep existing build/install outputs
  -h, --help  Show this help message

Environment overrides:
  WS_DIR      Workspace path (default: ${WS_DIR})
  PKG_NAME    Package name (default: ${PKG_NAME})
  ROS_SETUP   ROS setup script (default: ${ROS_SETUP})
  PYTHON_BIN  Python executable for colcon/cmake
  CLEAN_BUILD Set to 1 or 0 to control cleanup by default
EOF
}

remove_path_entry() {
  local target="$1"
  local entry
  local filtered=()

  IFS=':' read -r -a path_entries <<< "${PATH}"
  for entry in "${path_entries[@]}"; do
    if [[ "${entry}" != "${target}" ]] && [[ -n "${entry}" ]]; then
      filtered+=("${entry}")
    fi
  done

  PATH="$(IFS=:; echo "${filtered[*]}")"
  export PATH
}

sanitize_conda_env() {
  local conda_base=""
  local conda_sh=""

  if [[ -n "${CONDA_EXE:-}" ]]; then
    conda_base="$(cd -- "$(dirname -- "${CONDA_EXE}")/.." && pwd)"
    conda_sh="${conda_base}/etc/profile.d/conda.sh"
  fi

  if [[ "${CONDA_SHLVL:-0}" -gt 0 ]] && [[ -f "${conda_sh}" ]]; then
    set +u
    # Load the conda shell helpers when the current shell only has the binary.
    source "${conda_sh}"
    set -u
    conda deactivate || true
  fi

  if [[ -n "${conda_base}" ]]; then
    remove_path_entry "${conda_base}/bin"
    remove_path_entry "${conda_base}/condabin"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      CLEAN_BUILD=1
      ;;
    --no-clean)
      CLEAN_BUILD=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      echo >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

if [[ ! -d "${WS_DIR}" ]]; then
  echo "Workspace not found: ${WS_DIR}" >&2
  exit 1
fi

cd "${WS_DIR}"

sanitize_conda_env

if [[ -z "${PYTHON_BIN}" ]]; then
  if [[ -x /usr/bin/python3 ]]; then
    PYTHON_BIN="/usr/bin/python3"
  else
    PYTHON_BIN="$(command -v python3 || true)"
  fi
fi

if [[ -z "${PYTHON_BIN}" ]]; then
  echo "python3 not found in PATH" >&2
  exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

export AMENT_PYTHON_EXECUTABLE="${PYTHON_BIN}"

if [[ "${CLEAN_BUILD}" == "1" ]]; then
  rm -rf "build/${PKG_NAME}" "install/${PKG_NAME}" log/latest*
fi

colcon build \
  --packages-up-to "${PKG_NAME}" \
  --symlink-install \
  --cmake-args "-DPython3_EXECUTABLE=${PYTHON_BIN}"

echo "Build finished."
echo "To load the workspace in your current shell, run:"
echo "  source \"${WS_DIR}/install/setup.bash\""
