#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/navlab_ws/install/setup.bash ]]; then
  source /opt/navlab_ws/install/setup.bash
fi
set -u

NAVLAB_CONFIG="${NAVLAB_CONFIG:-/workspace/profiles/navlab-gazebo.toml}"

exec /opt/companion-venv/bin/python -m lab_env.navlab.runtime.cli launch-companion --config "${NAVLAB_CONFIG}"
