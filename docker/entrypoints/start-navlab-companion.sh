#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/navlab_ws/install/setup.bash ]]; then
  source /opt/navlab_ws/install/setup.bash
fi
set -u

NAVLAB_RUNTIME_CONFIG="${NAVLAB_RUNTIME_CONFIG:-/workspace/navlab/config.toml}"

exec /opt/companion-venv/bin/python -m navlab.companion.cli launch-companion --config "${NAVLAB_RUNTIME_CONFIG}"
