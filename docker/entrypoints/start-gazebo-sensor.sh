#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/navlab_sensor_ws/install/setup.bash ]]; then
  source /opt/navlab_sensor_ws/install/setup.bash
fi
set -u

auto_start_arg="--no-auto-start"
if [[ "${X2_AUTO_START:-false}" == "true" ]]; then
  auto_start_arg="--auto-start"
fi

driver_smoke_args=()
if [[ "${X2_MODE:-emulator}" == "driver-smoke" ]]; then
  driver_smoke_args=(
    "--driver-smoke"
    "--duration-sec" "${X2_SMOKE_DURATION_SEC:-15}"
    "--artifact-dir" "${X2_ARTIFACT_DIR:-/artifacts/ros/x2_driver_smoke/manual}"
    "--startup-timeout-sec" "${X2_STARTUP_TIMEOUT_SEC:-20}"
  )
fi

if [[ "${X2_MODE:-runtime}" == "runtime" ]]; then
  exec /opt/gazebo-sensor-venv/bin/python -m lab_env.sim.sensors.x2.runtime \
    --scan-source "${X2_SCAN_SOURCE:-x2_virtual_serial}" \
    --virtual-serial-link "${X2_VIRTUAL_SERIAL_LINK:-/tmp/navlab_x2}" \
    --status-topic "${X2_STATUS_TOPIC:-/sim/x2/status}" \
    --scan-ideal-topic "${X2_SCAN_IDEAL_TOPIC:-/scan_ideal}" \
    --scan-topic "${X2_SCAN_TOPIC:-/scan}" \
    --profile-path "${X2_PROFILE_PATH:-/workspace/profiles/x2-vendor-sim.yaml}" \
    --scan-frequency-hz "${X2_SCAN_FREQUENCY_HZ:-7.0}" \
    --sample-rate-hz "${X2_SAMPLE_RATE_HZ:-3000.0}" \
    --range-min-m "${X2_RANGE_MIN_M:-0.1}" \
    --range-max-m "${X2_RANGE_MAX_M:-8.0}" \
    --static-range-m "${X2_STATIC_RANGE_M:-1.5}" \
    --range-noise-stddev-m "${X2_RANGE_NOISE_STDDEV_M:-0.0}" \
    --dropout-rate "${X2_DROPOUT_RATE:-0.0}" \
    "${auto_start_arg}"
fi

exec /opt/gazebo-sensor-venv/bin/python -m lab_env.sim.sensors.x2.cli \
  --virtual-serial-link "${X2_VIRTUAL_SERIAL_LINK:-/tmp/navlab_x2}" \
  --status-topic "${X2_STATUS_TOPIC:-/sim/x2/status}" \
  --scan-ideal-topic "${X2_SCAN_IDEAL_TOPIC:-/scan_ideal}" \
  --profile-path "${X2_PROFILE_PATH:-/workspace/profiles/x2-vendor-sim.yaml}" \
  --scan-frequency-hz "${X2_SCAN_FREQUENCY_HZ:-7.0}" \
  --sample-rate-hz "${X2_SAMPLE_RATE_HZ:-3000.0}" \
  --range-min-m "${X2_RANGE_MIN_M:-0.1}" \
  --range-max-m "${X2_RANGE_MAX_M:-8.0}" \
  --static-range-m "${X2_STATIC_RANGE_M:-1.5}" \
  --range-noise-stddev-m "${X2_RANGE_NOISE_STDDEV_M:-0.0}" \
  --dropout-rate "${X2_DROPOUT_RATE:-0.0}" \
  "${auto_start_arg}" \
  "${driver_smoke_args[@]}"
