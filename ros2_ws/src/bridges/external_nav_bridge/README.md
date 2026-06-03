# external_nav_bridge

This package is the skeleton of the indoor ExternalNav bridge.

Current intent:

- subscribe to `/odom`
- optionally observe `/imu/data`
- publish `/external_nav/odom`
- publish internal bridge status
- publish `/ap/tf` as the ArduPilot ROS2 DDS-shaped diagnostic/future entry point
- feed `lab_env.sim.nodes.mavlink_external_nav_sender`, which emits MAVLink `ODOMETRY` for the current Stage 1 SITL path
- optionally observe `/height/estimate` for future height gating and diagnostics

This package is intentionally minimal at this stage. It provides:

- package structure
- `/external_nav/odom` publishing from `/odom`
- `/ap/tf` publishing from `/odom`
- JSON status publishing on `/external_nav/status`
- input freshness, rate, frame, and IMU readiness diagnostics
- optional height input readiness diagnostics

Current coordinate mode:

- `pass_through_enu_flu`
- Coordinate conversion is centralized in this bridge, but P0.3 does not yet
  perform full ENU/NED or FLU/FRD conversion.

Host-side smoke test without installing ROS2 locally:

```bash
just stage1-bridge-smoke line
```

See:

- `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`
- `docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`
- `docs/scenarios/indoor/stage1_mavlink_odometry_mapping.md`
- `docs/scenarios/indoor/stage1_height_estimate_contract.md`
