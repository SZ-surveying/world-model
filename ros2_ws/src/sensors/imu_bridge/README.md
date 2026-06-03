# imu_bridge

This package is the skeleton of the IMU bridge used by the indoor
ExternalNav pipeline.

Current intent:

- adapt the onboard IMU from the UAV/FCU side
- normalize it to ROS2 `/imu/data`
- publish `/imu/status`
- keep flight-controller-specific protocol details out of SLAM

This package is not intended to drive a separate external IMU module.
Its job is to expose the FCU IMU to ROS2 so `cartographer_indoor` can
consume it.

It provides:

- a topic-based adapter for FCU-side `sensor_msgs/msg/Imu`
- normalization to `/imu/data`
- placeholder mode for contract-only smoke tests
- structured JSON status publishing on `/imu/status`

Recommended upstream sources:

- ArduPilot ROS2 DDS: `/ap/imu/experimental/data`
- MAVROS bridge: prefer `/mavros/imu/data_raw`, use `/mavros/imu/data` only if needed

The ArduPilot DDS IMU topic is the closest match to the target architecture,
but ArduPilot documents it as an experimental interface, so pinning versions
matters.

Current parameters:

- `source_mode`: `topic` or `placeholder`
- `source_topic`: upstream `sensor_msgs/msg/Imu` topic
- `source_label`: source name shown in `/imu/status`
- `output_frame_id`: fallback frame when input frame is empty
- `use_input_frame_id`: keep incoming frame if available
- `replace_zero_timestamp`: replace zero stamp with current ROS time
- `input_timeout_ms`: maximum source age before status becomes stale
- `min_input_rate_hz`: minimum accepted input rate

Status states:

- `waiting_for_fcu_imu_source`
- `streaming_fcu_imu`
- `low_rate_fcu_imu_source`
- `stale_fcu_imu_source`
- `publishing_placeholder_fcu_imu`

Example:

```bash
ros2 run imu_bridge imu_bridge_node \\
  --ros-args \\
  -p source_topic:=/mavros/imu/data_raw \\
  -p source_label:=mavros_raw
```

Stage 1 smoke:

```bash
just stage1-imu-bridge-smoke
```

See:

- `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`
- `docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`
