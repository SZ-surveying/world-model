# fake_external_nav

This package publishes deterministic `nav_msgs/msg/Odometry` on `/odom` for
Stage 1 SITL ExternalNav validation.

It is intentionally independent from Cartographer, LiDAR, and real FCU IMU
input. Use it to verify the ArduPilot ExternalNav receiver path before
introducing SLAM noise or frame-conversion complexity.

Modes:

- `static`: fixed pose at the configured start position and yaw.
- `line`: moves along +X at `linear_velocity_x`.
- `yaw`: keeps position fixed and rotates at `yaw_rate`.

Example:

```bash
ros2 launch fake_external_nav fake_external_nav.launch.py mode:=line
```

Host-side smoke test without installing ROS2 locally:

```bash
just stage1-fake-nav-smoke line
just stage1-fake-nav-smoke static
just stage1-fake-nav-smoke yaw
```

See:

- `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`
- `docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`
