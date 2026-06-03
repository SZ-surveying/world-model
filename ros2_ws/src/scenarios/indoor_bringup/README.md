# indoor_bringup

This package is the skeleton of the indoor scenario bringup layer.

Current intent:

- assemble `imu_bridge`
- assemble `cartographer_indoor`
- optionally launch `fake_external_nav` as the `/odom` source for Stage 1 P0
- assemble `external_nav_bridge`
- later connect the lidar driver launch from the x3 workspace

At this stage it only provides:

- package structure
- launch entry with `launch_fake_external_nav:=true|false`
- scenario-level config location

Stage 1 P0 fake odom example:

```bash
ros2 launch indoor_bringup indoor_bringup.launch.py \
  launch_fake_external_nav:=true \
  fake_external_nav_mode:=line \
  require_imu_for_external_nav:=false
```

See:

- `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`
- `docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`
