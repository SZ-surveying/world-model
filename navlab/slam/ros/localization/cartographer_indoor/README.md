# cartographer_indoor

This package is the ROS2-side wiring for the indoor Cartographer 2D module.

Current intent:

- subscribe to `/scan`
- subscribe to `/imu/data`
- launch `cartographer_ros` with indoor-focused config when available
- convert Cartographer TF output into `/odom` for `external_nav_bridge`

Current files:

- `launch/cartographer_indoor.launch.py`: starts the ROS2-side adapter and can
  optionally launch the `cartographer_ros` backend
- `config/cartographer_indoor_2d.lua`: indoor 2D Cartographer config
- `config/cartographer_indoor.params.yaml`: adapter parameters
- `src/cartographer_indoor_node.cpp`: watches `/scan`, `/imu/data`, `/tf` and
  republishes `odom -> base_link` as `/odom`

Modes:

- `publish_placeholder_odom:=true`: smoke test only
- `publish_placeholder_odom:=false`: expect real Cartographer TF and publish
  TF-backed `/odom`

The adapter publishes structured JSON on `/cartographer/status`. The key
states are:

- `waiting_for_scan_and_imu`
- `waiting_for_scan`
- `waiting_for_imu`
- `waiting_for_cartographer_tf`
- `publishing_tf_backed_odom`
- `publishing_placeholder_odom`

Example:

```bash
ros2 launch cartographer_indoor cartographer_indoor.launch.py \
  launch_cartographer_backend:=true \
  publish_placeholder_odom:=false
```

Important:

- this package does not vendor `cartographer_ros`
- the real backend path requires `cartographer_ros` to be installed separately
- when the backend is not installed, keep `launch_cartographer_backend:=false`
- Stage 1 P1.2 validates the Cartographer-compatible TF adapter with synthetic
  `/scan`, `/imu/data`, and `/tf`; real scan matching is introduced only when
  `cartographer_ros` is available.

Stage 1 smoke:

```bash
just stage1-cartographer-smoke
```

See:

- `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`
- `docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`
