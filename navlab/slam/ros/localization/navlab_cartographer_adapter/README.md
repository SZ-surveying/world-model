# navlab_cartographer_adapter

ROS 2 wiring for the current NavLab Cartographer backend.

Contract:

- consume `sensor_msgs/msg/LaserScan` from `/scan`
- consume normalized `sensor_msgs/msg/Imu` from `/imu`
- launch `cartographer_ros` when `launch_cartographer_backend:=true`
- convert Cartographer `odom -> base_link` TF into `nav_msgs/msg/Odometry` on `/odom`
- publish structured backend health on `/navlab/slam/status`

This package is an adapter, not a standalone SLAM implementation. Real scan
matching is performed by `cartographer_ros`; this node validates inputs and
turns backend TF into the odometry topic consumed by `navlab_external_nav_bridge`.

`config/navlab_cartographer_2d.lua` is the current simulation bringup config.
It is not a final real-machine profile. Before flying on hardware, retune at
least LiDAR/IMU extrinsics, range limits, scan matcher windows, motion filter,
pose graph behavior, timestamp handling, and whether Cartographer should use
IMU data directly.

Example:

```bash
ros2 launch navlab_cartographer_adapter navlab_cartographer_adapter.launch.py \
  launch_cartographer_backend:=true \
  publish_placeholder_odom:=false
```
