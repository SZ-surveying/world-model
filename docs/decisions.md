# Decisions

## 2026-05-26 P1 Static Lidar Rig And YAML Bridge

Decision: implement P1 with a static `uav_lidar_rig` model at the world origin and a dedicated `ros_gz_bridge` YAML config.

Basis: codebase research plus official Gazebo / ros_gz_bridge docs.

Reason: P1 only needs to prove that Gazebo can generate a ROS2 `/scan` matching the real `x3` contract. A static rig is the smallest slice that validates sensor geometry without dragging in UAV dynamics. A YAML bridge config keeps topic name, message types, QoS, and frame override explicit in one place.
