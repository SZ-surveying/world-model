# Decisions

## 2026-06-03: NavLab Gazebo uses IQ-style quad with ArduPilot Gazebo plugin

Decision: use `iq_sim` as the Iris quad + lidar model reference, but build the runtime plugin from official `ardupilot_gazebo`.

Basis: codebase research and upstream docs.

Reason: `iq_sim` demonstrates the quad layout, rotor channel mapping, and mounted lidar structure the lab wants, but it targets Gazebo Classic / ROS 1. The current runtime is ROS Jazzy / Gazebo Harmonic, so the actual SITL-Gazebo JSON bridge should come from official `ardupilot_gazebo`. Model files stay in `world-model` and are mounted into the Gazebo container; plugin code and base image construction stay in `sim-infra`.

## 2026-06-03: NavLab service images use separate Dockerfiles

Decision: keep `navlab-companion`, `navlab-slam-cartographer`, and `navlab-gazebo-sensor` in separate Dockerfiles.

Basis: codebase research and service ownership.

Reason: companion, SLAM, and Gazebo/sensor runtimes have different dependency owners. The sensor image owns YDLidar SDK and the vendor driver, the SLAM image owns Cartographer and NavLab ROS localization packages, and the companion image owns mission/MAVLink/runtime Python plus shared ROS message compatibility. Separate Dockerfiles make manual builds, cache behavior, and future algorithm swaps easier to reason about.

## 2026-06-04: NavLab replay publishes self-contained quadrotor markers

Decision: publish the moving `navlab_iq_quad` replay model on `/sim/markers` as self-contained primitive geometry: body, heading arrow, arms, rotor discs, motors, and X2 lidar marker.

Basis: codebase research and Foxglove replay constraints.

Reason: the Gazebo world already flies `model://navlab_iq_quad`, but ROS MCAP replay cannot directly render Gazebo's nested model tree. File-path mesh resources such as `file:///workspace/...` only work on machines with that exact path, so they fail for Foxglove Cloud and for other developers' local replay. MCAP attachments are not automatically resolved by `visualization_msgs/Marker` mesh resources, and large Collada payloads are not a good fit for repeated ROS Marker messages. Primitive markers are portable and visible anywhere the MCAP is opened. Exact mesh replay should be added later as a Foxglove `SceneUpdate` channel using `ModelPrimitive.data`, not as ROS Marker `file://` resources.

## 2026-06-04: NavLab hover gate requires SLAM-derived ExternalNav

Decision: current completion gate must feed `external_nav_bridge` from SLAM `/odom`, not from `/gazebo/truth/odom`.

Basis: current phase goal and real-machine migration requirement.

Reason: Gazebo truth is useful for diagnosing ArduPilot Gazebo plugin, SITL parameters, coordinate transforms, and FCU ExternalNav acceptance, but it does not exist on the real machine. If the acceptance gate passes while ExternalNav comes from Gazebo truth, the result only proves a diagnostic FCU path, not a real no-GPS SLAM feedback loop. The current phase is complete only when `/scan + /imu/data -> SLAM -> /odom -> /external_nav/odom -> MAVLink ODOMETRY -> SITL EKF -> LOCAL_POSITION_NED` holds during takeoff and hover.

Implementation note: `/gazebo/truth/odom` should still be recorded in rosbag and summary for error analysis. It must be labeled as diagnostic truth, and acceptance should mark the run blocked or not complete if it is used as the ExternalNav source.
