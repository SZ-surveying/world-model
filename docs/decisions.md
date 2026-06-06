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

## 2026-06-04: NavLab figure-eight world uses narrow corridors

Decision: use a horizontal figure-eight indoor world with about `0.60 m` side corridors, about `0.725 m` north/south corridors, and a wider `1.55 m` shared waist around the origin.

Basis: current phase goal and local Gazebo validation.

Reason: the current phase needs a world that is tight enough to expose lidar orientation, TF, marker frame, and SLAM map issues before adding exploration. The side and top/bottom corridors are deliberately much narrower than the earlier wide room, while the center waist remains wider so the quad can take off and hover at the origin without immediately colliding. Gazebo truth remains diagnostic, but `/scan_ideal` now publishes real ray data against the narrowed walls.

## 2026-06-04: NavLab down rangefinder is a gazebo-sensor FCU peripheral

Decision: model the down-facing altitude rangefinder in `gazebo-sensor`, not in companion.

Basis: current phase goal and real-machine mechanism.

Reason: on the real drone, altitude hold can be handled by the FCU using its own rangefinder input; the companion computer should not be required just to hold height. In simulation, the equivalent mechanism is `Gazebo range sensor -> gazebo-sensor -> MAVLink DISTANCE_SENSOR -> ArduPilot rangefinder/EKF/altitude controller`. Companion may observe `/rangefinder/down/status`, FCU telemetry, and Gazebo truth for summary and rosbag, but it must not send rangefinder data or directly control throttle/pose for hover.

## 2026-06-04: NavLab replay TF closes through navlab_world-map-odom-base_link

Decision: publish replay/display transforms as `navlab_world -> map -> odom -> base_link -> laser_frame/imu_link`.

Basis: P3 TF validation and Foxglove replay requirements.

Reason: Foxglove needs every displayed sensor frame to resolve into the fixed frame. `navlab_world` is the stable Gazebo/replay frame, while `map`, `odom`, `base_link`, `laser_frame`, and `imu_link` are the frames expected by SLAM and ROS tooling. The bridge is diagnostic/display infrastructure; it does not command Gazebo and does not replace the real SLAM feedback gate. When a SLAM backend later owns `map -> odom`, the replay bridge can be configured or disabled without changing the sensor and FCU topic contracts.

## 2026-06-04: NavLab X2 simulation keeps ROS 0 degrees as vehicle front

Decision: map Gazebo `/scan_ideal` ROS angle `0` directly into the virtual X2 protocol path and keep the vendor driver profile `reversion=false`, `inverted=false`.

Basis: local container smoke against the figure-eight world and YDLidar X2 profile defaults.

Reason: the previous 180-degree remapping plus driver reversion/inversion made `/scan` appear opposite the vehicle heading in Foxglove. The P3 contract is that `/scan_ideal` and `/scan` use the same geometric convention: `0 deg` is the vehicle front, `+90 deg` is left, `-90 deg` is right, and rear is `+/-180 deg`. Mission and scan feature code should consume that convention directly instead of compensating for a reversed scan.

## 2026-06-04: NavLab SLAM runtime uses a backend registry wrapper

Decision: start SLAM through `navlab.slam.cli` and a backend registry instead of assembling Cartographer launch arguments in orchestration.

Basis: codebase research and future backend replacement requirement.

Reason: orchestration should know which container/backend to start, not how a specific SLAM backend launches internally. The stable contract is `/scan + /imu/data -> /odom + SLAM health`; Cartographer is only the current backend. A registry wrapper lets future backends keep the same input/output topics while owning their own launch command, status topic, and internal parameters. It also prevents the host layer from accidentally feeding `external_nav_bridge` with `/gazebo/truth/odom`.

## 2026-06-05: NavLab SLAM ROS packages use NavLab-scoped names

Decision: replace the old Stage 1 ROS package names with NavLab-scoped packages: `navlab_slam_bringup`, `navlab_cartographer_adapter`, `navlab_external_nav_bridge`, `navlab_slam_imu_bridge`, and `navlab_fake_odom`.

Basis: codebase research and SLAM backend replacement requirement.

Reason: names such as `indoor_bringup`, `cartographer_indoor`, and `fake_external_nav` made the current runtime look like the old synthetic Stage 1 path. The SLAM container now exposes a generic contract, `/scan + /imu/data -> /odom + /navlab/slam/status`, while Cartographer remains only the selected backend implementation. This keeps future SLAM backends from inheriting Cartographer-specific or fake-ExternalNav naming.

## 2026-06-05: Cartographer config is part of the completion gate

Decision: treat `navlab_cartographer_2d.lua` and its runtime launch arguments as first-class acceptance inputs, not incidental files behind the adapter.

Basis: codebase research and hover-SLAM diagnostic failures showing `/odom` can be healthy at the transport level while still drifting against Gazebo truth.

Reason: P4 is not complete just because a node publishes `/odom`. The completion gate must prove that `cartographer_ros` is running with an explicit configuration, that the adapter only converts backend TF into odometry, and that real-machine tuning items are documented separately from simulation defaults. The stable contract stays backend-agnostic, but the current backend must still be configured and audited like real SLAM.

## 2026-06-05: Cartographer dependency uses ROS Jazzy binary package first

Decision: use `ros-jazzy-cartographer-ros` in `docker/Dockerfile.slam` as the default Cartographer dependency source.

Basis: local Docker build and current need to configure Cartographer rather than patch its source.

Reason: the SLAM image already builds successfully with the ROS Jazzy binary package, and the project-owned surface is the NavLab launch/config/adapter contract. Downloading upstream Cartographer source into this repo is only needed if the binary package is unavailable, a specific upstream commit must be locked, or the algorithm source must be patched. Keeping the default route binary-based reduces build complexity while still allowing a future source-build stage with pinned commits.

## 2026-06-04: NavLab orchestration uses a task registry

Decision: dispatch orchestration workflows through `src.tasks.registry` instead of hardcoding workflow bodies in `host.py`.

Basis: codebase research and P5 workflow separation.

Reason: `host.py` should stay focused on Docker, compose, container, and runtime-command primitives. Workflows such as image build, doctor, full acceptance, hover acceptance, and future exploration acceptance have different completion gates and artifact semantics. A task registry keeps the CLI stable while allowing each workflow to own its own run logic, output checks, and future companion command. This prevents P5 hover acceptance from being mixed with the older obstacle demonstration gate.

## 2026-06-04: NavLab hover gate is split into FCU diagnostic and SLAM feedback gates

Decision: add a smaller `hover-diagnostic` gate that starts no SLAM container, does not require ExternalNav readiness, and does not send horizontal local-position setpoints.

Basis: codebase research and failing hover acceptance artifacts.

Reason: the previous hover gate mixed two separate failure classes: FCU altitude/rangefinder/GUIDED/takeoff stability and SLAM-derived horizontal ExternalNav drift. The diagnostic gate proves `GUIDED -> arm -> takeoff -> hover` with rangefinder altitude input and observation-only replay topics first. The full `hover` gate remains the completion gate for `/scan + /imu/data -> SLAM /odom -> /external_nav/odom -> MAVLink ODOMETRY -> SITL EKF -> LOCAL_POSITION_NED` feedback.

## 2026-06-05: NavLab must converge to official ArduPilot ROS2 first

Decision: treat the official ArduPilot ROS2, Gazebo, and Cartographer tutorials plus `ArduPilot/ardupilot_ros` as the baseline for NavLab indoor no-GPS SLAM work.

Basis: external research against `ardupilot.org/dev/docs/ros2.html`, `ros2-sitl.html`, `ros2-gazebo.html`, `ros2-cartographer-slam.html`, and the `ardupilot_cartographer` package.

Reason: the current custom `/odom -> /external_nav/odom -> MAVLink ODOMETRY` path is useful for diagnostics, but it is not the official ROS2/DDS route. To avoid proving a synthetic bridge instead of the real mechanism, the next convergence target is `/ap` DDS visibility, official `ardupilot_gz_bringup` structure, official Cartographer Lua/EKF baseline, and eventually ExternalNav feedback through the official ROS2 interface. The MAVLink sender can remain as a fallback or transition tool, not the completion definition.

## 2026-06-05: P0 official baseline has a separate failing gate

Decision: implement P0 as `official-baseline-doctor` plus `official-baseline-acceptance`, separate from the NavLab hover and obstacle acceptance tasks.

Basis: codebase research and P0 execution.

Reason: Cartographer dependencies can be present while the runtime is still using NavLab's custom SITL/Gazebo/MAVLink fallback route. The P0 gate must therefore report `ok=false` when `/ap` DDS nodes/topics are absent, `external_nav_route` is not `official_dds`, or the Gazebo bringup mode is not the official equivalent. This makes the current failure useful: it proves the project has not accidentally relabeled diagnostic MAVLink feedback as the official ArduPilot ROS2 baseline.

## 2026-06-06: P0 needs an official baseline runtime image

Decision: keep `world-model/navlab-companion`, `world-model/navlab-slam-cartographer`, and `remote-sitl-lab/ardupilot-sitl` as their current service images, but do not treat any of them as the official ROS2/DDS baseline runtime.

Basis: codebase research, official ArduPilot package inspection, and P0 doctor execution.

Reason: the current SITL image contains `arducopter` but no ROS2 CLI; companion and SLAM images contain ROS2, but lack `ardupilot_sitl`, `ardupilot_msgs`, `ardupilot_dds_tests`, `micro_ros_agent`, `ardupilot_gz_bringup`, `ardupilot_gz_application`, `ardupilot_gazebo`, `ardupilot_gz_gazebo`, `ardupilot_sitl_models`, and `ardupilot_cartographer`. Official bringup requires `ardupilot_sitl sitl_dds_udp.launch.py`, which includes the micro-ROS agent, SITL, and MAVProxy, while `ardupilot_gz_bringup` includes that DDS launch when `use_dds_agent=true`. The next P0 implementation step should therefore create or select a dedicated official baseline image/layer that installs these packages from ArduPilot source repos, rather than weakening P0 to pass through the existing MAVLink fallback route.

## 2026-06-06: P0 pins ardupilot_ros Cartographer source to humble

Decision: keep the official baseline image on ROS Jazzy, but clone `ArduPilot/ardupilot_ros` from the `humble` branch for the `ardupilot_cartographer` package.

Basis: codebase research plus official-source inspection.

Reason: the current `main` and `jazzy` branches of `ArduPilot/ardupilot_ros` only expose the `ardupilot_ros` metapackage, while the official Cartographer launch/config package still exists on the `humble` branch as `ardupilot_cartographer`. P0 is a baseline conformance check for the official DDS/SITL/Gazebo/Cartographer route, so the image must include that package explicitly instead of silently replacing it with NavLab's custom SLAM wrapper. This pin should be revisited when the official repository restores or renames the Cartographer package on a Jazzy-native branch.

## 2026-06-06: P0 uses the Jazzy micro-ROS Agent source

Decision: build `micro-ROS/micro-ROS-Agent` from its `jazzy` branch in the official baseline image.

Basis: Docker build failure and source branch inspection.

Reason: the Humble micro-ROS Agent source fetches an XRCE-DDS Agent that requires `fastcdr` v1, while the ROS Jazzy base provides `fastcdr` v2.2.7. P0 still keeps `ardupilot_cartographer` on the ArduPilot `humble` branch because that package is missing from the current ArduPilot `jazzy` branch, but Micro-ROS Agent must match the ROS distribution ABI so the official DDS bridge can build.

Follow-up: install `ros-jazzy-micro-ros-msgs` from the ROS Jazzy binary repository rather than cloning `micro_ros_msgs` source. The Jazzy Agent source depends on this message package, and using the distro binary keeps message generation and middleware ABI aligned with the base ROS installation.

## 2026-06-06: P0 Dockerfile keeps apt dependencies independent from source refs

Decision: declare ArduPilot and micro-ROS source `ARG`s after the official baseline image's apt dependency layer.

Basis: Docker build behavior during P0 image iteration.

Reason: changing a source branch such as `MICRO_ROS_AGENT_REF` should invalidate only the source clone/build layers. If source refs are in scope before the apt `RUN`, Docker can rerun the expensive and network-sensitive system dependency layer even though the package list did not change. Keeping the dependency layer independent makes repeated official-baseline convergence practical.

## 2026-06-06: P0 installs Micro-XRCE-DDS-Gen explicitly

Decision: clone `ardupilot/Micro-XRCE-DDS-Gen` at `v4.7.0`, build it with Gradle, and put its `scripts` directory on `PATH` inside the official baseline image.

Basis: Docker build failure while compiling `ardupilot_sitl`.

Reason: the official ArduPilot DDS SITL package invokes `microxrceddsgen` during configure. Building only `micro_ros_agent` is not enough: the agent is the runtime DDS bridge, while `Micro-XRCE-DDS-Gen` is the code generator needed by ArduPilot's ROS2 package build. The image should fail early if `microxrceddsgen -help` is unavailable, because later DDS launch checks cannot be meaningful without generated message support.

## 2026-06-06: P0 builds Micro-XRCE-DDS-Gen with Java 17

Decision: install `openjdk-17-jdk-headless` and set `JAVA_HOME` for the official baseline image instead of using the distro `default-jdk`.

Basis: Docker build failure in `Micro-XRCE-DDS-Gen` Gradle configure.

Reason: Ubuntu noble's `default-jdk` can resolve to Java 21, while the `Micro-XRCE-DDS-Gen` Gradle wrapper currently uses Gradle 7.6 and fails with `Unsupported class file major version 65`. Java 17 is supported by that Gradle generation and is sufficient for building the XRCE-DDS code generator, so pinning Java 17 keeps the P0 image reproducible without patching upstream Gradle files.

## 2026-06-06: P0 official DDS probe uses Cyclone DDS on Jazzy

Decision: set the P0 official baseline runtime to `rmw_cyclonedds_cpp` and keep the official baseline ROS/DDS domain at `0`.

Basis: P0 manual probes and `just navlab-official-baseline-acceptance 30`.

Reason: in the current ROS Jazzy environment, Fast DDS can discover ArduPilot bare DDS endpoints, but `/ap/v1/time` does not deliver samples to `ros2 topic echo` during the P0 probe. Cyclone DDS receives `/ap/v1/time` samples and records them into MCAP, although it may print type-hash warnings for bare DDS endpoints. ArduPilot's DDS launch defaults to domain `0`, so P0 aligns `ROS_DOMAIN_ID=0` with `DDS_DOMAIN_ID=0` for the official baseline while NavLab's normal custom runtime can keep its separate domain.

## 2026-06-06: P1 keeps the official maze and swaps only the lidar path

Decision: after P0, keep the official `ardupilot_gz_bringup iris_maze.launch.py` world and Iris lidar model for P1, and introduce only the NavLab X2 virtual-serial plus vendor-driver scan path.

Basis: local comparison against `/home/nn/workspace/3588/ardupilot_ros` and current P0 baseline artifacts.

Reason: replacing the world, vehicle model, lidar mechanism, and SLAM input at the same time makes failures hard to attribute. The official maze/Iris baseline already exercises ArduPilot Gazebo bringup, DDS, and Cartographer; P1 should isolate the next variable to the X2 mechanism: Gazebo scan source -> X2 protocol emulator -> `ydlidar_ros2_driver` -> `/scan`. P7/P8 should still stay in the official maze for motion and exploration because that scene is already richer than the current NavLab figure-eight world. NavLab's 8 字形 world and custom vehicle model move to a later optional migration phase after the official maze path has proven scan, rangefinder/IMU, SLAM quality, hover, motion, and exploration.

## 2026-06-06: P1 overrides only the official scan bridge output

Decision: keep launching `ardupilot_gz_bringup iris_maze.launch.py`, but bind-mount a P1 bridge YAML over the official `iris_3Dlidar_bridge.yaml` so the official `ros_gz_bridge` no longer publishes ROS `/scan`.

Basis: official launch inspection inside `world-model/navlab-official-baseline:latest` and P1 acceptance runs.

Reason: the official Iris lidar bridge maps Gazebo `/lidar` directly to ROS `/scan`. P1 needs `/scan` to mean “X2 virtual serial -> `ydlidar_ros2_driver` output”, otherwise Cartographer would receive a mixed topic from both `ros_gz_bridge` and the vendor driver. The bridge override preserves the official maze, Iris model, SITL, DDS, odometry, IMU, TF, and point cloud bridges, while freeing `/scan` for the vendor driver. The P1 acceptance blocks if `/scan` has a `ros_gz_bridge` publisher or if Cartographer is not subscribed to the vendor `/scan`.
