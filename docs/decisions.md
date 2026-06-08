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

## 2026-06-07: NavLab P6 records a portable vehicle shell layer

Decision: record `/navlab/vehicle/markers` in the P6 SLAM hover MCAP and generate it from the FCU pose with primitive `MarkerArray` geometry.

Basis: current replay gap and Foxglove portability.

Reason: P6 already proves the SLAM -> ExternalNav -> EKF -> hover loop, but Foxglove replay still needs a vehicle shell layer to make the motion readable. The shell must not depend on local mesh paths because the same MCAP should open on another laptop or in Foxglove Cloud. A primitive `MarkerArray` following `/ap/v1/pose/filtered` is self-contained, portable, and easy to keep in acceptance as a required topic.

## 2026-06-07: NavLab P7 doctor stays a fast config gate

Decision: make `motion-gate-doctor` validate P7 configuration, rosbag profile, topic contracts, and truth-control boundaries without re-running the full P0-P6 dependency doctor chain by default.

Basis: local P7 implementation and doctor runtime behavior.

Reason: P7 acceptance already launches and validates the full official stack, sensors, SLAM, frame contract, FCU bootstrap, and motion gate. Nesting all prior doctor dependency probes inside P7 doctor makes the lightweight prerequisite check slow enough to obscure config failures. The fast doctor keeps P7 iteration usable while preserving full-stack proof in `motion-gate-acceptance`.

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

Basis: P0 manual probes and `uv run --project orchestration python orchestration/main.py official-baseline-acceptance 30`.

Reason: in the current ROS Jazzy environment, Fast DDS can discover ArduPilot bare DDS endpoints, but `/ap/v1/time` does not deliver samples to `ros2 topic echo` during the P0 probe. Cyclone DDS receives `/ap/v1/time` samples and records them into MCAP, although it may print type-hash warnings for bare DDS endpoints. ArduPilot's DDS launch defaults to domain `0`, so P0 aligns `ROS_DOMAIN_ID=0` with `DDS_DOMAIN_ID=0` for the official baseline while NavLab's normal custom runtime can keep its separate domain.

## 2026-06-06: P1 keeps the official maze and swaps only the lidar path

Decision: after P0, keep the official `ardupilot_gz_bringup iris_maze.launch.py` world and Iris lidar model for P1, and introduce only the NavLab X2 virtual-serial plus vendor-driver scan path.

Basis: local comparison against `/home/nn/workspace/3588/ardupilot_ros` and current P0 baseline artifacts.

Reason: replacing the world, vehicle model, lidar mechanism, and SLAM input at the same time makes failures hard to attribute. The official maze/Iris baseline already exercises ArduPilot Gazebo bringup, DDS, and Cartographer; P1 should isolate the next variable to the X2 mechanism: Gazebo scan source -> X2 protocol emulator -> `ydlidar_ros2_driver` -> `/scan`. P7/P8 should still stay in the official maze for motion and exploration because that scene is already richer than the current NavLab figure-eight world. NavLab's 8 字形 world and custom vehicle model move to a later optional migration phase after the official maze path has proven scan, rangefinder/IMU, SLAM quality, hover, motion, and exploration.

## 2026-06-06: P1 overrides only the official scan bridge output

Decision: keep launching `ardupilot_gz_bringup iris_maze.launch.py`, but bind-mount a P1 bridge YAML over the official `iris_3Dlidar_bridge.yaml` so the official `ros_gz_bridge` no longer publishes ROS `/scan`.

Basis: official launch inspection inside `world-model/navlab-official-baseline:latest` and P1 acceptance runs.

Reason: the official Iris lidar bridge maps Gazebo `/lidar` directly to ROS `/scan`. P1 needs `/scan` to mean “X2 virtual serial -> `ydlidar_ros2_driver` output”, otherwise Cartographer would receive a mixed topic from both `ros_gz_bridge` and the vendor driver. The bridge override preserves the official maze, Iris model, SITL, DDS, odometry, IMU, TF, and point cloud bridges, while freeing `/scan` for the vendor driver. The P1 acceptance blocks if `/scan` has a `ros_gz_bridge` publisher or if Cartographer is not subscribed to the vendor `/scan`.

## 2026-06-07: P7 FCU local-position rate gate allows DDS jitter

Decision: set the P7 `min_fcu_local_position_rate_hz` threshold to 1.5 Hz while keeping the latest-age gate at 1.5 seconds.

Basis: P7 acceptance artifact `artifacts/ros/navlab_companion_sitl_gazebo/20260607_114343/summary.json`.

Reason: the ArduPilot DDS filtered local-position stream is nominally about 2 Hz, but a full P7 run measured 1.93 Hz across the motion-probe window while latest samples stayed fresh and all motion, SLAM, ExternalNav, rangefinder, and rosbag gates passed. A 1.5 Hz floor preserves a real liveness/rate check without failing on scheduler jitter around the nominal 2 Hz publisher.

## 2026-06-07: P7 separates motion coordination from FCU setpoint ownership

Decision: run P7 as a motion coordinator plus the existing `navlab_fcu_controller` owner instead of letting the P7 probe publish `/ap/v1/cmd_vel` directly.

Basis: P7 design/TODO audit after the first green acceptance artifact.

Reason: the P7 design requires the mission/coordinator layer to publish motion intent while the unique FCU controller converts that intent into the movement setpoint output. The first accepted run proved motion but used the probe as the setpoint publisher, which made the owner boundary weaker than the written contract. The P7 controller runtime now subscribes to `/navlab/fcu/setpoint/intent`, publishes `/navlab/fcu/setpoint/output`, `/navlab/fcu/controller/status`, `/navlab/fcu/owner/status`, `/navlab/hover/status`, and owns `/ap/v1/cmd_vel`; the P7 coordinator publishes `/navlab/motion/status` and intent only. The verified artifact is `artifacts/ros/navlab_companion_sitl_gazebo/20260607_121115/summary.json`.

## 2026-06-07: P7 yaw scan window is four seconds

Decision: set the default P7 `yaw_window_sec` to 4.0 seconds while keeping `yaw_rate_radps=0.20` and `min_yaw_delta_rad=0.25`.

Basis: split coordinator/controller P7 acceptance run `artifacts/ros/navlab_companion_sitl_gazebo/20260607_120406/summary.json`.

Reason: after routing motion through the FCU controller intent path, the yaw command includes an extra intent-to-output hop and startup/settle latency. A 3 second yaw window produced about 0.243 rad, just below the gate, while the same controller path with a 4 second window produced 0.450 rad in `artifacts/ros/navlab_companion_sitl_gazebo/20260607_121115/summary.json`. Extending the action window preserves the stricter yaw delta gate instead of lowering the minimum accepted motion.

## 2026-06-07: P8 starts with bounded frontier-lite exploration

Decision: implement P8 as a bounded `frontier_lite` exploration gate that publishes exploration goals and FCU setpoint intents, while the existing `navlab_fcu_controller` remains the only `/ap/v1/cmd_vel` owner.

Basis: P8 design/TODO and the verified P7 coordinator/controller split.

Reason: P8 needs to prove an exploration claim without prematurely requiring a full production Nav2 stack. The bounded strategy uses SLAM map growth, scan clearance, TF/FCU readiness, and task state to choose forward probes or yaw scans, records coverage/progress metrics, and blocks on safety/stuck/owner/truth-input violations. This preserves the P6 hover and P7 motion boundaries while adding a real exploration acceptance artifact: `artifacts/ros/navlab_companion_sitl_gazebo/20260607_144800/summary.json`.

## 2026-06-07: P8 records the official Iris lidar TF as static

Decision: publish `base_link -> base_scan` from the SLAM bringup static TF path with the official Iris lidar offset `z=0.075077`, and make generated official-maze SLAM runtime files pass `laser_frame=base_scan` instead of only `laser_frame_id`.

Basis: Foxglove inspection of `artifacts/ros/navlab_companion_sitl_gazebo/20260607_153832/rosbag/rosbag_0.mcap` and comparison with `/home/nn/workspace/3588/ardupilot_gz/ardupilot_gz_description/models/iris_with_lidar/model.sdf`.

Reason: the original bag used `/scan.header.frame_id=base_scan`, but `/tf_static` contained `base_link -> laser_frame` while `base_link -> base_scan` appeared only on dynamic `/tf`. Foxglove can then report `Missing transform from frame <base_scan> to frame <map>` when visualizing scans/maps. The official model fixes the lidar at `base_link -> base_scan` with `z=0.075077`, so future P3-P8 runs should record that relationship on `/tf_static` directly instead of relying on a post-process MCAP patch.

## 2026-06-07: P9 uses representative replay runs

Decision: keep P8 acceptance conservative, but define a separate P9 representative replay profile that moves farther and moderately faster before building the official-maze Foxglove overlay.

Basis: P8 artifacts and P9 overlay design review.

Reason: the minimum P8 gate proves the exploration control loop, but its default speed and action windows can produce less than a meter of travel, which is not enough to make the official maze overlay useful. P9 should prefer a longer replay run, for example 0.18 m/s with at least 2.5 m path length and five accepted goals, while keeping the same no-truth-input, unique-owner, clearance, stop-drift, SLAM, ExternalNav, and FCU health gates. This preserves P8 as a safety gate and makes P9 a publishable replay artifact instead of an over-aggressive flight gate.

## 2026-06-07: P9 suppresses post-run ROS graph noise for transient topics

Decision: filter CycloneDDS type-hash discovery warnings from ROS shell capture and skip post-run `ros2 topic info` for transient P8 exploration status topics.

Basis: P9 display replay `artifacts/ros/navlab_companion_sitl_gazebo/20260607_223132/summary.json` passed with recorded `/navlab/exploration/*` messages, but terminal output still printed CycloneDDS type-hash warnings and `Unknown topic` lines after the exploration coordinator exited.

Reason: `Failed to parse type hash ... USER_DATA '(null)'` comes from DDS discovery metadata for ArduPilot/micro-ROS participants and does not imply message loss. `/navlab/exploration/status`, `/navlab/exploration/goal`, `/navlab/exploration/coverage`, and `/navlab/motion/status` are transient runtime publishers; after the run exits, `ros2 topic info` can report them as unknown even though rosbag and summary counts prove data was recorded. The acceptance evidence should come from rosbag/profile counts and runtime summaries, not from a late ROS graph query for transient publishers.

## 2026-06-08: P10 prioritizes body-fixed lidar scan integrity before real flight

Decision: make P10 the body-fixed lidar attitude compensation and scan integrity gate, and move the earlier true-lidar exploration strategy optimization out of P10.

Basis: P9 official-maze overlay replay validation and the real-drone risk that motor/ESC/prop mismatch can tilt a non-gimbaled 2D lidar scan plane.

Reason: P8/P9 prove exploration and replay in simulation, but they still assume the 2D lidar scan is close enough to a horizontal slice. On a real drone, small roll/pitch from actuator mismatch can make the scan hit the floor, ceiling, or wrong wall height and silently contaminate SLAM. P10 should therefore make `/scan` an attitude-validated topic, preserve raw scan for diagnostics, reject or clip unsafe tilted scans, and require normal plus fault-injection gates before pushing exploration farther or attempting real-machine flight.

## 2026-06-08: P10 owns `/scan` through a scan integrity filter

Decision: split the X2 chain into `/navlab/x2/scan_raw -> /navlab/x2/scan_normalized -> navlab_scan_integrity_filter -> /scan` for P10.

Basis: P10 implementation and green artifact `artifacts/ros/navlab_companion_sitl_gazebo/20260608_095523/summary.json`.

Reason: keeping the vendor driver directly on `/scan` makes it impossible to prove that SLAM only consumes attitude-validated scans. The raw topic is now diagnostic, the normalizer owns timestamp/frame normalization, and `navlab_scan_integrity_filter` is the unique `/scan` publisher. P10 also exposes a runtime fault-injection topic so mild and hard roll/pitch bias can prove that unsafe tilted scans are warned/dropped instead of silently reaching SLAM.

## 2026-06-08: P10.1 records attitude observability before compensation

Decision: add P10.1 flight attitude metrics, scan attitude quality schema, and best-effort motor-output observability before attempting any richer scan compensation.

Basis: P10 green artifact `artifacts/ros/navlab_companion_sitl_gazebo/20260608_103819/summary.json` and the real-drone concern that non-synchronized motor/ESC outputs can tilt a body-fixed 2D lidar.

Reason: before correcting tilted scans, the gate should first quantify how much the vehicle actually rolled/pitched during flight and whether actuator output is observable at all. The summary now records max/RMS roll and pitch, yaw and attitude-rate metrics, scan tilt/filter/warn counts, and motor-output fields. If no motor/servo/actuator/ESC output topic is exposed in the ROS graph, the summary explicitly reports `motor_output_claim=not_available` with null PWM/RPM/bias fields rather than inventing actuator evidence.

## 2026-06-08: P11 uses P9 replay for bounded 2D scan stabilization

Decision: define P11 as a bounded 2D lidar scan stabilization gate based on the P9 representative replay profile, rather than as a P8 slow-exploration check or a 3D lidar migration.

Basis: P10 proved that hard tilted scans can be dropped safely, but higher-speed representative replay can reduce SLAM scan availability if P10 remains purely drop-only.

Reason: the next risk is medium roll/pitch during faster motion, not a new exploration strategy. P11 should compare a P10 drop-only baseline against a bounded 2D projection candidate under the P9 replay motion profile, keep all pass/compensate/drop thresholds configurable, and preserve the P10 rule that hard tilt and floor-hit risk are rejected instead of being projected into fake walls. Broader exploration strategy optimization moves after P12 airframe disturbance robustness, once the scan input contract is stable under realistic motor/ESC/vibration profiles.

## 2026-06-08: P11 keeps live replay conservative when tilt stays below passthrough

Decision: accept the P11 live P9 representative replay when the vehicle remains in the passthrough tilt zone, while proving bounded projection behavior through the P11 fault profile and recording `compensation_not_triggered_reason` in the summary.

Basis: P11 acceptance artifact `artifacts/ros/navlab_companion_sitl_gazebo/20260608_122544/summary.json` passed with `ok=true`, `blockers=[]`, and `compensation_not_triggered_reason=tilt_never_exceeded_passthrough_tilt_deg`.

Reason: the acceptance run is still the correct P9 representative replay profile, but the simulated FCU held roll/pitch below the configured `passthrough_tilt_deg`, so forcing live compensation would require injecting artificial attitude into the flight loop and would weaken the real replay evidence. P11 therefore records a same-run P10 drop-only baseline estimate, keeps `/scan` uniquely owned by `navlab_scan_stabilization_filter`, and uses the fault profile to prove medium safe tilt, floor-hit risk, hard tilt, stale attitude, and invalid config behavior without projecting unsafe beams into SLAM.

## 2026-06-08: P11 waits longer for replay readiness than P8 slow gates

Decision: give P11 representative replay an explicit `replay_readiness_timeout_sec=90.0` and collect the FCU controller runtime summary after rosbag recording finishes, with `controller_summary_timeout_sec=45.0`.

Basis: failed P11 artifact `artifacts/ros/navlab_companion_sitl_gazebo/20260608_135843/summary.json` had healthy scan stabilization topics but failed because the P8 replay probe timed out before map/controller readiness fully settled, while `controller_runtime_summary.json` appeared later in the same artifact.

Reason: P11 is testing bounded scan stabilization under the P9 representative replay profile, not the minimum P8 slow exploration profile. The scan chain can be healthy while the replay layer needs a longer readiness window for Cartographer map publication and FCU hold-ready completion. Making these waits explicit config fields avoids hiding the timing dependency in code and prevents a late controller summary from being misreported as missing.

## 2026-06-08: P12 focuses on airframe disturbance scan robustness

Decision: define P12 as the motor bias / ESC lag / thrust multiplier / vibration robustness gate for body-fixed 2D lidar scan stabilization, not as an active frontier exploration phase.

Basis: P10/P11 proved scan integrity and bounded 2D stabilization under the current simulated attitude envelope, but the simulated airframe is still too ideal compared with a real drone whose motors, props and ESCs are not perfectly matched.

Reason: before making exploration more aggressive, the system must prove that P11 horizontal recovery remains safe when realistic disturbance sources create sustained roll/pitch bias, response lag, dynamic overshoot and IMU vibration/noise. P12 should run clean and disturbed P9 representative replay profiles, keep all disturbance parameters configurable, compare scan/SLAM/ExternalNav/FCU/map health against clean baseline, and make hard disturbance fail clearly instead of silently polluting SLAM. Active frontier exploration moves after this robustness envelope is known.

## 2026-06-08: P12 uses plugin-level ESC first-order lag

Decision: implement P12 ESC lag as an ArduPilot Gazebo plugin extension instead of the earlier PID/frequency proxy. The official-baseline image now applies `patches/ardupilot_gazebo_esc_lag.patch`, each `<control>` may declare `<escTimeConstantMs>`, and P12 SDF overlays write per-motor ESC time constants directly.

Basis: the proxy approach was useful for the first deterministic profile sweep, but it did not actually model command response delay. The user explicitly required real plugin-level first-order lag before treating P12 as complete.

Reason: P12 is the last scan/airframe robustness gate before real-drone trials. A real first-order command filter keeps the disturbance at the motor-control boundary, preserves per-motor thrust multiplier semantics, avoids pretending `p_gain` is ESC physics, and lets `mild_bias`, `esc_lag`, and `vibration` live P9 replays exercise the same stabilized `/scan` contract under more realistic attitude dynamics.

## 2026-06-08: P10/P11/P12 review follow-up tightens scan robustness contracts

Decision: add explicit attitude-source age gates for P10/P11, document P11 live compensation limits, and make P12 ESC patch reproducibility and map-risk scope explicit.

Basis: review found that scan-attitude timestamp offset is not enough to detect a silent attitude source, P11 same-run baseline is not a true A/B flight, and P12's `escTimeConstantMs` plugin patch is an external dependency.

Reason: stale attitude, biased baseline estimates, and patch drift are all failure modes that can look like healthy topic flow. P10/P11 now expose `max_attitude_source_age_ms=250.0`; P11 docs state current P9 live replay may not trigger compensation unless P12 disturbance profiles push tilt above passthrough; P12 docs record `ardupilot_gazebo` baseline commit `cc0290d964dfa373531963a8fc39093a0836af0a` and downgrade `map_artifact_score` to optional/future rather than a soft hard-gate placeholder.

## 2026-06-08: P12 gates FCU mode during disturbed replay

Decision: require each P12 live disturbed replay to prove FCU mode stays `GUIDED` throughout the configured disturbance window by reading `/ap/v1/status.mode` from the raw MCAP and comparing it to `required_fcu_mode_number=4`.

Basis: P12 can otherwise pass scan/SLAM health even if a disturbance profile pushes ArduPilot into RTL/LAND/failsafe and the replay keeps publishing stale or degraded data.

Reason: the mode gate uses `/navlab/exploration/status` first-to-last samples as the conservative P9 replay disturbance window, so pre-replay bootstrap is excluded but the active exploration/motion period is covered. `/ap/v1/status` and the window topic are required in the P12 raw rosbag profile; missing status data, invalid `mode_number`, or any non-GUIDED sample now produces an explicit blocker instead of being hidden inside generic FCU health.
