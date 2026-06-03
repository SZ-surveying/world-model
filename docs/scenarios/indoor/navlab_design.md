# 阶段 1.5 设计：无 GPS SITL + Gazebo + Companion 闭环

## 1. 目标

阶段 1.5 的目标是把阶段 0 的 Gazebo 世界和阶段 1 的 ArduPilot SITL ExternalNav 闭环合成一个更接近真实无人机的系统：

```text
ArduPilot SITL = 无人机飞控
companion computer = 机载计算盒子
Gazebo = 无 GPS 室内世界、障碍物和传感器环境
mavlink-router = 飞控通信总线
```

真实任务场景默认没有 GPS。飞控必须依赖机载计算盒子提供的外部导航反馈完成状态估计，计算盒子从飞控和传感器拿数据，处理 SLAM、避障和任务控制，再通过 MAVLink 把 ExternalNav 与控制指令下发给 SITL。

一句话定义：

**阶段 1.5 是“独立 companion 镜像驱动的无 GPS SITL + Gazebo 感知避障闭环”。**

## 2. 为什么单独拆出阶段 1.5

阶段 0 已经能做到：

```text
Gazebo /scan
  -> /scan_features
  -> /planner/cmd_vel
  -> cmd_vel_executor
  -> Gazebo set_pose
```

这条链路适合验证 Gazebo 传感器、障碍物和最小 stop guard，但它不是飞控闭环，因为规划器直接移动 Gazebo 模型。

阶段 1 已经能做到：

```text
/odom
  -> external_nav_bridge
  -> MAVLink ODOMETRY
  -> ArduPilot SITL
  -> hold / local setpoint
```

这条链路适合验证飞控消费 ExternalNav 和 MAVLink setpoint，但还没有把 Gazebo 世界中的障碍物、传感器和可视化机体纳入真实任务闭环。

阶段 1.5 连接二者，但仍然不引入复杂室内场景模型和世界模型推理。它的边界是：

- 不再让 `cmd_vel_executor` 直接控制 Gazebo UAV 位姿
- 由 ArduPilot SITL 执行 arm、takeoff、hover、forward、avoid、hold
- 由 companion computer 处理 SLAM、ExternalNav、传感器特征、避障状态机和 rosbag
- 由 Gazebo 提供无 GPS 室内世界、障碍物、可视化 UAV 和 `/scan`

## 3. 系统边界

### 3.1 ArduPilot SITL

职责：

- 模拟真实无人机飞控
- 在无 GPS 参数配置下运行
- 接收 companion 发来的 MAVLink `ODOMETRY`
- 接收 companion 发来的 `COMMAND_LONG` 和 local setpoint
- 输出飞控状态、IMU、EKF、local position、COMMAND_ACK 等遥测

不负责：

- 运行 SLAM
- 解释 Gazebo `/scan`
- 做高层避障决策
- 直接读取世界模型或地图

### 3.2 Companion computer 镜像

职责：

- 模拟真实机载计算盒子
- 从 MAVLink 或 DDS/MAVROS 入口读取飞控 IMU 和状态
- 从 Gazebo/ROS 读取 `/scan` 和 `/scan_features`
- 运行或托管 `imu_bridge`、`cartographer_indoor`、`external_nav_bridge`
- 将 `/external_nav/odom` 转成 MAVLink `ODOMETRY`
- 通过 MAVLink 发 arm、takeoff、local velocity/position setpoint
- 统一输出状态 topic 和 rosbag，方便 Foxglove 回放

建议镜像内容：

```text
ROS2 Jazzy
Cartographer / SLAM backend
imu_bridge
cartographer_indoor
external_nav_bridge
mavlink_external_nav_sender
mavlink telemetry decoder
mavlink_obstacle_mission_controller
rosbag2 + MCAP
Foxglove-compatible topic/status output
```

### 3.3 Gazebo

职责：

- 提供室内无 GPS 世界
- 提供障碍物和可视化 UAV 模型
- 提供随 UAV 运动的 2D lidar
- 发布 `/scan`
- 接收 pose mirror 对 UAV 模型的位姿同步

不负责：

- 直接执行 planner 输出
- 直接控制飞控模式
- 代替 ArduPilot 做飞行动力学判定

## 4. 主闭环

阶段 1.5 的主闭环是：

```text
Gazebo UAV + lidar
  -> /scan
  -> /scan_features
  -> companion obstacle mission controller
  -> MAVLink SET_POSITION_TARGET_LOCAL_NED / COMMAND_LONG
  -> ArduPilot SITL
  -> LOCAL_POSITION_NED / IMU / EKF telemetry
  -> companion pose mirror + imu_bridge
  -> Gazebo UAV pose + /imu/data
  -> SLAM / ExternalNav
  -> MAVLink ODOMETRY
  -> ArduPilot SITL EKF
```

这里有两条反馈线：

1. **定位反馈线**：`/scan + FCU IMU -> SLAM -> /odom -> ExternalNav -> MAVLink ODOMETRY -> SITL`
2. **世界同步线**：`SITL LOCAL_POSITION_NED -> Gazebo pose mirror -> Gazebo UAV/lidar pose`

第一版允许用 SITL `LOCAL_POSITION_NED` 同步 Gazebo UAV 可视化模型。这样 Gazebo 传感器会随着飞控认为的无人机位置变化，能先验证消息边界和任务闭环。

## 5. 无 GPS 约束

阶段 1.5 默认按无 GPS 室内飞行设计：

- SITL 参数禁止依赖 GPS 作为主定位来源
- EKF 的水平位置、速度和 yaw 来源优先使用 ExternalNav
- companion 必须能观测 ExternalNav 是否 healthy
- `latest_visodom_unhealthy_sec` 不能在启动宽限期后持续出现
- mission controller 在 ExternalNav、IMU、scan 任一关键输入不健康时必须进入 hold 或 fail-safe stop

无 GPS 并不表示没有高度接口。阶段 1.5 第一版高度可以继续使用 SITL/ArduPilot 内部高度估计和固定 takeoff altitude，后续再把高度估计 gate 接入 `external_nav_bridge`。

## 6. MAVLink 接口

### 6.1 Companion 从 SITL 读取

推荐读取：

```text
HEARTBEAT
COMMAND_ACK
STATUSTEXT
ATTITUDE
LOCAL_POSITION_NED
GLOBAL_POSITION_INT
EKF_STATUS_REPORT
EXTENDED_SYS_STATE
HIGHRES_IMU 或 RAW_IMU
```

用途：

- `HEARTBEAT`：确认飞控在线、模式、arm 状态
- `COMMAND_ACK`：确认 arm/takeoff/mode 命令是否被接受
- `LOCAL_POSITION_NED`：验收飞控本地位置、驱动 Gazebo pose mirror
- `EKF_STATUS_REPORT`：观测 EKF 状态
- `HIGHRES_IMU` / `RAW_IMU`：转成 `/imu/data` 供 SLAM 使用
- `STATUSTEXT`：捕捉 `VisOdom: not healthy` 等飞控侧故障

### 6.2 Companion 写入 SITL

推荐写入：

```text
MAVLink ODOMETRY
COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM
COMMAND_LONG MAV_CMD_NAV_TAKEOFF
SET_POSITION_TARGET_LOCAL_NED
```

使用原则：

- `MAV_CMD_*` 只用于离散命令，例如 arm、takeoff、land、mode 类动作
- 连续运动控制使用 `SET_POSITION_TARGET_LOCAL_NED`
- ExternalNav 使用 MAVLink v2 `ODOMETRY`
- `/ap/tf` 只作为诊断或 DDS 预留路径，不和 MAVLink `ODOMETRY` 同时作为同一 SITL run 的独立控制源

## 7. SLAM 输入边界

SLAM 不应该消费飞控融合后的 `LOCAL_POSITION_NED` 作为定位输入。否则会形成不可信的自反馈：

```text
FCU fused position -> SLAM -> ExternalNav -> FCU
```

正确边界是：

```text
Gazebo /scan
  + FCU IMU
  -> SLAM
  -> /odom
  -> external_nav_bridge
  -> MAVLink ODOMETRY
```

`LOCAL_POSITION_NED` 可以用于：

- Gazebo pose mirror
- 验收指标
- Foxglove 回放
- 控制器状态观测

但不能作为 SLAM 的真值输入。

## 8. 悬停、前进、绕障任务

第一版 mission controller 采用保守状态机：

```text
WAIT_READY
  等 SITL heartbeat、/scan fresh、/imu/status streaming、ExternalNav healthy

GUIDED
  切 GUIDED 或确认当前模式可接受 local setpoint

ARM
  发 MAV_CMD_COMPONENT_ARM_DISARM

TAKEOFF
  发 MAV_CMD_NAV_TAKEOFF，高度例如 0.8 m

HOVER_SETTLE
  连续发送 vx=0, vy=0, z=-takeoff_alt 的 local setpoint
  等本地位置和 ExternalNav 稳定

FORWARD
  发送低速 vx，例如 0.1-0.2 m/s

AVOID
  front_min 低于 obstacle_seen/avoid 阈值后降低 vx，并发送横向 vy
  第一版可固定向 +Y 绕行

PASS_OBSTACLE
  x 越过障碍物后继续前进

RETURN_TRACK
  横移回 y=0 或目标航线

FINAL_HOLD
  vx=0, vy=0，保持高度和位置
```

阶段 1.5 第一版可以先使用已知场景绕障：

```text
障碍物前缘 x=5
障碍物宽度 y=-2..2
绕行目标 y=2.6
越障后 x>=6.5
返回 y=0
```

第二版再把 `/scan_features` 扩展为左右扇区特征，让控制器根据实时 clearance 选择左绕或右绕。

## 9. Rosbag 与 Foxglove 回放

阶段 1.5 每次 acceptance 必须绑定 rosbag，默认使用 MCAP，topic profile 固定为：

```text
profiles/navlab-rosbag-topics.txt
```

rosbag 必须覆盖四类信息：

1. **世界和传感器**：`/scan`、`/scan_features`、`/scan_nearest_point`、`/sim/markers`
2. **定位反馈**：`/imu/data`、`/imu/status`、`/tf`、`/odom`、`/cartographer/status`
3. **ExternalNav 和 MAVLink 注入状态**：`/external_nav/odom`、`/external_nav/status`、`/mavlink_external_nav/status`
4. **任务与回放状态**：`/navlab/mission/status`、`/navlab/mavlink/status`、`/sim/uav_pose`、`/sim/log`

artifact 目录建议：

```text
artifacts/ros/navlab_companion_sitl_gazebo/<timestamp>/
  summary.json
  rosbag/
    metadata.yaml
    rosbag_0.mcap
  logs/
    sitl.log
    mavlink-router.log
    companion.log
    gazebo.log
  foxglove_notes.md
```

Foxglove 回放最低要求：

- 能看到 Gazebo/UAV marker
- 能看到 `/sim/uav_pose` 或等价 pose
- 能回放 `/scan` 和 `/scan_features.front_min`
- 能看到 mission 状态从 wait_ready、guided、arm、takeoff、hover、forward、avoid 到 final_hold
- 能看到 ExternalNav/IMU 是否持续 healthy
- 能根据 rosbag 判断是感知、SLAM、ExternalNav、MAVLink 控制还是 SITL 侧出了问题

## 10. 验收标准

阶段 1.5 完成时，`summary.json` 至少应证明：

```json
{
  "ok": true,
  "gps_free": true,
  "rosbag_started_before_mission": true,
  "rosbag_covers_full_mission": true,
  "companion_ready": true,
  "sitl_heartbeat": true,
  "imu_source": "fcu_mavlink_or_dds",
  "scan_fresh": true,
  "external_nav_healthy": true,
  "gazebo_pose_mirror_ok": true,
  "hover_ok": true,
  "forward_progress_ok": true,
  "obstacle_detected": true,
  "avoidance_setpoint_sent": true,
  "lateral_detour_ok": true,
  "final_hold_ok": true,
  "rosbag_profile_ok": true
}
```

最低判定规则：

- companion 镜像必须独立于 SITL 镜像启动
- SITL 不能依赖 GPS 完成任务
- companion 能从飞控侧拿到 IMU 和状态
- Gazebo `/scan` 能驱动 SLAM 或避障逻辑
- ExternalNav 能持续注入 SITL
- UAV 能先悬停，再前进，再根据障碍物触发绕行动作，最后 hold
- rosbag 能在 Foxglove 中回放完整任务链路

## 11. 实现决策记录

### 2026-06-02：NavLab 第一刀使用 pose mirror，而不是 Gazebo physics plugin

Decision: 第一版用 `LOCAL_POSITION_NED -> Gazebo set_pose` 的 pose mirror 同步 Gazebo UAV，而不是直接接 Gazebo 物理插件。

Basis: 本地代码和阶段边界检查。

Reason: 阶段 0 已经验证 Gazebo `set_pose` 能驱动带 lidar 的 UAV marker，阶段 1 已经验证 SITL local setpoint/ExternalNav；pose mirror 能最快证明 companion/SITL/Gazebo 消息边界，避免把 Gazebo 动力学插件、控制器、SLAM、避障同时放进一个不可定位的失败点。

### 2026-06-02：rosbag profile 兼容 `required /topic` 与纯 `/topic`

Decision: NavLab 复用 Stage 1 的 `required|optional` topic profile 表达验收要求，同时让 Python rosbag loader 继续兼容 Stage 0 的纯 topic 文件。

Basis: 本地 profile 约定检查。

Reason: NavLab acceptance 需要校验 required topics，而现有 Stage 0 runtime 仍依赖纯 topic 列表；兼容解析能避免分裂两套录包工具。

### 2026-06-02：FCU IMU bridge 优先 HIGHRES_IMU，RAW_IMU 仅作显式 fallback

Decision: `mavlink_imu_bridge` 默认优先消费 MAVLink `HIGHRES_IMU` 和 `SCALED_IMU`，`RAW_IMU` 需要显式 `--allow-raw-imu` 才启用。

Basis: MAVLink common message units 与本地 SLAM 输入需求。

Reason: `HIGHRES_IMU` 使用 SI units，更适合直接填充 `sensor_msgs/msg/Imu`；`SCALED_IMU` 可按 millig / millirad/s 转成 SI units；`RAW_IMU` 的 raw fallback 不应默认进入 SLAM，避免把单位不明确或未归一化的数据冒充可用 FCU IMU。

### 2026-06-02：NavLab 第一版绕障采用固定左绕策略

Decision: `mavlink_obstacle_mission_controller` 第一版使用当前 `uav_obstacle_5m` 世界的已知几何，检测到前方障碍后固定向 `+Y` 绕行。

Basis: 阶段 0 当前世界与 `/scan_features` 合约。

Reason: 现有 `/scan_features` 已有 `front_min/left_min/right_min`，但 NavLab 第一目标是证明 companion 通过 MAVLink 驱动 SITL 完成 hover -> forward -> avoid -> hold 的闭环；固定左绕能先把控制链路和 rosbag 回放跑通，后续再把策略升级为左右 clearance 自适应选择。

### 2026-06-02：NavLab acceptance 只用 rosbag metadata 判断 Foxglove topic 可回放

Decision: NavLab acceptance 生成 `foxglove_notes.md`，并用 `profiles/navlab-rosbag-topics.txt` 校验 required topics 是否存在且 message count 大于 0；不在脚本里启动 Foxglove GUI。

Basis: 本地 Stage 1 artifact 模式和无头 CI/容器环境约束。

Reason: Foxglove 回放能力的可自动化判据是 MCAP rosbag 是否包含可视化和诊断所需 topics。GUI 启动和人工查看属于后续人工复盘，不应成为 headless acceptance 的阻塞条件。

### 2026-06-02：NavLab mission 必须在 rosbag 启动后开始

Decision: `navlab-acceptance` 禁用 companion 的 mission autostart，只让 pose mirror 和 IMU bridge 自动启动；脚本等待 `ros2 bag record` 启动后，再显式运行 `mavlink_obstacle_mission_controller`。

Basis: 用户要求真实跑一趟后，rosbag 必须能复现完整任务，而不是只录到任务中段或结束状态。

Reason: NavLab 的验收对象不是单个 topic 是否存在，而是无 GPS 任务闭环是否完整发生。先录包再启动 mission controller，才能在 Foxglove 中看到 wait_ready、GUIDED/arm/takeoff、hover、forward、avoid 和 final hold 的完整时序。

### 2026-06-02：NavLab companion 镜像从 ROS Jazzy base 单独构建

Decision: 新增 `docker/Dockerfile.companion`，以 `remote-sitl-lab/ros-jazzy-base:latest` 为基础构建 `world-model/navlab-companion:latest`；NavLab compose 服务只使用这个 companion 镜像，不再复用 `world-model/sim-python:latest`。

Basis: NavLab 架构中，SITL 是飞控，Gazebo 是世界和传感器，companion 是真实无人机上的计算盒子模拟体。

Reason: 独立 companion 镜像能把 MAVLink telemetry、pose mirror、mission controller、rosbag acceptance 和 Foxglove artifact 依赖边界固定下来。SLAM 后端另由 `navlab-slam-cartographer` 镜像承载，避免 Cartographer 依赖污染 companion 编排层。

### 2026-06-02：NavLab SLAM/ExternalNav 运行在 runtime，而不是 acceptance runner

Decision: companion runtime 默认启动 pose mirror、ExternalNav bridge 和 MAVLink ExternalNav sender；SLAM 后端由 orchestration 作为独立容器启动；mission controller 默认不自启。acceptance 只负责录 rosbag、启动 mission、采样 topic 和生成 summary。

Basis: 用户要求模拟真实无 GPS 飞行架构，其中计算盒子从 SITL MAVLink 拆分 IMU/状态，运行 SLAM，再通过 MAVLink 给 SITL 下发 ExternalNav 和任务 setpoint。

Reason: 这样 rosbag 记录到的是 runtime/SLAM 后端真实发布的 `/cartographer/status`、`/external_nav/status`、`/mavlink_external_nav/status` 和 `/navlab/mission/status`，而不是测试容器临时拼出来的链路。

### 2026-06-03：NavLab runtime 合并 Stage 0 仿真辅助节点

Decision: 删除独立 `sim-runtime` 编排和 `sim` CLI；`world_marker_publisher` 与 `scan_features_publisher` 作为 NavLab companion runtime 的 function process 启动。NavLab acceptance 不再启动单独 runner 容器，而是 `docker exec` 到同一个 `navlab-companion` 容器执行录包、mission 和 summary 生成。

Basis: Stage 0 和 Stage 1 已完成，NavLab 的目标是模拟真实无人机无 GPS 架构：SITL 是飞控，Gazebo 是世界，companion 是唯一计算盒子。

Reason: marker、scan feature、MAVLink mission 和 rosbag acceptance 放在 companion runtime 边界内，可以避免“测试容器拼链路”的假象；Foxglove 回放看到的是一次 NavLab runtime 驱动 SITL/Gazebo 的完整任务，而不是 Stage 0 辅助容器和 NavLab 容器混合出来的流程。

### 2026-06-03：SLAM 后端应作为可替换镜像，而不是绑定在 companion 镜像内

Decision: NavLab 使用独立 `navlab-slam` 动态容器运行 `world-model/navlab-slam-cartographer:latest`，companion 只依赖固定 ROS 接口，不依赖具体 SLAM 算法实现。

Basis: 后续可能替换 Cartographer、FAST-LIO、VINS、RTAB-Map 或自研定位算法；算法镜像的系统依赖、GPU/CPU 需求、workspace 和调参文件都不应污染 companion 的 MAVLink/mission/ExternalNav 编排层。

Reason: companion 的职责是从 SITL MAVLink 拆 IMU/状态、发布任务状态、消费定位结果并下发 ExternalNav/setpoint；SLAM 后端的职责是消费 `/scan`、`/navlab/fcu_imu/data`、`/tf` 等输入，稳定输出 `/odom`、`/tf` 和健康状态。只要保留这组接口，SLAM 镜像就可以独立替换，NavLab acceptance 也能保持同一套 rosbag/Foxglove 验收标准。

### 2026-06-03：NavLab build/run 镜像标签统一由全局配置驱动

Decision: `lab_env/config.toml` 新增 `[navlab.images]`、`[navlab.images.companion]` 和 `[navlab.images.slam]`，统一定义 companion/SLAM 的 repository、tag strategy、Dockerfile、build context 和 build target；`justfile` 只调用 `python -m lab_env.main navlab build ...`，不再直接写 `docker build` 参数。

Basis: profile 里的 `companion_image` / `orchestration.slam.image` 是运行时覆盖项；如果 justfile 的 build tag 写死，就会出现 build 和 run 指向不同镜像的维护风险。

Reason: 默认情况下，build 和 run 都从同一个 NavLab images 配置生成镜像名；最终 tag 的解析顺序是 CLI `--tag` 覆盖配置策略，否则根据 `tag_strategy` 生成，例如 `latest` 或 `git-commit`。需要临时测试某个运行镜像时仍可在 profile 覆盖，但常规构建入口不会和默认运行配置分裂。

### 2026-06-03：正式名称收敛为 NavLab

Decision: 原阶段编号代码包、CLI、just recipes、Docker image、profile、artifact 和 ROS topic namespace 收敛为 `navlab` / `NavLab`。

Basis: 阶段编号适合任务拆解，但不适合作为长期工程模块名；当前能力已经从 Stage 0/1 过渡到一个可持续迭代的无 GPS 导航实验栈。

Reason: `NavLab` 是一个短名称，不绑定 Cartographer、Gazebo、ArduPilot 或任何单一 SLAM 后端。后续新增 `navlab-slam-fastlio`、`navlab-slam-vins` 或自研算法镜像时，可以复用同一套 companion、MAVLink、rosbag 和 Foxglove acceptance 边界。
