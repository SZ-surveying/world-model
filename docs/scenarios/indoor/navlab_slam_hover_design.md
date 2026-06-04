# NavLab 无 GPS SLAM 反馈悬停设计

## 1. 阶段目标

当前阶段只解决一件事：在 Gazebo 室内世界中，让 ArduPilot SITL 像真实飞控一样工作，让机载计算链路只通过传感器、SLAM、ExternalNav 和 MAVLink 闭环影响飞行，最终完成可观测、可回放、可复现实验的真实悬停。

这个阶段通过后，才进入绕圈探索、主动避障、Nav2 或世界模型规划。原因很简单：如果“真实雷达 + IMU + SLAM + ExternalNav + FCU 悬停”没有打牢，后续任何复杂任务都会被定位漂移、坐标系错位、飞控未真正接管、Gazebo 被脚本直接移动等问题污染。

本阶段的完成定义：

- Gazebo 中的无人机由 ArduPilot Gazebo JSON plugin 和 SITL 物理闭环驱动。
- 上游 Python/ROS 节点不直接调用 Gazebo `set_pose` 移动无人机。
- X2 雷达链路输出真实可用的 `/scan`，不是只满足消息结构的假数据。
- SLAM 后端消费 `/scan` 和 FCU IMU，输出连续稳定的 `/odom`。
- ExternalNav bridge 把 SLAM odom 转成 `/external_nav/odom`。
- MAVLink sender 把 `/external_nav/odom` 转成 MAVLink `ODOMETRY` 发给 SITL。
- SITL EKF 接受 ExternalNav，进入 `GUIDED`、arm、takeoff，并输出 `LOCAL_POSITION_NED`。
- companion 只读取 FCU 本地位置做可视化和状态记录，不把它回灌成 SLAM 输入。
- rosbag MCAP 能在 Foxglove 中复现起飞、悬停、传感器、SLAM、ExternalNav、FCU 状态和模型运动。

## 2. 非目标

当前阶段不做这些事情：

- 不做完整区域探索。
- 不做绕 8 字一圈的任务验收。
- 不把 Gazebo truth 当作最终 ExternalNav 来源。
- 不用 Python 直接改 Gazebo 机体位姿来伪造飞行。
- 不把 `LOCAL_POSITION_NED` 当作 SLAM 输入。
- 不要求世界模型或学习算法参与飞行决策。
- 不要求 Nav2 接管导航。

这些能力可以作为后续阶段加入，但不能提前混进当前验收，否则很难判断问题到底出在世界、传感器、SLAM、飞控还是任务算法。

## 3. 系统边界

### 3.1 Orchestration

Orchestration 是 host 侧批处理编排层，负责：

- 读取 `orchestration/config.toml`。
- 启动 Gazebo、SITL、mavlink-router、gazebo-sensor、SLAM、companion。
- 等待各服务 ready。
- 启动 acceptance runner。
- 录制 rosbag MCAP。
- 收集 `summary.json`、日志和 rosbag topic profile。
- 运行完成后上传 Foxglove 云端。

Orchestration 不发布 ROS topic，不执行 SLAM，不直接控制无人机。

### 3.2 Gazebo

Gazebo 是虚拟物理世界，负责：

- 加载室内 8 字形走廊世界。
- 加载真实可见的 `navlab_iq_quad` 无人机模型。
- 通过 ArduPilot Gazebo JSON plugin 和 SITL 交换物理状态与电机控制。
- 提供 X2 雷达 ray sensor 的理想扫描来源。
- 发布 Gazebo 内部 pose、传感器和调试状态。

Gazebo 只提供物理和传感器，不做任务控制。

### 3.3 ArduPilot SITL

SITL 在本系统里等价于真实无人机飞控，负责：

- 运行 ArduCopter。
- 接收 Gazebo JSON plugin 的物理状态。
- 接收 MAVLink `ODOMETRY` 作为 ExternalNav。
- 接收 `GUIDED`、arm、takeoff 和 local position target。
- 输出 `HEARTBEAT`、`COMMAND_ACK`、`STATUSTEXT`、`ATTITUDE`、`LOCAL_POSITION_NED`、`EKF_STATUS_REPORT` 等遥测。

真实机器部署时，SITL 会被真实 FCU 替换，但 companion、SLAM、ExternalNav、rosbag 的接口应尽量保持不变。

### 3.4 gazebo-sensor

gazebo-sensor 是传感器仿真服务，负责把 Gazebo 理想 ray sensor 转成更接近真实 X2 的 ROS 输出：

```text
Gazebo /scan_ideal
  -> X2 virtual serial emulator
  -> ydlidar_ros2_driver_node
  -> /scan
```

这里的关键不是“造一个 LaserScan 消息”，而是尽量复现真实 X2 驱动链路，包括扫描频率、角度方向、量程、丢点、噪声和驱动输出约定。

### 3.5 SLAM

SLAM 独立成单独服务镜像，当前可以先用 Cartographer，后续可以替换为其他后端。SLAM 服务的接口契约必须稳定：

输入：

- `/scan`
- `/imu/data`
- `/tf`
- `/tf_static`

输出：

- `/odom`
- `/tf`
- `/cartographer/status` 或等价 SLAM health topic
- 可选 `/map`、`/submap_list`、`/trajectory_node_list`

SLAM 不允许消费 Gazebo truth 或 FCU fused local position 作为定位输入。

### 3.6 Companion

companion 模拟真实无人机上的计算盒子，负责：

- 从 MAVLink 读取 FCU IMU、姿态、模式、arm 状态、EKF 状态、本地位置。
- 发布 `/imu/data`、`/imu/status`、`/navlab/mavlink/status`。
- 把 `/external_nav/odom` 转成 MAVLink `ODOMETRY`。
- 下发 `GUIDED`、arm、takeoff 和 hover setpoint。
- 把 FCU `LOCAL_POSITION_NED` 发布成 `/navlab/fcu/local_position_pose`。
- 把可视化 pose 发布成 `/sim/uav_pose`，供 Foxglove 回放。
- 发布 `/sim/markers`，显示无人机、墙体、雷达和关键轨迹。

companion 不能直接移动 Gazebo 机体。

## 4. 目标数据流

### 4.1 传感器链路

```text
Gazebo ray sensor
  -> /scan_ideal
  -> X2 virtual serial emulator
  -> ydlidar_ros2_driver_node
  -> /scan
  -> SLAM
```

要求：

- `/scan_ideal` 只作为 Gazebo 传感器源和调试对照。
- `/scan` 才是 SLAM 使用的雷达输入。
- `/scan` 的 `frame_id` 应是 `laser_frame`。
- 激光角度约定必须和真实安装方向一致：0 度指向机头前方，正角度向左，负角度向右。
- 如果驱动输出和 Gazebo ray sensor 方向相反，应在传感器仿真层修正，不应让 mission 或 SLAM 通过硬编码反向理解。

### 4.2 IMU 链路

```text
SITL MAVLink HIGHRES_IMU / RAW_IMU
  -> companion MAVLink decoder
  -> /imu/data
  -> SLAM
```

要求：

- `/imu/data` 是 SLAM 的唯一 IMU 输入。
- `/imu/status` 必须包含输入频率、最近消息时间、来源、是否 healthy。
- 实机迁移时，IMU 来源从 SITL MAVLink 切换到真实 FCU 或 MAVROS，但 topic 契约保持一致。

### 4.3 SLAM 到飞控链路

```text
/scan + /imu/data
  -> SLAM backend
  -> /odom
  -> external_nav_bridge
  -> /external_nav/odom
  -> mavlink_external_nav_sender
  -> MAVLink ODOMETRY
  -> SITL EKF
  -> LOCAL_POSITION_NED
```

要求：

- `/odom` 必须来自真实 SLAM 后端。
- `/external_nav/odom` 是飞控 ExternalNav 的 ROS 出口。
- Gazebo truth 可以记录为 `/gazebo/truth/odom`，但只用于对照和误差评估。
- acceptance 通过条件不能依赖 `/gazebo/truth/odom` 替代 SLAM odom。

### 4.4 悬停控制链路

```text
mission hover controller
  -> MAVLink mode/arm/takeoff/local target
  -> SITL
  -> Gazebo plugin
  -> Gazebo physical motion
  -> sensors
  -> SLAM
  -> ExternalNav
  -> SITL EKF
```

悬停不是让 Gazebo 机体位置不变，而是让 SITL 在 ExternalNav 支持下自己保持位置。验收时必须观察 FCU 本地位置、Gazebo 物理位姿、SLAM odom 三者是否一致。

## 5. 坐标系契约

推荐固定以下 frame：

- `navlab_world`：Foxglove replay 固定参考系，等价于 Gazebo 世界显示坐标。
- `map`：SLAM 全局地图系。
- `odom`：SLAM 连续里程计系。
- `base_link`：无人机机体系。
- `imu_link`：FCU IMU 坐标系。
- `laser_frame`：X2 雷达坐标系。

最低要求：

- Foxglove 固定参考系使用 `navlab_world` 或 `map`，但同一个 rosbag 内必须有完整 TF 链。
- `laser_frame -> base_link` 必须来自静态外参。
- `imu_link -> base_link` 必须来自静态外参。
- `map -> odom -> base_link` 必须由 SLAM 或桥接节点负责。
- `/sim/markers` 中墙体不能挂在移动机体系下，必须挂在固定世界系下。
- 无人机 marker 才能挂在 `base_link` 或由 `/sim/uav_pose` 更新。

如果 Foxglove 报缺少 `map`、`odom`、`base_link`、`imu_link`、`laser_frame` 到 `navlab_world` 的转换，说明 TF 链没有闭合，不能靠手动切换显示选项掩盖。

## 6. 世界设计

当前世界应从简单墙体改为横向 8 字形走廊：

```text
左环形通道  <->  中腰通道  <->  右环形通道
```

几何要求：

- 外围墙形成两个并排闭合环。
- 中间有两个对称不可穿越障碍，形成类似 LED 数码管“8”的可通行路径。
- 原点附近是起飞区，留出起飞和初始悬停空间。
- 通道宽度先按无人机最大水平外形宽度加少量余量设计。
- 初始建议通道净宽为无人机 footprint + `0.15 m` 到 `0.25 m`，后续根据 Gazebo 碰撞和雷达盲区调整。
- 墙体、内障碍、地面都应有稳定 collision，不只做 visual。

当前阶段只要求在该世界中起飞和悬停稳定，不要求完成左右环绕探索。之所以先改世界，是为了提前暴露雷达方向、TF、SLAM 地图和障碍物可视化问题。

## 7. SLAM 健康标准

SLAM health 至少需要输出这些信息：

- 输入 `/scan` 是否存在。
- `/scan` 频率和最近消息 age。
- 输入 `/imu/data` 是否存在。
- IMU 频率和最近消息 age。
- `/odom` 是否持续发布。
- `/odom` frame 是否正确。
- 最近一次 pose 是否跳变。
- 是否依赖 Gazebo truth。
- 后端状态，例如 Cartographer trajectory 是否 active。

acceptance 中应把“没有真实 SLAM”作为 blocker，而不是降级成 synthetic 通过。

## 8. 悬停验收标准

建议 acceptance 时长为 60 到 90 秒，其中 90 秒只是默认最大运行窗口，不代表必须飞满 90 秒。runner 可以在关键条件满足并完成最短稳定窗口后提前结束。

通过条件：

- Gazebo、SITL、gazebo-sensor、SLAM、companion 都启动成功。
- `/scan` 连续发布，方向和障碍物显示正确。
- `/imu/status.state` 为 healthy。
- SLAM 输出 `/odom`，且不是 Gazebo truth 替代。
- `/external_nav/status.state` 为 healthy。
- `/mavlink_external_nav/status.state` 为 sending。
- SITL 收到 ExternalNav 后进入 `GUIDED`。
- SITL arm 成功。
- SITL takeoff 成功。
- SITL 输出 `LOCAL_POSITION_NED`。
- 悬停稳定窗口内水平漂移低于阈值。
- 高度保持在阈值内。
- companion 的 direct set pose 计数为 0。
- rosbag required topics 全部存在。

建议阈值：

- SLAM odom age 小于 `0.5 s`。
- ExternalNav odom age 小于 `0.5 s`。
- FCU local position age 小于 `0.5 s`。
- 悬停稳定窗口至少 `20 s`。
- 水平漂移初始阈值 `0.5 m`，收敛后调到 `0.25 m`。
- 高度误差初始阈值 `0.3 m`，收敛后调到 `0.15 m`。

## 9. Rosbag 和 Foxglove

每次 acceptance 必须输出 MCAP，并记录足够复盘的 topic。最低 required topics：

- `/clock`
- `/tf`
- `/tf_static`
- `/scan`
- `/scan_ideal`
- `/imu/data`
- `/imu/status`
- `/odom`
- `/external_nav/odom`
- `/external_nav/status`
- `/mavlink_external_nav/status`
- `/navlab/mavlink/status`
- `/navlab/fcu/local_position_pose`
- `/navlab/mission/status`
- `/sim/uav_pose`
- `/sim/markers`
- `/gazebo/truth/odom`
- `/gazebo/truth/status`

可选调试 topic：

- `/map`
- `/submap_list`
- `/trajectory_node_list`
- `/cartographer/status`
- `/scan_features`
- `/scan_nearest_point`
- `/rosout`

Foxglove 配置建议：

- 固定参考系：优先 `navlab_world`，如果 SLAM 只发布 `map`，则使用 `map`。
- 显示：打开 3D、Raw Messages、Plot、Topic Graph。
- 跟踪：可以跟踪 `base_link` 或 `/sim/uav_pose` 对应对象，但不要把固定参考系设成移动机体系。
- 墙体 marker 必须固定在世界系，不应跟随无人机一起动。
- 如果墙体跟着无人机动，优先检查 `/sim/markers` 的 `header.frame_id` 和 TF，而不是怀疑 Foxglove。

## 10. 实机迁移口径

仿真通过后，实机替换关系如下：

```text
Gazebo + SITL
  -> 真实 FCU + 真实机体

Gazebo /scan_ideal + X2 emulator
  -> 真实 X2 雷达驱动 /scan

SITL MAVLink IMU
  -> 真实 FCU MAVLink IMU 或 MAVROS IMU

Gazebo truth
  -> 不存在，只保留仿真诊断意义
```

保持不变的接口：

- `/scan`
- `/imu/data`
- `/odom`
- `/external_nav/odom`
- `/external_nav/status`
- `/mavlink_external_nav/status`
- `/navlab/mavlink/status`
- `/navlab/mission/status`
- rosbag profile

这就是当前阶段必须先把真实 SLAM 反馈和悬停打牢的原因：只有接口不依赖 Gazebo truth，后续才能自然迁移到真实机器。
