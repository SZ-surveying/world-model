# 阶段 1 设计：ArduPilot SITL ExternalNav 闭环

## 1. 目标

阶段 1 的目标不是一步到位完成“SLAM + 飞控 + 航点飞行”，而是把 ArduPilot SITL 接入拆成可验证的工程层次。

本阶段最终要证明：

- `ArduPilot SITL` 能在无 GPS 配置下稳定启动
- 飞控能持续消费来自本仓库的 `ExternalNav` 输入
- `external_nav_bridge` 的坐标系、时间戳、频率和状态输出可观测
- 用 fake odom / 仿真位姿验证飞控接收后，再切换到 `Cartographer 2D` 的 `/odom`
- 在 `x / y / yaw` 平面维度上完成最小 `hold/hover`，并为后续低速平移做准备

一句话定义：

**阶段 1 是“先验证飞控接收 ExternalNav，再接 SLAM odom，最后做最小位置保持”的 SITL 闭环阶段。**

## 2. 为什么要这样拆

如果阶段 1 直接写成：

```text
LiDAR + IMU -> Cartographer -> ExternalNav -> ArduPilot -> 起飞/悬停/航点飞行
```

问题会很明显：

- 起不来时不知道是 SITL 参数问题、DDS/MAVLink 问题、SLAM 问题还是坐标系问题
- SLAM `/odom` 本身还没稳定前，无法判断飞控侧是否真的能消费 ExternalNav
- 直接做航点飞行会把定位、控制、任务规划和安全边界混在一起
- 后续要接真实环境时，缺少分层验收证据

所以阶段 1 必须先用可控输入隔离飞控接入问题：

```text
fake odom / sim pose
  -> external_nav_bridge
  -> ArduPilot SITL
  -> ExternalNav 接收验证
```

飞控接收链路通过后，再替换上游：

```text
/scan + /imu/data
  -> cartographer_indoor
  -> /odom
  -> external_nav_bridge
  -> ArduPilot SITL
```

## 3. 范围

### 3.1 本阶段包含

- `ArduPilot SITL` 启动和版本固定
- `ExternalNav` 接收链路验证
- `external_nav_bridge` 的最小可用实现和状态观测
- fake `/odom` 或仿真 `/sim/uav_pose` 到 `/odom` 的适配入口
- `imu_bridge` 输出 `/imu/data`
- `cartographer_indoor` 输出 `/odom`
- 最小 `hold/hover` 验证
- 小范围 `local position / velocity setpoint` 的预留或后半段验证
- rosbag、日志和故障排查 topic 收口

### 3.2 本阶段不包含

- 不引入世界模型决策
- 不做复杂室内场景 `obj` 导入
- 不追求完整 3D 飞行能力
- 不把高度估计作为第一验收门槛
- 不做复杂航点任务或自主探索
- 不把真实试飞纳入阶段 1 验收

高度在本阶段只保留接口，主验收集中在：

- `x`
- `y`
- `yaw`
- ExternalNav 输入连续性
- 飞控接收状态

## 4. 总体链路

阶段 1 分两条链路推进。

### 4.1 接收验证链路

这条链路用于先证明飞控侧是通的：

```text
fake_external_nav 或 sim pose adapter
  -> /odom
  -> external_nav_bridge
  -> /external_nav/odom
  -> /external_nav/status
  -> /ap/tf 或 MAVLink ODOMETRY
  -> ArduPilot SITL
```

验收重点：

- `/odom` 输入可控、连续、可重复
- `/external_nav/status` 能明确显示健康状态
- ArduPilot 侧能看到 ExternalNav 输入
- 不依赖 Cartographer，不依赖真实 SLAM

### 4.2 SLAM 替换链路

飞控接收验证通过后，再替换成真实定位链路：

```text
Gazebo / real x3 /scan
  + ArduPilot SITL / FCU IMU
  -> imu_bridge
  -> /imu/data
  -> cartographer_indoor
  -> /odom
  -> external_nav_bridge
  -> /external_nav/odom + /external_nav/status + /ap/tf
  -> ArduPilot SITL
```

验收重点：

- `/imu/data` 频率稳定
- Cartographer 输出 `/odom` 连续
- `/odom -> /external_nav/odom` 坐标转换正确
- `x / y / yaw` 没有明显跳变或发散
- 飞控侧 ExternalNav 不超时、不频繁失效

## 5. 模块设计

### 5.1 `sitl_bringup`

职责：

- 固定 ArduPilot SITL 启动方式
- 固定 DDS/MAVLink 接入方式
- 固定参数文件和环境变量
- 提供最短启动命令

输出：

- SITL 进程
- ArduPilot 调试入口
- 飞控状态 topic 或 MAVLink 状态观测入口

不负责：

- 生成 SLAM odom
- 做 ExternalNav 坐标转换
- 做高层任务规划

### 5.2 `fake_external_nav`

职责：

- 生成确定性的 `/odom`
- 或把阶段 0 的 `/sim/uav_pose` 适配成 `/odom`
- 用于隔离验证 ArduPilot ExternalNav 接收链路

建议能力：

- 静止模式：固定 `x=0, y=0, yaw=0`
- 慢速直线模式：低速改变 `x`
- 小范围 yaw 模式：只改变 yaw
- 可配置发布频率

输出：

- `/odom`
- 可选 `/fake_external_nav/status`

验收：

- `/odom` 时间戳连续
- 频率稳定
- pose 与 twist 字段一致
- frame 命名稳定

### 5.3 `imu_bridge`

职责：

- 订阅 SITL / FCU IMU 来源
- 统一发布 `/imu/data`
- 输出 `/imu/status`

输入候选：

- ArduPilot ROS2 DDS: `/ap/imu/experimental/data`
- MAVROS: `/mavros/imu/data_raw`

输出：

- `/imu/data`
- `/imu/status`

验收：

- `/imu/data` 频率稳定
- frame 明确
- 时间戳策略明确
- 切换 IMU 来源不影响 `cartographer_indoor`

### 5.4 `cartographer_indoor`

职责：

- 消费 `/scan + /imu/data`
- 运行 Cartographer 2D 或等价 SLAM
- 输出 `/odom`
- 输出定位状态

输入：

- `/scan`
- `/imu/data`

输出：

- `/odom`
- `/cartographer/status`

验收：

- 静止时 `/odom` 不明显漂移
- 低速运动时 `x / y / yaw` 连续
- 短时间内不出现明显跳变
- 输出能直接被 `external_nav_bridge` 消费

### 5.5 `external_nav_bridge`

职责：

- 消费 `/odom`
- 做坐标系转换和时间戳治理
- 输出仓库内部 ExternalNav 状态
- 输出 ArduPilot 可消费接口

输入：

- `/odom`
- 可选 `/height/estimate`
- 可选 `/imu/data` 作为诊断输入

输出：

- `/external_nav/odom`
- `/external_nav/status`
- `/ap/tf`
- 后续可扩展 `MAVLink ODOMETRY`

验收：

- 输出频率满足 ArduPilot ExternalNav 要求
- 坐标系转换明确
- 状态 topic 能区分健康、超时、低质量、frame 错误
- reset 行为有明确字段或日志记录

## 6. Topic 和 frame 契约

### 6.1 核心 topic

| Topic | 类型 | 来源 | 用途 |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo 或 real x3 | SLAM 原始激光输入 |
| `/scan_features` | `ydlidar_interfaces/msg/ScanFeatures` | scan feature publisher | 行为和调试观察 |
| `/imu/data` | `sensor_msgs/msg/Imu` | `imu_bridge` | SLAM IMU 输入 |
| `/imu/status` | 状态消息或 JSON 字符串 | `imu_bridge` | IMU 健康状态 |
| `/odom` | `nav_msgs/msg/Odometry` | fake odom 或 `cartographer_indoor` | ExternalNav 输入 |
| `/cartographer/status` | 状态消息或 JSON 字符串 | `cartographer_indoor` | SLAM 健康状态 |
| `/external_nav/odom` | `nav_msgs/msg/Odometry` | `external_nav_bridge` | 仓库内部标准 ExternalNav 输出 |
| `/external_nav/status` | 状态消息或 JSON 字符串 | `external_nav_bridge` | ExternalNav 健康状态 |
| `/ap/tf` | `tf2_msgs/msg/TFMessage` | `external_nav_bridge` | ArduPilot DDS 接入口 |
| `/height/estimate` | `std_msgs/msg/String` JSON | height estimator | 高度估计预留输入，阶段 1 默认不要求 |

### 6.2 frame 建议

| Frame | 含义 |
| --- | --- |
| `map` | SLAM 全局/局部地图坐标 |
| `odom` | 连续局部里程计坐标 |
| `base_link` | UAV 主体坐标 |
| `laser_frame` | LiDAR 坐标 |
| `imu_link` | IMU 坐标 |
| `ap_local` | 输出给 ArduPilot 的本地坐标语义 |

阶段 1 必须明确：

- ROS 内部默认按 ENU / FLU 语义处理
- ArduPilot 侧需要 NED / FRD 或其 DDS 接口约定
- 坐标转换只允许集中在 `external_nav_bridge`

## 7. 分阶段验收

### 7.1 P1.0：SITL 基线启动

目标：

- `ArduPilot SITL` 可重复启动
- DDS 或 MAVLink 入口可观测
- 参数和版本固定

验收命令或证据：

- SITL 启动日志
- ArduPilot 参数截图或日志
- 飞控状态 topic / MAVProxy 状态输出

完成标准：

- 连续启动两次结果一致
- 没有依赖手工临时修环境变量

### 7.2 P1.1：fake ExternalNav 接收

目标：

- 不依赖 SLAM，先证明飞控能消费本仓库输出的 ExternalNav

链路：

```text
fake_external_nav -> /odom -> external_nav_bridge -> /ap/tf -> ArduPilot SITL
```

完成标准：

- `/odom` 稳定发布
- `/external_nav/status` 为健康
- ArduPilot 侧能看到 ExternalNav 输入
- 连续运行 2 到 5 分钟不超时

### 7.3 P1.2：接入 IMU

目标：

- 从 SITL / FCU 获取 IMU
- 输出统一 `/imu/data`

完成标准：

- `/imu/data` 频率稳定
- `/imu/status` 可观测
- frame 和时间戳策略明确

### 7.4 P1.3：接入 Cartographer odom

目标：

- 用 `/scan + /imu/data` 生成真实 `/odom`
- 替换 fake `/odom`

完成标准：

- `/odom` 连续
- `/cartographer/status` 健康
- `/external_nav/status` 健康
- `x / y / yaw` 没有明显跳变

### 7.5 P1.4：最小 hold/hover

目标：

- ArduPilot 基于 ExternalNav 做最小位置保持

完成标准：

- 进入预期飞控模式
- 不发送横向运动指令时，`x / y / yaw` 不明显发散
- ExternalNav 不超时
- 日志中没有持续 EKF 失效或定位拒收

### 7.6 P1.real-feedback：真实反馈 hold/hover

目标：

- 用 Gazebo `/scan`、real/SITL IMU 和真实 `cartographer_ros` 后端替换 synthetic `/scan`、placeholder IMU 和 synthetic TF
- 在真实 `/odom` 进入 `external_nav_bridge` 后复跑最小 `hold/hover`

完成标准：

- `cartographer_ros` 可用，并由 Gazebo `/scan` 和 real/SITL IMU 驱动输出 `odom -> base_link` TF
- `cartographer_indoor` 从真实 Cartographer TF 输出 `/odom`
- `/external_nav/status` 健康，ArduPilot 不持续报 ExternalNav 超时
- `hold/hover` 仍能进入预期飞控模式，且 `x / y / yaw` 不明显发散

验收入口：

```bash
just stage1-real-feedback-acceptance 60
```

### 7.7 P2.1：低速平移

目标：

- 在 hold/hover 稳定后，验证小范围高层 setpoint 控制

完成标准：

- 能执行小范围前后/左右速度或位置 setpoint
- 停止指令后能回到稳定 hold
- 控制链路仍经过安全边界，不绕过飞控

## 8. rosbag 和日志

阶段 1 的 rosbag 至少应记录：

- `/scan`
- `/scan_features`
- `/imu/data`
- `/imu/status`
- `/odom`
- `/cartographer/status`
- `/external_nav/odom`
- `/external_nav/status`
- `/ap/tf`
- ArduPilot 状态 topic
- ArduPilot 控制命令或 setpoint topic

日志至少要能回答：

- SITL 是否启动成功
- ExternalNav 是否被飞控接收
- 输入是否超时
- 坐标系转换是否启用
- 当前使用的是 fake odom 还是 SLAM odom
- `hold/hover` 是否进入预期状态

建议 artifact 路径：

```text
artifacts/ros/<SESSION_ID>/stage1_sitl_external_nav/<YYYYMMDD_HHMMSS>/
```

目录内至少包含：

- `sim.log` 或 `sitl.log`
- `rosbag_0.mcap`
- `run_config.toml`
- `README.md` 或 `summary.md`

## 9. 风险和处理策略

| 风险 | 现象 | 处理策略 |
| --- | --- | --- |
| SITL 参数不对 | ExternalNav 不被 EKF 使用 | 先固定参数文件和 runbook，再做自动化 |
| 坐标系错 | 方向反、yaw 反、位置发散 | fake odom 阶段先做单轴测试 |
| 时间戳不连续 | ExternalNav 超时或被拒收 | bridge 输出 status 暴露 age 和频率 |
| SLAM 不稳定 | `/odom` 跳变或漂移 | 先用 fake odom 验证飞控，再替换 SLAM |
| 高度来源不稳定 | 起飞/悬停难验证 | 阶段 1 先聚焦平面，z 只保留接口 |
| DDS/MAVLink 路径混乱 | 接口重复、状态不可观测 | 阶段 1 优先固定一条路径，另一条作为后续扩展 |

## 10. 当前下一步

按当前工程状态，阶段 1 的接口闭环已经通过，下一步应收敛真实反馈链路：

1. 接入真实 `cartographer_ros` 后端，让 Gazebo `/scan` 经 scan matching 产生 `odom -> base_link` TF
2. 接入 real/SITL IMU 到 `imu_bridge`，不要继续使用 placeholder IMU 作为阶段 1 完成证据
3. 运行 `just stage1-real-feedback-acceptance 60`，验证 `Gazebo /scan + real/SITL IMU -> cartographer_ros -> /odom -> ExternalNav -> hold/hover`
4. real-feedback hold/hover 通过后，复跑 P1.3/P1.4/P2.1，确认真实反馈链路下 ExternalNav、hold 和低速 setpoint 都仍健康
5. 更新阶段 1 artifact summary，把 synthetic adapter 验收和真实反馈验收分开记录

这条顺序的关键是：**已经证明飞控接收和 synthetic 控制链路，接下来必须证明真实 scan matching 和 real/SITL IMU 反馈链路。**

## 11. 与其他阶段的关系

- 阶段 0 提供可移动仿真、LiDAR、`/scan_features` 和最小 stop guard
- 阶段 1 接入 ArduPilot SITL 和 ExternalNav，不引入世界模型
- 阶段 2 在阶段 1 成立后导入真实室内 `obj` 场景
- 阶段 3 把已经验证过的链路迁移到真实环境试飞
- 阶段 4 才让世界模型参与移动决策

## 12. 决策记录

Decision: 用阶段 tag 固定 SITL 和 mavlink-router 镜像。
Basis: 本地镜像没有可用 `RepoDigest`，但阶段 1 需要避免继续依赖 `latest`。
Reason: `compose/docker-compose.yaml` 默认使用 `remote-sitl-lab/ardupilot-sitl:stage1-f10500ae45aa` 和 `remote-sitl-lab/mavlink-router:stage1-4ee567d97525`；后续如果 registry 暴露 digest，再替换成 `repo@sha256:...`。

Decision: 增加 real-feedback acceptance gate，synthetic adapter 通过不等同于真实 SLAM/IMU 闭环完成。
Basis: P1.2/P1.3/P1.4 已验证的是 TF-backed adapter、placeholder IMU 或 synthetic TF。
Reason: 阶段 1 真正完成前，必须额外验证 `Gazebo /scan + real/SITL IMU -> cartographer_ros -> /odom -> ExternalNav -> hold/hover`。

## 13. 参考文档

- `docs/scenarios/indoor/task_breakdown_progress_tracking.md`
- `docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`
