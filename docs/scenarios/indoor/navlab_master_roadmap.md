# NavLab 室内无 GPS 真实闭环总 Roadmap

## 1. 目标

这份 roadmap 用来统一 NavLab 后续实现顺序。它把“官方 ArduPilot ROS2/Gazebo/Cartographer 路线”作为真实闭环完成标准，把四个参考仓库作为工程模块参考，并把每个 phase 连接到后续独立 TODO 文档。

当前核心目标不是先完成 8 字形探索，而是先证明：

```text
Gazebo 物理世界
  -> ArduPilot SITL / FCU 控制无人机运动
  -> Gazebo sensor 模拟真实外设
  -> SLAM backend 消费真实 /scan、/imu、TF
  -> SLAM 输出 /odom
  -> ExternalNav 进入 ArduPilot EKF
  -> FCU 输出 local position
  -> hover / motion / exploration 只通过 FCU setpoint 执行
```

## 2. 基线和参考来源

### 2.1 完成标准来源

官方路线定义“什么叫真实闭环完成”：

- ArduPilot ROS2 文档：`https://ardupilot.org/dev/docs/ros2.html`
- ArduPilot ROS2 with SITL
- ArduPilot ROS2 with SITL in Gazebo
- ArduPilot Cartographer SLAM with ROS2 in SITL
- `ardupilot_ros`
- `ardupilot_gz`

因此 NavLab 的最终验收不能只证明自定义 MAVLink bridge 能跑，而要逐步证明 `/ap/*` DDS 接口、官方 Gazebo bringup 结构、官方 Cartographer 口径和 ArduPilot EKF ExternalNav 机制成立。

### 2.2 工程参考来源

四个仓库提供局部工程参考：

| 仓库 | 用途 | 在 NavLab 中的定位 |
|---|---|---|
| `claudedrone` | FCU 状态确认、MAVROS 起飞 FSM、传感器健康、rangefinder 思路 | 借鉴飞控状态机和传感器职责 |
| `Altair-Silent` | StateWatcher、VehicleController、Nav2 action、唯一 setpoint owner | 借鉴控制权管理和任务分层 |
| `PX4-ROS2-SLAM-Control` | NED/ENU、FRD/FLU frame 转换、SLAM 输入契约、Offboard heartbeat | 借鉴坐标转换和 SLAM backend contract |
| `Autonomous-Indoor-Drone-Navigation-ROS2` | Gazebo ray sensor、rf2o laser odom、slam_toolbox localization | 借鉴传感器/诊断 backend，不借鉴自写飞控 |

详细分析见：

- `docs/scenarios/indoor/navlab_reference_projects_analysis.md`
- `docs/scenarios/indoor/navlab_ardupilot_ros2_official_alignment.md`

## 3. Roadmap 总览

| Phase | 名称 | 完成标准 | 主要参考 | TODO |
|---|---|---|---|---|
| P0 | 官方基线验收 | 官方 `/ap/*`、SITL、Gazebo、Cartographer 基线可观测 | ArduPilot official | `todos/P0_official_baseline_todo.md` |
| P1 | NavLab world/model 接入官方链路 | 8 字形 world 和 IQ quad 模型在官方结构下运行 | `ardupilot_gz` + NavLab world | 待建 |
| P2 | 传感器机制验收 | X2 `/scan`、down rangefinder、IMU 都来自正确机制 | `claudedrone` + Gazebo ray | 待建 |
| P3 | SLAM backend 质量验收 | Cartographer 输出真实 `/odom`，并可对照诊断 | official Cartographer + PX4 SLAM | 待建 |
| P4 | FCU 状态机和唯一控制器 | 只有一个 owner 向 FCU 发运动 setpoint | `Altair-Silent` + `claudedrone` | 待建 |
| P5 | Frame contract 自动验收 | NED/ENU、FRD/FLU、TF 链和 scan 方向可自动检查 | PX4 odom converter | 待建 |
| P6 | 真实 SLAM hover gate | SLAM ExternalNav 悬停稳定通过 | official + NavLab acceptance | 待建 |
| P7 | 小范围运动 gate | forward/back/yaw scan/stop drift 都通过 | VehicleController | 待建 |
| P8 | 8 字形探索任务 | 右环、中腰、左环探索可回放可验收 | Nav2 / exploration | 待建 |

## 4. Phase 详细定义

### P0：官方基线验收

目标：

- 先跑通或至少明确实现官方 ArduPilot ROS2/Gazebo/Cartographer 的最小基线。
- 确认 `/ap` 节点、DDS domain、SITL、Gazebo、Cartographer、EKF 参数这些基础机制。
- 建立以后所有 NavLab 自定义能力的对照标准。

非目标：

- 不接 NavLab 8 字形 world。
- 不接 X2 virtual serial。
- 不做探索任务。
- 不把自定义 MAVLink ExternalNav bridge 当作完成标准。

设计文档：

- `docs/scenarios/indoor/navlab_p0_official_baseline_design.md`

TODO：

- `docs/scenarios/indoor/todos/P0_official_baseline_todo.md`

### P1：NavLab world/model 接入官方链路

目标：

- 在 P0 官方启动结构基础上换入 NavLab 的 8 字形 world 和 IQ quad 模型。
- 保证无人机仍由 ArduPilot SITL 和 Gazebo plugin 控制。
- Gazebo truth 只作为诊断。

非目标：

- 不允许 Python direct set pose。
- 不允许 companion 控制 Gazebo。

输出：

- NavLab world/model 接入设计文档。
- P1 TODO。
- Gazebo model/world acceptance。

### P2：传感器机制验收

目标：

- X2 链路从 Gazebo ray sensor 到厂商 driver 输出 `/scan`。
- 下视 rangefinder 作为飞控外设进入 ArduPilot。
- IMU 链路按官方/真机口径输出给 SLAM。

参考：

- `claudedrone` 的传感器健康和 rangefinder 思路。
- `Autonomous-Indoor-Drone-Navigation-ROS2` 的 Gazebo ray sensor。

完成标准：

- `/scan` 不使用 synthetic fallback。
- `/rangefinder/down/range` 和 FCU 接收状态可验收。
- scan 方向和 TF 正确。

### P3：SLAM backend 质量验收

目标：

- Cartographer 是默认真实 backend。
- backend registry 允许替换 SLAM，但接口不变。
- `/odom` 质量可诊断，而不是只检查 topic 存在。

参考：

- 官方 Cartographer。
- `PX4-ROS2-SLAM-Control` 的 SLAM input contract。
- `Autonomous-Indoor-Drone-Navigation-ROS2` 的 `rf2o` 诊断链。

完成标准：

- `/scan + /imu + /odometry + TF` 驱动 SLAM。
- `/odom` 连续、低跳变、误差可对照。
- artifact 记录 backend、配置 hash、版本。

### P4：FCU 状态机和唯一控制器

目标：

- 任务层只表达意图。
- 只有一个 controller 向 FCU 发 setpoint。
- FCU 未 ready 前不发运动 setpoint。

参考：

- `Altair-Silent` 的 `StateWatcher`、`VehicleController`。
- `claudedrone` 的起飞状态机。

完成标准：

- summary 能证明 GUIDED、arm、takeoff、local position ready。
- setpoint owner 唯一。
- hover、yaw、local position target 都走同一个输出通道。

### P5：Frame contract 自动验收

目标：

- 把 frame 和坐标转换从“靠看 Foxglove 猜”变成自动验收。

要覆盖：

- MAVLink/ArduPilot NED 到 ROS ENU。
- body FRD 到 ROS FLU。
- `map -> odom -> base_link -> imu_link/laser_frame/rangefinder_down_frame`。
- scan 角度方向和 yaw 正方向。

完成标准：

- TF 连通、无循环、parent 唯一。
- `/scan` 可叠到固定墙体。
- FCU local position、SLAM `/odom`、Gazebo truth 诊断方向一致。

### P6：真实 SLAM hover gate

目标：

- 真正证明 SLAM ExternalNav 支撑悬停。

完成标准：

- FCU 定高稳定。
- SLAM `/odom` 是 ExternalNav 输入。
- EKF 接收 ExternalNav。
- hover drift 在阈值内。
- `set_pose_count == 0`。

### P7：小范围运动 gate

目标：

- 在 hover gate 后验证最小水平运动。

动作：

- forward。
- back。
- yaw scan。
- stop drift。

完成标准：

- 每个动作有 FCU setpoint、SLAM `/odom`、FCU local position、Gazebo truth 诊断对照。
- 停止后漂移在阈值内。

### P8：8 字形探索任务

目标：

- 在机制稳定后做任务探索。

任务流：

```text
起飞
  -> hover settle
  -> 进入右环
  -> 右环探索一圈
  -> 中腰切换
  -> 左环探索一圈
  -> 回到安全点或 final hover
```

完成标准：

- rosbag/Foxglove 可复现完整任务。
- 不碰撞。
- 不直接读答案式 Gazebo truth 做规划输入。
- scan_left/scan_right 只在必要时由规划策略触发。

## 5. 全局不可违反规则

- 不允许用 Gazebo truth 作为正式 ExternalNav 输入。
- 不允许 direct set pose 伪造飞行。
- 不允许多个节点同时向 FCU 发运动 setpoint。
- 不允许把 synthetic/fake odom gate 当作真实 SLAM 完成标准。
- 不允许跳过 P0/P6 直接把探索任务标为完成。
- 不允许把 PX4 `/fmu/*` 直接混入 ArduPilot 主链路。

## 6. 文档推进规则

每个 phase 都必须有：

- 设计文档：解释机制、接口、非目标、验收语义。
- TODO 文档：列出可打勾任务、验收、验证记录。
- acceptance summary 字段定义。
- rosbag required topics。
- Foxglove 回放口径。

TODO 的 checkbox 只有在代码、配置、测试或验收记录都能支撑时才打勾。不能因为“看起来能跑”就打勾。
