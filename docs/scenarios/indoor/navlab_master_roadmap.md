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
| P1 | 官方 maze + NavLab X2 雷达 | 继续使用官方 `iris_maze` 和 Iris 模型，只把雷达链路换成 X2 机制 | `ardupilot_gz` + X2 driver | `todos/P1_official_maze_x2_todo.md` |
| P2 | 下视 rangefinder 和 IMU 机制验收 | down rangefinder、IMU 和 FCU 接收状态来自正确机制 | `claudedrone` + Gazebo sensor | `todos/P2_rangefinder_imu_todo.md` |
| P3 | SLAM backend 质量验收 | Cartographer 输出真实 `/odom`，并可对照诊断 | official Cartographer + PX4 SLAM | `todos/P3_slam_backend_quality_todo.md` |
| P4 | FCU 状态机和唯一控制器 | 只有一个 owner 向 FCU 发运动 setpoint | `Altair-Silent` + `claudedrone` | `todos/P4_fcu_state_machine_todo.md` |
| P5 | Frame contract 自动验收 | NED/ENU、FRD/FLU、TF 链和 scan 方向可自动检查 | PX4 odom converter | 待建 |
| P6 | 真实 SLAM hover gate | SLAM ExternalNav 悬停稳定通过 | official + NavLab acceptance | 待建 |
| P7 | 官方 maze 小范围运动 gate | 在官方 maze 中 forward/back/yaw scan/stop drift 都通过 | VehicleController | 待建 |
| P8 | 官方 maze 探索任务 | 在官方 maze 中完成可回放探索，不碰撞 | Nav2 / exploration | 待建 |
| P9 | NavLab world/model 迁移 | 可选地把官方 maze/Iris 替换为 NavLab world/model | `ardupilot_gz` + NavLab world | 待建 |

## 4. Phase 详细定义

### P0：官方基线验收

目标：

- 先跑通或至少明确实现官方 ArduPilot ROS2/Gazebo/Cartographer 的最小基线。
- 确认 `/ap/v1/time` sample、`/ap/v1/prearm_check` service、DDS domain、SITL、Gazebo、Cartographer、EKF 参数这些基础机制。
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

### P1：官方 maze + NavLab X2 雷达

目标：

- 继续使用 P0 已跑通的官方 `ardupilot_gz_bringup iris_maze.launch.py`。
- 继续使用官方 Iris / lidar 模型和官方 maze world，不在本 phase 替换 NavLab 8 字形 world 或自定义机体。
- 只把雷达链路替换或旁路接入为 NavLab X2 机制：Gazebo ray / `/scan_ideal` -> X2 virtual serial -> vendor driver -> `/scan`。
- 保证 Cartographer 消费的 `/scan` 来自 X2 vendor-driver 链路，而不是 synthetic fallback。

非目标：

- 不接 NavLab 8 字形 world。
- 不接 NavLab 自定义无人机模型。
- 不改变官方 Gazebo/SITL 启动结构。
- 不允许 Python direct set pose 或 companion 控制 Gazebo。

输出：

- 官方 maze + X2 雷达接入设计文档。
- P1 TODO。
- X2 scan acceptance，包含 `/scan_ideal`、virtual serial status、vendor `/scan` 和 Cartographer 输入检查。

### P2：下视 rangefinder 和 IMU 机制验收

目标：

- 下视 rangefinder 作为飞控外设进入 ArduPilot。
- IMU 链路按官方/真机口径输出给 SLAM。
- 继续使用官方 maze/Iris baseline，不在 P2 替换 NavLab world/model。
- summary 明确 P2 不代表 hover 完成，也不代表 SLAM `/odom` 质量完成。

参考：

- `claudedrone` 的传感器健康和 rangefinder 思路。
- `Autonomous-Indoor-Drone-Navigation-ROS2` 的 Gazebo ray sensor。

完成标准：

- `/rangefinder/down/range` 和 FCU 接收状态可验收。
- `/imu` 输入来源、频率和 frame 可验收。

设计文档：

- `docs/scenarios/indoor/navlab_p2_rangefinder_imu_design.md`

TODO：

- `docs/scenarios/indoor/todos/P2_rangefinder_imu_todo.md`

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

设计文档：

- `docs/scenarios/indoor/navlab_p3_slam_backend_quality_design.md`

TODO：

- `docs/scenarios/indoor/todos/P3_slam_backend_quality_todo.md`

### P4：FCU 状态机和唯一控制器

目标：

- 任务层只表达意图。
- 只有一个 controller 向 FCU 发 setpoint。
- FCU 未 ready 前不发运动 setpoint。
- 用官方示例兼容 route 完成 GUIDED、arm、takeoff、local position ready 的状态机验收：
  MAVLink bootstrap 负责起飞状态转换，DDS `/ap/v1/*` 负责状态观测和 `/ap/v1/cmd_vel` 运动输出。

参考：

- `Altair-Silent` 的 `StateWatcher`、`VehicleController`。
- `claudedrone` 的起飞状态机。

完成标准：

- summary 能证明 GUIDED、arm、takeoff、local position ready。
- setpoint owner 唯一。
- hover、yaw、local position target 都走同一个输出通道。
- summary 明确 `hover_claim=not_evaluated`，不把 P4 当作 P6 hover completion。
- summary 明确 `official_control_claim=false`、`mavlink_bootstrap_claim=true`，不把当前 route 误标为纯 DDS service control。

设计文档：

- `docs/scenarios/indoor/navlab_p4_fcu_state_machine_design.md`

TODO：

- `docs/scenarios/indoor/todos/P4_fcu_state_machine_todo.md`

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

### P7：官方 maze 小范围运动 gate

目标：

- 在官方 maze/Iris 场景中验证最小水平运动。
- 保持 P1-P6 已验证的官方 bringup、X2、rangefinder/IMU、SLAM、ExternalNav、FCU 控制机制不变。
- 每个动作都通过 FCU setpoint 执行，不直接控制 Gazebo。

动作：

- forward。
- back。
- yaw scan。
- stop drift。

非目标：

- 不替换 NavLab 8 字形 world。
- 不替换 NavLab 自定义机体模型。
- 不把 Gazebo truth 作为规划或控制输入。

完成标准：

- 每个动作有 FCU setpoint、SLAM `/odom`、FCU local position、Gazebo truth 诊断对照。
- 停止后漂移在阈值内。
- 不碰撞。
- `set_pose_count == 0`。

### P8：官方 maze 探索任务

目标：

- 在官方 maze/Iris 场景里做探索任务，因为官方 maze 比当前 NavLab 8 字形场景更复杂，更适合先验证导航策略。
- 探索策略只能使用 SLAM map、scan、TF、FCU state 和任务状态，不直接读取 Gazebo truth。

任务流示例：

```text
起飞
  -> hover settle
  -> 建图/定位稳定
  -> 选择局部目标
  -> 前进
  -> 必要时 yaw scan
  -> 避开障碍
  -> 覆盖 maze 中可达区域
  -> final hover 或返航点
```

完成标准：

- rosbag/Foxglove 可复现完整探索任务。
- 不碰撞。
- 不直接读答案式 Gazebo truth 做规划输入。
- scan_left/scan_right 只在必要时由规划策略触发。
- summary 记录覆盖区域、碰撞状态、stuck 状态和最终任务状态。

### P9：NavLab world/model 迁移

目标：

- 在官方 maze 场景已经完成 P1-P8 后，可选地把 world/model 替换为 NavLab 8 字形 world 和 NavLab 机体模型。
- 这个阶段是迁移和定制，不是当前真实闭环主线的前置条件。
- 保证替换 world/model 后，无人机仍由 ArduPilot SITL 和 Gazebo plugin 控制。

完成标准：

- NavLab world/model 可在官方 bringup 结构下启动。
- 墙体、无人机、雷达和 TF 在 Foxglove 中固定参考系正确。
- 替换 world/model 不破坏 P6 hover gate、P7 小范围运动 gate 和 P8 探索 gate。

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
