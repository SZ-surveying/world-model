# 室内无 GPS 激光 SLAM 飞行项目：任务拆解与进度跟踪

## 1. 这一节在文档里要表达什么

这一节不应该写成“想到什么做什么”的流水账，而应该回答四个问题：

1. 总目标是什么
2. 当前先做到哪一步
3. 每个阶段的交付物和验收标准是什么
4. 现在卡在哪、下一步做什么

结合当前仓库状态，这个项目建议明确拆成两条主线：

- 主线 A：先在**没有世界模型**的前提下，完成室内无 GPS 飞行闭环
- 主线 B：在主线 A 跑通后，再由**世界模型**接管更高层的移动决策

也就是先证明“能飞、能定位、能按航点移动、能安全停住”，再证明“飞得更聪明”。

当前新增一个阶段 1.5，用于连接阶段 0 和阶段 1：

- 阶段 0 验证 Gazebo 世界、`/scan`、`/scan_features` 和最小 stop guard
- 阶段 1 验证 ArduPilot SITL 在无 GPS 配置下消费 ExternalNav
- 阶段 1.5 把 SITL 当飞控、把独立 companion 镜像当机载计算盒子、把 Gazebo 当真实世界，完成可回放的悬停、前进和绕障闭环

## 2. 建议写进飞书的阶段定义

### 阶段 0：最小仿真运动闭环

目标：

- 在 Gazebo 中先完成可观测、可控制的最小运动闭环
- 不引入真实飞控，不引入世界模型
- 先验证 `/scan -> /scan_features -> /planner/cmd_vel -> 模型运动` 这条链路

关键任务：

- 启动 Gazebo 仿真环境
- 让 UAV 模型可移动
- 让激光雷达随 UAV 一起运动
- 打通 `/scan` 和 `/scan_features`
- 打通 `manual` 和 `auto` 两种仿真模式
- 验证 stop guard，可以在障碍物前安全停住

交付物：

- `sim up --mode manual|auto` 可运行
- 仿真中可见 UAV 运动
- `/sim/log` 可用于状态观测
- 最小航点自动执行能力

验收标准：

- 手动模式下可通过 `/planner/cmd_vel` 驱动模型前进和停止
- 自动模式下可按简单航点文件直线运动
- 前方障碍距离收敛到约 `0.5 m` 时系统会稳定停住

当前状态：

- 已基本完成
- 依据 `docs/sim/todo.md`，当前已完成最小仿真链路、`manual|auto` 双模式、`/scan_features` 发布和 stop guard
- 未完成项主要是 `manual` 模式的完整人工验收，以及后续真实传感器来源切换收口

### 阶段 1：接入 ArduPilot SITL，验证 ExternalNav 飞控闭环

目标：

- 在阶段 0 的可移动仿真基础上引入 `ArduPilot SITL`
- 把“脚本直接推动模型”的链路，逐步升级为“飞控消费 ExternalNav 后执行位置保持和低速移动”
- 先验证无 GPS 条件下的飞控接收、状态估计和最小控制闭环，不把真实 SLAM、复杂动力学和世界模型一次性塞进同一个验收点

关键任务：

- `SITL` 基线启动：固定 `ArduPilot SITL` 版本、启动参数、DDS/MAVLink 接入方式和观测命令
- `ExternalNav` 接收验证：先用仿真位姿或确定性的假 `/odom` 输入 `external_nav_bridge`，确认 ArduPilot 能持续消费外部导航数据
- 飞控参数收口：配置无 GPS / ExternalNav 相关 EKF 参数，确认输入频率、新鲜度、坐标系和质量字段满足要求
- `SLAM` 替换输入：在飞控接收链路已经验证后，再把假 `/odom` 替换为 `Cartographer 2D` 或等价 SLAM 输出
- 最小控制验证：先做 `hold/hover`，再做小范围 `local position / velocity setpoint` 平移，不在本阶段引入世界模型决策

交付物：

- `sitl_bringup` 或等价启动流程：能稳定启动 ArduPilot SITL 并暴露调试入口
- `external_nav_bridge` 最小链路：输入 `/odom`，输出 `/external_nav/odom`、`/external_nav/status` 和 ArduPilot 可消费接口
- `fake_external_nav` 或仿真位姿适配入口：用于在不依赖 SLAM 的情况下先验证飞控侧接收
- `imu_bridge`：把 SITL / FCU IMU 统一成 `/imu/data`
- `cartographer_indoor`：消费 `/scan + /imu/data` 并输出 `/odom`
- SITL 联调 runbook、rosbag topic 列表、状态观测和故障排查方法

验收标准：

- `SITL` 可重复启动，且能在无 GPS / ExternalNav 配置下进入预期状态
- 使用假 `/odom` 或仿真位姿时，ArduPilot 能稳定接收 ExternalNav，频率、时间戳和坐标系检查通过
- 将输入切换为 `Cartographer 2D` 的 `/odom` 后，`/external_nav/status` 保持健康，`x/y/yaw` 不出现明显跳变或发散
- 最小 `hold/hover` 可维持稳定；进一步的小范围平移只作为阶段 1 后半段验收，不作为第一步门槛
- rosbag 能记录 `/scan`、`/imu/data`、`/odom`、`/external_nav/odom`、`/external_nav/status`、ArduPilot 状态和控制命令，便于复盘

当前状态：

- 接口闭环已完成：`SITL + fake_external_nav + external_nav_bridge + MAVLink ODOMETRY` 已通过，`hold/hover` 和小范围 setpoint 已有 synthetic 输入验收记录
- 真实反馈闭环待验收：当前 `cartographer_indoor` 主要证明 Cartographer-compatible TF adapter，尚未完成 `Gazebo /scan + real cartographer_ros scan matching + real/SITL IMU` 的闭环验收
- 下一步应优先跑通 `Gazebo /scan + real/SITL IMU -> cartographer_ros -> /odom -> external_nav_bridge -> ArduPilot hold/hover`，复跑 P1.3/P1.4/P2.1 后再把阶段 1 标为真实 SLAM/IMU 反馈闭环完成

### 阶段 1.5：独立 companion 镜像驱动的 SITL + Gazebo 闭环

目标：

- 将 `ArduPilot SITL` 明确作为无人机飞控
- 将独立 companion 镜像明确作为机载计算盒子
- 将 Gazebo 明确作为无 GPS 室内世界、障碍物和传感器环境
- 让 companion 从 SITL/MAVLink 拆分飞控状态和 IMU，给 SLAM 和控制器使用
- 让 companion 通过 MAVLink `ODOMETRY` 和 local setpoint 反馈/控制 SITL
- 让 SITL 的本地位置同步回 Gazebo，驱动可视化 UAV 和 lidar 在世界中运动
- 每次验收绑定 rosbag，方便后续在 Foxglove 中回放

关键任务：

- 拆出独立 companion 镜像，包含 ROS2、SLAM、bridge、pymavlink、rosbag2 和 mission controller
- 禁用 Stage 0 中 planner 直接 `set_pose` 的 `cmd_vel_executor` 主控制路径
- 新增 `mavlink_gazebo_pose_mirror`，把 SITL `LOCAL_POSITION_NED` 同步到 Gazebo UAV 位姿
- 新增或固定 MAVLink IMU bridge，将 FCU IMU 输出为 `/imu/data`
- 使用 Gazebo `/scan + FCU IMU -> SLAM -> /odom -> ExternalNav -> MAVLink ODOMETRY -> SITL`
- 新增 MAVLink obstacle mission controller，完成 hover、forward、avoid、final hold
- 固定 Stage 1.5 rosbag topic profile，确保 Foxglove 可回放

交付物：

- `docs/scenarios/indoor/stage1_5_companion_sitl_gazebo_design.md`
- `docs/scenarios/indoor/stage1_5_companion_sitl_gazebo_todo.md`
- `profiles/stage1_5-rosbag-topics.txt`
- Stage 1.5 compose profile 或等价启动入口
- `mavlink_gazebo_pose_mirror`
- `mavlink_obstacle_mission_controller`
- Stage 1.5 acceptance artifact：`summary.json`、MCAP rosbag、SITL/router/companion/Gazebo 日志、Foxglove 回放说明

验收标准：

- SITL 不依赖 GPS
- companion 镜像独立启动并能连接 SITL MAVLink
- companion 能拿到 FCU IMU，并输出 `/imu/data` 与 `/imu/status`
- Gazebo `/scan` 和 `/scan_features` 可被 companion 消费
- SLAM 不使用 `LOCAL_POSITION_NED` 作为输入，只使用 `/scan + /imu/data`
- ExternalNav 持续 healthy，SITL 不在启动宽限期后持续报 `VisOdom: not healthy`
- Gazebo UAV 位姿跟随 SITL `LOCAL_POSITION_NED`
- UAV 能先悬停，再前进，接近障碍物后绕行，最后稳定 hold
- rosbag profile required topics 全部存在，并能在 Foxglove 中回放完整任务链路

当前状态：

- 设计文稿和 TODO 已建立
- rosbag topic profile 已建立
- 代码实现未开始
- 依赖阶段 1 real-feedback gate 继续收口，但 Stage 1.5 可以先用同一 ExternalNav 主线推进 companion / pose mirror / mission controller 边界

### 阶段 2：导入真实场景模型，提升仿真真实性

目标：

- 从简单障碍物世界升级到更接近真实室内环境的仿真场景
- 验证 SLAM、ExternalNav 和基础规划在复杂环境中的表现

关键任务：

- 导入 `obj` 或等价室内场景模型
- 调整碰撞体、坐标系和尺度
- 确保激光雷达扫描结果与真实环境结构一致
- 在复杂环境中验证起飞、移动、避障和停靠

交付物：

- 可重复加载的室内场景资产
- 场景启动脚本
- 复杂环境下的 rosbag 和评估记录

验收标准：

- 场景模型可稳定加载
- SLAM 在室内场景下可持续输出可用定位
- SITL 在复杂环境下仍能完成基本航点移动

当前状态：

- 未开始，依赖阶段 1 先把飞控闭环跑通

### 阶段 3：真实环境试飞

目标：

- 将仿真中验证过的室内无 GPS 闭环迁移到真实 UAV
- 验证真实 `LiDAR + FCU IMU + SLAM + ExternalNav + ArduPilot` 的端到端效果

关键任务：

- 接入真实雷达
- 接入真实 FCU IMU
- 完成外参、时间同步和坐标系校准
- 完成地面静态测试、低速滑移测试、受控试飞
- 建立安全接管和故障保护流程

交付物：

- 真实环境 bring-up 流程
- 真实环境联调记录
- 安全检查清单和试飞报告

验收标准：

- 真实环境下可稳定输出 `/odom` 和 external nav
- 低速受控飞行稳定
- 丢失定位、传感器异常、通信超时等场景有明确降级策略

当前状态：

- 未开始
- 依赖阶段 1 和阶段 2 收敛后再推进

### 阶段 4：引入世界模型指导移动决策

目标：

- 在基础闭环已经成立后，引入世界模型参与路径选择、风险评估和运动决策
- 从“按固定航点移动”升级为“根据环境理解来决定怎么移动”

关键任务：

- 定义世界模型输入输出契约
- 让世界模型消费局部环境状态、历史观测或轨迹候选
- 让 planner 或上层决策模块消费世界模型输出
- 建立模型失效时回退到规则策略的机制

交付物：

- 世界模型接口定义
- 在线推理服务或离线评估链路
- 与 planner 的联调结果

验收标准：

- 世界模型输出可被稳定消费
- 相比固定航点或规则方法，决策质量有可观测提升
- 世界模型失效时系统仍能回退到保守安全策略

当前状态：

- 未开始
- 当前明确不应把世界模型前置到阶段 0 或阶段 1 之前

## 3. 建议写成这样的总任务拆解

可以在飞书里直接写成下面这段：

> 本项目分两步推进。第一步是在没有世界模型的情况下，先完成室内无 GPS 飞行的最小闭环，具体包括：仿真中模型可移动、激光雷达与 SLAM 可提供定位、ArduPilot SITL 可基于 ExternalNav 完成基本飞行、复杂室内环境仿真可复现、最终完成真实环境试飞。第二步是在基础闭环稳定后，引入世界模型，对局部环境和运动风险进行建模，用于指导路径选择和运动决策，从而替代当前以固定航点和规则策略为主的移动方式。

## 4. 建议写成这样的进度跟踪表

| 阶段 | 名称 | 目标 | 当前状态 | 里程碑 | 验收标准 | 下一步 |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 最小仿真运动闭环 | 先在 Gazebo 中让模型运动起来，并完成激光感知和基础安全停障 | 进行中，主体已打通 | `manual|auto` 可启动，模型可移动，stop guard 生效 | 能手动控制，能按简单航点移动，障碍前稳定停住 | 完成 manual 模式人工验收，补齐配置收口 |
| 1 | ArduPilot SITL ExternalNav 闭环 | 先验证 SITL 能消费 ExternalNav，再接入 SLAM odom 和最小位置控制 | 接口闭环完成，真实 SLAM/IMU 反馈待验收 | `sitl_bringup`、`fake_external_nav`、`external_nav_bridge`、`imu_bridge`、`cartographer_indoor` 接口分层跑通；real-feedback acceptance 待跑 | 假 odom 和 Cartographer-compatible odom 能被飞控稳定消费，最小 hold/hover 不发散；真实 Gazebo scan + real/SITL IMU + cartographer_ros 仍需通过 | 跑 `stage1-real-feedback-acceptance`，再复跑 P1.3/P1.4/P2.1 |
| 1.5 | Companion + SITL + Gazebo 闭环 | SITL 当飞控，companion 当计算盒子，Gazebo 当无 GPS 世界，完成悬停、前进和绕障 | 实现中；独立 companion 镜像、pose mirror、FCU IMU bridge、SLAM/ExternalNav 启动入口、MAVLink mission controller 和 rosbag profile 已落地；`cartographer_ros` 仍阻塞真实反馈验收 | 独立 companion、pose mirror、MAVLink mission controller、rosbag/Foxglove profile | 无 GPS 条件下完成 hover -> forward -> avoid -> final_hold，且 rosbag 可完整回放 | 把 `cartographer_ros` 装进 companion 镜像，复跑 `stage1-5-companion-doctor`，再跑 `stage1-5-companion-gazebo-acceptance` |
| 2 | 真实场景模型仿真 | 导入 `obj` 室内场景，提高仿真真实性 | 未开始 | 场景资产可加载，复杂环境可复现实验 | SLAM 和基本飞行在复杂环境下仍可用 | 选定场景资产并建立启动脚本 |
| 3 | 真实环境试飞 | 将整条无 GPS 闭环迁移到真实 UAV | 未开始 | 完成地面联调、低风险试飞 | 真实环境下可稳定定位和受控飞行 | 制定试飞前检查清单和安全流程 |
| 4 | 世界模型接管高层决策 | 让移动策略从固定航点升级为基于环境理解的决策 | 未开始 | 定义模型接口并完成 planner 对接 | 决策质量提升，模型失效可安全回退 | 定义输入输出契约和评估方法 |

## 5. 建议单独加一个“本周/当前进展”小节

如果飞书模板里允许写短进展，建议写成：

- 已完成：Gazebo 最小仿真闭环已基本打通，`manual|auto` 双模式可启动，激光扫描、`/scan_features`、`/planner/cmd_vel` 和 stop guard 已联通。
- 进行中：补做 `manual` 模式人工遥控验收，收口仿真 source 配置和 rosbag 细节。
- 下一步：跑 `stage1-real-feedback-acceptance`，用 Gazebo `/scan`、真实 `cartographer_ros` 后端和 real/SITL IMU 复核 `/odom -> ExternalNav -> hold/hover`。
- 风险点：SITL 接收链路和 synthetic hold/hover 已通，但真实 SLAM scan matching、真实 IMU 输入、复杂场景资产和实机安全试飞都还未完成。

## 6. 进度跟踪建议

这部分不要只写“完成了多少”，建议按下面 5 个维度持续更新：

- 阶段状态：未开始 / 进行中 / 已完成 / 阻塞
- 当前里程碑：这个阶段最近一个必须达成的可验收目标
- 实际证据：命令、topic、rosbag、日志、视频、试飞记录
- 风险与阻塞：技术风险、环境依赖、安全依赖
- 下一步动作：必须是 1 到 3 条可以立刻执行的具体动作

## 7. 当前推荐表述

如果你只想先填一版简洁的“任务拆解与进度跟踪”，建议直接用下面这段：

> 当前项目按“先闭环、后智能”的路线推进。第一阶段先在无世界模型前提下完成室内无 GPS 飞行闭环，路线依次为：Gazebo 中模型可移动并具备激光感知能力，接入 ArduPilot SITL 并先用 fake odom / 仿真位姿验证 ExternalNav 接收链路，再拆出独立 companion 镜像作为机载计算盒子，把 SITL 当飞控、Gazebo 当真实世界，完成可 rosbag/Foxglove 回放的悬停、前进和绕障闭环；随后导入真实室内场景模型提升仿真真实性，最后推进真实环境试飞。在此基础上，第二阶段再引入世界模型，让系统从固定航点执行升级为根据环境理解指导移动决策。当前进度上，最小仿真运动闭环、SITL fake ExternalNav 接收、synthetic hold/hover、低速 setpoint、Stage 1.5 companion 镜像边界、pose mirror、FCU IMU bridge、MAVLink mission controller 和 rosbag profile 已落地；下一步重点是把 `cartographer_ros` 装进 companion 镜像，用 Gazebo `/scan`、FCU MAVLink IMU 和真实 `cartographer_ros` 后端跑通 Stage 1.5 real-feedback obstacle acceptance。

## 8. 对应仓库依据

- 仿真最小闭环现状：`docs/sim/todo.md`
- 仿真主线说明：`docs/sim/README.md`
- 阶段 1 SITL ExternalNav 设计：`docs/scenarios/indoor/stage1_sitl_external_nav_design.md`
- 阶段 1 SITL ExternalNav TODO：`docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`
- 阶段 1.5 companion + SITL + Gazebo 设计：`docs/scenarios/indoor/stage1_5_companion_sitl_gazebo_design.md`
- 阶段 1.5 companion + SITL + Gazebo TODO：`docs/scenarios/indoor/stage1_5_companion_sitl_gazebo_todo.md`
