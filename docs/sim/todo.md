# 仿真 TODO

这个 TODO 只覆盖当前两阶段仿真路线：

- P1：Gazebo lidar 产生真实仿真 `/scan`
- P2：把 hover-then-forward 和后续真实传感器切换准备好

旧 `ros2_ws/` 不纳入当前任务。

## P1：Gazebo Lidar 仿真输入

目标：让 Gazebo 世界和模拟 2D lidar 产生符合 `/scan` 契约的 ROS2 输入。

前提：

- Gazebo `/scan` 必须与 `docs/sim/scan_contract.md` 一致

### P1.1 完善 5m 障碍物世界

内容：

- 确认 `docker/worlds/uav_obstacle_5m.sdf` 是当前默认仿真世界。
- 保留原点起点标记。
- 保留 `+X` 方向路径标记。
- 保留 `x=5`, `y=0` 的固定障碍物。

验收：

- Gazebo 能加载世界。
- 障碍物位置明确可见。
- 世界文件不依赖旧 `ros2_ws/`。

### P1.2 新建 Gazebo lidar rig

内容：

- 新建 `docker/models/uav_lidar_rig/`。
- 添加一个最小模型，初始位姿在原点。
- 模型包含 2D ray lidar。
- lidar frame 为 `laser_frame`。
- lidar topic 设计为 `/scan` 或可桥接到 `/scan`。
- lidar 参数对齐真实 `x3` 基线。

验收：

- rig 能在 Gazebo 中加载。
- lidar 面向 `+X`。
- 原点时能看到 5 m 前方障碍。

### P1.3 增加 Gazebo 到 ROS2 scan bridge

内容：

- 在 `lab_env` compose 编排中增加 Gazebo scan bridge。
- 将 Gazebo lidar 输出桥接为 ROS2 `/scan`。
- 消息类型为 `sensor_msgs/msg/LaserScan`。
- ROS2 侧 QoS、frame、range 和角度语义与真实基线一致。

验收：

- 启动仿真后 ROS2 能看到 `/scan`。
- `ros2 topic hz /scan` 稳定。
- front-sector consumer 可以不改代码直接消费 Gazebo `/scan`。

### P1.4 用同一 consumer 验证 Gazebo 输出

内容：

- 使用 front-sector consumer。
- 对 Gazebo `/scan` 计算 `front_min`。
- 对比障碍物真实位置。

验收：

- sensor 在原点时，`front_min` 接近 5 m。
- sensor 向前移动后，`front_min` 变小。
- 切换 `/scan` 来源时，下游 consumer 不需要改逻辑。

## P2：简单运动闭环与真实传感器切换准备

目标：在 `/scan` 稳定后，增加最小 hover-then-forward 行为，并为后续真实 `x3` 接入留下干净边界。

### P2.1 实现 hover-then-forward 状态机

内容：

- 初始状态为 `hover`。
- 悬停 N 秒后进入 `forward`。
- `forward` 状态沿 `+X` 方向前进。
- 当前阶段可以先控制 rig 或简化 UAV 模型，不要求完整飞行动力学。

验收：

- 状态能从 `hover` 切到 `forward`。
- 前进过程中 `/scan` 的 `front_min` 会逐渐变小。
- 行为可重复运行。

### P2.2 增加避障状态切换

内容：

- 使用 `front_min` 做简单规则判断。
- `front_min < avoid_distance` 时进入 `avoid_required`。
- `front_min < stop_distance` 时进入 `stop`。

初始参数：

- `avoid_distance = 1.0 m`
- `stop_distance = 0.5 m`
- `forward_speed = 0.2 m/s`

验收：

- Gazebo `/scan` 下能触发对应状态。
- 状态输出可观测。

### P2.3 明确 `/scan` source 切换机制

内容：

- 定义 `SCAN_SOURCE=gazebo|real` 或等价配置。
- gazebo 使用 Gazebo bridge。
- real 预留给未来 `x3/ydlidar_ros2_driver`。
- 两种 source 都必须满足 `docs/sim/scan_contract.md`。

验收：

- gazebo 和 real 两种 source 可以用配置切换。
- 下游只依赖 `/scan`。
- 切换 source 不需要修改避障逻辑代码。

### P2.4 准备真实 `x3` 接入边界

内容：

- 不把 `x3` 代码复制进仿真模块。
- 只记录真实 `x3` 输出应满足的 `/scan` 契约。
- 对齐 `frame_id = laser_frame`、频率、range 参数。
- 如果真实参数文件切换，仿真基线也要同步切换。

验收：

- 文档明确 gazebo、real 两种 `/scan` 来源。
- 真实接入时只需要启动 `x3` 驱动并关闭 Gazebo scan source。
- 仿真代码不依赖 `x3` 内部实现。

## 当前完成定义

P1 完成后，应能用 Gazebo 世界中的 5 m 障碍物生成真实仿真 `/scan`。

P2 完成后，应能跑一个最小 hover-then-forward 行为，并在接近障碍物时触发 stop 或 avoid 状态。
