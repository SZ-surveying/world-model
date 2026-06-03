# 仿真设计

这个目录是当前“先把仿真跑通”的主入口。

当前设计里先忽略旧的 `ros2_ws/` 目录。那个目录是早期草稿，不作为这次仿真感知 / 最小控制链路的 active design。

## 当前模块结构

当前 `lab_env/sim/` 目录按职责分成四层：

```text
lab_env/sim/
  cli.py
  rosbag.py
  waypoints.py

  nodes/
    cmd_vel_executor.py
    front_sector_consumer.py
    scan_features_publisher.py
    waypoint_follower.py
    world_marker_publisher.py

  perception/
    contract.py
    front_sector.py
    scan_features.py

  world/
    world_markers.py
```

约定是：

- `nodes/` 只放真正会起 ROS2 node 的入口
- `perception/` 只放纯扫描/状态逻辑，方便单测和复用
- `waypoints.py` 只放自动模式的最小航点文件契约与解析
- `world/` 只放 SDF / world 解析逻辑

`lab_env.sim` 顶层也做了轻量导出，所以外部如果只是想复用感知 helper，可以直接从 `lab_env.sim` 导入常用符号，不必每次都写完整长路径。

## 目标

先做一个最小可用的无人机仿真感知 / 安全停止链路，再考虑真实 YDLidar 雷达、world model 和更复杂的飞控闭环。

目标场景：

- 无人机从世界坐标原点开始。
- 障碍物固定在无人机正前方 `+X` 方向 5 m 处。
- 无人机随后向前移动。
- 系统通过 ROS2 `/scan` 话题观察障碍物。
- `/scan` 的消息类型是 `sensor_msgs/msg/LaserScan`。

这里最重要的接口契约是 `/scan`、`/scan_features` 和 `/sim/log`。下游 world model / 行为逻辑不应该关心它们是 Gazebo，还是未来真实 `x3` 雷达驱动发出来的。

当前最小契约：

- `/scan`: `sensor_msgs/msg/LaserScan`，`frame_id=laser_frame`
- `/scan_features`: `ydlidar_interfaces/msg/ScanFeatures`
- `/sim/log`: JSON 字符串，至少包含 `source`、`event` 和可选 `mission_state`

## 不做什么

- 不接入旧 `ros2_ws/` 包。
- 不把 `x3/` 当仿真器使用。`x3` 是真实 YDLidar 驱动源码。
- 不从完整 SLAM、Cartographer、ExternalNav 或真实 ArduPilot 控制开始。
- 不一开始就建完整无人机动力学模型。

当前优先级是：先让 Gazebo 世界真实产出 `/scan`，并让现有 consumer 直接消费。

## 当前方案：Gazebo 世界 + 模拟 2D Lidar

### 目的

从硬编码 `/scan` 过渡到由仿真环境真实生成 `/scan`。

这个阶段回答的问题是：

```text
如果 Gazebo 世界中无人机前方 5 m 有障碍物，模拟雷达能不能发布与真实雷达契约一致的 /scan？
```

### 世界

当前世界文件：

```text
docker/worlds/uav_obstacle_5m.sdf
```

这个世界保持最小化：

- 地面
- 原点处的无人机机体标记（`0.5 m` 立方体）
- 沿 `+X` 方向的前进路径标记
- 前方面向航线的固定障碍物，尺寸 `4 m x 1 m x 2 m`，前缘从 `x=5` 开始

### 传感器模型

在模拟无人机模型或临时 sensor rig 上加一个 2D ray lidar。

初始参数不要“差不多”，而是直接对齐当前仿真主线：

- frame: `laser_frame`
- topic: `/scan`
- 水平视场角：优先 360 deg
- 更新频率：10 Hz
- 最小距离：`0.1 m`
- 最大距离：`12.0 m`
- 分辨率：能稳定看到 5 m 处 1 m 宽障碍物

如果无人机动力学模型还没准备好，先用静态或脚本移动的 sensor rig。这个阶段重点是验证 `/scan`，不是先追求完整飞行物理。

### Gazebo 到 ROS2 桥接

Gazebo 产生的传感器输出要桥接到 ROS2：

```text
/scan sensor_msgs/msg/LaserScan
```

桥接逻辑应该放在 `lab_env` 的 compose 编排里，和 Gazebo 服务放在一起。不要通过旧 `ros2_ws/` 接线。

### 建议位置

```text
docker/worlds/
  uav_obstacle_5m.sdf

lab_env/sim/
  gazebo_scan_bridge.md 或启动 helper
```

当前这条仿真主线已经收敛成：把 lidar 直接挂到可移动的 `uav_start_marker` 上，让原始 `/scan` 和可见 UAV 机体保持同一个 Gazebo 位姿。

### 当前运行命令

如果你不想在宿主机安装 ROS2，可以直接用仓库里的 `remote-sitl-lab/ros-jazzy-base:latest`：

```bash
just sim-p1-up
just sim-p1-consumer
```

`sim-p1-consumer` 现在默认优先消费 `/scan_features`；如果当前环境没有 `ydlidar_interfaces` 类型支持，或者还没有特征发布链路，才会回退到原始 `/scan`。

它现在不再只是打印 front sector 摘要，而是带一个最小前进/停车状态机：

- `front_min > stop_distance` 时发布 `forward`
- `front_min <= stop_distance` 或没有有效前向观测时发布 `stop`
- 控制输出 topic：`/planner/cmd_vel`

这条路径会启动：

- `gazebo`
- `scan-bridge`
- `sim-runtime`
- `foxglove`

默认情况下，`sim up` 会在 `sim-runtime` 里自动启动 world marker publisher，所以 Foxglove 3D 直接就能看到静态场景，不需要再单独手动起一条命令。

同时也会自动启动一个和 `x3` 对齐的 `/scan_features` 发布节点：

- 直接订阅会跟着 UAV 一起运动的 `/scan`
- 发布 `/scan_features`
- 发布 `/scan_nearest_point`

其中 `/scan_features` 的消息类型与 `x3` 保持一致：

```text
ydlidar_interfaces/msg/ScanFeatures
```

另外，`scan-bridge` 侧还会自动启动一个真正消费 `/planner/cmd_vel` 的执行器节点。它会：

- 把速度指令积分成 `/sim/uav_pose`
- 用 Gazebo `set_pose` 推动可见的 `uav_start_marker` 沿 `+X` 前进
- 让绑定在 `uav_start_marker` 上的 lidar 也一起运动，所以原始 `/scan` 会真的随 UAV 前进而变化
- 在执行器内部额外做一个最小前向净空钳位，默认值来自 `lab_env/config.toml` 的 `[sim].stop_distance = 0.5`

## `sim up` 的两种模式

当前 `sim up` 已经显式分成：

- `manual`：默认模式；起 Gazebo、Foxglove、`/scan_features` 发布链和 `cmd_vel` 执行器，但不自动推进 UAV
- `auto`：在 `manual` 的基础上，额外自动启动一个最小直线航点执行器，按航点文件从起点跑到终点

两种模式都会复用同一条安全边界：

```text
/scan -> /scan_features -> stop guard
```

也就是：

- 高层自动控制只看 `/scan_features`
- 底层 `cmd_vel_executor` 也会再用同一个 `stop_distance` 做一次几何钳位

两种模式现在还会共同发布一个可订阅的 JSON 日志 topic：

```text
/sim/log std_msgs/msg/String
```

这个 topic 适合在 Foxglove、`ros2 topic echo` 或下游脚本里直接订阅。常见事件包括：

- `cmd_vel_executor.executor_ready`
- `cmd_vel_executor.cmd_received`
- `cmd_vel_executor.executor_state`
- `waypoint_follower.mission_loaded`
- `waypoint_follower.heading_to_waypoint`
- `waypoint_follower.mission_complete`

其中 `waypoint_follower` 的 `mission_state` 现在收敛成稳定枚举：

- `ready`
- `running`
- `blocked_by_stop_guard`
- `complete`

### `manual` 模式

适合联调和人工操控：

```bash
uv run --project lab_env --no-sync --group host python -m lab_env.main sim up --mode manual
```

这时你可以从 Foxglove 或其他 ROS2 客户端往 `/planner/cmd_vel` 发 `geometry_msgs/msg/Twist`，手动把 UAV 在 Gazebo 场景里开起来。

推荐最小观察集合：

- `/sim/markers`
- `/sim/uav_pose`
- `/scan_features`
- `/sim/log`

当前 CLI 语义上，`manual` 更像“起一个可交互环境”：

- 命令返回前只负责把 sim 环境拉起
- 首次启动会先基于 `docker/Dockerfile.sim` 构建 sim 专用镜像；上层用 `uv` 安装 Python 依赖，底层运行环境仍是 `ros-jazzy-base`
- 会打印 Foxglove websocket 和控制 topic
- 后续由用户自己连接 Foxglove 或发布 `/planner/cmd_vel`
- 结束时再手动执行 `sim down`

推荐最短操作路径：

1. 启动 `sim up --mode manual`
2. 在 Foxglove 连接 `ws://<host>:8765`
3. 加一个 3D 面板订阅 `/sim/markers`
4. 加一个 Raw Messages / Topic Graph 面板看 `/sim/uav_pose`、`/scan_features`、`/sim/log`
5. 用 Publish 面板往 `/planner/cmd_vel` 发 `geometry_msgs/msg/Twist`

Foxglove 发布 `Twist` 时，最小 forward 示例可以直接填：

```json
{
  "linear": { "x": 0.2, "y": 0.0, "z": 0.0 },
  "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
}
```

如果你只是想快速验证，不想手填 Publish 面板，也可以直接用 CLI preset：

```bash
uv run --project lab_env --no-sync --group host python -m lab_env.main sim cmd-vel-preset forward
uv run --project lab_env --no-sync --group host python -m lab_env.main sim cmd-vel-preset stop
```

也可以显式指定持续时间或速度：

```bash
uv run --project lab_env --no-sync --group host python -m lab_env.main sim cmd-vel-preset forward \
  --linear-x 0.2 \
  --duration 2.0 \
  --rate 10.0
```

### `auto` 模式

适合“给一个最小航点文件，直接跑直线”：

```bash
uv run --project lab_env --no-sync --group host python -m lab_env.main sim up \
  --mode auto \
  --waypoint-file docs/sim/examples/straight_line_demo.yaml
```

当前 CLI 语义上，`auto` 更像“一次批作业”：

- 命令会先启动 sim 环境
- 如果 sim 专用镜像还没构建好，会先构建镜像
- 然后等待 mission 结束并持续打印关键状态
- mission 到达终点或触发 `blocked_by_stop_guard` 后，默认自动执行 `sim down`
- 运行日志会通过 `loguru` 输出到终端，并写到本次 artifact 目录里的 `sim.log`

注意：

- `--waypoint-file` 目前必须放在仓库目录内，因为 `sim-runtime` 容器只挂载了当前工作区
- 第一版只支持 `straight_line`，也就是沿当前机头方向的单轴 `+X` 直线前进
- `auto` 模式默认会为这次 mission 开一份 rosbag；`manual` 模式默认不录
- 到达终点后会发布 `mission_complete` 到 `/sim/log`，随后 `waypoint_follower` 自行退出
- 到达终点前，`mission_state` 会稳定经过 `ready -> running -> complete`
- 如果前方被 stop guard 卡住，`mission_state` 会变成 `blocked_by_stop_guard`，随后当前 mission runner 退出
- 如果前方 `front_min <= stop_distance`，自动模式也会停住，不会穿过去
- 默认 rosbag 输出目录：`artifacts/ros/<SESSION_ID>/auto_waypoint_follower/<timestamp>/`
- 默认 rosbag 数据文件名会是 `rosbag_0.mcap`

如果你想专门验收 `blocked_by_stop_guard`，可以直接跑这个样例：

```bash
uv run --project lab_env --no-sync --group host python -m lab_env.main sim up \
  --mode auto \
  --waypoint-file docs/sim/examples/blocked_by_stop_guard.yaml
```

这个 mission 会朝 `x=6.0` 前进，但因为前方障碍物前缘位于 `x=5`，预期现象是：

- `/sim/log` 出现 `event = "stop_distance_reached"`
- `/sim/log.mission_state = "blocked_by_stop_guard"`
- UAV 停在 `front_min ~= 0.5 m`

## `stop_distance` 配置

当前 `0.5 m` 不再写死在代码里，而是收敛到 `lab_env/config.toml` 的 `[sim]` 段：

```toml
[sim]
stop_distance = 0.5
```

这个配置会同时作用到：

- `front_sector_consumer` 的 `/scan_features` stop 判定
- `cmd_vel_executor` 的底层最小净空钳位
- `waypoint_follower` 自动模式的直线执行 stop 判定

## 自动模式航点文件格式

当前最小契约只支持一个受限的 `yaml` 子集。

当前推荐字段：

- `start`: 可选；不写时默认原点 `(0, 0, 0)`
- `goal`: 推荐；单段直线终点

兼容字段：

- `waypoints`: 非空数组；如果你以后想在同一直线上放多个检查点，也还可以继续写这个

可选字段：

- `version`: 默认 `1`
- `mode`: 默认 `straight_line`
- `frame_id`: 默认 `map`
- `forward_speed`: 默认 `0.2`
- `position_tolerance`: 默认 `0.05`

`start` / `goal` / `waypoints[*]` 当前字段：

- `x`: 必填
- `y`: 可选，默认 `0.0`
- `z`: 可选，默认 `0.0`

当前第一版额外约束：

- 只支持 `mode = straight_line`
- 只支持 `frame_id = map`
- `start`、`goal` 和所有 waypoint 的 `y/z` 必须保持不变
- `x` 必须从 `start` 开始单调不减，也就是只做沿 `+X` 的直线前进

一个最小 YAML 例子：

```yaml
version: 1
mode: straight_line
start:
  x: 0.0
goal:
  x: 2.0
```

YAML 示例见 `docs/sim/examples/straight_line_demo.yaml`。

专门用于触发 stop guard 的样例见：

- `docs/sim/examples/blocked_by_stop_guard.yaml`

如果你明确不想启动 marker publisher，可以：

```bash
just sim-p1-up --no-markers
```

自动发布的默认 topic：

```text
/sim/markers
/sim/log
/sim/uav_pose
/scan_features
/scan_nearest_point
/planner/cmd_vel
```

自动模式默认 rosbag 会把上面这组主线 topic 一起录下来，这样回放时不仅能看位姿和日志，也能直接看到场景 marker。

当前会自动发布：

- 原点 UAV 机体 marker（机尾蓝色、机头浅黄的 `0.6 m x 0.4 m x 0.2 m` 扁长方体分区）
- 前方障碍物 marker（柔和半透明 `1 m x 4 m x 2 m` 长方体，`4 m x 2 m` 面朝向 UAV，底面落在 `z=0`）

如果你这次运行需要留 rosbag，可以直接用现成入口：

```bash
just sim-p1-consumer-record
```

这里的含义是：

- `manual` 模式默认 `sim-runtime` 普通执行，不录制
- `auto` 模式默认会给 mission 留一份 rosbag
- 只有显式使用 `*-record` 命令时，才会额外对这条前台命令先起 `ros2 bag record`
- 主命令结束后，录制会自动结束

当前实测会稳定留下 `.mcap` 文件；目录级 `metadata.yaml` / `ros2 bag info` 兼容性还在 TODO 里继续收口。

`/sim/log` 里的 `mission_state` 当前收敛为稳定枚举：`ready`、`running`、`blocked_by_stop_guard`、`complete`。

如果你想直接走 `lab_env` 这个 CLI，而不是通过 `just`：

```bash
uv run --project lab_env --no-sync --group host python -m lab_env.main sim up --mode manual
uv run --project lab_env --no-sync --group host python -m lab_env.main sim consumer
```

同样也可以直接关闭自动 marker：

```bash
uv run --project lab_env --no-sync --group host python -m lab_env.main sim up --mode manual --no-markers
```

### 验收标准

- `gz sim` 能加载 `uav_obstacle_5m.sdf`。
- Gazebo 世界里障碍物位于 `y=0`，前缘位于 `x=5`，且宽面朝向 UAV。
- ROS2 能看到 `/scan`。
- ROS2 能看到 `/sim/uav_pose`。
- ROS2 能看到 `/sim/log`。
- ROS2 能看到 `/scan_features`。
- `ros2 topic hz /scan` 频率稳定。
- sensor 在原点时，`/scan` 前方扇区距离接近 5 m。
- sensor 在原点时，`/scan_features.front_min` 接近 5 m。
- 执行器推动可见 UAV 沿 `+X` 前进后，原始 `/scan` 和 `/scan_features` 都会同步收敛。
- 即使上游持续发 forward，执行器也会把 UAV 钳在 `front_min ~= 0.5 m` 处，不再继续前进。
- 自动模式到达终点后，`/sim/log` 会出现 `mission_complete`。
- 自动模式使用 `docs/sim/examples/blocked_by_stop_guard.yaml` 时，`/sim/log` 会出现 `mission_state = blocked_by_stop_guard`。

## 第一版控制闭环范围

第一版先把无人机当成平面运动体，只保留：

```text
manual:
  人手往 /planner/cmd_vel 发 Twist

auto:
  waypoint_follower 沿 +X 发布 forward
  到终点或触发 stop_distance 就停
```

初始阈值：

- `stop_distance = 0.5 m`
- `avoid_distance = 1.0 m`
- `forward_speed = 0.2 m/s`

这足够验证障碍物检测和状态决策。SLAM、ExternalNav、真实飞控控制都应该放到后面。

## 与 `x3` 的关系

`x3/` 仍然有价值，但它只代表真实硬件路径：

```text
x3/ydlidar_ros2_driver -> /scan
```

仿真不应该依赖 `x3` 驱动进程本身。两者共享的是同一份 ROS2 契约：

```text
/scan sensor_msgs/msg/LaserScan
frame_id: laser_frame
/scan_features ydlidar_interfaces/msg/ScanFeatures
```

当当前 Gazebo 链路跑通后，真实硬件接入应该只是替换 `/scan` 来源：

```text
Gazebo /scan    -> 下游避障逻辑
real x3 /scan   -> 下游避障逻辑
```

## 推荐执行顺序

1. 启动 `gazebo` + `scan-bridge` + `sim-runtime` + `foxglove`。
2. 如果是 `manual` 模式，就从 Foxglove、CLI preset 或命令行往 `/planner/cmd_vel` 发 `Twist`。
3. 如果是 `auto` 模式，就给 `sim up --mode auto --waypoint-file ...` 一个起点/终点直线航点文件。
4. 观察 `/sim/log`，确认 executor 和 mission 状态在更新。
5. 确认原点前方障碍物对应 `front_min ~= 5.0`，随后观察原始 `/scan` 与 `/scan_features.front_min` 一起逐步减小。
6. 确认 UAV 会在到达终点或 `front_min ~= stop_distance` 时稳定停住；如果到达终点，`/sim/log` 会出现 `mission_complete`。

## 设计规则

所有模块都应该卡在 `/scan` 这个接口后面。

如果后续从 Gazebo 切到真实 `x3` 时，需要改下游避障逻辑，说明接口边界设计错了。
