# 仿真设计

这个目录是当前“先把仿真跑通”的主入口。

当前设计里先忽略旧的 `ros2_ws/` 目录。那个目录是早期草稿，不作为这次避障仿真的 active design。

## 目标

先做一个最小可用的无人机避障仿真，再考虑真实 YDLidar 雷达和更复杂的飞控闭环。

目标场景：

- 无人机从世界坐标原点开始。
- 无人机先悬停。
- 障碍物固定在无人机正前方 `+X` 方向 5 m 处。
- 无人机随后向前移动。
- 系统通过 ROS2 `/scan` 话题观察障碍物。
- `/scan` 的消息类型是 `sensor_msgs/msg/LaserScan`。

这里最重要的接口契约是 `/scan`。下游避障逻辑不应该关心 `/scan` 是 Gazebo，还是未来真实 `x3` 雷达驱动发出来的。

真实 `/scan` 契约定义见：

- `docs/sim/scan_contract.md`

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
- 原点处的无人机起点标记
- 沿 `+X` 方向的前进路径标记
- 中心在 `x=5`, `y=0` 的固定方块障碍物

### 传感器模型

在模拟无人机模型或临时 sensor rig 上加一个 2D ray lidar。

初始参数不要“差不多”，而是直接对齐 `docs/sim/scan_contract.md`：

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

docker/models/
  uav_lidar_rig/
    model.sdf
    model.config

lab_env/sim/
  gazebo_scan_bridge.md 或启动 helper
```

如果 Gazebo 镜像里已经有合适的无人机模型，就优先在无人机模型上加 lidar。否则先做 `uav_lidar_rig`，把它作为一个小模型放在原点。

### 当前运行命令

如果你不想在宿主机安装 ROS2，可以直接用仓库里的 `remote-sitl-lab/ros-jazzy-base:latest`：

```bash
just sim-p1-up
just sim-p1-consumer
```

这条路径会启动：

- `gazebo`
- `scan-bridge`
- `sim-runtime`

### 验收标准

- `gz sim` 能加载 `uav_obstacle_5m.sdf`。
- Gazebo 世界里障碍物位于 `x=5`, `y=0`。
- ROS2 能看到 `/scan`。
- `ros2 topic hz /scan` 频率稳定。
- sensor 在原点时，`/scan` 前方扇区距离接近 5 m。
- sensor 向前移动后，前方障碍物距离会变小。

## 第一版控制闭环范围

第一版先把无人机当成平面运动体：

```text
state: hover
  等待 N 秒
  切到 forward

state: forward
  沿 +X 前进
  监控 front_min
  如果 front_min < avoid_distance，则 stop 或 turn
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

仿真不应该依赖 `x3` 内部实现。两者共享的是同一份 ROS2 `/scan` 契约：

```text
/scan sensor_msgs/msg/LaserScan
frame_id: laser_frame
```

契约细节统一收敛到：

- `docs/sim/scan_contract.md`

当当前 Gazebo 链路跑通后，真实硬件接入应该只是替换 `/scan` 来源：

```text
Gazebo /scan    -> 下游避障逻辑
real x3 /scan   -> 下游避障逻辑
```

## 推荐执行顺序

1. 启动 `gazebo` + `scan-bridge` + `sim-runtime`。
2. 用现有 front-sector consumer 直接消费 Gazebo `/scan`。
3. 确认原点前方障碍物对应 `front_min ~= 5.0`。
4. `/scan` 稳定后，再加 hover-then-forward 的简单运动。

## 设计规则

所有模块都应该卡在 `/scan` 这个接口后面。

如果后续从 Gazebo 切到真实 `x3` 时，需要改下游避障逻辑，说明接口边界设计错了。
