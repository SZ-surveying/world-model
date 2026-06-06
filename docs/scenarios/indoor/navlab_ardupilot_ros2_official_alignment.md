# NavLab 与 ArduPilot ROS2 官方路线对齐审计

日期：2026-06-05

这份文档只回答一个问题：当前 NavLab 室内无 GPS 仿真链路是否已经匹配
ArduPilot 官方 ROS2 / Gazebo / Cartographer 教程。结论是：还没有全匹配。
当前系统已经有 Gazebo、SITL、LiDAR、Cartographer、ExternalNav 和 rosbag
批处理验收能力，但它仍然保留了较多 NavLab 自定义桥接逻辑，不能说已经等价于官方
`ardupilot_ros` 路线。

## 官方参考

- ArduPilot ROS2 总入口：<https://ardupilot.org/dev/docs/ros2.html>
- ROS2 with SITL：<https://ardupilot.org/dev/docs/ros2-sitl.html>
- ROS2 with SITL in Gazebo：<https://ardupilot.org/dev/docs/ros2-gazebo.html>
- Cartographer SLAM with ROS2 in SITL：<https://ardupilot.org/dev/docs/ros2-cartographer-slam.html>
- 官方仓库：<https://github.com/ArduPilot/ardupilot_ros>
- 官方 Gazebo 集成仓库：<https://github.com/ArduPilot/ardupilot_gz>

## 官方 package 来源和启动入口

当前 P0 doctor 按下面这些官方来源检查依赖：

| package / 组件 | 来源 | P0 用途 |
|---|---|---|
| `ardupilot_sitl` | ArduPilot 源码 `Tools/ros2/ardupilot_sitl` | 提供 `sitl_dds_udp.launch.py`、`micro_ros_agent.launch.py`、SITL 默认参数 |
| `ardupilot_msgs` | ArduPilot 源码 `Tools/ros2/ardupilot_msgs` | `/ap/*` DDS 消息和服务类型 |
| `ardupilot_dds_tests` | ArduPilot 源码 `Tools/ros2/ardupilot_dds_tests` | 官方 DDS 接口 smoke/测试参考 |
| `micro_ros_agent` | `ardupilot_gz/ros2_gz.repos` 指向 `micro-ROS/micro-ROS-Agent` | SITL DDS 客户端和 ROS2 graph 之间的 agent |
| `ardupilot_gz_bringup` | `ArduPilot/ardupilot_gz` | 官方 Gazebo + SITL + DDS bringup |
| `ardupilot_gz_application` | `ArduPilot/ardupilot_gz` | Gazebo 应用层 package |
| `ardupilot_gazebo` | `ArduPilot/ardupilot_gazebo` | Gazebo ArduPilot plugin 和参数 |
| `ardupilot_gz_gazebo` | `ArduPilot/ardupilot_gz` | 官方 worlds / Gazebo 系统包 |
| `ardupilot_sitl_models` | `ardupilot_gz/ros2_gz.repos` 指向 SITL models | 官方模型资源 |
| `ardupilot_cartographer` | `ArduPilot/ardupilot_ros` | 官方 Cartographer launch / Lua baseline |

官方最小 SITL DDS 入口是：

```bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py
```

这个 launch 会 include：

- `micro_ros_agent.launch.py`
- `sitl.launch.py`
- `mavproxy.launch.py`

官方 DDS 参数文件 `dds_udp.parm` 至少包含：

```text
DDS_ENABLE 1
DDS_UDP_PORT 2019
```

官方 Gazebo lidar 示例入口是：

```bash
ros2 launch ardupilot_gz_bringup iris_maze.launch.py
```

其中 `robots/iris_lidar.launch.py` 默认使用 `model=json`，并把 `copter.parm`、
`gazebo-iris.parm`、`dds_udp.parm`、`dds_use_ns.parm` 组合成 SITL defaults。
`robots/robot.launch.py` 在 `use_dds_agent=true` 时会 include
`ardupilot_sitl/launch/sitl_dds_udp.launch.py`。

官方 Cartographer 入口是：

```bash
ros2 launch ardupilot_cartographer cartographer.launch.py
```

它启动 `cartographer_ros cartographer_node` 和
`cartographer_ros cartographer_occupancy_grid_node`，并执行 remap：

```text
/odom -> /odometry
```

Lua baseline 使用 `tracking_frame="imu_link"`、`published_frame="base_link"`、
`odom_frame="odom"`、`use_odometry=true`。

## 官方链路应是什么

官方教程的机制可以整理成下面这条链：

```text
ardupilot_gz_bringup
  -> 启动 Gazebo + ArduPilot SITL + ROS2/DDS 相关进程
  -> SITL 通过 micro-ROS-Agent / Micro-XRCE-DDS 暴露 ROS2 接口
  -> ROS2 graph 中出现 /ap 节点和 /ap/* topic
  -> Gazebo/ROS2 发布 LiDAR、IMU、odometry 等输入
  -> ardupilot_cartographer 启动 cartographer_ros
  -> Cartographer 使用 /scan、/imu、/odometry
  -> Cartographer 输出 TF / map / occupancy grid
  -> 外部里程计进入 ArduPilot EKF
  -> ArduPilot 在 GUIDED / takeoff / navigation 中使用 ExternalNav
```

其中官方 Cartographer 示例不是自己写一个 MAVLink ODOMETRY sender 作为首选路径，
而是围绕 ArduPilot ROS2/DDS 接口和官方 package bringup 工作。MAVLink 仍然可以作为
地面站、调参、日志和备用控制通道，但不应替代官方 ROS2 ExternalNav 接口成为当前
对齐目标。

## 对齐矩阵

| 项目 | 官方要求 | NavLab 当前状态 | 结论 |
|---|---|---|---|
| SITL 启动 | `ros2 launch ardupilot_sitl sitl_dds_udp.launch.py`，DDS enabled | 当前通过 Docker + `arducopter --model JSON` + mavlink-router 启动 | 不匹配 |
| Gazebo 启动 | `ros2 launch ardupilot_gz_bringup iris_maze.launch.py` 或同类官方 bringup | 当前使用自定义 compose、world、model，并已接入 `ardupilot_gazebo` JSON plugin | 部分匹配 |
| ROS2/DDS 接口 | ROS graph 中应有 `/ap` 节点，`DDS_ENABLE=1`，`DDS_DOMAIN_ID` 匹配 `ROS_DOMAIN_ID` | 当前没有把 `/ap/*` 作为主接口验收条件 | 不匹配 |
| 官方 Cartographer package | `ardupilot_cartographer cartographer.launch.py` | 当前用 `navlab_slam_bringup` + `navlab_cartographer_adapter` 包装 Cartographer | 部分匹配 |
| Cartographer Lua | 官方 `tracking_frame=imu_link`、`use_odometry=true`、`/odom -> /odometry` | 已将 `navlab_cartographer_2d.lua` 收敛到官方 baseline | 匹配配置，运行链仍需验证 |
| EKF 参数 | 官方 SITL baseline：`VISO_TYPE=1`、`EK3_SRC1_POSZ=1`、`EK3_SRC1_VELXY=6`、`EK3_SRC1_VELZ=6` | 已将 `profiles/navlab-sitl-external-nav.parm` 收敛到官方 baseline | 匹配配置，运行链仍需验证 |
| ExternalNav 入口 | 官方路线应优先使用 ArduPilot ROS2/DDS 外部里程计接口 | 当前仍有 `/odom -> /external_nav/odom -> MAVLink ODOMETRY -> SITL` 自定义桥 | 不匹配 |
| Rangefinder 定高 | 官方 Cartographer 示例用 Baro 作为 POSZ；NavLab 可以另外注入下视 rangefinder 作为 FCU 外设 | 当前下视 rangefinder 在 `gazebo-sensor` 中通过 MAVLink `DISTANCE_SENSOR` 注入 | 可保留，但不能混成官方 Cartographer EKF baseline |
| rosbag / Foxglove | 官方教程不定义批处理上传；这是 NavLab 自己的验收需求 | 当前已实现 MCAP 记录和上传 | NavLab 扩展，不影响官方对齐 |

## 已经做的收敛

### EKF 参数

`profiles/navlab-sitl-external-nav.parm` 已按官方 Cartographer SITL 参数收敛：

- `AHRS_EKF_TYPE=3`
- `EK2_ENABLE=0`
- `EK3_ENABLE=1`
- `EK3_SRC1_POSXY=6`
- `EK3_SRC1_POSZ=1`
- `EK3_SRC1_VELXY=6`
- `EK3_SRC1_VELZ=6`
- `EK3_SRC1_YAW=6`
- `VISO_TYPE=1`
- `ARMING_CHECK=388598`

NavLab 的下视 rangefinder 参数仍保留，但定位身份改变为 FCU 外设输入，不再作为
这个官方 Cartographer profile 的 EKF 主高度源。

### Cartographer Lua

`navlab_cartographer_2d.lua` 已按官方 `ardupilot_cartographer/config/cartographer.lua`
收敛，包括：

- `tracking_frame="imu_link"`
- `published_frame="base_link"`
- `provide_odom_frame=true`
- `publish_frame_projected_to_2d=false`
- `use_odometry=true`
- `TRAJECTORY_BUILDER_2D.max_range=30`
- `POSE_GRAPH.optimize_every_n_nodes=30`

同时 SLAM launch 增加了官方 `/odom -> /odometry` remap 参数：
`cartographer_odometry_topic="/odometry"`。

## 当前最大的缺口

### 1. 没有以官方 `/ap` DDS 接口作为主通道

官方 ROS2 with SITL 文档明确说，SITL ROS2 launch 会启动：

- `micro-ROS-Agent`
- `ardupilot_sitl`
- MAVLink GCS 进程

并且 ROS2 CLI 应该能看到 `/ap` 节点和 `/ap/*` topic。如果没有 topic，要先检查
`DDS_ENABLE=1` 和 `DDS_DOMAIN_ID` 是否匹配。

NavLab 当前主链路仍然是：

```text
Cartographer /odom
  -> navlab_external_nav_bridge
  -> /external_nav/odom
  -> companion MAVLinkExternalNavSender
  -> MAVLink ODOMETRY
  -> SITL
```

这个能做诊断，但它不是官方首选路线。后续必须把验收条件改成：

- `/ap` 节点存在；
- `/ap/tf` 或官方 ExternalNav 接口持续可用；
- Cartographer / ExternalNav 回灌飞控走官方 ROS2/DDS 路径，MAVLink sender 只做过渡或备用。

P0 doctor 当前实测结果：

- `world-model/navlab-companion:latest` 有 ROS2 CLI，但缺所有官方 ROS2/DDS package。
- `world-model/navlab-slam-cartographer:latest` 有 `cartographer_ros`，但缺官方
  `ardupilot_sitl`、`micro_ros_agent`、`ardupilot_gz_bringup`、
  `ardupilot_cartographer` 等 package。
- `remote-sitl-lab/ardupilot-sitl:stage1-f10500ae45aa` 只有 `arducopter`，
  没有 ROS2 CLI，因此不能单独作为官方 DDS runtime。
- 已新增 dedicated runtime 构建入口：
  `docker/Dockerfile.official-baseline` 和
  `just navlab-official-baseline-image-build`。P0 doctor 现在默认检查
  `world-model/navlab-official-baseline:latest`，该镜像构建完成前 P0 会继续 blocked。

### 2. Gazebo bringup 还不是 `ardupilot_gz_bringup`

当前模型和 world 可以继续保留，因为它们是 NavLab 室内任务需求。但启动机制应逐步迁到
官方 `ardupilot_gz_bringup` 的结构：

- 官方 package 启动 SITL、Gazebo、RViz 的方式；
- 官方模型插件、topic 和 odometry 约定；
- NavLab 只替换 world、model、LiDAR/rangefinder 配置和 rosbag acceptance。

### 3. SLAM backend 仍有自定义 adapter

`navlab_cartographer_adapter` 现在负责把 Cartographer TF 转成 `/odom` 并输出
`/navlab/slam/status`。这对 acceptance 很方便，但和官方 `ardupilot_cartographer`
的简洁 launch 不完全一致。

合理保留方式：

- Cartographer backend 内部尽量直接复用官方 launch/config；
- NavLab adapter 只作为健康检查和 artifact 辅助，不参与改变 SLAM 数学输出；
- ExternalNav 回灌路径迁到官方 DDS 后，`/external_nav/odom` bridge 应降级为过渡组件。

## 后续 Phase

### P0：官方接口验收

- 启动后 `ros2 node list` 必须看到 `/ap`。
- `ros2 node info /ap` 必须能列出 ArduPilot DDS topics。
- summary 记录 `DDS_ENABLE`、`DDS_DOMAIN_ID`、`ROS_DOMAIN_ID`。
- rosbag required topics 加入 `/ap/tf` 和关键 `/ap/*` 状态 topic。

### P1：官方 Gazebo bringup 结构

- 在 NavLab world/model 不丢失的前提下，参考 `ardupilot_gz_bringup` 重组启动。
- 不再由 Python 直接控制 Gazebo pose。
- Gazebo truth 只用于诊断和误差分析。

### P2：官方 Cartographer backend

- Cartographer backend 默认使用官方 Lua baseline。
- Cartographer 订阅 `/scan`、`/imu`、`/odometry`。
- adapter 只做状态导出，不更改 Cartographer 输出含义。

### P3：ExternalNav 回灌从 MAVLink 迁到官方 DDS

- 明确官方 `/ap/tf` 或对应 ExternalNav ROS2 输入消息的发布格式。
- 用官方接口把 Cartographer odom 回灌到 ArduPilot。
- MAVLink ODOMETRY sender 保留为 fallback，不作为完成标准。

### P4：真实机器迁移记录

- 真机必须重新记录 X2 外参、IMU 外参、时间同步、scan matcher 参数、EKF source/lane。
- 真机必须配置第二 EKF source，避免外部里程计丢失后飞控失控。
- NavLab 仿真数值可以和真机不同，但接口机制必须一致。

## 当前结论

现在不能说“完全匹配官方教程”。准确说法是：

> NavLab 已具备室内无 GPS SLAM hover 的自定义仿真验收框架，并且 EKF 参数和
> Cartographer Lua 已开始向官方 ArduPilot ROS2 Cartographer baseline 收敛；
> 但启动、ExternalNav 回灌和主通信接口还没有完全切到官方 `ardupilot_ros` /
> `ardupilot_gz` / DDS `/ap/*` 路线。
