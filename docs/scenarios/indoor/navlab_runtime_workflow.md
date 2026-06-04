# NavLab 无 GPS SLAM 悬停运行工作流

这份文档描述当前批处理 acceptance 的运行口径。它只覆盖世界、传感器、真实 SLAM feedback、ExternalNav 和 FCU 悬停，不覆盖完整探索任务。

## 1. 启动顺序

```text
1. Orchestration 读取 orchestration/config.toml
2. 启动 Gazebo
3. 启动 ArduPilot SITL
4. 启动 mavlink-router
5. 启动 gazebo-sensor
6. 启动 SLAM 服务
7. 启动 companion 服务
8. companion 等待 scan、IMU、SLAM odom、ExternalNav、FCU heartbeat
9. acceptance 启动 rosbag record
10. mission hover controller 下发 GUIDED、arm、takeoff、hover setpoint
11. acceptance 收集 summary、日志、rosbag profile summary
12. orchestration 上传 MCAP 到 Foxglove
```

关键原则：

- Gazebo 中的无人机由 SITL 和 Gazebo plugin 驱动。
- companion 不直接写 Gazebo pose。
- SLAM 是 ExternalNav 验收输入。
- Gazebo truth 只作为诊断和误差对照。

## 2. 服务角色

### Orchestration

Host 侧批处理入口。它负责 Docker 生命周期、artifact 汇总和 Foxglove 上传，不参与 ROS 数据发布。

### Gazebo

加载 8 字形室内世界和无人机模型，提供物理、碰撞、ray sensor、Gazebo pose 信息。

### SITL

模拟真实飞控。它接收 ExternalNav、执行 `GUIDED`、arm、takeoff 和 hover setpoint，并输出 FCU 遥测。

### mavlink-router

提供 MAVLink 总线。真实部署时仍然需要 router，因为地面站和机载计算盒子会同时使用飞控数据，只是地面站通常是低频观察。

### gazebo-sensor

把 Gazebo `/scan_ideal` 通过 X2 虚拟串口和厂商 driver 转成 `/scan`。

### SLAM

消费 `/scan + /imu/data`，输出 `/odom` 和 SLAM health。后端可以替换，但接口必须保持稳定。

### companion

读取 MAVLink、发布 IMU、发送 ExternalNav、执行 hover mission、发布可视化 pose 和状态。

## 3. 核心 topic

### 传感器

- `/scan_ideal`：Gazebo ray sensor 输出，只做诊断。
- `/scan`：X2 driver 输出，SLAM 的真实输入。
- `/imu/data`：FCU IMU 桥接输出，SLAM 的真实输入。
- `/imu/status`：IMU 输入健康状态。

### SLAM 和 ExternalNav

- `/odom`：SLAM 输出。
- `/external_nav/odom`：发给飞控的 ExternalNav ROS 出口。
- `/external_nav/status`：ExternalNav bridge 状态。
- `/mavlink_external_nav/status`：MAVLink `ODOMETRY` sender 状态。
- `/cartographer/status`：当前 Cartographer 后端状态；替换 SLAM 后可以换成等价 status topic。

### FCU 和可视化

- `/navlab/mavlink/status`：heartbeat、mode、armed、EKF、ACK、STATUSTEXT、local position 观测。
- `/navlab/fcu/local_position_pose`：FCU `LOCAL_POSITION_NED` 转 ROS pose。
- `/navlab/mission/status`：hover mission 状态。
- `/sim/uav_pose`：Foxglove 显示用无人机 pose。
- `/sim/markers`：世界、无人机和调试 marker。
- `/gazebo/truth/odom`：Gazebo 物理真值，只做诊断。

### TF

- `/tf`
- `/tf_static`

必须保证 `navlab_world`、`map`、`odom`、`base_link`、`imu_link`、`laser_frame` 可以通过 TF 链连接。

## 4. Acceptance 判定

`summary.json` 至少应包含：

- `ok`
- `blocked`
- `blocker`
- `scan_status`
- `imu_status`
- `slam_status`
- `external_nav_status`
- `mavlink_external_nav_status`
- `fcu_status`
- `mission_summary`
- `hover_summary`
- `rosbag_profile`
- `set_pose_count`
- `external_nav_input_topic`
- `slam_odom_source`

通过条件：

- `ok == true`
- `blocked` 不存在或不是 `true`
- `/scan` healthy
- `/imu/data` healthy
- SLAM `/odom` healthy
- `external_nav_input_topic` 是 SLAM odom，不是 `/gazebo/truth/odom`
- ExternalNav healthy
- MAVLink ODOMETRY sending
- FCU 进入 `GUIDED`
- FCU arm 成功
- FCU takeoff 成功
- FCU 输出 `LOCAL_POSITION_NED`
- hover drift 在阈值内
- `set_pose_count == 0`
- rosbag required topics 全部存在

失败但有价值的诊断情况：

- SLAM 不 healthy：不能标记当前阶段完成。
- ExternalNav 输入来自 Gazebo truth：只能说明 FCU gate 或 Gazebo plugin 诊断通过，不能算真实 feedback。
- FCU 没有输出 `LOCAL_POSITION_NED`：优先查 SITL 参数、ExternalNav 消息频率、时间戳、frame 和 EKF 状态。
- Foxglove 中墙体跟着无人机动：优先查 marker frame，不是飞控问题。
- `/scan` 显示方向反了：优先查 X2 emulator 和 driver 输出方向，不要在 mission 里硬编码反向解释。

## 5. Rosbag 口径

rosbag 采用最小 required topic 加配置追加 extras 的方式。

最小 required topic：

```text
/clock
/tf
/tf_static
/scan
/scan_ideal
/imu/data
/imu/status
/odom
/external_nav/odom
/external_nav/status
/mavlink_external_nav/status
/navlab/mavlink/status
/navlab/fcu/local_position_pose
/navlab/mission/status
/sim/uav_pose
/sim/markers
/gazebo/truth/odom
/gazebo/truth/status
```

配置中可以追加调试 topic，例如 `/map`、`/submap_list`、`/scan_features`、`/rosout`。代码里的硬编码白名单只表达“最低不可删除集合”，不表达全部记录集合。

## 6. Foxglove 回放

推荐面板：

- 3D
- Raw Messages
- Plot
- Topic Graph

3D 配置：

- 固定参考系：`navlab_world`。如果当前 SLAM 只发布 `map` 且没有桥接，则临时使用 `map`。
- 显示：打开 `/sim/markers`、TF、LaserScan `/scan`、Pose/Odometry `/sim/uav_pose`、`/odom`。
- 跟踪：可以跟踪无人机，但不要把固定参考系设成 `base_link`。

回放时应能看到：

- 8 字形走廊墙体固定不动。
- 无人机模型从原点起飞并悬停。
- `/scan` 和墙体方向一致。
- `/odom`、`/external_nav/odom`、FCU local position 趋势一致。
- mission status 从 wait ready 进入 hover complete。

## 7. 实机迁移

仿真中被替换的部分：

- Gazebo/SITL 替换为真实无人机和真实 FCU。
- X2 emulator 替换为真实 X2 driver。
- Gazebo truth 删除。

保持不变的部分：

- `/scan`
- `/imu/data`
- `/odom`
- `/external_nav/odom`
- `/external_nav/status`
- `/mavlink_external_nav/status`
- `/navlab/mavlink/status`
- rosbag profile

只要当前阶段不依赖 Gazebo truth 和 direct set pose，实机迁移才有意义。
