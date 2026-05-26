# `just x3-trip` 话题与 Foxglove 使用说明

## 1. 这个命令会启动什么

在目录 `/home/admin/workspace/world-model` 下执行：

```bash
just x3-trip
```

会一次性启动下面几部分：

1. `ydlidar_launch.py`
2. `foxglove_bridge`
3. `ydlidar_ros2_driver_scan_features`
4. `ros2 bag record`

对应关系大致是：

- `justfile` 里，`x3-trip` 最终调用了 `ros2 launch ydlidar_ros2_driver x3_remote_trip.launch.py`
- `x3_remote_trip.launch.py` 里又包含了 `ydlidar_launch.py`
- `ydlidar_launch.py` 实际会启动：
  - `ydlidar_ros2_driver_node`
  - `static_transform_publisher`

所以 `just x3-trip` 本质上是把：

- 雷达驱动
- 扫描特征提取
- Foxglove 远程可视化
- rosbag 录制

这几件事一起跑起来。

当前代码已经改成：

- `just x3-trip` 默认使用 `mcap` 作为 rosbag2 存储后端
- 如果需要临时退回 sqlite3，可以显式传：`just x3-trip bag_storage:=sqlite3`

## 2. 一般会看到哪些 topic

这一节是根据下面这些内容整理出来的：

- 启动文件：`x3/src/ydlidar_ros2_driver/launch/x3_remote_trip.launch.py`
- 驱动代码：`x3/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp`
- 特征节点代码：`x3/src/ydlidar_ros2_driver/src/ydlidar_ros2_scan_features.cpp`
- 实际运行日志：`log.log`

### 2.1 核心雷达话题

| Topic | 类型 | 来源 | 说明 |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | `ydlidar_ros2_driver_node` | 原始 2D 激光扫描，是最核心的话题 |
| `/point_cloud` | `sensor_msgs/msg/PointCloud2` | `ydlidar_ros2_driver_node` | 由同一帧扫描转换出来的点云，适合直接进 Foxglove 3D |

### 2.2 由 `/scan` 派生出来的特征话题

| Topic | 类型 | 来源 | 说明 |
|---|---|---|---|
| `/scan_features` | `ydlidar_interfaces/msg/ScanFeatures` | `ydlidar_ros2_driver_scan_features` | 对扫描结果做的紧凑障碍物特征总结 |
| `/scan_nearest_point` | `geometry_msgs/msg/PointStamped` | `ydlidar_ros2_driver_scan_features` | 当前一帧扫描里最近的有效点 |

`/scan_features` 当前包含这些字段：

- `front_min`
- `left_min`
- `right_min`
- `rear_min`
- `nearest_range`
- `nearest_angle_deg`
- `nearest_point`
- `valid_count`
- `total_count`

如果你的目标不是看完整激光束，而是快速判断：

- 前方最近障碍有多远
- 左右是否贴墙
- 当前最近点在哪里

那 `/scan_features` 往往比直接盯 `/scan` 更方便。

### 2.3 TF 和系统相关话题

| Topic | 类型 | 来源 | 说明 |
|---|---|---|---|
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `static_transform_publisher` | `base_link` 到 `laser_frame` 的静态变换 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 默认录制目标 | 动态 TF；默认会加入录制列表，但不一定真的有消息 |
| `/rosout` | `rcl_interfaces/msg/Log` | ROS 2 运行时 | ROS 日志输出 |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ROS 2 运行时 | 参数变化事件 |
| `/ydlidar_ros2_driver_node/transition_event` | lifecycle event | 生命周期节点 | 驱动节点状态切换事件 |
| `/events/write_split` | `rosbag2_interfaces/msg/WriteSplitEvent` | rosbag2 | rosbag 分片写入事件 |

这里有一个需要特别说明的点：

- `/tf` 在默认 rosbag 录制列表里
- 但它是否真正出现，要看运行时有没有节点在发动态 TF
- 你给的这份 `log.log` 对应的 bag 元数据里没有 `/tf`
- 也就是说，这一次运行虽然配置里要录 `/tf`，但实际上没有 `/tf` 消息写进去

### 2.4 只在 Foxglove 交互时出现的话题

在 `log.log` 里还看到过下面这些 topic：

| Topic | 类型 | 说明 |
|---|---|---|
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 初始位姿工具 |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | 点击工具输出 |
| `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | 目标点工具输出 |

这些不是 `just x3-trip` 本身固定产出的核心话题，而是 Foxglove 客户端连接后，在交互过程中临时广告出来的话题。

所以如果别人问“`just x3-trip` 会生成哪些 topic”，这 3 个更适合归类为：

- Foxglove 交互话题

而不是：

- 雷达主流程核心输出

### 2.5 为什么现在要用 `PointCloud2`

`/point_cloud` 之前如果是旧的 `sensor_msgs/msg/PointCloud`，在 Foxglove 里经常会遇到：

- `Raw Messages` 能看到
- 但 `3D` 面板不按预期显示点云

现在这里改成 `sensor_msgs/msg/PointCloud2`，原因很直接：

- Foxglove `3D` 面板对点云的主流支持类型是 `PointCloud2`
- `PointCloud2` 也是当前 ROS 2 生态里更通用的点云消息格式
- 所以改完之后，`/point_cloud` 更适合直接拿来在 Foxglove `3D` 面板里渲染

如果改完后仍然看不到，一般再检查这两件事：

1. `3D` 面板里有没有勾选 `/point_cloud`
2. `Display frame` / `Fixed frame` 是否和 `laser_frame`、`base_link` 的 TF 关系对得上

## 3. 默认会录哪些 rosbag topic

在默认参数下，`just x3-trip` 会录这些话题：

- `/scan`
- `/point_cloud`
- `/tf`
- `/tf_static`
- `/rosout`
- `/scan_features`
- `/scan_nearest_point`

原因是：

- `bag_storage` 默认是 `mcap`
- `with_features` 默认是 `true`
- `record_all` 默认是 `false`
- 所以不是全录，而是按固定 topic 列表录制

也就是说，现在默认产物应该是：

- `metadata.yaml`
- `*.mcap`

如果你这样启动：

```bash
just x3-trip record_all:=true
```

那就会变成录制当前图里所有 topic。

如果你想额外追加一些话题，可以这样：

```bash
just x3-trip extra_topics:=/diagnostics,/imu/data
```

这样会在默认录制列表后面再加上这些 topic。

如果你想临时切回旧格式，也可以这样：

```bash
just x3-trip bag_storage:=sqlite3
```

## 4. 这次 `log.log` 里实际发生了什么

这一节对应的是你之前提供的一次历史运行日志；那次运行发生在默认切到 MCAP 之前，所以它仍然是旧的 sqlite3 / db3 输出。

从 `log.log` 可以直接看出，这次运行里：

- Foxglove WebSocket 地址是：`ws://<robot-ip>:8765`
- rosbag 输出目录是：`/home/admin/workspace/world-model/x3/bags/x3_trip_20260522_150040`
- 实际数据库文件是：`/home/admin/workspace/world-model/x3/bags/x3_trip_20260522_150040/x3_trip_20260522_150040_0.db3`

录制器在日志里明确订阅了：

- `/tf_static`
- `/scan_nearest_point`
- `/scan_features`
- `/scan`

对应的 bag 元数据 `x3/bags/x3_trip_20260522_150040/metadata.yaml` 里，最终实际写入的是：

| Topic | 消息数 |
|---|---:|
| `/scan` | 10197 |
| `/scan_features` | 10197 |
| `/scan_nearest_point` | 10197 |
| `/tf_static` | 1 |

所以这次运行最值得记住的结论是：

- 默认配置里包含 `/tf`
- 但这次实际落盘的数据只有 `/scan`、`/scan_features`、`/scan_nearest_point`、`/tf_static`

## 5. Foxglove 用什么面板看比较合适

如果目标是“打开 Foxglove 后，能比较快地看懂这一趟数据”，我建议优先用下面这套面板组合：

1. `3D`
2. `Plot`
3. `Raw Messages`
4. `Log`

### 5.1 `3D` 面板

建议重点看这些 topic：

- `/scan`
- `/point_cloud`（可选）
- `/tf_static`
- `/clicked_point`（如果你在用点击交互）

适合原因：

- `/scan` 能直接看实时激光形状
- `/point_cloud` 现在是 `PointCloud2`，更适合直接进 Foxglove `3D`
- `/tf_static` 能保证坐标关系正常显示

### 5.2 `Plot` 面板

建议直接画这些字段：

- `/scan_features.front_min`
- `/scan_features.left_min`
- `/scan_features.right_min`
- `/scan_features.rear_min`
- `/scan_features.nearest_range`

适合原因：

- 这是看“障碍距离随时间怎么变化”最快的方法
- 回放 bag 时也很直观，特别适合看是否有贴边、逼近障碍物的情况

### 5.3 `Raw Messages` 面板

建议放这些 topic：

- `/scan_features`
- `/scan_nearest_point`
- `/scan`

适合原因：

- 可以直接看消息原始字段
- 对调试特征提取结果尤其有帮助

### 5.4 `Log` 面板

建议关注：

- `/rosout`

主要用途是：

- 看驱动警告
- 看断连信息
- 看生命周期变化
- 看 rosbag 状态

### 5.5 一个实用的布局建议

如果你只想先有一套能直接用的布局，可以这样排：

- 左边：`3D`
- 右上：`Plot`
- 右中：`Raw Messages`
- 右下：`Log`

这套布局既适合实时看，也适合回放 rosbag。

## 6. rosbag 存储位置

### 6.1 默认根目录

默认 rosbag 根目录是：

```text
/home/admin/workspace/world-model/x3/bags/
```

每次运行会新建一个带时间戳的目录，格式类似：

```text
/home/admin/workspace/world-model/x3/bags/x3_trip_YYYYMMDD_HHMMSS/
```

目录里面通常会有：

```text
x3_trip_YYYYMMDD_HHMMSS_0.mcap
```

以及：

```text
metadata.yaml
```

### 6.2 这次日志对应的具体路径

你给的 `log.log` 对应的是一次旧的 sqlite3 运行，它的 bag 路径是：

```text
/home/admin/workspace/world-model/x3/bags/x3_trip_20260522_150040/
```

对应数据库文件是：

```text
/home/admin/workspace/world-model/x3/bags/x3_trip_20260522_150040/x3_trip_20260522_150040_0.db3
```

## 7. 最简短结论

如果只看最短版，可以直接记这几条：

- `just x3-trip` 的核心输出 topic 主要是 `/scan`、`/point_cloud`、`/scan_features`、`/scan_nearest_point`、`/tf_static`
- 默认 rosbag 录制列表是 `/scan`、`/point_cloud`、`/tf`、`/tf_static`、`/rosout`、`/scan_features`、`/scan_nearest_point`
- 默认 rosbag 存储后端现在是 `mcap`
- 这次 `log.log` 对应的 bag 实际写入的是 `/scan`、`/scan_features`、`/scan_nearest_point`、`/tf_static`
- Foxglove 最推荐先用 `3D + Plot + Raw Messages + Log`
- rosbag 默认存储根目录是 `/home/admin/workspace/world-model/x3/bags/`
- 这次运行对应的 bag 目录是 `/home/admin/workspace/world-model/x3/bags/x3_trip_20260522_150040/`
