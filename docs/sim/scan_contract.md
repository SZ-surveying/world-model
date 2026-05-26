# `/scan` 契约

这份文档定义当前仿真路径里的 `/scan` 统一契约。

目标不是“Gazebo 内部实现看起来像真实驱动”，而是让 **ROS2 外部观察到的行为** 与真实 `x3` 驱动保持一致。后续 Gazebo lidar、真实 `x3` 都必须满足这份契约。

## 基准来源

当前真实基准来自 `x3` 驱动源码和默认 launch / 参数文件：

- [x3/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp](/home/nn/workspace/3588/world-model/x3/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp:194)
- [x3/src/ydlidar_ros2_driver/launch/ydlidar_launch.py](/home/nn/workspace/3588/world-model/x3/src/ydlidar_ros2_driver/launch/ydlidar_launch.py:20)
- [x3/src/ydlidar_ros2_driver/params/X2.yaml](/home/nn/workspace/3588/world-model/x3/src/ydlidar_ros2_driver/params/X2.yaml:1)

当前默认基线按 `ydlidar_launch.py -> X2.yaml` 定义。

这意味着：如果未来你切换到别的参数文件，例如 `X4.yaml`、`G2.yaml`，那么 Gazebo 也要跟着切换到新的基线，而不是继续写死旧参数。

## 必须一致的内容

下面这些是 Gazebo / real 两种 source 必须一致的外部契约。

### 1. Topic 和消息类型

- topic 名：`/scan`
- 消息类型：`sensor_msgs/msg/LaserScan`

这是最基本的订阅边界。下游逻辑不应该订阅别的 topic 再自己 remap 回来。

### 2. QoS

真实驱动使用：

```text
rclcpp::SensorDataQoS()
```

所以 Gazebo bridge 也应该使用等价的 sensor-data QoS，而不是默认 QoS。

### 3. Frame

- `header.frame_id = "laser_frame"`

当前默认 launch 还会发布一个静态 TF：

```text
base_link -> laser_frame
translation: (0, 0, 0.02)
rotation: identity
```

如果 Gazebo 需要提供 TF，也应该对齐这个默认关系，除非你明确切换到另一套 rig 参数。

### 4. 参数语义

当前 `X2.yaml` 基线参数：

- `angle_min = -180 deg`
- `angle_max = 180 deg`
- `range_min = 0.1 m`
- `range_max = 12.0 m`
- `frequency = 10.0 Hz`
- `frame_id = laser_frame`
- `invalid_range_is_inf = false`
- `intensity = false`

转成 `LaserScan` 后，应体现为：

- `angle_min = -pi`
- `angle_max = pi`
- `range_min = 0.1`
- `range_max = 12.0`
- `scan_time` 约等于 `0.1 s`

### 5. 时间字段语义

真实驱动会填写：

- `header.stamp`
- `scan_time`
- `time_increment`

Gazebo 至少要做到：

- `header.stamp` 是每帧真实发布时间或仿真时间
- `scan_time` 与发布频率一致
- `time_increment` 合理，不留空或全乱值

### 6. 数组形状

真实驱动会：

- 按 `angle_min`、`angle_max`、`angle_increment` 计算 beam 数量
- `ranges.size()` 与 `intensities.size()` 一致

Gazebo 输出也必须满足这一点。

### 7. 无效值策略

当前 `X2.yaml` 基线：

- `invalid_range_is_inf = false`

也就是说，Gazebo 默认不应该把“未命中”直接发成 `inf`，而应该与真实基线策略保持一致。

如果后面你决定把真实雷达配置改成 `invalid_range_is_inf = true`，那 Gazebo 也要同步改。

## 可以近似一致的内容

下面这些不要求与真实驱动逐行等价，但要保持统计和语义一致。

### 1. 内部扫描实现

- Gazebo 可以通过 ray sensor + bridge 生成数据
- 真实 `x3` 是 SDK 扫描循环

两者内部实现不同没关系，只要 ROS2 外部看到的 `/scan` 契约一致。

### 2. 噪声模型

- P1 Gazebo 可以先理想模型
- 后续再逐步加入噪声、丢点、量化误差

第一阶段先追求契约一致，不先追求噪声一致。

### 3. `point_cloud` 与服务

真实驱动还会发布：

- `/point_cloud`

并提供服务：

- `start_scan`
- `stop_scan`

这些不是当前最小避障主链路的硬要求，但如果后续你希望仿真环境完全模拟真实驱动行为，可以再补。

建议：

- 当前阶段不做
- P1 可选
- P2 若开始做 source 无缝切换，再评估是否补齐

## 当前建议的统一基线

在没有切换真实参数文件之前，Gazebo 仿真统一按下面这组基线对齐：

- topic：`/scan`
- type：`sensor_msgs/msg/LaserScan`
- QoS：sensor-data
- frame：`laser_frame`
- angle range：`[-pi, pi]`
- beam count：当前 Gazebo 基线默认 `361`
- range：`[0.1, 12.0]`
- publish rate：`10 Hz`
- invalid range policy：与 `X2.yaml` 一致，默认不是 `inf`

这里的 `beam count = 361` 是当前 Gazebo 基线，用 1 deg 增量覆盖 `[-180, 180]`。
因为 `x3` 真实驱动的 `angle_increment` 来自运行时扫描配置，而不是 `X2.yaml` 直接写死的常量，所以后续拿到真实输出后，需要再做一次对拍确认。

## 对 Gazebo scan 的要求

Gazebo 侧允许内部使用自己的 sensor topic，但桥接到 ROS2 后必须收敛成真实基线：

- ROS2 暴露 topic 为 `/scan`
- ROS2 消息类型为 `sensor_msgs/msg/LaserScan`
- frame 为 `laser_frame`
- 发布频率接近真实驱动
- beam 排布和角度语义与真实基线一致

如果 Gazebo 原生输出与真实基线不一致，就应在 bridge 或 sensor 配置层修正，而不是让下游 consumer 兼容两个版本。

## 设计规则

`/scan` 契约只允许有一份真相。

后续如果：

- Gazebo `/scan` 需要另一套特判
- real `x3` `/scan` 又需要第三套特判

那说明 source 适配边界设计错了。
