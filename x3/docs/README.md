# 项目说明

## 1. 项目简介

本项目是一个基于 ROS 2 的 YDLIDAR 激光雷达工作空间，核心功能由 `src/ydlidar_ros2_driver` 包提供，用于连接 YDLIDAR 设备、读取扫描数据，并通过 ROS 2 话题和服务对外发布。

从当前目录结构来看，这个工作空间已经进行过编译，包含：

- `src/`：源码目录
- `build/`：colcon 构建产物
- `install/`：安装产物与环境脚本
- `log/`：构建日志

当前源码部分主要是 `ydlidar_ros2_driver`，同时工作空间中已经存在 `ydlidar_sdk` 的安装结果，说明该驱动依赖的 SDK 已经在本环境中参与过构建或安装。

## 2. 当前最小目标

当前最小链路非常简单，只保留这四步：

- 启动 YDLIDAR 驱动
- 发布 `/scan`
- 消费 `/scan` 并生成 `/scan_features`
- 在终端直接查看 `/scan_features`

也就是：

`ydlidar_launch.py -> /scan -> ydlidar_ros2_driver_scan_features -> /scan_features`

## 3. 核心组成

### 3.1 当前相关可执行程序

当前最相关的是这两个：

- `ydlidar_ros2_driver_node`
  - 主驱动节点
  - 负责读取雷达数据并发布 `/scan`
- `ydlidar_ros2_driver_scan_features`
  - 特征提取节点
  - 订阅 `/scan` 并发布 `/scan_features`

另外现在有一个轻量接口包：

- `ydlidar_interfaces`
  - 提供 `ScanFeatures.msg`
  - 用来替代之前不易维护的 `Float32MultiArray`

### 3.2 当前相关 Launch 文件

当前只需要关注：

- `ydlidar_launch.py`
  - 启动驱动节点
  - 发布 `/scan`

### 3.3 当前默认参数文件

当前默认使用的是：

- `src/ydlidar_ros2_driver/params/X2.yaml`

## 4. 目录结构说明

建议重点关注以下目录：

```text
x3/
├── src/
│   └── ydlidar_ros2_driver/
│       ├── launch/          # ROS 2 启动文件
│       ├── params/          # 雷达参数配置
│       ├── startup/         # 设备权限和别名脚本
│       ├── src/             # C++ 源码
│       ├── README.md        # 原始英文说明
│       └── details.md       # 型号参数对照表
├── build/                   # colcon 构建目录
├── install/                 # 安装目录与 setup 脚本
└── log/                     # 构建日志
```

## 5. 运行依赖

根据当前包定义和源码，项目依赖主要包括：

- ROS 2
- `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `std_srvs`
- `ydlidar_sdk`

## 6. 构建方式

如果你是在这个工作空间根目录执行构建，可以使用：

```bash
colcon build --symlink-install
```

构建完成后加载环境：

```bash
source install/setup.bash
```

如果后续经常使用，建议把环境加载命令加入 `~/.bashrc`：

```bash
echo "source /home/admin/workspace/x3/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 7. 当前默认配置

当前如果你直接运行：

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

实际采用的是 launch 文件中默认指定的参数文件。你当前关注的是 `X2`，所以后续建议把启动配置统一收敛到 `X2.yaml`。

`src/ydlidar_ros2_driver/params/X2.yaml` 里常见配置项包括：

| 参数名 | 说明 | 当前默认值 |
| --- | --- | --- |
| `port` | 雷达设备串口 | `/dev/ttyUSB0` |
| `frame_id` | 雷达坐标系名称 | `laser_frame` |
| `baudrate` | 串口波特率 | `115200` |
| `lidar_type` | 雷达类型 | `1` |
| `device_type` | 设备类型 | `0` |
| `sample_rate` | 采样率 | `9` |
| `intensity_bit` | 强度位数 | `0` |
| `fixed_resolution` | 是否固定角分辨率 | `true` |
| `reversion` | 是否反转安装方向 | `true` |
| `inverted` | 是否反向输出角度 | `true` |
| `auto_reconnect` | 是否自动重连 | `true` |
| `isSingleChannel` | 是否单通道雷达 | `false` |
| `intensity` | 是否输出强度信息 | `false` |
| `support_motor_dtr` | 是否通过 DTR 控制电机 | `false` |
| `angle_max` | 最大扫描角度 | `180.0` |
| `angle_min` | 最小扫描角度 | `-180.0` |
| `range_max` | 最大量程 | `64.0` |
| `range_min` | 最小量程 | `0.01` |
| `frequency` | 扫描频率 | `10.0` |
| `invalid_range_is_inf` | 无效距离是否输出为无穷大 | `false` |

## 8. 启动方式

如果本机已经安装了 `just`，也可以直接使用仓库根目录下的 `justfile`。当前只保留了最小工作流。

```bash
just x3-build
just x3-launch
just x3-trip
just x3-features
just x3-echo-features
```

查看所有已封装命令：

```bash
just --list
```

### 8.1 启动主驱动

```bash
source install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### 8.2 启动结构化特征节点

如果你想把 `/scan` 进一步转成后续算法更容易消费的结构化输出，可以启动这个节点：

```bash
source install/setup.bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_scan_features
```

这个节点会订阅 `/scan`，并发布：

- `/scan_features`
  - 类型：`ydlidar_interfaces/msg/ScanFeatures`
  - 字段：
    - `front_min`
    - `left_min`
    - `right_min`
    - `rear_min`
    - `nearest_range`
    - `nearest_angle_deg`
    - `nearest_point`
    - `valid_count`
    - `total_count`
- `/scan_nearest_point`
  - 类型：`geometry_msgs/msg/PointStamped`
  - 表示最近障碍点在雷达坐标系下的位置

### 8.3 查看结构化特征

```bash
source install/setup.bash
ros2 topic echo /scan_features
```

### 8.4 一趟采集 + 远程查看 + 自动录包

如果你现在的目标是：

- 插上雷达
- 跑一趟
- 远程用 Foxglove 看实时数据
- 同时把这一趟录成 rosbag2

那么优先用这条：

```bash
cd /home/admin/workspace/world-model
just x3-trip
```

这个命令会一起启动：

- `ydlidar_launch.py`
- `foxglove_bridge`
- `ydlidar_ros2_driver_scan_features`
- `ros2 bag record`

默认行为：

- Foxglove WebSocket 地址：`ws://<机器人IP>:8765`
- rosbag 输出目录：`/home/admin/workspace/world-model/x3/bags/`
- 默认 rosbag 存储后端：`mcap`
- 默认录制话题：
  - `/scan`
  - `/point_cloud`
  - `/tf`
  - `/tf_static`
  - `/rosout`
  - `/scan_features`
  - `/scan_nearest_point`

如果你想直接把 launch 参数透传进去，也可以这样用：

```bash
just x3-trip record_all:=true
just x3-trip foxglove_port:=9001
just x3-trip bag_storage:=sqlite3
just x3-trip extra_topics:=/diagnostics,/imu/data
just x3-trip params_file:=/home/admin/workspace/world-model/x3/src/ydlidar_ros2_driver/params/X2.yaml
```

如果你不想通过 `just`，也可以直接运行：

```bash
cd /home/admin/workspace/world-model/x3
source install/setup.bash
ros2 launch ydlidar_ros2_driver x3_remote_trip.launch.py
```

Foxglove 客户端里填：

```text
ws://<机器人IP>:8765
```

关于 `/point_cloud`：

- 现在驱动发布的是 `sensor_msgs/msg/PointCloud2`
- 这样可以直接被 Foxglove 的 `3D` 面板识别
- 之前如果是旧的 `sensor_msgs/msg/PointCloud`，Foxglove `3D` 面板通常不会按点云正常渲染

结束采集时，直接在启动终端按 `Ctrl-C`，`rosbag2` 会正常收尾。

### 8.5 每次开跑前检查

如果下面 5 件事已经满足，那么基本就是“接上雷达就能用”：

1. `x3` 工作空间已经成功构建过，并且有 `install/setup.bash`
2. 本机已经安装 `foxglove_bridge`
3. 如果默认录制 MCAP，本机已经安装 `rosbag2_storage_mcap`
4. 雷达串口权限没问题，当前用户能访问设备
5. 参数文件里的 `port`、`baudrate`、型号相关参数和你的设备一致

也就是说，不需要你每次再改代码；但第一次环境准备没做好时，仍然会卡在串口、权限或参数不匹配上。

## 9. 最小使用顺序

建议开 3 个终端，按下面顺序跑：

### 终端 1：启动雷达驱动

```bash
cd /home/admin/workspace/world-model/x3
source install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

或：

```bash
just x3-launch
```

### 终端 2：启动特征提取

```bash
cd /home/admin/workspace/world-model/x3
source install/setup.bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_scan_features
```

或：

```bash
just x3-features
```

### 终端 3：查看特征输出

```bash
cd /home/admin/workspace/world-model/x3
source install/setup.bash
ros2 topic echo /scan_features
```

或：

```bash
just x3-echo-features
```

## 10. 话题与服务

### 10.1 发布话题

| 话题名 | 类型 | 说明 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | 激光扫描结果 |
| `/scan_features` | `ydlidar_interfaces/msg/ScanFeatures` | 结构化扇区距离和最近障碍摘要 |
| `/scan_nearest_point` | `geometry_msgs/msg/PointStamped` | 最近障碍点坐标 |

### 10.2 提供服务

| 服务名 | 类型 | 说明 |
| --- | --- | --- |
| `/start_scan` | `std_srvs/srv/Empty` | 启动雷达扫描 |
| `/stop_scan` | `std_srvs/srv/Empty` | 停止雷达扫描 |

### 10.3 常用调试命令

查看扫描数据：

```bash
ros2 topic echo /scan
```

查看结构化特征：

```bash
ros2 topic echo /scan_features
```

查看最近障碍点：

```bash
ros2 topic echo /scan_nearest_point
```

停止扫描：

```bash
ros2 service call /stop_scan std_srvs/srv/Empty
```

重新开始扫描：

```bash
ros2 service call /start_scan std_srvs/srv/Empty
```

## 11. 权限与串口问题

如果雷达通过 USB 串口连接，常见问题是设备权限不足。项目里提供了 `startup/initenv.sh` 脚本，可用于创建串口别名或设置权限。示例命令：

```bash
chmod 0777 src/ydlidar_ros2_driver/startup/*
sudo sh src/ydlidar_ros2_driver/startup/initenv.sh
```

执行后建议重新插拔雷达设备，再确认实际设备名是否为：

- `/dev/ttyUSB0`
- `/dev/ydlidar`

然后再对应修改参数文件里的 `port`。

## 12. 常见问题排查

### 12.1 构建时报找不到 `ydlidar_sdk`

说明 SDK 没有正确安装或环境没有加载。优先检查：

- 是否已成功安装 `ydlidar_sdk`
- 是否重新执行过 `source install/setup.bash`
- `CMAKE_PREFIX_PATH` 中是否包含工作空间安装路径

### 12.2 启动后没有扫描数据

优先检查以下几项：

- 串口是否正确，例如 `/dev/ttyUSB0`
- 波特率是否和设备型号匹配
- `isSingleChannel`、`intensity`、`sample_rate` 是否与设备型号一致
- 当前用户是否有串口访问权限
- 雷达是否正常供电并已转动

### 12.3 启动了特征节点但没有 `/scan_features`

优先检查：

- 驱动节点是否已经先启动
- `/scan` 是否有持续输出
- 是否已经运行 `ydlidar_ros2_driver_scan_features`
- 当前终端是否执行过 `source install/setup.bash`

## 13. 参考文件

可进一步查看以下文件获取细节：

- `src/ydlidar_ros2_driver/README.md`
- `src/ydlidar_ros2_driver/details.md`
- `src/ydlidar_ros2_driver/params/X2.yaml`
- `src/ydlidar_ros2_driver/launch/ydlidar_launch.py`
- `src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp`
- `src/ydlidar_ros2_driver/src/ydlidar_ros2_scan_features.cpp`
- `src/ydlidar_interfaces/msg/ScanFeatures.msg`
