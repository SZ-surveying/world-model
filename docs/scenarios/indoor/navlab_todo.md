# 阶段 1.5 TODO：无 GPS Companion + SITL + Gazebo 闭环

## 1. 目标

这份 TODO 对应 `docs/scenarios/indoor/navlab_companion_sitl_gazebo_design.md`。

阶段 1.5 的目标是把阶段 0 的 Gazebo 感知世界和阶段 1 的 ArduPilot SITL ExternalNav 闭环接成一个可回放、可验收、接近真实无人机架构的无 GPS 仿真系统。

核心原则：

- `ArduPilot SITL` 只当飞控
- companion 镜像只当机载计算盒子
- Gazebo 只当世界和传感器环境
- 任务控制必须通过 MAVLink 下发给 SITL
- Gazebo UAV 位姿必须跟随 SITL/飞控状态，不再由 planner 直接 set_pose
- 每次 acceptance 必须绑定 rosbag，便于 Foxglove 回放

## 2. 当前状态总览

- [x] 阶段 0 Gazebo `/scan -> /scan_features` 主线已打通
- [x] 阶段 0 可移动 UAV marker 与 lidar 已打通
- [x] 阶段 1 SITL ExternalNav synthetic acceptance 已通过
- [x] 阶段 1 hold/hover 和 local setpoint synthetic acceptance 已通过
- [x] 阶段 1 MAVLink `ODOMETRY` 字段映射已固定
- [x] 独立 companion 服务骨架已拆出
- [x] 独立 companion 镜像 Dockerfile 已拆出，基于 `remote-sitl-lab/ros-jazzy-base:latest`
- [x] SITL `LOCAL_POSITION_NED -> Gazebo set_pose` pose mirror 核心实现已落地
- [x] FCU MAVLink IMU 到 `/imu/data` 的桥接入口已固定
- [x] Gazebo `/scan + FCU IMU -> SLAM -> ExternalNav -> SITL` 启动入口已放入 companion
- [ ] Gazebo `/scan + FCU IMU -> SLAM -> ExternalNav -> SITL` 真正无 GPS 反馈链路尚未作为 NavLab 实跑验收通过
- [x] MAVLink obstacle mission controller 第一版已实现
- [x] NavLab rosbag profile 已规划，并已兼容 Python rosbag topic loader
- [ ] Foxglove 回放检查流程待形成固定 artifact

## 3. P0：架构和镜像边界固定

目标：

- 固定 NavLab 的服务拆分
- 明确 SITL、Gazebo、companion、mavlink-router 的职责
- 防止后续又回到 planner 直接移动 Gazebo 模型的路径

任务：

- [x] 新增 companion 服务或 profile，镜像独立于 `sitl` 和 `gazebo`
- [x] companion 镜像从 `remote-sitl-lab/ros-jazzy-base:latest` 单独构建
- [x] companion 镜像内构建 `ydlidar_interfaces`、`imu_bridge`、`cartographer_indoor`、`external_nav_bridge`、`indoor_bringup`
- [x] companion 镜像内包含 Python MAVLink/mission controller 运行依赖
- [ ] companion 镜像内包含或可解析 `cartographer_ros`
- [x] companion 镜像内包含 rosbag2 MCAP 插件并经 doctor 确认
- [x] compose 中提供 NavLab profile，例如 `navlab`
- [x] NavLab 启动时禁用 `SIM_CMD_VEL_EXECUTOR_AUTOSTART`
- [x] 明确 mavlink-router 对 SITL、companion、observer 的端口分配
- [x] 明确所有 NavLab ROS 服务使用同一个 `ROS_DOMAIN_ID`
- [x] 形成最短启动入口，例如 `just navlab-up`
- [x] 形成停止入口，例如 `just navlab-down`
- [x] 形成 companion 镜像检查入口，例如 `just navlab-companion-doctor`

验收：

- [ ] SITL、Gazebo、companion、mavlink-router 可独立启动
- [ ] companion 能看到 ROS2 `/scan`
- [ ] companion 能连接 SITL MAVLink heartbeat
- [ ] Gazebo UAV 不会被 `cmd_vel_executor` 直接移动
- [ ] `just navlab-companion-doctor` 生成 `summary.json` 且 `ok=true`

最新 doctor 证据：

- artifact：`artifacts/ros/navlab_companion_doctor/20260602_202859/summary.json`
- `ros2_available=true`
- `colcon_available=true`
- `rosbag2_storage_mcap=true`
- `ydlidar_interfaces`、`imu_bridge`、`cartographer_indoor`、`external_nav_bridge`、`indoor_bringup` 均存在
- `pymavlink` 和 NavLab Python MAVLink 节点均存在
- blocked：`cartographer_ros_present=false`

## 4. P1：飞控遥测拆分与 IMU 桥接

目标：

- companion 从 SITL 获取飞控状态和 IMU
- 将 FCU IMU 统一成 SLAM 可消费的 `/imu/data`

任务：

- [x] 固定 `/navlab/mavlink/status` 作为 MAVLink telemetry decoder 输出
- [x] 解析 `HEARTBEAT`
- [x] 解析 `COMMAND_ACK`
- [x] 解析 `STATUSTEXT`
- [x] 解析 `LOCAL_POSITION_NED`
- [x] 解析 `EKF_STATUS_REPORT`
- [x] 解析 `HIGHRES_IMU` 或 `RAW_IMU`
- [x] 如果 DDS/MAVROS IMU 可用，明确优先级和 fallback
- [x] 输出 `/imu/data`
- [x] 输出 `/imu/status`
- [x] 输出 `/navlab/mavlink/status`

验收：

- [ ] `/imu/status.state == "streaming_fcu_imu"`
- [ ] `/imu/data` 频率满足 SLAM 最低要求
- [x] `/navlab/mavlink/status` 编码字段包含 heartbeat、mode、armed、EKF、COMMAND_ACK、STATUSTEXT、local position
- [ ] `/navlab/mavlink/status` 实跑能显示 heartbeat、mode、armed、EKF、local position
- [ ] rosbag 中可回放 IMU 和 MAVLink 状态

## 5. P2：SITL 到 Gazebo 的 pose mirror

目标：

- 让 Gazebo 中的可视化 UAV 和 lidar 位姿跟随飞控本地位置
- 替代 Stage 0 中 planner 直接 `set_pose` 的执行器路径

任务：

- [x] 新增 `mavlink_gazebo_pose_mirror`
- [x] 从 MAVLink `LOCAL_POSITION_NED` 读取飞控本地位置
- [x] 做 NED 到 Gazebo ENU/world 坐标转换
- [x] 调用 Gazebo `/world/<world>/set_pose`
- [x] 发布 `/sim/uav_pose`
- [x] 发布 `/navlab/pose_mirror/status`
- [x] 处理坐标原点、yaw、z 高度和时间戳
- [x] 加入异常保护：飞控位置超时则停止更新并报 unhealthy

验收：

- [ ] SITL local position 变化时 Gazebo UAV marker 同步移动
- [ ] lidar 跟随 Gazebo UAV 运动
- [ ] UAV 前进时 `/scan_features.front_min` 会随位置变化
- [ ] pose mirror 状态可在 rosbag/Foxglove 中回放

## 6. P3：无 GPS SLAM + ExternalNav 反馈链路

目标：

- 用 Gazebo `/scan` 和 FCU IMU 产生 `/odom`
- 通过 ExternalNav 把定位反馈给 SITL
- 不使用 GPS，不把 `LOCAL_POSITION_NED` 当 SLAM 输入

任务：

- [x] companion entrypoint 支持启动 `cartographer_ros` + `cartographer_indoor` + `external_nav_bridge`
- [x] SLAM 输入固定为 `/scan + /imu/data`
- [x] 输出 `/odom`
- [x] `external_nav_bridge` 消费 `/odom`
- [x] `mavlink_external_nav_sender` 发送 MAVLink `ODOMETRY`
- [x] ExternalNav status 暴露 odom age、frame、rate、IMU gating
- [x] 记录 SLAM 状态到 `/cartographer/status`
- [x] 禁止把飞控 fused local position 作为 SLAM 输入；`LOCAL_POSITION_NED` 只用于 Gazebo pose mirror 和任务状态

验收：

- [ ] `/cartographer/status` 表明真实 SLAM backend 正在消费 scan/IMU
- [ ] `/odom` 连续、frame 正确、频率稳定
- [ ] `/external_nav/status.state == "healthy"`
- [ ] `/mavlink_external_nav/status.state == "sending"`
- [ ] SITL 不在启动宽限期后持续报 `VisOdom: not healthy`
- [ ] rosbag 能回放 `/scan -> /imu/data -> /odom -> /external_nav/odom`

## 7. P4：MAVLink 悬停、前进、绕障任务控制器

目标：

- 由 companion 通过 MAVLink 控制 SITL 完成任务
- 先悬停，再前进，接近障碍物后绕行，最后 hold

任务：

- [x] 新增 `mavlink_obstacle_mission_controller`
- [x] 订阅 `/scan_features`
- [x] 订阅 `/external_nav/status`
- [x] 订阅 `/imu/status`
- [x] 读取 MAVLink heartbeat、ACK、local position、EKF 状态
- [x] 状态机实现 `WAIT_READY`
- [x] 状态机实现 `GUIDED`
- [x] 状态机实现 `ARM`
- [x] 状态机实现 `TAKEOFF`
- [x] 状态机实现 `HOVER_SETTLE`
- [x] 状态机实现 `FORWARD`
- [x] 状态机实现 `AVOID`
- [x] 状态机实现 `PASS_OBSTACLE`
- [x] 状态机实现 `RETURN_TRACK`
- [x] 状态机实现 `FINAL_HOLD`
- [x] 发布 `/navlab/mission/status`
- [ ] 输出 summary metrics：hover span、forward span、min clearance、lateral detour、final drift

第一版绕障策略：

- [x] 障碍物固定在 `x=5`
- [x] `front_min` 小于阈值后固定向 `+Y` 横移
- [x] `y >= 2.6` 后沿 `+X` 越过障碍物
- [x] `x >= 6.5` 后回到 `y=0`
- [x] 全程保持 takeoff altitude

验收：

- [ ] UAV 能 arm
- [ ] UAV 能 takeoff
- [ ] UAV 能稳定 hover
- [ ] UAV 能沿 `+X` 前进
- [ ] `/scan_features.front_min` 逐渐减小
- [ ] 触发 obstacle_detected
- [ ] controller 发送 avoidance setpoint
- [ ] 本地位置出现横向绕行动作
- [ ] 绕过障碍物后进入 final hold

## 8. P5：Rosbag、artifact 和 Foxglove 回放绑定

目标：

- NavLab acceptance 每次都留下可复盘的 rosbag
- 回放时能判断问题出在 Gazebo、SLAM、ExternalNav、MAVLink 控制还是 SITL

任务：

- [x] 新增 NavLab rosbag topic profile：`profiles/navlab-rosbag-topics.txt`
- [x] acceptance 脚本使用该 profile 启动 `ros2 bag record`
- [x] 默认输出 MCAP
- [x] artifact 目录固定为 `artifacts/ros/navlab_companion_sitl_gazebo/<timestamp>/`
- [x] 每次运行生成 `summary.json`
- [x] 每次运行生成 `foxglove_notes.md`
- [x] 记录 SITL、router、companion、Gazebo 日志
- [x] 校验 rosbag metadata 中 required topics 都存在
- [x] 提供 Foxglove 回放命令或说明
- [x] acceptance 禁用 mission autostart，先启动 rosbag，再启动 mission controller，保证 rosbag 覆盖完整任务

必录 topic：

```text
/scan
/scan_features
/scan_nearest_point
/sim/markers
/sim/uav_pose
/sim/log
/imu/data
/imu/status
/tf
/odom
/cartographer/status
/external_nav/odom
/external_nav/status
/mavlink_external_nav/status
/navlab/mavlink/status
/navlab/pose_mirror/status
/navlab/mission/status
```

验收：

- [ ] rosbag 中 required topics 全部存在
- [ ] Foxglove 能看到 UAV 位姿、marker、scan、front_min、mission status
- [ ] Foxglove 能看到 wait_ready、guided、arm、takeoff、hover、forward、avoid、final_hold 状态变化
- [ ] rosbag_profile.ok == true

## 9. P6：NavLab acceptance

目标：

- 形成最终一条命令判断 NavLab 是否通过

建议入口：

```bash
just navlab-acceptance 90
```

当前实现：

- [x] 新增 `scripts/navlab_companion_gazebo_acceptance.sh`
- [x] 新增 `just navlab-acceptance`
- [x] 新增 `scripts/navlab_companion_doctor.sh`
- [x] 新增 `just navlab-companion-doctor`
- [x] acceptance 默认 artifact 路径为 `artifacts/ros/navlab_companion_sitl_gazebo/<timestamp>/`
- [x] acceptance 生成 `summary.json`
- [x] acceptance 生成 `foxglove_notes.md`
- [x] acceptance 调用 `scripts/stage1_validate_rosbag_profile.py` 校验 `profiles/navlab-rosbag-topics.txt`
- [x] acceptance 在 rosbag 开始后才启动 `mavlink_obstacle_mission_controller`
- [ ] acceptance 尚未完成实跑通过

summary 最低字段：

```json
{
  "ok": true,
  "gps_free": true,
  "rosbag_started_before_mission": true,
  "rosbag_covers_full_mission": true,
  "companion_ready": true,
  "sitl_heartbeat": true,
  "imu_source": "fcu_mavlink_or_dds",
  "scan_fresh": true,
  "external_nav_healthy": true,
  "gazebo_pose_mirror_ok": true,
  "hover_ok": true,
  "forward_progress_ok": true,
  "obstacle_detected": true,
  "avoidance_setpoint_sent": true,
  "lateral_detour_ok": true,
  "final_hold_ok": true,
  "rosbag_profile_ok": true
}
```

通过标准：

- [ ] `.ok == true`
- [ ] `.gps_free == true`
- [ ] `.companion_ready == true`
- [ ] `.external_nav_healthy == true`
- [ ] `.gazebo_pose_mirror_ok == true`
- [ ] `.hover_ok == true`
- [ ] `.forward_progress_ok == true`
- [ ] `.obstacle_detected == true`
- [ ] `.avoidance_setpoint_sent == true`
- [ ] `.lateral_detour_ok == true`
- [ ] `.final_hold_ok == true`
- [ ] `.rosbag_profile_ok == true`

失败判定：

- [ ] 如果只完成 hover 和 forward，但没有绕障，NavLab 不算完成
- [ ] 如果 Gazebo 仍由 planner 直接 set_pose，NavLab 不算完成
- [ ] 如果 SLAM 输入使用了 `LOCAL_POSITION_NED`，NavLab 不算完成
- [ ] 如果没有 rosbag 或 Foxglove 无法回放，NavLab 不算完成
