# 阶段 1 TODO：ArduPilot SITL ExternalNav 闭环

## 1. 目标

这份 TODO 对应 `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`，用于把阶段 1 拆成可逐项验收的 P0 / P1 / P2 任务。

阶段 1 的核心原则：

- 先验证 `ArduPilot SITL` 能消费 `ExternalNav`
- 再把 fake `/odom` 替换成 `Cartographer 2D` 的 SLAM `/odom`
- 最后做最小 `hold/hover` 和低速平移
- 本阶段不引入世界模型，不做真实试飞，不把复杂航点任务作为验收门槛

一句话定义：

**先证明飞控接收链路，再证明 SLAM 定位链路，最后证明最小飞控闭环。**

## 2. 当前状态总览

- [x] `SITL` 基线启动流程固定
- [x] `fake_external_nav` 或 sim pose adapter 输出 `/odom`
- [x] `external_nav_bridge` 可消费 `/odom` 并输出 `/external_nav/odom`、`/external_nav/status`、`/ap/tf`
- [x] `ArduPilot SITL` 能稳定消费 fake ExternalNav
- [x] `imu_bridge` 输出 `/imu/data`
- [x] `cartographer_indoor` 输出 Cartographer-compatible `/odom`
- [x] Cartographer-compatible `/odom` 替换 fake `/odom` 后 ExternalNav 仍健康
- [x] synthetic TF-backed 最小 `hold/hover` 验证通过
- [x] 小范围 `local position / velocity setpoint` 平移验证通过
- [x] 阶段 1 rosbag topic 收口
- [x] 阶段 1 artifact 目录和运行摘要收口
- [x] MAVLink `ODOMETRY` 扩展入口和字段映射收口
- [x] 高度估计接口和 bridge gating 预留
- [ ] Gazebo `/scan` + real/SITL IMU + real `cartographer_ros` feedback acceptance 通过

## 3. P0：SITL + fake ExternalNav 最小接收链路

目标：

- 不依赖真实 SLAM，先证明 ArduPilot SITL 侧 ExternalNav 接收链路是通的
- 把 SITL、bridge、topic、frame、日志和 rosbag 的最小边界固定下来
- 避免把 SLAM 误差和飞控接入问题混在一起排查

### P0.1 固定 `ArduPilot SITL` 基线启动

任务：

- [x] 固定 ArduPilot SITL 版本和镜像/工作区来源
- [x] 固定 DDS 或 MAVLink 接入路径，阶段 1 优先选一条主路径
- [x] 固定无 GPS / ExternalNav 相关参数文件
- [x] 固定启动命令和必要环境变量
- [x] 明确 ArduPilot 侧状态观测方式

交付物：

- 固定环境参数：`profiles/stage1-sitl-baseline.env`
- 固定 ArduPilot 参数：`profiles/stage1-sitl-external-nav.parm`
- 固定启动命令：`just stage1-sitl-up`
- 状态检查命令：`just stage1-sitl-doctor`
- 日志查看命令：`just stage1-sitl-logs`
- 停止命令：`just stage1-sitl-down`

当前 P0.1 固定选择：

- SITL 镜像：`remote-sitl-lab/ardupilot-sitl:stage1-f10500ae45aa`
- SITL 当前本机镜像 ID：`sha256:f10500ae45aabaf6b6e44174a8a66a10a07f8c410b40817bd75e5a8d9184dae7`
- Router 镜像：`remote-sitl-lab/mavlink-router:stage1-4ee567d97525`
- Router 当前本机镜像 ID：`sha256:4ee567d975255b1440e20d6713b84bac79e04afee79c86088bed12bb07812972`
- 主接入路径：`SITL serial0 -> mavlink-router`
- P0.1 暂不启用 DDS；DDS 留到后续 IMU / ExternalNav ROS2 链路
- SITL 模型：`quad`
- SITL speedup：`1`
- SITL instance：`0`
- SITL upstream：`mavlink-router:14550`
- SITL extra args：`--defaults /workspace/profiles/stage1-sitl-external-nav.parm`
- Router listen：`0.0.0.0:14550`
- Router downstream：`127.0.0.1:14552`
- ExternalNav 参数基线：`VISO_TYPE=1`、`EK3_SRC1_POSXY=6`、`EK3_SRC1_VELXY=6`、`EK3_SRC1_POSZ=1`、`EK3_SRC1_VELZ=0`、`EK3_SRC1_YAW=6`

说明：

- 当前镜像没有可用 `RepoDigest`，所以 P0.1 先用本机镜像 ID 派生不可变阶段 tag，并把 compose 默认 image 从 `latest` 改为上述阶段 tag；后续如果镜像仓库暴露 digest，再把 compose image 改成 `repo@sha256:...`。
- `just stage1-sitl-doctor` 的 Runtime 表仍会显示 `lab_env/config.toml` 里的 router 默认配置；实际容器参数以 Service 明细里的 `session id`、`upstream endpoint` 和 `downstream endpoints` 为准。

P0.1 操作：

```bash
just stage1-sitl-up
just stage1-sitl-doctor
just stage1-sitl-logs
just stage1-sitl-down
```

验收标准：

- [x] SITL 能连续启动两次且结果一致
- [x] 不依赖临时手工修环境变量
- [x] 能看到飞控状态、参数状态和 ExternalNav 相关入口

已验证：

- 第一次启动：`mavlink-router` 和 `sitl` 均为 `running / healthy`
- 第二次启动：`mavlink-router` 和 `sitl` 均为 `running / healthy`
- SITL 命令稳定为 `/usr/local/bin/arducopter --model quad --speedup 1 --instance 0 --serial0 udpclient:mavlink-router:14550 --defaults /workspace/profiles/stage1-sitl-external-nav.parm`
- Router 命令稳定为 `mavlink-routerd -t 0 -e 127.0.0.1:14552 0.0.0.0:14550`
- 参数文件加载日志稳定出现：`Loaded defaults from @ROMFS/default_params/copter.parm,/workspace/profiles/stage1-sitl-external-nav.parm`
- 日志路径：`artifacts/sessions/stage1_sitl_baseline/sitl.log`、`artifacts/sessions/stage1_sitl_baseline/router.log`

### P0.2 实现 `fake_external_nav` 或 sim pose adapter

任务：

- [x] 提供确定性的 `/odom` 发布入口
- [x] 支持静止模式：固定 `x=0, y=0, yaw=0`
- [x] 支持慢速直线模式：低速改变 `x`
- [x] 支持小范围 yaw 模式：只改变 yaw
- [x] 支持可配置发布频率

交付物：

- `ros2_ws/src/localization/fake_external_nav`
- `fake_external_nav_node`
- `/odom`
- `/fake_external_nav/status`
- `ros2 launch fake_external_nav fake_external_nav.launch.py mode:=static|line|yaw`
- `just stage1-fake-nav-build`
- `just stage1-fake-nav-smoke static|line|yaw`

验收标准：

- [x] `/odom` 时间戳连续
- [x] 发布频率稳定
- [x] pose 与 twist 字段一致
- [x] frame 命名稳定
- [x] 不依赖 Cartographer 或真实雷达

已验证：

- `fake_external_nav` 单包编译通过
- `fake_external_nav`、`imu_bridge`、`cartographer_indoor`、`external_nav_bridge`、`indoor_bringup` 组合编译通过
- `line` 模式发布 `/odom`，示例字段：`frame_id=odom`、`child_frame_id=base_link`、`twist.linear.x=0.05`
- `static` 模式发布 `/fake_external_nav/status`，示例状态：`mode=static ... x=0 yaw=0`
- `yaw` 模式发布 `/fake_external_nav/status`，示例状态：`mode=yaw ... yaw=0.130009`
- `indoor_bringup launch_fake_external_nav:=true fake_external_nav_mode:=line` 可发布 `/odom` 和 `/fake_external_nav/status`

后续可选：

- 如果 P0.3/P0.4 需要复用阶段 0 轨迹，再补 `/sim/uav_pose -> /odom` adapter；当前 P0.2 先用确定性 fake odom 隔离飞控接收链路。

### P0.3 收口 `external_nav_bridge` 最小链路

任务：

- [x] 消费 `/odom`
- [x] 输出 `/external_nav/odom`
- [x] 输出 `/external_nav/status`
- [x] 输出 `/ap/tf`
- [x] 明确 ENU/FLU 到 ArduPilot 侧坐标语义的转换位置
- [x] 在 status 中暴露输入频率、输入 age、frame、质量状态

交付物：

- `external_nav_bridge` 最小可运行实现
- `/external_nav/odom`
- `/external_nav/status`
- `/ap/tf`
- bridge 参数说明
- `just stage1-bridge-smoke line`

验收标准：

- [x] fake `/odom` 输入时 bridge 输出稳定
- [x] `/external_nav/status` 能显示 healthy / timeout / invalid frame 等状态
- [x] 坐标转换逻辑集中在 bridge 内部
- [x] 输出频率满足 ArduPilot ExternalNav 最低要求

当前 bridge 参数：

- 输入：`/odom`
- 输出：`/external_nav/odom`
- 状态：`/external_nav/status`
- ArduPilot DDS TF：`/ap/tf`
- 期望输入 frame：`odom -> base_link`
- 当前坐标模式：`pass_through_enu_flu`
- 最小输入频率：`4.0 Hz`
- odom 超时：`500 ms`

已验证：

- `indoor_bringup launch_fake_external_nav:=true fake_external_nav_mode:=line require_imu_for_external_nav:=false` 可驱动 bridge 输出
- `/external_nav/odom` 输出稳定，示例字段：`frame_id=external_nav`、`child_frame_id=base_link`、`twist.linear.x=0.05`
- `/external_nav/status` 输出 JSON，healthy 示例：`"state":"healthy"`、`"ready":true`、`"rate_hz":19.997`、`"frame_ok":true`
- `/ap/tf` 输出稳定，示例 transform：`odom -> base_link`
- 错误输入 frame 验证通过，status 示例：`"state":"invalid_frame"`、`"frame_ok":false`

边界：

- P0.3 当前只做 `pass_through_enu_flu`，确认坐标转换位置固定在 `external_nav_bridge`；完整 ENU/NED、FLU/FRD 转换在后续 P0.4/P1 与 ArduPilot 消费链路一起收口。

### P0.4 跑通 `SITL + fake_external_nav + external_nav_bridge`

任务：

- [x] 启动 SITL
- [x] 启动 fake `/odom`
- [x] 启动 `external_nav_bridge`
- [x] 确认 ArduPilot 侧能看到 ExternalNav 输入
- [x] 连续运行 2 到 5 分钟
- [x] 记录 rosbag 和日志

交付物：

- 一次可复现的 P0 验收记录：`just stage1-external-nav-acceptance 120`
- rosbag：`artifacts/ros/stage1_sitl_external_nav/20260528_001835/rosbag/rosbag_0.mcap`
- SITL 日志：`artifacts/ros/stage1_sitl_external_nav/20260528_001835/sitl_stack_tail.log`
- bridge 日志：`artifacts/ros/stage1_sitl_external_nav/20260528_001835/indoor_bringup.log`
- 简短 summary：`artifacts/ros/stage1_sitl_external_nav/20260528_001835/summary.json`

验收标准：

- [x] `/odom` 稳定发布
- [x] `/external_nav/status` 保持 healthy
- [x] ArduPilot 侧不持续报 ExternalNav 超时或拒收
- [x] 连续运行 2 到 5 分钟不崩溃
- [x] 出错时能从日志判断是 SITL、fake odom 还是 bridge 问题

当前 P0.4 固定选择：

- MAVLink ExternalNav sender：`lab_env.sim.nodes.mavlink_external_nav_sender`
- MAVLink observer：`lab_env.sim.nodes.mavlink_stage1_observer`
- sender 输入：`/external_nav/odom`
- sender 输出：MAVLink v2 `ODOMETRY`
- sender endpoint：`tcp:sitl:5762`
- observer endpoint：`tcp:sitl:5763`
- `ODOMETRY` 频率：`20 Hz`
- MAVLink frame：`MAV_FRAME_LOCAL_FRD` + `MAV_FRAME_BODY_FRD`
- ArduPilot 参数关键修正：`VISO_TYPE=1`，即通用 MAVLink Visual Odometry；`VISO_TYPE=3` 是 VOXL/ModalAI 路径，不适合作为本仓库 P0.4 通用 MAVLink 验收参数。

决策记录：

- 试过将 sender 发到 `udpout:mavlink-router:14550`，router TCP 侧可观察到 `ODOMETRY`，但 ArduPilot 未输出 `LOCAL_POSITION_NED`，并持续出现 `PreArm: VisOdom: not healthy`。
- 对照实验将 sender 直连 `tcp:sitl:5762` 后，ArduPilot 输出 `LOCAL_POSITION_NED`，EKF flags 为 `831`，并且不再出现 `VisOdom: not healthy`。
- 因此 P0.4 验收固定为 `SITL TCP serial` 直连 MAVLink 注入；`mavlink-router` 暂保留为 SITL 基线服务和后续遥测入口，router 注入 ExternalNav 留到后续 P1/P2 再单独收口。

已验证：

- 30 秒脚本化验收通过：`just stage1-external-nav-acceptance 30`
- 120 秒脚本化验收通过：`just stage1-external-nav-acceptance 120`
- 120 秒 summary：`ok=true`
- 120 秒 `ODOMETRY` 计数：`2400`
- 120 秒 `LOCAL_POSITION_NED` 计数：`478`
- 120 秒 `EKF_STATUS_REPORT` 计数：`478`
- 120 秒本地位置变化：`x=0.2557609379 -> 6.2180213928`，跨度 `5.9623m`
- 120 秒 `latest_visodom_unhealthy_sec=null`
- 120 秒只剩已知非 P0.4 阻塞项：`PreArm: Param storage failed`

### P0 完成标准

P0 完成后，应该能明确回答：

- SITL 是否能启动
- 本仓库是否能向 ArduPilot 提供 ExternalNav
- ArduPilot 是否能持续消费这路输入
- 坐标系、时间戳和频率是否有基本观测手段

## 4. P1：接入 IMU、SLAM `/odom` 和最小 hold/hover

目标：

- 把 P0 的 fake `/odom` 替换为真实定位链路
- 用 `/scan + /imu/data -> cartographer_indoor -> /odom` 形成 SLAM odom
- 在 ExternalNav 健康的前提下做最小 `hold/hover`

### P1.0 固定 P1 入口基线

任务：

- [x] 固定 P1 topic contract
- [x] 固定 IMU source fallback 策略
- [x] 固定 P1 baseline 启动命令
- [x] 固定 P1 baseline rosbag topic
- [x] 用 placeholder IMU 证明 `external_nav_bridge` 在 `require_imu_for_external_nav=true` 时仍可 healthy

交付物：

- P1 baseline 命令：`just stage1-p1-baseline-smoke 15`
- P1 baseline 脚本：`scripts/stage1_p1_baseline_smoke.sh`
- P1 baseline rosbag：`artifacts/ros/stage1_p1_baseline/20260528_181557/rosbag/rosbag_0.mcap`
- P1 baseline summary：`artifacts/ros/stage1_p1_baseline/20260528_181557/summary.json`

当前 P1 topic contract：

- `/scan`：`sensor_msgs/msg/LaserScan`，后续由 Gazebo scan bridge 或真实雷达提供
- `/imu/data`：`sensor_msgs/msg/Imu`，由 `imu_bridge` 统一输出
- `/imu/status`：`std_msgs/msg/String`，由 `imu_bridge` 输出
- `/odom`：`nav_msgs/msg/Odometry`，P1.2 后由 `cartographer_indoor` 输出；P1.0/P1.1 仍可用 fake `/odom` 隔离 IMU 链路
- `/cartographer/status`：`std_msgs/msg/String`，由 `cartographer_indoor` 输出
- `/external_nav/odom`：`nav_msgs/msg/Odometry`，由 `external_nav_bridge` 输出
- `/external_nav/status`：`std_msgs/msg/String`，由 `external_nav_bridge` 输出
- `/ap/tf`：`tf2_msgs/msg/TFMessage`，由 `external_nav_bridge` 输出
- `/mavlink_external_nav/status`：`std_msgs/msg/String`，MAVLink ExternalNav sender 状态，仅在 SITL 注入验收时需要
- `/height/estimate`：`std_msgs/msg/String` JSON，高度估计预留输入，阶段 1 默认不要求出现

当前 IMU source fallback 策略：

- 第一优先级：ArduPilot DDS `/ap/imu/experimental/data`
- 第二优先级：MAVROS `/mavros/imu/data_raw`
- 第三优先级：`imu_bridge` placeholder，仅用于 P1.0 smoke 和无真实 IMU 源时的 topic contract 验证

验收标准：

- [x] `/imu/data` 有消息
- [x] `/imu/status` 可观测
- [x] `/external_nav/status` 在 `require_imu_for_external_nav=true` 时为 healthy
- [x] rosbag 包含 `/imu/data`、`/imu/status`、`/odom`、`/external_nav/odom`、`/external_nav/status`、`/ap/tf`
- [x] P1.0 不要求 `/scan` 和真实 SLAM `/odom`，这两个留给 P1.1/P1.2

已验证：

- `just stage1-p1-baseline-smoke 15` 通过
- summary：`ok=true`
- `/imu/status` 示例：`publishing_placeholder_fcu_imu`
- `/external_nav/status` 示例：`"state":"healthy"`、`"ready":true`、`"imu":{"required":true,"present":true,"fresh":true}`
- rosbag 时长：约 `19.4s`
- rosbag 消息总数：`973`
- rosbag topic：`/imu/data`、`/imu/status`、`/odom`、`/fake_external_nav/status`、`/external_nav/odom`、`/external_nav/status`、`/ap/tf`

边界：

- P1.0 使用 `imu_bridge` placeholder，只验证 P1 topic contract 和 `external_nav_bridge` 的 IMU gating 行为。
- 真实 ArduPilot DDS IMU 或 MAVROS IMU 仍属于 P1.1；不要把 P1.0 的 placeholder 验收等同于真实 IMU 接入完成。

### P1.1 接入 `imu_bridge`

任务：

- [x] 订阅 ArduPilot ROS2 DDS IMU，例如 `/ap/imu/experimental/data`
- [x] 或支持 MAVROS IMU fallback，例如 `/mavros/imu/data_raw`
- [x] 统一输出 `/imu/data`
- [x] 输出 `/imu/status`
- [x] 固定 frame 和时间戳策略

交付物：

- `imu_bridge` 节点
- `/imu/data`
- `/imu/status`
- IMU source 参数说明
- P1.1 smoke 命令：`just stage1-imu-bridge-smoke 8`
- P1.1 smoke 脚本：`scripts/stage1_imu_bridge_smoke.sh`
- P1.1 smoke summary：`artifacts/ros/stage1_imu_bridge/20260528_182223/summary.json`

验收标准：

- [x] `/imu/data` 频率稳定
- [x] `/imu/status` 可观测
- [x] 切换 IMU 来源不影响 `cartographer_indoor` 的输入契约

当前 `imu_bridge` 参数：

- `source_mode`：`topic` 或 `placeholder`
- `source_topic`：上游 IMU topic，默认 `/ap/imu/experimental/data`
- `source_label`：状态中显示的来源标签，默认 `ardupilot_dds`
- `output_frame_id`：输入 frame 为空或禁用输入 frame 时的 fallback frame，默认 `imu_link`
- `use_input_frame_id`：是否保留输入 frame，默认 `true`
- `replace_zero_timestamp`：是否把零时间戳替换为当前 ROS 时间，默认 `true`
- `input_timeout_ms`：输入超时阈值，默认 `500`
- `min_input_rate_hz`：最低输入频率，默认 `4.0`

当前 `/imu/status` JSON 字段：

- `state`：`waiting_for_fcu_imu_source`、`streaming_fcu_imu`、`low_rate_fcu_imu_source`、`stale_fcu_imu_source`、`publishing_placeholder_fcu_imu`
- `ready`：输入存在、未超时且频率满足阈值
- `source`：`mode`、`label`、`topic`
- `input`：`present`、`fresh`、`age_ms`、`rate_hz`、`rate_ok`、`min_rate_hz`、`frame_id`、`count`
- `output`：`topic`、`frame_id`、`fallback_frame_id`、`use_input_frame_id`、`replace_zero_timestamp`、`last_timestamp_replaced`、`timestamp_replaced_count`

已验证：

- `just stage1-imu-bridge-smoke 8` 通过
- `ardupilot_dds` case：输入 `/ap/imu/experimental/data`，状态 `streaming_fcu_imu`，`ready=true`，`rate_hz=19.996`，`rate_ok=true`
- `mavros_raw` case：输入 `/mavros/imu/data_raw`，状态 `streaming_fcu_imu`，`ready=true`，`rate_hz=20.001`，`rate_ok=true`
- 两个 case 都输出 `/imu/data` 和 `/imu/status`
- 两个 case 都保留输入 frame：`fcu_imu`
- 两个 case 都验证零时间戳替换：`last_timestamp_replaced=true`
- `ardupilot_dds` rosbag：`/imu/data=208`、`/imu/status=21`
- `mavros_raw` rosbag：`/imu/data=209`、`/imu/status=20`

边界：

- 当前 P1.1 smoke 使用模拟上游 `sensor_msgs/msg/Imu` topic 验证桥接行为；当前 SITL 栈没有真实 ArduPilot DDS IMU 或 MAVROS 进程。
- 真正连接硬件/飞控侧 IMU 时，只应替换 `source_topic/source_label`，下游 `cartographer_indoor` 仍只消费 `/imu/data`。

### P1.2 接入 `cartographer_indoor`

任务：

- [x] 消费 `/scan`
- [x] 消费 `/imu/data`
- [x] 运行 Cartographer 2D 或等价 SLAM
- [x] 输出 `/odom`
- [x] 输出 `/cartographer/status`

交付物：

- `cartographer_indoor` launch/config
- `/odom`
- `/cartographer/status`
- SLAM 参数说明
- P1.2 smoke 命令：`just stage1-cartographer-smoke 12`
- P1.2 smoke 脚本：`scripts/stage1_cartographer_smoke.sh`
- P1.2 smoke summary：`artifacts/ros/stage1_cartographer_indoor/20260528_183457/summary.json`

验收标准：

- [x] 静止时 `/odom` 不明显漂移
- [x] 低速运动时 `x / y / yaw` 连续
- [x] 短时间内不出现明显跳变
- [x] 输出能直接替换 P0 的 fake `/odom`

当前 `cartographer_indoor` 参数：

- `scan_timeout_ms`：`/scan` 新鲜度阈值，默认 `500`
- `imu_timeout_ms`：`/imu/data` 新鲜度阈值，默认 `500`
- `tf_timeout_ms`：`/tf` 新鲜度阈值，默认 `500`
- `publish_placeholder_odom`：只用于 smoke fallback，默认 `false`
- `odom_source_mode`：`tf` 或 `placeholder`，默认 `tf`
- `odom_frame_id`：默认 `odom`
- `base_frame_id`：默认 `base_link`
- `tf_topic`：默认 `/tf`

当前 `/cartographer/status` JSON 字段：

- `state`：`waiting_for_scan_and_imu`、`waiting_for_scan`、`waiting_for_imu`、`waiting_for_cartographer_tf`、`publishing_tf_backed_odom`、`publishing_placeholder_odom`
- `ready`：是否已经有可输出 `/odom` 的定位结果
- `mode`：当前 odom source mode
- `scan`：`present`、`fresh`、`age_ms`、`count`
- `imu`：`present`、`fresh`、`age_ms`、`count`
- `tf`：`present`、`fresh`、`age_ms`、`count`、`topic`、`frame_id`、`child_frame_id`
- `output`：`odom_topic`、`status_topic`、`odom_count`、`last_x`、`last_y`、`last_yaw_z`

已验证：

- 当前 ROS 镜像没有 `cartographer_ros`，所以 P1.2 smoke 使用 synthetic `/scan`、`/imu/data` 和 `odom -> base_link` TF 验证 Cartographer-compatible adapter。
- `just stage1-cartographer-smoke 12` 通过
- summary：`ok=true`
- 状态：`state=publishing_tf_backed_odom`、`ready=true`
- 输入计数：`scan=329`、`imu=329`、`tf=329`
- 输出 `/odom` 计数：`odom_count=329`
- 输出位置示例：`last_x=0.836`，`last_y=0.0`，`last_yaw_z=0.167`
- rosbag topic：`/scan`、`/imu/data`、`/tf`、`/odom`、`/cartographer/status`
- rosbag 消息计数：`/scan=282`、`/imu/data=282`、`/tf=282`、`/odom=282`、`/cartographer/status=28`

边界：

- P1.2 当前验证的是 `cartographer_indoor` 的 TF-backed `/odom` adapter，可替代 P0 fake `/odom` 进入下游链路。
- 当前还没有真实 Cartographer scan matching；真实 `cartographer_ros` 后端需要镜像/依赖提供后再把 `launch_cartographer_backend:=true` 打开。
- 因此 P1.2 证明的是 `/scan + /imu/data + Cartographer-compatible TF -> /odom` 的仓库接口闭合，不证明 SLAM 质量。

### P1.3 用 SLAM `/odom` 替换 fake `/odom`

任务：

- [x] 停用 fake `/odom`
- [x] 将 `cartographer_indoor` 的 `/odom` 接入 `external_nav_bridge`
- [x] 对比 fake `/odom` 与 SLAM `/odom` 下的 bridge 状态
- [x] 记录 `/odom -> /external_nav/odom -> /ap/tf` 全链路 rosbag

交付物：

- SLAM `/odom` 接入验收命令：`just stage1-slam-external-nav-acceptance 30`
- SLAM `/odom` 接入验收脚本：`scripts/stage1_slam_external_nav_acceptance.sh`
- 验收记录目录：`artifacts/ros/stage1_slam_external_nav/20260528_195240/`
- rosbag：`artifacts/ros/stage1_slam_external_nav/20260528_195240/rosbag/rosbag_0.mcap`
- summary：`artifacts/ros/stage1_slam_external_nav/20260528_195240/summary.json`
- MAVLink summary：`artifacts/ros/stage1_slam_external_nav/20260528_195240/mavlink_summary.json`
- SITL 日志：`artifacts/ros/stage1_slam_external_nav/20260528_195240/sitl_stack_tail.log`
- bringup 日志：`artifacts/ros/stage1_slam_external_nav/20260528_195240/indoor_bringup.log`

验收标准：

- [x] `/external_nav/status` 保持 healthy
- [x] `x / y / yaw` 不出现明显跳变或发散
- [x] ArduPilot 侧 ExternalNav 不频繁超时
- [x] 出错时能区分 SLAM 输入问题和 bridge 输出问题

当前 P1.3 固定选择：

- `indoor_bringup` 禁用 fake odom：`launch_fake_external_nav:=false`
- `cartographer_indoor` 不启用 placeholder odom：`publish_placeholder_odom:=false`
- `cartographer_indoor` 当前使用 TF-backed adapter：`launch_cartographer_backend:=false`
- `/odom` 来源：synthetic `/scan` + placeholder `/imu/data` + synthetic `odom -> base_link` TF，经 `cartographer_indoor` 输出
- `external_nav_bridge` 输入：`/odom`
- MAVLink ExternalNav sender 输入：`/external_nav/odom`
- MAVLink sender endpoint：`tcp:sitl:5762`
- MAVLink observer endpoint：`tcp:sitl:5763`

已验证：

- `just stage1-slam-external-nav-acceptance 30` 通过
- summary：`ok=true`
- `cartographer_indoor` 状态：`state=publishing_tf_backed_odom`、`ready=true`
- `cartographer_indoor` 输出：`odom_count=707`、`last_x=1.813`、`last_y=0.0`、`last_yaw_z=0.18`
- `external_nav_bridge` 状态：`state=healthy`、`ready=true`、`frame_ok=true`、`rate_hz=19.636`
- IMU gating：`required=true`、`fresh=true`
- MAVLink summary：`ok=true`
- MAVLink `ODOMETRY` 计数：`600`
- MAVLink `LOCAL_POSITION_NED` 计数：`118`
- ArduPilot 本地位置跨度：`local_position_x_span_m=1.4107`
- ArduPilot ExternalNav 超时：`latest_visodom_unhealthy_sec=null`
- rosbag 时长：约 `35.0s`
- rosbag 消息总数：`2540`
- rosbag topic：`/scan`、`/imu/data`、`/imu/status`、`/tf`、`/cartographer/status`、`/odom`、`/external_nav/odom`、`/external_nav/status`、`/ap/tf`、`/mavlink_external_nav/status`
- rosbag 消息计数：`/scan=682`、`/tf=682`、`/odom=682`、`/cartographer/status=71`、`/external_nav/odom=71`、`/external_nav/status=71`、`/ap/tf=71`、`/imu/data=70`、`/imu/status=70`、`/mavlink_external_nav/status=70`

对比结论：

- P0.4 fake `/odom` 验收证明 ArduPilot 能消费确定性 fake ExternalNav，120 秒 `LOCAL_POSITION_NED=478`，`latest_visodom_unhealthy_sec=null`。
- P1.3 停用 fake `/odom` 后，`cartographer_indoor` 输出的 `/odom` 仍能让 `external_nav_bridge` 保持 healthy，并让 ArduPilot 输出 `LOCAL_POSITION_NED`。
- 这说明当前主要链路已经从 `fake_external_nav -> external_nav_bridge -> MAVLink -> SITL` 推进到 `cartographer_indoor -> external_nav_bridge -> MAVLink -> SITL`。

边界：

- P1.3 当前仍是 `cartographer_indoor` 的 TF-backed `/odom` adapter 验收，不是真实 Cartographer scan matching 质量验收。
- 当前 `/scan` 和 `odom -> base_link` TF 仍由脚本合成；后续接入 Gazebo scan bridge 或真实雷达后，需要重新跑同一条 P1.3 验收命令。
- 当前 IMU 仍使用 `imu_bridge` placeholder；真实 FCU IMU / MAVROS IMU 接入后，需要复跑 P1.3 判断时间戳、frame 和频率是否仍满足 bridge gating。

### P1.4 最小 `hold/hover` 验证

任务：

- [x] 配置 ArduPilot 无 GPS / ExternalNav 相关 EKF 参数
- [x] 进入预期飞控模式
- [x] 起飞或进入受控 hold 测试条件
- [x] 不发送横向运动指令，观察 `x / y / yaw`
- [x] 记录 ArduPilot 状态、ExternalNav 状态和 rosbag

交付物：

- `hold/hover` 验收命令：`just stage1-hold-hover-acceptance 45`
- `hold/hover` 验收脚本：`scripts/stage1_hold_hover_acceptance.sh`
- MAVLink 控制器：`lab_env/sim/nodes/mavlink_hold_hover_controller.py`
- 验收记录目录：`artifacts/ros/stage1_hold_hover/20260528_201835/`
- rosbag：`artifacts/ros/stage1_hold_hover/20260528_201835/rosbag/rosbag_0.mcap`
- summary：`artifacts/ros/stage1_hold_hover/20260528_201835/summary.json`
- 控制器 summary：`artifacts/ros/stage1_hold_hover/20260528_201835/hold_hover_summary.json`
- SITL 日志：`artifacts/ros/stage1_hold_hover/20260528_201835/sitl_stack_tail.log`
- bringup 日志：`artifacts/ros/stage1_hold_hover/20260528_201835/indoor_bringup.log`

验收标准：

- [x] ExternalNav 不超时
- [x] `x / y / yaw` 不明显发散
- [x] 日志中没有持续 EKF 失效或定位拒收
- [x] 停止测试后能复盘定位、bridge 和飞控状态

当前 P1.4 固定选择：

- 飞控模式：`GUIDED`，ArduCopter custom mode `4`
- 起飞高度：`0.8m`
- 最小 airborne 判定：`0.25m`
- 最大允许 hold 漂移：`x/y <= 0.35m`，`yaw <= 0.35rad`
- `ARMING_CHECK`：运行时由控制器临时设为 `0`，仅用于 SITL 验收
- 不发送横向移动 setpoint；控制器只负责切模式、解锁、起飞和状态观测
- P1.4 使用静态 synthetic `odom -> base_link` TF，不再像 P1.3 一样让 TF 沿 `x` 方向主动移动
- SITL 工作目录固定到 `artifacts/sessions/<SESSION_ID>/sitl_work`，避免 ArduPilot 因容器默认目录不可写产生 `Logging failed` 和 `Param storage failed`

已验证：

- `just stage1-hold-hover-acceptance 45` 通过
- summary：`ok=true`
- `cartographer_indoor` 状态：`state=publishing_tf_backed_odom`、`ready=true`、`last_x=0.0`、`last_y=0.0`、`last_yaw_z=0.0`
- `external_nav_bridge` 状态：`state=healthy`、`ready=true`、`frame_ok=true`、`rate_hz=19.529`
- 飞控模式：`expected_mode_seen=true`，`last_mode_number=4`
- 解锁：`armed_seen=true`
- 起飞：`takeoff_command_sent=true`，`airborne_seen=true`
- 高度：`max_relative_alt_m=0.897`，`min_local_z_m=-0.8824`
- hold 时长：`39.497s`
- hold 漂移：`hold_x_span_m=0.0016`，`hold_y_span_m=0.0020`，`hold_yaw_span_rad=0.0002`
- ArduPilot ExternalNav 超时：`latest_visodom_unhealthy_sec=null`
- EKF flags：`831`
- MAVLink 计数：`ODOMETRY=900`、`LOCAL_POSITION_NED=404`、`EKF_STATUS_REPORT=178`
- rosbag 时长：约 `50.4s`
- rosbag 消息总数：`3674`
- rosbag topic：`/scan`、`/imu/data`、`/imu/status`、`/tf`、`/cartographer/status`、`/odom`、`/external_nav/odom`、`/external_nav/status`、`/ap/tf`、`/mavlink_external_nav/status`
- rosbag 消息计数：`/scan=989`、`/tf=989`、`/odom=989`、`/cartographer/status=101`、`/external_nav/odom=101`、`/external_nav/status=101`、`/ap/tf=101`、`/imu/data=101`、`/imu/status=101`、`/mavlink_external_nav/status=101`

边界：

- P1.4 证明的是 SITL 中基于 ExternalNav 的最小 `GUIDED takeoff + hold/hover`，不是复杂航点飞行。
- 当前 SLAM 输入仍是 TF-backed adapter + synthetic scan/tf，不是真实 Cartographer scan matching。
- 当前 `ARMING_CHECK=0` 是 SITL 验收开关；真实机或硬件在环不能沿用这个安全边界。

### P1.real-feedback `Gazebo /scan + real/SITL IMU -> cartographer_ros -> ExternalNav -> hold/hover`

任务：

- [ ] 启动 Gazebo 和 scan bridge，使用 Gazebo `/scan`，不再使用 synthetic `/scan`
- [ ] 使用 real/SITL IMU topic 驱动 `imu_bridge`，不再使用 placeholder IMU 作为完成证据
- [ ] 启动真实 `cartographer_ros` 后端，输出 Cartographer TF
- [ ] 由 `cartographer_indoor` 将真实 Cartographer TF 转为 `/odom`
- [ ] 复跑 ExternalNav 注入和最小 `hold/hover`

交付物：

- real-feedback 验收命令：`just stage1-real-feedback-acceptance 60`
- real-feedback 验收脚本：`scripts/stage1_real_feedback_acceptance.sh`
- rosbag topic profile：`profiles/stage1-rosbag-topics.txt`
- 验收目录格式：`artifacts/ros/stage1_real_feedback/stage1_sitl_external_nav/<YYYYMMDD_HHMMSS>/`

验收标准：

- [ ] `cartographer_ros` 后端可用，且 `/cartographer/status.state=publishing_tf_backed_odom`
- [ ] `/imu/status.state=streaming_fcu_imu`，且 `ready=true`
- [ ] `/external_nav/status.state=healthy`，且 IMU gating 为 fresh
- [ ] ArduPilot ExternalNav 不持续超时，`hold/hover` 不明显发散
- [ ] rosbag 可复盘 `/scan`、`/imu/data`、`/odom`、`/external_nav/odom`、`/external_nav/status` 和控制状态

当前状态：

- 验收入口已增加，但当前仓库镜像历史记录显示 `world-model/sim-python:latest` 还没有 `cartographer_ros` 后端。
- 2026-05-30 以 `just stage1-real-feedback-acceptance 5` 做入口 smoke，脚本按预期生成失败 artifact：`artifacts/ros/stage1_real_feedback/stage1_sitl_external_nav/20260530_084210/summary.json`，blocker 为 `cartographer_ros is not installed in world-model/sim-python:latest`。
- 该 gate 是阶段 1 从 “synthetic adapter acceptance” 升级为 “真实 SLAM/IMU feedback acceptance” 的必过项。
- 通过该 gate 后，应复跑 P1.3/P1.4/P2.1，确认真实反馈链路下 ExternalNav、hold 和低速 setpoint 仍健康。

### P1 完成标准

P1 完成后，应该能明确回答：

- `/scan + /imu/data` 是否能产生可用 `/odom`
- SLAM `/odom` 是否能稳定进入 `external_nav_bridge`
- ArduPilot 是否能基于 ExternalNav 做最小位置保持：可以，P1.4 已通过 `GUIDED takeoff + hold/hover`
- 当前主要问题是否已经从“接不上”转为“真实 SLAM/IMU 反馈质量验证”

## 5. P2：低速平移、日志录包和工程化收口

目标：

- 在 P1 的 hold/hover 成立后，扩展到小范围移动
- 补齐阶段 1 的诊断、录包、artifact 和扩展接口
- 为阶段 2 的真实场景模型仿真准备输入输出边界

### P2.1 小范围 `local position / velocity setpoint` 平移

任务：

- [x] 定义高层 setpoint 输入方式
- [x] 支持小范围前后/左右速度或位置 setpoint
- [x] 支持停止指令后回到 hold
- [x] 明确控制链路经过安全边界，不绕过飞控

交付物：

- setpoint 验收命令：`just stage1-local-setpoint-acceptance 70`
- setpoint 验收脚本：`scripts/stage1_local_setpoint_acceptance.sh`
- MAVLink setpoint 控制器：`lab_env/sim/nodes/mavlink_local_setpoint_controller.py`
- 验收记录目录：`artifacts/ros/stage1_local_setpoint/20260528_214852/`
- rosbag：`artifacts/ros/stage1_local_setpoint/20260528_214852/rosbag/rosbag_0.mcap`
- summary：`artifacts/ros/stage1_local_setpoint/20260528_214852/summary.json`
- 控制器 summary：`artifacts/ros/stage1_local_setpoint/20260528_214852/local_setpoint_summary.json`
- rosbag profile summary：`artifacts/ros/stage1_local_setpoint/20260528_214852/rosbag_profile_summary.json`
- SITL 日志：`artifacts/ros/stage1_local_setpoint/20260528_214852/sitl_stack_tail.log`
- bringup 日志：`artifacts/ros/stage1_local_setpoint/20260528_214852/indoor_bringup.log`

验收标准：

- [x] 能执行小范围低速移动
- [x] 停止指令后能回到稳定 hold
- [x] ExternalNav 和飞控状态不持续报错

当前 P2.1 固定选择：

- 飞控模式：`GUIDED`，ArduCopter custom mode `4`
- 起飞高度：`0.8m`
- setpoint 类型：MAVLink `SET_POSITION_TARGET_LOCAL_NED`
- setpoint 坐标系：`MAV_FRAME_LOCAL_NED`
- 水平速度：`vx=0.1m/s`，`vy=0.0m/s`
- z 方向：setpoint 内给 `z=-0.8m` 作为高度保持目标
- 移动窗口：`5s`
- 停止/hold 窗口：`10s`
- 最小水平位移验收：`>=0.25m`
- 停止后最大漂移验收：`<=0.12m`
- `ARMING_CHECK`：运行时由控制器临时设为 `0`，仅用于 SITL 验收
- P2.1 仍经过 ArduPilot：高层只发 MAVLink setpoint，不直接改 Gazebo 模型 pose

已验证：

- `just stage1-local-setpoint-acceptance 70` 通过
- summary：`ok=true`
- `cartographer_indoor` 状态：`state=publishing_tf_backed_odom`、`ready=true`、`last_x=0.5`
- `external_nav_bridge` 状态：`state=healthy`、`ready=true`、`frame_ok=true`、`rate_hz=19.655`
- 飞控模式：`expected_mode_seen=true`，`last_mode_number=4`
- 解锁：`armed_seen=true`
- 起飞：`takeoff_command_sent=true`，`airborne_seen=true`
- setpoint：`POSITION_TARGET_LOCAL_NED=349`
- setpoint 发送计数：`setpoints_sent_count=295`，其中移动段 `23`，停止段 `272`
- 控制状态 topic：`/stage1/control/status=70`
- 水平位移：`horizontal_span_m=0.5915`
- 移动段位移：`move_span_m=0.1154`
- 停止后漂移：`stop_drift_m=0.061`
- ArduPilot ExternalNav 超时：`latest_visodom_unhealthy_sec=null`
- EKF flags：`831`
- MAVLink 计数：`ODOMETRY=1400`、`LOCAL_POSITION_NED=676`、`EKF_STATUS_REPORT=278`
- rosbag 时长：约 `75.5s`
- rosbag 消息总数：`8552`
- rosbag topic：`/scan`、`/scan_features`、`/scan_nearest_point`、`/imu/data`、`/imu/status`、`/tf`、`/cartographer/status`、`/odom`、`/external_nav/odom`、`/external_nav/status`、`/ap/tf`、`/mavlink_external_nav/status`、`/stage1/control/status`
- rosbag 消息计数：`/scan=1485`、`/scan_features=1485`、`/scan_nearest_point=1485`、`/tf=1485`、`/odom=1485`、`/cartographer/status=151`、`/external_nav/odom=151`、`/external_nav/status=151`、`/ap/tf=151`、`/imu/data=151`、`/imu/status=151`、`/mavlink_external_nav/status=151`、`/stage1/control/status=70`

边界：

- P2.1 当前验证的是低速 setpoint 控制链路和 ExternalNav 消费链路，不是真实 SLAM 闭环质量。
- 当前 `/odom` 仍来自 TF-backed synthetic 轨迹；它被用来让 ExternalNav 与 setpoint 期望轨迹对齐，后续必须替换为 Gazebo/SLAM 真实位姿反馈。
- z 方向仍不是阶段 1 的严格验收项；虽然 setpoint 中加入了 `z=-0.8m` 目标，但高度控制质量要留到高度接口和真实传感器闭环继续收口。

### P2.2 阶段 1 rosbag topic 收口

任务：

- [x] 固定阶段 1 rosbag topic 列表
- [x] 记录 `/scan`
- [x] 记录 `/scan_features`
- [x] 记录 `/scan_nearest_point`
- [x] 记录 `/imu/data`
- [x] 记录 `/imu/status`
- [x] 记录 `/tf`
- [x] 记录 `/odom`
- [x] 记录 `/cartographer/status`
- [x] 记录 `/external_nav/odom`
- [x] 记录 `/external_nav/status`
- [x] 记录 `/ap/tf`
- [x] 记录 ExternalNav 到 ArduPilot 注入状态 topic：`/mavlink_external_nav/status`
- [x] 记录 ArduPilot 控制命令或 setpoint 状态 topic：`/stage1/control/status`

交付物：

- 阶段 1 rosbag topic profile：`profiles/stage1-rosbag-topics.txt`
- rosbag profile 校验脚本：`scripts/stage1_validate_rosbag_profile.py`
- 一份可回放的 `.mcap`：`artifacts/ros/stage1_local_setpoint/20260528_214852/rosbag/rosbag_0.mcap`
- `ros2 bag info` 等价验收记录：`artifacts/ros/stage1_local_setpoint/20260528_214852/rosbag_profile_summary.json`
- requested topic 列表：`artifacts/ros/stage1_local_setpoint/20260528_214852/rosbag_requested_topics.txt`

验收标准：

- [x] 回放时能看到定位输入、SLAM 输出、ExternalNav 输出、飞控注入状态和控制 setpoint 状态
- [x] 出问题时能用 rosbag 判断故障发生在哪一层

已验证：

- `just stage1-local-setpoint-acceptance 70` 通过
- `summary.ok=true`
- `rosbag_profile.ok=true`
- required topic 缺失：`[]`
- required topic 零消息：`[]`
- recorded required topic：`/scan`、`/scan_features`、`/scan_nearest_point`、`/imu/data`、`/imu/status`、`/tf`、`/cartographer/status`、`/odom`、`/external_nav/odom`、`/external_nav/status`、`/ap/tf`、`/mavlink_external_nav/status`
- recorded optional topic：`/stage1/control/status`
- optional 预留但本次未出现：`/planner/cmd_vel`
- rosbag 消息计数：`/scan=1485`、`/scan_features=1485`、`/scan_nearest_point=1485`、`/tf=1485`、`/odom=1485`、`/cartographer/status=151`、`/external_nav/odom=151`、`/external_nav/status=151`、`/ap/tf=151`、`/imu/data=151`、`/imu/status=151`、`/mavlink_external_nav/status=151`、`/stage1/control/status=70`

边界：

- `/stage1/control/status` 记录的是阶段 1 控制器观测到的 `GUIDED/armed/airborne/setpoint_count` 等状态，不是 ArduPilot 全量 telemetry dump。
- `/planner/cmd_vel` 是阶段 2 世界模型/规划器接口预留；阶段 1 不要求出现。

### P2.3 artifact 目录和运行摘要

任务：

- [x] 固定 artifact 路径格式
- [x] 保存 SITL 日志
- [x] 保存 bridge 日志
- [x] 保存运行配置
- [x] 保存 summary

固定路径：

```text
artifacts/ros/<SESSION_ID>/stage1_sitl_external_nav/<YYYYMMDD_HHMMSS>/
```

目录固定包含：

- `sitl.log`
- `external_nav_bridge.log`
- `rosbag/rosbag_0.mcap`
- `run_config.toml`
- `summary.md`
- `summary.json`
- `rosbag_profile_summary.json`

交付物：

- artifact finalize 脚本：`scripts/stage1_finalize_artifact.py`
- 已接入脚本：`scripts/stage1_external_nav_acceptance.sh`
- 已接入脚本：`scripts/stage1_slam_external_nav_acceptance.sh`
- 已接入脚本：`scripts/stage1_hold_hover_acceptance.sh`
- 已接入脚本：`scripts/stage1_local_setpoint_acceptance.sh`
- 最新 P2 验收目录：`artifacts/ros/stage1_local_setpoint/stage1_sitl_external_nav/20260528_220419/`

验收标准：

- [x] 每次阶段 1 验收都能定位到独立 artifact 目录
- [x] 目录内能直接判断使用的是 fake odom 还是 SLAM odom
- [x] summary 能说明本次是否通过 P0 / P1 / P2 的哪一项

已验证：

- `just stage1-local-setpoint-acceptance 70` 通过
- 新路径生效：`artifacts/ros/stage1_local_setpoint/stage1_sitl_external_nav/20260528_220419/`
- `summary.json`：`ok=true`
- `rosbag_profile_summary.json`：`ok=true`
- `run_config.toml` 记录 `session_id`、`run_id`、`stage_id=P2.1`、`stage_gate=P2`、`odom_source=cartographer_indoor_motion_tf_backed`、`control_mode=guided_local_velocity_setpoint`、SITL 参数和输出路径
- `summary.md` 记录 `Result=PASS`、`Stage gate=P2`、odom 来源、控制模式、rosbag profile、ExternalNav 状态和关键控制指标
- 稳定日志别名已生成：`sitl.log`、`external_nav_bridge.log`

边界：

- P2.3 已把 P0.4、P1.3、P1.4、P2.1 四条带 SITL 的阶段 1 验收脚本接入统一 artifact finalize；P1.0/P1.1/P1.2 smoke 脚本仍保留原轻量 artifact 结构。
- `summary.md` 是从本目录内的 JSON 和日志派生，权威机器可读结果仍以 `summary.json` 和 `rosbag_profile_summary.json` 为准。

### P2.4 MAVLink `ODOMETRY` 扩展预留

任务：

- [x] 明确当前阶段主路径是否使用 `/ap/tf`
- [x] 如果后续不用 DDS，定义 MAVLink `ODOMETRY` 输出入口
- [x] 明确 frame、quality、reset_counter、频率字段来源

交付物：

- MAVLink `ODOMETRY` 字段映射说明：`docs/scenarios/indoor/stage1_mavlink_odometry_mapping.md`
- MAVLink 输出入口：`lab_env.sim.nodes.mavlink_external_nav_sender`
- bridge 输出扩展点：`/external_nav/odom`
- status topic：`/mavlink_external_nav/status`
- bridge README 更新：`ros2_ws/src/bridges/external_nav_bridge/README.md`

验收标准：

- [x] DDS 路径和 MAVLink 路径不会混在同一个不清晰入口里
- [x] 后续切换 MAVLink 时不需要重写 SLAM 或 IMU 模块

当前固定结论：

- 阶段 1 已验收主路径是 MAVLink `ODOMETRY`，不是 DDS。
- `/ap/tf` 仍由 `external_nav_bridge` 输出并录包，但用途是 ROS 侧诊断和未来 DDS-shaped 入口，不作为当前 SITL 控制主路径。
- MAVLink 输出入口固定为 `/external_nav/odom -> lab_env.sim.nodes.mavlink_external_nav_sender -> MAVLink ODOMETRY -> tcp:sitl:5762`。
- `external_nav_bridge` 仍是 ROS 内部归一化边界：`/odom -> /external_nav/odom + /external_nav/status + /ap/tf`。

字段来源：

- `frame_id`：固定 `MAV_FRAME_LOCAL_FRD`
- `child_frame_id`：固定 `MAV_FRAME_BODY_FRD`
- `estimator_type`：固定 `MAV_ESTIMATOR_TYPE_VIO`
- `quality`：sender CLI `--quality`，默认 `100`
- `reset_counter`：sender CLI `--reset-counter`，阶段 1 默认 `0`
- `rate_hz`：sender CLI `--rate-hz`，阶段 1 默认 `20`
- `time_usec`：sender monotonic clock microseconds
- pose / twist / covariance：来自 `/external_nav/odom`，按 `stage1_mavlink_odometry_mapping.md` 中的 ENU/FLU 到 LOCAL_FRD/BODY_FRD 映射输出

已验证：

- `just stage1-local-setpoint-acceptance 70` 通过
- 验证目录：`artifacts/ros/stage1_local_setpoint/stage1_sitl_external_nav/20260528_222346/`
- `summary.json`：`ok=true`
- `rosbag_profile_summary.json`：`ok=true`
- `/mavlink_external_nav/status` 消息数：`151`
- MAVLink `ODOMETRY` 计数：`1400`
- `LOCAL_POSITION_NED` 计数：`664`
- rosbag 中可直接看到 `reset_counter=0`、`MAV_FRAME_LOCAL_FRD`、`MAV_FRAME_BODY_FRD`、`MAV_ESTIMATOR_TYPE_VIO` 和完整 `field_map`
- `mavlink_sender.log` 无 shutdown traceback

### P2.5 高度接口预留

任务：

- [x] 预留 `/height/estimate`
- [x] 明确 `z`
- [x] 明确 `vz`
- [x] 明确 covariance
- [x] 明确 source_type
- [x] 让 `external_nav_bridge` 能识别高度是否可用

交付物：

- `/height/estimate` 接口说明：`docs/scenarios/indoor/stage1_height_estimate_contract.md`
- `external_nav_bridge` 高度输入预留参数：`height_topic`、`height_timeout_ms`、`require_height_for_output`、`max_height_covariance`
- `external_nav_bridge` 状态扩展：`/external_nav/status.height`
- rosbag optional topic：`/height/estimate`

验收标准：

- [x] 阶段 1 不依赖高度接口通过
- [x] 后续接气压计、超声波或激光测距时，不需要改 SLAM 主链路

当前接口固定：

- topic：`/height/estimate`
- type：`std_msgs/msg/String`
- payload：JSON
- `z`：局部垂直高度估计，单位 `m`，正方向向上
- `vz`：垂直速度，单位 `m/s`，正方向向上
- `covariance`：`z` 的标量方差，单位 `m^2`
- `source_type`：高度来源，例如 `barometer`、`rangefinder`、`lidar_floor`、`depth`、`motion_capture`、`synthetic`

已验证：

- 编译通过：`fake_external_nav`、`imu_bridge`、`cartographer_indoor`、`external_nav_bridge`、`indoor_bringup`
- 默认 `require_height_for_external_nav=false`，不发布 `/height/estimate` 时 `/external_nav/status.state=healthy`
- 默认不依赖高度时，`/external_nav/status.height.required=false`、`present=false`
- 开启 `require_height_for_external_nav=true` 并发布 `{"z":0.82,"vz":0.01,"covariance":0.04,"source_type":"synthetic"}` 后，`/external_nav/status.state=healthy`
- 高度 gating 开启时，status 中 `height.required=true`、`present=true`、`fresh=true`、`parse_ok=true`、`covariance_ok=true`、`source_type=synthetic`、`z=0.82`、`vz=0.01`、`covariance=0.04`

边界：

- P2.5 不把 `/height/estimate` 融合进 `/external_nav/odom`，只做接口预留、gating 和诊断。
- 阶段 1 SITL 验收仍不要求高度接口存在；高度控制质量留到后续真实传感器闭环。

### P2 完成标准

P2 完成后，应该能明确回答：

- SITL ExternalNav 闭环是否可以做小范围移动：可以，P2.1 已通过低速 local velocity setpoint 验收
- rosbag 和日志是否足够复盘阶段 1 问题：可以，P2.2/P2.3 已收口
- artifact 是否能支撑后续阶段对比：可以，P2.3 已固定目录、配置和 summary
- MAVLink 和高度接口是否有清晰扩展位置：可以，P2.4/P2.5 已收口

## 6. 阶段 1 总验收标准

阶段 1 完成时，至少应满足：

- [x] SITL 可重复启动
- [x] fake `/odom` 能驱动 ExternalNav 接收链路通过
- [x] Cartographer-compatible `/odom` adapter 能替换 fake `/odom`
- [x] `/external_nav/status` 能稳定暴露健康状态
- [x] ArduPilot 能持续消费 ExternalNav
- [x] synthetic TF-backed 最小 `hold/hover` 不明显发散
- [x] rosbag 和日志能复盘完整链路
- [ ] Gazebo `/scan` + real/SITL IMU + real `cartographer_ros` feedback acceptance 通过

可以不满足：

- [ ] 复杂航点任务
- [ ] 真实环境试飞
- [ ] 世界模型决策
- [ ] 完整 3D 高度融合
- [ ] 复杂室内 `obj` 场景验证

## 7. 参考文档

- `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`
- `docs/scenarios/indoor/task_breakdown_progress_tracking.md`
