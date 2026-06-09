# 真机 Prepare 与 Task Doctor TODO

## 目标

在 real preflight doctor 证明 host、硬件和依赖边界可用之后，由同一个
`run <task>` wrapper 启动真机辅助进程，并在启动 companion / 进入 arm/takeoff
之前运行 task doctor。目标是把真机 Stage 2 拆成清晰的内部 phase：

```text
run <task>
  -> real preflight doctor
  -> real prepare / bringup
  -> task doctor
  -> companion / task run
```

prepare 负责启动非 companion 的辅助进程；task doctor 负责确认 FCU、scan、SLAM
和 task-specific readiness。它们都不是单独的 operator CLI。

设计文档：

- `docs/scenarios/indoor/navlab_real_prepare_and_task_doctor_design.md`

前置文档：

- `docs/scenarios/indoor/navlab_real_flight_preflight_doctor_design.md`
- `docs/scenarios/indoor/todos/real_flight_preflight_doctor_todo.md`
- `docs/scenarios/indoor/navlab_unified_landing_sequence_design.md`
- `docs/scenarios/indoor/todos/unified_landing_sequence_todo.md`

适用真机 wrapper task：

- `run hover` with `NAVLAB_RUNTIME_BACKEND=process NAVLAB_RUNTIME_MODE=real`
- `run exploration` with `NAVLAB_RUNTIME_BACKEND=process NAVLAB_RUNTIME_MODE=real`
- `run scan-robustness` with `NAVLAB_RUNTIME_BACKEND=process NAVLAB_RUNTIME_MODE=real`

## RTD.0 文档和边界

任务：

- [x] 新增真机 prepare / task doctor 设计文档。
- [x] 新增真机 prepare / task doctor TODO 文档。
- [x] 在 `docs/README.md` 中加入 prepare / task doctor TODO 入口。
- [x] 文档明确 `run <task>` 是唯一 operator 入口。
- [x] 文档明确 preflight doctor、real prepare、task doctor 都是 wrapper 内部 phase。
- [x] 文档明确 prepare 有副作用，doctor 不负责启动辅助进程。
- [x] 文档明确 prepare 不启动 companion。
- [x] 文档明确 task doctor 不 arm、不 takeoff、不 land。
- [x] 文档明确 task doctor 使用统一 upstream topic helper。

验收：

- [x] design 文档只写边界和 contract，不承载 implementation checklist。
- [x] TODO 文档单独承载任务拆分和验收标准。
- [x] README 中能从室内主线导航到 design 和 TODO。
- [x] 后续真机 wrapper 实现必须引用本 TODO。

## RTD.1 Wrapper-only 入口收敛

任务：

- [x] orchestration CLI 收敛到 `run hover`、`run exploration`、`run scan-robustness`。
- [x] justfile 收敛到 `just navlab-run hover`、`just navlab-run exploration`、`just navlab-run scan-robustness`。
- [x] wrapper 从 `NAVLAB_RUNTIME_BACKEND` 读取 backend。
- [x] wrapper 从 `NAVLAB_RUNTIME_MODE` 读取 mode。
- [x] wrapper 不接受 `--stage real`。
- [x] wrapper 不要求 operator 手动执行 `doctor`。
- [x] wrapper 不要求 operator 手动执行 `prepare`。
- [x] simulation mode 仍走 Gazebo/SITL Stage 1 路径。
- [x] real mode 走 preflight -> prepare -> task doctor -> task run 路径。

验收：

- [x] `NAVLAB_RUNTIME_MODE=real run hover` 能进入 real wrapper dispatch。
- [x] `NAVLAB_RUNTIME_MODE=simulation run hover` 不会进入 real prepare。
- [x] CLI help 不把 standalone `doctor` / `prepare` 作为真机主入口。
- [x] 不存在通过 `--stage real` 绕过 env mode 的路径。

## RTD.2 Real prepare 配置

任务：

- [x] 新增或固化 `orchestration/configs/real_prepare.toml`。
- [x] 配置 `mavlink-router` executable、serial、baud、local endpoint。
- [x] 配置 MAVROS executable / launch file / FCU URL。
- [x] 配置 real lidar driver executable / launch file / output scan topic。
- [x] 配置 SLAM runtime executable / launch file / input scan / IMU / odom topic。
- [x] 配置 optional rangefinder bridge。
- [x] 配置每个辅助进程的 startup timeout、health topic 和 shutdown policy。
- [x] 配置 process log、pid file 和 summary artifact 路径。
- [x] real prepare 配置不能引用 Gazebo/SITL/gazebo-sensor 服务。

验收：

- [x] 缺少 required prepare 配置时 wrapper blocked。
- [ ] real prepare 配置加载失败时 summary `ok=false`。
- [x] real prepare 配置中的 FCU endpoint 可追溯到真实 serial。
- [x] real prepare 配置不允许使用 SITL endpoint 冒充 FCU。

## RTD.3 MAVLink Router bringup

任务：

- [x] prepare 启动 `mavlink-routerd` 或 `mavlink-router`。
- [x] `mavlink-router` 独占真实 FCU serial，例如 `/dev/ttyACM0:115200`。
- [x] `mavlink-router` 暴露本机 MAVLink endpoint 给 MAVROS / probe / GCS。
- [x] prepare 记录 router command、pid、serial、baud、endpoint。
- [x] prepare 通过 pymavlink endpoint probe 确认 HEARTBEAT。
- [x] prepare 检查 endpoint evidence 能追溯到真实 serial。
- [x] prepare 失败时关闭已启动的 router process。

验收：

- [ ] 串口被其他进程占用时 prepare blocked。
- [x] router process 未启动时 prepare blocked。
- [x] router endpoint 无 HEARTBEAT 时 prepare blocked。
- [x] SITL TCP/UDP endpoint 没有 real serial provenance 时 prepare blocked。
- [x] prepare summary 包含 `mavlink_router.ok=true` 和 serial provenance。

## RTD.4 MAVROS / FCU ROS bridge bringup

任务：

- [x] prepare 启动 MAVROS 或等价 FCU ROS bridge。
- [x] MAVROS 使用 mavlink-router local endpoint，不直接抢 FCU serial。
- [x] prepare 检查 MAVROS state topic。
- [x] prepare 检查 FCU pose/status/twist topic。
- [ ] prepare 检查 FCU topic freshness。
- [ ] prepare 记录 FCU bridge source claim。
- [x] prepare 失败时关闭 MAVROS process。

验收：

- [x] MAVROS 直接配置 `/dev/ttyACM0` 时 blocked 或 warning 升级为 blocker。
- [x] `/mavros/state` 或等价 bridge state 不存在时 blocked。
- [x] `/ap/v1/status`、`/ap/v1/pose/filtered`、`/ap/v1/twist/filtered` 缺失时 blocked。
- [ ] FCU topic stale 时 blocked。
- [ ] FCU topic source claim 指向 SITL 时 blocked。

## RTD.5 Lidar、rangefinder 和 SLAM bringup

### RTD.5A 当前最高优先级：按 simulation 链路打通 real SLAM yaw

目标：不要为了让 prepare 过而伪造 yaw；real prepare 必须完整模仿当前
simulation 中已跑通的 SLAM contract，而不是只抄 `/scan` 这一段。simulation
SLAM 是一个整体输入链路：`scan + IMU + odometry/TF + Cartographer +
ExternalNav`。real 只替换输入来源为真实硬件，输出 contract 保持一致。

simulation 完整对照：

| Contract | simulation 来源 | simulation topic / 参数 | real 对应来源 |
|---|---|---|---|
| 2D lidar scan | Gazebo `/scan_ideal` 经过 X2 virtual serial 和 `ydlidar_ros2_driver` | `/scan` | `/dev/ttyUSB0` 真实 YDLidar，经真实 driver 发布 `/scan` |
| FCU / IMU evidence | simulation FCU / official Gazebo IMU bridge / NavLab IMU bridge | `imu_source_topic=/navlab/fcu_imu/data` 或 P3 helper 的 `/imu`，统一输出 `/imu` | `/dev/ttyUSB1` MAVLink `HIGHRES_IMU` / `SCALED_IMU` / `RAW_IMU` 发布 `/imu/data`，再归一化到 `/imu` |
| Cartographer backend | `launch_cartographer_backend=true`，不允许 placeholder | `scan_topic=/scan`、`imu_topic=/imu`、`cartographer_odometry_topic=/odometry` | 同样启动 `cartographer_ros`，消费真实 `/scan` 和真实 `/imu` |
| SLAM odom contract | adapter 输出 canonical odom | simulation runtime 默认 `/odom`，orchestration P3/hover 验收使用 `/slam/odom` | real prepare 固定 `/slam/odom` |
| ExternalNav yaw gate | external nav bridge 消费 SLAM odom | `external_nav_input_odom_topic=/slam/odom`，`/external_nav/status.ready=true` | 同样消费 `/slam/odom`，以 `/external_nav/status.ready=true` 作为 yaw evidence |

必须一起参考这五个 contract；缺任一项都不能把 real prepare 判定为 yaw ready。

real prepare 对齐后的目标链路：

```text
/dev/ttyUSB0 real lidar
  -> real lidar driver /scan
/dev/ttyUSB1 FCU MAVLink IMU
  -> navlab_mavlink bridge /imu/data
  -> navlab_slam_imu_bridge /imu
/scan + /imu + Cartographer odometry/TF contract
  -> cartographer_ros backend
  -> navlab_cartographer_adapter
  -> /slam/odom + /navlab/slam/status ready=true
  -> navlab_external_nav_bridge
  -> /external_nav/status ready=true
  -> prepare yaw gate passes
```

任务：

- [x] 固化 real lidar 参数：`/dev/ttyUSB0 @ 115200`，发布真实
  `sensor_msgs/msg/LaserScan` 到 `/scan`，不能使用 `/scan_ideal`、
  `/sim/x2/status` 或 X2 virtual serial。
- [x] 修复或替换当前真机 lidar bringup，使 `/scan` 不只是 topic 存在，
  而是能在 prepare 窗口内收到真实 LaserScan 样本。
- [x] 固化 real IMU 输入：`/dev/ttyUSB1` MAVLink 中的 `HIGHRES_IMU`、
  `SCALED_IMU` 或 `RAW_IMU` 必须通过 NavLab bridge 发布真实 `/imu/data`，
  再由 `navlab_slam_imu_bridge` 归一化到 `/imu`。
- [x] real prepare 禁止使用 synthetic IMU 作为 SLAM yaw gate evidence；
  如果 MAVLink IMU 缺失，应 blocked 为 `real_imu_no_data` 或等价 blocker，
  不能 silent fallback。
- [x] prepare readiness 必须检查 `/imu/data` 和 `/imu` 的 presence、type、
  freshness、frame，并在 `/navlab/slam/status` 中记录 `imu.present=true`、
  `imu.fresh=true`、`imu.count>0`。
- [x] prepare readiness 必须检查 Cartographer backend 真正运行并产生 TF /
  odometry evidence；`/slam/odom` 必须来自 `navlab_cartographer_adapter`
  的真实 backend 输出，而不是 placeholder/fake。
- [x] real prepare 启动 SLAM 时显式设置
  `launch_cartographer_backend:=true`、`publish_placeholder_odom:=false`、
  `scan_topic:=/scan`、`imu_source_topic:=/imu/data`、`imu_topic:=/imu`、
  `cartographer_odometry_topic:=/odometry`、`odom_topic:=/slam/odom`、
  `external_nav_input_odom_topic:=/slam/odom`。
- [x] preflight 依赖检查包含 `cartographer_ros`；缺失时 doctor 先 warning /
  可安装，不能静默降级成 placeholder odom。
- [x] prepare readiness 不只看 topic list；必须读取 `/navlab/slam/status`
  JSON 并确认 `ready=true`，同时记录 scan/imu/tf/odom 计数和 state。
- [x] prepare readiness 必须读取 `/external_nav/status` JSON 并确认
  `ready=true`；`ready=true` 是当前 `external_nav_yaw_ready` 的 accepted
  evidence。
- [x] `/external_nav/status` 的 `odom.input_topic` 必须等于 `/slam/odom`，
  且 `odom.frame_ok=true`、`odom.rate_ok=true`。
- [x] prepare fail 时 summary 必须区分：
  `real_lidar_no_scan_data`、`real_imu_no_data`、`cartographer_ros_missing`、
  `slam_status_not_ready`、`external_nav_status_not_ready`、
  `external_nav_yaw_not_ready`。
- [x] 禁止为了通过 `run hover --dry-run` 而启用
  `publish_placeholder_odom`、`launch_fake_odom` 或直接把 FCU pose TF 当作
  SLAM yaw gate。
- [x] `run hover --dry-run` 在 real mode 下通过 prepare / task doctor 后，
  summary 要证明 companion 未启动、无人机未 arm/takeoff。

验收：

- [x] `just navlab-doctor` 能提示/安装 `cartographer_ros` 等缺失依赖，不把
  Cartographer 缺失误报成 yaw 问题。
- [x] `NAVLAB_RUNTIME_BACKEND=process NAVLAB_RUNTIME_MODE=real uv run --project orchestration python orchestration/main.py run hover --dry-run`
  能启动真实 `/dev/ttyUSB0` lidar、SLAM 和 ExternalNav，并通过 prepare yaw gate。
- [x] 最新 prepare summary 中：
  `/scan` 有真实样本、`/imu/data` 和 `/imu` 有真实样本、
  `/navlab/slam/status.ready=true`、`/external_nav/status.ready=true`、
  `accepted_topic=/external_nav/status`。
- [x] latest runtime logs 中没有 `/scan_ideal`、`/sim/x2/status`、Gazebo、
  SITL、placeholder odom 或 fake odom 作为 real evidence。
- [x] prepare 通过时仍不启动 companion、不 arm、不 takeoff、不 land。

任务：

- [x] prepare 启动真实 lidar driver。
- [x] prepare 检查 `/scan` 存在、类型正确、数据新鲜。
- [x] prepare 检查 `/scan` frame 符合配置。
- [x] 文档明确 2D lidar `/scan` 是水平 SLAM/yaw evidence，不是高度/rangefinder evidence。
- [x] prepare 启动 SLAM runtime。
- [x] SLAM runtime 消费真实 `/scan` 和真实 IMU/odom evidence。
- [x] prepare 检查 `/slam/odom` 存在、类型正确、数据新鲜。
- [x] prepare 检查 `/navlab/slam/status` ready。
- [x] 文档明确 real Stage 2 需要三链路：FCU bridge、2D lidar/SLAM/yaw、height/rangefinder。
- [x] 文档记录 2026-06-10 真机举高测试：FCU `DISTANCE_SENSOR` 会随高度变化，作为 real height bridge 首选输入。
- [ ] prepare 检查 optional `/rangefinder/down/range`、`/rangefinder/down/status` 或 FCU telemetry evidence。
- [ ] 若存在 ROS rangefinder bridge，必须发布 `/rangefinder/down/range` 和 `/rangefinder/down/status`，与 simulation topic contract 一致。
- [ ] 无 ROS rangefinder bridge 时，prepare/task doctor summary 必须记录 FCU MAVLink `DISTANCE_SENSOR` 有效性；`RANGEFINDER`、baro 或 EKF height 只能作为辅助 evidence。
- [ ] real rangefinder bridge 优先从 FCU `DISTANCE_SENSOR` 生成 `/rangefinder/down/range` 和 `/rangefinder/down/status`，并按 orientation/min/max/current 过滤。
- [x] prepare 禁止 `/scan_ideal`、`/sim/x2/status`、Gazebo rangefinder 作为 real evidence。

验收：

- [x] `/scan` 缺失、类型错误或 stale 时 prepare blocked。
- [x] `/slam/odom` 缺失、类型错误或 stale 时 prepare blocked。
- [x] SLAM 未 ready 时 prepare blocked。
- [ ] required rangefinder evidence 缺失时 hover / landing readiness blocked。
- [ ] 2D `/scan` 存在但 rangefinder / height evidence 缺失时，hover / landing readiness 仍 blocked。
- [ ] ROS rangefinder bridge 只发布其他 topic 名时 blocked 或明确 migration blocker。
- [ ] `DISTANCE_SENSOR.current_distance` 为 0、低于 `min_distance`、高于 `max_distance` 或 orientation 非下视时，hover / landing readiness blocked。
- [x] 真实 lidar 不能被 X2 virtual serial 替代。

## RTD.6 Prepare summary、blocker 和进程清理

任务：

- [x] prepare 写出 `prepare_claim=evaluated`。
- [x] prepare summary 记录 `started_services`。
- [x] prepare summary 记录每个 service 的 command、pid、logs、health topic。
- [x] prepare summary 记录 MAVLink router serial provenance。
- [ ] prepare summary 分别记录 FCU bridge、2D lidar/SLAM/yaw、height/rangefinder readiness。
- [x] prepare summary 使用稳定 blocker 字符串。
- [x] prepare fail 时关闭已启动但不再需要的辅助进程。
- [x] wrapper exit 时按 shutdown policy 清理 process。

验收：

- [x] `ok=true` 时 prepare summary 足够复现 bringup 状态。
- [x] `blocked=true` 时 blockers 可直接定位缺依赖、缺 topic 或进程失败。
- [x] prepare fail 不留下孤儿 helper process。
- [x] prepare summary 不把 companion 标记为 started service。

## RTD.7 统一 Task Doctor helper

任务：

- [x] 实现 `check_real_task_upstream_topics(task_name, config)` 或等价 helper。
- [ ] helper 检查 `/scan` presence、type、freshness、frame。
- [x] helper 检查 `/tf`、`/tf_static`。
- [x] helper 检查 FCU status / pose / velocity topic。
- [x] helper 检查 MAVROS state 或等价 FCU bridge state。
- [ ] helper 检查 `/slam/odom` presence、type、freshness、frame。
- [x] helper 检查 `/navlab/slam/status` ready。
- [ ] helper 检查 rangefinder / height evidence；若 ROS bridge 存在，topic 必须是 `/rangefinder/down/range` 和 `/rangefinder/down/status`。
- [x] helper 检查 yaw source evidence，室内 SLAM 真机任务只接受 `external_nav_yaw_ready=true`。
- [x] helper 在无 GPS / 室内 real mode 下不把“未校准磁罗盘”单独作为 blocker，但必须要求 ExternalNav/SLAM yaw ready。
- [x] helper 记录 yaw source provenance，包括 ExternalNav yaw readiness topic 和 ready field。
- [x] helper 检查 forbidden simulation topic/source。
- [x] helper 输出结构化 result，供 hover / exploration / scan-robustness 复用。

验收：

- [x] 任一 required upstream topic 缺失时 task doctor blocked。
- [x] 任一 required upstream topic stale 时 task doctor blocked。
- [x] topic type 不匹配时 task doctor blocked。
- [x] `external_nav_yaw_ready=false` 时 task doctor blocked，即使 `compass_calibrated=true` 或 `manual_override_acknowledged=true`。
- [x] `external_nav_yaw_ready=true` 时 task doctor 不因未校准磁罗盘单独 blocked。
- [x] `manual_override_acknowledged=true` 不作为室内 SLAM 真机任务 yaw source 放行条件。
- [x] forbidden sim topic/source 存在时 task doctor blocked。
- [x] helper result 能写入 task doctor summary。

## RTD.8 Task-specific doctor

任务：

- [x] `hover` task doctor 检查 takeoff altitude、hover hold、landing policy。
- [x] `hover` task doctor 检查 FCU 初始 mode/armed 状态。
- [x] `hover` task doctor 检查 yaw source readiness：必须是 `external_nav_yaw_ready=true`。
- [x] `hover` task doctor 检查 `land_in_place` readiness。
- [x] `exploration` task doctor 检查 P8 Stage 2 return-home policy。
- [x] `exploration` task doctor 检查 home source 和 bounded movement limits。
- [x] `exploration` task doctor 检查 yaw source readiness，并在 P8 return-home 前确认 ExternalNav yaw provenance。
- [x] `exploration` task doctor 检查 `return_home_then_land` readiness。
- [x] `scan-robustness` task doctor 检查 P12 scan stabilization / tilt robustness status。
- [x] `scan-robustness` task doctor 检查 yaw source readiness，避免倾斜/扰动验收误用无来源 yaw。
- [x] `scan-robustness` task doctor 检查 `land_in_place` readiness。
- [x] 每个 task doctor 写出独立 summary，并被 wrapper 汇总。

验收：

- [x] hover 缺 takeoff altitude 或 landing policy 时 blocked。
- [x] hover / P8 / P12 缺 ExternalNav yaw readiness 时 blocked。
- [x] P8 缺 return-home policy 或 home source 时 blocked。
- [x] P12 缺 scan robustness status 时 blocked。
- [x] task doctor 通过不代表已经 arm/takeoff。
- [x] task doctor summary 能说明 task-specific readiness。

## RTD.9 Companion 启动边界

任务：

- [x] wrapper 只在 prepare 和 task doctor 通过后启动 companion。
- [ ] companion 不直接抢 FCU serial。
- [ ] companion 只消费 prepare/task doctor 已验证的 ROS topics。
- [ ] companion startup 写入 run summary。
- [ ] companion status topic ready 后才进入 task run。
- [ ] companion failure 产生稳定 blocker。

验收：

- [x] prepare 未通过时 companion 不启动。
- [x] task doctor 未通过时 companion 不启动。
- [ ] companion 抢 `/dev/ttyACM0` 时 blocked。
- [ ] companion status 不 ready 时 task run blocked。

## RTD.10 Stage 1 和 operator safety 串联

任务：

- [ ] wrapper 检查对应 task 的 Stage 1 `ideal` summary。
- [ ] wrapper 检查对应 task 的 Stage 1 `mild_disturbance` summary。
- [ ] wrapper 检查 Stage 1 summaries 未被错误标记为 real evidence。
- [ ] wrapper 检查 manual takeover confirmation。
- [ ] wrapper 检查 kill switch confirmation。
- [ ] wrapper 检查安全场地/保护措施确认。
- [ ] wrapper summary 记录 Stage 1 artifact、preflight artifact、prepare artifact、task doctor artifact。

验收：

- [ ] 缺 `ideal` Stage 1 artifact 时 real wrapper blocked。
- [ ] 缺 `mild_disturbance` Stage 1 artifact 时 real wrapper blocked。
- [ ] 缺 operator safety confirmation 时 arm/takeoff 前 blocked。
- [ ] wrapper summary 可以追溯所有前置 artifact。

## RTD.11 Rosbag 和审计 artifact

任务：

- [x] prepare 阶段保存 process logs。
- [ ] prepare 阶段保存 ROS topic list。
- [x] prepare 阶段保存 topic info / publisher 摘要。
- [x] task doctor 阶段保存 upstream topic freshness 样本。
- [ ] task doctor 阶段保存 source claim summary。
- [ ] real wrapper 录制真机 flight rosbag。
- [ ] real wrapper summary 引用 preflight / prepare / task doctor artifact。

验收：

- [x] prepare artifact 可复查辅助进程启动状态。
- [x] task doctor artifact 可复查 companion 启动前置 topic。
- [ ] real flight artifact 能追溯到 Stage 1 和所有 real phase artifact。
- [ ] artifact 不把 Gazebo/SITL 输入作为 required evidence。

## RTD.12 测试

任务：

- [x] 增加 CLI wrapper dispatch 测试。
- [x] 增加 real prepare config parser 测试。
- [x] 增加 mavlink-router command/provenance 测试。
- [x] 增加 MAVROS endpoint contract 测试。
- [x] 增加 lidar/SLAM prepare blocked 测试。
- [x] 增加 task doctor helper topic missing/stale/type 测试。
- [x] 增加 task doctor yaw source 测试：只有 ExternalNav yaw ready 可以放行。
- [x] 增加未校准磁罗盘但 ExternalNav yaw ready 时不 blocked 的测试。
- [x] 增加 compass/manual override 存在但 ExternalNav yaw 不 ready 时 blocked 的测试。
- [x] 增加 forbidden simulation source 测试。
- [x] 增加 companion 不提前启动测试。
- [ ] 增加 Stage 1 `ideal` + `mild_disturbance` gate 测试。
- [ ] 增加 wrapper summary schema 测试。

验收：

- [x] real prepare / task doctor 单元测试通过。
- [x] real preflight 现有测试仍通过。
- [ ] unified landing Stage 2 blocker 测试仍通过。
- [x] CLI help contract 测试通过。

## RTD.13 执行顺序

建议顺序：

1. RTD.0 文档和边界。
2. RTD.1 Wrapper-only 入口收敛。
3. RTD.2 Real prepare 配置。
4. RTD.3 MAVLink Router bringup。
5. RTD.4 MAVROS / FCU ROS bridge bringup。
6. RTD.5 Lidar、rangefinder 和 SLAM bringup。
7. RTD.6 Prepare summary、blocker 和进程清理。
8. RTD.7 统一 Task Doctor helper。
9. RTD.8 Task-specific doctor。
10. RTD.9 Companion 启动边界。
11. RTD.10 Stage 1 和 operator safety 串联。
12. RTD.11 Rosbag 和审计 artifact。
13. RTD.12 测试。

## RTD 完成标准

RTD 全部完成必须满足：

- [x] operator 只执行 `run <task>` / `just navlab-run <task>`。
- [x] real mode wrapper 顺序固定为 preflight -> prepare -> task doctor -> companion/task run。
- [x] prepare 只在 preflight 通过后启动非 companion 辅助进程。
- [x] prepare 启动 `mavlink-router`，并证明 MAVLink endpoint 可追溯到真实 serial。
- [x] MAVROS 通过 router endpoint 暴露真实 FCU ROS topics。
- [x] 真实 lidar 提供 `/scan`，SLAM 提供 `/slam/odom`。
- [x] task doctor 复用统一 upstream topic helper。
- [x] task doctor 明确检查 yaw source evidence；无 GPS / 未校准磁罗盘场景必须由 ExternalNav/SLAM yaw ready 补足。
- [x] companion 只在 prepare 和 task doctor 通过后启动。
- [ ] Stage 1 `ideal` 和 `mild_disturbance` artifacts 都通过后才允许 real task run。
- [ ] operator safety confirmation 缺失时不会进入 arm/takeoff。
- [ ] summary 能追溯 preflight、prepare、task doctor、Stage 1 和 real flight artifact。

## 验证记录

### 2026-06-09 RTD TODO 初始化

- 命令：未运行代码测试，纯文档初始化。
- 结果：新增 real prepare / task doctor TODO，按 P9 风格拆分任务、验收、执行顺序和完成标准。
- blocker：wrapper-only CLI、real prepare bringup、task doctor helper、companion 启动边界和真机 Stage 2 wrapper 尚未实现。

### 2026-06-09 RTD wrapper-only CLI implementation

- 命令：`uv run --project orchestration pytest orchestration/tests/test_cli.py orchestration/tests/test_runtime_backend.py orchestration/tests/test_config.py -q`
- 结果：CLI / justfile 已收敛到 `build`、`doctor`、`run <task>`；real mode 下 `run <task>` 先执行 runtime doctor，然后在 prepare/task doctor/flight wrapper 未实现前 blocked，不 fallback 到 simulation task。
- blocker：real prepare bringup、task doctor helper、companion 启动边界和真机 Stage 2 flight wrapper 尚未实现。

### 2026-06-09 RTD real prepare and task doctor implementation

- 命令：`uv run --project orchestration pytest orchestration/tests/test_cli.py orchestration/tests/test_config.py orchestration/tests/test_runtime_backend.py orchestration/tests/test_real_prepare.py -q`
- 结果：142 passed。`run <task>` 的 real mode 顺序固定为 preflight -> prepare -> task doctor -> flight boundary；prepare 配置、MAVLink router serial provenance、MAVROS 不直连串口、真实 upstream topic helper、task-specific doctor 和 companion 不提前启动均有测试覆盖。
- 命令：`git diff --check`
- 结果：通过。
- blocker：Stage 1 `ideal` + `mild_disturbance` real gate、operator safety confirmation、rangefinder evidence、topic frame/source-claim summary、flight rosbag 和真正 companion/arm/takeoff wrapper 尚未实现。
