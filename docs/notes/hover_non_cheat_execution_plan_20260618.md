# Hover 非作弊修复方案

日期：2026-06-18

## 目标

修复 hover mission，让飞机在 Gazebo 物理世界中真实完成垂直起飞、悬停、降落。

最终成功不能只看 FCU local-z、Foxglove 视觉、或 mission 自己的 hover_drift；必须同时满足物理证据、传感器证据、SLAM/external-nav 证据和 landing 证据。

## 用户硬要求

- 不允许再乱改；每个阶段先有目标、TODO、验收标准。
- 不复现旧方案作为下一步工作；旧方案已经确认不是最终路线。
- 不允许为了 Foxglove 好看而改 runtime 成功链。
- 不允许把 replay-only、diagnostic-only 的东西混进 hover success path。
- 不允许用 Gazebo truth、Gazebo odom、官方 maze map 作为 runtime 定位输入。
- 不允许再改全局 runtime `/tf map -> base_link` 来制造“贴墙”效果；runtime TF 只能表达真实控制/状态链，显示层只能 replay-only。
- 所有结论必须来自 run artifact、summary、MCAP、日志或测试，不能口头说成功。
- 失败就说失败，不准把 blocked run 包装成成功。

## 明确禁止的成功路径

以下内容可以做诊断，但不能作为 hover mission success 的 runtime 输入：

- `/gazebo/model/odometry`
- Gazebo `/odometry`
- 旧式 odom-assisted Cartographer，如果 odom 来源是 Gazebo/仿真 truth
- 官方 maze map / known map runtime localizer
- `hover_scan_map_localizer` 作为 `/slam/odom` runtime source
- `/scan_map_aligned`、`/scan_map_aligned_points`
- fixed XY prior，例如 `hover_cartographer_odom_prior.py`
- Foxglove display TF 反推出来的位姿

## 允许的正式输入

正式 hover 成功链只允许这些输入：

- LiDAR scan：`/scan`
- 机载/仿真传感器 IMU：原始 `/imu`，经 bridge 输出 `/navlab/slam/imu`
- Rangefinder 高度：`/rangefinder/down/range`、`/height/estimate`
- ArduPilot FCU 状态和 local pose：只用于闭环和交叉验证，不用于伪造 SLAM
- Cartographer 非 truth 输出：`/slam/odom`
- external-nav 输出：`/external_nav/odom`

## 正式目标链路

```text
/scan + /navlab/slam/imu
  -> Cartographer 非 truth SLAM
  -> /slam/odom
  -> external_nav_bridge
  -> /external_nav/odom
  -> mavlink_external_nav
  -> MAVLink ODOMETRY
  -> ArduPilot EKF
  -> Gazebo 物理机体真实 hover
```

Gazebo model odom 只允许在验收端做防假成功：

```text
/gazebo/model/odometry -> gate / MCAP review only
```

## 全任务 FSM 绑定 TODO 索引

这个索引是整个 hover task 的总 TODO，不只覆盖 landing。后续改代码、写测试、跑真实 hover，都必须能映射到下面某个状态或状态转换；如果一个新 TODO 无法映射到 FSM，必须先说明它是 diagnostic/replay-only，还是需要扩展 FSM。

```text
S0 wait_runtime
S1 wait_nav_ready
S2 set_guided
S3 arm
S4 takeoff
S5 hover_settle
S6 hover_hold
S7 pre_land_hold
S8 command_land
S9 land_mode_monitor
S10 touchdown_monitor
S11 disarm_monitor
S12 landing_complete
S13 task_success
S_abort
```

### FSM 总 TODO

- [x] `S0 wait_runtime`：生成 runtime plan/artifact，确保正式路径没有 `hover_scan_map_localizer`、`hover_cartographer_odom_prior`、Gazebo truth runtime input、官方 map localizer、Foxglove display TF。
- [x] `S0 wait_runtime`：rosbag/MCAP 录制 `/slam/odom`、`/external_nav/odom`、`/navlab/slam/imu`、`/gazebo/model/odometry`、hover/landing/status；Gazebo odom 只允许 review/gate。
- [x] `S1 wait_nav_ready`：SLAM-only 验证通过前不接 FCU 起飞；`/scan + /navlab/slam/imu -> Cartographer -> /slam/odom` 不吃 Gazebo odom/官方 map。
- [x] `S1 wait_nav_ready`：`/external_nav/status` 必须包含 `slam_quality`、`slam_quality_reason`、`slam_quality_good`、`slam_quality_report`，只有 `slam_quality_good=true` 才允许进入起飞链。
- [ ] `S1 wait_nav_ready`：补齐 `/scan` frame、`base_scan`、`imu_link`、`base_link` TF 稳定性检查；不能用全局 `/tf map -> base_link` 或 replay-only aligned scan 修 runtime。
- [x] `S1->S2 set_guided`：preflight 必须同时满足 external-nav、mavlink_external_nav、FCU local position、IMU、SLAM quality ready，并持续 `preflight_ready_sec`。
- [x] `S2 set_guided`：发送 GUIDED mode，必须看到 expected mode；airborne 后丢 GUIDED 必须进入 `S_abort`，不能继续 hover hold。
- [x] `S3 arm`：只在 `S1/S2` gate 通过后 arm；arm rejected 或 airborne 后 disarmed 必须明确 blocker。
- [x] `S4 takeoff`：不允许靠 FCU local-z 单独证明起飞；必须 `takeoff_ack_ok=true`，或者 rangefinder/external_nav 独立高度同时达到目标窗口。
- [x] `S5 hover_settle`：airborne 后继续检查 SLAM/external-nav/FCU local/GUIDED 未丢失；高度仍需 rangefinder/external_nav 独立证据。
- [x] `S6 hover_hold`：hover success 必须同时满足 `hover_altitude_crosscheck.ok=true`、`hover_drift.ok=true`、`hover_body_ok=true`、`crash_detected=false`，不能只看 FCU local-z 或 Foxglove。
- [ ] `S6 hover_hold`：量化同一 hover window 的 `/slam/odom`、`/external_nav/odom`、`/ap/v1/pose/filtered` 与 Gazebo review-only drift；Gazebo 只做验收对比，不进入控制。
- [ ] `S7 pre_land_hold`：冻结 hover evidence 并写入 summary；只允许 hold setpoint，不允许继续向下压 z。
- [ ] `S7->S8 command_land`：Phase 5A 正常路径必须直接发送 `MAV_CMD_NAV_LAND`，不进入 `guided_descent`；`guided_descent` 只允许 diagnostic/rollback。
- [ ] `S8 command_land`：记录 `land_command_sent`、`land_command_accepted`、`mode_before_land`、`mode_after_land`、`land_mode_seen`、`land_mode_seen_elapsed_sec`。
- [ ] `S8/S9 land_mode_monitor`：记录 FCU LAND 参数审计：`LAND_SPEED`/`LAND_SPD_MS`、`LAND_SPEED_HIGH`/`LAND_SPD_HIGH_MS`、`LAND_ALT_LOW_M`，以及 rangefinder/terrain/surface tracking 相关参数；不得把 `EK3_SRC1_POSZ` 改成 Rangefinder。
- [ ] `S9 land_mode_monitor`：companion 不再发送 GUIDED 下降 setpoint；只监控 LAND mode 下降、rangefinder raw/relative、FCU vz、landed_state、mode。
- [x] `S9/S10 descent_profile`：rangefinder outlier/validity summary 已覆盖单点/高平台离群、source switch、rate spike、VZ spike；在 `guided_descent` diagnostic 路径仍保留项目速度 gate。
- [x] `S9/S10 descent_profile`：在 `ap_land_mode` 下，下降速度和 post-touchdown bounce profile 只作为审计证据，不再以项目 `0.25m/s` gate 阻断成功；正式成功改由官方 LAND handoff、touchdown、disarm、motors safe 证明。
- [ ] `S10 touchdown_monitor`：touchdown 必须优先用 rangefinder relative height + vertical speed 或 landed_state；不能退回 FCU local-z 单独证明触地。
- [ ] `S11 disarm_monitor`：优先等待 ArduPilot LAND 自动 disarm/motor safe；force disarm 只能在 touchdown confirmed + grace timeout 后兜底，并写 `force_disarm_used=true`。
- [ ] `S12 landing_complete`：landing summary 必须区分 `landing_controller="ap_land_mode"` 与 `guided_descent`，区分 `auto_disarm_by_land_mode` 与 `force_disarm_after_touchdown`。
- [ ] `S12->S13 task_success`：只有 hover ok、landing ok、required probes ok、无禁止输入时才允许 `summary.ok=true`；`slam_hover_probe` killed 必须停在 `probe_failed`。
- [ ] `S_abort`：任意 command rejected、mode lost、SLAM/external-nav lost、crash、timeout、probe failed 都必须写明确 blocker；不得把局部 hover evidence 包装成完整成功。

### 当前实现优先级

1. 先实现 `S7->S8->S9->S10->S11->S12` 的 Phase 5A LAND-mode FSM 和定向单测。
2. 再跑真实 hover，按 S0-S13 状态线读 artifact。
3. 如果 landing 通过但 `S12->S13` 因 `slam_hover_probe` killed 失败，再单独修 probe；不能提前宣布成功。

### 当前代码实施 TODO：统一 FSM 记录/summary，然后实现 Phase 5A

目标：先从当前 `hover_mission.py` 里把主 hover 状态和 landing 状态收敛到统一 FSM 记录/summary，再实现 Phase 5A 的 `S7->S12` LAND-mode 路径。这个 TODO 是下一位执行者的代码实施清单，必须按顺序做；不能跳过 FSM 记录直接改 landing 下降逻辑。

#### Slice A：统一 FSM 记录和 summary，不改变控制行为

- [x] `A1 / S0-S13` 在 `navlab/sim/companion/nodes/hover_mission.py` 增加统一 FSM 状态记录结构，例如 `mission_fsm_state`、`mission_fsm_history`、`mission_fsm_current_entered_at`。
  - [x] 每次状态变化记录：`state`、`entered_at_monotonic`、`exited_at_monotonic`、`duration_sec`、`reason`、`guard`、`blocker`。
  - [x] 初始状态为 `S0 wait_runtime` 或现有 runtime 已进入 controller 时的 `S1 wait_nav_ready`，必须写清楚映射原因。
  - [x] 不改变现有 setpoint、takeoff、landing 控制行为；本 slice 只做观测/summary。
- [x] `A2 / hover 主状态映射` 把 `decide_hover(...)` 输出映射到统一 FSM：
  - [x] `wait_ready -> S1 wait_nav_ready`
  - [x] `guided -> S2 set_guided`
  - [x] `arm -> S3 arm`
  - [x] `takeoff -> S4 takeoff`
  - [x] `hover_settle -> S5 hover_settle`
  - [x] `hover_hold -> S6 hover_hold`
  - [x] `complete -> S7 pre_land_hold`，如果 `require_landing=false` 则只能进入 diagnostic complete，不能当正式 hover mission 成功。
  - [x] `abort -> S_abort`
- [x] `A3 / landing 子状态映射` 把当前 `_landing_state` 映射到统一 FSM：
  - [x] `task_body_complete/pre_land_hold -> S7 pre_land_hold`
  - [x] 当前旧 `guided_descent` 先映射为 `legacy_guided_descent_diagnostic`，不得映射到正式 success path。
  - [x] `land_command_sent -> S8 command_land`
  - [x] `descent_monitoring -> S9 land_mode_monitor`，在 Phase 5A 前必须标记 controller 仍可能是 `guided_descent`，不能伪装成 `ap_land_mode`。
  - [x] `touchdown_candidate -> S10 touchdown_monitor`
  - [x] `disarm_requested -> S11 disarm_monitor`
  - [x] `landing_complete -> S12 landing_complete`
- [x] `A4 / status 输出` 在 `/navlab/hover/status`、`/navlab/landing/status` 和 final summary 中增加：
  - [x] `mission_fsm_state`
  - [x] `mission_fsm_state_entered_at_sec`
  - [x] `mission_fsm_history`
  - [x] `mission_fsm_last_transition_reason`
  - [x] `mission_fsm_blocker`
  - [x] `landing_controller`，当前旧路径写 `guided_descent`，Phase 5A 写 `ap_land_mode`
- [ ] `A5 / summary gate` final summary 必须能从 FSM 判断完整路径：本轮只完成 FSM 观测字段；强制 `summary.ok` gate 留到后续 Slice，避免 Slice A 改变既有成功判定。
  - [ ] 未到 `S13 task_success` 时，`summary.ok` 不能为 true。
  - [ ] 到 `S12 landing_complete` 但 required probe killed 时，必须停在 `S12->S13 probe_failed`，不能 success。
  - [ ] `S_abort` 必须输出明确 `blockerCodes` 和最后状态。

#### Slice A 定向测试

- [x] `test_hover_mission.py` 增加 hover 主状态到 FSM 的映射测试：`wait_ready/guided/arm/takeoff/hover_settle/hover_hold/complete/abort` 都映射正确。
- [x] 增加 landing 子状态到 FSM 的映射测试：旧 `guided_descent` 只能标记 diagnostic/legacy，不能作为正式 success path。
- [x] 增加 FSM recorder 测试：`S_abort` 时 blocker 保留；`summary.ok`/probe failed 强 gate 测试留到后续 Slice。
- [x] 运行：`uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_hover_mission.py -q`，结果 `40 passed`。

#### Slice B：实现 Phase 5A 的 S7->S12 LAND-mode 路径

- [x] `B1 / config` 增加 landing policy，例如 `landing_policy=ap_land_mode_after_hover`。
  - [x] 默认 hover mission 使用 `ap_land_mode_after_hover`。
  - [x] 旧 `guided_descent` 只保留为 diagnostic/rollback policy，并且 summary 必须写 `landing_controller="guided_descent"`。
- [x] `B2 / S7 pre_land_hold` 进入 landing 前冻结 hover evidence：
  - [x] `takeoff_ack_ok`
  - [x] `hover_altitude_crosscheck`
  - [x] `hover_drift`
  - [x] `hover_body_ok`
  - [x] `crash_detected=false`
  - [x] 只发送 hold setpoint，不允许向下压 z。
- [x] `B3 / S7->S8 command_land` 修改 `_tick_landing(...)` 正常路径：
  - [x] `pre_land_hold` 结束后立即发送 `MAV_CMD_NAV_LAND`。
  - [x] 正常路径不调用 `_send_landing_descent_setpoint(...)`。
  - [x] `land_command_sent=true`、`land_command_sent_time`、`land_command_ack` 写入 landing status/summary。
  - [x] `landing_command_rejected` 必须进入 `S_abort` 或 landing fail，不能继续监控。
- [x] `B4 / S8/S9 mode 观测` 增加 LAND mode 观测字段：
  - [x] `mode_before_land`
  - [x] `mode_after_land`
  - [x] `land_mode_seen`
  - [x] `land_mode_seen_elapsed_sec`
  - [x] `landed_state_timeline`
  - [x] 如果当前 telemetry 不能可靠读 mode，不假写 true；只有 HEARTBEAT custom_mode 看到 LAND 才写 true。
- [x] `B5 / S8/S9 LAND 参数审计` 记录 FCU LAND 参数快照，只用于解释 artifact：
  - [x] `LAND_SPEED` 或 `LAND_SPD_MS`
  - [x] `LAND_SPEED_HIGH` 或 `LAND_SPD_HIGH_MS`
  - [x] `LAND_ALT_LOW_M`
  - [x] rangefinder/terrain/surface tracking 相关参数
  - [x] 禁止把 `EK3_SRC1_POSZ` 改成 Rangefinder，禁止设置 `EK3_RNG_USE_HGT` 来绕过 landing 问题。
- [x] `B6 / S9 land_mode_monitor` companion 不再发送 GUIDED 下降 setpoint：
  - [x] 只记录 rangefinder raw/relative、FCU z/vz、external_nav height、landed_state、mode。
  - [x] descent profile 继续评估真实/FCU 认为真实的下降速度，但 `ap_land_mode_after_hover` 下只写审计字段，不再产生 `landing_descent_too_fast` / `ap_land_mode_descent_too_fast` blocker。
  - [x] 旧 `guided_descent` diagnostic/rollback 路径仍保留项目 `max_landing_descent_rate_mps` gate。
- [x] `B7 / S10 touchdown_monitor` touchdown 确认逻辑保持非作弊：
  - [x] 优先 `landed_state_on_ground` 或 rangefinder relative height + vertical speed 持续 `touchdown_confirm_sec`。
  - [x] 不能退回 FCU local-z 单独确认触地。
- [x] `B8 / S11 disarm_monitor` 等待 LAND 自动 disarm：
  - [x] 默认先等待 auto disarm/motor safe，touchdown 后经过 `force_disarm_grace_sec` 才允许 companion disarm。
  - [x] force disarm 只能在 `touchdown_confirmed=true` 且 disarm grace timeout 后使用。
  - [x] 如果用了 force disarm，summary 必须写 `force_disarm_used=true`，并区分 `auto_disarm_by_land_mode=false`。
- [x] `B9 / S12 landing_complete` landing summary 必须包含：
  - [x] `landing_controller="ap_land_mode"`
  - [x] `auto_disarm_by_land_mode`
  - [x] `force_disarm_after_touchdown`
  - [x] `force_disarm_used`
  - [x] `land_command_sent`
  - [x] `land_command_accepted`
  - [x] `land_mode_seen`
  - [x] `fcu_land_params`
  - [x] `mission_fsm_history`

#### Slice B 定向测试

- [x] `S7->S8`：pre_land_hold 结束后直接发送 `MAV_CMD_NAV_LAND`。
- [x] `S8`：`ap_land_mode_after_hover` 正常路径不调用 `_send_landing_descent_setpoint(...)`。
- [x] `S8->S_abort`：land command rejected 会 fail，并写 `landing_command_rejected`。
- [x] `S9`：landing status 中 `landing_controller="ap_land_mode"`，并记录 mode/LAND 参数字段。
- [x] `S10`：touchdown 不能只靠 FCU local-z；rangefinder relative height 和 landed_state 路径仍通过。
- [x] `S11`：auto disarm 优先；force disarm 只有 touchdown confirmed + grace timeout 后触发，并写 `force_disarm_used=true`。
- [x] `S12->S13`：required probe killed 时不能 success。
- [x] 运行：`uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_hover_mission.py navlab/tests/companion/test_config.py -q`，结果 `50 passed`。

#### Slice C：真实 run 前检查

- [x] 不跑 hover 前先确认上述 Slice A/B 定向测试通过：Python `50 passed`，Go `./internal/tasks/helpers ./internal/tasks` 通过。
- [x] 不改 hover 高度门槛、landing 验收阈值、Gazebo drift gate、Foxglove 显示补丁、全局 runtime TF、SLAM quality gate。
- [x] 不接 Gazebo truth、官方 maze map、fixed XY prior、replay-only aligned scan 到 runtime success path；dry-run artifact 未出现 `hover_scan_map_localizer`、`hover_cartographer_odom_prior`、`scan_map_aligned`、`map->base_link` runtime success helper。
- [x] 测试通过后才允许真实 hover run：`cd orchestration/sim && env -u NAVLAB_SIM_OVERLAY_SOURCE_MODE NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover`。
- [x] 真实 run 后只按 `summary.json`、`mission_summary.json`、`/navlab/hover/status`、`/navlab/landing/status`、`slam_hover_probe.json/log` 判断，不凭 Foxglove 或局部字段宣布成功。
- [x] Slice C 结论：真实 run `artifacts/sim/hover/20260619T032621Z` 仍失败，`summary.ok=false`、`summary.blocked=true`、FSM 到 `S_abort`，不能宣称 hover mission 成功。

## 全链路官方依据矩阵

这个矩阵回答“整个任务每个环节是否有官方文档支撑”。结论不是所有东西都是 ArduPilot 官方行为：飞控控制接口、ExternalNav、Cartographer 配置、ROS frame/TF、MAVLink frame、LAND mode 有官方/标准依据；SLAM quality gate、防作弊边界、Gazebo review-only 对比、Foxglove 复查是 NavLab 项目 gate，必须保留但不能伪装成官方文档要求。旧 `0.25m/s` landing speed gate 已被限定为 `guided_descent` diagnostic/rollback 和审计字段，不再阻断 `ap_land_mode_after_hover`。

| FSM 环节 | 设计内容 | 官方/标准依据 | 支撑等级 | 备注 |
| --- | --- | --- | --- | --- |
| `S0 wait_runtime` | runtime artifact、rosbag/MCAP、summary、blocker 追踪 | 无单一 ArduPilot 官方要求；这是 NavLab 验收/可审计策略 | 项目 gate | 官方文档不会要求我们的 summary 格式；但为防止假成功必须保留。 |
| `S0 wait_runtime` | Gazebo model odom 只做 review/gate，不进入控制 | ArduPilot/Gazebo/SITL 可以用于仿真，但“truth 不能进控制”是 NavLab 非作弊边界 | 项目 gate | 这是为了保证真实机可迁移，不是 ArduPilot 官方命令。 |
| `S1 wait_nav_ready` | ROS frame：`map/odom/base_link/imu_link/base_scan/laser_frame` 不乱改全局 `/tf` | ROS REP 105 定义移动机器人常用坐标系；Cartographer ROS 明确 frame id、tf2、incoming sensor frame 到 `tracking_frame/published_frame` 的 TF 必须可用 | 官方/标准支撑 | 依据：https://www.ros.org/reps/rep-0105.html ，https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html ，https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html |
| `S1 wait_nav_ready` | Cartographer 使用 `/scan + /navlab/slam/imu`，不吃 Gazebo odom | Cartographer ROS 配置支持 `use_odometry=false`、`use_imu_data`、`tracking_frame`、`published_frame`；ArduPilot 开发文档有 “Cartographer SLAM for Non-GPS Navigation” 路径 | 官方/标准支撑 | 官方支持“用 Cartographer/LiDAR 给 ArduPilot 提供本地位置估计”，但不保证我们的墙边 hover 场景一定水平可观测。依据：https://ardupilot.org/dev/docs/ros-cartographer-slam.html |
| `S1 wait_nav_ready` | SLAM-only 前置验证，不接 FCU 起飞试错 | 没有 ArduPilot 硬性要求；这是 NavLab 安全流程 | 项目 gate | 用来证明 SLAM 链路先独立工作，避免坏 odom 直接拉飞。 |
| `S1 wait_nav_ready` | SLAM quality gate：`slam_quality_good=true` 才发 `/external_nav/odom`/允许起飞 | ArduPilot 官方要求 ExternalNav 输入足够频率/可用，但没有定义 NavLab 的 `slam_quality` 字段 | 项目 gate，受官方 ExternalNav 方向约束 | 质量字段和 low-observability 判定是本项目实现；不能说是官方 gate。 |
| `S1 wait_nav_ready` | ExternalNav 通过 MAVLink `ODOMETRY` 进入 EKF | ArduPilot Non-GPS Position Estimation 明确 `ODOMETRY` 是 preferred method，4Hz+；参数包含 `VISO_TYPE`、`EK3_SRC1_POSXY=6`、`EK3_SRC1_VELXY=6/0`、`EK3_SRC1_POSZ=6/1`、`EK3_SRC1_YAW=6/1` | 官方支撑 | 依据：https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html |
| `S1 wait_nav_ready` | EKF source：`EK3_SRC1_POSXY=6`，`EK3_SRC1_VELXY=0`，Z 不强行改 Rangefinder | ArduPilot EKF Source 文档支持 ExternalNav source；也说明 RangeFinder 作为 POSZ 几乎不应使用；Terrain Following 文档警告不要把 `EK3_SRC1_POSZ` 设成 Rangefinder、不要设 `EK3_RNG_USE_HGT` | 官方支撑 | 依据：https://ardupilot.org/copter/docs/common-ekf-sources.html ，https://ardupilot.org/copter/docs/terrain-following.html |
| `S1 wait_nav_ready` | ROS ENU 到 MAVLink LOCAL_FRD/NED 映射 | MAVLink `ODOMETRY` 定义 pose frame；`MAV_FRAME_LOCAL_FRD` 为前/右/下，`MAV_FRAME_LOCAL_FLU` 为前/左/上；旧 vision/mocap NED/ENU frames 已被 LOCAL_FRD/FLU 替代 | 标准支撑 | 依据：https://mavlink.io/en/messages/common.html |
| `S2 set_guided` | GUIDED mode 下发送 position hold setpoint | ArduPilot Guided Mode 文档支持 `SET_POSITION_TARGET_LOCAL_NED`，NED 中 z 正向下；位置/速度/加速度至少提供一组且三轴齐全 | 官方支撑 | 依据：https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html |
| `S3 arm` | `MAV_CMD_COMPONENT_ARM_DISARM` arm/disarm/force disarm | ArduPilot MAVLink arming 文档明确 `param1=1/0` arm/disarm，`param2=21196` force | 官方支撑 | 依据：https://ardupilot.org/dev/docs/mavlink-arming-and-disarming.html |
| `S4 takeoff` | `MAV_CMD_NAV_TAKEOFF` | ArduPilot Guided Mode 文档列出 `MAV_CMD_NAV_TAKEOFF` 可处理 | 官方支撑 | takeoff ACK 后还要 rangefinder/external_nav 独立高度，是 NavLab 防假成功 gate。 |
| `S4/S5/S6` | 不靠 FCU local-z 单独确认起飞/hover，高度需 rangefinder/external_nav 交叉证据 | ArduPilot 支持 rangefinder 和 ExternalNav，但“不能靠 FCU local-z 单独通过”是 NavLab 验收规则 | 项目 gate，受官方传感器能力支撑 | 这是针对之前假 hover 的防回归规则。 |
| `S6 hover_hold` | hover_drift、Gazebo review-only drift、SLAM/Gazebo 比值 | 无官方 ArduPilot 验收阈值 | 项目 gate | Gazebo 对比只用于防假成功，不能进入 runtime input。 |
| `S7 pre_land_hold` | hover evidence 冻结后进入 landing，不再继续向下压 z | 无单一官方要求；是 FSM 安全边界 | 项目 gate | 防止 hover 未完成就 landing 或继续用 GUIDED descent 造成近地快速下降。 |
| `S8 command_land` | `MAV_CMD_NAV_LAND` 切 LAND mode | ArduPilot Guided Mode 文档明确 `MAV_CMD_NAV_LAND` 会切到 Land mode | 官方支撑 | 依据：https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html |
| `S9 land_mode_monitor` | LAND mode 下降由 FCU 控制，companion 不再发 GUIDED 下降 setpoint | ArduPilot Land Mode 文档说明 Land mode 直下降、使用 `LAND_SPD_HIGH_MS/WP_SPD_DN` 和 `LAND_SPD_MS/LAND_ALT_LOW_M` | 官方支撑 | 依据：https://ardupilot.org/copter/docs/land-mode.html |
| `S9/S10` | rangefinder 用于近地高度控制/landed detection 辅助 | ArduPilot Rangefinder 文档说明下视 rangefinder 自动用于有高度控制的模式；Land Mode 文档说明有健康 rangefinder 且低于 2m 是 landed detection 条件之一 | 官方支撑 | 依据：https://ardupilot.org/copter/docs/common-rangefinder-landingpage.html ，https://ardupilot.org/copter/docs/land-mode.html |
| `S9/S10` | landing descent profile outlier/validity filter | ArduPilot 有 rangefinder/terrain/surface tracking 机制，但 NavLab 的 summary filter 和 source-aware profile 是项目实现 | AP LAND 下为 audit；guided diagnostic 下为项目 gate | `ap_land_mode_after_hover` 下官方 LAND mode 接管下降，profile 只记录；`guided_descent` 回滚路径仍可用它阻断。 |
| `S11 disarm_monitor` | LAND 自动 motor stop/disarm，force disarm 只做 touchdown 后兜底 | Land Mode 文档说明触地后会自动关电机并 disarm；MAVLink arming 文档支持 force disarm | 官方支撑 + 项目安全约束 | force disarm 可用，但必须标记，不准伪装成 LAND 自动 disarm。 |
| `S12/S13` | required probes、summary.ok、Foxglove 人工复查 | 无 ArduPilot 官方要求 | 项目 gate/diagnostic | Foxglove 只能复查，不能作为唯一成功证据；probe killed 不能 success。 |

### 官方依据审查结论

- 可以继续按当前总体架构做：`/scan + /navlab/slam/imu -> Cartographer -> /slam/odom -> /external_nav/odom -> MAVLink ODOMETRY -> ArduPilot EKF -> GUIDED hover -> MAV_CMD_NAV_LAND -> LAND mode disarm`，这条链路的关键接口都有官方或标准文档支撑。
- 不能说“官方保证现在这个 hover 场景一定能靠 2D LiDAR + IMU SLAM 稳定悬停”。官方只支持 Cartographer/ExternalNav 机制，不保证 NavLab 当前环境的水平可观测性；所以 SLAM-only、SLAM quality gate、Gazebo review-only drift 仍必须保留。
- 不能把 NavLab 自己的 gate 说成官方要求：例如 `slam_quality_good`、Gazebo drift 阈值、Foxglove 复查、probe required 都是项目验收合同。旧 `0.25m/s` landing speed gate 不再作为 AP LAND success gate。
- 也不能反过来绕过官方警告：后续不得为了 landing/高度好看把 `EK3_SRC1_POSZ` 改成 Rangefinder，或设置 `EK3_RNG_USE_HGT` 来掩盖控制问题。

## 当前已知事实

### 旧方案为什么看起来好

旧方案看起来像“SLAM 做到了”，但关键差异是它使用了 odom-assisted Cartographer：

- `use_odometry = true`
- `provide_odom_frame = true`
- `/odom` 或 `/odometry` 输入来自 Gazebo/仿真 odom
- Foxglove 中 map/odom/base_link/scan 因此自洽

这能解释旧版视觉表现，但不能作为当前非作弊 hover 成功标准。

### 当前失败模式

已经观察到几类失败：

- fixed XY prior 会把 `/slam/odom` 锁住，掩盖 Gazebo 真实水平漂移。
- LiDAR-only Cartographer 能看到漂移，但可能横向过估，导致 FCU 被拉偏。
- IMU + Cartographer 不再 fatal 后，仍可能低估真实水平平移，导致 FCU 继续假稳定。
- external-nav 过强限速会抹平真实 SLAM 位移；完全不限速又可能把 SLAM 跳变直接打进 EKF。
- 用官方 maze map 做 runtime scan-to-map localizer 仍属于已知地图先验，不能作为正式成功路径。

### 可观测性风险

当前 hover 场景存在一个根本风险：如果没有 Gazebo odom、已知地图、光流、VIO、轮速等独立水平观测源，纯 2D LiDAR + IMU 在近似原地 hover 时可能无法可靠估计真实水平漂移。

这不是参数问题，而是可观测性问题。后续不得默认“Cartographer 一定能看见水平漂移”。必须先用 artifact 证明 `/scan + /navlab/slam/imu -> /slam/odom` 对水平运动可观测、稳定、不过估也不低估。

如果 Phase 1/2 证明该场景中纯 Cartographer 不可观测或不可靠，正式结论应是“当前传感器组合不足以非作弊完成水平定位闭环”，而不是继续用 truth、已知地图或 Foxglove 补丁绕过去。

## Phase 0：停止越界改动，恢复正式成功路径边界

状态：已完成静态实现与 dry-run/doctor 验证；尚未进行真实 hover 实跑。

目标：先把作弊/诊断/临时方案从 hover runtime success path 移除，保证后续 run 的失败和成功都可信。

### Phase 0 冻结 baseline

Phase 0 只恢复边界，不继续调参。为了避免每轮 run 互相不可比，先冻结以下 baseline；这不是成功参数，只是防止 `0.01` 抹平漂移或 `0` 直通跳变继续污染判断：

- `max_horizontal_speed_mps = 0.25`
- `max_yaw_rate_radps = 0.6`
- `EK3_SRC1_VELXY` 暂不在 Phase 0 中新增改动；Phase 3 再按 MCAP 证据决定是否禁用/启用 ExternalNav velocity fusion。

如果 Phase 0 后仍失败，只能根据 artifact 判断是 SLAM 不可信、external-nav/EKF 未融合，还是真实物理漂移；不能边跑边随手改这些 baseline。

### TODO

- [x] 从 hover execution plan 移除 `hover_scan_map_localizer` runtime service。
- [x] 删除或隔离 `navlab/sim/companion/nodes/hover_scan_map_localizer.py`，至少确保它不作为 `/slam/odom` 发布者参与 hover。
- [x] 恢复 hover runtime 中 `/slam/odom` 的唯一正式来源为 Cartographer adapter。
- [x] 保留 IMU topic 分离：source `/imu`，output `/navlab/slam/imu`。
- [x] 保留 Gazebo model odom gate：只做验收 blocker，不进入控制。
- [x] 固定 external-nav sender baseline，禁止继续用旧实验值污染后续判断：
  - [x] 不使用 `max_horizontal_speed_mps=0.01` 这种会抹平真实漂移的限速。
  - [x] 不使用 `max_horizontal_speed_mps=0` 这种完全不限速的跳变直通。
  - [x] Phase 3 前只允许使用文档指定的 baseline 值，并在每轮 run 记录该值。
- [x] 确认 `hover_cartographer_odom_prior.py` 不生成、不启动、不发布 `/slam/odom`。
- [x] 确认 `/scan_map_aligned*` 不在默认 hover Foxglove/replay 成功路径中。

### Phase 0 预计改动文件

执行 Phase 0 时只允许围绕这些文件做最小改动；如果需要改更多文件，必须先把原因写进日志：

- `orchestration/sim/internal/tasks/helpers/execution_plan.go`：移除 `hover_scan_map_localizer` service。
- `orchestration/sim/internal/tasks/runtime_artifacts.go`：恢复 hover 正式 `/slam/odom` 来源为 Cartographer adapter。
- `navlab/sim/companion/nodes/hover_scan_map_localizer.py`：删除、隔离或确保不会在 hover runtime 启动。
- `navlab/common/slam/ros/localization/navlab_cartographer_adapter/launch/navlab_cartographer_adapter.launch.py`：只在需要恢复 Cartographer adapter topic 时修改。
- `navlab/tests/**`、`orchestration/sim/internal/**/**_test.go`：同步测试合同。

Phase 0 不应该修改 hover 高度门槛、landing 速度门槛、Gazebo drift 验收阈值、Foxglove 显示补丁或全局 runtime `/tf map -> base_link`。

### 验收

- [x] `runtime_plan.json` 中没有 `hover_scan_map_localizer` service。
- [x] `runtime_plan.json` 中没有 `hover_cartographer_odom_prior` service。
- [x] `slam_runtime.toml` 中：
  - [x] `imu_source_topic = '/imu'`
  - [x] `imu_topic = '/navlab/slam/imu'`
  - [x] `external_nav_input_odom_topic = '/slam/odom'`
  - [x] `cartographer_odometry_topic != '/odometry'`
- [x] hover rosbag 录制 `/slam/odom`、`/external_nav/odom`、`/navlab/slam/imu`、`/gazebo/model/odometry`。
- [x] `/gazebo/model/odometry` 只出现在 rosbag/gate/review，不出现在 runtime 控制输入配置。
- [x] `mavlink_external_nav` runtime command 使用 `max_horizontal_speed_mps=0.25`、`max_yaw_rate_radps=0.6`；真实 `/mavlink_external_nav/status` 仍需下一次实跑后复核。

## Phase 1：修 Cartographer 非 truth SLAM 基础链

目标：让 Cartographer 在不吃 Gazebo odom、不吃官方 map 的情况下，稳定输出可信 `/slam/odom`。

状态：SLAM-only 验证已通过一次；尚未接入 FCU 起飞链，不能视为 hover mission 成功。

当前通过证据：`artifacts/sim/hover-slam-only/20260618T070431Z`。该 run 没有启动 `hover_mission`、`fcu_controller`、`mavlink_external_nav` 或 `official_maze_overlay`；`summary.ok=true`，`slam_only_probe.ok=true`，rosbag 中 `/imu=27850`、`/navlab/slam/imu=27850`、`/scan=284`、`/slam/odom=6484`、`/external_nav/odom=0`、`/ap/v1/pose/filtered=0`。

### TODO

- [x] 恢复 hover Cartographer backend 启动。
- [x] hover Cartographer config 保持：
  - [x] `use_odometry = false`
  - [x] 不订阅 Gazebo `/odometry`
  - [x] 不订阅 `/gazebo/model/odometry`
  - [x] 不加载官方 maze map、pbstream、known-map scan matcher 或 scan-to-map localizer 作为初始/闭环定位来源
  - [x] `use_imu_data = true`
- [x] Cartographer TF 隔离在 `/navlab/slam/tf`，不直接污染全局 `/tf`。
- [x] `navlab_cartographer_adapter` 从 `/navlab/slam/tf` 生成 `/slam/odom`。
- [x] 检查 `/navlab/slam/imu` 时间戳是否单调，避免再出现 `Non-sorted data added to queue: '(0, imu)'`。
- [ ] 检查 `/scan` frame、`base_scan`、`imu_link`、`base_link` TF 是否稳定。
- [x] 先做不接 FCU 的 SLAM-only 验证：只启动传感器、SLAM、rosbag，不让 `/slam/odom` 进入 ArduPilot EKF。
- [x] 在 SLAM-only 验证中，量化 `/slam/odom` 的平滑漂移、跳变、yaw drift、频率和 stale age。

### 验收

- [x] `slam_backend.runtime.log` 无 Cartographer fatal。
- [x] `slam_backend.runtime.log` 无 `Non-sorted data added to queue`。
- [x] `/slam/odom` 消息数大于 0，频率满足 external-nav 最低要求。
- [x] `/navlab/slam/status` 为 ready/healthy，且没有 stale/jump。
- [x] SLAM-only 验证通过前，不允许把 `/slam/odom` 接入 external-nav/FCU 起飞链。
- [x] 只跑短时地面/低风险检查时，`/slam/odom` 不出现米级瞬时跳变。
- [ ] 如果 SLAM-only 阶段已经出现平滑但明显不可信的水平漂移，应停止进入 Phase 3/4，并记录为可观测性失败。

## Phase 2：增加 SLAM 质量门控，禁止坏 odom 拉飞

目标：不是有 `/slam/odom` 就发给 FCU；必须先判断 SLAM 是否可信。

状态：源码、生成参数、hover mission 决策、单元测试、正式 slam runtime 镜像 rebuild 和 SLAM-only 前置验证已完成；当前不能直接进入真实 hover，因为低可观测场景下 `/external_nav/status` 保守输出 `slam_quality=uncertain`，`slam_quality_good=false`。

当前实现证据：`navlab_external_nav_bridge_node.cpp` 输出 `slam_quality`、`slam_quality_reason`、`slam_quality_good` 和 `slam_quality_report`，只在 quality 为 `good` 时发布 `/external_nav/odom`；hover 生成的 `external_nav_bridge_params.yaml` 开启 `require_imu_for_quality=true`、`require_scan_for_quality=true`、`low_observability_mode=true`。dry-run：`/tmp/navlab-phase2-quality-check/hover/20260618T073905Z`。正式镜像验证：`navlab/slam-cartographer:jazzy-latest` image id `sha256:bce9daf3e749a60b536a76f8b00323f914ce53f3e1063d702b0d63d5ce509f70`，`hover-slam-only` run `artifacts/sim/hover-slam-only/20260618T170255Z` 真实看到 `external_nav_quality_fields_ok=true`，`/external_nav/status` 中 `scan.present=true`、`scan.fresh=true`、`scan.rate_hz=7`、`slam_quality="uncertain"`、`slam_quality_reason="low_observability_horizontal_span"`、`slam_quality_good=false`。

### TODO

- [x] 在 `navlab_cartographer_adapter` 或 external-nav 前增加质量判断。
- [x] 质量判断只能用非 truth 信息：
  - [x] SLAM TF/odom rate
  - [x] scan rate
  - [x] IMU rate
  - [x] odom jump
  - [x] yaw jump
  - [ ] covariance 或内部 score，如果可用
  - [x] stale age
- [x] 明确质量门控的局限：仅靠 rate/stale/jump 无法发现“平滑但错误”的 odom。
- [x] 对“平滑但错误”的风险采用保守策略：
  - [x] 如果没有非 truth 独立水平观测源，不允许质量门控声称该类错误可被完全检测。
  - [x] 对低可观测场景，quality 必须降级为 `uncertain`，mission 不得把 `uncertain` 当 success。
- [x] 质量差时 external-nav 不发布 `/external_nav/odom` 或标记 `ready=false`。
- [x] hover mission 必须等待 `slam_quality=good` 后才能 arm/takeoff。
- [x] 如果飞行中 SLAM quality 变差，mission 不能进入 hover success；应 fail 或触发安全降落策略。

### 验收

- [x] `/external_nav/status` 明确包含 SLAM quality 字段或 reason。
- [x] SLAM 跳变时不会继续向 FCU 发送“可信 external-nav”。
- [x] SLAM quality 至少包含 `good`、`uncertain`、`bad`、`stale`、`jump`，其中只有 `good` 允许进入 takeoff/hover success。
- [x] `hover_mission` 不允许在 SLAM quality bad/stale/jump 时进入 `hover_hold`。
- [x] `hover_mission` 不允许在 SLAM quality uncertain 时进入 `hover_hold`。
- [x] 单元测试覆盖：
  - [x] stale odom
  - [x] low rate odom
  - [x] frame mismatch
  - [x] pose jump
  - [x] uncertain low-observability odom
  - [x] healthy odom

## Phase 3：外部导航和 EKF 参数收敛

目标：让 ArduPilot EKF 正确融合 external-nav，而不是被抹平或被跳变拉飞。

### TODO

- [ ] 保留 `VISO_TYPE=1`。
- [ ] 确认 `EK3_SRC1_POSXY=6`。
- [ ] 默认禁用 ExternalNav velocity fusion，除非 `/slam/odom.twist` 已通过独立测试证明可信。
  - [ ] 默认目标：`EK3_SRC1_VELXY=0` 或等价禁用策略。
  - [ ] 只有在 twist rate、方向、幅值均通过 MCAP 验证后，才允许改为 `EK3_SRC1_VELXY=6`。
- [ ] 重新评估 `EK3_SRC1_YAW`：
  - [ ] 如果 SLAM yaw 稳定，可用 ExternalNav yaw。
  - [ ] 如果 SLAM yaw 不稳，先保留 compass/其他 yaw，不让 yaw 错误拉飞。
- [ ] external-nav 发送器保留合理限速，但不能低到抹平真实漂移。
- [ ] external-nav 限速必须有明确 baseline：
  - [ ] 水平限速不能小于任务允许真实漂移速度的 2 倍。
  - [ ] 水平限速不能大到把单帧米级跳变直接打进 FCU。
  - [ ] 每轮 run 记录 `max_horizontal_speed_mps` 和 `max_yaw_rate_radps`。
- [ ] 记录每轮 `last_sent_x/y`、`/external_nav/odom`、`/ap/v1/pose/filtered` 的差异。

### 验收

- [ ] `/slam/odom`、`/external_nav/odom`、`/ap/v1/pose/filtered` 在非跳变情况下趋势一致。
- [ ] external-nav 不再把 1m+ SLAM 位移压成 0.01m 级别。
- [ ] external-nav 不再把瞬时错误跳变直接全量打进 FCU。
- [ ] 若 `EK3_SRC1_VELXY=6`，必须附带 `/slam/odom.twist` 验证结果；否则视为未通过。
- [ ] hover 前 FCU local pose ready 且 external-nav ready。

## Phase 4：短 hover 实跑

目标：先证明起飞和短时 hover 真实物理稳定，不急着 landing。

前置条件：Phase 0-3 全部通过。尤其是 SLAM-only 验证和 SLAM quality gate 必须通过；不能在 SLAM 质量未证明时直接起飞试错。

### TODO

- [ ] 跑短 hover，例如 5-8 秒 hold。
- [ ] 录制 MCAP：
  - [ ] `/gazebo/model/odometry`
  - [ ] `/slam/odom`
  - [ ] `/external_nav/odom`
  - [ ] `/ap/v1/pose/filtered`
  - [ ] `/rangefinder/down/range`
  - [ ] `/height/estimate`
  - [ ] `/navlab/hover/status`
  - [ ] `/navlab/slam/status`
  - [ ] `/external_nav/status`
  - [ ] `/mavlink_external_nav/status`
- [ ] 跑完后先读 MCAP 数字，不上传、不说成功。

### 验收

- [ ] takeoff ACK 成功。
- [ ] rangefinder/external_nav/FCU 高度一致。
- [ ] Gazebo model horizontal drift <= 0.10m。
- [ ] `/slam/odom` 与 Gazebo review-only drift 的对比必须量化：
  - [ ] 同一 hover 窗口内，`slam_horizontal_drift_m / gazebo_horizontal_drift_m` 在 `[0.5, 2.0]` 内；如果 Gazebo drift 小于 `0.10m`，则改用绝对误差。
  - [ ] 同一 hover 窗口内，SLAM 与 Gazebo 水平位移向量夹角 <= `60deg`；若两者 drift 都小于 `0.10m`，该项可跳过。
  - [ ] 同一 hover 窗口内，`abs(slam_x_span - gazebo_x_span) <= 0.50m` 且 `abs(slam_y_span - gazebo_y_span) <= 0.50m`。
  - [ ] 注意：Gazebo 只用于验收对比，不进入 runtime 控制。
- [ ] hover_drift 不靠 FCU local-z 单独判定。
- [ ] 没有 crash_detected。

## Phase 5：完整 hover + landing

目标：完成正式 hover mission。

当前状态：hover 本体已有正向证据，但完整任务仍未成功。最新真实 run `artifacts/sim/hover/20260619T032621Z` 中 `takeoff_ack_ok=true`、`hover_altitude_crosscheck.ok=true`、`hover_drift.ok=true`，但 top-level 仍 `blocked`。本轮已按官方 LAND mode 接管原则把 AP LAND 下降速度 profile 改成审计，不再用项目 `0.25m/s` gate 阻断；仍不能把局部 hover/landing 字段包装成完整成功，required probes 和非作弊 gate 仍必须通过。

实现状态：landing descent profile 的孤立 rangefinder outlier/validity 处理已经实现并通过单元测试；尚未经过真实 hover artifact 验证，所以 Phase 5 仍不能标记完成。

### Phase 5A：按官方 LAND 模式重做 arm 到 disarm 的完整闭环

目标：把当前“GUIDED 里自行慢降，touchdown 后才发 LAND”的 landing 逻辑，改成更贴近 ArduPilot 官方语义的闭环：hover 成功后尽早发送 `MAV_CMD_NAV_LAND` / 进入 LAND mode，由 FCU landing controller 完成下降、landed detection、motor stop/disarm；companion 只做前置条件、状态监控、安全超时和 artifact 证据，不再在近地阶段持续用 GUIDED z setpoint 把飞机压向地面。

当前代码对照：`hover_mission.py` 的 hover 主状态由 `decide_hover(...)` 产生，状态包括 `wait_ready`、`guided`、`arm`、`takeoff`、`hover_settle`、`hover_hold`、`complete`、`abort`；当前 landing 子状态在 `_tick_landing(...)` 中，包括 `task_body_complete`、`pre_land_hold`、`guided_descent`、`land_command_sent`、`descent_monitoring`、`touchdown_candidate`、`disarm_requested`、`landing_complete`。A 方案必须保留前半段 hover 状态机，但删除正常路径中的 `guided_descent` 作为下降控制阶段。

#### 官方依据审查

这部分是对 Phase 5A 的“是不是官方支持路径”的审查结论。结论分两类：ArduPilot 官方语义支持的内容可以进入正式实现；项目自定义验收/防作弊内容必须保留为本项目 gate，但不能伪装成 ArduPilot 官方行为。

官方支持的行为：

- `MAV_CMD_NAV_LAND`：ArduPilot 官方 Guided Mode MAVLink 文档明确列出 `MAV_CMD_NAV_LAND`，并说明它会切到 Land mode。依据：https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- `MAV_CMD_NAV_TAKEOFF` 和 `SET_POSITION_TARGET_LOCAL_NED`：同一官方文档列出 Guided mode 可处理 takeoff 和位置/速度/姿态目标；位置坐标是 NED，z 为正向下。依据：https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- Land mode 下降策略：官方 Land Mode 文档说明高处用 `LAND_SPD_HIGH_MS` 或 `WP_SPD_DN`，在有 rangefinder/terrain 时到 `LAND_ALT_LOW_M` 下切到 `LAND_SPD_MS`；落地后会自动关电机并 disarm。依据：https://ardupilot.org/copter/docs/land-mode.html
- Landed detection：官方 Land Mode 文档列出了 landed 条件，包括低电机输出、油门最小、姿态误差限制、近零垂直速度、健康 rangefinder 且高度低于 2m 等。依据：https://ardupilot.org/copter/docs/land-mode.html
- bounce/近地异常排查：官方 Land Mode 文档建议 bounce/balloon 时降低 `LAND_SPD_MS`，近地 erratic 时检查 barometer 是否受 prop-wash 影响，并用日志 altimeter spike/oscillation 验证。依据：https://ardupilot.org/copter/docs/land-mode.html
- arm/disarm/force disarm：官方 MAVLink arming 文档说明 `MAV_CMD_COMPONENT_ARM_DISARM` 的 `param1=1/0` 表示 arm/disarm，`param2=21196` 是 force arm/disarm。依据：https://ardupilot.org/dev/docs/mavlink-arming-and-disarming.html
- External Navigation：官方 Non-GPS Position Estimation 文档说明 companion 可用 `ODOMETRY` 等 MAVLink 消息把外部位置/速度估计送入 EKF，并给出 `EK3_SRC1_POSXY=6`、`EK3_SRC1_VELXY=6/0`、`EK3_SRC1_POSZ=6/1`、`EK3_SRC1_YAW=6/1` 等配置方向。依据：https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
- Rangefinder 参与高度控制：官方 Rangefinder 文档说明下视 rangefinder 会在带高度控制的模式中自动用于近地高度控制，直到超过 `RNGFNDx_MAX` 后切回 barometer。依据：https://ardupilot.org/copter/docs/common-rangefinder-landingpage.html
- 不把 rangefinder 强塞进 EKF Z：官方 Terrain Following 文档警告不要把 `EK3_SRC1_POSZ` 设成 Rangefinder，不要设置 `EK3_RNG_USE_HGT`；EKF Source 文档也说 `EK3_SRC1_POSZ=RangeFinder` 几乎不应使用，只适合平坦室内且无杂物。依据：https://ardupilot.org/copter/docs/terrain-following.html 和 https://ardupilot.org/copter/docs/common-ekf-sources.html

项目自定义但必须保留的验收/防作弊规则：

- `slam_quality_good=true` 才允许进入 arm/takeoff/hover hold：这是本项目为了避免坏 `/slam/odom` 拉飞 FCU 的质量门控，不是 ArduPilot 内建 landed/takeoff 条件。
- 不用 Gazebo truth、官方 maze map、fixed XY prior、Foxglove display TF 作为 runtime 输入：这是本项目“非作弊 hover”的边界，不是 ArduPilot 官方限制。
- Gazebo drift gate、MCAP/summary artifact 必须通过；`max_landing_descent_rate_mps=0.25` 只保留给 `guided_descent` diagnostic/rollback 和 AP LAND 审计字段，不再阻断 `ap_land_mode_after_hover` 成功，因为官方 LAND mode 已按 `LAND_SPD_MS`/`LAND_ALT_LOW_M` 接管下降。

审查结论：Phase 5A 的核心控制路径“GUIDED 前置起飞/悬停，hover 证据冻结后发送 `MAV_CMD_NAV_LAND`，随后由 ArduPilot Land mode 负责下降、落地检测、自动 motor stop/disarm，companion 只监控并记录证据”有官方文档支持。SLAM gate、防作弊边界、required probes、Gazebo drift gate 仍是项目安全/验收合同；AP LAND 下降速度不再使用项目 0.25m/s 阈值阻断成功，只审计记录。

#### 全任务实施规则

后续整个 hover task 必须按下面的 FSM 做代码、测试和真实 run，不再把“修一个局部函数”当成完整任务：

- 每个代码 TODO 必须标明落在哪个 FSM 状态或状态转换上，例如 `S7->S8 command_land`、`S9 land_mode_monitor`。
- 每个单测必须验证一个状态转换、一个 guard 或一个 fail/blocker，不再只测孤立 helper。
- 每次真实 run 的 summary 必须能还原 S0 到 S13 的状态时间线；如果中断，必须停在明确的 `S_abort` blocker。
- 正常成功路径只能是 `S0->...->S13`；`guided_descent` 只能作为 diagnostic/rollback policy，不能参与正式 success path。
- Phase 5A 实现完成并通过定向单测前，不再继续跑完整 hover 试错。

#### A 方案完整有限状态机

```text
S0 wait_runtime
  entry: 连接 MAVLink/ROS，启动 runtime services/probes/rosbag。
  guard: target_system/target_component 已知，summary/artifact 路径可写。
  next: S1 wait_nav_ready。
  fail: target_missing、runtime_timeout。

S1 wait_nav_ready
  entry: 等待 external-nav、mavlink_external_nav、FCU local position、IMU、SLAM quality。
  guard: slam_quality_good=true，external_nav_ready=true，mavlink_external_nav_ready=true，fcu_local_position_ready=true，imu_ready=true，并持续 preflight_ready_sec。
  action: 不 arm、不 takeoff、不发 hover hold setpoint。
  next: S2 set_guided。
  fail/abort: quality bad/stale/jump、external_nav stale、IMU stale。

S2 set_guided
  entry: 发送 GUIDED mode set_mode。
  guard: expected_mode_seen=true，mode_number=GUIDED。
  next: S3 arm。
  retry: 每 1s 重发 set_mode，直到 timeout。
  fail: guided_mode_not_accepted。

S3 arm
  entry: command_arm(force_arm 配置保持现状)。
  guard: armed_seen=true。
  next: S4 takeoff。
  retry: 每 2s 重发 arm。
  fail: arm_rejected、disarmed_after_airborne。

S4 takeoff
  entry: command_takeoff(takeoff_alt_m)。
  guard: takeoff_ack_ok=true，或 external_nav/rangefinder 独立高度同时证明达到 takeoff_alt_m 窗口。
  next: S5 hover_settle。
  retry: 每 2s 重发 takeoff，直到 airborne_seen。
  fail: takeoff_rejected、only_fcu_local_z_airborne。

S5 hover_settle
  entry: airborne_seen 后等待 hover_settle_sec；持续检查 SLAM/external-nav/FCU local/GUIDED 未丢失。
  guard: airborne_elapsed_sec >= hover_settle_sec，rangefinder/external_nav 独立高度达到目标。
  next: S6 hover_hold。
  fail: guided_mode_lost_after_airborne、slam_quality_lost_after_airborne、external_nav_lost_after_airborne、fcu_local_position_lost_after_airborne。

S6 hover_hold
  entry: 锁定 hold_x/hold_y/hold_yaw，按 hover_hold_sec 发送 position hold setpoint。
  guard: hover_altitude_crosscheck.ok=true，hover_drift.ok=true，hover_body_ok=true，crash_detected=false。
  next: S7 pre_land_hold。
  fail: hover_unstable、height_crosscheck_failed、hover_drift_failed。

S7 pre_land_hold
  entry: 发布 landing intent；保持当前位置 pre_land_hold_sec，让机体/估计稳定。
  guard: hover evidence 已冻结并写入 summary，仍 armed，仍有 FCU target。
  action: 只允许发送 hold setpoint，不允许继续向下压 z。
  next: S8 command_land。
  fail: landing_target_system_missing、nav_source_lost_before_land。

S8 command_land
  entry: 立即发送 MAV_CMD_NAV_LAND，不再进入 GUIDED guided_descent。
  guard: land_command_accepted=true；若能读到 mode，则要求 mode 切到 LAND 或 landed state 开始变化。
  action: 记录 land_command_sent_time、land_command_ack、LAND mode seen、FCU landing 参数快照。
  next: S9 land_mode_monitor。
  retry: 每 2s 重发 MAV_CMD_NAV_LAND，直到 accepted 或 timeout。
  fail: landing_command_rejected、land_mode_not_seen_timeout。

S9 land_mode_monitor
  entry: companion 不再发送 GUIDED 下降 setpoint；只监控 FCU LAND 下降。
  guard: rangefinder/external_nav/FCU/Gazebo review-only 证据显示下降中；descent_profile 继续审计真实下降速度。
  action: 记录 LAND_SPD/LAND_ALT_LOW 相关参数、rangefinder raw/relative、FCU vz、landed_state、mode。
  next: S10 touchdown_monitor。
  fail: landing_descent_too_fast、landing_timeout、mode_left_land、crash_detected。

S10 touchdown_monitor
  entry: 等待 landed_state_on_ground 或 rangefinder relative height + vertical speed 持续 touchdown_confirm_sec。
  guard: touchdown_confirmed=true。
  action: 不用 FCU local-z 单独确认触地；rangefinder 有效时优先 rangefinder/external evidence。
  next: S11 disarm_monitor。
  fail: touchdown_not_confirmed、landing_timeout。

S11 disarm_monitor
  entry: 优先等待 FCU Land mode 自动 disarm/motor safe。
  guard: disarmed=true，motors_safe=true。
  action: 如果超过 disarm grace 且 touchdown_confirmed=true，可使用当前已有 force_disarm_after_touchdown 作为兜底，并必须在 summary 标记 `force_disarm_used=true`。
  next: S12 landing_complete。
  fail: disarm_not_confirmed、motors_not_safe。

S12 landing_complete
  entry: 写 mission_summary，发布 final landing status，停止 vehicle。
  guard: hover ok + landing.ok=true + required probes ok。
  next: S13 task_success。
  fail: probe_failed、summary_gate_failed。

S_abort
  entry: 任意 airborne 后丢 GUIDED/SLAM/external-nav/FCU local、crash、timeout、command rejected。
  action: 写明确 blocker；若仍 armed，按安全策略 LAND 或 disarm；不得把局部 hover evidence 包装成 success。
```

#### A 方案 TODO

- [ ] `S0/S1` 增加 mission FSM 记录结构：每个状态 entry time、exit time、guard 结果、fail reason 都写入 `/navlab/hover/status`、`/navlab/landing/status` 和最终 summary。
- [ ] `S7/S8` 增加 landing policy 开关，例如 `landing_policy=ap_land_mode_after_hover`，默认用于 hover mission；保留旧 `guided_descent` 仅作 diagnostic/rollback，不能作为正式成功路径。
- [ ] `S7->S8` 修改 `_tick_landing(...)` 正常路径：`pre_land_hold` 后直接进入 `command_land`，发送 `MAV_CMD_NAV_LAND`；正常路径不再调用 `_send_landing_descent_setpoint(...)`。
- [ ] `S8/S9` 新增 LAND mode 观测字段：
  - [ ] `land_command_sent=true`
  - [ ] `land_command_accepted=true`
  - [ ] `land_mode_seen=true/false/unknown`
  - [x] `land_mode_seen_elapsed_sec`
  - [x] `mode_before_land`
  - [x] `mode_after_land`
  - [x] `landed_state_timeline`
- [ ] `S8/S9` 新增 FCU landing 参数审计，不用于 runtime cheat，只用于解释 artifact：
  - [ ] 记录实际支持的 `LAND_SPEED`/`LAND_SPD_MS`
  - [ ] 记录实际支持的 `LAND_SPEED_HIGH`/`LAND_SPD_HIGH_MS`
  - [ ] 记录 `LAND_ALT_LOW_M` 或等价参数
  - [ ] 记录 rangefinder 是否进入 FCU landing/terrain/surface tracking 相关配置，但不得把 `EK3_SRC1_POSZ` 改成 Rangefinder。
- [ ] `S10/S11` 修改 landing summary：
  - [ ] 区分 `landing_controller="ap_land_mode"` 与 `landing_controller="guided_descent"`
  - [ ] 区分 `auto_disarm_by_land_mode` 与 `force_disarm_after_touchdown`
  - [ ] 若使用 force disarm，必须写 `force_disarm_used=true`，不得伪装成官方 LAND 自动 disarm。
- [ ] `S9/S10` 修改 descent profile 解释：
  - [x] 在 `ap_land_mode` 下，descent profile 只作为 `descent_profile` 审计，不再写 `landing_descent_too_fast` blocker。
  - [x] `guided_descent` diagnostic/rollback 路径仍保留真实连续过快下降失败判据。
  - [x] 若官方 LAND 参数如 `LAND_SPD_MS=0.5` 高于项目旧 `0.25m/s`，以官方 LAND mode 为准，项目阈值只记录审计，不阻断。
- [ ] `S12/S13` 修改最终 summary gate：只有 hover ok、landing ok、required probes ok、无禁止输入时才允许 `summary.ok=true`；probe killed 必须停在 `probe_failed`，不能被 landing 局部成功覆盖。
- [ ] 单测覆盖 FSM：
  - [ ] `S7->S8`：pre_land_hold 后直接发 `MAV_CMD_NAV_LAND`。
  - [ ] `S8`：A 方案正常路径不调用 `_send_landing_descent_setpoint`。
  - [ ] `S8->S_abort`：land command rejected 会 fail。
  - [ ] `S10->S11`：touchdown 后优先等待自动 disarm；force disarm 只在 touchdown confirmed + grace timeout 后兜底。
  - [ ] `S12`：landing summary 包含 controller、mode、LAND 参数、force disarm 使用情况。
  - [x] `S12->S13`：required probe killed 时不能 success。
- [ ] 真实 run 验收按 FSM：
  - [ ] artifact 中能看到 `S8 land_command_sent=true` 和 `land_command_accepted=true`。
  - [ ] 若可观测 mode，能看到 `S2 GUIDED -> S8/S9 LAND`。
  - [ ] `S6` hover 本体仍满足 `takeoff_ack_ok=true`、`hover_altitude_crosscheck.ok=true`、`hover_drift.ok=true`。
  - [ ] landing 若失败，必须能分辨是 `S8` LAND command/mode 失败、`S9` LAND 模式下降过快、`S10` touchdown 失败、`S11` disarm 失败、`S12` required probe killed。

### TODO

- [x] landing descent profile 的 rangefinder outlier/validity 处理已经实现并有单测；后续不再继续扩大 summary filter 来掩盖真实快速下降。
- [x] 检查下降速度和 touchdown 的单元测试已覆盖：单点/高平台 outlier、连续真实快速下降、rangefinder 缺失 fallback、touchdown/bounce 优先 rangefinder relative height。
- [x] 按 Phase 5A FSM 实现 `ap_land_mode_after_hover`，这是当前唯一代码下一步。
- [x] Phase 5A 定向单测通过后，再恢复完整 hover hold 秒数并执行真实完整 landing run；真实 run `artifacts/sim/hover/20260619T032621Z` 已执行但失败。
- [x] 检查 motor safe/disarm 必须落到 `S11 disarm_monitor`，并区分官方 LAND 自动 disarm 与 touchdown 后 force-disarm 兜底；本次 run 为 `force_disarm_used=true`、`auto_disarm_by_land_mode=false`。
- [x] 已按官方 LAND mode 接管原则处理 `S9/S10` LAND-mode descent profile 与项目 `0.25m/s` gate 的冲突：AP LAND 下速度 profile 不再阻断，旧 gate 只保留为审计/diagnostic。
- [ ] 构建 Foxglove artifact 仅用于复查，不用于改变结论。
- [ ] `S12` landing 通过后单独修 `slam_hover_probe` killed：
  - [ ] 不允许因为 `mission_summary` 局部 hover/landing 证据变好就忽略 required probe。
  - [x] 检查 `slam_hover_probe.json` 是否为 0 bytes、`slam_hover_probe.log` 是否只有 DDS/type-hash warning、probe container 是否被 signal killed；本次顶层 summary 记录 probe 被 `signal: killed`，但后续文件变为可解析 JSON，`ok=false`，blockers 为 `topic_sample_missing:/navlab/landing/status` 和 `topic_sample_missing:/ap/v1/pose/filtered`。
  - [ ] 优先降低 probe 对 runtime 的资源压力，例如缩短 probe duration、减少订阅/采样压力或改为读取 rosbag/summary 采样。
  - [ ] 如果改为 rosbag/summary 采样，必须仍从真实 artifact 提取 status，不能伪造 `/external_nav/status` 或绕过 required probe。

### 验收

- [ ] `summary.ok=true`。
- [ ] `blockerCodes=[]` 或无 hover/landing blocker。
- [ ] `takeoff_ack_ok=true`。
- [ ] `hover_altitude_crosscheck.ok=true`。
- [ ] `hover_drift.ok=true`。
- [ ] Gazebo model horizontal drift <= 0.10m。
- [ ] `landing.ok=true`。
- [ ] `touchdown_confirmed=true`。
- [ ] `motors_safe=true`。
- [x] `ap_land_mode_after_hover` 下 `landing_descent_too_fast` 不再阻断；descent profile 仍记录 raw speed/outlier 审计。
- [x] 人为构造或测试中的连续真实过快下降在 `guided_descent` diagnostic 路径仍触发项目 gate。
- [ ] `slam_hover_probe` required check 真实产出非空结果，不能被 killed，也不能被标记跳过。
- [ ] Foxglove 里 robot/map/scan 一致，但 Foxglove 只作为人工复查，不作为唯一证据。

## 每轮 run 后必须产出的证据

每次运行必须记录到 `docs/notes/hover_hold_gate_20260617.md` 或本文件追加日志：

- run id
- command
- status
- blockerCodes
- takeoff ACK
- hover 高度交叉检查
- Gazebo model horizontal drift
- `/slam/odom` drift
- `/external_nav/odom` drift
- `/ap/v1/pose/filtered` drift
- landing 结果
- 是否使用任何 diagnostic/truth/replay-only 输入

## 当前立即下一步

只做 Phase 5A 的 FSM 代码实现和定向测试。

第一步不是继续调 hover 高度、landing 阈值、Gazebo drift gate、Foxglove 显示补丁或全局 runtime TF，也不是直接重跑完整 hover 试错。下一步必须把 `_tick_landing(...)` 正常路径改为 `S7 pre_land_hold -> S8 command_land -> S9 land_mode_monitor -> S10 touchdown_monitor -> S11 disarm_monitor -> S12 landing_complete`，并让 summary/status 能还原这条状态线。定向单测通过后，才允许真实 hover run；如果真实 run 失败，必须按 FSM blocker 定位，不再说“局部 hover 看起来成功所以任务成功”。

## 执行纪律

- 每完成一个子步骤，必须追加到 `docs/notes/hover_hold_gate_20260617.md`，写清楚改了什么、为什么改、验证了什么。
- Phase 0 没过之前不跑“完整 hover 成功宣称”；最多跑生成 runtime artifact/计划文件的定向检查。
- Phase 1 SLAM-only 没过之前，不允许把 `/slam/odom` 接进 FCU 并起飞试错。
- 任意 run 如果用了 Gazebo truth、known map、replay-only topic 或 display TF 作为 runtime 输入，必须标记为 diagnostic，不得作为 hover success。
