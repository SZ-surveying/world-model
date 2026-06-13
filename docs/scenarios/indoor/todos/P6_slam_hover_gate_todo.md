# P6 真实 SLAM hover gate TODO

## 目标

在 P0-P5 已通过的基础上，验收真实 SLAM `/slam/odom` 通过 ExternalNav 进入 ArduPilot EKF 后，FCU 是否能在官方 maze/Iris 场景中稳定悬停。P6 完成后，只能说明真实 SLAM hover gate 已通过；不能说明小范围运动、避障、探索或 NavLab 自定义 world/model 迁移已完成。

设计文档：

- `docs/scenarios/indoor/navlab_p6_slam_hover_gate_design.md`

前置条件：

- `docs/scenarios/indoor/todos/P0_official_baseline_todo.md` 已通过。
- `docs/scenarios/indoor/todos/P1_official_maze_x2_todo.md` 已通过。
- `docs/scenarios/indoor/todos/P2_rangefinder_imu_todo.md` 已通过。
- `docs/scenarios/indoor/todos/P3_slam_backend_quality_todo.md` 已通过。
- `docs/scenarios/indoor/todos/P4_fcu_state_machine_todo.md` 已通过。
- `docs/scenarios/indoor/todos/P5_frame_contract_todo.md` 已通过。

总 roadmap：

- `docs/scenarios/indoor/navlab_master_roadmap.md`

## P6.0 文档和边界

任务：

- [x] 新增 P6 设计文档。
- [x] 新增 P6 TODO 文档。
- [x] 在 `docs/README.md` 中加入 P6 design / TODO 入口。
- [x] 在 master roadmap 中把 P6 TODO 从“待建”改成具体文档。
- [x] 文档明确 P6 是 hover gate，不是 P7 小范围运动 gate。
- [x] 文档明确 P6 不是 P8 探索 gate。
- [x] 文档明确 P6 不替换 NavLab 8 字形 world/model。
- [x] 文档明确 Gazebo truth 只能作为诊断，不能作为 SLAM、ExternalNav、规划或控制输入。
- [x] 文档明确 P6 不允许 direct set pose。
- [x] 文档明确 P6 必须保持 P4 唯一 setpoint owner。

验收：

- [x] P6 文档中没有把 Gazebo truth 当作控制来源。
- [x] P6 文档中没有把 direct set pose 当作允许路径。
- [x] P6 文档中明确 `hover_claim=evaluated`。
- [x] P6 文档中明确 `exploration_claim=not_evaluated`。

## P6.1 配置

任务：

- [x] 新增 P6 SLAM hover 配置段。
- [x] 配置 rosbag profile，默认 `docker/profiles/navlab-slam-hover-rosbag-topics.txt`。
- [x] 配置 SLAM odom topic，默认 `/slam/odom`。
- [x] 配置 SLAM status topic，默认 `/navlab/slam/status`。
- [x] 配置 ExternalNav status topic，默认 `/external_nav/status`。
- [x] 配置 FCU pose topic，默认 `/ap/v1/pose/filtered`。
- [x] 配置 FCU twist topic，默认 `/ap/v1/twist/filtered`。
- [x] 配置 FCU status topic，默认 `/ap/v1/status`。
- [x] 配置 FCU command topic，默认 `/ap/v1/cmd_vel`。
- [x] 配置 rangefinder topic，默认 `/rangefinder/down/range`。
- [x] 配置 IMU topic，默认 `/imu`。
- [x] 配置 Gazebo truth diagnostic topic，默认 `/odometry`。
- [x] 配置 hover status topic，默认 `/navlab/hover/status`。
- [x] 配置 hover window、settle window 和 final hold window。
- [x] 配置 drift、altitude、yaw、rate 和 latest age 阈值。
- [x] 配置 `uses_gazebo_truth_as_input=false`。

验收：

- [x] 配置可从 `orchestration/config.toml` 加载。
- [x] P6 runtime config 写入 artifact 并记录 hash。
- [x] 配置中如果允许 Gazebo truth 作为输入，doctor blocked。
- [x] 配置中如果 SLAM odom topic 不是 `/slam/odom` 或等价 allowed topic，doctor blocked。

## P6.2 ExternalNav bridge gate

任务：

- [x] 固化 ExternalNav bridge 输入必须来自 `/slam/odom`。
- [x] ExternalNav bridge 记录 input count、sent count、rate、latest input age 和 latest sent age。
- [x] ExternalNav bridge 输出 `/external_nav/status`。
- [x] 检查 ExternalNav 不读取 `/odometry`。
- [x] 检查 ExternalNav 不读取 Gazebo truth diagnostic。
- [x] 检查 ExternalNav 不读取 FCU fused local position 作为输入。
- [x] 检查 ExternalNav 输出 route 是官方或支持的 ArduPilot EKF ExternalNav route。
- [x] 检查 ExternalNav 发送频率大于阈值。
- [x] 检查 ExternalNav 最新输入和最新发送年龄小于阈值。

验收：

- [x] `external_nav.ok=true`。
- [x] `external_nav.input_topic == "/slam/odom"`。
- [x] `external_nav.sent_count > 0`。
- [x] `external_nav.rate_hz >= min_external_nav_rate_hz`。
- [x] `external_nav.latest_sent_age_sec <= max_latest_age_sec`。
- [x] `external_nav.uses_gazebo_truth_as_input=false`。
- [x] ExternalNav 不 healthy 时，P6 blocked。

## P6.3 SLAM odom gate

任务：

- [x] 启动 P3 SLAM backend。
- [x] 检查 `/slam/odom` topic 存在。
- [x] 检查 `/slam/odom.header.frame_id` 和 `child_frame_id` 符合 P5 frame contract。
- [x] 检查 `/slam/odom` rate 大于阈值。
- [x] 检查 `/slam/odom` latest age 小于阈值。
- [x] 检查 stationary drift 小于阈值。
- [x] 检查 jump / yaw jump 小于阈值。
- [x] 检查 `/navlab/slam/status` healthy。
- [x] summary 记录 SLAM backend 名称、配置路径、config hash 和状态。

验收：

- [x] `slam_odom.ok=true`。
- [x] `slam_odom.topic == "/slam/odom"`。
- [x] `slam_odom.rate_hz >= min_slam_odom_rate_hz`。
- [x] `slam_odom.latest_age_sec <= max_latest_age_sec`。
- [x] `slam_odom.stationary_drift_m <= max_slam_stationary_drift_m`。
- [x] `/navlab/slam/status` message count 大于 0。
- [x] SLAM 不 healthy 时，P6 blocked。

## P6.4 FCU readiness 和 EKF gate

任务：

- [x] 复用 P4 FCU controller readiness gate。
- [x] 检查 `/ap/v1/time` 可收到。
- [x] 检查 mode 可切到 GUIDED。
- [x] 检查 arm 成功。
- [x] 检查 takeoff 成功。
- [x] 检查 `/ap/v1/pose/filtered` 持续输出。
- [x] 检查 `/ap/v1/twist/filtered` 持续输出。
- [x] 检查 local position rate 大于阈值。
- [x] 检查 rangefinder 持续可用。
- [x] 检查 IMU 持续可用。
- [x] 检查 EKF 或等价状态能证明 ExternalNav 被接受。

验收：

- [x] `guided_ok=true`。
- [x] `arm_ok=true`。
- [x] `takeoff_ok=true`。
- [x] `fcu_local_position.ok=true`。
- [x] `fcu_local_position.rate_hz >= min_fcu_local_position_rate_hz`。
- [x] `rangefinder.ok=true`。
- [x] `imu.ok=true`。
- [x] EKF/ExternalNav 接收状态不满足时，P6 blocked。

## P6.5 Hover controller gate

任务：

- [x] 复用 P4 唯一 setpoint owner。
- [x] 支持 P6 hover intent。
- [x] controller 等待 SLAM odom ready。
- [x] controller 等待 ExternalNav healthy。
- [x] controller 等待 FCU local position ready。
- [x] controller 只通过 `/ap/v1/cmd_vel` 或配置允许的唯一输出通道发 hover setpoint。
- [x] controller 输出 `/navlab/hover/status`。
- [x] controller 输出 hover settle、hover hold、final hold 状态。
- [x] controller 未 ready 时拒绝 hover intent 并写明原因。
- [x] 不允许 mission 层直接发布 `/ap/v1/cmd_vel`。

验收：

- [x] `owner.unique=true`。
- [x] `owner.set_pose_count==0`。
- [x] `owner.competing_publishers=[]`。
- [x] `/navlab/fcu/controller/status` message count 大于 0。
- [x] `/navlab/fcu/setpoint/output` message count 大于 0。
- [x] `/navlab/hover/status` message count 大于 0。
- [x] controller readiness 不满足时，P6 blocked。

## P6.6 Hover drift 统计

任务：

- [x] 定义 settle window。
- [x] 定义 hover window。
- [x] 定义 final hold window。
- [x] 在 hover window 内统计 FCU local position 水平漂移。
- [x] 在 hover window 内统计 SLAM odom 水平漂移。
- [x] 在 hover window 内统计 Gazebo truth diagnostic 水平漂移。
- [x] 在 hover window 内统计高度误差。
- [x] 在 hover window 内统计 yaw 漂移。
- [x] 在 final hold window 内统计 stop drift。
- [x] summary 记录每个窗口的起止时间、样本数和误差。

验收：

- [x] `hover.ok=true`。
- [x] `hover.window_sec >= min_hover_window_sec`。
- [x] `hover.horizontal_drift_m <= max_hover_horizontal_drift_m`。
- [x] `hover.altitude_error_m <= max_hover_altitude_error_m`。
- [x] `hover.yaw_drift_rad <= max_hover_yaw_drift_rad`。
- [x] `hover.stop_drift_m <= max_stop_drift_m`。
- [x] hover drift 超阈值时，P6 blocked。

## P6.7 Rosbag 和 Foxglove

任务：

- [x] 新增 P6 rosbag profile。
- [x] required topics 至少包含：
  - `/clock`
  - `/tf`
  - `/tf_static`
  - `/scan`
  - `/imu`
  - `/rangefinder/down/range`
  - `/rangefinder/down/status`
  - `/slam/odom`
  - `/navlab/slam/status`
  - `/external_nav/status`
  - `/ap/v1/time`
  - `/ap/v1/pose/filtered`
  - `/ap/v1/twist/filtered`
  - `/ap/v1/status`
  - `/ap/v1/cmd_vel`
  - `/navlab/fcu/state`
  - `/navlab/fcu/controller/status`
  - `/navlab/fcu/setpoint/intent`
  - `/navlab/fcu/setpoint/output`
  - `/navlab/fcu/owner/status`
  - `/navlab/hover/status`
- [x] optional topics 至少包含：
  - `/map`
  - `/submap_list`
  - `/trajectory_node_list`
  - `/odometry`
  - `/navlab/frame_contract/status`
  - `/navlab/x2/vendor_scan`
  - `/navlab/x2/scan_ideal`
  - `/sim/x2/status`
  - `/rangefinder/down/scan_ideal`
- [x] Foxglove notes 说明 P6 固定参考系使用 `map`。
- [x] Foxglove notes 说明 `/odometry` 只用于诊断对照。
- [x] summary 记录 MCAP 路径。

验收：

- [x] rosbag profile summary 中 required topics 全部存在且 message count 大于 0。
- [x] MCAP 可回放 TF、scan、SLAM odom、FCU pose、rangefinder、ExternalNav status 和 hover status。
- [x] P6 summary 记录 `rosbag_profile.ok=true`。

## P6.8 Acceptance task

任务：

- [x] 新增 `navlab-slam-hover-doctor` orchestration task。
- [x] 新增 `navlab-slam-hover-acceptance` orchestration task。
- [x] orchestration CLI 增加同名 task；历史 justfile 便捷入口已回收。
- [x] acceptance 先验证 P0 official baseline。
- [x] acceptance 再验证 P1 X2 scan gate。
- [x] acceptance 再验证 P2 IMU/rangefinder gate。
- [x] acceptance 再验证 P3 SLAM backend gate。
- [x] acceptance 再验证 P4 FCU controller gate。
- [x] acceptance 再验证 P5 frame contract gate。
- [x] acceptance 启动 P6 hover probe/controller。
- [x] summary 包含 `p6_slam_hover` section。
- [x] summary 包含 `slam_odom` section。
- [x] summary 包含 `external_nav` section。
- [x] summary 包含 `fcu` section。
- [x] summary 包含 `hover` section。
- [x] summary 包含 `owner` section。
- [x] summary 包含 `hover_claim=evaluated`。
- [x] summary 包含 `exploration_claim=not_evaluated`。

验收：

- [x] P0 official DDS 条件失败时，P6 blocked。
- [x] P1 X2 `/scan` 条件失败时，P6 blocked。
- [x] P2 IMU/rangefinder 条件失败时，P6 blocked。
- [x] P3 SLAM backend 条件失败时，P6 blocked。
- [x] P4 FCU controller 条件失败时，P6 blocked。
- [x] P5 frame contract 条件失败时，P6 blocked。
- [x] ExternalNav 条件失败时，P6 blocked。
- [x] hover drift 条件失败时，P6 blocked。
- [x] summary 把 P6 hover completion 和 P7/P8 分开。

## P6.9 测试

任务：

- [x] 增加 config 测试：P6 SLAM hover 配置可加载。
- [x] 增加 task registry 测试：P6 doctor/acceptance 已注册。
- [x] 增加 blocker 测试：ExternalNav input 不是 `/slam/odom` 不能通过。
- [x] 增加 blocker 测试：Gazebo truth 作为输入不能通过。
- [x] 增加 blocker 测试：direct set pose 不能通过。
- [x] 增加 blocker 测试：多个 setpoint owner 不能通过。
- [x] 增加 hover drift 测试：水平漂移超阈值不能通过。
- [x] 增加 hover drift 测试：高度误差超阈值不能通过。
- [x] 增加 rosbag profile 测试：P6 required topics 配置存在。

验收：

- [x] P6 相关单元测试通过。
- [x] 不影响 P0 official baseline 测试。
- [x] 不影响 P1 official maze X2 测试。
- [x] 不影响 P2 rangefinder/IMU 测试。
- [x] 不影响 P3 SLAM backend 测试。
- [x] 不影响 P4 FCU controller 测试。
- [x] 不影响 P5 frame contract 测试。

## P6.10 执行顺序

建议执行：

```text
1. uv run --project orchestration python orchestration/main.py official-baseline-acceptance 30
2. uv run --project orchestration python orchestration/main.py official-maze-x2-acceptance 45
3. uv run --project orchestration python orchestration/main.py rangefinder-imu-acceptance 60
4. uv run --project orchestration python orchestration/main.py slam-backend-acceptance 90
5. uv run --project orchestration python orchestration/main.py fcu-controller-acceptance 90
6. uv run --project orchestration python orchestration/main.py frame-contract-acceptance 90
7. uv run --project orchestration python orchestration/main.py slam-hover-doctor
8. uv run --project orchestration python orchestration/main.py slam-hover-acceptance 90
```

验收：

- [x] 每一步失败都能在 summary.blockers 中定位到具体层级。
- [x] 失败不会退回 synthetic TF、fake odom、direct set pose 或 Gazebo truth control 兜底。

## P6 完成标准

P6 全部完成必须满足：

- [x] 官方 `iris_maze` bringup 未被替换。
- [x] P1 X2 `/scan` 链路仍 healthy。
- [x] P2 IMU/rangefinder 机制仍 healthy。
- [x] P3 SLAM backend 仍 healthy。
- [x] P4 FCU controller 和唯一 owner 仍 healthy。
- [x] P5 frame contract 仍 healthy。
- [x] `/slam/odom` 是 ExternalNav 输入。
- [x] ExternalNav 不使用 Gazebo truth。
- [x] ExternalNav 持续 healthy。
- [x] EKF/FCU local position 持续输出。
- [x] GUIDED、arm、takeoff 全部成功。
- [x] hover window 达到最小时长。
- [x] hover horizontal drift 小于阈值。
- [x] hover altitude error 小于阈值。
- [x] hover yaw drift 小于阈值。
- [x] stop drift 小于阈值。
- [x] direct set pose 计数为 0。
- [x] Gazebo truth 没有进入控制、规划、SLAM 或 ExternalNav 输入。
- [x] rosbag required topics 全部有数据。
- [x] summary 明确标注 P6 已评价 hover。
- [x] summary 明确标注 P6 不代表探索完成。

## 验证记录

后续每次验证按下面格式记录：

```text
- 命令：
- 时间：
- artifact：
- 结果：
- blocker：
- 备注：
```

### 2026-06-06 P6 文档初始化

- 命令：文档检查，未运行 acceptance
- 时间：2026-06-06
- artifact：无
- 结果：新增 P6 设计文档和 TODO 文档
- blocker：P6 orchestration task、ExternalNav hover gate、hover rosbag profile 和 acceptance 尚未实现
- 备注：P6.0 文档和边界已完成；后续从 P6.1 配置和 P6.2 ExternalNav bridge gate 开始实现。

### 2026-06-06 P6 实现和验收通过

- 命令：`python3 -m py_compile orchestration/src/config.py orchestration/src/tasks/slam_hover.py orchestration/src/tasks/fcu_controller.py orchestration/src/cli.py orchestration/src/tasks/registry.py`
- 时间：2026-06-06
- artifact：无
- 结果：通过
- blocker：无
- 备注：P6 orchestration、runtime config、rosbag profile 和 CLI 入口可静态加载。

- 命令：`uv run --project orchestration pytest orchestration/tests/test_config.py`
- 时间：2026-06-06
- artifact：无
- 结果：45 passed
- blocker：无
- 备注：覆盖 P6 配置、registry、rosbag profile、truth 输入禁用和 hover gate blocker；P4 owner/direct set pose blocker 继续作为 P6 前置 gate 复用。

- 命令：`uv run --project orchestration python orchestration/main.py slam-hover-doctor`
- 时间：2026-06-06
- artifact：`artifacts/ros/navlab_slam_hover_doctor/20260606_200908/summary.json`
- 结果：`ok=true`，`blocked=false`，`blockers=[]`
- blocker：无
- 备注：doctor 确认 P6 不允许 Gazebo truth 作为控制、规划、SLAM 或 ExternalNav 输入。

- 命令：`uv run --project orchestration python orchestration/main.py slam-hover-acceptance 90`
- 时间：2026-06-06
- artifact：`artifacts/ros/navlab_companion_sitl_gazebo/20260606_201653/summary.json`
- 结果：`ok=true`，`blocked=false`，`blockers=[]`
- blocker：无
- 备注：`/slam/odom -> /external_nav/status -> FCU local position -> hover` 真实 gate 通过；rosbag required topics 全部有数据；`uses_gazebo_truth_as_input=false`；`hover_claim=evaluated`；`exploration_claim=not_evaluated`。hover window 水平漂移约 0.014m，高度误差约 0.040m，yaw 漂移约 0.001rad，ExternalNav rate 约 70.8Hz，FCU local position rate 约 16.4Hz。
