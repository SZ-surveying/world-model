# 真机飞行前 Preflight Doctor TODO

## 目标

在任何真机 hover、P8 exploration 或 P12 scan robustness 飞行入口进入
arm/takeoff 之前，先运行独立的 real preflight doctor，证明当前系统处在
`process + real` 边界内，真实 FCU、真实传感器、真实 SLAM 和基础安全前置条件
可观测，并且没有混入 Gazebo/SITL/X2 virtual serial/SDF overlay 等仿真输入。

设计文档：

- `docs/scenarios/indoor/navlab_real_flight_preflight_doctor_design.md`

前置文档：

- `docs/general/orchestration_runtime_mode_real_vs_sim_design.md`
- `docs/general/orchestration_runtime_backend_guide.md`
- `docs/scenarios/indoor/navlab_unified_landing_sequence_design.md`
- `docs/scenarios/indoor/todos/unified_landing_sequence_todo.md`

适用真机 flight task：

- `hover-real`: `land_in_place`
- `exploration-real` / P8 Stage 2: `return_home_then_land`
- `scan-robustness-real` / P12 Stage 2: `land_in_place`

## RFD.0 文档和边界

任务：

- [x] 新增真机飞行前 preflight doctor 设计文档。
- [x] 新增真机飞行前 preflight doctor TODO 文档。
- [x] 在 `docs/README.md` 中加入 real preflight design / TODO 入口。
- [x] 在 unified landing TODO 的 Stage 2 章节引用本 TODO。
- [x] 文档明确 `just navlab-hover` 只代表 Gazebo/SITL Stage 1，不代表真机 readiness。
- [x] 文档明确 real preflight doctor 不 arm、不 takeoff、不 land、不发布 movement setpoint。
- [x] 文档明确真机 flight task 必须引用最新通过的 real preflight summary。
- [x] 文档明确真实 lidar/rangefinder 不能用 X2 virtual serial 或 Gazebo rangefinder 替代。

验收：

- [x] design 文档只写边界和 contract，不承载 implementation checklist。
- [x] TODO 文档单独承载任务拆分和验收标准。
- [x] README 中能从室内主线导航到 design 和 TODO。
- [x] 后续真机 flight issue / task 应引用 design 和本 TODO。

## RFD.1 Runtime mode 和 CLI 入口

任务：

- [x] 保留现有 `real-preflight-doctor` CLI 入口。
- [x] real preflight 要求 `NAVLAB_RUNTIME_BACKEND=process`。
- [x] real preflight 要求 `NAVLAB_RUNTIME_MODE=real`。
- [ ] 新增或固化 `orchestration/config.real.toml` 示例。
- [ ] 支持配置 real preflight summary 有效窗口，例如 `valid_for_sec`。
- [ ] summary 记录 runtime backend/mode 来源，区分 env override 和 config。
- [ ] 非 `process + real` 组合直接 blocked，不能 fallback 到 Docker。

验收：

- [ ] `NAVLAB_RUNTIME_BACKEND=docker NAVLAB_RUNTIME_MODE=simulation` 时 real preflight blocked。
- [ ] `NAVLAB_RUNTIME_BACKEND=process` 但未设置 `NAVLAB_RUNTIME_MODE=real` 时 blocked。
- [ ] `process + real` 下不会启动 Docker/Gazebo/SITL service。
- [ ] summary 中有 `runtime_backend=process` 和 `runtime_mode=real`。

## RFD.2 真实 topic contract

任务：

- [x] 配置 required real topics。
- [x] 基础 doctor 检查 `/scan`、`/tf`、`/tf_static`、`/slam/odom`、`/ap/v1/status`、`/ap/v1/pose/filtered`。
- [ ] 增加 topic freshness 检查，不能只看 topic name 存在。
- [ ] 增加 topic type 检查，确认 `/scan`、`/slam/odom`、FCU pose/status 类型符合预期。
- [ ] 增加 topic publisher owner/source 摘要，记录每个 required topic 的 publisher。
- [ ] 增加 `/ap/v1/twist/filtered`、`/imu` 的真机 flight readiness 检查。
- [ ] 增加 `/rangefinder/down/range`、`/rangefinder/down/status` 的可选/必选策略。
- [ ] 支持 FCU 内部已接收 rangefinder 但 ROS topic 不存在时的 evidence 字段。

验收：

- [ ] 缺任一 required real topic 时 summary `ok=false`。
- [ ] topic 存在但过期时 summary `ok=false`。
- [ ] topic type 不匹配时 summary `ok=false`。
- [ ] rangefinder 未经 ROS 暴露时不会自动使用仿真 topic 顶替。
- [ ] summary 能说明每个 required topic 的 source claim 和 publisher 证据。

## RFD.3 Forbidden simulation source gate

任务：

- [x] 配置 forbidden simulation input topics。
- [x] real preflight 检查 `/gazebo/*`、`/scan_ideal`、`/sim/x2/status`、`/rangefinder/down/scan_ideal`。
- [ ] 增加 source claim 检查，不能只按 topic 名称判断。
- [ ] real mode 下禁止 Gazebo/SITL/official baseline/gazebo-sensor service 启动痕迹。
- [ ] real mode 下禁止 SDF overlay / motor disturbance overlay source claim。
- [ ] real mode 下禁止 official maze 作为任何输入。
- [ ] real mode 下禁止 set_pose path 可用或 set_pose count 非零。

验收：

- [ ] 出现 forbidden simulation topic 时 summary `ok=false`。
- [ ] 出现 Gazebo/SITL/official baseline service 痕迹时 summary `ok=false`。
- [ ] 出现 X2 virtual serial source claim 时 summary `ok=false`。
- [ ] 真实 lidar 若使用 `/lidar` 原始 topic，能通过 source claim 区分，不被硬编码误杀。

## RFD.4 非执行性和安全边界

任务：

- [ ] real preflight doctor 不创建 `/ap/v1/cmd_vel` publisher。
- [ ] real preflight doctor 不发布 movement intent。
- [ ] real preflight doctor 不发布 landing intent。
- [ ] real preflight doctor 不调用 arm/takeoff/land service。
- [ ] real preflight doctor 不改变 FCU mode。
- [ ] doctor summary 记录 `flight_claim=not_evaluated`。
- [ ] doctor summary 记录 `landing_claim=not_evaluated`。

验收：

- [ ] ros graph / topic info 能证明 doctor 没有成为 FCU setpoint owner。
- [ ] 运行 doctor 前后 FCU armed/mode 不被 doctor 改变。
- [ ] summary 明确 doctor 通过不等于 hover/landing 通过。
- [ ] 如果 doctor 试图发布控制 topic，测试必须失败。

## RFD.5 Summary 和 blocker schema

任务：

- [ ] summary 新增或固化 `preflight_claim`。
- [ ] summary 新增或固化 `flight_claim=not_evaluated`。
- [ ] summary 新增或固化 `landing_claim=not_evaluated`。
- [ ] summary 记录 `checked_at` 和 `valid_for_sec`。
- [ ] summary 记录 `source_claims`。
- [ ] summary 记录 required real topic 检查结果。
- [ ] summary 记录 forbidden simulation source 检查结果。
- [ ] summary 记录 planned takeoff altitude，但不执行起飞。
- [ ] blocker 使用稳定字符串，例如 `real_preflight_failed`、`real_preflight_expired`。

验收：

- [ ] `ok=true` 时 summary 足够作为真机 flight task 入口证据。
- [ ] `blocked=true` 时 blockers 可直接定位 topic/source/runtime 问题。
- [ ] summary 不把 preflight doctor 误标为 `hover_claim=evaluated`。
- [ ] summary 不把 preflight doctor 误标为 `real_landing_claim=evaluated`。

## RFD.6 Stage 1 artifact gate

任务：

- [ ] 真机 Stage 2 入口检查对应 task 的 Stage 1 summary `ok=true`。
- [ ] 真机 Stage 2 入口检查对应 task 的 `ideal` 和 `mild_disturbance` 都通过。
- [ ] 真机 Stage 2 入口检查 Stage 1 artifact 已归档。
- [ ] 真机 Stage 2 入口记录 Stage 1 artifact path。
- [ ] Stage 1 profile matrix 缺失时 Stage 2 blocked。

验收：

- [ ] 未提供 Stage 1 artifact 时真机 flight task blocked。
- [ ] 只有 `ideal` 通过、`mild_disturbance` 缺失时真机 flight task blocked。
- [ ] Stage 1 summary `ok=false` 时真机 flight task blocked。
- [ ] Stage 2 summary 能追溯到 Stage 1 artifact。

## RFD.7 Operator safety confirmation

任务：

- [ ] 真机 flight task 入口要求 manual takeover 确认。
- [ ] 真机 flight task 入口要求 kill switch 确认。
- [ ] 真机 flight task 入口要求安全场地/保护措施确认。
- [ ] 支持非交互环境下通过显式 flag 或 env 提供确认。
- [ ] 未确认时在 arm/takeoff 之前 blocked。
- [ ] summary 记录 operator safety confirmation 的来源和时间。

验收：

- [ ] 缺 manual takeover 确认时 blocked。
- [ ] 缺 kill switch 确认时 blocked。
- [ ] 缺安全场地确认时 blocked。
- [ ] 确认字段不能由默认值静默通过。

## RFD.8 Real flight entry contract

任务：

- [ ] 设计或实现 `hover-real` entry。
- [ ] 设计或实现 `exploration-real` / P8 Stage 2 entry。
- [ ] 设计或实现 `scan-robustness-real` / P12 Stage 2 entry。
- [ ] real flight entry 必须读取最新通过的 real preflight summary。
- [ ] real flight entry 必须检查 preflight summary 未过期。
- [ ] real flight entry 必须读取 `fcu_controller.takeoff_alt_m`。
- [ ] real flight entry 不能启动 Gazebo/SITL/gazebo-sensor。
- [ ] real flight entry 输出 `acceptance_stage=real`。
- [ ] real flight entry 成功后输出 `real_landing_claim=evaluated`。

验收：

- [ ] `hover-real` 只在 preflight passed 后允许 arm/takeoff。
- [ ] `exploration-real` 只在 preflight passed 后允许 P8 return-home flight。
- [ ] `scan-robustness-real` 只在 preflight passed 后允许 P12 in-place landing flight。
- [ ] preflight failed/expired 时所有 real flight entry blocked。
- [ ] real flight entry 不复用 `just navlab-hover` 的 Docker/Gazebo 路径。

## RFD.9 Rosbag 和审计 artifact

任务：

- [ ] real preflight doctor 写出 artifact dir。
- [ ] real preflight doctor 保存 topic list。
- [ ] real preflight doctor 保存 topic info / publisher 摘要。
- [ ] real preflight doctor 保存 source claim summary。
- [ ] 可选录制短窗口 preflight bag，用于审计真实 topic 新鲜度。
- [ ] real flight task summary 引用 preflight artifact。

验收：

- [ ] preflight artifact 可复查当时 topic/source 状态。
- [ ] real flight artifact 能追溯到对应 preflight artifact。
- [ ] preflight artifact 不包含 Gazebo/SITL 仿真输入作为 required evidence。

## RFD.10 测试

任务：

- [ ] 增加配置测试：`process + real` real preflight 可加载。
- [ ] 增加配置测试：非法 backend/mode 组合 blocked。
- [ ] 增加 topic contract 测试：缺 required topic blocked。
- [ ] 增加 forbidden topic 测试：出现 `/sim/x2/status` blocked。
- [ ] 增加 source claim 测试：真实 `/lidar` 不因名字被误杀。
- [ ] 增加非执行性测试：doctor 不发布 `/ap/v1/cmd_vel`。
- [ ] 增加 Stage 1 gate 测试：缺 `ideal` 或 `mild_disturbance` blocked。
- [ ] 增加 preflight expiry 测试。
- [ ] 增加 summary schema 测试。

验收：

- [ ] real preflight 单元测试通过。
- [ ] runtime mode / backend 现有测试仍通过。
- [ ] unified landing Stage 2 blocker 测试仍通过。
- [ ] CLI help 仍能看到 real preflight doctor。

## RFD.11 执行顺序

建议顺序：

1. RFD.0 文档和边界。
2. RFD.1 Runtime mode 和 CLI 入口。
3. RFD.2 真实 topic contract。
4. RFD.3 Forbidden simulation source gate。
5. RFD.5 Summary 和 blocker schema。
6. RFD.4 非执行性和安全边界。
7. RFD.6 Stage 1 artifact gate。
8. RFD.7 Operator safety confirmation。
9. RFD.9 Rosbag 和审计 artifact。
10. RFD.8 Real flight entry contract。
11. RFD.10 测试。

## RFD 完成标准

RFD 全部完成必须满足：

- [ ] 每次真机 flight task 进入 arm/takeoff 前都必须引用通过的 real preflight summary。
- [ ] real preflight summary `ok=true`。
- [ ] real preflight summary `runtime_backend=process`。
- [ ] real preflight summary `runtime_mode=real`。
- [ ] required real topics 存在、类型正确、数据新鲜。
- [ ] forbidden simulation topics/source claims 不存在。
- [ ] doctor 不 arm、不 takeoff、不 land、不发布 movement setpoint。
- [ ] doctor summary 标记 `flight_claim=not_evaluated`。
- [ ] doctor summary 标记 `landing_claim=not_evaluated`。
- [ ] 真机 flight entry 检查 Stage 1 `ideal` 和 `mild_disturbance` 都通过。
- [ ] 真机 flight entry 检查 manual takeover / kill switch 已确认。
- [ ] 真机 flight entry 不能 fallback 到 Gazebo/SITL hover。
- [ ] 真机 flight summary 记录 preflight artifact、Stage 1 artifact 和 `acceptance_stage=real`。

## 验证记录

### 2026-06-09 RFD 文档初始化

- 命令：未运行代码测试，纯文档初始化。
- 结果：新增 real flight preflight doctor 设计文档和 TODO 文档，并把 TODO 加入 README 导航。
- blocker：真实 topic freshness、source claim、preflight expiry、operator safety confirmation 和 real flight entry 尚未实现。
- 备注：现有 `real-preflight-doctor` 只覆盖基础 `process + real` 和 topic presence contract；本 TODO 定义的是进入真机飞行前所需的完整入口 gate。
