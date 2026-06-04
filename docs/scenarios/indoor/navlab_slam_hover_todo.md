# NavLab 无 GPS SLAM 反馈悬停 TODO

这份 TODO 只跟踪当前主线：8 字形室内世界、真实雷达链路、真实 SLAM feedback、ArduPilot SITL 悬停。探索、绕圈、主动避障和 Nav2 暂不作为本轮完成标准。

## P0：文档和旧口径清理

目标：

- 删除旧的阶段文档口径。
- 固定当前阶段的完成定义。
- 防止 acceptance 再使用 Gazebo truth 伪装成 SLAM feedback。

任务：

- [x] 删除旧的阶段设计和 TODO 文档。
- [x] 新增 `navlab_slam_hover_design.md`。
- [x] 新增 `navlab_slam_hover_todo.md`。
- [ ] 更新运行命令命名，避免继续使用旧阶段名。
- [ ] 更新 summary 字段，显式区分 `slam_odom_source`、`gazebo_truth_source` 和 `external_nav_source`。

验收：

- [ ] `docs/README.md` 只指向当前主线文档。
- [ ] docs 中不再把旧 synthetic gate 当作当前完成标准。
- [ ] acceptance summary 中如果 ExternalNav 来自 Gazebo truth，必须 `ok=false` 或 `blocked=true`。

## P1：8 字形室内世界收敛

目标：

- 构建横向 8 字形封闭走廊世界。
- 让无人机从原点起飞后处在真实可观测环境中。
- 墙体、内障碍和无人机模型在 Foxglove 中显示稳定，不跟随无人机错误移动。

任务：

- [ ] 调整 `worlds/navlab_iq_quad_figure8.sdf` 的通道宽度。
- [ ] 将通道净宽收窄到无人机 footprint + `0.15 m` 到 `0.25 m`。
- [ ] 保证外墙和中间障碍都有 collision。
- [ ] 保证起飞区有足够高度和水平净空。
- [ ] `/sim/markers` 中墙体使用固定 frame。
- [ ] `/sim/markers` 中无人机 marker 使用移动 pose 或 `base_link`。
- [ ] 删除或隐藏不必要的调试拖尾 marker。

验收：

- [ ] Gazebo 可加载世界。
- [ ] 无人机模型可见，墙体可见。
- [ ] Foxglove 中墙体不随无人机移动。
- [ ] `/scan_ideal` 能看到左右环和中腰通道墙体。

## P2：传感器方向和 TF 链闭合

目标：

- `/scan` 在 ROS 和 Foxglove 中方向正确。
- `map/odom/base_link/imu_link/laser_frame/navlab_world` 的 TF 链闭合。

任务：

- [ ] 固定 `laser_frame -> base_link` 外参。
- [ ] 固定 `imu_link -> base_link` 外参。
- [ ] 固定 `navlab_world -> map` 或 `map -> navlab_world` 的显示桥接策略。
- [ ] 修正 X2 virtual serial 到 driver 输出的角度方向。
- [ ] 删除 mission 或 scan feature 中针对方向错误的反向补偿。
- [ ] 在 rosbag profile 中保留 `/tf` 和 `/tf_static`。

验收：

- [ ] Foxglove 不再提示 `laser_frame -> navlab_world` 缺失。
- [ ] Foxglove 不再提示 `imu_link -> navlab_world` 缺失。
- [ ] `/scan` 前方障碍显示在无人机机头方向。
- [ ] `/scan_ideal` 和 `/scan` 的主要几何结构一致。

## P3：真实 SLAM feedback 链路

目标：

- SLAM 消费 `/scan + /imu/data` 并输出 `/odom`。
- ExternalNav 只能使用 SLAM odom 作为验收输入。

任务：

- [ ] SLAM 服务启动时读取自己的 runtime config。
- [ ] SLAM 后端输出 `/odom`。
- [ ] `/cartographer/status` 或等价 health topic 输出输入频率、odom age、trajectory state。
- [ ] `external_nav_bridge` 输入从 `/gazebo/truth/odom` 切换为 SLAM `/odom`。
- [ ] acceptance summary 记录 `external_nav_input_topic`。
- [ ] 如果输入是 `/gazebo/truth/odom`，summary 必须标记为诊断模式，不允许通过。
- [ ] 保留 `/gazebo/truth/odom` 仅做误差对照。

验收：

- [ ] `/odom` 连续发布。
- [ ] `/external_nav/status.state == "healthy"`。
- [ ] `/mavlink_external_nav/status.state == "sending"`。
- [ ] SLAM odom 和 Gazebo truth 的误差可被 summary 记录。
- [ ] 断开 Gazebo truth 后 ExternalNav 仍可运行。

## P4：SITL ExternalNav 和 FCU 悬停

目标：

- ArduPilot SITL 真正接受 ExternalNav。
- 飞控自己完成 GUIDED、arm、takeoff、hover。

任务：

- [ ] 收集 SITL 日志到 artifact，例如 `sitl.log`。
- [ ] 检查 `profiles/navlab-sitl-external-nav.parm` 的 EKF ExternalNav 参数。
- [ ] 验证 SITL 输出 `LOCAL_POSITION_NED`。
- [ ] mission controller 先只实现 wait_ready、guided、arm、takeoff、hover_hold。
- [ ] hover setpoint 使用 MAVLink local position target + yaw。
- [ ] 不在当前阶段执行 forward、avoid、loop exploration。
- [ ] summary 记录 mode、armed、takeoff ack、local position count、hover drift。

验收：

- [ ] SITL 进入 `GUIDED`。
- [ ] SITL arm 成功。
- [ ] SITL takeoff 成功。
- [ ] `LOCAL_POSITION_NED` 持续发布。
- [ ] hover 稳定窗口至少 `20 s`。
- [ ] 水平漂移低于当前阈值。
- [ ] `set_pose_count == 0`。

## P5：Rosbag、Foxglove 和报告

目标：

- 每次 acceptance 都能被复盘。
- Foxglove 中能看清世界、无人机、scan、TF、SLAM odom、ExternalNav、FCU 状态。

任务：

- [ ] 固定最小 required topic 白名单。
- [ ] 允许 config.toml 追加 extra topics。
- [ ] rosbag profile summary 区分 missing、zero-count、optional。
- [ ] 生成 `summary.json`。
- [ ] 生成 `summary.md`。
- [ ] 生成 `foxglove_notes.md`。
- [ ] acceptance 结束后上传 MCAP 到 Foxglove。

验收：

- [ ] MCAP 中包含 required topics。
- [ ] Foxglove 可看到 8 字世界墙体。
- [ ] Foxglove 可看到无人机真实运动。
- [ ] Foxglove 可看到 `/scan` 与墙体对齐。
- [ ] Foxglove 可看到 SLAM odom 和 FCU local position 的趋势。

## P6：实机准备

目标：

- 仿真接口可以迁移到真实机器。

任务：

- [ ] 列出真实 FCU MAVLink 接线和 router 拓扑。
- [ ] 列出真实 X2 雷达 topic 和参数。
- [ ] 列出 IMU 来源优先级：FCU MAVLink、MAVROS、其他 DDS bridge。
- [ ] 列出起飞前静态检查命令。
- [ ] 列出安全接管条件。
- [ ] 列出实机 rosbag profile。

验收：

- [ ] 仿真和实机共用 `/scan`、`/imu/data`、`/odom`、`/external_nav/odom` 契约。
- [ ] 实机不需要 Gazebo truth。
- [ ] 真实飞行前有明确 go/no-go checklist。
