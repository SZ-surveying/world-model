# Hover ExternalNav / Gazebo Body 分离根因修复计划

日期：2026-06-22
状态：执行前计划，禁止先改代码

## 0. 纪律规则

本文件是后续 hover 横漂修复的唯一执行清单。没有写进本文件的事项，不允许直接改代码。

执行规则：

1. 每次改动前，必须先在本文件对应 TODO 下写：目标、证据、允许改动范围、禁止改动范围。
2. 每次改动后，必须在同一 TODO 下写：实际改了哪些文件、为什么、跑了什么验证、结果是什么。
3. 不允许先做完再解释。
4. 不允许把局部通过说成最终成功。
5. 最终 hover 成功标准是 Gazebo review-only hover drift `<= 0.10m`，不是旧 `0.35m`。
6. 禁止用 Gazebo truth 作为 runtime control / SLAM / ExternalNav 输入。
7. 禁止用 official maze map runtime localizer、fixed XY prior、Foxglove display TF、全局 runtime TF 补丁、降低/关闭 quality gate、放宽 drift gate。
8. 禁止继续通过“更早/更强 correction”掩盖 EKF 稳但 Gazebo body 漂的问题。

## 1. 当前真实问题

最新关键 run：`artifacts/sim/hover/20260622T041059Z`

现象：

- 顶层 `summary.ok=false`。
- Gazebo review-only hover drift：`0.6998m`，远超当前最终标准 `0.10m`。
- Mission/FCU hover drift：`0.0607m`，小于 `0.10m`。
- `hover_xy_alignment.ok=false`，Gazebo 与 ExternalNav/FCU/SLAM corrected 方向不一致。
- correction 已经提前进入 hover_hold：`first_correction_allowed_offset_sec=0.121s`，`first_correction_intent_active_offset_sec=0.839s`。

结论：

这不是“correction 启动太晚”了。现在的问题是：FCU/EKF/ExternalNav 估计链看起来稳定，但 Gazebo body truth 仍然漂。下一步必须查 frame/odom/MAVLink 坐标合同和控制执行链，不能继续调 hover gate 或 correction 参数。

## 2. 历史成功链路基准

旧成功路径需要作为 reference contract，不是直接恢复作弊输入。

旧路径目标：理解为什么 Gazebo truth odom 可以让 EKF 和 Gazebo world 对齐。

需要检查的旧文件：

- `git show a3e0f7a:navlab/companion/nodes/gazebo_truth_odom.py`
- `git show a3e0f7a:navlab/companion/nodes/external_nav.py`
- `git show a3e0f7a:navlab/companion/nodes/pose_mirror.py`
- `git show a3e0f7a:navlab/companion/nodes/hover_mission.py`
- `git show a3e0f7a:orchestration/src/tasks/slam_hover.py`

已知初步差异，尚未允许改代码：

- 旧 `external_nav.py` 的 MAVLink ODOMETRY position 映射为 `x=odom.x`、`y=-odom.y`、`z=-odom.z`。
- 当前 `external_nav.py` 的 helper `ros_enu_position_to_mavlink_local_frd()` 映射为 `x=odom.y`、`y=odom.x`、`z=-odom.z`。
- 这可能是 EKF local 和 Gazebo world 分离的重要原因，但必须先用最新 run 的 topic 证据验证，不能直接拍脑袋改。

## 3. 当前链路候选断点

当前 runtime 链路：

```text
/gazebo/model/odometry                review-only Gazebo body truth
/scan + /imu                          runtime sensor input
/slam/odom                            Cartographer/SLAM output
/navlab/scan_reference_drift/status   scan-reference diagnostic/correction intent
/slam/odom_corrected                  ExternalNav 输入
/external_nav/odom                    bridge/status observation
MAVLink ODOMETRY                      送入 ArduPilot EKF
LOCAL_POSITION_NED                    FCU/EKF local position
/navlab/fcu/local_position_pose        LOCAL_POSITION_NED mirror
/ap/v1/cmd_vel / setpoints / motors    控制输出
/gazebo/model/odometry                真实机体响应
```

候选断点：

1. `/slam/odom_corrected` 坐标不等于 Gazebo world/body 的同一水平投影。
2. `external_nav.py` 把 ROS odom 转 MAVLink ODOMETRY 时 x/y/sign/yaw 映射不符合旧成功合同。
3. `/navlab/fcu/local_position_pose` mirror 与 MAVLink LOCAL_POSITION_NED 的坐标解释不一致，导致 gate 误判或掩盖问题。
4. FCU 控制输出没有作用到 `/gazebo/model/odometry` 所代表的同一个 model/link。
5. Gazebo review-only odometry source/window/model/link 仍有误，但已有多轮证据显示它大概率是真 body drift；仍需在本计划中最后确认。

## 4. 执行 TODO

### TODO A：旧成功链路坐标合同表

状态：已完成

目标：只读旧 commit 和当前代码，写出旧成功链路每个 topic/message 的 frame、origin、x/y/z/yaw 映射。

允许操作：

- `git show` 读取旧文件。
- `rg` / `sed` 读取当前文件。
- 写本文档和 P6 TODO。

禁止操作：

- 禁止改代码。
- 禁止跑 hover。

输出要求：

- 表格：旧 `gazebo_truth_odom -> external_nav -> MAVLink ODOMETRY -> LOCAL_POSITION_NED` 的坐标合同。
- 明确旧代码中 position/yaw/twist 映射。

执行记录：

- 2026-06-22T04:30:35Z 开始执行 TODO A。
- 本轮目的：先建立旧 `a3e0f7a` 成功链路的坐标合同，不改 runtime，不跑 hover。
- 本轮证据来源：只读 `git show a3e0f7a:<path>` 输出和当前文档上下文。
- 本轮允许范围：更新本文档中 TODO A 的旧链路合同表；必要时同步 P6 TODO 的执行索引。
- 本轮禁止范围：不改 `external_nav.py`、不改 hover mission、不改 correction、不改 gate、不运行 hover。
- 2026-06-22T04:34Z 完成只读检查：
  - `a3e0f7a:navlab/companion/nodes/gazebo_truth_odom.py`
  - `a3e0f7a:navlab/companion/nodes/external_nav.py`
  - `a3e0f7a:navlab/companion/nodes/pose_mirror.py`
  - `a3e0f7a:navlab/companion/nodes/hover_mission.py`
  - `a3e0f7a:orchestration/src/tasks/slam_hover.py`
- 实际改动文件：只改本文档。
- 验证：未跑 hover、未跑测试；TODO A 是只读合同提取。

旧成功链路合同表：

| 环节 | 输入/输出 | frame/origin | position 映射 | yaw / quaternion 映射 | twist 映射 | 结论 |
| --- | --- | --- | --- | --- | --- | --- |
| Gazebo truth odom | `/world/navlab_iq_quad_figure8/dynamic_pose/info` -> `/gazebo/truth/odom` | 输出 `header.frame_id=odom`、`child_frame_id=base_link`；第一帧作为相对 origin，并用 `-origin.yaw` 旋转到起始 yaw 坐标系 | `odom.x=relative.x`、`odom.y=relative.y`、`odom.z=relative.z` | `odom.q=quaternion_from_yaw(relative.yaw)` | `vx/vy/vz/yaw_rate` 是相对位姿差分 | 旧输入虽然来自 Gazebo truth，但进入 ExternalNav 前已经是“相对起点 odom” |
| ExternalNav MAVLink | `/external_nav/odom` 或旧配置接入的 odom -> MAVLink `ODOMETRY` | 输入声明为 ROS ENU position + FLU body twist；输出 `MAV_FRAME_LOCAL_FRD` / `MAV_FRAME_BODY_FRD` | `mav.x=odom.x`、`mav.y=-odom.y`、`mav.z=-odom.z` | 默认 `q=[w,x,-y,-z]`；若用 FCU roll/pitch，则 yaw 为 `-_yaw_from_ros_quat_enu()` | `vx=odom.vx`、`vy=-odom.vy`、`vz=-odom.vz`、`rollspeed=odom.wx`、`pitchspeed=-odom.wy`、`yawspeed=-odom.wz` | 旧合同不是 ENU `x/y` 交换，而是 `x` 保持、`y/z` 取反 |
| FCU local pose mirror | `LOCAL_POSITION_NED` -> replay `PoseStamped` | `ned_to_gazebo_pose()`；默认输出 frame 为 `navlab_world`，同时可发布 replay TF | `pose.x=msg.x North`、`pose.y=msg.y East`、`pose.z=max(min_z, -msg.z Down + offset)` | `pose.yaw=ATTITUDE.yaw` | mirror 只发布 pose，不转换 twist | 旧 mirror 把 FCU NED 的 x/y 直接显示成 Gazebo/replay x/y |
| Hover mission gate | `LOCAL_POSITION_NED` | 直接使用 FCU local NED | `current_x=msg.x`、`current_y=msg.y`、`current_z=msg.z` | `current_yaw=ATTITUDE.yaw` | 无 | 旧 hover hold/setpoint 都在 FCU local NED 里闭环 |
| Hover setpoint | `_send_local_position_yaw_setpoint()` | MAVLink local NED/FRD 语义由 helper 决定 | hold 点来自 `LOCAL_POSITION_NED x/y`，高度发 `-takeoff_alt_m` | hold yaw 来自 FCU yaw | 无 | 旧路径内部一致：ExternalNav 送入的 local frame、FCU local、hover setpoint 三者保持同一个 x/y 合同 |

TODO A 直接结论：

- 旧成功链路的关键不是“Gazebo truth 本身”，而是整条链路的坐标合同自洽：truth odom 相对化后，ExternalNav 用 `x, -y, -z` 送入 MAVLink，FCU local mirror/hover mission 又按 `LOCAL_POSITION_NED x/y/z` 原样闭环。
- 当前已知差异 `current helper = y, x, -z` 与旧 `x, -y, -z` 不一致，但现在还不能直接改；必须继续 TODO B/C 证明当前 artifact 中哪个映射和 Gazebo body 同向。

### TODO B：当前链路坐标合同表

状态：已完成

目标：只读当前代码，写出 `/slam/odom_corrected -> external_nav.py -> MAVLink ODOMETRY -> /navlab/fcu/local_position_pose` 的坐标合同。

允许操作：

- 读取当前 `external_nav.py`、`pose_mirror.py`、`gate_evaluation.go`、SLAM adapter、ExternalNav bridge。
- 写本文档和 P6 TODO。

禁止操作：

- 禁止改代码。
- 禁止跑 hover。

输出要求：

- 表格：当前每个 topic/message 的 frame、origin、x/y/z/yaw 映射。
- 与 TODO A 的 diff 表。

执行记录：

- 2026-06-22T04:35Z 开始执行 TODO B。
- 本轮目的：建立当前 `/slam/odom_corrected -> external_nav.py -> MAVLink ODOMETRY -> /navlab/fcu/local_position_pose` 坐标合同，并和 TODO A 旧合同做 diff。
- 本轮证据来源：只读当前代码。
- 本轮允许范围：读取当前 `external_nav.py`、`pose_mirror.py`、`gate_evaluation.go`、SLAM adapter、ExternalNav bridge、scan reference correction；更新本文档。
- 本轮禁止范围：不改 runtime 代码、不跑 hover、不调参数、不改 gate。
- 2026-06-22T04:41Z 完成只读检查：
  - `navlab/real/companion/nodes/external_nav.py`
  - `navlab/real/companion/nodes/pose_mirror.py`
  - `navlab/common/slam/ros/bridges/navlab_external_nav_bridge/src/navlab_external_nav_bridge_node.cpp`
  - `navlab/common/slam/ros/bridges/navlab_external_nav_bridge/config/navlab_external_nav_bridge.params.yaml`
  - `navlab/common/slam/ros/localization/navlab_cartographer_adapter/src/navlab_cartographer_adapter_node.cpp`
  - `navlab/common/slam/ros/localization/navlab_cartographer_adapter/config/navlab_cartographer_2d_hover.lua`
  - `navlab/sim/companion/nodes/scan_reference_correction.py`
  - `orchestration/sim/internal/tasks/helpers/slam.go`
  - `orchestration/sim/internal/tasks/runtime_artifacts.go`
- 实际改动文件：只改本文档。
- 验证：未跑 hover、未跑测试；TODO B 是只读合同提取。

当前链路合同表：

| 环节 | 输入/输出 | frame/origin | position 映射 | yaw / quaternion 映射 | twist 映射 | 结论 |
| --- | --- | --- | --- | --- | --- | --- |
| Cartographer adapter | Cartographer `/tf` -> `/slam/odom` | hover 配置：`map_frame=map`、`published_frame=base_link`、`provide_odom_frame=false`；adapter 默认从 `map -> base_link` TF 发布 odom | `odom.x=tf.translation.x`、`odom.y=tf.translation.y`、`odom.z=tf.translation.z` | `odom.q=tf.rotation` | 只填 IMU angular；linear twist 基本不来自 SLAM | `/slam/odom` 是 ROS map/base_link XY，没有 x/y/sign 转换 |
| Scan reference correction | `/slam/odom` -> `/slam/odom_corrected` | 要求 `frame_id=map`、`child_frame_id=base_link` | copy 原 odom 后 `x += applied_x`、`y += applied_y`；若 gate 失败则 passthrough | copy 原 orientation | copy 原 twist；仅加大 x/y covariance 下限 | correction 输出仍是 ROS map/base_link XY |
| ExternalNav bridge | `/slam/odom_corrected` -> `/external_nav/odom` | runtime override：hover 任务把 `input_odom_topic` 改成 `/slam/odom_corrected`；`expected_odom_frame_id=map`、`expected_child=base_link`；输出 `frame_id=external_nav`、`child=base_link` | copy input odom，只把 `z`、`vz`、z covariance 改成 height estimator | copy input orientation | copy input twist，改 z/vz | bridge 是 pass-through ENU/FLU + height；没有 x/y/sign 转换 |
| MAVLink ExternalNav sender | `/external_nav/odom` -> MAVLink `ODOMETRY` | 输入声明为 ROS ENU position + FLU body twist；输出 `MAV_FRAME_LOCAL_FRD` / `MAV_FRAME_BODY_FRD` | 当前 helper：`mav.x=odom.y`、`mav.y=odom.x`、`mav.z=-odom.z` | 当前 yaw：`yaw_frd=pi/2 - yaw_enu`，默认还会 initial-align 到 FCU yaw；roll/pitch 可用 level 或 ROS | 当前 velocity 仍是旧式：`vx=odom.vx`、`vy=-odom.vy`、`vz=-odom.vz`，没有和 position 的 `y,x` 交换保持一致 | 当前 position 合同和旧成功链路不一致，并且当前 position/twist 的水平轴合同内部也不一致 |
| FCU local pose mirror | MAVLink `LOCAL_POSITION_NED` -> `/navlab/fcu/local_position_pose` | `message.header.frame_id=map` | `pose.x=LOCAL_POSITION_NED.x`、`pose.y=LOCAL_POSITION_NED.y`、`pose.z=-LOCAL_POSITION_NED.z` | `pose.yaw=ATTITUDE.yaw` | mirror 只发 pose | mirror 和旧代码一样，把 FCU local x/y 原样当 map x/y 看 |
| Gate alignment | rosbag -> summary | 对 `/gazebo/model/odometry`、`/navlab/fcu/local_position_pose`、`/external_nav/odom`、`/slam/odom_corrected` 做同窗 XY 比较 | 当前代码对 FCU local 只标记 `comparison_frame=ros_enu_from_mavlink_local_position_ned`，但比较向量仍用原始 x/y，没有应用 x/y/sign 归一化 | yaw 不参与主要 cosine | 无 | gate 当前能暴露分离，但不能证明哪个 mapping 正确；TODO C 要用候选映射离线算 |

TODO A/B diff：

| 项 | 旧成功链路 | 当前链路 | 风险 |
| --- | --- | --- | --- |
| ExternalNav position | `mav.x=odom.x`、`mav.y=-odom.y`、`mav.z=-odom.z` | `mav.x=odom.y`、`mav.y=odom.x`、`mav.z=-odom.z` | 水平轴交换且 y 符号不同，足以造成 EKF local 与 Gazebo body 方向不一致 |
| ExternalNav yaw | old 默认 `[w,x,-y,-z]` 或 `yaw_ned=-yaw_enu` | `yaw_frd=pi/2-yaw_enu`，且默认 initial-align 到 FCU yaw | yaw 合同也发生了 90 度定义变化，可能影响 body-frame velocity/控制解释 |
| Velocity/twist | `vx=odom.vx`、`vy=-odom.vy` | 仍是 `vx=odom.vx`、`vy=-odom.vy` | 当前 position 用 `y,x`，velocity 仍像旧 `x,-y`，水平合同内部不一致 |
| Bridge | 旧 truth odom 相对化后直接送入 ExternalNav | 当前 SLAM/correction odom pass-through 到 bridge，再由 sender 转 MAVLink | bridge 本身不做 x/y 修正；真正风险集中在 sender 或 correction 输出 frame |
| FCU mirror | `LOCAL_POSITION_NED x/y` 原样显示 | `LOCAL_POSITION_NED x/y` 原样显示 | mirror 没变，因此如果 sender 输入合同错，FCU local 会“看起来稳定”但与 Gazebo body 分离 |

TODO B 直接结论：

- 当前最强代码级嫌疑是 `navlab/real/companion/nodes/external_nav.py` 的 ROS ENU -> MAVLink LOCAL_FRD position/yaw 合同；它和旧成功链路不同，且 position 与 twist 水平轴合同不自洽。
- 仍不允许直接改，因为还需要 TODO C 用最新失败 run 证明：哪一个候选映射能让 `/slam/odom_corrected`、`/external_nav/odom`、`/navlab/fcu/local_position_pose` 和 `/gazebo/model/odometry` 在同一 hover window 内同向。

### TODO C：最新失败 run 的同窗四源数值对齐

状态：已完成

目标：用 `artifacts/sim/hover/20260622T041059Z` 的 rosbag/summary，在同一个 hover_hold window 内提取四源时间序列，验证从哪一秒开始 Gazebo body 和 EKF local 分离。

必须比较：

- `/gazebo/model/odometry`
- `/slam/odom_corrected`
- `/external_nav/odom`
- `/navlab/fcu/local_position_pose`
- 如果 rosbag 有：`/ap/v1/cmd_vel`、setpoint topic、motor/servo output topic

允许操作：

- 离线解析 artifact。
- 写诊断脚本到临时位置或一次性 shell/python；若写入 repo，必须先在本 TODO 记录文件名，结束后说明是否保留。
- 写本文档和 P6 TODO。

禁止操作：

- 禁止改 runtime 代码。
- 禁止跑 hover。

输出要求：

- 表格：每个源在 hover window 的 start/final/max/span。
- 表格：旧映射候选 `x,-y`、当前映射 `y,x`、其他候选与 Gazebo final vector 的 cosine/scale。
- 明确根因候选是否被证据支持。

执行记录：

- 2026-06-22T04:42Z 开始执行 TODO C。
- 本轮目的：只用 `artifacts/sim/hover/20260622T041059Z` 的离线 artifact，量化四源同窗 XY 是否同向，并测试候选映射。
- 本轮证据来源：artifact summary、rosbag metadata/MCAP、已有 gate summary；`/gazebo/model/odometry` 只作 review-only 证据。
- 本轮允许范围：只读 artifact；如现有 summary 不够，可写一次性临时脚本到 `/tmp`，不进入 repo。
- 本轮禁止范围：不改 runtime 代码、不跑 hover、不改 gate、不调 correction。
- 2026-06-22T04:48Z 完成离线提取。
- 实际读取文件：
  - `artifacts/sim/hover/20260622T041059Z/summary.json`
  - `artifacts/sim/hover/20260622T041059Z/mission_summary.json`
  - `artifacts/sim/hover/20260622T041059Z/rosbag/hover_rosbag/metadata.yaml`
- 实际脚本：只在 shell 内运行一次性 Python 片段，没有写入 repo。
- 实际改动文件：只改本文档。
- 验证：未跑 hover、未改 runtime。

四源同窗结果，窗口来源均为 `hover_status_phase_hover_hold`：

| 源 | topic | frame/child | samples | final x/y | magnitude | max drift/span | 结论 |
| --- | --- | --- | ---: | --- | ---: | ---: | --- |
| Gazebo body review-only | `/gazebo/model/odometry` | `odom` / `base_link` | 636 | `(0.55795, -0.42239)` | `0.69981m` | `0.69981m` | 真实机体在 Gazebo ROS odom frame 下明显横漂 |
| Gazebo SDF projected | `gazebo_xyz_to_ned_projection` | review-only projection | 636 | `(-0.42239, -0.55795)` | `0.69981m` | `0.69981m` | SDF 声明 `gazeboXYZToNED=0 0 0 180 0 90`；projected 只作诊断 |
| SLAM corrected | `/slam/odom_corrected` | `map` / `base_link` | 2778 | `(-0.02502, 0.08254)` | `0.08624m` | `0.26884m` | SLAM/correction 估计幅值远小于 Gazebo body |
| ExternalNav bridge | `/external_nav/odom` | `external_nav` / `base_link` | 36 | `(-0.02123, 0.08649)` | `0.08906m` | `0.26669m` | 与 `/slam/odom_corrected` 几乎同向，bridge 没改变 XY |
| FCU local mirror | `/navlab/fcu/local_position_pose` | `map` / empty | 190 | `(0.05021, 0.03411)` | `0.06070m` | `0.17547m` | FCU/EKF local 认为横漂很小，和 Gazebo body 分离 |

已有 gate pairwise：

| pair | cosine | swapped cosine | scale | 结论 |
| --- | ---: | ---: | ---: | --- |
| ExternalNav vs SLAM corrected | `0.99857` | `-0.50988` | `0.96843` | bridge 基本 pass-through |
| FCU vs ExternalNav | `0.66933` | `0.34854` | `0.68162` | sender/EKF 后方向已经不完全一致 |
| Gazebo raw vs SLAM corrected | `-0.80891` | `0.93810` | `0.12324` | 原始 Gazebo frame 下表现为明显 x/y swap suspicious |
| Gazebo raw vs ExternalNav | `-0.77627` | `0.91821` | `0.12726` | 同上 |
| Gazebo raw vs FCU | `-0.05122` | `0.32031` | `0.08674` | FCU local 幅值太小且方向不可靠 |

候选映射离线计算，source=`/slam/odom_corrected` final `(-0.02502, 0.08254)`：

| 参考向量 | 候选映射 | mapped vector | cosine | scale |
| --- | --- | --- | ---: | ---: |
| Gazebo raw ROS odom `(0.55795,-0.42239)` | 当前 `y,x` | `(0.08254,-0.02502)` | `0.93810` | `0.12324` |
| Gazebo raw ROS odom `(0.55795,-0.42239)` | 旧成功 `x,-y` | `(-0.02502,-0.08254)` | `0.34636` | `0.12324` |
| Gazebo SDF projected `(-0.42239,-0.55795)` | 旧成功 `x,-y` | `(-0.02502,-0.08254)` | `0.93810` | `0.12324` |
| Gazebo SDF projected `(-0.42239,-0.55795)` | 当前 `y,x` | `(0.08254,-0.02502)` | `-0.34636` | `0.12324` |

候选映射离线计算，source=`/external_nav/odom` final `(-0.02123, 0.08649)`：

| 参考向量 | 候选映射 | mapped vector | cosine | scale |
| --- | --- | --- | ---: | ---: |
| Gazebo raw ROS odom `(0.55795,-0.42239)` | 当前 `y,x` | `(0.08649,-0.02123)` | `0.91821` | `0.12726` |
| Gazebo raw ROS odom `(0.55795,-0.42239)` | 旧成功 `x,-y` | `(-0.02123,-0.08649)` | `0.39609` | `0.12726` |
| Gazebo SDF projected `(-0.42239,-0.55795)` | 旧成功 `x,-y` | `(-0.02123,-0.08649)` | `0.91821` | `0.12726` |
| Gazebo SDF projected `(-0.42239,-0.55795)` | 当前 `y,x` | `(0.08649,-0.02123)` | `-0.39609` | `0.12726` |

TODO C 直接结论：

- “SLAM/correction/bridge 没估到足够横漂幅值”是事实：SLAM corrected/ExternalNav final 只有 `0.086-0.089m`，Gazebo body 是 `0.700m`。
- “ExternalNav bridge 丢了 XY”不成立：`/external_nav/odom` 和 `/slam/odom_corrected` cosine `0.99857`。
- 方向判断不能只看 Gazebo raw ROS odom，因为 model SDF 同时声明了 `gazeboXYZToNED=0 0 0 180 0 90`。如果使用 SDF projected 作为 ArduPilot/NED review reference，旧成功 `x,-y` 映射与 Gazebo projected 同向；当前 `y,x` 反向。
- 所以最小修复不能是继续加强 correction。下一步 TODO D 必须评审：是否把 ExternalNav sender 的 MAVLink `LOCAL_FRD` 合同恢复成旧成功 `x,-y,-z`，并同步 yaw/twist 合同；同时保留 Gazebo raw/projection 两套 evidence，避免把 review frame 误当 runtime frame。

### TODO D：最小修复方案评审

状态：已完成

进入条件：TODO A/B/C 完成。

目标：只写方案，不改代码。确定最小修复点到底是：

- `external_nav.py` position/yaw mapping；或
- `scan_reference_correction.py` 输出 frame/sign；或
- `gate_evaluation.go` comparison normalization；或
- Gazebo odometry evidence source/model/link；或
- FCU control output / model link 执行链。

输出要求：

- 写明选中方案。
- 写明为什么不选其他方案。
- 写明需要改哪些文件。
- 写明单测和真实 hover 验证标准。

执行记录：

- 2026-06-22T04:55Z 完成方案评审，未改 runtime 代码。
- 官方依据：
  - MAVLink `ODOMETRY` 是 ArduPilot Non-GPS ExternalNav 推荐输入，ArduPilot 文档要求 `frame_id`/`child_frame_id` 使用 `MAV_FRAME_BODY_FRD` 或 `MAV_FRAME_LOCAL_FRD`，且 `z/vz` 正方向为 down：https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
  - MAVLink common 定义 `MAV_FRAME_LOCAL_FRD` 为 local tangent FRD，`x=Forward`、`y=Right`、`z=Down`；`MAV_FRAME_BODY_FRD` 为 body FRD：https://mavlink.io/en/messages/common.html
  - ArduPilot Cartographer non-GPS 页面说明 Cartographer/ROS SLAM 可作为 ArduPilot local position estimate 输入，但不要求把 Gazebo truth/official map 接成 runtime 输入：https://ardupilot.org/dev/docs/ros-cartographer-slam.html
- 项目证据：
  - TODO A：旧成功链路在 MAVLink sender 处使用 `x=odom.x`、`y=-odom.y`、`z=-odom.z`，等价于把项目的 ROS local FLU/ENU-like odom 转成 MAVLink local FRD。
  - TODO B：当前 sender 改成 `x=odom.y`、`y=odom.x`、`z=-odom.z`，但 twist 仍是 `vx=odom.vx`、`vy=-odom.vy`，导致 position/twist 水平合同不自洽。
  - TODO C：bridge 不是根因，因为 `/external_nav/odom` 与 `/slam/odom_corrected` cosine `0.99857`；Gazebo SDF projected reference 下，旧 `x,-y` 与 `/slam/odom_corrected` cosine `0.93810`，当前 `y,x` 为 `-0.34636`。

选中方案：修 `navlab/real/companion/nodes/external_nav.py` 的 MAVLink FRD 坐标合同。

必须改的内容：

1. `ros_enu_position_to_mavlink_local_frd()` 从当前 `return y, x, -z` 改为 `return x, -y, -z`。
2. `ros_enu_yaw_to_mavlink_local_frd()` 从当前 `pi/2-yaw` 改为 `-yaw`，保持 ROS local yaw -> MAVLink FRD yaw 与旧成功合同一致。
3. `_odometry_mapping_status()` 的 field map 同步更新，不能继续显示错误合同。
4. 单测先锁死：
   - position：`(1, 2, 3)` -> `(1, -2, -3)`。
   - yaw：`+0.25rad` -> `-0.25rad`。
   - sender status mapping 必须显示 `x=odom.pose.pose.position.x`、`y=-odom.pose.pose.position.y`。

暂不改的内容：

- 不改 `scan_reference_correction.py`：TODO C 证明它和 bridge 前后的方向基本一致，继续加 correction 只会掩盖 frame bug。
- 不改 `navlab_external_nav_bridge_node.cpp`：它是 pass-through + height，TODO C 证明 bridge 没丢 XY。
- 不改 `gate_evaluation.go`：gate 已经正确把 drift/alignment 失败暴露出来；后续可以增强 evidence，但不是本次最小修复。
- 不改 Gazebo odometry source/model/link：TODO C 的 evidence `ok=true`，SDF/model/link 没 blocker。
- 不改 hover 高度、landing、drift gate、quality gate、Foxglove、全局 TF。

TODO E 单测/验证标准：

- 先改/新增 `navlab/tests/companion/test_external_nav_sender.py` 的坐标合同测试。
- 再改 `navlab/real/companion/nodes/external_nav.py`。
- 跑：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_sender.py -q
```

- 单测通过后才进入 TODO F 真实 hover。

### TODO E：实现最小修复

状态：已完成

进入条件：TODO D 完成。

允许操作：

- 只改 TODO D 指定文件。
- 必须先加/改单测锁死坐标合同，再改实现。

禁止操作：

- 禁止同时改 hover 高度、landing、drift gate、quality gate、Foxglove、Gazebo truth input、official map、fixed prior。

输出要求：

- 改动文件列表。
- 每个文件为什么改。
- 单测结果。

执行记录：

- 2026-06-22T04:58Z 开始执行 TODO E。
- 本轮目的：只实现 TODO D 选定的 ExternalNav sender MAVLink FRD 合同修复。
- 本轮允许改动文件：
  - `navlab/tests/companion/test_external_nav_sender.py`
  - `navlab/real/companion/nodes/external_nav.py`
- 本轮禁止范围：不改 hover mission、不改 correction、不改 bridge、不改 Gazebo drift gate、不改 landing、不改 Foxglove/TF、不改 Cartographer 参数。
- 执行顺序：先改测试锁死合同，再改实现，再跑定向单测。
- 2026-06-22T05:01Z 完成实现。
- 实际改动文件：
  - `navlab/tests/companion/test_external_nav_sender.py`
    - 把 position 合同测试从旧错误 `y,x,-z` 改为项目/MAVLink FRD 合同 `x,-y,-z`。
    - 把 raw yaw 测试从 `pi/2-yaw` 改为 `-yaw`。
    - 新增 `_odometry_mapping_status()` 合同测试，锁死状态输出中的 `x=odom.x`、`y=-odom.y`、`z=-odom.z`。
  - `navlab/real/companion/nodes/external_nav.py`
    - `ros_enu_position_to_mavlink_local_frd()` 改为 `return x_enu_m, -y_enu_m, -z_enu_m`。
    - `ros_enu_yaw_to_mavlink_local_frd()` 改为 `return normalize_angle_rad(-yaw_enu_rad)`。
    - `_odometry_mapping_status()` 同步更新 field map，避免 status 继续报告错误合同。
- 未改文件：hover mission、correction、bridge、Gazebo gate、landing、Foxglove、Cartographer 参数均未改。
- 定向验证：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_sender.py -q
```

- 结果：`11 passed in 0.04s`。

### TODO F：真实 hover 验证

状态：已失败，必须回到 TODO D 重新评审；禁止继续直接改代码

进入条件：TODO E 单测通过。

命令：

```bash
cd orchestration/sim && env NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover
```

最终成功标准：

- 顶层 `summary.ok=true`。
- `hover_gazebo_model_horizontal_drift <= 0.10m`。
- `hover_xy_alignment.ok=true`。
- `takeoff_ack_ok=true`。
- `hover_altitude_crosscheck.ok=true`。
- `rangefinder_probe.ok=true`。
- `landing_ok=true`、`disarmed=true`。
- 不允许 `hover_window_applied_without_phase4b_count > 0`。
- 不允许 `hover_window_applied_without_runtime_consistency_count > 0`。

如果失败：

- 只记录 blocker 和对应证据。
- 禁止马上继续改；必须回到 TODO D 更新方案。

执行记录：

- 2026-06-22T05:02Z 开始执行 TODO F。
- 本轮目的：验证 TODO E 的 ExternalNav sender 坐标合同修复是否让真实 hover 顶层 summary 通过。
- 本轮命令：

```bash
cd orchestration/sim && env NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover
```

- 本轮判定：只有 `summary.ok=true`、`hover_gazebo_model_horizontal_drift <= 0.10m`、`hover_xy_alignment.ok=true` 等全部最终标准满足，才记录为成功。
- 本轮禁止范围：如果失败，只记录 blocker 和证据，不马上继续乱改。
- 2026-06-22T04:38:12Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T043812Z`
  - `artifacts/sim/hover/20260622T043812Z/summary.json`
  - `artifacts/sim/hover/20260622T043812Z/mission_summary.json`
- 结果：失败，`summary.ok=false`，`status=TASK_STATUS_BLOCKED`。
- 顶层 blocker：
  - `hover_gazebo_model_horizontal_drift`
  - `hover_mission_body_not_ok`
  - `hover_mission_drift_not_ok`
  - `hover_mission_drift_quality_unacceptable`
  - `hover_mission_not_ok`
  - `hover_xy_alignment_direction_mismatch:external_nav_odom__slam_odom_corrected`
  - `hover_xy_alignment_direction_mismatch:fcu_local_position_pose__external_nav_odom`
  - `hover_xy_alignment_direction_mismatch:fcu_local_position_pose__slam_odom_corrected`
  - `hover_xy_alignment_direction_mismatch:gazebo_model_odometry__external_nav_odom`
  - `hover_xy_alignment_direction_mismatch:gazebo_model_odometry__fcu_local_position_pose`
  - `hover_xy_evidence_disagreement`
- 关键数值：
  - Gazebo review-only drift：`7.7285m`，远超 `0.10m`。
  - Mission/FCU hover drift：`0.2016m`，也超过 `0.10m`。
  - Hover altitude crosscheck：`ok=true`，高度不是这轮主因。
  - Takeoff ACK：`true`。
  - Landing/disarm：`land_command_accepted=true`、`disarmed=true`，但 descent profile 仍因过快下降 evidence 失败；这不是本轮改动目标。
  - Runtime status 确认新合同已经生效：`mapping.field_map.x="odom.pose.pose.position.x"`、`y="-odom.pose.pose.position.y"`、`z="-odom.pose.pose.position.z"`。
- 四源方向：
  - `/slam/odom_corrected` final `(0.19280, 0.10584)`，magnitude `0.21995m`。
  - `/external_nav/odom` final `(-0.27475, 0.05170)`，magnitude `0.27957m`。
  - `/navlab/fcu/local_position_pose` final `(-0.20094, -0.00970)`，magnitude `0.20117m`。
  - `/gazebo/model/odometry` final `(7.70187, -0.64093)`，magnitude `7.72850m`.
- 直接结论：
  - TODO E 的改动确实进入 runtime，但没有修好 hover。
  - 单独恢复旧 `x,-y,-z` 合同，在当前 SLAM/correction/yaw/FCU 控制链下会显著放大真实横漂。
  - 因此不能说成功，也不能继续加 correction 或调 gate。
  - 必须回到 TODO D 做第二轮方案评审：先判定是 `yaw_alignment_offset / yaw contract`、`/slam/odom_corrected -> /external_nav/odom` 的 timestamp/sample 对齐、还是 `LOCAL_FRD` 与当前 SLAM map frame 的 origin/yaw 定义不一致。

### TODO D2：失败后的二次方案评审

状态：已完成

进入条件：TODO F 失败已记录。

目标：解释为什么 TODO E 的旧合同修复让 Gazebo drift 从 `0.6998m` 恶化到 `7.7285m`，再决定是回滚、改成可配置 frame mode、还是修 yaw/origin 对齐。

必须检查：

- 对比 `20260622T041059Z` 和 `20260622T043812Z`：
  - `/external_nav/odom`
  - `/slam/odom_corrected`
  - `/navlab/fcu/local_position_pose`
  - `/gazebo/model/odometry`
  - `/mavlink_external_nav/status.last_sent_x/last_sent_y/last_sent_yaw_rad/yaw_alignment_offset_rad`
- 检查 `max_horizontal_speed_mps=0.25` rate limiter 是否和新 mapping/yaw 组合导致 FCU 追错方向。
- 检查当前 SLAM map frame 是否已经不是旧 truth relative frame；如果 map frame 已经是 Cartographer map，而不是旧 relative-forward frame，则旧 `x,-y` 不能直接套。
- 检查 yaw：当前 run `yaw_alignment_offset_rad=-1.7786`，旧 run 是否接近另一个固定偏置；判断 `yaw=-yaw_ros` 是否和 `align_yaw_to_fcu` 叠加出错。

禁止操作：

- 禁止继续改 correction。
- 禁止改 drift gate。
- 禁止继续跑 hover 试错。
- 禁止把 `7.7285m` 解释成成功或偶然。

输出要求：

- 写明本轮失败的具体链路原因。
- 如果需要回滚 TODO E，必须先在本 TODO 写明回滚理由和验证方式。
- 如果需要新修复，必须写 TODO E2，先测后改。

执行记录：

- 2026-06-22T05:08Z 开始执行 TODO D2。
- 本轮目的：只做离线对比，解释为什么旧合同修复让 drift 恶化。
- 本轮允许范围：读取 `20260622T041059Z` 和 `20260622T043812Z` 的 summary/mission summary/status。
- 本轮禁止范围：不改代码、不跑 hover、不调参数。
- 2026-06-22T05:12Z 完成离线对比。

对比结果：

| 项 | `20260622T041059Z`，修改前 | `20260622T043812Z`，TODO E 后 | 结论 |
| --- | ---: | ---: | --- |
| 顶层 ok | `false` | `false` | 两者都不是成功 |
| mission ok | `true` | `false` | TODO E 后 mission 自身也失败 |
| Gazebo drift | `0.6998m` | `7.7285m` | TODO E 明显恶化真实机体漂移 |
| FCU hover drift | `0.0607m` | `0.2016m` | TODO E 也让 FCU local drift 变差 |
| ExternalNav mapping | `x=odom.y, y=odom.x` | `x=odom.x, y=-odom.y` | runtime 确实吃到了 TODO E |
| yaw alignment offset | `+1.5927rad` | `-1.7786rad` | yaw 合同/初始对齐符号大幅变化 |
| `/slam/odom_corrected` final | `(-0.0250, 0.0825)` | `(0.1928, 0.1058)` | SLAM/correction 仍只有小幅估计，远小于 Gazebo |
| `/external_nav/odom` final | `(-0.0212, 0.0865)` | `(-0.2747, 0.0517)` | TODO E 后 ExternalNav/SLAM 同窗方向也不一致 |
| `/navlab/fcu/local_position_pose` final | `(0.0502, 0.0341)` | `(-0.2009, -0.0097)` | FCU local 方向随 mapping/yaw 改变 |
| scan quality good ratio | `1.0` | `0.4524` | 大漂移后 scan quality 变差 |
| phase4B | `false` | `false` | correction 仍不能安全接控制 |

具体原因：

- 旧 `x,-y,-z` 合同只适用于旧 `gazebo_truth_odom.py` 产生的 truth-relative frame：第一帧作 origin，并按 `-origin.yaw` 旋到起始机体系。
- 当前 `/slam/odom_corrected` 来自 Cartographer `map -> base_link`，不是旧 truth-relative frame。把旧合同直接套到当前 Cartographer map frame，会改变 FCU/EKF 的控制方向和 yaw alignment offset。
- TODO E 后 `yaw_alignment_offset_rad` 从 `+1.5927rad` 变成 `-1.7786rad`，同时 `last_sent_x/last_sent_y` 和 FCU local 方向翻转，结果 Gazebo body 漂到 `7.7285m`。
- 所以 TODO E 的假设“旧合同可直接恢复到当前 SLAM frame”被真实 hover 证伪。

下一步决策：

- 必须回滚 TODO E 的 ExternalNav sender mapping/yaw 改动，恢复到修改前较小漂移状态，避免保留一个已证伪的回归。
- 回滚不是宣称 hover 成功；回滚后仍然回到旧问题：Gazebo body drift `0.6998m`，而 SLAM/correction/FCU 只看到 `0.06-0.09m`。
- 真正下一阶段根因应转回 SLAM/scan-reference drift estimator 幅值与 runtime correction eligibility：scan runtime 能看到大漂移，但 phase4B/correction 未安全输出，不能让 FCU 用到有效 counter-drift。

### TODO E2：回滚 TODO E 的已证伪 ExternalNav mapping 改动

状态：已完成

目标：撤销 TODO E 对 `external_nav.py` 的旧合同恢复，回到当前 Cartographer map frame 下原先的 `y,x,-z` / `pi/2-yaw` 合同，避免保留导致 `7.7285m` Gazebo drift 的回归。

允许改动文件：

- `navlab/real/companion/nodes/external_nav.py`
- `navlab/tests/companion/test_external_nav_sender.py`
- 本文档

禁止改动：

- 不改 correction。
- 不改 bridge。
- 不改 hover mission。
- 不改 drift gate。
- 不改 landing。
- 不跑 hover 试错；只跑定向单测确认回滚。

验证：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_sender.py -q
```

完成后状态：

- 代码恢复到 TODO E 前的 mapping/yaw 行为。
- 文档保留 TODO E/F/D2 的失败证据，说明为什么不能再走“直接恢复旧 Gazebo truth 合同”这条路。

执行记录：

- 2026-06-22T05:14Z 开始执行 TODO E2。
- 本轮只允许回滚 TODO E 改动和对应测试预期。
- 2026-06-22T05:16Z 完成回滚。
- 实际改动文件：
  - `navlab/real/companion/nodes/external_nav.py`
    - 恢复 `ros_enu_position_to_mavlink_local_frd()` 为 `return y_enu_m, x_enu_m, -z_enu_m`。
    - 恢复 `ros_enu_yaw_to_mavlink_local_frd()` 为 `return normalize_angle_rad((math.pi * 0.5) - yaw_enu_rad)`。
    - 恢复 `_odometry_mapping_status()` field map 为当前 Cartographer map frame 使用的 `x=odom.y`、`y=odom.x`。
  - `navlab/tests/companion/test_external_nav_sender.py`
    - 恢复 position/yaw 测试预期。
    - 删除 TODO E 新增的旧合同 status 测试。
- 定向验证：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_sender.py -q
```

- 结果：`10 passed in 0.04s`。
- 结论：代码已撤回导致 `7.7285m` drift 的回归；hover 仍未成功，下一步不能再走 ExternalNav 旧 truth-frame 直接恢复路线。

### TODO D3：下一轮真正根因评审，转向 SLAM/scan-reference 幅值不足

状态：已完成

进入条件：TODO E2 已回滚。

目标：解释为什么在较安全的当前 mapping 下，Gazebo body drift `0.6998m`，而 `/slam/odom_corrected`、`/external_nav/odom`、FCU local 只显示 `0.06-0.09m`。

已知事实：

- `20260622T041059Z` 中 `/slam/odom_corrected` 与 `/external_nav/odom` 同向，bridge 不是主要丢失点。
- scan-reference runtime drift 可观测到约 `0.6329m` 级别漂移，但 phase4B/correction 没有安全通过到控制链。
- `hover_window_quality_good_ratio=1.0`，但 `phase4B_consistency_ok=false`，allowed axes 最终为空。
- 这说明真正没闭环的是 scan-reference drift estimator / correction eligibility / SLAM odom correction 幅值，而不是 ExternalNav sender 的旧 truth-frame mapping。

下一步允许：

- 只做离线诊断：对比 `/navlab/scan_reference_drift/status`、`/navlab/scan_reference_correction/status`、`/slam/odom`、`/slam/odom_corrected`、`/gazebo/model/odometry`。
- 明确为什么 scan-reference 看到大漂移但 correction 没有稳定输出。
- 写 TODO E3 后才能改代码。

下一步禁止：

- 不改 ExternalNav sender mapping。
- 不跑 hover 试错。
- 不放宽 quality gate。
- 不扩大 correction cap。
- 不接 Gazebo truth/official map/fixed prior。

执行记录：

- 2026-06-22T05:20Z 开始执行 TODO D3。
- 本轮目的：只用离线证据解释 scan-reference 为什么看见 `~0.63m` 漂移但没有形成稳定 correction 输出。
- 本轮证据范围：
  - `artifacts/sim/hover/20260622T041059Z/summary.json`
  - `artifacts/sim/hover/20260622T041059Z/mission_summary.json`
  - 必要时只读 `rosbag/hover_rosbag/hover_rosbag_0.mcap.zstd`
- 本轮允许范围：只读 artifact、只写本文档；如需要脚本，只用 `/tmp` 一次性脚本。
- 本轮禁止范围：不改代码、不跑 hover、不改 gate、不改 ExternalNav mapping、不扩大 correction cap。
- 2026-06-22T05:28Z 完成离线诊断。

关键证据，run `20260622T041059Z`：

| 项 | 数值/状态 | 解释 |
| --- | --- | --- |
| Gazebo body drift | `0.699806865m` | 真实机体横漂明显 |
| scan-reference hover drift | final `(0.57548, -0.46826)`，max `0.74192m` | `/scan` 能观测到与 Gazebo 同量级漂移 |
| scan-reference runtime drift | max `0.63292m`，window quality good ratio `1.0` | runtime estimator 在 hover window 内长期可用 |
| `/slam/odom_corrected` drift | final `(-0.02502, 0.08254)`，mag `0.08624m` | correction 输出仍让 ExternalNav 看到很小漂移 |
| correction applied | hover window `31/36` status samples applied | correction 曾经启用，但输出幅值受限 |
| correction cap | `max_correction_m=0.25`，`max_correction_step_m=0.03` | 绝对 measurement delta 被裁到 0.25m |
| phase4B | `phase4b_consistency_ok=false`，常见 blocker 是 saturation/intent 不稳定 | 当前 phase4B 把“漂移超过 0.25m”当成 unsafe saturation |

代码级原因：

- `scan_reference_drift.py` 估计的是 scan-only 真实漂移 measurement：`estimate.x_m / estimate.y_m`。
- 同一文件的 `evaluate_correction_intent()` 又生成反方向 intent：`correction_x=-source_x`，它表示“反漂移控制意图”，不是 ExternalNav odometry measurement。
- `scan_reference_correction.py` 里 `decide_correction()` 对 intent 反号后作为 measurement delta；当 intent/phase4B 失败时，`_measurement_decision()` 会直接用 scan measurement。
- 但是 `_measurement_decision()` 和 intent 路径共用 `max_correction_m=0.25` 作为绝对 cap。这样即使 `/scan` 观测到 `0.63-0.74m`，`/slam/odom_corrected` 最多只能额外加 `0.25m`，不可能让 EKF/Gazebo 差距小于 `0.10m`。
- 这不是“放宽 quality gate”的问题，而是 measurement 输出的物理语义错了：真实 odom measurement 需要能报告实际位移；安全应由质量门控、方向连续、符号稳定和 per-step 限速保证，而不是用 0.25m 绝对截断把 odom 假装稳定。

直接结论：

- 下一步不是改 ExternalNav mapping，也不是加强控制 correction。
- 应把 scan-reference correction 分成两类限制：
  - `max_control_intent_m=0.25`：仍用于 shadow/counter-drift intent，不扩大。
  - `max_measurement_delta_m`：用于 ExternalNav odometry measurement 的物理可信上限，应覆盖当前任务允许的可观测室内漂移，例如 `1.25m`，并继续受 quality/runtime consistency/per-step limit 保护。
- `max_correction_step_m=0.03` 保持不变，用于避免 odom measurement 突跳。

### TODO E3：实现受门控的 scan measurement 输出上限，替代 0.25m 绝对截断

状态：已完成

目标：让 `/slam/odom_corrected` 能在 gate 通过时报告 scan-reference 观测到的真实横漂 measurement，而不是被 0.25m 绝对 cap 裁成“看起来稳定”的 odom。

允许改动文件：

- `navlab/sim/companion/nodes/scan_reference_correction.py`
- `navlab/tests/companion/test_scan_reference_correction.py`
- `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
- 本文档

允许设计：

- 新增 `--max-measurement-delta-m`，默认由 runtime spec 传入。
- 保持 `--max-correction-m=0.25` 不变，用于 legacy intent/counter-drift path。
- `_measurement_decision()` 使用 `max_measurement_delta_m`，并继续要求：
  - hover phase in `hover_settle/hover_hold`
  - status fresh
  - `quality_good=true`
  - `correction_eligibility.correction_allowed=true`
  - runtime history stable
  - sign flip 不超过限制
  - direction cosine 达标
  - per-step limit 仍由 `max_correction_step_m=0.03` 执行
- status 输出必须同时暴露 `max_correction_m` 和 `max_measurement_delta_m`。

禁止：

- 不改 ExternalNav mapping。
- 不改 drift gate。
- 不接 Gazebo truth/official map/fixed prior。
- 不关闭/降低 quality gate。
- 不扩大 `max_correction_m`。

测试要求：

- 新增/修改单测证明：
  - intent path 仍被 `max_correction_m=0.25` 限制。
  - measurement fallback 可在 `max_measurement_delta_m=1.25` 下输出 `0.7m` 级 measurement。
  - measurement fallback 仍在质量坏/历史不稳定/phase 不对时 fail-closed。

验证命令：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_scan_reference_correction.py -q
cd orchestration/sim && GOCACHE=/tmp/go-cache go test ./internal/tasks/helpers -count=1
```

执行记录：

- 2026-06-22T05:30Z 开始执行 TODO E3。
- 本轮只改 E3 允许文件；先改测试，再改实现，再跑定向验证。
- 2026-06-22T05:36Z 完成实现。
- 实际改动文件：
  - `navlab/tests/companion/test_scan_reference_correction.py`
    - 新增 `test_runtime_measurement_delta_has_separate_plausibility_limit()`，证明 measurement fallback 可输出 `0.70m`，同时 legacy `max_correction_m=0.25` 不变。
  - `navlab/sim/companion/nodes/scan_reference_correction.py`
    - 新增 CLI 参数 `--max-measurement-delta-m`，默认 `1.25`。
    - `_measurement_decision()` 改用 `max_measurement_delta_m` 裁剪 scan measurement。
    - `decide_correction()` 保持旧调用兼容；没有显式传参时仍退回 `max_correction_m`。
    - status 输出新增 `max_measurement_delta_m`。
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
    - `ScanReferenceCorrectionSpec` 新增 `MaxMeasurementDeltaM`。
    - 默认值 `1.25`。
    - runtime script 生成时传 `--max-measurement-delta-m`。
- 未改内容：
  - `max_correction_m` 仍是 `0.25`。
  - `max_correction_step_m` 仍是 `0.03`。
  - 未改 ExternalNav mapping、hover gate、landing、bridge、Cartographer 参数。
- 定向验证：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_scan_reference_correction.py -q
```

- 结果：`10 passed in 0.04s`。

```bash
cd orchestration/sim && GOCACHE=/tmp/go-cache go test ./internal/tasks/helpers -count=1
```

- 结果：`ok navlab/orchestration-sim/internal/tasks/helpers 0.029s`。

### TODO F2：真实 hover 验证 E3

状态：已失败，进入 TODO D4；禁止直接继续试跑

进入条件：TODO E3 定向测试通过。

目标：验证受门控 scan measurement 输出是否让 `/slam/odom_corrected`、`/external_nav/odom`、FCU local 与 Gazebo body 的横漂差距缩小。

命令：

```bash
cd orchestration/sim && env NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover
```

成功标准仍不变：

- `summary.ok=true`
- Gazebo review-only hover drift `<=0.10m`
- `hover_xy_alignment.ok=true`
- EKF/ExternalNav/Gazebo 不再大幅分离

如果失败：

- 只记录 blocker 和四源差距。
- 禁止直接继续改；必须写下一轮 TODO。

执行记录：

- 2026-06-22T05:38Z 开始执行 TODO F2。
- 本轮只运行真实 hover 验证 E3，不追加代码改动。
- 2026-06-22T04:50:57Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T045057Z`
  - `artifacts/sim/hover/20260622T045057Z/summary.json`
- 结果：失败，`summary.ok=false`，`status=TASK_STATUS_BLOCKED`。
- blocker：
  - `hover_gazebo_model_horizontal_drift`
  - `hover_xy_alignment_direction_mismatch:external_nav_odom__slam_odom_corrected`
  - `hover_xy_alignment_direction_mismatch:gazebo_model_odometry__external_nav_odom`
  - `hover_xy_evidence_disagreement`
- 好消息：
  - mission hover 自身 `ok=true`。
  - FCU hover drift `0.0273m`，高度 crosscheck `ok=true`。
  - scan-reference runtime drift `x_m=0.7065`、`y_m=-0.1410`，仍能看到 `~0.72m` 级漂移。
  - correction status 显示 measurement fallback 可用：`phase4b_consistency_source=scan_reference_runtime_measurement_window`。
- 坏消息：
  - Gazebo drift 仍是 `0.7347m`。
  - `/slam/odom_corrected` final 只有 `(0.0482, 0.0266)`，mag `0.0550m`。
  - `/external_nav/odom` final 只有 `(-0.0444, 0.0267)`，mag `0.0518m`。
  - 说明 E3 没有让 ExternalNav 看到真实 scan measurement。
- 关键新线索：
  - `scan_reference_runtime_drift.x_m=0.7065`，但 `/slam/odom_corrected.final_x=0.0482`。
  - 当前 correction node 做的是 `out.pose.x = raw_slam.pose.x + applied_x`。
  - 如果 raw `/slam/odom` 在该窗口中约为 `-0.65m`，那么 `raw + scan_measurement` 会互相抵消，正好得到接近 0 的 `/slam/odom_corrected`。
  - 这解释了为什么 E3 放开 measurement 上限后，ExternalNav 仍然看不到真实漂移。

### TODO D4：二次诊断，measurement 应输出绝对 scan pose，不是叠加 delta

状态：已完成

进入条件：TODO F2 失败。

目标：解释 E3 为什么仍失败，并确定最小修复。

结论：

- scan-reference 的 `x_m/y_m` 语义是“相对 hover reference 的 scan-only body displacement measurement”。
- 当走 measurement fallback 时，`/slam/odom_corrected` 应该报告这个 measurement 本身，或至少报告与它同一 absolute frame 的值。
- 当前代码把 measurement 当 delta 加到 `/slam/odom` 上；如果 `/slam/odom` 已经有反号/不同 frame 的漂移估计，就会抵消 measurement，导致 ExternalNav 继续看到近零横漂。
- 因此 E4 最小修复是：给 `CorrectionDecision` 增加 `measurement_absolute`，measurement fallback 设置为 `true`；发布 odom 时 absolute 模式使用 `out.pose.pose.position.x = applied_x`、`y = applied_y`，legacy intent path 保持 `raw + delta`。

禁止：

- 不改 ExternalNav mapping。
- 不扩大 cap。
- 不放宽 quality gate。
- 不接 Gazebo truth。

### TODO E4：实现 scan measurement absolute 输出模式

状态：已完成

允许改动：

- `navlab/sim/companion/nodes/scan_reference_correction.py`
- `navlab/tests/companion/test_scan_reference_correction.py`
- 本文档

测试要求：

- 新增/修改单测证明：
  - measurement fallback decision 标记 `measurement_absolute=true`。
  - legacy intent path 标记 `measurement_absolute=false`。
  - absolute publish 逻辑不能把 raw SLAM 反号值与 scan measurement 相加。

验证：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_scan_reference_correction.py -q
```

执行记录：

- 2026-06-22T05:50Z 开始执行 TODO E4。
- 本轮只改 E4 允许文件；先测后改。
- 2026-06-22T05:55Z 完成实现。
- 实际改动文件：
  - `navlab/tests/companion/test_scan_reference_correction.py`
    - 新增 absolute measurement 替换 raw SLAM XY 的测试。
    - 新增 legacy delta path 仍叠加 raw SLAM XY 的测试。
    - 断言 measurement fallback `measurement_absolute=true`，legacy intent path `false`。
  - `navlab/sim/companion/nodes/scan_reference_correction.py`
    - `CorrectionDecision` 新增 `measurement_absolute`。
    - `_measurement_decision()` 设置 `measurement_absolute=True`。
    - legacy intent path 设置 `measurement_absolute=False`。
    - 新增 `apply_measurement_to_odom_xy()`，absolute 模式直接输出 scan measurement，避免 `raw_slam + scan_measurement` 互相抵消。
    - status 输出新增 `measurement_absolute`。
- 验证：

```bash
python3 -m py_compile navlab/sim/companion/nodes/scan_reference_correction.py
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_scan_reference_correction.py -q
```

- 结果：`12 passed in 0.04s`。

### TODO F3：真实 hover 验证 E4

状态：已失败，进入 TODO D5；禁止直接继续试跑

进入条件：TODO E4 定向测试通过。

目标：验证 absolute scan measurement 是否让 `/slam/odom_corrected`、`/external_nav/odom` 不再被 raw SLAM 反号抵消，并缩小 Gazebo/EKF 差距。

命令：

```bash
cd orchestration/sim && env NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover
```

成功标准仍不变：

- `summary.ok=true`
- Gazebo review-only hover drift `<=0.10m`
- `hover_xy_alignment.ok=true`
- EKF/ExternalNav/Gazebo 不再大幅分离

如果失败：

- 只记录 blocker 和四源差距。
- 禁止直接继续改；必须写下一轮 TODO。

执行记录：

- 2026-06-22T05:56Z 开始执行 TODO F3。
- 本轮只运行真实 hover 验证 E4，不追加代码改动。
- 2026-06-22T04:56:12Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T045612Z`
  - `artifacts/sim/hover/20260622T045612Z/summary.json`
- 结果：失败，`summary.ok=false`，`status=TASK_STATUS_BLOCKED`。
- blocker：
  - `hover_gazebo_model_horizontal_drift`
  - `hover_xy_alignment_direction_mismatch:gazebo_model_odometry__external_nav_odom`
  - `hover_xy_alignment_direction_mismatch:gazebo_model_odometry__fcu_local_position_pose`
  - `hover_xy_alignment_direction_mismatch:gazebo_model_odometry__slam_odom_corrected`
  - `hover_xy_evidence_disagreement`
- 数值：
  - Gazebo drift `1.8509m`，比 F2 `0.7347m` 更差。
  - `/slam/odom_corrected` final `(-0.0436, 0.0912)`，mag `0.1011m`，仍远小于 Gazebo。
  - `/external_nav/odom` final `(-0.0381, 0.0944)`，mag `0.1018m`。
  - FCU local final `(0.0539, -0.0297)`，mag `0.0615m`。
  - scan-reference runtime last `x_m=1.8958`、`y_m=0.0319`，说明 scan estimator 看到大漂移，但 corrected/external_nav final 没跟上。
  - correction applied ratio `25/36=0.6944`，last correction not applied，runtime consistency sample count 最后为 `0`。
- 重要诊断：
  - artifact 生成脚本确实带了 `max_measurement_delta_m=1.25` 和 `--max-measurement-delta-m`。
  - gate summary 里 `max_measurement_delta_m`、`measurement_absolute` 为 `null`，因为 `gate_evaluation.go` 的 subset 白名单还没收录新字段，不代表 runtime 没收到参数。
  - E4 后没有解决最终问题，反而放大 Gazebo 漂移；不能继续按“直接 absolute 输出”盲跑。

### TODO D5：F3 后诊断，先补 evidence 再决定下一修复

状态：部分完成，下一步需用新 summary 字段做 D6 诊断；禁止直接试跑

目标：解释为什么 scan estimator 看到 `~1.9m`，但 `/slam/odom_corrected` final 仍只有 `~0.1m`，并确认 E4 absolute 模式是否实际在 hover window 中启用、启用时是否轴向错误或间歇归零。

必须先做：

1. gate summary 白名单补齐：
   - `measurement_absolute`
   - `max_measurement_delta_m`
   - `measurement_delta_x_m/y_m`
   - `source_intent_x_m/y_m`
   - `runtime_consistency_sample_count`
   - `phase4b_consistency_source`
2. 用 F3 artifact 离线或现有 summary 判断：
   - correction applied 的那些时刻，absolute 是否启用。
   - corrected odom max `0.5066m` 是不是短时变大但 final 又归零。
   - corrected odom 与 Gazebo 的 mismatch 是轴 swap/sign，还是 intermittent fail-closed 造成。

禁止：

- 不再直接跑 hover 试错。
- 不扩大 cap。
- 不放宽 gate。
- 不改 ExternalNav mapping。

执行记录：

- 2026-06-22T06:06Z 开始执行 TODO D5。
- 本轮只补 gate evidence 字段，不改变 runtime 控制行为。
- 2026-06-22T06:10Z 完成 evidence 字段补丁。
- 实际改动文件：
  - `orchestration/sim/internal/tasks/gate_evaluation.go`
    - 在实时 status subset 和 rosbag correction summary subset 中加入：
      - `measurement_absolute`
      - `max_measurement_delta_m`
    - 原有 `measurement_delta_x_m/y_m`、`source_intent_x_m/y_m`、`runtime_consistency_sample_count`、`phase4b_consistency_source` 保持。
- 验证：

```bash
cd orchestration/sim && gofmt -w internal/tasks/gate_evaluation.go && GOCACHE=/tmp/go-cache go test ./internal/tasks -count=1
```

- 结果：`ok navlab/orchestration-sim/internal/tasks 6.361s`。
- 注意：旧 artifact `20260622T045612Z` 的 `summary.json` 不会自动回填新字段；新字段需要下一次 gate evaluation / hover run 生成后才能出现在 summary 里。

### TODO D6：基于新 evidence 字段判断 E4 失败模式

状态：已完成

目标：确认 E4 是因为：

- absolute measurement 没有持续启用；
- 或 absolute measurement 轴向/符号不匹配；
- 或 scan-reference 估计本身在 hover 后段低质量导致 fail-closed；
- 或 raw SLAM/correction topic 时间窗导致 final 采样错位。

允许：

- 可以跑一次 hover 只为生成带新 evidence 字段的 summary。
- 跑前必须把本 TODO 状态改为执行中。

禁止：

- 不再同时改代码和跑 hover。
- 不扩大 cap。
- 不放宽 quality gate。
- 不改 ExternalNav mapping。

执行记录：

- 2026-06-22T06:11Z 开始执行 TODO D6。
- 本轮只跑一次 hover 生成带 `measurement_absolute/max_measurement_delta_m` 的 evidence summary；不改代码。
- 2026-06-22T05:01:36Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T050136Z`
- 结果：失败，`summary.ok=false`。
- blocker：
  - `hover_gazebo_model_horizontal_drift`
  - `hover_xy_alignment_direction_mismatch:gazebo_model_odometry__slam_odom_corrected`
  - `hover_xy_evidence_disagreement`
- 数值：
  - Gazebo drift `0.3386m`，仍高于 `0.10m`，但低于 F3 的 `1.8509m`。
  - `/slam/odom_corrected` final `(0.0121, 0.0779)`，mag `0.0788m`。
  - `/external_nav/odom` final `(0.0216, 0.0401)`，mag `0.0455m`。
  - FCU local final `(0.0193, 0.0201)`，mag `0.0279m`。
  - scan-reference hover window quality ratio `1.0`，但 final correction not applied。
- 新字段证据：
  - `max_measurement_delta_m=1.25`，说明 E3 参数进入 runtime。
  - final `measurement_absolute=false`、`correction_applied=false`、`measurement_delta=0`。
  - `runtime_consistency_sample_count=0`，final blockers 包含 `scan_reference_runtime_eligibility_not_allowed`、`scan_reference_runtime_no_stable_axis`。
  - scan-reference last phase4B blockers：`scan_reference_phase4b_direction_not_continuous`、`scan_reference_phase4b_x_sign_flips`、`scan_reference_phase4b_y_sign_flips`。
- 直接结论：
  - E4 的 absolute mode 不是没有编译/没有传参。
  - 真正失败模式是：hover 后段 scan-reference consistency/sign-flip 失败，correction fail-closed 归零，导致 final `/slam/odom_corrected` 又回到接近 raw/near-zero。
  - 不能放宽 sign-flip/direction gate；否则可能把坏 scan measurement 拉进 EKF。

### TODO E5：设计短时 latched-safe measurement，不放宽 quality gate

状态：已完成

目标：在 scan-reference 曾经连续通过 gate 后，遇到短暂 sign-flip/direction transient 时，不立刻把 `/slam/odom_corrected` 跳回 raw near-zero；而是在很短时间内保持上一帧已通过 gate 的 absolute measurement，并且超时后 fail-closed。

允许设计：

- 增加 `max_measurement_hold_ms`，例如 `500ms`。
- 只有满足以下条件才允许 hold：
  - 当前仍在 `hover_settle/hover_hold`。
  - 当前 status fresh。
  - 当前 `quality_good=true`，不能在 quality_bad 时 hold。
  - 上一次成功 decision 是 `measurement_absolute=true`。
  - 上一次成功 decision 的 `phase4b_consistency_ok=true` 和 `runtime_consistency_ok=true`。
  - 距离上一次成功不超过 `max_measurement_hold_ms`。
- hold 输出必须在 status 中标记：
  - `measurement_hold_active=true`
  - `measurement_hold_age_ms`
  - `measurement_absolute=true`
- 超过 hold 时间或 quality_bad 必须归零 fail-closed。

禁止：

- 不放宽 sign flip gate。
- 不放宽 direction cosine。
- 不扩大 cap。
- 不改 ExternalNav mapping。

测试要求：

- 单测证明短暂 transient 可以 hold 上一次 absolute measurement。
- 单测证明 quality_bad 不允许 hold。
- 单测证明超时后归零。

执行记录：

- 2026-06-22T06:18Z 开始执行 TODO E5。
- 本轮允许改动：
  - `navlab/sim/companion/nodes/scan_reference_correction.py`
  - `navlab/tests/companion/test_scan_reference_correction.py`
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
  - `orchestration/sim/internal/tasks/gate_evaluation.go`
  - 本文档
- 本轮禁止：不改 ExternalNav mapping、不放宽 quality/sign/direction gate、不扩大 measurement cap、不跑 hover 试错。
- 执行顺序：先加单测，再实现 hold，再跑定向测试。
- 2026-06-22T06:27Z 完成实现。
- 实际改动文件：
  - `navlab/tests/companion/test_scan_reference_correction.py`
    - 新增短暂 transient 可 hold 上一帧 safe absolute measurement 的单测。
    - 新增 quality_bad 不允许 hold 的单测。
    - 新增 hold timeout 后必须 fail-closed 的单测。
  - `navlab/sim/companion/nodes/scan_reference_correction.py`
    - `CorrectionDecision` 新增 `measurement_hold_active`、`measurement_hold_age_ms`。
    - 新增 `maybe_hold_measurement_decision()`。
    - node 保存上一帧通过 gate 的 absolute measurement。
    - 当前 status fresh、quality_good、hover phase 正确、frame ok、上一帧 absolute measurement 通过 runtime/phase4B 且未超 `max_measurement_hold_ms` 时，短时 hold；否则 fail-closed。
    - status 输出 hold 状态。
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
    - `ScanReferenceCorrectionSpec` 新增 `MaxMeasurementHoldMS=500.0`。
    - runtime script 传 `--max-measurement-hold-ms`。
  - `orchestration/sim/internal/tasks/gate_evaluation.go`
    - summary 白名单新增 `measurement_hold_active`、`measurement_hold_age_ms`、`max_measurement_hold_ms`。
- 验证：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_scan_reference_correction.py -q
```

- 结果：`15 passed in 0.35s`。

```bash
cd orchestration/sim && gofmt -w internal/tasks/gate_evaluation.go internal/tasks/helpers/runtime_specs.go && GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -count=1
```

- 结果：
  - `ok navlab/orchestration-sim/internal/tasks 6.377s`
  - `ok navlab/orchestration-sim/internal/tasks/helpers 0.024s`

### TODO F4：真实 hover 验证 E5

状态：已失败，E4/E5 方向证伪；进入 TODO E6 回滚

进入条件：TODO E5 定向测试通过。

目标：验证短时 hold 是否避免 hover 后段 correction 归零，从而缩小 Gazebo 与 EKF/ExternalNav 的漂移差距。

命令：

```bash
cd orchestration/sim && env NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover
```

成功标准仍不变：

- `summary.ok=true`
- Gazebo review-only hover drift `<=0.10m`
- `hover_xy_alignment.ok=true`
- EKF/ExternalNav/Gazebo 不再大幅分离

如果失败：

- 只记录 blocker 和四源差距。
- 禁止直接继续改；必须写下一轮 TODO。

执行记录：

- 2026-06-22T06:28Z 开始执行 TODO F4。
- 本轮只运行真实 hover 验证 E5，不追加代码改动。
- 2026-06-22T05:09:38Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T050938Z`
- 结果：失败，`summary.ok=false`。
- 关键数值：
  - Gazebo drift `1.9221m`，明显比 D6 `0.3386m` 更差。
  - `/slam/odom_corrected` final `(-0.0714,-0.0216)`，mag `0.0746m`。
  - `/external_nav/odom` final `(-0.0706,-0.0295)`，mag `0.0765m`。
  - FCU local drift `0.3387m`，mission hover 自身也失败。
  - final `measurement_hold_active=false`、`correction_applied=false`，quality_bad 时按设计未 hold。
- 结论：
  - absolute measurement + short hold 没有缩小 Gazebo/EKF 差距，反而使真实 Gazebo drift 恶化。
  - 按“不保留已证伪补丁”的纪律，必须回滚 E4/E5 的 absolute/hold 行为。
  - 保留 E3 的独立 measurement delta cap 和 evidence 字段，因为 E3 没引入 absolute 输出，且仍可作为后续诊断信息。

### TODO E6：回滚 E4/E5 的 absolute/hold 行为

状态：已完成

目标：撤销导致 Gazebo drift 恶化的 absolute measurement 和 short hold 逻辑，回到 E3 的“受门控 measurement delta + 独立 max_measurement_delta_m”状态。

允许改动：

- `navlab/sim/companion/nodes/scan_reference_correction.py`
- `navlab/tests/companion/test_scan_reference_correction.py`
- `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
- `orchestration/sim/internal/tasks/gate_evaluation.go`
- 本文档

保留：

- `max_measurement_delta_m`
- `max_measurement_delta_m` summary evidence

删除/回滚：

- `measurement_absolute`
- `measurement_hold_active`
- `measurement_hold_age_ms`
- `max_measurement_hold_ms`
- absolute 替换 raw SLAM XY 的发布逻辑

验证：

```bash
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_scan_reference_correction.py -q
cd orchestration/sim && GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -count=1
```

执行记录：

- 2026-06-22T06:38Z 开始执行 TODO E6。
- 本轮只回滚已证伪的 absolute/hold 行为，保留 E3 measurement cap。
- 2026-06-22T06:45Z 继续执行 TODO E6，先记录再改代码。
- 本轮只允许清理已确认残留：
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go` 中的 `MaxMeasurementHoldMS` 字段、默认值和 runtime payload。
  - `orchestration/sim/internal/tasks/gate_evaluation.go` 中的 `measurement_absolute`、`measurement_hold_active`、`measurement_hold_age_ms`、`max_measurement_hold_ms` summary 白名单。
- 本轮确认不改：
  - 不改 hover 高度门槛。
  - 不改 landing 阈值。
  - 不改 Gazebo drift gate。
  - 不改 ExternalNav mapping。
  - 不改 Foxglove/TF 显示。
  - 不跑真实 hover。
- 本轮验收：
  - `rg` 查不到 E4/E5 absolute/hold 残留。
  - Python 编译通过。
  - scan-reference correction 单测通过。
  - orchestration sim 相关 Go 测试通过。
- 2026-06-22T06:50Z 完成 E6 清理。
- 实际改动文件：
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
    - 删除 `ScanReferenceCorrectionSpec.MaxMeasurementHoldMS`。
    - 删除默认 `MaxMeasurementHoldMS: 500.0`。
    - 删除 runtime payload `max_measurement_hold_ms`。
    - 删除 runtime argv `--max-measurement-hold-ms`。
    - 保留 `MaxMeasurementDeltaM=1.25` 和 `--max-measurement-delta-m`。
  - `orchestration/sim/internal/tasks/gate_evaluation.go`
    - 删除 scan-reference correction summary 白名单中的 `measurement_absolute`、`measurement_hold_active`、`measurement_hold_age_ms`、`max_measurement_hold_ms`。
    - 保留 `measurement_delta_x_m/y_m`、`source_intent_x_m/y_m`、`runtime_consistency_sample_count`、`phase4b_consistency_source`、`max_measurement_delta_m`。
  - `docs/notes/hover_external_nav_frame_root_cause_plan_20260622.md`
    - 补充本轮执行范围、禁止事项和验证结果。
- 残留检查：

```bash
rg -n "measurement_absolute|measurement_hold|max_measurement_hold|apply_measurement_to_odom_xy|maybe_hold_measurement_decision|MaxMeasurementHold" \
  navlab/sim/companion/nodes/scan_reference_correction.py \
  navlab/tests/companion/test_scan_reference_correction.py \
  orchestration/sim/internal/tasks/helpers/runtime_specs.go \
  orchestration/sim/internal/tasks/gate_evaluation.go
```

- 结果：无输出，E4/E5 absolute/hold 残留已清空。
- 验证：

```bash
python3 -m py_compile navlab/sim/companion/nodes/scan_reference_correction.py
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_scan_reference_correction.py -q
cd orchestration/sim && gofmt -w internal/tasks/gate_evaluation.go internal/tasks/helpers/runtime_specs.go && GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -count=1
```

- 结果：
  - `10 passed in 0.03s`
  - `ok navlab/orchestration-sim/internal/tasks 6.356s`
  - `ok navlab/orchestration-sim/internal/tasks/helpers 0.025s`
- E6 结论：
  - 已证伪的 absolute/hold 路线已回滚干净。
  - 当前没有宣称 hover 成功。
  - 下一步不能恢复 absolute/hold，也不能扩大 correction；必须先诊断 scan-reference measurement 与 Gazebo/body、SLAM/map 的 x/y/sign/yaw 映射关系。

### TODO D7：诊断 scan-reference measurement 与 SLAM/Gazebo frame 的 swap/sign/yaw 根因

状态：已完成

目标：解释为什么 scan-reference estimator 能看到大横漂，但写入 `/slam/odom_corrected` 后方向/幅值不能让 Gazebo body、ExternalNav、FCU 对齐。

核心假设：

- 不是 correction cap 太小。
- 不是 hover gate 太严。
- 不是 Foxglove 显示问题。
- 更可能是 `/scan` 角度约定、`laser_frame/base_scan/base_link` mounting yaw、SLAM map frame、Gazebo review-only odom frame 之间存在 x/y swap、sign 或 yaw 旋转合同不一致。

允许操作：

- 只读当前代码、launch、URDF/SDF、Cartographer lua 和现有 artifact。
- 写离线诊断脚本到 `/tmp`；如果要进入 repo，必须先在本 TODO 追加文件名和目的。
- 读取官方 Cartographer 配置文档作为参数/坐标解释依据。
- 用现有 artifact 做候选映射枚举：`(x,y)`、`(x,-y)`、`(-x,y)`、`(-x,-y)`、`(y,x)`、`(y,-x)`、`(-y,x)`、`(-y,-x)`，必要时加固定 yaw 旋转。

禁止操作：

- 不改 ExternalNav mapping。
- 不接 Gazebo truth / official map / fixed prior 到 runtime。
- 不改 hover 高度门槛、landing 阈值、Gazebo drift gate。
- 不改 Foxglove display TF 或全局 runtime TF。
- 不把 scan-reference correction 接控制。
- 不跑真实 hover；D7 是离线根因诊断。

必须产出：

- 表格：`/scan`、`laser_frame`、`base_scan`、`base_link`、`/slam/odom`、`/slam/odom_corrected`、`/gazebo/model/odometry` 的 frame、parent、axis 证据。
- 表格：每个候选映射与 Gazebo review-only drift 的 cosine、magnitude ratio、final error。
- 明确判断：
  - scan-reference 估计的是正确漂移但 frame 写错；
  - 或 scan-reference 本身观测模型错；
  - 或 Gazebo body drift 与 lidar scan 几何不一致；
  - 或 SLAM map frame/ExternalNav bridge 丢失了正确映射。

验收：

- 不能只写“可能是 frame 问题”；必须给出数值排序最高的候选映射和对应证据。
- 如果没有单一候选稳定胜出，下一 TODO 必须是修 estimator/观测模型，而不是接控制。

执行记录：

- 2026-06-22T06:52Z 开始执行 TODO D7。
- 本轮只做离线根因诊断；先记录再读取证据，不改 runtime 代码。
- 本轮证据范围：
  - 当前代码中的 scan-reference estimator、scan/correction node、Cartographer lua/launch、Gazebo sensor/bridge 配置。
  - 已有 artifact：优先 `artifacts/sim/hover/20260622T050136Z`，必要时对比 `20260622T041059Z`、`20260622T050938Z`。
  - Cartographer 官方 Configuration 文档，仅用于核对参数语义和 frame 配置语义。
- 本轮禁止：
  - 不改 ExternalNav mapping。
  - 不改 correction 控制接入。
  - 不跑真实 hover。
  - 不改 Foxglove/TF/hover/landing/gate。
- 2026-06-22T07:08Z 完成 D7 离线诊断。
- 官方依据：
  - Cartographer ROS Configuration：`map_frame` 是 submap/pose parent，`tracking_frame` 是 SLAM 跟踪 frame，IMU 场景通常用 IMU frame，`published_frame` 是发布 pose 的 child，`use_odometry=true` 会订阅 `nav_msgs/Odometry` 并把 odom 信息纳入 SLAM。
  - Cartographer ROS API：所有输入 sensor frame 到 `tracking_frame` / `published_frame` 的 TF 必须可用；如果 `provide_odom_frame=false`，Cartographer 提供的是 `map_frame -> published_frame`。
- 当前 frame 证据表：

| 项 | 当前证据 | 结论 |
| --- | --- | --- |
| `/scan` | artifact probe：`frame_id=base_scan`、`angle_min=-pi`、`angle_max=pi`、`angle_increment=0.014646` | scan 自身是 `base_scan`，不是 `laser_frame` |
| `base_link -> base_scan` | artifact `/tf_static`：translation `(0,0,0.075077)`、rotation identity | LiDAR yaw 没有 90 度旋转；scan x/y 不应 swap |
| scan-reference estimator | `scan_reference_drift.py` 使用 `ux=cos(theta)`、`uy=sin(theta)`；输出 `frame_id=scan_reference`、`child=base_link`、`x_m/y_m` | 输出是 scan/base_link native XY 的观测位移 |
| Cartographer hover config | runtime log 确认使用 `navlab_cartographer_2d_hover.lua`；`map_frame=map`、`tracking_frame=imu_link`、`published_frame=base_link`、`provide_odom_frame=false`、`use_odometry=true` | 配置形态符合官方 frame 语义，且 TF 可用 |
| `/slam/odom` | artifact probe：`frame_id=map`、`child=base_link`，但 status `last_x≈-0.002,last_y≈0` | Cartographer 输出在 hover 中接近零，没有跟随 scan 观测的大横漂 |
| `/slam/odom_corrected` | 由 correction node 在 `/slam/odom` 上加 `measurement_delta`；fail-closed 时回到 raw SLAM near-zero | 这里会把正确 scan measurement 间歇丢掉 |
| `/gazebo/model/odometry` | review-only，`frame_id=odom`、`child=base_link`，bridge topic `/model/iris_with_lidar/odometry` | 作为真值审查，不进入 runtime 输入 |

- 候选映射数值诊断，左侧统一和 Gazebo raw `/gazebo/model/odometry` final 向量比 cosine：

| Run | Gazebo final | scan hover native `(x,y)` cosine / ratio | scan runtime last native `(x,y)` cosine / ratio | 最佳结论 |
| --- | --- | --- | --- | --- |
| `20260622T041059Z` | `(0.558,-0.422)` | `0.9994 / 1.06` | `1.0000 / 1.91` | scan native x/y 与 Gazebo 同向 |
| `20260622T045057Z` | `(0.733,-0.052)` | `0.9714 / 0.79` | `0.9920 / 0.98` | scan native x/y 与 Gazebo 同向 |
| `20260622T050136Z` | `(0.273,-0.200)` | `1.0000 / 0.82` | `0.9997 / 3.09` | scan native x/y 与 Gazebo 同向 |
| `20260622T050938Z` | `(1.811,-0.644)` | `0.8976 / 0.77`，`(-y,-x)` 为 `0.9087 / 0.77` | `0.9977 / 0.75` | runtime last 仍支持 native x/y，同窗 hover 有后段失稳 |

- 对照 `/slam/odom_corrected` / `/external_nav/odom`：

| Run | `/slam/odom_corrected` final mag/Gazebo mag | `/external_nav/odom` final mag/Gazebo mag | 结论 |
| --- | ---: | ---: | --- |
| `20260622T041059Z` | `0.123` | `0.127` | corrected/external_nav 只剩 Gazebo 的约 12%，且方向需要 swap 才看起来接近，说明不是固定 frame 合同 |
| `20260622T045057Z` | `0.075` | `0.070` | corrected/external_nav 幅值过小 |
| `20260622T050136Z` | `0.233` | `0.134` | corrected/external_nav 幅值过小，final fail-closed 后接近 raw SLAM |
| `20260622T050938Z` | `0.039` | `0.040` | E5 后 corrected/external_nav 几乎丢失 scan displacement |

- D7 结论：
  - scan-reference estimator 不是主要 x/y swap/sign 根因；它的 native `(x,y)` 与 Gazebo raw body drift 在多轮 artifact 中高度同向。
  - `base_link -> base_scan` yaw 为 identity，因此没有证据支持“LiDAR 安装 yaw 90 度导致必须 swap”。
  - 真实断点在 scan measurement 之后：Cartographer `/slam/odom` 在 hover 中把可见横漂压到 near-zero；correction node 又在 gate/fail-closed/step 限制下把 scan measurement 间歇丢掉，最终 `/slam/odom_corrected` 和 `/external_nav/odom` 仍接近零。
  - 所以 EKF/ExternalNav 看起来稳，Gazebo body 继续漂；这不是 Foxglove，也不是 Gazebo odom source 错。
  - 下一步不能再调 hover 高度/landing/gazebo gate，也不能恢复 E4/E5 absolute/hold；要修的是“ExternalNav 输入源合同”：当 Cartographer 输出 near-zero 但 scan-only odometry 质量稳定时，必须有受 gate 保护的 LiDAR odometry source，而不是把 scan measurement 当一个会随 fail-closed 归零的小补丁。

### TODO E7：设计受 gate 保护的 scan-reference ExternalNav candidate，不再把正确 scan 位移丢回 raw SLAM near-zero

状态：Slice A 已完成，未接 runtime

目标：把 D7 已证明同向的 scan-reference odometry 作为 ExternalNav 候选源进行设计，但仍 fail-closed，不接 Gazebo truth、不用 official map、不改 hover mission。

允许改动范围：

- 先写单测，再改代码。
- 允许新增/修改一个 runtime source-selector 或 bridge 输入选择逻辑，候选输入只能来自：
  - `/slam/odom`
  - `/navlab/scan_reference_drift/odom`
  - `/navlab/scan_reference_drift/status`
  - `/navlab/hover/status`
- 允许新增状态 topic，例如 `/external_nav/source_selector/status`，但必须写入 summary evidence。
- 允许让 ExternalNav bridge 在 hover 阶段订阅 selector 输出 topic，例如 `/external_nav/odom_candidate`。

禁止：

- 禁止 Gazebo truth / official map / fixed prior runtime 输入。
- 禁止把 Gazebo review-only drift 作为 selector 条件。
- 禁止改 ExternalNav ROS->MAVLink x/y mapping。
- 禁止放宽 `hover_gazebo_model_horizontal_drift <= 0.10m`。
- 禁止用 Foxglove display TF 掩盖。
- 禁止恢复 E4/E5 absolute/hold。

selector 必须 fail-closed：

- scan-reference status fresh。
- `quality_good=true`。
- residual / inlier / valid beams 达标。
- `correction_eligibility.correction_allowed=true`。
- runtime window 方向连续、x/y sign flip 不超过阈值。
- 输出 frame 必须显式为 `map/base_link` 或明确新 frame 合同；不能含糊地把 `scan_reference` frame 直接冒充 `map`。
- 如果 Cartographer `/slam/odom` 与 scan-reference 严重不一致，status 必须标记 `cartographer_scan_disagreement=true`，不能静默融合。

验收：

- 单测覆盖：
  - scan-reference 稳定时 selector 输出 scan candidate。
  - scan-reference quality_bad / stale / sign flip 时 fail-closed。
  - Cartographer near-zero 但 scan 稳定时不再输出 raw near-zero。
  - selector status 说明当前 source、blockers、是否使用 Gazebo truth。
- 定向测试通过后，才允许下一 TODO 跑真实 hover。

执行记录：

- 2026-06-22T07:13Z 开始执行 TODO E7 Slice A。
- Slice A 只做 ROS-independent selector 决策函数和单测，不接 runtime、不跑 hover。
- Slice A 允许改动：
  - `navlab/sim/companion/nodes/external_nav_source_selector.py`
  - `navlab/tests/companion/test_external_nav_source_selector.py`
  - 本文档
- Slice A 禁止：
  - 不改 `runtime_artifacts.go`。
  - 不改 external nav bridge 输入 topic。
  - 不改 hover mission / landing / Gazebo gate / Foxglove / TF。
  - 不跑真实 hover。
- 2026-06-22T07:21Z 完成 TODO E7 Slice A。
- 实际改动文件：
  - `navlab/sim/companion/nodes/external_nav_source_selector.py`
    - 新增 ROS-independent `select_external_nav_source()`。
    - 稳定 scan-reference 时输出 `source=scan_reference`，输出 observed displacement，不输出 counter-drift intent。
    - scan-reference bad/stale/sign flip/non-hover 时 fail-closed 到 `source=slam_passthrough`。
    - status 明确 `cartographer_scan_disagreement`、`uses_gazebo_truth_input=false`、`uses_known_map_input=false`。
    - ROS node 入口已写好，但本 Slice 未接 runtime。
  - `navlab/tests/companion/test_external_nav_source_selector.py`
    - 覆盖稳定 scan-reference 胜过 near-zero Cartographer。
    - 覆盖 quality bad、stale、sign flip、non-hover fail-closed。
- 验证：

```bash
python3 -m py_compile navlab/sim/companion/nodes/external_nav_source_selector.py
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_source_selector.py -q
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_source_selector.py navlab/tests/companion/test_scan_reference_correction.py -q
```

- 结果：
  - `5 passed in 0.02s`
  - `15 passed in 0.02s`
- Slice A 结论：
  - 已有可测的 selector 决策层，证明下一步可以让稳定 scan-reference 不再被 raw SLAM near-zero 覆盖。
  - 仍未接 runtime，所以不能跑 hover，也不能说 hover 已修好。
  - 下一步必须新增 Slice B 文档后再做：把 selector runtime script 加入 artifacts/plan，并把 hover ExternalNav bridge 输入从 `/slam/odom_corrected` 切到 `/external_nav/odom_candidate`。

### TODO E8：E7 Slice B runtime 接入 selector，但仍不跑 hover

状态：已完成，未跑真实 hover

目标：把 `external_nav_source_selector.py` 接入 hover runtime artifact/服务/rosbag/summary，使 ExternalNav bridge 在 hover 任务中读 `/external_nav/odom_candidate`，但本 TODO 只做生成和单测，不跑真实 hover。

允许改动：

- `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
- `orchestration/sim/internal/tasks/helpers/execution_plan.go`
- `orchestration/sim/internal/tasks/runtime_artifacts.go`
- `orchestration/sim/internal/tasks/helpers/rosbag_topic_sets.go`
- `orchestration/sim/internal/tasks/gate_evaluation.go`
- 相关 Go 单测
- 本文档

必须改出的合同：

- 新增 selector runtime script artifact：`external_nav_source_selector_runtime.py`。
- 新增 runtime service：`external_nav_source_selector`，必须在 ExternalNav bridge 前启动。
- hover SLAM runtime 的 `ExternalNavInputOdomTopic` 改为 `/external_nav/odom_candidate`。
- summary 必须收录 `/external_nav/source_selector/status`：
  - `source`
  - `blockers`
  - `cartographer_scan_disagreement`
  - `uses_gazebo_truth_input`
  - `uses_known_map_input`
  - `output_odom_topic`
  - `output_frame_id`
  - `output_child_frame_id`

禁止：

- 不跑真实 hover。
- 不改 ExternalNav ROS->MAVLink mapping。
- 不改 hover/landing/gazebo gate。
- 不接 Gazebo truth / official map / fixed prior。
- 不删旧 `/slam/odom_corrected`，但 ExternalNav hover 输入不能继续指向它。

验收：

- Go 单测证明 generated artifact 和 execution plan 包含 selector。
- Go 单测证明 hover 的 `external_nav_input_odom_topic = '/external_nav/odom_candidate'`。
- Go 单测证明 summary 收录 selector status，并拒绝 `uses_gazebo_truth_input=true`。

执行记录：

- 2026-06-22T07:28Z 开始执行 TODO E8。
- 本轮先写记录再改代码。
- 本轮只做 runtime 接入与测试，不跑真实 hover。
- 本轮补充验收命令：

```bash
python3 -m py_compile navlab/sim/companion/nodes/external_nav_source_selector.py
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_source_selector.py -q
cd orchestration/sim && gofmt -w internal/tasks/helpers/runtime_specs.go internal/tasks/helpers/execution_plan.go internal/tasks/runtime_artifacts.go internal/tasks/helpers/rosbag_topic_sets.go internal/tasks/gate_evaluation.go internal/tasks/helpers/execution_plan_test.go internal/tasks/runtime_artifacts_test.go internal/tasks/gate_evaluation_test.go && GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -count=1
```

- 本轮明确禁止：
  - 不运行 `go run ./cmd/navlab-sim run hover`。
  - 不改 `navlab/real/companion/nodes/external_nav.py` 的 MAVLink x/y mapping。
  - 不改 hover/landing/Gazebo drift gate。
- 2026-06-22T07:40Z 完成 TODO E8。
- 实际改动文件：
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
    - 新增 `ExternalNavSourceSelectorSpec`。
    - 新增默认 selector topic：`/external_nav/odom_candidate` 和 `/external_nav/source_selector/status`。
    - 新增 `ExternalNavSourceSelectorRuntimeScript()`。
    - `HoverProbeScript()` 增加 selector output/status topic。
  - `orchestration/sim/internal/tasks/runtime_artifacts.go`
    - hover 的 `ExternalNavInputOdomTopic` 改为 `/external_nav/odom_candidate`。
    - `external_nav_bridge_params.yaml` 也改为订阅 `/external_nav/odom_candidate`。
    - 新增生成 `external_nav_source_selector_runtime.py`。
  - `orchestration/sim/internal/tasks/helpers/execution_plan.go`
    - 新增 `external_nav_source_selector_runtime.py` artifact。
    - 新增 `external_nav_source_selector` runtime service。
    - hover 任务中把 `external_nav_source_selector` service 排到 `slam_backend` 前；因为 ExternalNav bridge 由 `slam_backend` launch 内部启动，这样是当前 orchestration 能表达的最早启动顺序。
    - `slam_hover_probe` 增加 `/external_nav/odom_candidate` 和 `/external_nav/source_selector/status`。
  - `orchestration/sim/internal/tasks/helpers/rosbag_topic_sets.go`
    - hover review/required topics 增加 selector output/status。
  - `orchestration/sim/internal/tasks/gate_evaluation.go`
    - `MetricSummary` 新增 `external_nav_source_selector`。
    - summary 收录 `/external_nav/source_selector/status` 的 source/blockers/truth/map/output frame 证据。
    - `externalNavSourceSelectorBlockers()` 拒绝 Gazebo truth、known map、错误 output topic、错误 output frame。
    - `/external_nav/odom_candidate` 加入 hover ExternalNav 允许输入。
  - `orchestration/sim/internal/tasks/helpers/execution_plan_test.go`
    - 证明 execution plan 包含 selector artifact/service/probe/rosbag topics。
    - 证明 selector service 在 `slam_backend` 前。
  - `orchestration/sim/internal/tasks/runtime_artifacts_test.go`
    - 证明 generated artifact 包含 `external_nav_source_selector_runtime.py`。
    - 证明 hover runtime 和 bridge params 使用 `/external_nav/odom_candidate`。
  - `orchestration/sim/internal/tasks/gate_evaluation_test.go`
    - 证明 summary 收录 selector status。
    - 证明 selector truth input 会被 blocker 拒绝。
    - 证明 `/external_nav/odom_candidate` 是允许的 hover ExternalNav 输入。
- 验证：

```bash
python3 -m py_compile navlab/sim/companion/nodes/external_nav_source_selector.py
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_source_selector.py -q
cd orchestration/sim && gofmt -w internal/tasks/helpers/runtime_specs.go internal/tasks/helpers/execution_plan.go internal/tasks/runtime_artifacts.go internal/tasks/helpers/rosbag_topic_sets.go internal/tasks/gate_evaluation.go internal/tasks/helpers/execution_plan_test.go internal/tasks/runtime_artifacts_test.go internal/tasks/gate_evaluation_test.go && GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -count=1
```

- 结果：
  - `5 passed in 0.01s`
  - `ok navlab/orchestration-sim/internal/tasks 6.347s`
  - `ok navlab/orchestration-sim/internal/tasks/helpers 0.026s`
- E8 结论：
  - selector 已经接入 runtime artifact/plan/rosbag/summary。
  - hover ExternalNav 输入现在按生成配置应从 `/external_nav/odom_candidate` 进入，而不是继续直接吃 `/slam/odom_corrected`。
  - 尚未跑真实 hover，所以还不能说 Gazebo/EKF 偏移已解决。

### TODO F5：真实 hover 验证 selector runtime 接入

状态：已失败，进入 TODO E9

进入条件：

- TODO E8 全部定向测试通过。

目标：

- 跑一次真实 hover，验证 `/external_nav/source_selector/status` 真实出现，ExternalNav bridge 实际输入为 `/external_nav/odom_candidate`，并检查 Gazebo body 与 EKF/ExternalNav 的水平偏移是否缩小。

命令：

```bash
cd orchestration/sim && env NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover
```

必须检查：

- `summary.ok`
- `metrics.gate.external_nav_source_selector`
- `metrics.gate.external_nav.odom.input_topic`
- `metrics.gate.mavlink_external_nav.input_topic`
- `metrics.gate.hover_xy_alignment`
- `metrics.gate.gazebo_model_hover_drift.max_horizontal_drift_m`
- `/external_nav/odom_candidate`
- `/external_nav/source_selector/status`

成功标准：

- `summary.ok=true`
- Gazebo review-only hover drift `<= 0.10m`
- `hover_xy_alignment.ok=true`
- `external_nav_source_selector.uses_gazebo_truth_input=false`
- `external_nav_source_selector.uses_known_map_input=false`
- `external_nav.odom.input_topic=/external_nav/odom_candidate`

如果失败：

- 不直接继续调参数。
- 先写下一轮 TODO，按 selector status / hover_xy_alignment / Gazebo drift 证据判断是 selector 没启动、scan gate fail-closed、ExternalNav 没吃 candidate，还是 EKF 仍和 Gazebo 分离。

执行记录：

- 2026-06-22T07:43Z 开始执行 TODO F5。
- 本轮只运行一次真实 hover 验证 E8 runtime 接入，不追加代码改动。
- 跑后必须记录 artifact、summary.ok、Gazebo drift、selector status、ExternalNav input topic。
- 2026-06-22T05:33:24Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T053324Z`
- 结果：失败，`summary.ok=false`，`status=TASK_STATUS_ERROR`。
- 这次不能作为 hover 横漂成功/失败判定，因为 required probes 被 killed：
  - `rangefinder_probe: signal: killed`
  - `frame_contract_probe: signal: killed`
  - `slam_hover_probe: signal: killed`
- 但 runtime 接入证据是真实存在的：
  - `slam_backend.runtime.log` 显示 `external_nav_input_odom_topic:=/external_nav/odom_candidate`。
  - `external_nav_bridge_params.yaml` 显示 `input_odom_topic: /external_nav/odom_candidate`。
  - rosbag metadata 显示 `/external_nav/odom_candidate` 有 `18666` 条。
  - rosbag metadata 显示 `/external_nav/source_selector/status` 有 `179` 条。
  - rosbag metadata 显示 `/external_nav/status` 有 `179` 条。
  - `/external_nav/odom` 为 `0` 条，所以 ExternalNav bridge 没真正输出 odom。
- 当前 blocker 的关键不是 Gazebo 横漂，而是：
  - summary 只从 probe 取 `/external_nav/source_selector/status` 和 `/external_nav/status`；probe 被 participant limit 杀掉后，summary 报 `external_nav_source_selector_status_missing` / `external_nav_status_missing`，但 rosbag 里实际有这些 status。
  - 必须先补 rosbag fallback summary，才能准确判断 selector/source/bridge 为什么没有输出 `/external_nav/odom`。

### TODO E9：summary 从 rosbag 兜底解析 selector/external_nav status，避免 probe killed 后误判 missing

状态：已完成

目标：当 probe 被 killed 或没有采到时，gate summary 必须从 hover rosbag 的 status topic 兜底提取最新 JSON status。这样才能继续判断 runtime 真问题，而不是停在 `*_status_missing` 假 blocker。

允许改动：

- `orchestration/sim/internal/tasks/gate_evaluation.go`
- `orchestration/sim/internal/tasks/gate_evaluation_test.go`
- 本文档

必须支持的 rosbag fallback topic：

- `/external_nav/source_selector/status`
- `/external_nav/status`
- `/mavlink_external_nav/status`

必须收录：

- source selector：
  - `source`
  - `blockers`
  - `cartographer_scan_disagreement`
  - `uses_gazebo_truth_input`
  - `uses_known_map_input`
  - `output_odom_topic`
  - `output_frame_id`
  - `output_child_frame_id`
- external_nav：
  - `state`
  - `ready`
  - `odom.input_topic`
  - `odom.rate_hz`
  - `odom.ready`
  - `height.ready`
  - `output.topic`
- mavlink_external_nav：
  - `state`
  - `ready`
  - `input_topic`
  - `sent_count`
  - `fcu_local_position_ready`

禁止：

- 不改 selector 控制逻辑。
- 不改 ExternalNav mapping。
- 不跑 hover。

验收：

- 单测构造 MCAP String topic，证明 probe 缺失时也能从 rosbag 填充 `ExternalNavSourceSelector`。
- 单测证明 probe 缺失时也能从 rosbag 填充 `ExternalNav`。
- `go test ./internal/tasks -count=1` 通过。

执行记录：

- 2026-06-22T07:54Z 开始执行 TODO E9。
- 本轮只做 rosbag fallback summary 和单测，不改 runtime 控制链，不跑 hover。
- 2026-06-22T08:04Z 完成 TODO E9。
- 实际改动文件：
  - `orchestration/sim/internal/tasks/gate_evaluation.go`
    - 新增 `summarizeLatestJSONStatusFromRosbag()`。
    - 新增 `summarizeLatestJSONStatusFromHoverRosbag()`。
    - 当 probe 缺失时，从 hover rosbag 兜底填充：
      - `ExternalNavSourceSelector`
      - `ExternalNav`
      - `MAVLinkExternalNav`
  - `orchestration/sim/internal/tasks/gate_evaluation_test.go`
    - 新增 selector status rosbag fallback 单测。
    - 新增 external_nav status rosbag fallback 单测。
- 验证：

```bash
cd orchestration/sim && gofmt -w internal/tasks/gate_evaluation.go internal/tasks/gate_evaluation_test.go && GOCACHE=/tmp/go-cache go test ./internal/tasks -count=1
python3 -m py_compile navlab/sim/companion/nodes/external_nav_source_selector.py
uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_source_selector.py navlab/tests/companion/test_scan_reference_correction.py -q
cd orchestration/sim && GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -count=1
```

- 结果：
  - `ok navlab/orchestration-sim/internal/tasks 6.342s`
  - `15 passed in 0.02s`
  - `ok navlab/orchestration-sim/internal/tasks 6.332s`
  - `ok navlab/orchestration-sim/internal/tasks/helpers 0.028s`
- E9 结论：
  - summary 不再只能依赖 probe 来拿 selector/external_nav/mavlink status。
  - 下一次 hover 如果 probe 再被 killed，仍应能从 rosbag 看到 selector 和 ExternalNav 的真实状态。

### TODO F6：重新跑真实 hover，使用 E9 rosbag fallback 判定真实 blocker

状态：已失败，进入 TODO D10

目标：重新跑 hover，确认 E9 后 summary 能显示 selector/external_nav status，并定位 `/external_nav/odom` 为 0 的真实原因。

命令：

```bash
cd orchestration/sim && env NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover
```

必须检查：

- `metrics.gate.external_nav_source_selector`
- `metrics.gate.external_nav`
- `metrics.gate.mavlink_external_nav`
- `/external_nav/odom_candidate` message count
- `/external_nav/odom` message count
- `external_nav.odom.input_topic`
- `external_nav.odom.ready`
- `external_nav.height.ready`

如果 `/external_nav/odom_candidate` 有数据但 `/external_nav/odom` 仍为 0：

- 下一 TODO 只查 ExternalNav bridge 为什么不发布，重点看 frame、height readiness、bridge quality gate、source selector output frame，不改 hover/gazebo gate。

执行记录：

- 2026-06-22T08:06Z 开始执行 TODO F6。
- 本轮只重新跑 hover 验证 E9 后 summary 和 ExternalNav bridge 真实 blocker，不追加代码改动。
- 2026-06-22T05:40:45Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T054045Z`
- 结果：失败，`summary.ok=false`，`status=TASK_STATUS_BLOCKED`。
- E9 生效证据：
  - `metrics.gate.external_nav_source_selector.status_source=rosbag_latest_json_status`
  - `metrics.gate.external_nav.status_source=rosbag_latest_json_status`
  - `metrics.gate.mavlink_external_nav.status_source=rosbag_latest_json_status`
- selector/bridge 已真实接入：
  - `/external_nav/odom_candidate` count `14353`
  - `/external_nav/source_selector/status` count `180`
  - `/external_nav/status` count `179`
  - `/external_nav/odom` count `179`
  - `external_nav.odom.input_topic=/external_nav/odom_candidate`
  - `external_nav.ready=true`
  - `mavlink_external_nav.ready=true`
  - `mavlink_external_nav.sent_count=1793`
- hover 仍失败：
  - Gazebo review-only hover drift `0.8575m`
  - `/external_nav/odom` final `(-0.5065, 0.3588)`，magnitude `0.6207m`，和 Gazebo direction cosine `0.7802`
  - FCU local hover drift `0.2451m`，和 Gazebo direction cosine `0.1000`
  - `/slam/odom_corrected` final magnitude `0.0788m`
- 直接结论：
  - E7/E8 确实把 scan-reference candidate 接进 ExternalNav bridge 了，已经不是之前 `/external_nav/odom` near-zero 的问题。
  - 现在主要断点变成：`/external_nav/odom` 已经看到一部分 Gazebo 横漂，但 FCU/EKF local 只响应了更小的一部分，而且方向不一致。
  - 下一步不能继续调 selector cap 或 hover gate；必须诊断 `/external_nav/odom -> MAVLink ODOMETRY -> LOCAL_POSITION_NED` 的 x/y/sign/yaw/covariance/quality 合同。

### TODO D10：诊断 ExternalNav -> MAVLink/EKF 映射和融合差距

状态：已完成

目标：解释为什么 `/external_nav/odom` 已经有 `0.62m` 级别水平位移，但 FCU local 只有 `0.245m` 且与 Gazebo 方向 cosine 只有 `0.10`。

允许操作：

- 只读当前代码和 `artifacts/sim/hover/20260622T054045Z`。
- 离线计算候选映射：
  - old 成功合同 `x,-y`
  - current helper 合同 `y,x`
  - `x,y`
  - `x,-y`
  - `-x,y`
  - `y,-x`
  - `-y,x`
  - yaw `pi/2-yaw_enu` 与 `-yaw_enu`
- 读取 `navlab/real/companion/nodes/external_nav.py` 和 MAVLink sender status/log。
- 必要时查 ArduPilot 官方 EKF ExternalNav / MAVLink ODOMETRY frame 文档。

禁止：

- 不直接改 ExternalNav mapping。
- 不改 hover/gazebo drift gate。
- 不改 selector gate/cap。
- 不接 Gazebo truth。
- 不跑 hover。

必须产出：

- 表格：`/external_nav/odom`、MAVLink sender status、`/navlab/fcu/local_position_pose`、Gazebo raw 的 final vector。
- 表格：候选 mapping 下 ExternalNav 与 FCU/Gazebo 的 cosine/scale。
- 明确判断：
  - 是 ROS->MAVLink x/y/sign/yaw 合同错；
  - 还是 EKF 没充分融合 ExternalNav；
  - 还是 selector output frame 与 ExternalNav bridge frame 合同错。

验收：

- 只能在 D10 证据指向单一 mapping 或单一 fusion 参数问题后，才允许下一 TODO 改代码。

执行记录：

- 2026-06-22T08:15Z 开始执行 TODO D10。
- 官方依据：
  - ArduPilot Non-GPS Position Estimation 文档说明，ExternalNav 可通过 MAVLink `ODOMETRY` 给 EKF 提供位置/速度估计，且 `ODOMETRY` 是 preferred method，消息频率应不低于 4Hz。
  - ArduPilot EKF Source Selection 文档说明，`EK3_SRC*_POSXY/VELXY/YAW` 可选择 ExternalNav 作为来源。
- 本轮只做离线诊断，不改代码、不跑 hover。

#### D10.1 本轮执行前记录：先写文档，再做诊断

状态：已完成

目的：先把本轮动作钉死，避免继续“先改再解释”。本轮只定位 `/external_nav/odom -> MAVLink ODOMETRY -> FCU LOCAL_POSITION_NED` 之间的真实差距，不做任何 runtime 修复。

本轮允许操作：

- 读取 `navlab/real/companion/nodes/external_nav.py`，只提取 ROS ENU/FLU 到 MAVLink LOCAL_FRD/BODY_FRD 的 position、yaw、twist、covariance/status 映射。
- 读取 `artifacts/sim/hover/20260622T054045Z/summary.json`、rosbag metadata、runtime logs、status JSON；只做离线计算。
- 查官方文档并把链接写回本文档，只使用 ArduPilot / MAVLink / Cartographer 这类官方或主文档作为依据。
- 使用一次性 `/tmp` 脚本做候选映射计算；不把临时脚本纳入 repo。

本轮禁止操作：

- 不改 `external_nav.py`。
- 不改 hover mission、landing、height gate、Gazebo drift gate。
- 不改 selector/correction cap/gate。
- 不跑 hover。
- 不用 Gazebo truth、official map、fixed prior 做 runtime 输入。

本轮必须回答的问题：

1. 当前代码实际发给 MAVLink 的 position/yaw/twist 合同是什么。
2. 最新 F6 artifact 中 `/external_nav/odom`、FCU local、Gazebo body 三个 final vector 是否能由某个 x/y/sign 映射解释。
3. 如果 mapping 解释不了，是否是 EKF source/covariance/quality 融合导致 ExternalNav 被削弱或拒绝。

本轮产物：

- 本文档 D10.1 结果表。
- 若证据足够单一，再新增下一项 TODO；否则只列出还缺的证据，不进入代码修改。

执行结果：

- 2026-06-22T09:02Z 按 D10.1 执行，只读代码和 F6 artifact，没有改 runtime 代码，没有跑 hover。
- 官方依据：
  - ArduPilot Non-GPS Position Estimation：ExternalNav 可通过 MAVLink `ODOMETRY` 输入 EKF，`ODOMETRY` 是 preferred method，要求 4Hz 或更高；示例参数包含 `EK3_SRC1_POSXY=6`、`EK3_SRC1_VELXY=6 or 0`、`EK3_SRC1_YAW=6 or 1`。链接：https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
  - ArduPilot EKF Source Selection：`EK3_SRCx_POSXY=6` 是 ExternalNAV XY Position；若融合速度，必须保证速度测量在同一 reference frame / coordinate system。链接：https://ardupilot.org/copter/docs/common-ekf-sources.html
  - MAVLink Common：`MAV_FRAME_LOCAL_FRD=20` 是 earth-fixed local tangent frame，x Forward、y Right、z Down；`MAV_FRAME_BODY_FRD=12` 是 body frame，x Forward、y Right、z Down。链接：https://mavlink.io/en/messages/common.html

当前代码实际 MAVLink 合同：

| 项 | 当前值 | 证据 |
| --- | --- | --- |
| pose frame | `MAV_FRAME_LOCAL_FRD` / id `20` | `external_nav.py` 常量和 tlog `ODOMETRY.frame_id=20` |
| twist frame | `MAV_FRAME_BODY_FRD` / id `12` | `external_nav.py` 常量和 tlog `ODOMETRY.child_frame_id=12` |
| position | `x=odom.pose.y`、`y=odom.pose.x`、`z=-odom.pose.z` | `ros_enu_position_to_mavlink_local_frd()` 和 status `mapping.field_map` |
| yaw | `yaw_frd=pi/2-yaw_enu`，再 initial-align 到 FCU yaw | `_odometry_quaternion()` |
| velocity | `vx=odom.twist.x`、`vy=-odom.twist.y`、`vz=-odom.twist.z` | `odometry_send()` |
| FCU EKF source | `EK3_SRC1_POSXY=6`、`EK3_SRC1_VELXY=0`、`EK3_SRC1_YAW=1` | `mav.parm` |

F6 hover window 核心向量：

| 源 | frame | final delta x/y | magnitude | 结论 |
| --- | --- | --- | ---: | --- |
| `/external_nav/odom` | `external_nav -> base_link` native XY | `(-0.5065, 0.3588)` | `0.6207m` | bridge/selector 已经输出明显水平位移 |
| MAVLink `ODOMETRY` sent | `LOCAL_FRD -> BODY_FRD` | `(-0.0306, -0.3618)` | `0.3631m` | sender 实际发给 FCU 的方向已和 `/external_nav/odom` 不一致 |
| FCU `LOCAL_POSITION_NED` | FCU local | `(-0.0435, -0.2412)` | `0.2451m` | FCU local 与 MAVLink `ODOMETRY` 方向几乎一致 |
| Gazebo raw review-only | `odom -> base_link` | `(-0.2359, 0.8244)` | `0.8575m` | 真实 Gazebo body 与 FCU local 方向不一致 |
| Gazebo projected review-only | SDF `gazeboXYZToNED` projection | `(0.8244, 0.2359)` | `0.8575m` | 只作坐标诊断，不作 runtime 输入 |

候选映射离线结果：

| 候选 mapping，从 `/external_nav/odom` 到 MAVLink local XY | vector | cos(Gazebo raw) | cos(Gazebo projected) | cos(FCU local) | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| 当前 `y,x` | `(0.3588, -0.5065)` | `-0.9435` | `0.3314` | `0.7005` | 和 Gazebo raw 反向，和 projected 也差；不应继续保留 |
| yaw=90deg 的 LOCAL_FRD 投影 `y,-x` | `(0.3588, 0.5065)` | `0.6255` | `0.7802` | `-0.9056` | 符合 LOCAL_FRD x forward/y right 的方向定义，比当前更合理 |
| native `x,y` | `(-0.5065, 0.3588)` | `0.7802` | `-0.6255` | `-0.4242` | 能解释 Gazebo raw，但不是 MAVLink LOCAL_FRD 合同 |
| old success `x,-y` | `(-0.5065, -0.3588)` | `-0.3314` | `-0.9435` | `0.7137` | 旧 truth odom 的输入 frame 不等于当前 SLAM frame，不能直接照搬 |

关键判断：

- 这轮证据不支持“EKF 完全没融合 ExternalNav”。相反，tlog 内 MAVLink `ODOMETRY` hover-window vector 与 FCU `LOCAL_POSITION_NED` vector 的 cosine 是 `0.9956`，说明 FCU/EKF 基本沿着 sender 发出的方向走。
- 真正断点在 sender 前后：`/external_nav/odom` 的方向和 Gazebo raw 有一定同向证据，但当前 `external_nav.py` 把 position 发成 `x=odom.y, y=odom.x`，没有按 `LOCAL_FRD` 的 y-right 语义对横向轴取反，也没有用初始 yaw 做通用 ENU->FRD 投影。
- `EK3_SRC1_VELXY=0`、`EK3_SRC1_YAW=1` 表示当前主要使用 ExternalNav 的 XY position，不使用 ExternalNav velocity/yaw 作为主来源；所以先修 position frame 合同，而不是先调 twist/yaw/covariance。

### TODO E10：修正 MAVLink ExternalNav position frame 合同，先单测，不跑 hover

状态：已完成

目标：把 `external_nav.py` 的 ROS ENU/map XY 到 MAVLink `MAV_FRAME_LOCAL_FRD` position 转换从硬编码 `y,x` 收敛为“按初始 yaw 的 LOCAL_FRD 投影”。在当前 hover 初始 yaw 约 `+pi/2` 的情况下，投影应等价于 `x=odom.y`、`y=-odom.x`，不能继续把 right 轴发成 `+odom.x`。

允许操作：

- 修改 `navlab/real/companion/nodes/external_nav.py` 中 position helper 和状态 mapping 文本。
- 修改/新增 `navlab/tests/companion/test_external_nav_sender.py` 中的纯函数单测。
- 如当前接口需要保持兼容，可以先让 helper 接收 `yaw_reference_rad`，默认仍使用当前 hover 的 `pi/2` 合同；不得引入 Gazebo truth 输入。
- 只跑相关单测，不跑 hover。

禁止操作：

- 不改 hover mission / landing / height gate。
- 不改 Gazebo drift gate。
- 不改 selector/correction gate/cap。
- 不改 Foxglove display TF 或全局 runtime TF。
- 不接 Gazebo truth、official map、fixed prior。

验收：

- 单测证明：
  - yaw reference `0` 时，ENU `(x,y,z)` 投影到 LOCAL_FRD 是 `(x,-y,-z)` 或项目选择的明确合同，并在测试名里说明；
  - yaw reference `pi/2` 时，ENU `(x,y,z)` 投影为 `(y,-x,-z)`；
  - 当前错误的 `y,x` 不再是默认 mapping。
- 代码状态 mapping 文本必须和真实 helper 一致。
- 完成后在本 TODO 写入实际改动文件和测试结果，再进入下一 TODO 跑短验证/hover。

执行记录：

- 2026-06-22T09:11Z 开始执行 TODO E10。
- 本轮只改 `external_nav.py` 的 position frame helper/状态文本，以及对应 `test_external_nav_sender.py` 单测。
- 本轮不跑 hover，不改任何 gate/correction/mission。
- 2026-06-22T09:18Z 完成 E10 最小修改：
  - `navlab/real/companion/nodes/external_nav.py`
    - `ros_enu_position_to_mavlink_local_frd()` 从错误的 `x=odom.y, y=odom.x, z=-odom.z` 改为 `x=odom.y, y=-odom.x, z=-odom.z`。
    - `mapping.field_map.y` 同步改成 `-odom.pose.pose.position.x`，避免 status 继续撒谎。
  - `navlab/tests/companion/test_external_nav_sender.py`
    - 更新 position mapping 单测期望。
    - 新增 `test_ros_enu_position_no_longer_uses_the_bad_y_x_mapping()`，明确禁止旧 `y,x` 默认合同回归。
- 本轮没有引入 `yaw_reference_rad` 通用接口；原因是 D10 证据已经指向当前 hover/Gazebo/SDF 合同的 y 符号错误，先做最小修复，避免扩大改动面。通用 yaw-aware 投影如果还需要，必须另起 TODO，不夹在本次修改里。
- 验证：
  - `python3 -m py_compile navlab/real/companion/nodes/external_nav.py`：通过。
  - `uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_sender.py -q`：`11 passed in 0.04s`。
- 未验证：
  - 未跑 hover；本 TODO 明确禁止直接跑 hover。
  - 未验证 tlog 中新 `ODOMETRY` 是否变成 `y,-x`；下一 TODO 只做短验证/证据，不直接宣布成功。

### TODO E11：短验证新 MAVLink ODOMETRY 方向，不先宣称 hover 成功

状态：已完成

目标：在不改 gate、不改 mission 的前提下，跑一次最短可接受验证，确认新 sender 发出的 MAVLink `ODOMETRY` hover-window vector 不再是旧 `y,x` 方向，并检查 FCU `LOCAL_POSITION_NED` 是否跟随新方向。

允许操作：

- 跑短 hover 或项目已有 hover 前验证，只为采集 artifact/tlog。
- 读取新 artifact 的：
  - `/external_nav/odom`
  - MAVLink `ODOMETRY`
  - FCU `LOCAL_POSITION_NED`
  - `/gazebo/model/odometry` review-only
  - `/mavlink_external_nav/status`
- 写本文档结果。

禁止操作：

- 不改 hover 高度门槛。
- 不改 landing 阈值。
- 不改 Gazebo drift gate。
- 不改 selector/correction gate/cap。
- 不用 Gazebo truth、official map、fixed prior 做 runtime 输入。

验收：

- 新 artifact 中 MAVLink `ODOMETRY` field/status 必须显示 `x=odom.y`、`y=-odom.x`。
- 新 hover-window `ODOMETRY` vector 必须不再和旧错误 `y,x` 同向。
- 只有当顶层 `summary.ok=true` 且 Gazebo review-only hover drift `<=0.10m`，才能说 hover task 成功；否则只报告新的失败断点。

执行记录：

- 2026-06-22T09:24Z 开始执行 E11。
- 本轮命令计划：`cd orchestration/sim && env -u NAVLAB_SIM_OVERLAY_SOURCE_MODE NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover`。
- 目的：只采集新 artifact，验证 E10 的 MAVLink `ODOMETRY` direction 是否真实进入 runtime；不在本轮命令前追加任何新代码修改。
- 2026-06-22T05:55:24Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T055524Z`
- 结果：失败，`summary.ok=false`、`status=TASK_STATUS_BLOCKED`。
- E10 runtime 生效证据：
  - `mission_summary.mavlink_external_nav_status.mapping.field_map.x="odom.pose.pose.position.y"`
  - `mission_summary.mavlink_external_nav_status.mapping.field_map.y="-odom.pose.pose.position.x"`
  - hover window 中 `/external_nav/odom` final vector 为 `(-0.0022, 0.0430)`。
  - MAVLink tlog `ODOMETRY` hover-window delta 为 `(0.0430, 0.0023)`，与 `y,-x` 期望方向 cosine `1.0000`。
  - 旧错误 `y,x` 在这轮因为 `/external_nav/odom.x` 接近 0，方向上也近似；但 status 文本和 helper 已证明 `y=-x` 真实进入 runtime。
- 新失败断点：
  - Gazebo review-only hover drift 变成 `3.2679m`，远超 `0.10m`。
  - FCU `LOCAL_POSITION_NED` hover-window native delta 为 `(0.1977, -0.0706)`，和 Gazebo raw direction 基本同向，但 magnitude 只有 Gazebo 的约 `6.4%`。
  - `/external_nav/odom` magnitude 只有 `0.0431m`，说明 FCU/EKF 没有收到足够大的横向 ExternalNav 位移证据。
  - `scan_reference_hover_drift` 同窗估计到约 `1.5516m`，`scan_reference_runtime_drift.horizontal_drift_m=1.5616m`，说明 `/scan` 里确实有横漂证据。
  - 但 `/external_nav/odom_candidate` 最终仍退回近零，`external_nav_source_selector.source=slam_passthrough`，最新 blocker 包含 `scan_reference_quality_not_good`、`scan_reference_inlier_ratio_low`、`scan_reference_eligibility_not_allowed`、`scan_reference_xy_axes_not_allowed`。
  - `scan_reference_correction` 也显示 hover window 中只 applied 7/24，最终 `state=passthrough`，而实际 ExternalNav bridge 输入是 `/external_nav/odom_candidate`，不是 `/slam/odom_corrected`。
- 本轮判断：
  - E10 解决了 MAVLink position sign 的一部分问题，但没有让 Gazebo/EKF 对齐。
  - 现在的主要断点不是 hover mission，也不是 landing，也不是 EKF 完全不融合，而是：`/scan` 已经观测到横漂，但 source selector / correction 在 hover window 内反复退回 near-zero SLAM passthrough，导致 ExternalNav 给 FCU 的位移幅值远小于 Gazebo body。
  - 下一步不能调高度/landing/gazebo gate，也不能加大 correction cap；必须修 selector 对“短暂质量波动”的处理，避免 hover_hold 内从 scan-reference 大位移突然 snap 回 near-zero SLAM。

### TODO E12：修 source selector 的 hover_hold 稳态输出，禁止从有效 scan-reference 瞬间跳回 near-zero SLAM

状态：已完成

目标：在不接 Gazebo truth、不放宽 quality gate、不扩大 correction cap 的前提下，让 `/external_nav/odom_candidate` 在 hover_hold 内使用“最近一次通过 gate 的 scan-reference candidate”做短时间 bounded hold。这样 scan-reference 短暂 quality/inlier 波动时不会把 ExternalNav 瞬间拉回 near-zero SLAM，避免 EKF/Gazebo 进一步分离。

依据：

- ArduPilot ExternalNav 要求持续、稳定地向 EKF 提供位置估计；MAVLink `ODOMETRY` 也提供 `reset_counter` 用于估计源 discontinuity。当前 selector 在 hover_hold 内 scan/SLAM 之间直接切换，会制造未声明的 position discontinuity。
- E11 证据显示 `/scan` 同窗有 `~1.56m` 横漂证据，但 ExternalNav 最终只有 `0.043m`；断点在 selector/correction 没有稳定把 scan-reference 证据送进 `/external_nav/odom_candidate`。

允许操作：

- 修改 `navlab/sim/companion/nodes/external_nav_source_selector.py`：
  - 增加纯函数状态机或小状态对象，记录 last good scan candidate。
  - 仅在 `hover_settle/hover_hold`、scan candidate 近期通过原 gate、未超过短 TTL、与当前 slam/candidate jump 不超过既有 cap 时，允许 `scan_reference_hold` 输出。
  - 一旦 TTL 超时、非 hover phase、scan status stale、truth/map 输入标记出现，必须 fail-closed 到 SLAM passthrough。
  - status 必须显式标出 `source=scan_reference_hold`、`hold_age_ms`、`hold_reason`、`last_good_scan_age_ms`。
- 修改 `navlab/tests/companion/test_external_nav_source_selector.py`：
  - 新增“有效 scan 后短暂 quality_bad 时 hold last scan”单测。
  - 新增“TTL 超时后回到 slam_passthrough”单测。
  - 新增“非 hover phase 不允许 hold”单测。
- 只跑相关单测，不跑 hover。

禁止操作：

- 不降低现有 scan quality / inlier / residual 阈值。
- 不扩大 correction cap。
- 不改 hover mission、landing、height gate、Gazebo drift gate。
- 不接 Gazebo truth、official map、fixed prior。

验收：

- 单测覆盖 `scan_reference`、`scan_reference_hold`、`slam_passthrough` 三条路径。
- status 文本能解释为什么 hold 或 fail-closed。
- 完成后必须新增下一 TODO 再跑 hover 验证，不能在 E12 单测通过后直接宣称成功。

执行记录：

- 2026-06-22T09:45Z 开始执行 E12。
- 本轮只改 source selector 和对应单测；不跑 hover，不改 mission/gate/correction cap。
- 2026-06-22T09:54Z 完成 E12：
  - `navlab/sim/companion/nodes/external_nav_source_selector.py`
    - `SourceDecision` 新增 `hold_age_ms`、`hold_reason`。
    - `select_external_nav_source()` 新增 last-good scan-reference hold 逻辑。
    - hold 只在 `hover_settle/hover_hold`、last good scan 未超过 `max_hold_age_ms=750ms`、当前 scan status 未 stale、未标记 truth/map、last-good 与 slam jump 不超过 `max_hold_jump_m=1.25m` 时启用。
    - runtime node 记录 `_last_good_scan_candidate`，只有 `source=scan_reference` 真正通过原 gate 时更新；`scan_reference_hold` 不刷新 last-good，避免无限延长。
    - status 新增 `hold_age_ms`、`hold_reason`。
  - `navlab/tests/companion/test_external_nav_source_selector.py`
    - 新增 quality 短暂变坏时 `scan_reference_hold` 单测。
    - 新增 TTL 超时 fail-closed 单测。
    - 新增非 hover phase 禁止 hold 单测。
- 验证：
  - `python3 -m py_compile navlab/sim/companion/nodes/external_nav_source_selector.py`：通过。
  - `uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_source_selector.py -q`：`8 passed in 0.03s`。
  - `uv run --project navlab --directory /home/nn/workspace/3588/world-model pytest navlab/tests/companion/test_external_nav_source_selector.py navlab/tests/companion/test_external_nav_sender.py -q`：`19 passed in 0.03s`。
- 未验证：
  - 未跑 hover；E12 验收只允许单测。下一步必须新增 TODO 再跑真实 artifact。

### TODO E13：验证 E12 selector hold 是否让 ExternalNav 幅值跟上 scan-reference

状态：执行中

目标：跑一轮 hover，检查 `/external_nav/odom_candidate` 和 `/external_nav/odom` 是否不再在 hover_hold 内频繁 snap 回 near-zero SLAM；同时检查 Gazebo/EKF 偏移是否缩小。

允许操作：

- 跑真实 hover。
- 解析新 artifact 的：
  - `summary.ok/status/blockers`
  - `external_nav_source_selector.source`
  - `hold_age_ms/hold_reason`
  - `/external_nav/odom_candidate`、`/external_nav/odom`
  - `scan_reference_runtime_drift`
  - MAVLink `ODOMETRY`
  - FCU `LOCAL_POSITION_NED`
  - Gazebo review-only drift
- 写本文档结果。

禁止操作：

- 不改 hover mission / landing / height gate。
- 不改 Gazebo drift gate。
- 不改 selector 阈值、scan quality 阈值、correction cap。
- 不接 Gazebo truth、official map、fixed prior。

验收：

- 如果 `summary.ok=true` 且 Gazebo review-only drift `<=0.10m`，才可说 hover 成功。
- 如果仍失败，必须明确是：
  - selector hold 仍没进 runtime；
  - selector hold 生效但 ExternalNav 幅值仍不够；
  - ExternalNav 幅值够但 EKF/FCU 不跟；
  - 或 Gazebo body 与 FCU 控制实体/坐标仍不一致。

执行记录：

- 2026-06-22T09:58Z 开始执行 E13。
- 本轮命令计划：`cd orchestration/sim && env -u NAVLAB_SIM_OVERLAY_SOURCE_MODE NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover`。
- 本轮只验证 E12 runtime 效果；不在命令前追加任何新代码修改。
- 2026-06-22T06:05:23Z run 完成，artifact：
  - `artifacts/sim/hover/20260622T060523Z`
- 结果：失败，`summary.ok=false`、`status=TASK_STATUS_ERROR`。
- 该 run 不能作为最终 hover 成功/失败的完整结论，因为 required probe `imu_probe` 失败：
  - `probe_failed:imu_probe:rc=20`
  - `topic_sample_missing:/imu`
- 但该 run 仍提供了 selector/ExternalNav 的有效证据：
  - `external_nav_source_selector.status` 已包含 E12 新字段 `hold_age_ms`、`hold_reason`，说明新 selector 代码进入 runtime。
  - 最新 status 仍是 `source=slam_passthrough`、`hold_age_ms=null`、`hold_reason=null`，没有看到 `scan_reference_hold`。
  - `/external_nav/odom` hover-window magnitude 只有 `0.0683m`。
  - `scan_reference_runtime_drift.horizontal_drift_m=0.9335m`，说明 scan-reference 仍观测到接近 1m 横漂。
  - Gazebo review-only drift 仍为 `2.8421m`。
  - FCU hover drift 只有 `0.0346m`，mission 局部 hover 看起来稳定，但 Gazebo body 实际漂走。
- E13 结论：
  - E12 代码进了 runtime，但从 summary/latest status 看，`scan_reference_hold` 没有真正成为 `/external_nav/odom_candidate` 主输出。
  - 下一步不能再猜；必须离线解析 rosbag 中 `/external_nav/source_selector/status` 的 hover window source 序列，确认是“从未进入 scan_reference”，还是“进入过但 summary/latest 看不到”。
  - 如果 rosbag 证明从未进入，下一步要修 selector 对 scan status 字段/phase 的消费；如果进过但输出幅值仍小，再修 output continuity。

### TODO D11：离线解析 E13 rosbag 的 selector source 序列

状态：执行中

目标：只读 `artifacts/sim/hover/20260622T060523Z` 的 rosbag，回答 E12 的 `scan_reference_hold` 是否在 hover window 内出现过，以及 `/external_nav/odom_candidate` 是否真正跟随 scan-reference。

允许操作：

- 解压 rosbag `.mcap.zstd` 到 `/tmp`。
- 用 ROS2/rosbag2 只读解析：
  - `/external_nav/source_selector/status`
  - `/external_nav/odom_candidate`
  - `/external_nav/odom`
  - `/navlab/scan_reference_drift/status`
  - `/navlab/scan_reference_drift/odom`
- 写本文档结果。

禁止操作：

- 不改代码。
- 不跑 hover。
- 不改任何阈值。

必须回答：

1. hover window 内 `source=scan_reference`、`source=scan_reference_hold`、`source=slam_passthrough` 各出现多少次。
2. 如果没有 `scan_reference/hold`，当时的 blocker 是哪些。
3. `/navlab/scan_reference_drift/odom` 的同窗 vector 与 `/external_nav/odom_candidate` 的同窗 vector 差距是多少。
4. 下一步应修 selector 逻辑、runtime script，还是 scan estimator/status。

执行记录：

- 2026-06-22T10:18Z 开始执行 D11。
- 本轮只解压/解析 rosbag，不改代码、不跑 hover。

### TODO D12：补齐 Gazebo / Cartographer TF evidence，锁定 raw drift 断点

状态：执行中

目标：下一轮 hover 只补证据采集和离线对齐，不改 hover 控制、mission、landing、drift gate、correction 或 selector 阈值。必须判断 drift 断点到底在 Gazebo evidence source、Cartographer 原始 TF、还是 navlab adapter 输出。

允许操作：

- 修改 rosbag/review-only topic profile，把以下 topic 加入 hover rosbag：
  - `/navlab/slam/tf`：Cartographer `/tf` 被 remap 后的真实动态 TF 输出。
  - `/gazebo/tf`：Gazebo model pose bridge 输出，用于和 `/gazebo/model/odometry` 互证。
  - `/gazebo/tf_static`：Gazebo static pose bridge 输出，用于补 frame/link 关系。
- 增加/更新只验证 topic profile 的单测。
- 跑针对性单测，不跑完整 hover 也不能宣称问题解决。

禁止操作：

- 不改 hover mission / landing / height gate。
- 不改 Gazebo drift gate。
- 不改 Cartographer 参数、scan estimator、source selector、correction cap。
- 不接 Gazebo truth、official map、fixed prior 到 runtime 控制链。
- 不用 Foxglove display TF 假对齐。

必须回答：

1. `/navlab/slam/tf` hover_hold 内 `map -> base_link` 是否 near-zero，还是接近 `/cartographer/odometry_input` 的约 1m。
2. `/gazebo/tf` 或 Gazebo model pose 是否与 `/gazebo/model/odometry` 同窗 drift 一致。
3. 如果 `/navlab/slam/tf` 有漂移但 `/slam/odom` near-zero，断点在 `navlab_cartographer_adapter`。
4. 如果 `/navlab/slam/tf` 本身 near-zero，但 scan-derived odometry input 有约 1m，断点在 Cartographer 输入使用/配置/scan matching。
5. 如果 `/gazebo/tf` 与 `/gazebo/model/odometry` 不一致，先修 review-only evidence source，不改控制。

验收：

- hover rosbag review profile 明确包含 `/navlab/slam/tf`、`/gazebo/tf`、`/gazebo/tf_static`。
- 单测覆盖这些 topic，不允许以后缺录又误判。
- 文档记录本轮只改 evidence collection，没有改控制路径。

执行记录：

- 2026-06-22T14:30Z 开始执行 D12。
- 本轮第一步只写 TODO；随后只改 rosbag/review-only evidence topic 配置和对应测试。
- 2026-06-22T15:00Z 完成 D12 代码侧最小修改：
  - `orchestration/sim/internal/tasks/helpers/slam.go`
    - 新增 `CartographerTFTopic=/navlab/slam/tf`、`DiagnosticGazeboTFTopic=/gazebo/tf`、`DiagnosticGazeboTFStaticTopic=/gazebo/tf_static` 常量，避免以后手写漏 topic。
  - `orchestration/sim/internal/tasks/helpers/execution_plan.go`
    - hover rosbag review-only topic 追加 `/navlab/slam/tf`、`/gazebo/tf`、`/gazebo/tf_static`。
    - 这些 topic 没有加入 RequiredTopics，避免把 review-only Gazebo/TF 证据变成控制或验收输入。
  - `orchestration/sim/internal/tasks/helpers/execution_plan_test.go`
    - hover plan 单测现在要求 hover rosbag 包含 `/navlab/slam/tf`、`/gazebo/tf`、`/gazebo/tf_static`。
    - 单测同时确认这些 topic 是 review-only，不在 required topics 内。
- 验证：
  - `go test ./internal/tasks/helpers -run 'TestBuildExecutionPlanHoverKeepsMatureSLAMRosbagTopics|TestMatureReviewTopicsExcludeHeavyDiagnosticTopics'`：通过。
  - `go test ./internal/tasks/helpers`：通过。
  - `go run ./cmd/navlab-sim run hover --dry-run --duration-sec 30`：通过，生成 dry-run artifact `artifacts/sim/hover/20260622T070052Z`。
  - `artifacts/sim/hover/20260622T070052Z/task_plan.json` 和 `profiles/hover_rosbag.txt` 已确认包含：
    - `/navlab/slam/tf`
    - `/gazebo/tf`
    - `/gazebo/tf_static`
    - `/gazebo/model/odometry`
    - `/cartographer/odometry_input`
- 未验证：
  - 还没有跑真实 hover；本轮只保证下一次真实 hover 会录到能定位断点的 TF/Gazebo evidence。
- 2026-06-22T15:05Z 开始 D12 真实 hover evidence run：
  - 命令：`cd orchestration/sim && env -u NAVLAB_SIM_OVERLAY_SOURCE_MODE NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover`
  - 目标：只采集 `/navlab/slam/tf`、`/gazebo/tf`、`/gazebo/tf_static` 与既有 `/gazebo/model/odometry`、`/cartographer/odometry_input` 的同窗证据。
  - 本 run 仍不改控制、不改阈值、不宣称 hover 成功。
- 2026-06-22T15:12Z D12 真实 hover evidence run 完成，artifact：
  - `artifacts/sim/hover/20260622T070235Z`
  - 顶层结果：`summary.ok=false`、`status=TASK_STATUS_BLOCKED`、`exitCode=20`。
  - 该 run 不能作为 hover 成功；但新增 TF/Gazebo evidence 已成功录到。
- 新增 topic 录制结果：
  - `/navlab/slam/tf`：`13037` 条，有效。
  - `/gazebo/tf`：`3111` 条，有效。
  - `/gazebo/tf_static`：`0` 条；本轮未提供静态 Gazebo TF，但不影响动态 pose/odom 互证。
- hover_hold 窗口：`1782111801.554 .. 1782111819.553`，约 `17.999s`。
- 同窗 drift 证据：
  - `/gazebo/model/odometry` `odom->base_link`：`9.7066m`。
  - `/gazebo/tf` `odom->base_link`：`9.7066m`。
  - 结论：本轮 Gazebo odometry 与 Gazebo dynamic TF 完全一致，不能再把 `/gazebo/model/odometry` 先当作显示误报。
  - `/cartographer/odometry_input`：`0.9856m`。
  - `/navlab/scan_reference_drift/odom`：`0.9855m`。
  - `/navlab/slam/tf` `map->base_link`：`0.0704m`，max from first `0.1329m`。
  - `/slam/odom`：`0.0704m`。
  - `/slam/odom_corrected`：`0.0704m`。
  - `/external_nav/odom_candidate`：`0.5731m`。
  - `/external_nav/odom`：`0.5736m`。
  - `/navlab/fcu/local_position_pose`：`0.7726m`。
- selector 同窗 source 计数：
  - `slam_passthrough`: `24`
  - `scan_reference`: `8`
  - `scan_reference_hold`: `4`
- D12 结论：
  - Gazebo dynamic TF 与 Gazebo odometry 一致，Gazebo evidence source 当前成立。
  - Cartographer 原始 `/navlab/slam/tf` 本身 near-zero，和 `/slam/odom` 一致；断点不在 `navlab_cartographer_adapter` 把有漂移 TF 压成 near-zero。
  - 断点收敛到 Cartographer 本体/配置/输入使用/scan matching：scan-derived odometry input 约 `0.986m`，但 Cartographer 输出 TF 只有 `0.070m`。
  - 同时 Gazebo body 本轮漂到 `9.7m`，已经不是厘米/几十厘米级问题；下一步不能继续调 selector/correction，必须查为什么 Cartographer 在 `use_odometry=true` 时没有让 scan-derived odom/input 进入最终 pose。

### TODO D13：Cartographer odometry input 已被消费但未进入最终 pose 的根因诊断

状态：执行中

目标：基于 D12 证据，只查/修 Cartographer 输入使用、frame 配置、scan matching 权重或 adapter 对 Cartographer 原始输出的读取方式。不得继续调 selector/correction 来掩盖 Cartographer near-zero 输出。

官方依据：

- Cartographer ROS 官方配置文档说明：`use_odometry=true` 会订阅 `nav_msgs/Odometry` 的 `odom` topic，odometry 必须提供且会被纳入 SLAM。
- Cartographer ROS `SensorBridge::ToOdometryData()` 源码显示：odometry pose 会按 `child_frame_id` 查到 tracking frame 后传入 `trajectory_builder_->AddSensorData()`。
- 本项目 D12 log 证明 Cartographer 实际收到 odom：`slam_backend.runtime.log` 中有 `odom rate: 6.98 Hz`。

必须先回答：

1. `/cartographer/odometry_input` 的 `child_frame_id=base_link` 到 `tracking_frame=imu_link` 的 TF 是否在 Cartographer 时间戳处持续可查。
2. Cartographer 是否只是把 odometry 当 extrapolator/prior，而非最终 pose 强约束；如果是，当前 `ceres_scan_matcher.translation_weight=1`、`real_time_correlative_scan_matcher.linear_search_window=0.50` 是否导致 scan matching 在低可观测墙面中压掉 odom drift。
3. 是否应该做一轮 replay-only Cartographer config ablation：
   - 不接 FCU、不起飞、不改 mission。
   - 使用同一个 D12 rosbag replay `/scan`、`/navlab/slam/imu`、`/cartographer/odometry_input`。
   - 对比不同 Cartographer 参数输出 `/navlab/slam/tf` 是否从 `0.070m` 接近 `0.986m`。
4. 如果 ablation 证明参数可修，再把参数变更写入 hover lua，并跑 SLAM-only，再跑 hover。

禁止操作：

- 不改 hover mission / landing / drift gate。
- 不改 selector/correction 来绕过 Cartographer near-zero。
- 不接 Gazebo truth、official map、fixed XY prior。
- 不把 `/gazebo/tf` 或 `/gazebo/model/odometry` 接入 runtime 控制。

验收：

- 先产出 replay-only 证据：同一输入 bag 下 Cartographer 输出是否能跟随 scan-derived odometry。
- 只有 replay-only 证据成立，才允许改 runtime Cartographer 参数。

执行记录：

- 2026-06-22T15:25Z 开始执行 D13.0/D13.1。
- 本阶段只做 Cartographer 输入链验证和 replay-only baseline；不改 hover mission、landing、drift gate、selector/correction，也不接 Gazebo truth。
- 2026-06-22T15:32Z 完成 D13.0 输入链验证：
  - `/cartographer/odometry_input` 共 `627` 条，所有消息均为 `frame_id=odom`、`child_frame_id=base_link`。
  - odom input stamp 范围：`5.490s .. 95.127s`。
  - `/tf_static` 存在 `base_link -> imu_link`、`base_link -> base_scan`、`base_link -> rangefinder_down_link`。
  - `/tf` 动态也持续发布 `base_link -> imu_link` 和 `base_link -> base_scan`，各 `1244` 条。
  - `slam_backend.runtime.log` 多次显示 Cartographer 输入速率：`odom rate: 6.98 Hz`、`scan rate: ~6.98 Hz`、`imu rate: ~990 Hz`。
- D13.0 结论：
  - “odom 输入没被 Cartographer 收到”已排除。
  - “odom child frame 到 tracking frame 缺 TF”已排除。
  - 下一步进入 D13.1 replay-only baseline：确认离线重放是否复现 `odom input ~0.986m -> Cartographer output ~0.070m`。
- 2026-06-22T15:45Z D13.1 replay-only baseline 尝试记录：
  - 第一次直接 `ros2 bag play` 失败：原始 bag metadata 仍带 `compression_format=zstd`，解压后的只读目录导致 rosbag2 试图写解压文件失败。
  - 修复为去压缩 metadata 后，直接 `ros2 bag play` 又失败：Cartographer 报 `Non-sorted data added to queue: '(0, imu)'`。
  - 量化原始 D12 bag：单 topic header stamp 均单调，但 `/scan`、`/cartographer/odometry_input`、`/navlab/slam/imu` 混合后的全局 header stamp inversion 有 `1094` 次；因此原始 bag 不能直接作为 Cartographer replay 输入。
  - 改用按 header stamp 排序的 replay publisher，只发布 `/scan`、`/navlab/slam/imu`、`/cartographer/odometry_input`，并发布 `/clock`；不启动 Gazebo、FCU、hover mission。
  - baseline2 输出目录：`/tmp/navlab_d13_replay_baseline2/replay_out`。
  - replay publisher 发布计数：`/scan=627`、`/navlab/slam/imu=62244`、`/cartographer/odometry_input=627`。
  - Cartographer replay log 显示输入速率正常：`odom rate=6.98 Hz`、`scan rate=6.98 Hz`、`imu rate≈980 Hz`。
- D13.1 replay-only baseline 结果：
  - 全程 replay：`/cartographer/odometry_input` drift `3.7051m`，`/navlab/slam/tf` drift `8.4033m`，max `19.9634m`。
  - 用 D12 live hover_hold 对应的 header stamp 窗口 `32.410037573 .. 50.308786578` 裁剪：
    - replay `/cartographer/odometry_input`：`0.9856m`。
    - replay `/navlab/slam/tf`：`5.3783m`。
    - replay `/slam/odom`：`5.3783m`。
  - 对比 D12 live 同窗：live `/navlab/slam/tf` 只有 `0.0704m`。
- D13.1 结论：
  - replay-only baseline 没有复现 live 的 near-zero Cartographer 输出；它反而让 Cartographer 明显移动。
  - 因此当前不能进入 D13.2 Cartographer 参数 ablation；否则会在一个与 live 问题不同的 replay 行为上调参。
  - 新断点变为：live runtime 的消息时序/clock/colllation 与 replay-only sorted 输入不一致，导致 Cartographer live hover_hold 输出 near-zero。
  - 下一步必须先做 D13.1b：构造能复现 live near-zero 的 replay harness，或在 live runtime 中直接记录 Cartographer sensor collation/输入时间关系；未复现前禁止改 Cartographer 参数。

### TODO D13.1b：复现 live near-zero 的 Cartographer replay/时序诊断

状态：已完成

目标：解释为什么 live hover_hold 中 Cartographer `/navlab/slam/tf` 只有 `0.0704m`，但按 header stamp 排序的 replay-only baseline 同窗输出 `5.3783m`。必须先复现或定位时序差异，再进入参数 ablation。

允许操作：

- 只改 replay/diagnostic harness，不改 runtime 控制。
- 增加只读解析脚本，比较 live bag 中 `/scan`、`/navlab/slam/imu`、`/cartographer/odometry_input` 的 receive-time 顺序、header-stamp 顺序、topic latency。
- 设计 replay variants：
  - receive-order replay（需要处理 Cartographer non-sorted 问题）。
  - header-stamp sorted replay（已完成，不能复现 near-zero）。
  - 截取 hover_hold 前后一小段 replay。
  - 调整 replay rate/clock 发布策略，验证是否影响 Cartographer 输出。

禁止操作：

- 不做 D13.2 参数 ablation，直到 replay 能复现 live near-zero 或明确证明 live-only 时序问题。
- 不改 hover mission / landing / drift gate / selector / correction。
- 不接 Gazebo truth、official map、fixed prior。

执行记录：

- 2026-06-22T15:55Z 开始执行 D13.1b。
- 本阶段只做 replay/diagnostic harness 与离线解析：比较 live D12 bag 和 D13 sorted replay bag 的 receive-time、header-stamp、latency、Cartographer log/collation 行为。
- 禁止项继续生效：不改 hover mission、landing、drift gate、selector/correction，不做 Cartographer 参数 ablation，不接 Gazebo truth。
- 2026-06-22T07:41Z 继续执行 D13.1b。
- 本轮目标：把前一轮临时 `/tmp/navlab_d13_timing_diagnose.py` 固化为只读诊断 harness，输出可复查的 JSON/文本证据，并继续查 `/scan`、`/cartographer/odometry_input`、`/navlab/slam/imu` 的时间戳来源。
- 本轮允许新增文件：
  - `scripts/diagnostics/hover_timing_diagnose.py`：只读 rosbag/log/artifact 诊断脚本；不进入 runtime 控制链。
- 本轮允许读取：
  - `artifacts/sim/hover/20260622T070235Z`
  - `/tmp/navlab_d12_hover_rosbag`
  - `/tmp/navlab_d13_replay_baseline2/replay_out`
  - scan/IMU/Cartographer launch 与 normalizer 源码。
- 本轮禁止范围：不改 hover mission、landing、Gazebo drift gate、selector/correction、ExternalNav、Cartographer lua 参数；不跑真实 hover；不做 D13.2 参数 ablation。
- 本轮验收：
  - 明确 live receive-window near-zero 是否由 header-stamp lag/不同时间基造成。
  - 明确 sorted replay 没复现 near-zero 的原因是否是 replay 消除了 live 的跨 topic header stamp 乱序/滞后。
  - 明确下一步应该修 timestamp/clock/diagnostic window，还是仍需构造更接近 live 的 replay。
- 2026-06-22T07:50Z 完成 D13.1b harness 固化和第一轮输出：
  - 新增 `scripts/diagnostics/hover_timing_diagnose.py`。
  - 该脚本只读 rosbag/log，输出：
    - input topics 的 receive-time 顺序、header-stamp 顺序、跨 topic header inversion。
    - per-topic receive span、header stamp span、relative skew、`recv-stamp` 粗 latency。
    - hover receive window 与 input header-stamp window 两套裁剪下的 `/cartographer/odometry_input`、`/navlab/slam/tf`、`/slam/odom`、`/gazebo/model/odometry` drift。
    - Cartographer runtime log 中 `Queue waiting for data`、`Non-sorted`、`Dropped`、rate lines。
  - 验证命令：
    - `python3 -m py_compile scripts/diagnostics/hover_timing_diagnose.py`：通过。
    - `scripts/diagnostics/hover_timing_diagnose.py --help`：通过；脚本只有真正读 bag 时才要求 ROS2 Python 环境。
    - Docker 内运行诊断脚本：通过。
  - 输出 artifact：
    - `artifacts/sim/hover/20260622T070235Z/d13_timing_diagnosis.json`
    - `artifacts/sim/hover/20260622T070235Z/d13_timing_diagnosis.txt`
- D13.1b live/replay 时序结果：
  - Live D12 输入：
    - `/scan`：`627` 条，receive span `89.429s`，stamp span `89.637s`。
    - `/cartographer/odometry_input`：`627` 条，receive span `89.427s`，stamp span `89.637s`；它直接继承 `/scan` stamp。
    - `/navlab/slam/imu`：`62244` 条，receive span `89.592s`，stamp span `62.243s`。
    - `/lidar`：`1246` 条，receive span `89.519s`，stamp span `62.200s`。
    - 跨 `/scan`、`/navlab/slam/imu`、`/cartographer/odometry_input` 的 receive-order header stamp inversion：`1094` 次。
  - Live hover receive window `1782111801.554 .. 1782111819.553`：
    - `/cartographer/odometry_input` 在这个 receive window 内的 stamp 是 `32.410 .. 50.309`，drift `0.9856m`。
    - `/navlab/slam/tf` 在同一个 receive window 内的 stamp 只有 `22.307 .. 35.050`，drift `0.0704m`。
    - `/slam/odom` 同样是 stamp `22.302 .. 35.050`，drift `0.0704m`。
    - `/gazebo/model/odometry` 同 receive window stamp `22.320 .. 35.040`，drift `9.7066m`。
  - Live 按 input header-stamp window `32.410 .. 50.309` 对齐后：
    - `/cartographer/odometry_input` drift `0.9856m`。
    - `/navlab/slam/tf` drift `4.5837m`，receive time `1782111815.792 .. 1782111841.757`。
    - `/slam/odom` drift `4.5837m`，receive time `1782111815.792 .. 1782111841.757`。
    - 结论：live Cartographer 不是完全 near-zero；它对同一 header-stamp window 的大运动输出晚到了 hover receive window 之后。
  - Sorted replay：
    - 跨 input topics 的 receive-order header inversion 为 `0`。
    - 同一 input header-stamp window `32.410 .. 50.309` 下，replay `/navlab/slam/tf` 与 `/slam/odom` drift 均为 `5.3783m`。
    - sorted replay 没有复现 live receive-window near-zero；它消除了 live 的乱序/滞后，所以不能用它直接做 Cartographer 参数 ablation。
- D13.1b timestamp source 代码证据：
  - `navlab/sim/gazebo_sensor/scan_time_normalizer.py`：
    - 输出 `/scan` 的 stamp 优先用 `/clock`；如果没有有效 `/clock`，再用输入 scan stamp；如果也不行，才用 wall monotonic fallback。
    - 实际 D12 中 `/scan` stamp span `89.637s`，而 `/lidar` 和 `/navlab/slam/imu` stamp span 约 `62.2s`；这更像 normalizer 使用了 wall elapsed fallback，而不是 Gazebo sim stamp。
  - `navlab/sim/companion/nodes/scan_reference_drift.py`：
    - `/cartographer/odometry_input` 的 `msg.header.stamp = scan.header.stamp`，因此它继承了 `/scan` 的时间基。
  - `navlab/common/slam/ros/sensors/navlab_slam_imu_bridge/src/navlab_slam_imu_bridge_node.cpp`：
    - IMU bridge 只有输入 stamp 为 `0` 时才替换 stamp；D12 `/navlab/slam/imu` 保留了 Gazebo IMU stamp。
  - `artifacts/sim/hover/20260622T070235Z/slam_runtime.toml`：
    - `use_sim_time=true`、`imu_source_topic=/imu`、`imu_topic=/navlab/slam/imu`、`scan_topic=/scan`。
  - `artifacts/sim/hover/20260622T070235Z/gazebo_sensor_runtime.toml`：
    - `scan_source=x2_virtual_serial`、`scan_topic=/scan`、`vendor_scan_topic=/navlab/x2/vendor_scan`，说明 `/scan` 经过 X2 virtual serial + normalizer 路径。
- D13.1b 结论：
  - 前面说“Cartographer live hover_hold 输出 near-zero”必须修正为：“按 hover receive-time window 看是 near-zero；按 input header-stamp window 对齐后，Cartographer live 确实输出了 `4.58m` 级运动，但输出 receive-time 明显滞后。”
  - 根因已从“Cartographer 参数没有采用 odom/scan”转为“SLAM 输入时间基混用导致 Cartographer collation/output 与 hover/gate receive window 错位”。
  - 具体时间基混用是：`/scan` 和 `/cartographer/odometry_input` 使用接近 wall elapsed 的 stamp，`/navlab/slam/imu` 与 `/lidar` 使用 Gazebo sim stamp。
  - 在修复 timestamp policy / window 对齐前，继续做 D13.2 Cartographer 参数 ablation 是错误方向，继续禁止。

### TODO D13.1c：SLAM 输入时间基统一方案评审

状态：代码侧已完成，待真实 hover 验证

目标：只评审并实现最小 timestamp policy 修复，让 `/scan`、`/cartographer/odometry_input`、`/navlab/slam/imu` 在同一时钟基准下进入 Cartographer。仍不改 hover 控制、不改 landing、不改 selector/correction、不改 Cartographer 参数。

候选方向：

1. 修 `scan_time_normalizer`，不要在没有可靠 `/clock` 时把 `/scan` 推到 wall elapsed；优先保留上游 Gazebo scan stamp，只有输入 stamp 为零/非单调时才最小递增。
2. 把 `/clock` 是否被 normalizer 看到写进 runtime status/artifact，而不是只靠推断。
3. 增加单测覆盖：
   - 有有效上游 scan stamp 时，normalizer 不得用 wall fallback 覆盖成另一套时间基。
   - `/cartographer/odometry_input` 继续继承 `/scan` stamp。
   - hover summary/gate 对 Cartographer 输出做 header-stamp 对齐诊断，避免再把 receive-window lag 误判为 near-zero。

禁止操作：

- 不跑 D13.2 参数 ablation。
- 不改 Cartographer lua 搜索窗口/权重。
- 不改 ExternalNav/correction/selector/hover mission。
- 不把 Gazebo truth、official map、fixed prior 接入 runtime。

执行记录：

- 2026-06-22T07:57Z 开始执行 D13.1c。
- 本轮目标：修复 `/scan` 时间戳策略，让 `/scan`、`/cartographer/odometry_input`、`/navlab/slam/imu` 不再混用 wall elapsed 与 Gazebo sim time；同时增加 normalizer 状态证据和单测。
- 允许改动范围：
  - `navlab/sim/gazebo_sensor/scan_time_normalizer.py`
  - `navlab/tests/gazebo_sensor/x2/test_sensor_runtime.py`
  - 必要时只把 normalizer status topic 加入 hover rosbag/evidence profile，不能变成控制输入。
- 禁止改动范围：
  - 不改 hover mission、landing、drift gate、selector/correction、ExternalNav、Cartographer lua 参数。
  - 不接 Gazebo truth、official map、fixed prior 到 runtime。
  - 不跑 D13.2 参数 ablation；本轮最多跑单测、dry-run 和 timestamp 诊断。
- 本轮验收：
  - 单测证明有效上游 scan stamp 优先于 wall fallback。
  - 单测证明 ideal `/lidar` stamp 可作为 X2 virtual serial 路径的 sim-time anchor。
  - runtime normalizer 有结构化 status，能看到 `clock_seen`、`ideal_scan_seen`、`stamp_source`、fallback 次数。
- 2026-06-22T08:05Z 完成 D13.1c 代码侧修改：
  - `navlab/sim/gazebo_sensor/scan_time_normalizer.py`
    - 新增 `select_scan_stamp_ns()`，时间源优先级改为：
      1. fresh ideal scan stamp（`/lidar` / `config.scan_ideal_topic`）。
      2. fresh `/clock`。
      3. valid vendor input scan stamp。
      4. wall elapsed fallback。
    - fresh anchor 限制为 `1.0s`，避免 `/lidar` 或 `/clock` 停住后继续用 stale stamp。
    - 保留 `monotonic_scan_stamp_ns()`，仍按 scan duration 做最小递增，避免 Cartographer non-sorted。
    - 新增 status topic `/navlab/x2/scan_time_normalizer/status`，字段包括：
      - `clock_seen`
      - `ideal_scan_seen`
      - `latest_stamp_source`
      - `stamp_source_counts`
      - `wall_fallback_count`
      - `monotonic_adjust_count`
      - `latest_clock_age_ms`
      - `latest_ideal_scan_age_ms`
  - `navlab/tests/gazebo_sensor/x2/test_sensor_runtime.py`
    - 新增单测证明 ideal scan stamp、clock、input scan stamp 都会优先于 wall fallback。
    - 新增单测证明 monotonic adjustment 会被标记。
    - 新增单测覆盖 status payload 的 `clock_seen`、`ideal_scan_seen`、`wall_fallback_count`。
  - `orchestration/sim/internal/tasks/helpers/rosbag_topic_sets.go`
    - 把 `/navlab/x2/scan_time_normalizer/status` 加入 mature SLAM review rosbag topics。
    - 该 topic 只是 review/evidence，不加入 hover required topics，不进入控制链。
- 本轮验证：
  - `uv run ruff check sim/gazebo_sensor/scan_time_normalizer.py tests/gazebo_sensor/x2/test_sensor_runtime.py`：通过。
  - `uv run pytest tests/gazebo_sensor/x2/test_sensor_runtime.py -q`：`19 passed`。
  - `python3 -m py_compile navlab/sim/gazebo_sensor/scan_time_normalizer.py scripts/diagnostics/hover_timing_diagnose.py`：通过。
  - `GOCACHE=/tmp/go-cache go test ./internal/tasks/helpers -run 'TestBuildExecutionPlanHoverKeepsMatureSLAMRosbagTopics|TestMatureReviewTopicsExcludeHeavyDiagnosticTopics'`：通过。
  - `go run ./cmd/navlab-sim run hover --dry-run --duration-sec 30`：通过，artifact `artifacts/sim/hover/20260622T080120Z`。
  - dry-run profile 已确认包含 `/navlab/x2/scan_time_normalizer/status`。
- D13.1c 当前结论：
  - 本轮已修复代码层 timestamp policy：`/scan` 不再在存在 fresh `/lidar` sim-time anchor 时退回 wall elapsed fallback。
  - `/cartographer/odometry_input` 仍由 `scan_reference_drift.py` 继承 `/scan` stamp，因此下一轮真实 run 应检查 `/scan`、`/cartographer/odometry_input`、`/navlab/slam/imu` 的 stamp span 是否重新接近。
  - 还不能说 hover 成功；必须下一轮真实 hover/replay 证明 normalizer status 中 `latest_stamp_source=ideal_scan_stamp` 或 `clock`，并且 D13 timing diagnosis 中跨 input topics 的时间基不再明显分裂。

### TODO D13.1d：D13.1c 后真实 hover 时间基验证

状态：执行中

目标：只验证 D13.1c 的 timestamp policy 是否进入真实 runtime，并判断 `/scan`、`/cartographer/odometry_input`、`/navlab/slam/imu` 是否回到同一时间基。仍不改控制、不改 Cartographer 参数、不改 hover mission。

允许操作：

- 跑一轮真实 hover。
- 解析新 artifact 的 rosbag、`/navlab/x2/scan_time_normalizer/status`、`/scan`、`/lidar`、`/navlab/slam/imu`、`/cartographer/odometry_input`、`/navlab/slam/tf`。
- 复用 `scripts/diagnostics/hover_timing_diagnose.py` 输出新 timing 证据。

禁止操作：

- 不改 hover mission、landing、drift gate、selector/correction、ExternalNav、Cartographer lua 参数。
- 不接 Gazebo truth、official map、fixed prior 到 runtime。
- 不把本轮失败包装成成功；只有顶层 `summary.ok=true` 且最终 gate 满足才能说 hover 成功。

验收：

1. `/navlab/x2/scan_time_normalizer/status` 必须出现在 rosbag。
2. status 中 `latest_stamp_source` 应为 `ideal_scan_stamp` 或 `clock`，`wall_fallback_count` 不应持续增长。
3. `/scan`、`/cartographer/odometry_input`、`/navlab/slam/imu` 的 stamp span 应明显接近，不再出现 D12 那种 `89.6s` vs `62.2s` 分裂。
4. 如果 hover 仍失败，要明确失败是否已经从“时间基混用”转移到其他问题。

执行记录：

- 2026-06-22T08:10Z 开始 D13.1d。
- 本轮命令计划：`cd orchestration/sim && env -u NAVLAB_SIM_OVERLAY_SOURCE_MODE NAVLAB_SIM_IMAGE_TAG=jazzy-latest GOCACHE=/tmp/go-cache timeout 900 go run ./cmd/navlab-sim run hover`。
- 本轮不在命令前追加任何新代码修改。

### TODO D13.1e：修复 sim-time anchor 被 scan_duration 单调保护推快

状态：执行中

目标：修复 D13.1d 发现的新断点：normalizer 已经使用 `ideal_scan_stamp`，但 `monotonic_scan_stamp_ns(... min_increment_ns=scan_duration_ns)` 每帧都把输出 stamp 推到 `previous + scan_duration`，导致 `/scan` stamp span 仍然按 wall/sensor duration 跑快，而不是跟随 Gazebo sim time。

D13.1d 新证据：

- artifact：`artifacts/sim/hover/20260622T081517Z`
- `/navlab/x2/scan_time_normalizer/status` 已录到 `744` 条。
- status 证明 D13.1c 进入 runtime：
  - `latest_stamp_source=ideal_scan_stamp`
  - `wall_fallback_count=0`
  - `ideal_scan_seen=true`
- 但最后状态显示：
  - `latest_ideal_scan_stamp_sec=62.1`
  - `latest_output_scan_stamp_sec=94.01679895`
  - `monotonic_adjust_count=651` / `count=652`
- timing diagnosis 仍显示时间基分裂：
  - `/scan` stamp span `89.493s`
  - `/cartographer/odometry_input` stamp span `89.493s`
  - `/navlab/slam/imu` stamp span `58.975s`
  - `/lidar` stamp span `58.900s`

结论：

- D13.1c 修掉了 wall fallback，但没有修掉 scan stamp 被 `scan_duration_ns` 强制推进的问题。
- 当有 trusted sim-time anchor（ideal scan / clock / input scan stamp）时，单调保护只应该保证严格递增，例如 `+1ns`；不能用 scan duration 推进，否则会重新制造一套比 Gazebo sim time 更快的 `/scan` 时间基。

允许操作：

- 只改 `navlab/sim/gazebo_sensor/scan_time_normalizer.py` 的 `select_scan_stamp_ns()` 单调保护策略。
- 只改 `navlab/tests/gazebo_sensor/x2/test_sensor_runtime.py` 对应单测。
- 跑单测、dry-run；必要时再跑真实 hover 验证。

禁止操作：

- 不改 hover mission、landing、drift gate、selector/correction、ExternalNav、Cartographer lua 参数。
- 不接 Gazebo truth、official map、fixed prior 到 runtime。

验收：

- 对 trusted anchor，preferred stamp 小于等于 previous 时只推进 `1ns`，不推进 scan duration。
- 对 wall fallback，仍可用 scan duration 作为最小推进，避免没有任何时间源时输出重复/倒退。

执行记录：

- 2026-06-22T08:21Z 完成 D13.1e 代码侧修改：
  - `navlab/sim/gazebo_sensor/scan_time_normalizer.py`
    - `select_scan_stamp_ns()` 对 trusted anchor（ideal scan / clock / input scan stamp）只使用 `+1ns` 做严格单调保护。
    - 只有完全没有 trusted anchor、走 `wall_elapsed_fallback` 时，才继续使用 `scan_duration_ns` 做最小推进。
  - `navlab/tests/gazebo_sensor/x2/test_sensor_runtime.py`
    - 修改 trusted ideal scan stamp 的 monotonic adjustment 期望：`12.000000000s -> 12.000000001s`。
    - 新增 wall fallback 单测，确认只有 fallback 才按 `scan_duration_ns` 推进。
- 本轮验证：
  - `uv run ruff check sim/gazebo_sensor/scan_time_normalizer.py tests/gazebo_sensor/x2/test_sensor_runtime.py`：通过。
  - `uv run pytest tests/gazebo_sensor/x2/test_sensor_runtime.py -q`：`20 passed`。
  - `python3 -m py_compile navlab/sim/gazebo_sensor/scan_time_normalizer.py scripts/diagnostics/hover_timing_diagnose.py`：通过。
  - `GOCACHE=/tmp/go-cache go test ./internal/tasks/helpers -run 'TestBuildExecutionPlanHoverKeepsMatureSLAMRosbagTopics|TestMatureReviewTopicsExcludeHeavyDiagnosticTopics'`：通过。
- 下一步：跑第二轮真实 hover，重点检查 `/scan` stamp span 是否贴近 `/lidar` 和 `/navlab/slam/imu`，以及 `monotonic_adjust_count` 是否不再接近每帧增长。

### TODO D13.1f：修复 LaserScan beam timing 与 sim-time stamp 重叠

状态：执行中

目标：修复 D13.1e 后出现的 Cartographer fatal：`sensor_bridge.cpp:211 Ignored subdivision ... previous subdivision time is not before current subdivision time` 和 `map_by_time.h:43 Check failed ... same time`。

D13.1e 验证证据：

- artifact：`artifacts/sim/hover/20260622T082035Z`
- timing diagnosis 已证明时间基分裂修复：
  - `/scan` stamp span `60.400s`
  - `/cartographer/odometry_input` stamp span `60.500s`
  - `/navlab/slam/imu` stamp span `60.550s`
  - `/lidar` stamp span `60.500s`
- normalizer status：
  - `latest_stamp_source=ideal_scan_stamp`
  - `wall_fallback_count=0`
- 新 fatal：
  - `sensor_bridge.cpp:211 Ignored subdivision of a LaserScan message ... previous subdivision time ... is not before current subdivision time`
  - `map_by_time.h:43 Check failed: data.time > ... (same time)`
- scan 字段证据：
  - `/scan.header.stamp` 约每 `0.1s` 前进。
  - `/scan.scan_time` 约 `0.143s`。
  - `/scan.time_increment * (ranges-1)` 约 `0.143s`。
  - 因此前一帧 beam subdivision 会延伸到后一帧 header stamp 之后，造成 Cartographer 内部时间重叠。

结论：

- D13.1e 修好了外部 topic 时间基，但 normalizer 还保留 vendor driver 的 beam timing 字段；这些字段属于 7Hz wall/vendor 输出，不适合直接叠加到 10Hz Gazebo sim-time anchor 上。
- 对 sim-time anchored `/scan`，normalizer 必须把 `time_increment` 置 `0`，避免 Cartographer 按错误 beam timing 做 subdivision。
- `scan_time` 可以保留为状态/频率参考，但只要 `time_increment=0`，Cartographer 不应再生成跨帧 beam time 重叠。

允许操作：

- 只改 `navlab/sim/gazebo_sensor/scan_time_normalizer.py`：当 stamp source 是 trusted anchor 时，把输出 `LaserScan.time_increment` 清零，并记录状态计数。
- 只改 `navlab/tests/gazebo_sensor/x2/test_sensor_runtime.py`：增加纯函数/策略单测。
- 跑单测、必要时跑真实 hover 验证。

禁止操作：

- 不改 hover mission、landing、drift gate、selector/correction、ExternalNav、Cartographer lua 参数。
- 不接 Gazebo truth、official map、fixed prior 到 runtime。

验收：

- trusted anchor 下输出 scan 的 `time_increment=0`。
- wall fallback 下仍保留 vendor timing 字段。
- Cartographer 不再出现 `sensor_bridge.cpp:211` subdivision 时间重叠或 `map_by_time.h:43` duplicate time fatal。

执行记录：

- 2026-06-22T08:30Z 完成 D13.1f 代码侧修改：
  - `navlab/sim/gazebo_sensor/scan_time_normalizer.py`
    - 新增 `should_zero_scan_time_increment()`。
    - 当 stamp source 是 `ideal_scan_stamp`、`clock` 或 `input_scan_stamp` 时，输出 `/scan.time_increment=0.0`。
    - 新增 `time_increment_zeroed_count` status 字段。
    - `wall_elapsed_fallback` 下不清零，保留原 vendor timing 字段。
  - `navlab/tests/gazebo_sensor/x2/test_sensor_runtime.py`
    - 新增 trusted anchor 清零 `time_increment` 的单测。
    - status payload 单测覆盖 `time_increment_zeroed_count`。
- 本轮验证：
  - `uv run ruff check sim/gazebo_sensor/scan_time_normalizer.py tests/gazebo_sensor/x2/test_sensor_runtime.py`：通过。
  - `uv run pytest tests/gazebo_sensor/x2/test_sensor_runtime.py -q`：`21 passed`。
  - `python3 -m py_compile navlab/sim/gazebo_sensor/scan_time_normalizer.py scripts/diagnostics/hover_timing_diagnose.py`：通过。
  - `GOCACHE=/tmp/go-cache go test ./internal/tasks/helpers -run 'TestBuildExecutionPlanHoverKeepsMatureSLAMRosbagTopics|TestMatureReviewTopicsExcludeHeavyDiagnosticTopics'`：通过。
- 下一步：第三轮真实 hover，验证 Cartographer 不再出现 `sensor_bridge.cpp:211` 和 `map_by_time.h:43` fatal。

### TODO D13.1g：修复 Cartographer 100ns 时间粒度下的重复 scan stamp

状态：执行中

目标：修复 D13.1f 后仍出现的 Cartographer fatal。D13.1f 已把 trusted anchor 下的 `/scan.time_increment` 清零，但真实 run 仍然在 Cartographer 内部看到相同时间戳，因此本轮只查/修 timestamp 粒度，不改控制链。

D13.1f 验证证据：

- artifact：`artifacts/sim/hover/20260622T082705Z`
- D13.1f 进入 runtime：
  - `/scan.time_increment=0.0`
  - `/scan.scan_time≈0.143s`
  - `/navlab/x2/scan_time_normalizer/status` 中 `latest_stamp_source=ideal_scan_stamp`、`wall_fallback_count=0`、`time_increment_zeroed_count=662`
- Cartographer 仍 fatal：
  - `sensor_bridge.cpp:211 Ignored subdivision ... previous subdivision time 621355968009000000 is not before current subdivision time 621355968009000000`
  - `map_by_time.h:43 Check failed: data.time > ... (621355968009000000 vs. 621355968009000000)`
- 离线读 D13.1f bag：
  - `/scan` 没有 exact duplicate header stamp，`dups=0`、`noninc=0`。
  - 但存在 normalizer 生成的 `+1ns` 单调修正，例如 `5.800000000s -> 5.800000001s`、`7.900000000s -> 7.900000001s`。

官方/源码依据：

- ROS `sensor_msgs/LaserScan` 定义中，`header.stamp` 是第一束光的采集时间，`time_increment` 是每束之间的时间，`scan_time` 是两帧 scan 之间的时间。因此 D13.1f 清零的是每束 timing，不应该把 `scan_time` 当作每束时间继续补丁。
- Cartographer 头文件 `/opt/ros/jazzy/include/cartographer/common/time.h` 定义 `UniversalTimeScaleClock::period = 1/10000000`，即内部时间 tick 是 `100ns`，并注释说明 Universal Time Scale timestamps 是自公元 1 年以来的 `100 nanosecond ticks`。
- 因此 normalizer 的 `+1ns` 严格递增在 ROS nanosecond 层面有效，但进入 Cartographer `common::Time` 后会被折叠到同一个 100ns tick，仍触发 `map_by_time` 的 strictly increasing 检查。

结论：

- 当前 fatal 不是 hover 控制、landing、ExternalNav、Cartographer 参数、Gazebo truth 或 Foxglove 问题。
- 真实断点是 normalizer 的单调修正粒度小于 Cartographer 内部时间粒度。
- 最小修复应把 trusted anchor 下的最小单调推进从 `1ns` 改为 `100ns`，仍不恢复 `scan_duration_ns` 推快时钟。

允许操作：

- 只改 `navlab/sim/gazebo_sensor/scan_time_normalizer.py`：新增 Cartographer time tick 常量，trusted anchor 重复/倒退时用 `100ns` 推进。
- 只改 `navlab/tests/gazebo_sensor/x2/test_sensor_runtime.py`：加单测覆盖重复 trusted stamp 时输出 `+100ns`，防止再次退回 `+1ns`。
- 跑 targeted Python 测试、ruff、py_compile、必要 Go helper 测试。
- 再跑真实 hover 验证 Cartographer fatal 是否消失。

禁止操作：

- 不改 hover mission、landing、drift gate、selector/correction、ExternalNav、Cartographer lua 参数。
- 不接 Gazebo truth、official map、fixed prior 到 runtime。
- 不把失败 hover 说成成功；真实 hover 仍以顶层 `summary.ok=true` 为准。

验收：

- trusted anchor 重复 stamp 时 normalizer 输出至少推进 `100ns`。
- `/scan`、`/cartographer/odometry_input`、`/navlab/slam/imu` 的 stamp span 仍保持同一 sim-time 量级，不能回到 D13.1d 的 `89s vs 59s` 时间基分裂。
- Cartographer log 不再出现 `sensor_bridge.cpp:211` 和 `map_by_time.h:43` duplicate-time fatal。
- 如果 hover 仍失败，下一步只基于新 summary/blocker 继续定位，不回头乱调高度/landing/correction。

执行记录：

- 2026-06-22T08:37Z 开始 D13.1g。
- 2026-06-22T08:39Z 完成 D13.1g 代码侧修改：
  - `navlab/sim/gazebo_sensor/scan_time_normalizer.py`
    - 新增 `CARTOGRAPHER_TIME_TICK_NS = 100`。
    - trusted anchor 下重复/倒退 stamp 的最小单调推进从 `1ns` 改为 `100ns`。
    - wall fallback 仍沿用 `scan_duration_ns` 推进，不改变 D13.1e 的时间基修复。
  - `navlab/tests/gazebo_sensor/x2/test_sensor_runtime.py`
    - 修改 trusted anchor 单调修正断言为 `+100ns`。
    - 新增 clock trusted repeat 单测，明确禁止再退回 `+1ns`。
- 本轮验证：
  - `uv run ruff check sim/gazebo_sensor/scan_time_normalizer.py tests/gazebo_sensor/x2/test_sensor_runtime.py`：通过。
  - `uv run pytest tests/gazebo_sensor/x2/test_sensor_runtime.py -q`：`22 passed`。
  - `python3 -m py_compile navlab/sim/gazebo_sensor/scan_time_normalizer.py scripts/diagnostics/hover_timing_diagnose.py`：通过。
  - `GOCACHE=/tmp/go-cache go test ./internal/tasks/helpers -run 'TestBuildExecutionPlanHoverKeepsMatureSLAMRosbagTopics|TestMatureReviewTopicsExcludeHeavyDiagnosticTopics'`：通过/cached。
- 下一步：跑真实 hover 验证；重点不是判成功，而是先确认 Cartographer log 是否不再出现 duplicate-time fatal。

### TODO D13.1h：修复 source selector 完成态 blocker 被误提升为顶层失败

状态：执行中

目标：修复 D13.1g 后真实 hover 的新断点：SLAM duplicate-time fatal 已消失，hover mission 和 landing 均成功，但顶层 summary 仍因 `/external_nav/source_selector/status` 的完成态 fail-closed blocker 被 probe 聚合逻辑误判为 gate blocker。

D13.1g 真实验证证据：

- artifact：`artifacts/sim/hover/20260622T083826Z`
- hover mission：`ok=true`
- takeoff：`takeoff_ack_ok=true`
- 高度证据一致：
  - `external_nav_height_m=0.498`
  - `fcu_local_height_m≈0.494`
  - `rangefinder_relative_height_m=0.49`
  - target `0.5m`
- hover hold：`18.0s`，本体 hover drift gate 内部 `ok=true`。
- landing：`ok=true`，`land_command_accepted=true`，`S12 landing_complete`，`disarmed=true`。
- Cartographer：log 无 `FATAL`、无 `sensor_bridge.cpp:211`、无 `map_by_time.h:43`。
- 顶层 blockers 只剩：
  - `not_hover_correction_phase`
  - `scan_reference_eligibility_not_allowed`
  - `scan_reference_xy_axes_not_allowed`
- 这些 blockers 来自 `/external_nav/source_selector/status` 的最新状态：`hover_phase=complete`、`source=slam_passthrough`。在 complete 阶段 selector fail-closed 是正常行为，不应作为 hover_hold 失败证据。

代码证据：

- `orchestration/sim/internal/tasks/gate_evaluation.go` 的 `probeBlockers()` 会把 probe sample 的 `parsed.blockers` 直接提升为顶层 blocker。
- 当前只跳过：
  - `/landing/status`
  - `/scan_reference_drift/status`
  - `/scan_reference_correction/status`
- 没有跳过 `/external_nav/source_selector/status`，导致 source selector 在非 correction phase 的正常 fail-closed 被误判为任务失败。
- source selector 的真实合同检查已经由 `externalNavSourceSelectorBlockers()` 覆盖：禁止 Gazebo truth、official map、错误 output topic/frame/child frame。无需把状态机上下文 blocker 原样提升。

结论：

- 这不是 hover 控制失败，也不是重新调 SLAM/correction 的问题。
- 最小修复是 gate 聚合层忽略 `/external_nav/source_selector/status` 的上下文 blockers，只保留 source selector 的安全合同检查。
- 仍保留 correction hover-window 证据：`scanReferenceCorrectionBlockers()` 和 hover-window status summary 继续生效，不能绕过 Phase 4A/4B safety gate。

允许操作：

- 只改 `orchestration/sim/internal/tasks/gate_evaluation.go`：`probeBlockers()` 排除 `/external_nav/source_selector/status` 的 parsed blockers。
- 只改 `orchestration/sim/internal/tasks/gate_evaluation_test.go`：新增单测证明 source selector 完成态 blockers 不会进入顶层，同时 source selector truth/map/output/frame 合同仍由 `externalNavSourceSelectorBlockers()` 单独拦截。
- 跑 targeted Go tests。
- 再跑真实 hover，只有顶层 `summary.ok=true` 才能说完整 hover 成功。

禁止操作：

- 不改 hover mission、landing、drift gate、selector/correction runtime、ExternalNav、Cartographer lua 参数。
- 不接 Gazebo truth、official map、fixed prior 到 runtime。

验收：

- D13.1g artifact 的 blocker 机制不再把 `/external_nav/source_selector/status` 的 complete-phase blockers 提升成顶层失败。
- 真实 hover 若 hover mission、landing、SLAM fatal、source selector 安全合同、correction safety gate、Gazebo review-only drift 都通过，则顶层 `summary.ok=true`。

执行记录：

- 2026-06-22T08:45Z 开始 D13.1h。
- 2026-06-22T08:48Z 完成 D13.1h 代码侧修改：
  - `orchestration/sim/internal/tasks/gate_evaluation.go`
    - `probeBlockers()` 新增跳过 `/external_nav/source_selector/status` 的 parsed blockers。
    - 其它 topic blockers 不变；source selector 安全合同仍由 `externalNavSourceSelectorBlockers()` 检查。
  - `orchestration/sim/internal/tasks/gate_evaluation_test.go`
    - 新增 `TestProbeBlockersIgnoresSourceSelectorContextBlockers`，覆盖 complete-phase selector blockers 不再泄漏到顶层。
- 本轮验证：
  - `gofmt -w internal/tasks/gate_evaluation.go internal/tasks/gate_evaluation_test.go`：完成。
  - `GOCACHE=/tmp/go-cache go test ./internal/tasks -run 'TestProbeBlockersIgnoresSourceSelectorContextBlockers|TestMetricSummaryIncludesExternalNavSourceSelectorEvidence|TestExternalNavSourceSelectorBlockersRejectTruthInput|TestExternalNavCandidateIsAllowedExternalNavInput'`：通过。
- 下一步：重跑真实 hover，只有顶层 `summary.ok=true` 才能宣布完整 hover 成功。

### TODO D13.1i：修复 IMU probe 采样方式过脆导致的假失败

状态：执行中

目标：修复 D13.1h 后真实 hover 的新断点：任务执行到完整 run，但独立 `imu_probe` 因采样超时返回 `rc=20`，而同一 run 的其它证据显示 IMU runtime 实际正常。

D13.1h 真实验证证据：

- artifact：`artifacts/sim/hover/20260622T084314Z`
- 顶层失败：`probe_failed:imu_probe:rc=20`、`topic_sample_missing:/imu`。
- 但同一 artifact 中：
  - `frame_contract_probe.json` 的 `/imu` 采样成功。
  - `slam_backend.runtime.log` 持续显示 `imu rate≈990Hz`。
  - `/external_nav/status` 中 IMU fresh。
  - Cartographer 仍无 `FATAL`、无 `sensor_bridge.cpp:211`、无 `map_by_time.h:43`。
- `imu_probe.py` 当前对普通 topic 采用循环调用 `ros2 topic echo --once /imu`，8 秒内反复创建 CLI/DDS 节点；这比真实 runtime 订阅更容易受 discovery/participant churn 影响。

结论：

- 当前不是 IMU 数据链真实断裂，也不是 SLAM fatal 回归。
- 问题在 probe 基础设施：普通 message topic 用 subprocess `ros2 topic echo` 采样过脆。
- 最小修复是让 `rosProbeScript()` 对非 string topic 优先用单个 rclpy 节点订阅一次消息；失败时再 fallback 到原 `ros2 topic echo`。

允许操作：

- 只改 `orchestration/sim/internal/tasks/helpers/runtime_specs.go` 中生成的 probe 脚本。
- 只改相关 Go 测试，证明生成脚本包含 rclpy 普通 topic subscriber fallback。
- 跑 targeted helper tests。
- 再跑真实 hover。

禁止操作：

- 不改 hover mission、landing、drift gate、selector/correction runtime、ExternalNav、Cartographer lua 参数。
- 不把 probe 偶发失败当作飞行成功；真实验收仍看顶层 `summary.ok=true`。

验收：

- `imu_probe.py` 对 `/imu` 这类普通 message topic 优先使用 rclpy subscription。
- string topic 仍保留现有 JSON/status 采样逻辑。
- 真实 hover 不再因 `imu_probe` 假失败阻塞；若仍失败，必须是新的真实 blocker。

执行记录：

- 2026-06-22T08:52Z 开始 D13.1i。
- 2026-06-22T08:56Z 完成 D13.1i 代码侧修改：
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
    - `sample_topic()` 先调用 `sample_message_topic()`。
    - `sample_message_topic()` 使用单个 rclpy node 发现 topic type、订阅一条普通 ROS message；成功时标记 `sample_method=rclpy_subscription`。
    - rclpy 采样失败时仍 fallback 到原 `ros2 topic echo --once` 重试逻辑。
  - `orchestration/sim/internal/tasks/runtime_artifacts_test.go`
    - probe 脚本断言新增 `sample_message_topic`、`rclpy_subscription`、`get_message`，确保普通 topic 不再只靠 CLI echo。
- 本轮验证：
  - `gofmt -w internal/tasks/helpers/runtime_specs.go internal/tasks/runtime_artifacts_test.go`：完成。
  - `GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -run 'TestGeneratedRuntimeArtifacts|TestHoverRuntimeArtifacts|TestExecutionPlanHover|TestBuildExecutionPlanHover|TestProbe'`：通过。
- 下一步：重跑真实 hover，确认不再出现 `imu_probe` 假失败。

### TODO D13.1j：修复 hover mission 对单个 ExternalNav stale 样本过敏 abort

状态：执行中

目标：修复 D13.1i 后真实 hover 的新断点：probe 已稳定、SLAM fatal 已消失，但 hover mission 因 1 个瞬时 `slam_quality_stale/imu_stale` 状态样本立刻 abort，导致 hover hold 只有 `15.2s/18s`。

D13.1i 真实验证证据：

- artifact：`artifacts/sim/hover/20260622T084759Z`
- probe 全部成功：
  - `imu_probe`：`ok=true`，`sample_method=rclpy_subscription`
  - `frame_contract_probe`：`/imu`、`/scan`、`/slam/odom` 均成功
- Cartographer：无 `FATAL`、无 `sensor_bridge.cpp:211`、无 `map_by_time.h:43`
- Gazebo review-only hover drift：`max_horizontal_drift_m≈0.050m`，满足 `0.10m` gate。
- hover 高度证据一致：ExternalNav、FCU local、rangefinder 均约 `0.49~0.50m`。
- 失败原因：hover mission 在 `S6 hover_hold` 15.2s 处进入 `S_abort`，blocker 是 `slam_quality_lost_after_airborne`。
- rosbag 证据：`/external_nav/status` 在 hover_hold 内只有 1 条 bad 样本：
  - `state=slam_quality_stale`
  - `slam_quality_reason=imu_stale`
  - `imu.age_ms=597.527`
  - `odom.fresh=true`，`scan.fresh=true`
  - 下一条状态约 0.5s 后恢复 `ready=true`、`slam_quality=good`

结论：

- 当前不是持续 ExternalNav 失效，也不是 Gazebo drift、Cartographer fatal、probe 假失败。
- 真实断点是 hover mission 对 ExternalNav quality 的瞬时单样本 stale 过敏；一个 0.5s 级调度/状态抖动直接结束 hover。
- 保持 fail-closed 原则：不能忽略持续坏 SLAM，也不能把坏 `/slam/odom` 拉飞 FCU；但 mission abort 应该要求质量连续坏超过短 grace window。

允许操作：

- 只改 `navlab/sim/companion/nodes/hover_mission.py`：airborne 后 external nav / slam_quality lost 需要连续超过 grace 秒才 abort，单个短暂 bad status 记录为 warning/计数但不终止。
- 只改 `navlab/tests/companion/test_hover_mission.py`：新增状态机单测，证明短暂 quality lost 不 abort，持续 quality lost 会 abort。
- 如有 config/runtime 字段，默认值保守设置为 `1.0s` 左右；不能关闭 quality gate。
- 跑 targeted Python 单测。
- 再跑真实 hover。

禁止操作：

- 不改 hover 高度门槛、landing 阈值、Gazebo drift gate、Cartographer 参数、ExternalNav bridge quality gate、selector/correction runtime。
- 不接 Gazebo truth、official map、fixed prior 到 runtime。

验收：

- 单个约 0.5s `imu_stale` status 不会提前结束 18s hover_hold。
- 持续 ExternalNav/SLAM quality loss 仍会进入 abort。
- 真实 hover 顶层必须 `summary.ok=true` 才能宣布完整成功。

执行记录：

- 2026-06-22T09:00Z 开始 D13.1j。
- 2026-06-22T09:05Z 完成 D13.1j 代码侧修改：
  - `navlab/sim/companion/nodes/hover_mission.py`
    - `HoverInputs` 新增 `slam_quality_loss_duration_sec`、`external_nav_loss_duration_sec`、`mavlink_external_nav_loss_duration_sec`、`fcu_local_position_loss_duration_sec`。
    - `HoverRequirements` 新增 `external_nav_loss_grace_sec`，默认 `1.0s`。
    - airborne 后 ExternalNav/SLAM/MAVLink local position 只有连续 lost 超过 grace 才 abort。
    - airborne 后短暂 lost 不再回到 preflight `wait_ready`，而是继续当前 hover/hold；持续 lost 仍 fail-closed abort。
    - `/navlab/hover/status` 增加 loss duration 字段，方便 artifact 复核。
  - `navlab/tests/companion/test_hover_mission.py`
    - 持续 lost abort 单测改为 `loss_duration_sec=1.1`。
    - 新增短暂 `0.5s` nav quality loss 不 abort 的单测。
- 本轮验证：
  - `uv run ruff check sim/companion/nodes/hover_mission.py tests/companion/test_hover_mission.py`：通过。
  - `uv run pytest tests/companion/test_hover_mission.py -q`：`51 passed`。
  - `python3 -m py_compile navlab/sim/companion/nodes/hover_mission.py`：通过。
- 下一步：重跑真实 hover，确认单个 `imu_stale` 不再提前结束 hover_hold。
- 2026-06-22T09:08Z D13.1j 后真实 hover 验证通过：
  - artifact：`artifacts/sim/hover/20260622T085622Z`
  - 顶层 summary：`ok=true`、`status=TASK_STATUS_OK`、`blocked=false`、`blockers=[]`。
  - hover mission：`ok=true`、`reason=hover_complete`、`takeoff_ack_ok=true`。
  - hover_hold：`17.950s / 18s`，duration tolerance 内通过。
  - 高度证据一致：
    - `external_nav_height_m=0.495m`
    - `fcu_local_height_m≈0.494m`
    - `rangefinder_relative_height_m=0.49m`
    - target `0.5m`
  - hover drift：
    - mission horizontal drift `0.018m`
    - mission horizontal span `0.053m`
    - Gazebo review-only max horizontal drift `0.039m`
  - landing：`landing_ok=true`、`land_command_accepted=true`、`land_mode_seen=true`、`touchdown_confirmed=true`、`disarmed=true`、`motors_safe=true`、`S12 landing_complete`。
  - Cartographer：log 无 `FATAL`、无 `sensor_bridge.cpp:211`、无 `map_by_time.h:43`。
  - probes：4 个 probe 全部 `ok=true`；`imu_probe` 使用 `rclpy_subscription` 成功采到 `/imu`。
  - source/correction safety：
    - source selector 未使用 Gazebo truth / known map。
    - correction `phase4b_consistency_ok=true`、`runtime_consistency_ok=true`、`hover_window_applied_without_*_count=0`。
- D13 结论：
  - 根因链路按证据收敛为：`scan_time_normalizer` 时间基混用 -> trusted stamp 被 scan duration 推快 -> 修正后暴露 Cartographer 100ns tick 重复 -> 修复后 SLAM fatal 消失。
  - 后续失败分别是 gate/probe/mission 抖动处理问题，不是 hover 高度门槛或 Gazebo truth 问题。
  - 当前完整 hover task 已通过，不能再把旧失败 artifact 当当前状态。

### TODO D13.1k：重复 hover 验证，排除单次偶然成功

状态：执行中

目标：在 `20260622T085622Z` 首次顶层 `summary.ok=true` 后，再连续跑至少 2 次真实 hover，确认低漂移不是偶然。

为什么这次 drift 会明显变小：

- 之前看到的大漂移不是靠调高度门槛解决的，而是时间/SLAM链路和任务状态机链路叠加出的问题：
  - `/scan` 时间基混用导致 Cartographer 输出窗口错位。
  - trusted stamp 被 scan duration 推快后又触发 Cartographer duplicate-time fatal。
  - probe 偶发假失败会把正常 run 判失败。
  - hover mission 对单个 `imu_stale` 状态样本过敏 abort，导致没有完成完整 hold window。
- D13.1g-D13.1j 后，Cartographer 正常持续输出，ExternalNav 高度/FCU/rangefinder 证据一致，hover mission 能跑完整 hold window；因此 Gazebo review-only body drift 回到厘米级。
- 仍需重复验证，避免把一次“运气好”误认为稳定。

重复验证验收：

- 每次顶层 `summary.ok=true`。
- 每次 `summary.md` 为 `Result: PASS`。
- 每次 Cartographer log 无 `FATAL`、无 `sensor_bridge.cpp:211`、无 `map_by_time.h:43`。
- 每次 probes 全部 ok。
- 每次 hover hold 接近 `18s`，在 duration tolerance 内。
- 每次 Gazebo review-only max horizontal drift 低于 `0.10m` gate；同时记录实际数值。

执行记录：

- 2026-06-22T09:12Z 开始重复验证 V1/V2。

### TODO D13.1l：修复 probe rclpy fallback 对 std_msgs/String 状态不解析导致的假 gate 失败

状态：执行中

目标：修复重复验证 V1 `20260622T102033Z` 暴露的新问题：hover 本体和 Gazebo drift 均通过，但顶层因 `x2_status_missing`、`landing_not_evaluated` 失败。

V1 证据：

- artifact：`artifacts/sim/hover/20260622T102033Z`
- hover mission：`ok=true`、`reason=hover_complete`、`takeoff_ack_ok=true`。
- hover hold：`17.950s / 18s`，duration tolerance 内通过。
- mission drift：`0.021m`。
- Gazebo review-only max drift：`0.063m`，仍低于 `0.10m` gate。
- landing 在 mission summary 内：`landing.ok=true`、`S12 landing_complete`、`disarmed=true`、`motors_safe=true`。
- probes 全部 `ok=true`，但 `/sim/x2/status` 和 `/navlab/landing/status` 样本使用 `sample_method=rclpy_subscription` 后没有 `data/parsed` JSON。
- 顶层 blockers：`x2_status_missing`、`landing_not_evaluated`。

结论：

- 这次不是 hover 漂移回归，也不是 Cartographer fatal 回归。
- 是 D13.1i probe generic rclpy sampler 的回归：当 `std_msgs/String` status topic 走 generic rclpy path 时，只生成摘要，没有保留 `msg.data` 和 parsed JSON，导致 gate 缺证据。

允许操作：

- 只改 `orchestration/sim/internal/tasks/helpers/runtime_specs.go` 的 generated probe script：generic `sample_message_topic()` 如果消息有 `data: str`，必须像 string sampler 一样填充 `data`、`parsed`、`stdout`。
- 只改相关测试，防止 String status fallback 再丢 JSON。
- 跑 targeted Go tests。
- 然后继续重复 hover 验证，直到获得两次额外 `summary.ok=true` 或暴露真实新 blocker。

禁止操作：

- 不改 hover mission、landing 阈值、Gazebo drift gate、Cartographer 参数、ExternalNav/selector/correction runtime。

执行记录：

- 2026-06-22T10:27Z 开始 D13.1l。
- 2026-06-22T10:29Z 完成 D13.1l 代码侧修改：
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
    - generic `sample_message_topic()` 对 `std_msgs/String` 这类有 `data: str` 的消息，保留 `data`、`stdout="data: ..."`，并写入 `parsed` JSON。
    - 非 String 普通消息仍使用摘要输出。
  - `orchestration/sim/internal/tasks/runtime_artifacts_test.go`
    - probe 脚本断言新增 String fallback JSON 解析证据。
- 本轮验证：
  - `gofmt -w internal/tasks/helpers/runtime_specs.go internal/tasks/runtime_artifacts_test.go`：完成。
  - `GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -run 'TestGeneratedRuntimeArtifacts|TestHoverRuntimeArtifacts|TestExecutionPlanHover|TestBuildExecutionPlanHover|TestProbe'`：通过。
- 下一步：重新开始重复 hover 验证 V1/V2；`20260622T102033Z` 由于 probe status JSON 丢失导致假 gate 失败，不计为通过，但其 drift 证据记录为 hover 本体仍低漂移。

### TODO D13.1m：提高 CycloneDDS probe participant 上限，消除 IMU probe 偶发 participant exhaustion

状态：执行中

目标：修复重复验证 V2 `20260622T102639Z` 暴露的 probe 基础设施问题：hover 本体成功，但 `imu_probe` 因 CycloneDDS participant index 耗尽失败。

V2 证据：

- artifact：`artifacts/sim/hover/20260622T102639Z`
- hover 本体：`hover_mission.ok=true`、hover hold `18.0s`、mission drift `0.010m`、高度证据一致、landing complete。
- SLAM：Cartographer 持续 `imu rate≈990Hz`，无 fatal。
- `frame_contract_probe` 同 run 成功采到 `/imu`。
- `imu_probe` 失败 stderr：`Failed to find a free participant index for domain 0`。

官方依据：

- CycloneDDS 配置中 `Discovery/ParticipantIndex=auto` 会自动寻找可用 participant index，受 `MaxAutoParticipantIndex` 上限约束；默认上限较低，多个 ROS/DDS 进程并发/快速创建时可能耗尽。

结论：

- 当前不是 hover 漂移、IMU runtime 或 SLAM 真实失败。
- 是 probe 容器在 domain 0 上创建 DDS participant 时撞到 CycloneDDS auto participant index 上限。
- 最小修复是给 runtime/probe ROS env 设置 CycloneDDS URI，提高 `MaxAutoParticipantIndex`。

允许操作：

- 只改 orchestration ROS env 生成：给 baseline ROS env 增加 `CYCLONEDDS_URI`，设置 `ParticipantIndex=auto` 和更高 `MaxAutoParticipantIndex`。
- 只改相关 Go 测试。
- 跑 targeted Go tests。
- 然后继续重复 hover V1/V2。

禁止操作：

- 不改 hover mission、landing、Gazebo drift gate、Cartographer 参数、ExternalNav/selector/correction runtime。

执行记录：

- 2026-06-22T10:34Z 开始 D13.1m。
- 2026-06-22T10:37Z 完成 D13.1m 代码侧修改：
  - `orchestration/sim/internal/tasks/runtime_specs.go`
    - `baselineEnv()` 增加 `CYCLONEDDS_URI`，设置 `ParticipantIndex=auto` 与 `MaxAutoParticipantIndex=512`。
    - 该 env 会进入 ROS probes 和 rosbag runtime，降低 domain 0 下 participant exhaustion 概率。
  - `orchestration/sim/internal/tasks/helpers/runtime_specs.go`
    - helper baseline env 同步包含 `CYCLONEDDS_URI`。
  - `orchestration/sim/internal/tasks/runtime_specs_test.go`
    - 断言 probe/rosbag env 包含 `MaxAutoParticipantIndex`。
- 本轮验证：
  - `gofmt -w internal/tasks/runtime_specs.go internal/tasks/helpers/runtime_specs.go internal/tasks/runtime_specs_test.go`：完成。
  - `GOCACHE=/tmp/go-cache go test ./internal/tasks ./internal/tasks/helpers -run 'TestBuildRuntimeSpecsFromExecutionPlan|TestBuildExecutionPlanHoverKeepsMatureSLAMRosbagTopics|TestGeneratedRuntimeArtifacts|TestHoverRuntimeArtifacts|TestProbe'`：通过。
- 下一步：继续重复 hover 验证 V2。
- 2026-06-22T10:35Z 重复验证 V1 通过：
  - artifact：`artifacts/sim/hover/20260622T102416Z`
  - 顶层：`ok=true`、`blockers=[]`。
  - hover hold：`17.950s`。
  - mission drift：`0.017m`；mission horizontal span：`0.046m`。
  - Gazebo review-only max drift：`0.056m`。
  - 高度：ExternalNav `0.500m`、FCU `0.494m`、rangefinder relative `0.50m`。
  - landing：`ok=true`。
  - correction safety：`runtime_consistency_ok=true`、`phase4b_consistency_ok=true`。
  - Cartographer fatal check：无 `FATAL`、无 `sensor_bridge.cpp`、无 `map_by_time`。
- 2026-06-22T10:33Z 重复验证 V2 初次尝试 `20260622T102639Z` 暴露 probe participant exhaustion：
  - hover 本体仍通过：hold `18.0s`、mission drift `0.010m`、landing complete。
  - 失败源是 `imu_probe` 创建 DDS participant 失败：`Failed to find a free participant index for domain 0`。
  - 该 run 不计为通过，但作为 hover 本体低漂移证据保留。
- 2026-06-22T10:40Z D13.1m 后重复验证 V2 通过：
  - artifact：`artifacts/sim/hover/20260622T103144Z`
  - 顶层：`ok=true`、`blockers=[]`。
  - hover hold：`18.000s`。
  - mission drift：`0.021m`；mission horizontal span：`0.041m`。
  - Gazebo review-only max drift：`0.055m`。
  - 高度：ExternalNav `0.496m`、FCU `0.494m`、rangefinder relative `0.49m`。
  - landing：`ok=true`。
  - correction safety：`runtime_consistency_ok=true`、`phase4b_consistency_ok=true`。
  - Cartographer fatal check：无 `FATAL`、无 `sensor_bridge.cpp`、无 `map_by_time`。
- 重复验证结论：
  - 连续有效通过 run：`20260622T085622Z`、`20260622T102416Z`、`20260622T103144Z`。
  - 三次 Gazebo review-only max drift 分别为 `0.039m`、`0.056m`、`0.055m`，均低于 `0.10m` gate，且不是单次偶然。
  - hover mission drift 分别为 `0.018m`、`0.017m`、`0.021m`，均为厘米级。
