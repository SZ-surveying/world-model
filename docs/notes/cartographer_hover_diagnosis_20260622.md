# Cartographer hover drift diagnosis - 2026-06-22

## 结论先行

当前 hover 失败的直接原因不是 hover mission、landing、FCU arm/takeoff，也不是 correction 幅度太小。

直接原因是：Cartographer/ExternalNav 链路没有把 Gazebo body 的真实横向漂移估计给 FCU。FCU/EKF 看到的 `/slam/odom_corrected -> /external_nav/odom` 只有约 2-3 cm 横移，所以飞控认为自己几乎没漂；但 Gazebo review-only body odometry 显示机体在 hover window 内漂了约 0.78 m。因此控制器没有足够依据去修正真实 body drift。

当前 correction 也没有真正参与修漂移，因为 runtime correction 被设计成必须看到 `phase4b_consistency_ok=true` 才能启用，但 `/navlab/scan_reference_drift/status` 现在没有发布这个 runtime-safe 字段；所以 correction node 必然 fail-closed，`corrected_count=0`。

## 官方依据

- Cartographer 官方文档：<https://google-cartographer.readthedocs.io/en/latest/>
- Cartographer Configuration 官方页面：<https://google-cartographer.readthedocs.io/en/latest/configuration.html>
- ArduPilot Non-GPS / ExternalNav 官方页面：<https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html>
- ROS `nav_msgs/Odometry` 官方消息语义：<https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html>

对应关系：Cartographer 官方配置项决定 `/slam/odom` 估计如何从 scan/IMU 产生；ArduPilot ExternalNav/EKF 使用外部位姿估计来控制，而不是直接读取 Gazebo truth；ROS Odometry 的 pose 是 frame 内的位姿估计。因此如果 `/slam/odom_corrected` 没有反映真实横漂，FCU/EKF 就不会知道 Gazebo body 已经漂了。

## 当前配置对照

配置文件：

- hover：`navlab/common/slam/ros/localization/navlab_cartographer_adapter/config/navlab_cartographer_2d_hover.lua`
- real：`navlab/common/slam/ros/localization/navlab_cartographer_adapter/config/navlab_cartographer_2d_real.lua`

关键差异：

| 配置项 | hover | real | 诊断意义 |
| --- | --- | --- | --- |
| `use_odometry` | `false` | `false` | Cartographer 不吃外部里程计先验，符合不作弊要求。 |
| `use_imu_data` | `true` | `true` | 使用 IMU 姿态/外推。 |
| `provide_odom_frame` | `false` | `false` | 不让 Cartographer 发布 odom frame。 |
| `publish_frame_projected_to_2d` | `false` | `false` | 保持原始 frame 输出。 |
| `ceres_scan_matcher.translation_weight` | `1.` | `20` | hover 已经比 real 更放松平移 prior，理论上更允许 scan 牵引平移。 |
| `ceres_scan_matcher.rotation_weight` | `10.` | `20` | hover 旋转权重也较低。 |
| `use_online_correlative_scan_matching` | `true` | `true` | 已启用在线相关 scan matching。 |
| `real_time_correlative_scan_matcher.linear_search_window` | `0.50` | `0.03` | hover 搜索窗口已经显著加大，不是 3 cm 小窗口卡死。 |
| `translation_delta_cost_weight` | `1.` | `50.` | hover 已经大幅降低平移变化惩罚。 |
| `rotation_delta_cost_weight` | `10.` | `50.` | hover 已降低旋转变化惩罚。 |
| `motion_filter.max_angle_radians` | `0.2 deg` | `0.2 deg` | 角度 motion filter 一致。 |
| `num_accumulated_range_data` | `1` | `1` | 单帧 scan 更新。 |
| `POSE_GRAPH.optimize_every_n_nodes` | `0` | `0` | 关闭 pose graph 优化/loop closure，hover 主要依赖 local SLAM。 |

配置含义：hover profile 已经不是“搜索窗口太小/平移权重太高”的典型错误；它已经把 online correlative scan matching 打开，并把 linear search window 扩到 0.50 m。继续盲目加大这些参数，不是第一修复点。

## artifact 证据

artifact：`artifacts/sim/hover/20260622T015657Z`

### topic 记录

rosbag 有：

- `/scan`: 627
- `/slam/odom`: 14339
- `/slam/odom_corrected`: 14340
- `/navlab/scan_reference_drift/odom`: 627
- `/navlab/scan_reference_drift/status`: 627
- `/navlab/scan_reference_correction/status`: 179
- `/external_nav/odom`: 180
- `/gazebo/model/odometry`: 3088

rosbag 没有：

- `/trajectory_node_list`
- `/submap_list`

原因：当前 lite replay/profile 明确 drop 了这两个 topic：`docker/profiles/navlab-hover-foxglove-lite-topics.txt` 中有 `drop /submap_list` 和 `drop /trajectory_node_list`。所以本轮不能用这两个 topic 判断 Cartographer 内部 node/submap 演化；下一轮诊断必须把它们加回 raw/debug rosbag profile，而不是靠猜。

### 位置源对比

hover window 内：

| 来源 | hover drift / final magnitude | 结论 |
| --- | ---: | --- |
| `/gazebo/model/odometry` | `0.7806 m` | review-only body truth 显示机体大漂。 |
| `/slam/odom_corrected` | `0.0249 m` | 给 ExternalNav 的 SLAM-derived odom 几乎不漂。 |
| `/external_nav/odom` | `0.0234 m` | ExternalNav 基本继承 corrected odom，小漂。 |
| `/ap/v1/pose/filtered` | `0.0111 m` | FCU/EKF 认为水平位置几乎稳定。 |
| `/navlab/scan_reference_drift/odom` | `0.6544 m` | 独立 scan-reference estimator 能从 `/scan` 看出明显横漂。 |
| offline `/scan` hover estimator | `0.4665 m` | 只用 `/scan` 也能看出明显横漂。 |

这排除了“/scan 完全看不出横漂”。`/scan` 是有横漂证据的，问题是 Cartographer `/slam/odom` / corrected / ExternalNav 没有把这个横漂传给 FCU。

### correction 状态

`scan_reference_correction` 证据：

- `phase4b_consistency_ok=false`
- `phase4b_consistency_source=missing_runtime_phase4b_consistency`
- `runtime_consistency_ok=false`
- `blocked_axes=[x,y]`
- `axis_blockers.x/y=[scan_reference_runtime_saturation_ratio_high]`
- `hover_window_correction_applied_count=0`
- `corrected_count=0`
- `passthrough_count=7049`

原因：4B 的 consistency 现在只在 artifact gate 里离线计算；runtime `/navlab/scan_reference_drift/status` 没有发布 `phase4b_consistency_ok`，correction node 因此 fail-closed。这是设计断链，不是 ArduPilot 自动不听。

## 根因归类

按用户要求的三类判断：

1. Cartographer local SLAM 根本没估出横移：**成立**。至少从 `/slam/odom_corrected` 和 ExternalNav 结果看，Cartographer/SLAM 输出链路没有反映 0.78 m body drift；SLAM-derived drift 只有约 0.025 m。
2. Cartographer 估出了但 bridge/correction/ExternalNav 丢了：**目前没有证据支持**。因为 `/slam/odom_corrected` 本身已经近零，ExternalNav 与 corrected odom 高度一致；链路没有在 ExternalNav 处丢大漂移。需要下一轮加回 `/trajectory_node_list`、`/submap_list` 才能进一步证明 Cartographer 内部是否曾估出过。
3. Gazebo body drift 与 lidar scan 几何不一致：**不成立或不是主因**。scan-reference estimator 只用 `/scan` 就估出 0.47-0.65 m 级别横漂，方向一致性 review-only 也曾通过，说明 `/scan` 里有大漂移信息。

最终判断：主因是 Cartographer `/slam/odom` local SLAM 输出没有跟随 `/scan` 中可见的横漂；备用 scan-reference estimator 看到了漂移，但 correction runtime gate 因缺少 runtime-safe `phase4b_consistency_ok` 而没有把这个漂移接入 ExternalNav。

## 下一步唯一修复方向

不要改 hover mission、landing、Gazebo drift gate、Foxglove TF，也不要直接加大 correction。

下一步应该做两个最小闭环：

1. **Cartographer debug evidence**：把 `/trajectory_node_list` 和 `/submap_list` 加回 hover debug rosbag，确认 Cartographer 内部 local trajectory/submap 是否真的近零。如果内部也近零，修 Cartographer/scan matching 配置；如果内部有大漂而 `/slam/odom` 近零，修 adapter/TF/odom 发布链路。
2. **Runtime-safe 4B**：在 `scan_reference_drift.py` 中只基于 `/scan` 自己的滑窗发布 `phase4b_consistency_ok`、稳定轴、反号次数、方向连续性、饱和比例；让 `scan_reference_correction.py` 吃这个 runtime-safe 字段。Gazebo counter-drift 仍只做 review-only，不进入 runtime。

只有这两个闭环完成后，才进入真实 hover 验证；成功标准仍是顶层 `summary.ok=true`。

## 2026-06-22 更新：当前不再是 Cartographer debug 缺证据，而是 correction fallback 破坏 cap

`20260622T034706Z` 证明 scan-reference runtime 已能更早输出 correction intent，但 measurement fallback 没有限幅，导致 `max_correction_m=0.25` 时实际 `measurement_delta` 达到约 `0.65m`。因此最新失败不能再归因为“需要更大 correction”或“只差一点阈值”，而是 correction fallback 的安全边界实现错误。

修复原则：fallback 和正常 intent 路径必须共用 `max_correction_m` cap；修复后先验证 status 中 `measurement_delta_magnitude_m <= 0.25`，再评价 Gazebo drift 是否仍超阈值。

## 2026-06-22 最终验证

`artifacts/sim/hover/20260622T035140Z` 按旧 gate 顶层通过：`summary.ok=true`、`TASK_STATUS_OK`。但 Gazebo review-only drift 为 `0.3396m`，高于当前最终标准 `0.10m`，所以该 run 现在不能作为最终 hover 成功。rangefinder、takeoff ACK、hover altitude crosscheck、LAND/disarm、XY evidence 全部通过。

关键安全验证：scan-reference correction fallback 已受 `max_correction_m=0.25` 限制，summary 中 `measurement_delta_magnitude_m=0.25`，没有再出现 `0.65m` 级别绕过 cap 的输出。
