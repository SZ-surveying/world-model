# a3e0f7a vs 当前 hover 流程总体比较

日期：2026-06-17  
目的：解释为什么历史 commit `a3e0f7a` 附近虽然使用了 Gazebo odom / truth-like prior，但 Foxglove 中 robot、scan、map 看起来一致；以及当前 hover 修复链路为什么更严格，却更容易出现 TF / LiDAR 贴墙 / robot-map 错位问题。

## 一句话结论

历史链路是“显示和定位都跟着同一套 Gazebo/Cartographer odom 坐标链走”，所以 Foxglove 看起来很顺；当前链路是“尽量不让 Gazebo truth 进入控制验收”，但 hover 特例把控制 odom、全局 TF、Foxglove 可视化混在了一起，导致后来用 scan-match 贴墙时把 `/tf map -> base_link` 一起挪了，Foxglove 就出现 robot 和 map 不在一起。

当前修复方向不应该回到旧的 truth-as-control，而应该拆清楚两条链：

- 控制链：`/slam/odom -> /external_nav/odom -> FCU -> hover/landing`，不能用 Gazebo truth 或可视化 scan-match 污染。
- 可视化链：Foxglove 可以有 replay-only 的对齐层，但不能改全局 `/tf map -> base_link`，否则 robot 本体也会被挪。

## 端到端 hover 流程对比

| 环节 | a3e0f7a 附近旧流程 | 当前流程 | 差异/风险 |
| --- | --- | --- | --- |
| Gazebo/SITL 启动 | 官方 maze + Iris，Gazebo odom 被桥接出来 | 仍然是官方 maze + Iris，但关闭 synthetic GPS fallback，增加 rangefinder / ExternalNav 证据 | 当前更接近真实约束，但不能再依赖假 GPS 或 truth 直接过关 |
| 2D LiDAR | `/scan` 给 Cartographer；frame 通过 `base_link -> laser_frame/base_scan` 静态 TF 接入 | `/scan` 仍录制和显示；增加 X2 virtual serial / cloud projection，避免无真实 scan 时退回圆形静态 fallback | 当前 sensor 链更真实，但 scan 与 map 的对齐不再天然由 Cartographer 保证 |
| Cartographer | 启动 Cartographer backend；Lua 中 `provide_odom_frame=true`、`use_odometry=true`，使用 `/odometry` 作为 odom prior | hover 特例里 `launch_cartographer_backend=false`，不用完整 Cartographer 输出作为 hover 控制输入 | 旧流程“作弊但稳”：Gazebo odom prior 让 map/robot/scan 一致；当前避免作弊，但少了 Cartographer 统一坐标链 |
| canonical odom | adapter 从全局 `/tf` 取 `odom -> base_link`，发布 `/odom` | 当前标准是 `/slam/odom`；hover 特例由 `hover_cartographer_odom_prior.py` 生成 `/slam/odom` | 当前 topic contract 更严格，但 hover 的 `/slam/odom` 是特例生成，不是真 Cartographer 地图定位 |
| ExternalNav 输入 | 配置上 external nav 消费 `/odom`，而 `/odom` 来自 Cartographer/TF-backed odom | ExternalNav 必须消费 `/slam/odom`，并且 gate 拒绝 Gazebo truth / `/odometry` 作为输入 | 当前控制验收更正确；不能为了 Foxglove 好看把 truth 或 scan-match 塞回 ExternalNav |
| 高度证据 | 旧 gate 对 rangefinder / external_nav 高度交叉验证没现在严格 | 当前要求 takeoff ACK、rangefinder、external_nav、高度交叉证据，不能只靠 FCU local-z 假成功 | 当前修过的 hover gate 是正确方向，不应回退 |
| hover 状态机 | 更容易因为坐标链顺而看起来成功 | 已修为不能只靠 FCU local-z 进入 hold；必须 ACK 或真实高度证据 | 当前更严格，但也暴露了高度/模式/ExternalNav 的真实问题 |
| landing | 旧流程主要看任务完成 | 当前增加慢速 descent、bounce、touchdown、disarm 等检查 | 当前更真实；不应为了通过降低标准 |
| Foxglove replay | `/map`、`/tf`、`/scan` 基本来自同一套 Cartographer/Gazebo odom 链，所以视觉一致 | hover replay 保留 `/tf`、`/map`、`/scan`、`/slam/odom`，并额外 overlay `/navlab/official_maze/map` | 当前如果 `/tf map -> base_link` 被 scan-match 改，robot 也会被拖走；这是截图中 robot/map 错位的核心 |

## 旧流程为什么 Foxglove 表现正常

`a3e0f7a` 附近的核心特点是：Foxglove 看到的 map、robot、scan 大体都服从同一套坐标链。

关键点：

- Cartographer 配置使用 `map_frame="map"`、`odom_frame="odom"`、`published_frame="base_link"`。
- `provide_odom_frame=true`，Cartographer 负责提供 `map/odom/base_link` 关系。
- `use_odometry=true`，Cartographer 使用 `/odometry` 作为 odom prior。
- launch 中把 Cartographer 的 `/odom` remap 到 `/odometry`。
- adapter 订阅全局 `/tf`，提取 `odom -> base_link`，发布 canonical odom。

所以旧流程虽然有明显 truth-like prior：

```text
Gazebo /odometry -> Cartographer -> /tf map/odom/base_link -> adapter -> /odom -> ExternalNav/Foxglove
```

但对 Foxglove 来说这是一条自洽链：

```text
/map 和 /tf 同源
/scan 挂在 base_link/base_scan 下
robot 本体跟 TF 走
LiDAR 点云/scan 也跟同一个 TF 走
```

因此它看起来不会出现“为了让 LiDAR 贴墙，robot 被水平挪走”的问题。

## 当前流程为什么更容易改坏 Foxglove

当前 hover 为了避免旧的 truth-as-control，做了几件正确但会增加复杂度的事：

- 不允许 Gazebo truth 或 `/odometry` 直接作为 ExternalNav / 控制输入。
- canonical SLAM odom 改成 `/slam/odom`。
- hover gate 要求 rangefinder / external_nav / ACK 等证据，不允许只靠 FCU local-z 假成功。
- hover 特例关闭完整 Cartographer backend，用 `hover_cartographer_odom_prior.py` 维持 `/slam/odom` 和 `/tf`。

问题出在：`hover_cartographer_odom_prior.py` 后来被加入了 lightweight scan-match 贴墙逻辑，并且一度把 scan-match 的 x/y 写进全局 `/tf map -> base_link`。

这条 TF 同时被 Foxglove 用来放 robot 本体和 scan：

```text
/scan --base_scan/base_link--> /tf map->base_link --> map
robot model --------same /tf--------> map
```

所以如果为了让 scan 点贴墙而移动 `/tf map -> base_link`，Foxglove 看到的不是“scan 独立贴墙”，而是“整个机器人也被移走”。这就是用户截图里 robot 和 map 不在一起的根因。

## 旧流程的“作弊”和当前流程的“正确约束”

旧流程的优点：

- Foxglove 很稳定。
- robot/map/scan 显示自然对齐。
- Cartographer 有 Gazebo odom prior，定位不会乱飘。

旧流程的问题：

- `/odometry` 带有 Gazebo truth / simulator prior 色彩。
- 通过 hover 不等于证明真实 SLAM + ExternalNav 能独立支撑悬停。
- 如果把这套链路当成控制验收，就会产生假成功风险。

当前流程的优点：

- synthetic GPS fallback 已关闭。
- ExternalNav 输入被约束为 `/slam/odom`。
- hover 成功必须有高度和 rangefinder/external_nav 交叉证据。
- landing 也检查慢速下降、touchdown、disarm、bounce。

当前流程的问题：

- hover 特例不再有完整 Cartographer 统一坐标链。
- `/slam/odom`、`/tf`、Foxglove overlay 之间需要明确 ownership。
- 任何为了“视觉贴墙”而改全局 `/tf map -> base_link` 的逻辑，都会把 robot 本体也挪走。

## 当前应该保留的原则

1. 不回退到旧的 truth-as-control。
   - Gazebo `/odometry` 可以用于诊断或 replay-only 对照，但不能进入 hover gate、ExternalNav 控制输入或成功判定。

2. 全局 `/tf map -> base_link` 表示机器人本体位姿。
   - 不能为了 LiDAR 贴墙修改这条 TF。
   - 否则 Foxglove 中 robot、scan、map 会一起被扭曲。

3. `/slam/odom` 是控制链输入。
   - 不能被 lightweight scan-match 或 Foxglove-only 修正污染。
   - 如果 `/slam/odom` 是 hover 特例生成的，也必须明确它不是 Gazebo truth。

4. Foxglove 贴墙可以单独做，但必须是 replay-only。
   - 例如新增 `/scan_map_aligned` 或独立 `map_review` frame。
   - 这个层只服务审查显示，不反馈到 `/slam/odom`、ExternalNav、FCU 或 gate。

5. hover 成功标准不能降低。
   - 仍然要 takeoff ACK 或真实高度证据。
   - 仍然要 rangefinder/external_nav/FCU 高度交叉验证。
   - 仍然要 landing 慢速、无回弹、touchdown、disarm。

## 判断当前截图问题的具体结论

用户最新截图中的现象：

- robot 和 map 不在一起。
- LiDAR 为了贴墙导致机器人看起来水平偏移。
- hover 本应主要是垂直上下移动，但 Foxglove 中看起来水平漂移明显。

对应原因：

- 上传到 Foxglove 的 run `20260617T063649Z` 仍是旧的污染版本。
- 该 run 的 `hover_cartographer_odom_prior.py` 还没有 `enable_scan_match_tf=false`。
- 日志显示 scan-match 持续运行，`estimated_xy=(1.776, 3.116)`，说明 `/tf map -> base_link` 被 scan-match 结果改过。
- 因为 Foxglove 用同一条 `/tf` 放 robot 和 scan，所以 robot/map 错位不是单纯 Foxglove UI 问题，而是 replay 中 TF ownership 被污染。

## 建议的后续修复顺序

### 第一阶段：恢复自洽显示，不追求贴墙

目标：先证明 robot、map、scan 的基础 TF 不再互相打架。

做法：

- 保持 `enable_scan_match_tf=false`。
- 重新跑 hover。
- 重新 build/upload Foxglove replay。
- 检查 Foxglove：robot 不应因为 scan 贴墙被水平挪动。

验收：

- hover/landing summary 仍为 ok。
- `/tf map -> base_link` 水平漂移接近 hover 实际漂移。
- Foxglove 中 robot 和 map 不再明显错位。

### 第二阶段：如果 LiDAR 仍不贴墙，做 replay-only 对齐

目标：让 LiDAR 审查层贴墙，但不影响机器人本体和控制链。

做法：

- 新增独立 topic，例如 `/scan_map_aligned`。
- 或新增独立 frame，例如 `map_review` / `base_scan_review`。
- 明确该 topic/frame 只出现在 Foxglove lite replay，不进入 runtime 控制。

验收：

- `/slam/odom` 不变。
- `/external_nav/odom` 不变。
- `/tf map -> base_link` 不被 scan-match 改。
- Foxglove 可以选择看原始 `/scan` 或 aligned scan。

### 第三阶段：再考虑恢复更接近历史的 Cartographer 可视化链

目标：借鉴 a3e0f7a 的自洽显示优点，但不把 Gazebo truth 带回控制。

可选方案：

- 只在 replay/diagnostic 中运行“历史式 Cartographer+odom prior”链，输出 review-only map/TF。
- 明确标记为 visualization_only / diagnostic_only。
- gate 和 ExternalNav 不消费这条链。

风险：

- 如果命名混乱，很容易又被误用成控制输入。
- 必须用 topic 名和 summary truth_boundary 写清楚。

## 最重要的保护线

不要再做这件事：

```text
用 LiDAR scan-match 结果直接修改全局 /tf map -> base_link
```

它看起来能让 LiDAR 贴墙，但实际会把 robot 本体也挪走，是当前 Foxglove 错位的直接原因。

正确做法是：

```text
控制链 /slam/odom 和 /tf map->base_link 保持机器人本体位姿
Foxglove 贴墙修正走单独 replay-only topic/frame
```

