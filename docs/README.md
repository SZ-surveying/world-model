# 无人机世界模型项目文档

这个目录只保留当前主线需要的文档：

- Gazebo 最小仿真运动闭环
- 室内无 GPS 激光 SLAM + ArduPilot SITL 阶段拆解
- 阶段 1 `ExternalNav` 设计和 TODO

旧的泛化架构、重复设计草案和过期 runbook 已经删除，避免后续填文档或执行任务时被旧口径干扰。

## 主导航

### 室内无 GPS 场景

- `docs/scenarios/indoor/task_breakdown_progress_tracking.md`: 当前任务拆解和进度跟踪
- `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`: 阶段 1 SITL ExternalNav 设计
- `docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`: 阶段 1 P0 / P1 / P2 TODO
- `docs/scenarios/indoor/stage1_5_companion_sitl_gazebo_design.md`: 阶段 1.5 无 GPS companion + SITL + Gazebo 设计
- `docs/scenarios/indoor/stage1_5_companion_sitl_gazebo_todo.md`: 阶段 1.5 phase TODO、rosbag 和 Foxglove 回放验收

### 当前仿真路径

- `docs/sim/README.md`: 当前 Gazebo 仿真主线、`/scan -> /scan_features` 路径和 `0.5 m` 最小净空停止规则，明确排除旧 `ros2_ws/`
- `docs/sim/todo.md`: 当前仿真路线的已完成 / 未完成清单和验收标准
- `docs/sim/examples/*.yaml`: `auto` 模式 mission 输入样例

## 当前推荐阅读顺序

1. `docs/sim/README.md`
2. `docs/sim/todo.md`
3. `docs/scenarios/indoor/task_breakdown_progress_tracking.md`
4. `docs/scenarios/indoor/stage1_sitl_external_nav_design.md`
5. `docs/scenarios/indoor/stage1_sitl_external_nav_todo.md`
6. `docs/scenarios/indoor/stage1_5_companion_sitl_gazebo_design.md`
7. `docs/scenarios/indoor/stage1_5_companion_sitl_gazebo_todo.md`

## 当前目录结构

```text
docs/
  README.md
  scenarios/
    indoor/
      task_breakdown_progress_tracking.md
      stage1_sitl_external_nav_design.md
      stage1_sitl_external_nav_todo.md
      stage1_5_companion_sitl_gazebo_design.md
      stage1_5_companion_sitl_gazebo_todo.md
  sim/
    README.md
    todo.md
    examples/
      blocked_by_stop_guard.yaml
      straight_line_demo.yaml
```

## 当前约束

以下约束仍然成立，并会保留到新的通用文档体系里：

- 机上的计算盒子只负责在线推理、规划、安全裁决和飞控桥接。
- 数据整理、训练、离线评估和模型导出在源平台完成。
- 机上部署的是已冻结的模型产物，而不是训练环境。
- 世界模型不直接绕过规划和安全层控制飞控。

## 设计原则

- 先区分通用能力和场景特化能力。
- 先搭文档结构，再逐步迁移内容。
- 先仿真，后实机。
- 先高层控制，后更深层控制。
- 先安全边界，后模型能力。
