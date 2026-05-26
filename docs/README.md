# 无人机世界模型项目文档

这套文档现在以“通用设计 + 场景设计”的新结构作为主入口，面向一个后续可扩展到**多个场景**的无人机世界模型仓库。

当前正在重点推进的只是其中一个场景：

- 室内
- 无 GPS
- 激光雷达 / SLAM
- ArduPilot

但顶层设计本身不应该被这个场景完全绑定。后续如果扩展到室外、半室内半室外、不同传感器组合、不同任务模式，文档结构也应该能自然扩展。

当前文档已经按两层组织：

- `通用设计`：跨场景共用的架构、接口、安全、仓库结构原则
- `场景设计`：针对某个具体场景的特化方案、MVP 路线和约束

## 主导航

### 通用设计 `docs/general/`

- `docs/general/README.md`: 通用设计目录说明和边界
- `docs/general/architecture.md`: 多场景通用架构、分层、控制边界
- `docs/general/repository_structure.md`: 多场景仓库结构建议
- `docs/general/ros2_interfaces.md`: 多场景通用 ROS2 接口设计
- `docs/general/safety_and_validation.md`: 多场景通用安全与验证原则
- `docs/general/feishu_sync_ci.md`: GitHub Actions 同步 docs 到飞书的 CI 设计

### 场景设计 `docs/scenarios/`

- `docs/scenarios/README.md`: 场景设计目录说明
- `docs/scenarios/indoor/README.md`: 当前室内场景入口
- `docs/scenarios/indoor/architecture.md`: 室内无 GPS 场景架构
- `docs/scenarios/indoor/mvp_plan.md`: 室内场景 MVP 路线图
- `docs/scenarios/indoor/ardupilot_slam_design.md`: 室内 SLAM + ExternalNav + ArduPilot 详细设计评审

### 当前仿真路径 `docs/sim/`

- `docs/sim/README.md`: 当前只关注 fake `/scan` 和 Gazebo lidar 的两阶段仿真设计，明确排除旧 `ros2_ws/`
- `docs/sim/scan_contract.md`: 真实 `x3` 驱动对外暴露的 `/scan` 契约，fake 和 Gazebo 都必须对齐
- `docs/sim/todo.md`: 当前仿真路线的 P0 / P1 / P2 任务拆解和验收标准

## 当前推荐阅读顺序

如果你想先看顶层通用设计，建议按这个顺序：

1. `docs/general/architecture.md`
2. `docs/general/repository_structure.md`
3. `docs/general/ros2_interfaces.md`
4. `docs/general/safety_and_validation.md`
5. `docs/general/feishu_sync_ci.md`

如果你想直接看当前室内场景，建议按这个顺序：

1. `docs/scenarios/indoor/README.md`
2. `docs/scenarios/indoor/architecture.md`
3. `docs/scenarios/indoor/mvp_plan.md`
4. `docs/scenarios/indoor/ardupilot_slam_design.md`

如果你想直接推进当前避障仿真，先看：

1. `docs/sim/README.md`
2. `docs/sim/scan_contract.md`
3. `docs/sim/todo.md`

## 当前目录结构

```text
docs/
  README.md
  todo_docs_reorg.md
  general/
    README.md
    architecture.md
    repository_structure.md
    ros2_interfaces.md
    safety_and_validation.md
  scenarios/
    README.md
    indoor/
      README.md
      architecture.md
      mvp_plan.md
      ardupilot_slam_design.md
  sim/
    README.md
    scan_contract.md
    todo.md
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
