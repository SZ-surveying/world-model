# 多场景仓库结构建议

## 1. 文档定位

这份文档描述的是一个适合**多场景长期演进**的仓库结构方向。

目标不是为了当前室内场景临时拼一个目录，而是为后续这些扩展留出空间：

- 不同环境场景
- 不同传感器组合
- 不同飞控桥接方式
- 不同任务模式
- 通用能力与场景适配并行演进

## 2. 设计原则

建议仓库结构至少满足以下原则：

- 通用能力和场景特化分开
- 在线运行时和离线训练分开
- 接口定义和实现分开
- 场景配置和核心逻辑分开
- 仿真验证和真实部署都能自然挂接

## 3. 推荐结构

```text
world-model/
  README.md
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
  core/
    interfaces/
    perception/
    state_estimation/
    world_model/
    planning/
    safety/
    bridges/
    runtime/
  scenarios/
    indoor/
      configs/
      launch/
      adapters/
      missions/
    outdoor/
      configs/
      launch/
      adapters/
      missions/
  simulation/
    sitl/
    scenarios/
    replay/
  training/
    datasets/
    configs/
    models/
    scripts/
    evaluation/
  tools/
    diagnostics/
    calibration/
    bag_tools/
  artifacts/
    models/
    reports/
  tests/
    unit/
    integration/
    replay/
    sim/
```

## 4. 顶层目录职责

### `docs/`

职责：

- 放设计文档
- 区分通用设计和场景设计
- 承担长期知识沉淀

### `core/`

职责：

- 放跨场景复用的核心能力
- 尽量避免写死为某一个场景

建议子模块：

- `interfaces/`: 通用消息、数据结构、契约
- `perception/`: 通用预处理和观测抽象
- `state_estimation/`: 状态构建与估计相关能力
- `world_model/`: 世界模型推理和通用输入输出封装
- `planning/`: 任务规划、局部规划、轨迹接口
- `safety/`: 安全裁决、限幅、降级逻辑
- `bridges/`: 飞控桥接或系统桥接
- `runtime/`: 运行时编排、节点装配、监控

### `scenarios/`

职责：

- 放场景专属配置和适配逻辑
- 管理不同场景的启动方式、参数和特化约束

每个场景建议独立目录，例如：

- `indoor/`
- `outdoor/`
- `warehouse/`

一个场景目录里建议优先放这些内容：

- `configs/`: 参数和配置
- `launch/`: 启动编排
- `adapters/`: 场景特化适配逻辑
- `missions/`: 场景任务模板

### `simulation/`

职责：

- 放 SITL、仿真场景、回放资产、故障注入工具

### `training/`

职责：

- 放数据、训练、评估和模型导出流程

### `tools/`

职责：

- 放工程辅助工具

### `artifacts/`

职责：

- 放导出模型、评估报告等产物

### `tests/`

职责：

- 放单元测试、集成测试、回放测试和仿真测试

## 5. 为什么要把 `core/` 和 `scenarios/` 分开

这是多场景仓库最关键的边界之一。

### `core/` 解决的问题

- 哪些能力应该被多个场景共享
- 哪些接口应该长期稳定
- 哪些模块不应写死某个具体任务假设

### `scenarios/` 解决的问题

- 某个场景用哪些传感器
- 某个场景的参数怎么配
- 某个场景如何启动和编排
- 某个场景有哪些特殊约束

如果这两部分不分开，后续场景一多，仓库会很快演变成大量 if/else 和分散配置。

## 6. 与当前室内场景的映射

当前正在推进的室内方案，可以先映射成下面这样：

- `core/state_estimation/`: 通用状态估计接口与状态表示
- `core/world_model/`: 通用世界模型输入输出契约
- `core/planning/`: 通用规划接口
- `core/safety/`: 通用安全裁决逻辑
- `core/bridges/`: 通用飞控桥接口
- `scenarios/indoor/configs/`: 室内参数
- `scenarios/indoor/launch/`: 室内启动方式
- `scenarios/indoor/adapters/`: 室内特化接入逻辑

也就是说，当前室内方案应该成为“第一个场景实例”，而不是把整个仓库都写成“只支持室内”。

## 7. 当前迁移建议

考虑到仓库目前还在早期阶段，不建议一次性大改代码目录。更稳妥的做法是：

1. 先把文档体系改成多场景结构
2. 再把接口和模块边界在文档中固定下来
3. 后续新增代码时优先按 `core/` 和 `scenarios/` 的边界落位
4. 等代码规模足够大时，再进行代码目录迁移

这样可以避免在需求还没稳定前做过重的代码搬迁。

## 8. 结构结论

对这个仓库来说，最重要的不是马上把所有目录都建满，而是先明确一个长期稳定的方向：

- 文档按“通用 / 场景”分层
- 代码按“核心能力 / 场景适配”分层
- 训练、仿真、运行时、工具职责分离

只要这个边界先立住，后续无论新增室外场景、混合场景，还是新增不同飞控和传感器方案，仓库都不容易失控。

## 9. 相关文档

- 顶层分层见 `docs/general/architecture.md`
- 接口设计见 `docs/general/ros2_interfaces.md`
- 安全设计见 `docs/general/safety_and_validation.md`
- 当前室内场景入口见 `docs/scenarios/indoor/README.md`
