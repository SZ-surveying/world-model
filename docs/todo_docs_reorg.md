# 文档整理 TODO

## 1. 背景

当前仓库里的 `docs/` 主要还是按“单一方案”在写，已经有一部分内容适合保留，但后续仓库会面向**多个场景**，不应该只围绕当前室内方案组织。

你当前最明确的场景是：

- 室内
- 无 GPS
- 激光雷达 / SLAM
- ArduPilot

但后续可能扩展到：

- 室外有 GPS 场景
- 半室内半室外场景
- 不同传感器组合场景
- 不同飞控或不同任务模式场景

所以文档和仓库结构都需要从“一套单场景说明”升级成“顶层通用设计 + 场景化分目录”的形式。

## 2. 当前已完成的内容

这次对话里，已经做了两件事：

1. 新增了一份室内场景设计评审文档：
   - `docs/indoor_ardupilot_slam_design.md`
2. 更新了文档索引：
   - `docs/README.md`

但这还只是**补充文档**，还没有真正完成你要的“多场景归类重构”。

## 3. 本轮先做什么

这一轮先不直接大范围改文件，先把后续动作拆成可执行 TODO。

目标是先回答三个问题：

1. `docs/` 应该怎么分层？
2. 哪些文档属于顶层通用设计？
3. 哪些文档属于室内场景专属？

## 4. 计划中的目标目录结构

建议把文档改成下面这种结构：

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
```

后续如果新增场景，可以继续扩展：

```text
docs/scenarios/outdoor/
docs/scenarios/hybrid/
docs/scenarios/warehouse/
```

## 5. 顶层文档和场景文档的边界

### 顶层通用文档 `docs/general/`

这里放**不依赖单一场景**的内容，例如：

- 仓库总体目标
- 多场景统一架构原则
- 世界模型、Planner、安全层、飞控桥的职责边界
- 通用 ROS2 接口设计原则
- 通用安全验证原则
- 仓库目录组织方式

这些文档不应该写死“必须是室内 SLAM + ArduPilot”。

### 场景文档 `docs/scenarios/<scene>/`

这里放**某个具体场景的特化方案**，例如室内场景：

- 为什么室内要用 ExternalNav，而不是依赖 GPS
- 这一场景的传感器组合
- 这一场景的坐标系和 SLAM 约束
- 这一场景的 MVP 路线
- 这一场景与通用架构之间的映射关系

## 6. 接下来要做的具体步骤

### Step 1：重写 `docs/README.md`

要做的事：

- 把 `docs/README.md` 改成总导航页
- 明确区分 `general/` 和 `scenarios/`
- 标注哪些文档是通用文档，哪些是室内场景文档

完成标准：

- 新人打开 `docs/README.md` 就能知道先看哪里
- 状态：已完成

### Step 2：新建 `docs/general/` 和 `docs/scenarios/`

要做的事：

- 新建目录
- 加各自的 `README.md`
- 说明目录用途和边界

完成标准：

- 文档结构一眼能看懂
- 状态：已完成

### Step 3：迁移顶层通用文档

计划迁移：

- `docs/architecture.md` -> `docs/general/architecture.md`
- `docs/repository_structure.md` -> `docs/general/repository_structure.md`
- `docs/ros2_interfaces.md` -> `docs/general/ros2_interfaces.md`
- `docs/safety_and_validation.md` -> `docs/general/safety_and_validation.md`

处理原则：

- 保留仍然成立的原则
- 删除只适用于当前单一室内场景的表述
- 改成支持多场景扩展的写法
- 状态：进行中（已完成 `architecture.md` 和 `repository_structure.md` 的新版本骨架）
- 进展补充：已完成 `ros2_interfaces.md` 和 `safety_and_validation.md` 的新版本骨架

### Step 4：整理室内场景文档

计划迁移：

- `docs/indoor_ardupilot_slam_design.md` -> `docs/scenarios/indoor/ardupilot_slam_design.md`
- `docs/mvp_plan.md` -> `docs/scenarios/indoor/mvp_plan.md`

并补充：

- `docs/scenarios/indoor/README.md`
- `docs/scenarios/indoor/architecture.md`

处理原则：

- 明确这是“当前在做的场景”
- 强调室内无 GPS、SLAM、ExternalNav、ArduPilot 的特化约束
- 状态：进行中（已完成 `architecture.md` 和 `mvp_plan.md` 的新版本骨架）
- 进展补充：已完成 `ardupilot_slam_design.md` 迁入新目录，并补充了室内目录入口说明

### Step 5：同步更新交叉引用

要做的事：

- 修正文档中的路径引用
- 保证所有 README 的链接都正确
- 检查是否还有旧路径残留

完成标准：

- 文档导航无断链

### Step 6：补一份新的仓库结构建议

重点不是只讲 `docs/`，而是把整个仓库做成“多场景可扩展”的结构。

这一版会重点区分：

- 通用运行时模块
- 场景适配层
- 通用接口
- 场景配置
- 仿真与验证资产

## 7. 计划中的仓库结构方向

这个不是本步马上改代码，而是作为下一步文档设计目标：

```text
world-model/
  docs/
    general/
    scenarios/
  core/
    interfaces/
    planning/
    safety/
    world_model/
    bridges/
  scenarios/
    indoor/
      configs/
      launch/
      adapters/
    outdoor/
      configs/
      launch/
      adapters/
  simulation/
  training/
  tools/
```

核心思路是：

- `core/` 放通用能力
- `scenarios/` 放场景特化配置和适配逻辑
- `docs/general/` 放通用设计
- `docs/scenarios/` 放场景设计

## 8. 执行顺序

建议严格按下面顺序做，不要一次性全改：

1. 先确认文档目录结构
2. 再迁移通用文档
3. 再迁移室内场景文档
4. 再统一更新引用
5. 最后再写新的仓库结构建议

## 9. 本 TODO 完成后的预期结果

完成后，文档应该达到这几个效果：

- 顶层设计不再绑定单一室内场景
- 当前室内方案有独立目录，避免和后续场景混在一起
- 后续增加新场景时，只需要新增 `docs/scenarios/<name>/`
- 仓库结构设计也能自然支持“通用能力 + 场景适配”

## 10. 当前状态

- [x] 补了一份室内场景设计评审文档
- [x] 补了室内方案入口到 `docs/README.md`
- [x] 完成第 1 步：把 `docs/README.md` 改成按“通用设计 / 场景设计”理解的总导航页
- [x] 完成第 2 步：创建 `docs/general/`、`docs/scenarios/`、`docs/scenarios/indoor/` 的 README 骨架
- [x] 完成第 3 步的一部分：新增 `docs/general/architecture.md`
- [x] 完成第 3 步的一部分：新增 `docs/general/repository_structure.md`
- [x] 完成第 3 步的一部分：新增 `docs/general/ros2_interfaces.md`
- [x] 完成第 3 步的一部分：新增 `docs/general/safety_and_validation.md`
- [x] 完成第 4 步的一部分：新增 `docs/scenarios/indoor/architecture.md`
- [x] 完成第 4 步的一部分：新增 `docs/scenarios/indoor/mvp_plan.md`
- [x] 完成第 4 步的一部分：新增 `docs/scenarios/indoor/ardupilot_slam_design.md`
- [x] 更新 `docs/README.md`，将新结构设为主入口
- [x] 删除旧的平铺文档，统一收敛到新目录结构
- [x] 为通用文档和场景文档补充交叉引用，减少重复描述
- [x] 新增 GitHub Actions -> 飞书文档同步的 CI 设计文档
- [x] 完成文档分层重构
- [ ] 完成多场景导向的通用文档重写
- [ ] 完成室内场景目录化整理
- [ ] 完成多场景仓库结构建议
