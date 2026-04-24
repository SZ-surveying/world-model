# 室内场景文档

这个目录对应当前正在推进的主场景：

- 室内飞行
- 无 GPS
- 以激光雷达 / SLAM 为核心定位来源
- 与 ArduPilot 对接

## 这个场景为什么需要单独成目录

因为室内场景有一组非常明确的特化约束：

- 常规 GPS 不可用
- 需要依赖 SLAM 或其他 External Navigation 能力给飞控提供位置和速度估计
- 坐标系转换、定位连续性、回环跳变处理会成为关键问题
- 避障和局部规划通常更依赖局部地图和实时风险评估

这些内容不适合直接写死在通用顶层文档里，所以单独整理到场景目录。

## 计划收纳的文档

这个目录后续会逐步包含：

- `architecture.md`
- `mvp_plan.md`
- `ardupilot_slam_design.md`

## 当前目录入口

当前已经整理到这个目录下的文档有：

- `docs/scenarios/indoor/architecture.md`: 室内无 GPS 场景的系统拆分、数据流和关键闭环
- `docs/scenarios/indoor/mvp_plan.md`: 当前室内场景的阶段路线、MVP 范围和开发优先级
- `docs/scenarios/indoor/ardupilot_slam_design.md`: 围绕 SLAM、ExternalNav、ArduPilot 和世界模型的详细设计评审

## 推荐阅读顺序

1. `docs/scenarios/indoor/architecture.md`
2. `docs/scenarios/indoor/mvp_plan.md`
3. `docs/scenarios/indoor/ardupilot_slam_design.md`

## 与通用文档的关系

- 通用分层先看 `docs/general/architecture.md`
- 通用接口先看 `docs/general/ros2_interfaces.md`
- 通用安全先看 `docs/general/safety_and_validation.md`
- 当前目录描述的是这些通用原则在室内无 GPS 场景下的具体落地
