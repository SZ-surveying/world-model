# 通用设计文档

这个目录放的是**跨场景共用**的设计文档。

这里的内容不应该写死为某一个具体场景，比如不能默认只针对：

- 室内
- 无 GPS
- 激光雷达 SLAM
- ArduPilot

这些文档要回答的是更顶层的问题，例如：

- 世界模型系统的通用分层是什么
- 通用的 ROS2 接口边界怎么设计
- 通用的安全裁决原则是什么
- 仓库结构如何支持多个场景长期共存

## 这里会逐步收纳的文档

- `architecture.md`
- `repository_structure.md`
- `ros2_interfaces.md`
- `safety_and_validation.md`
- `feishu_sync_ci.md`

## 推荐阅读顺序

1. `docs/general/architecture.md`
2. `docs/general/repository_structure.md`
3. `docs/general/ros2_interfaces.md`
4. `docs/general/safety_and_validation.md`
5. `docs/general/feishu_sync_ci.md`

## 相互引用关系

- 先看 `docs/general/architecture.md`，理解系统分层和职责边界
- 再看 `docs/general/repository_structure.md`，理解这些能力如何落到仓库结构
- 再看 `docs/general/ros2_interfaces.md`，理解模块之间如何通过接口连接
- 再看 `docs/general/safety_and_validation.md`，理解通用安全边界
- 如果要把文档体系发布到外部协作平台，再看 `docs/general/feishu_sync_ci.md`

## 编写原则

- 优先写职责边界，不写死某个场景实现
- 优先抽象共性，不提前绑定单一硬件组合
- 如果某条设计只适用于特定场景，应下沉到 `docs/scenarios/`
