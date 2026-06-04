# 无人机世界模型项目文档

这个目录只保留当前主线需要的文档。当前重点不是旧的接口验收，而是：

- Gazebo 8 字形室内世界
- X2 雷达协议级仿真
- 无 GPS 真实 SLAM feedback
- ArduPilot SITL ExternalNav
- FCU 真实悬停
- MCAP rosbag 和 Foxglove 回放

## 主导航

### 室内无 GPS 主线

- `docs/scenarios/indoor/navlab_slam_hover_design.md`: 当前无 GPS SLAM feedback 悬停阶段设计
- `docs/scenarios/indoor/navlab_slam_hover_todo.md`: 当前 phase TODO 和验收标准
- `docs/scenarios/indoor/navlab_runtime_workflow.md`: 批处理 acceptance、topic、summary、rosbag 和 Foxglove 回放口径

### 仿真和传感器

- `docs/sim/README.md`: Gazebo 仿真和传感器主线说明
- `docs/sim/todo.md`: 仿真路线 TODO
- `docs/sim/x2_lidar_simulation_design.md`: X2 虚拟串口协议级仿真设计
- `docs/sim/x2_lidar_protocol_todo.md`: X2 协议级仿真 TODO 和验收标准
- `docs/sim/examples/*.yaml`: 历史 mission 输入样例，仅作参考

### 工程重构

- `docs/general/lab_env_service_refactor_todo.md`: Python 服务边界、目录重构和测试拆分记录

## 推荐阅读顺序

1. `docs/scenarios/indoor/navlab_slam_hover_design.md`
2. `docs/scenarios/indoor/navlab_runtime_workflow.md`
3. `docs/scenarios/indoor/navlab_slam_hover_todo.md`
4. `docs/sim/x2_lidar_simulation_design.md`
5. `docs/sim/x2_lidar_protocol_todo.md`
6. `docs/general/lab_env_service_refactor_todo.md`

## 当前目录结构

```text
docs/
  README.md
  decisions.md
  general/
    lab_env_service_refactor_todo.md
  scenarios/
    indoor/
      navlab_slam_hover_design.md
      navlab_slam_hover_todo.md
      navlab_runtime_workflow.md
  sim/
    README.md
    todo.md
    x2_lidar_simulation_design.md
    x2_lidar_protocol_todo.md
    examples/
      blocked_by_stop_guard.yaml
      straight_line_demo.yaml
```

## 当前约束

- Gazebo 只负责世界、物理和传感器。
- SITL 等价于真实飞控。
- companion 等价于机载计算盒子。
- SLAM 必须消费 `/scan + /imu/data`，不能消费 Gazebo truth 或 FCU fused local position。
- ExternalNav 验收必须来自 SLAM `/odom`。
- Gazebo truth 只能用于诊断和误差对照。
- 上游代码不能直接 `set_pose` 移动 Gazebo 无人机。
- 每次 acceptance 必须输出 MCAP rosbag，方便 Foxglove 回放。
