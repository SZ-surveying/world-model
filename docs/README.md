# 无人机世界模型项目文档

这个目录只保留当前主线需要的文档。当前重点不是旧的接口验收，也不是直接跳到 hover 演示，而是先按官方基线建立真实闭环标准：

- 官方 ArduPilot ROS2/Gazebo/Cartographer baseline
- NavLab world/model/sensor 接入官方或官方等价机制
- 无 GPS 真实 SLAM feedback
- FCU 状态机、唯一 setpoint owner 和真实悬停
- MCAP rosbag 和 Foxglove 回放验收

## 主导航

### 室内无 GPS 主线

- `docs/scenarios/indoor/navlab_master_roadmap.md`: 总 roadmap，定义 P0-P9 顺序和完成标准
- `docs/scenarios/indoor/navlab_p0_official_baseline_design.md`: P0 官方基线验收设计
- `docs/scenarios/indoor/todos/P0_official_baseline_todo.md`: P0 TODO 和验收标准
- `docs/scenarios/indoor/navlab_p1_official_maze_x2_design.md`: P1 官方 maze + NavLab X2 雷达接入设计
- `docs/scenarios/indoor/todos/P1_official_maze_x2_todo.md`: P1 TODO 和验收标准
- `docs/scenarios/indoor/navlab_p2_rangefinder_imu_design.md`: P2 下视 rangefinder 与 IMU 机制验收设计
- `docs/scenarios/indoor/todos/P2_rangefinder_imu_todo.md`: P2 TODO 和验收标准
- `docs/scenarios/indoor/navlab_p3_slam_backend_quality_design.md`: P3 SLAM backend 质量验收设计
- `docs/scenarios/indoor/todos/P3_slam_backend_quality_todo.md`: P3 TODO 和验收标准
- `docs/scenarios/indoor/navlab_p4_fcu_state_machine_design.md`: P4 FCU 状态机和唯一控制器设计
- `docs/scenarios/indoor/todos/P4_fcu_state_machine_todo.md`: P4 TODO 和验收标准
- `docs/scenarios/indoor/navlab_p5_frame_contract_design.md`: P5 Frame contract 自动验收设计
- `docs/scenarios/indoor/todos/P5_frame_contract_todo.md`: P5 TODO 和验收标准
- `docs/scenarios/indoor/navlab_p6_slam_hover_gate_design.md`: P6 真实 SLAM hover gate 设计
- `docs/scenarios/indoor/todos/P6_slam_hover_gate_todo.md`: P6 TODO 和验收标准
- `docs/scenarios/indoor/navlab_ardupilot_ros2_official_alignment.md`: 当前系统与官方路线的对齐审计
- `docs/scenarios/indoor/navlab_reference_projects_analysis.md`: 四个参考仓库的综合分析和改进建议
- `docs/scenarios/indoor/navlab_cartographer_real_machine_tuning.md`: Cartographer 真机调参记录口径

### 仿真和传感器

- `docs/sim/README.md`: Gazebo 仿真和传感器主线说明
- `docs/sim/todo.md`: 仿真路线 TODO
- `docs/sim/x2_lidar_simulation_design.md`: X2 虚拟串口协议级仿真设计
- `docs/sim/x2_lidar_protocol_todo.md`: X2 协议级仿真 TODO 和验收标准
- `docs/sim/examples/*.yaml`: 历史 mission 输入样例，仅作参考

### 工程重构

- `docs/general/lab_env_service_refactor_todo.md`: Python 服务边界、目录重构和测试拆分记录

## 推荐阅读顺序

1. `docs/scenarios/indoor/navlab_master_roadmap.md`
2. `docs/scenarios/indoor/navlab_p0_official_baseline_design.md`
3. `docs/scenarios/indoor/todos/P0_official_baseline_todo.md`
4. `docs/scenarios/indoor/navlab_p1_official_maze_x2_design.md`
5. `docs/scenarios/indoor/todos/P1_official_maze_x2_todo.md`
6. `docs/scenarios/indoor/navlab_p2_rangefinder_imu_design.md`
7. `docs/scenarios/indoor/todos/P2_rangefinder_imu_todo.md`
8. `docs/scenarios/indoor/navlab_p3_slam_backend_quality_design.md`
9. `docs/scenarios/indoor/todos/P3_slam_backend_quality_todo.md`
10. `docs/scenarios/indoor/navlab_p4_fcu_state_machine_design.md`
11. `docs/scenarios/indoor/todos/P4_fcu_state_machine_todo.md`
12. `docs/scenarios/indoor/navlab_p5_frame_contract_design.md`
13. `docs/scenarios/indoor/todos/P5_frame_contract_todo.md`
14. `docs/scenarios/indoor/navlab_p6_slam_hover_gate_design.md`
15. `docs/scenarios/indoor/todos/P6_slam_hover_gate_todo.md`
16. `docs/scenarios/indoor/navlab_ardupilot_ros2_official_alignment.md`
17. `docs/scenarios/indoor/navlab_reference_projects_analysis.md`
18. `docs/sim/x2_lidar_simulation_design.md`
19. `docs/sim/x2_lidar_protocol_todo.md`
20. `docs/general/lab_env_service_refactor_todo.md`

## 当前目录结构

```text
docs/
  README.md
  decisions.md
  general/
    lab_env_service_refactor_todo.md
  scenarios/
    indoor/
      navlab_master_roadmap.md
      navlab_p0_official_baseline_design.md
      navlab_p1_official_maze_x2_design.md
      navlab_p2_rangefinder_imu_design.md
      navlab_p3_slam_backend_quality_design.md
      navlab_p4_fcu_state_machine_design.md
      navlab_p5_frame_contract_design.md
      navlab_p6_slam_hover_gate_design.md
      navlab_ardupilot_ros2_official_alignment.md
      navlab_reference_projects_analysis.md
      navlab_cartographer_real_machine_tuning.md
      todos/
        P0_official_baseline_todo.md
        P1_official_maze_x2_todo.md
        P2_rangefinder_imu_todo.md
        P3_slam_backend_quality_todo.md
        P4_fcu_state_machine_todo.md
        P5_frame_contract_todo.md
        P6_slam_hover_gate_todo.md
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
- P0 必须先证明官方 `/ap/*` DDS baseline，而不是直接把自定义 MAVLink bridge 当作完成标准。
- SLAM 必须消费 `/scan + /imu + /odometry`，不能消费 Gazebo truth 或 FCU fused local position。
- ExternalNav 验收必须来自真实 SLAM `/odom` 或官方等价 ExternalNav 路线。
- Gazebo truth 只能用于诊断和误差对照。
- 上游代码不能直接 `set_pose` 移动 Gazebo 无人机。
- 每次 acceptance 必须输出 MCAP rosbag，方便 Foxglove 回放。
