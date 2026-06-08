# 仿真主线说明

这个目录现在只记录 NavLab 主线需要的仿真契约。旧 Stage 0 的单独启动入口已经退场：
不再有独立的 sim CLI、旧扫描桥服务或通用仿真 runtime。

当前仿真链路收敛为：

```text
Gazebo world
  -> /scan_ideal
  -> gazebo_sensor runtime
  -> X2 virtual serial packets
  -> ydlidar_ros2_driver_node
  -> /scan
  -> NavLab companion runtime
  -> /scan_features, /navlab/mission/status, MAVLink setpoint
```

## 当前服务边界

```text
orchestration/
  cli.py
  host.py
  config.py
  project_config.py

navlab/
  gazebo_sensor/
    x2/
  companion/
  slam/
  common/
  sim/
```

- `gazebo_sensor` 只负责 Gazebo ideal scan 到厂商驱动 `/scan` 的传感器链路。
- `navlab.companion` 只负责机载计算盒子侧 runtime：world markers、scan features、FCU IMU bridge、ExternalNav sender、mission controller 和 acceptance artifact。
- `navlab.slam` 是可替换 SLAM 后端镜像，当前默认是 Cartographer。
- `common` 只放跨服务复用的纯工具，不拥有 ROS runtime 职责。

## 关键 topic

- `/scan_ideal`: Gazebo 直出的理想 LaserScan，只用于传感器仿真输入和 Foxglove 对照。
- `/sim/x2/status`: X2 虚拟串口仿真状态。
- `/scan`: 厂商 `ydlidar_ros2_driver_node` 发布的最终 LaserScan。
- `/scan_features`: companion 从最终 `/scan` 提取的扇区特征。
- `/sim/markers`: companion 发布的 UAV、路径和障碍物 marker。
- `/navlab/mission/status`: mission controller 发布的任务阶段和 setpoint 状态。

下游 SLAM、mission 和 rosbag 只应消费最终 `/scan`，不能依赖 X2 协议包或虚拟串口内部细节。

## 当前命令

主线入口放在 `navlab`：

```bash
uv run --project orchestration python orchestration/main.py build all
uv run --project orchestration python orchestration/main.py doctor
X2_MODE=driver-smoke X2_SMOKE_DURATION_SEC=8 X2_ARTIFACT_DIR=/artifacts/ros/x2_driver_smoke/manual docker compose --file compose/docker-compose.yaml --project-directory . --profile x2_sensor up --abort-on-container-exit --exit-code-from gazebo-sensor gazebo-sensor
uv run --project orchestration python orchestration/main.py acceptance 90
```

NavLab acceptance 会启动 Gazebo、SITL、MAVLink router、gazebo sensor、companion、SLAM 和 Foxglove，
并生成 rosbag/MCAP、`summary.json`、`summary.md` 和 Foxglove 回放说明。

## Foxglove 回放

验收 rosbag 应至少包含：

- `/scan`
- `/scan_ideal`
- `/sim/x2/status`
- `/scan_features`
- `/sim/markers`
- `/navlab/mission/status`

3D 面板建议固定 frame 为 `map`。`/sim/markers` 用于显示 UAV、路径和障碍物；
`/scan` 与 `/scan_ideal` 用于对照最终厂商 scan 和 Gazebo ideal scan。

## 历史说明

`docs/sim/todo.md` 保留早期 Stage 0 仿真任务记录，只作为历史上下文。当前可执行主线以
NavLab 设计文档、X2 协议级仿真设计和 `docs/general/旧 Python 外壳_service_refactor_todo.md` 为准。
