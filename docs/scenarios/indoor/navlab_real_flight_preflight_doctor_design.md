# NavLab 真机飞行前 Preflight Doctor 设计

## 1. 背景

当前 `hover`、`exploration` 和 `scan-robustness` built-in task 的主线仍是
Gazebo/SITL Stage 1 验收。它们会启动 official baseline、Gazebo/SITL、
gazebo-sensor、X2 virtual serial、SDF overlay 等仿真组件。

真机起飞前不能复用这些仿真入口来“顺便检查一下”。真实机器的前置检查必须
走独立的 real preflight doctor：

```text
real-preflight-doctor
  -> checks process + real runtime boundary
  -> checks real ROS / FCU / sensor topics
  -> checks simulation sources are absent
  -> writes preflight summary
  -> does not arm, take off, land, or publish movement setpoints
```

也就是说，真机飞行入口的顺序必须是：

```text
Stage 1 Gazebo/SITL acceptance passed
  -> real-preflight-doctor passed
  -> operator safety confirmation
  -> real hover / P8 / P12 flight task
  -> landing summary
```

不能是：

```text
just navlab-hover
  -> use simulation hover result as real readiness
  -> fly the real machine
```

## 2. 目标

真机 preflight doctor 要回答一个问题：

> 当前系统是否真的处在 `process + real` 边界内，并且真实 FCU、真实传感器、
> 真实 SLAM 和真实 landing 所需 topic 已经可观测，足以允许后续真机 flight
> task 进入 arm/takeoff 阶段？

它的目标不是证明 hover 已经完成，也不是证明 landing 已经完成。它只证明：

- 当前 runtime backend 是 `process`。
- 当前 runtime mode 是 `real`。
- Gazebo/SITL/official baseline/gazebo-sensor 没有被当作输入。
- `/scan` 来自真实 lidar driver 或真实 scan validation/stabilization pipeline。
- FCU 状态和 filtered pose 来自真实飞控链路。
- SLAM odom 来自真实 SLAM。
- 真机任务需要的基础 topic 存在且新鲜。
- summary 能作为后续真机 flight task 的入口证据。

## 3. 不做什么

real preflight doctor 必须保持非执行性：

- 不 arm。
- 不 takeoff。
- 不 land。
- 不发布 `/ap/v1/cmd_vel`。
- 不发布 movement / landing intent。
- 不启动 Gazebo。
- 不启动 SITL。
- 不启动 official baseline。
- 不启动 gazebo-sensor。
- 不生成或加载 SDF / motor disturbance overlay。
- 不使用 `/scan_ideal`、`/sim/x2/status`、`/rangefinder/down/scan_ideal`。
- 不把仿真 Stage 1 artifact 当成真机数据源。

doctor 通过只表示“允许进入下一层真机飞行入口检查”，不表示飞机已经可以无条件
自动起飞。

## 4. 入口关系

### 4.1 仿真 Stage 1

Stage 1 仍由 Gazebo/SITL built-in task 负责：

| task | Stage 1 command | 结果 |
|---|---|---|
| hover | `just navlab-hover ... --simulation-profile ideal` 和 `mild_disturbance` | 证明仿真起飞、悬停、原地降落 |
| P8 exploration | `just navlab-exploration ... --simulation-profile ideal` 和 `mild_disturbance` | 证明仿真移动、返航、降落 |
| P12 scan robustness | `just navlab-scan-robustness ...` | 证明仿真扰动/scan 鲁棒性和原地降落 |

Stage 1 允许使用 Gazebo/SITL 生成可复现实验数据，但仍不能把 Gazebo truth
作为控制、SLAM、ExternalNav 或 landing 的输入。

### 4.2 Real Preflight Doctor

每一次真机飞行尝试之前都必须重新运行：

```bash
NAVLAB_RUNTIME_BACKEND=process NAVLAB_RUNTIME_MODE=real \
just navlab-real-preflight-doctor --config orchestration/config.real.toml
```

doctor 输出 `summary.json`。后续真机 flight task 必须读取或引用这个 summary，
并检查：

- `ok == true`
- `runtime_backend == "process"`
- `runtime_mode == "real"`
- 没有 required real topic missing blocker
- 没有 forbidden simulation topic blocker
- preflight summary 的时间没有超过配置的有效窗口

### 4.3 Real Flight Task

后续应新增或迁移真机 flight entry，例如：

```text
hover-real
exploration-real
scan-robustness-real
```

这些 entry 只能在 real preflight doctor 通过后执行。它们负责：

- 读取真机专用配置，例如 `takeoff_alt_m`。
- 获取 operator safety confirmation。
- 维护唯一 FCU owner。
- 执行 arm / takeoff / task body / landing。
- 输出 `acceptance_stage=real` 的 flight summary。

real flight task 不能重新启动仿真传感器，也不能 fallback 到 `just navlab-hover`
的 Docker/Gazebo 路径。

## 5. 真实数据源 contract

real preflight doctor 至少检查以下 required real topics：

```text
/scan
/tf
/tf_static
/slam/odom
/ap/v1/status
/ap/v1/pose/filtered
```

后续真机 flight task 还应额外要求：

```text
/ap/v1/twist/filtered
/imu
/rangefinder/down/range
/rangefinder/down/status
/navlab/fcu/controller/status
/navlab/fcu/owner/status
/navlab/landing/status
```

其中 `/rangefinder/down/range` 可以来自真实下视测距或 FCU telemetry bridge。
如果真实 FCU 已经内部消费下视测距，但 ROS 侧暂时没有直接 topic，summary 必须
明确写出：

```json
{
  "rangefinder_source_claim": "real_fcu_internal_or_bridge",
  "rangefinder_ros_topic_claim": "not_available",
  "rangefinder_fcu_receive_evidence": "required"
}
```

不能因为 ROS topic 缺失就用仿真 `/rangefinder/down/scan_ideal` 代替。

## 6. Forbidden Simulation Inputs

real mode 下出现以下 topic 或 source claim 必须 block：

```text
/gazebo/*
/scan_ideal
/sim/x2/status
/rangefinder/down/scan_ideal
Gazebo lidar source claim
SITL FCU source claim
SDF overlay source claim
motor disturbance overlay source claim
official maze input claim
```

注意：真实 lidar driver 也可能使用 `/lidar` 作为原始 topic。因此 forbidden
规则不能只靠名字硬编码，必须结合 source claim 判断。推荐真机链路把 SLAM 唯一
输入稳定在 `/scan`，并在 summary 里记录：

```json
{
  "source_claims": {
    "scan": "real_lidar_driver_or_real_scan_stabilization",
    "fcu": "real_ardupilot_dds_or_real_mavlink_bridge",
    "imu": "real_fcu_or_sensor",
    "rangefinder": "real_down_rangefinder_or_fcu_internal",
    "slam": "real_slam"
  }
}
```

## 7. 高度设置边界

真机起飞高度属于 real flight task 配置，不属于 preflight doctor 配置。

当前统一配置字段是：

```toml
[fcu_controller]
takeoff_alt_m = 0.5
```

real preflight doctor 可以读取并记录计划高度，但不能执行起飞。后续真机 flight
task 应在 summary 中记录：

```json
{
  "planned_takeoff_alt_m": 0.5,
  "takeoff_alt_source": "config:fcu_controller.takeoff_alt_m"
}
```

首飞时高度应使用独立真机配置文件控制，例如：

```bash
NAVLAB_RUNTIME_BACKEND=process NAVLAB_RUNTIME_MODE=real \
uv run --project orchestration python orchestration/main.py hover-real \
  --config orchestration/config.real.toml
```

`hover-real` 只有在 real preflight doctor 通过后才允许存在。

## 8. Summary Schema

real preflight doctor summary 建议包含：

```json
{
  "ok": true,
  "blocked": false,
  "blockers": [],
  "runtime_backend": "process",
  "runtime_mode": "real",
  "preflight_claim": "evaluated",
  "flight_claim": "not_evaluated",
  "landing_claim": "not_evaluated",
  "source_claims": {
    "scan": "real_lidar_driver",
    "fcu": "real_ardupilot_dds",
    "imu": "real_fcu_or_sensor",
    "rangefinder": "real_or_not_required",
    "slam": "real_slam"
  },
  "real_preflight": {
    "required_real_topics": ["/scan", "/tf", "/tf_static", "/slam/odom", "/ap/v1/status", "/ap/v1/pose/filtered"],
    "forbidden_simulation_input_topics": ["/gazebo/*", "/scan_ideal", "/sim/x2/status", "/rangefinder/down/scan_ideal"],
    "topic_count": 0,
    "checked_at": "2026-06-09T00:00:00Z",
    "valid_for_sec": 120
  }
}
```

real flight summary 应引用 preflight summary：

```json
{
  "acceptance_stage": "real",
  "real_preflight": {
    "ok": true,
    "artifact": "artifacts/ros/navlab_real_preflight_doctor/<run_id>/summary.json",
    "age_sec_at_takeoff": 42.0
  },
  "real_landing_claim": "evaluated"
}
```

## 9. Blocker Rules

后续真机 flight task 入口必须在以下情况 blocked：

```text
real_preflight_missing
real_preflight_failed
real_preflight_expired
runtime_backend_must_be_process:<actual>
runtime_mode_must_be_real:<actual>
required_real_topic_missing:<topic>
forbidden_simulation_topic_present:<topic>
simulation_stage_not_passed
manual_takeover_not_confirmed
kill_switch_not_confirmed
takeoff_altitude_not_configured
```

这些 blocker 应在 arm/takeoff 之前产生。只要出现 blocker，就不能进入飞行状态机。

## 10. 完成标准

本设计完成后，系统边界应当清楚：

- `just navlab-hover` 只代表 Gazebo/SITL Stage 1 hover，不代表真机 hover。
- 每次真机飞行前必须先跑 `real-preflight-doctor`。
- real preflight doctor 不触发电机或飞行动作。
- 真机 flight task 必须引用最新通过的 preflight summary。
- real mode 下不能使用模拟串口、Gazebo lidar、SITL、SDF overlay 或仿真 rangefinder。
- 起飞高度由真机 flight task 的配置读取，不由 doctor 执行。
