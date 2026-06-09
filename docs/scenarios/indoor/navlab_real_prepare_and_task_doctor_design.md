# NavLab 真机 Prepare 与 Task Doctor 设计

## 1. 背景

真机路径不是 Docker/compose 仿真路径，但也不是只让 companion 直接读
`/dev/ttyACM0`。真实系统应由 host process 管理多个辅助进程：

```text
FCU serial MAVLink
  -> mavlink-router
  -> local MAVLink endpoints
  -> MAVROS / FCU ROS bridge
  -> ROS FCU topics

real lidar driver
  -> /scan

MAVROS / FCU IMU + lidar
  -> SLAM
  -> /slam/odom

verified upstream topics
  -> companion
  -> task controller / mission / landing
```

因此真机入口对外只保留一个 wrapper：

```text
run <task>
  -> real preflight doctor
  -> real prepare / bringup
  -> task doctor
  -> task run
```

## 2. 目标

本设计定义真机 Stage 2 的 host process 启动顺序和检查边界：

- `run <task>` 是唯一对外入口。
- `real preflight doctor`、`real prepare`、`task doctor` 都是 wrapper 内部阶段。
- `real preflight doctor` 只检查硬件、依赖、配置和禁止仿真输入。
- `real prepare` 在 preflight 通过后启动非 companion 的辅助进程。
- `task doctor` 使用统一 helper 检查 companion 启动前置 topic 和 task readiness。
- `task run` 才允许进入 arm / takeoff / mission / landing。

## 3. 非目标

本设计不把 doctor 变成飞行执行器：

- doctor 不 arm。
- doctor 不 takeoff。
- doctor 不 land。
- doctor 不发布 movement setpoint。
- doctor 不启动 Gazebo/SITL/gazebo-sensor。
- preflight doctor 不长期占用 FCU 串口。
- prepare 不启动 companion，不执行具体任务。

## 4. 阶段定义

### 4.1 Real Preflight Doctor

`real preflight doctor` 是常规硬件和依赖检查。它回答：

> 当前 host 是否具备启动真机辅助链路的条件？

它检查：

- runtime 是 `process + real`。
- FCU serial path 存在，权限正确，例如 `/dev/ttyACM0` 或 udev symlink。
- `mavlink-routerd` 或 `mavlink-router` 可执行文件存在。
- `ros2` CLI 可用。
- MAVROS 相关 package 存在，例如 `mavros`、`mavros_msgs`。
- SLAM package 存在，例如 `navlab_slam_bringup`、`navlab_cartographer_adapter`。
- companion / SLAM Python 入口可 import，例如 `navlab.companion.cli`、`navlab.slam.cli`。
- 配置中没有 Gazebo/SITL/X2 virtual serial/SDF overlay 作为 real source claim。

preflight doctor 可以短暂打开串口证明物理边界，但必须立即关闭。后续运行时串口
所有权应交给 `mavlink-router`。

### 4.2 Real Prepare / Bringup

`real prepare` 是有副作用的预处理工作。它只能在 preflight doctor 通过后运行。

它负责启动 companion 之前的辅助进程：

| 辅助进程 | 输入 | 输出 | 作用 |
|---|---|---|---|
| `mavlink-router` | FCU serial，例如 `/dev/ttyACM0:115200` | local TCP/UDP MAVLink endpoints | 独占串口并分发 MAVLink |
| `mavros` | mavlink-router local endpoint | MAVROS ROS topics | 把 MAVLink 转成 ROS graph 可读状态 |
| real lidar driver | 真实雷达设备 | `/scan` | 提供 SLAM scan |
| SLAM runtime | `/scan`、IMU/odom source | `/slam/odom`、`/navlab/slam/status` | 提供 ExternalNav 输入 |
| optional rangefinder bridge | FCU telemetry 或真实测距 | range/status topic | 记录下视测距 evidence |

`real prepare` 不启动 companion。原因是 companion 依赖上游 FCU、scan、SLAM topic
已经可观测；上游没准备好时启动 companion 会把问题混成 companion runtime 失败。

prepare summary 应记录：

```json
{
  "prepare_claim": "evaluated",
  "started_services": ["mavlink-router", "mavros", "lidar", "slam"],
  "mavlink_router": {
    "serial": "/dev/ttyACM0",
    "baud": 115200,
    "local_endpoint": "tcp://127.0.0.1:5760"
  },
  "mavros": {
    "fcu_url": "tcp://127.0.0.1:5760",
    "state_topic": "/mavros/state"
  },
  "blocked": false,
  "blockers": []
}
```

### 4.3 Task Doctor

`task doctor` 是 companion 和具体任务前置检查。它回答：

> 当前上游 topic 是否足够让 companion 启动，并且当前 task 是否具备执行条件？

所有真机 task doctor 应复用统一 helper，例如：

```text
check_real_task_upstream_topics(task_name, config)
```

统一 helper 至少检查：

- `/scan` 存在、新鲜、frame 符合配置。
- `/tf`、`/tf_static` 存在。
- FCU ROS status / pose / velocity topic 存在。
- MAVROS state 或等价 FCU bridge state 存在。
- `/slam/odom` 存在、新鲜、frame 符合配置。
- `/navlab/slam/status` ready。
- 需要下视测距时，rangefinder topic 或 FCU telemetry evidence 存在。
- 没有 forbidden simulation topic/source。

各 task 再追加自己的要求：

| task | 追加检查 |
|---|---|
| `hover` in real mode | hover 高度、landing policy、FCU mode/armed 初始状态 |
| `exploration` in real mode / P8 | return-home policy、home source、bounded movement limits |
| `scan-robustness` in real mode / P12 | scan stabilization / tilt robustness status |

task doctor 可以检查 companion 是否已运行和 companion status topic 是否 ready，但不应自己
执行 arm/takeoff。若需要启动 companion，应由 wrapper 内部的 companion prepare/run
phase 完成。

### 4.4 Task Run

`task run` 才是真机飞行执行阶段。它必须引用：

- Stage 1 Gazebo/SITL `ideal` 和 `mild_disturbance` summaries。
- 最新通过且未过期的 real preflight summary。
- 最新通过的 real prepare summary。
- 最新通过的 task doctor summary。
- operator safety confirmation。

## 5. 推荐命令结构

建议 CLI/justfile 逐步收敛到一个对外 wrapper：

```text
just navlab-run hover
just navlab-run exploration
just navlab-run scan-robustness
```

对应 orchestration 命令可以是：

```text
run hover
run exploration
run scan-robustness
```

wrapper 读取环境变量决定 backend/mode 和 real/simulation 路径，不使用 `--stage`：

```text
NAVLAB_RUNTIME_BACKEND=process NAVLAB_RUNTIME_MODE=real run hover
```

wrapper 内部仍应按顺序执行：

```text
preflight doctor -> real prepare -> task doctor -> companion/task run
```

不能把 prepare 的副作用隐藏进 doctor，也不能把 `doctor` 或 `prepare`
暴露成单独的 operator 入口。它们是 wrapper 的内部 phase，只出现在日志、
summary 和调试 artifact 里。

## 6. MAVLink Router 和 MAVROS Contract

真机 FCU 串口推荐只由 `mavlink-router` 独占：

```text
mavlink-routerd -t 5760 -e 127.0.0.1:14550 /dev/ttyACM0:115200
```

MAVROS 不直接抢 `/dev/ttyACM0`，而是连接 mavlink-router 暴露的本机 endpoint。
这样 pymavlink probe、MAVROS、日志工具和 GCS 可以共享 MAVLink 流。

doctor 中允许的 MAVLink endpoint 必须能追溯到 real prepare summary 中的真实串口。
不能把 SITL endpoint 当成真实 FCU 证据。

## 7. 完成标准

- preflight doctor 能在不启动进程的情况下定位缺依赖、缺硬件、错误 source claim。
- prepare 只在 preflight 通过后启动非 companion 辅助进程。
- task doctor 复用统一 upstream topic helper。
- companion 只在 FCU、scan、SLAM 等上游 ready 后启动。
- task run 只在 preflight / prepare / task doctor / operator safety 全部通过后进入 arm/takeoff。
