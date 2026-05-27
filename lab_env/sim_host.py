from __future__ import annotations

import math
import os
from datetime import datetime
from pathlib import Path

from lab_env.config import load_compose_config, load_runtime_config, load_sim_config
from lab_env.logging_utils import configure_sim_logging, logger
from lab_env.sim.runtime import build_rosbag_options, invoke_python_target
from lab_env.sim.waypoints import load_straight_line_mission

_SIM_PROFILES = ("base_env", "sim_p1")
_SIM_SERVICES = ("gazebo", "scan-bridge", "sim-runtime", "foxglove")
_SIM_RUNTIME_ROOT = Path("/workspace")
_AUTO_ROSBAG_LABEL = "auto_waypoint_follower"
_AUTO_MISSION_MONITOR_TARGET = "lab_env.sim.nodes.auto_mission_monitor:run"


def _resolve_sim_runtime_path(path: str | Path) -> str:
    runtime = load_runtime_config()
    waypoint_path = Path(path).expanduser().resolve()
    try:
        relative = waypoint_path.relative_to(runtime.lab_root.resolve())
    except ValueError as exc:
        raise ValueError(
            f"Waypoint file must live under {runtime.lab_root} so sim-runtime can see it: {waypoint_path}"
        ) from exc
    return str(_SIM_RUNTIME_ROOT / relative)


def _runtime_artifact_dir(relative_dir: Path) -> str:
    return str(_SIM_RUNTIME_ROOT / relative_dir)


def wait_for_auto_mission(*, timeout_sec: float = 300.0, status_topic: str = "/sim/log") -> int:
    args = [
        "--status-topic",
        status_topic,
        "--timeout-sec",
        str(timeout_sec),
    ]
    return exec_runtime_python_target(_AUTO_MISSION_MONITOR_TARGET, args)


def _build_auto_artifact_paths(*, session_id: str | None = None, run_id: str | None = None) -> tuple[Path, str]:
    runtime = load_runtime_config()
    session = session_id or os.environ.get("SESSION_ID", "manual")
    final_run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    relative_dir = Path("artifacts") / "ros" / session / _AUTO_ROSBAG_LABEL / final_run_id
    return runtime.lab_root / relative_dir, _runtime_artifact_dir(relative_dir)


def _sim_compose_client():
    from python_on_whales import DockerClient

    runtime = load_runtime_config()
    compose = load_compose_config(runtime)
    env_file = runtime.lab_root / ".env"
    return DockerClient(
        compose_files=[compose.compose_file],
        compose_profiles=list(_SIM_PROFILES),
        compose_project_name=compose.project_name.value,
        compose_project_directory=runtime.lab_root,
        compose_env_files=([env_file] if env_file.is_file() else []),
    )


def sim_up(
    *,
    markers: bool,
    mode: str,
    waypoint_file: str | None = None,
    timeout_sec: float = 300.0,
) -> int:
    mode = mode.lower()
    if mode not in {"manual", "auto"}:
        raise ValueError(f"unsupported sim mode '{mode}': expected manual or auto")
    runtime = load_runtime_config()
    sim_config = load_sim_config(runtime)
    configure_sim_logging(
        console_level=sim_config.console_log_level.value,
        file_level=sim_config.file_log_level.value,
    )

    env_overrides = {
        "SIM_MARKERS_AUTOSTART": "true" if markers else "false",
        "SIM_UP_MODE": mode,
        "SIM_AUTO_ROSBAG_ENABLED": "true" if mode == "auto" else "false",
        "SIM_AUTO_ROSBAG_LABEL": _AUTO_ROSBAG_LABEL,
        "SIM_AUTO_RUN_ID": "",
        "SIM_AUTO_ARTIFACT_DIR": "",
        "SIM_AUTO_LOG_FILE": "",
    }
    host_artifact_dir: Path | None = None
    if mode == "auto":
        if waypoint_file is None:
            raise ValueError("auto mode requires --waypoint-file")
        load_straight_line_mission(waypoint_file)
        env_overrides["SIM_AUTO_WAYPOINT_FILE"] = _resolve_sim_runtime_path(waypoint_file)
        host_artifact_dir, runtime_artifact_dir = _build_auto_artifact_paths()
        env_overrides["SIM_AUTO_RUN_ID"] = host_artifact_dir.name
        env_overrides["SIM_AUTO_ARTIFACT_DIR"] = runtime_artifact_dir
        env_overrides["SIM_AUTO_LOG_FILE"] = f"{runtime_artifact_dir}/sim.log"
        configure_sim_logging(
            log_file=host_artifact_dir / "sim.log",
            console_level=sim_config.console_log_level.value,
            file_level=sim_config.file_log_level.value,
        )
    else:
        if waypoint_file is not None:
            raise ValueError("--waypoint-file is only valid with --mode auto")
        env_overrides["SIM_AUTO_WAYPOINT_FILE"] = ""

    previous = {key: os.environ.get(key) for key in env_overrides}
    for key, value in env_overrides.items():
        os.environ[key] = value
    try:
        _sim_compose_client().compose.up(services=list(_SIM_SERVICES), detach=True, build=True)
    finally:
        for key, old_value in previous.items():
            if old_value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old_value

    if mode == "manual":
        logger.info("[sim] manual mode started")
        logger.info("[sim] markers: {}", "enabled" if markers else "disabled")
        logger.info("[sim] foxglove websocket: ws://127.0.0.1:8765")
        logger.info("[sim] control topic: /planner/cmd_vel (geometry_msgs/msg/Twist)")
        logger.info("[sim] stop with: uv run --project lab_env --no-sync --group host python -m lab_env.main sim down")
        return 0

    assert waypoint_file is not None
    assert host_artifact_dir is not None
    logger.info("[sim] auto mode started")
    logger.info("[sim] waypoint file: {}", waypoint_file)
    logger.info("[sim] status topic: /sim/log")
    logger.info("[sim] artifact dir: {}", host_artifact_dir)
    logger.info("[sim] waiting for mission completion")
    exit_code = 0
    try:
        exit_code = wait_for_auto_mission(timeout_sec=timeout_sec)
    finally:
        logger.info("[sim] shutting down sim environment")
        sim_down()

    logger.info("[sim] rosbag dir: {}", host_artifact_dir)
    return exit_code


def sim_down() -> int:
    _sim_compose_client().compose.down()
    return 0


def exec_runtime_command(command: list[str]) -> int:
    client = _sim_compose_client()
    try:
        client.compose.execute(
            "sim-runtime",
            ["bash", "/usr/local/bin/sim-runtime-env.sh", *command],
            tty=True,
        )
    except Exception as exc:
        from python_on_whales.exceptions import DockerException

        if not isinstance(exc, DockerException):
            raise
        return exc.return_code
    return 0


def exec_runtime_python_target(
    target: str,
    argv: list[str] | None = None,
    *,
    record: bool = False,
    label: str = "run",
) -> int:
    code = (
        "from lab_env.sim.runtime import build_rosbag_options, invoke_python_target; "
        f"raise SystemExit(invoke_python_target({target!r}, {argv or []!r}, "
        f"rosbag_options=build_rosbag_options(enabled={record!r}, label={label!r})))"
    )
    return exec_runtime_command(["python3", "-c", code])


def _twist_message_literal(*, linear_x: float, angular_z: float) -> str:
    return (
        "{linear: {x: "
        f"{linear_x}, y: 0.0, z: 0.0"
        "}, angular: {x: 0.0, y: 0.0, z: "
        f"{angular_z}"
        "}}"
    )


def publish_cmd_vel_preset(
    *,
    preset: str,
    topic: str = "/planner/cmd_vel",
    linear_x: float = 0.2,
    angular_z: float = 0.0,
    duration: float = 1.0,
    rate: float = 10.0,
) -> int:
    preset = preset.lower()
    if preset == "forward":
        times = max(1, math.ceil(duration * rate))
        command = [
            "ros2",
            "topic",
            "pub",
            "-r",
            str(rate),
            "--times",
            str(times),
            topic,
            "geometry_msgs/msg/Twist",
            _twist_message_literal(linear_x=linear_x, angular_z=angular_z),
        ]
    elif preset == "stop":
        command = [
            "ros2",
            "topic",
            "pub",
            "--once",
            topic,
            "geometry_msgs/msg/Twist",
            _twist_message_literal(linear_x=0.0, angular_z=0.0),
        ]
    else:
        raise ValueError(f"unsupported cmd_vel preset '{preset}': expected forward or stop")
    return exec_runtime_command(command)
