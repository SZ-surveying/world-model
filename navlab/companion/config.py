from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import tomllib

from navlab.common.toml_values import (
    FloatWithSource,
    ValueWithSource,
    as_args,
    as_bool,
    as_str,
    load_navlab_config,
    nested_section,
    resolve_float_value,
    resolve_navlab_config_file,
    resolve_str_value,
    section,
)

DEFAULT_CONFIG = Path("navlab/config.toml")
DEFAULT_NAVLAB_COMPANION_IMAGE = "world-model/navlab-companion:latest"
DEFAULT_STOP_DISTANCE = 0.5
DEFAULT_CONSOLE_LOG_LEVEL = "DEBUG"
DEFAULT_FILE_LOG_LEVEL = "INFO"


@dataclass(frozen=True, slots=True)
class NodeConfig:
    autostart: bool
    endpoint: str = ""
    args: tuple[str, ...] = ()


@dataclass(slots=True)
class CompanionConfig:
    stop_distance: FloatWithSource
    console_log_level: ValueWithSource
    file_log_level: ValueWithSource


@dataclass(frozen=True, slots=True)
class RuntimeConfig:
    path: Path
    imu_source_label: str
    world_markers: NodeConfig
    scan_features: NodeConfig
    gazebo_truth_bridge: NodeConfig
    gazebo_truth_odom: NodeConfig
    pose_mirror: NodeConfig
    imu_bridge: NodeConfig
    external_nav_sender: NodeConfig
    mission: NodeConfig

    @classmethod
    def load(cls, path: str | Path | None = None) -> RuntimeConfig:
        config_path = resolve_config_path(path)
        data = tomllib.loads(config_path.read_text(encoding="utf-8"))
        companion = nested_section(data, "companion")
        runtime = nested_section(companion, "runtime")
        world_markers = nested_section(runtime, "world_markers")
        scan_features = nested_section(runtime, "scan_features")
        gazebo_truth_bridge = nested_section(runtime, "gazebo_truth_bridge")
        gazebo_truth_odom = nested_section(runtime, "gazebo_truth_odom")
        pose_mirror = nested_section(runtime, "pose_mirror")
        imu_bridge = nested_section(runtime, "imu_bridge")
        external_nav_sender = nested_section(runtime, "external_nav_sender")
        mission = nested_section(runtime, "mission")
        return cls(
            path=config_path,
            imu_source_label=as_str(runtime.get("imu_source_label"), "fcu_mavlink_navlab"),
            world_markers=NodeConfig(
                autostart=as_bool(world_markers.get("autostart"), True),
                args=as_args(world_markers.get("args")),
            ),
            scan_features=NodeConfig(
                autostart=as_bool(scan_features.get("autostart"), True),
                args=as_args(scan_features.get("args")),
            ),
            gazebo_truth_bridge=NodeConfig(
                autostart=as_bool(gazebo_truth_bridge.get("autostart"), True),
                args=as_args(
                    gazebo_truth_bridge.get("args")
                    or (
                        "/world/navlab_iq_quad_figure8/dynamic_pose/info"
                        "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    ),
                ),
            ),
            gazebo_truth_odom=NodeConfig(
                autostart=as_bool(gazebo_truth_odom.get("autostart"), True),
                args=as_args(gazebo_truth_odom.get("args")),
            ),
            pose_mirror=NodeConfig(
                autostart=as_bool(pose_mirror.get("autostart"), True),
                endpoint=as_str(pose_mirror.get("endpoint"), "tcp:mavlink-router:5760"),
                args=as_args(pose_mirror.get("args")),
            ),
            imu_bridge=NodeConfig(
                autostart=as_bool(imu_bridge.get("autostart"), False),
                endpoint=as_str(imu_bridge.get("endpoint"), "tcp:mavlink-router:5760"),
                args=as_args(imu_bridge.get("args")),
            ),
            external_nav_sender=NodeConfig(
                autostart=as_bool(external_nav_sender.get("autostart"), True),
                endpoint=as_str(external_nav_sender.get("endpoint"), "tcp:127.0.0.1:5762"),
                args=as_args(external_nav_sender.get("args")),
            ),
            mission=NodeConfig(
                autostart=as_bool(mission.get("autostart"), False),
                endpoint=as_str(mission.get("endpoint"), "tcp:127.0.0.1:5763"),
                args=as_args(mission.get("args")),
            ),
        )


def resolve_config_path(path: str | Path | None = None) -> Path:
    raw = path or os.environ.get("NAVLAB_RUNTIME_CONFIG") or DEFAULT_CONFIG
    return resolve_navlab_config_file(raw)


def load_config(path: str | Path | None = None) -> CompanionConfig:
    config_file, config = load_navlab_config(path)
    raw_companion = section(config, "companion", path=config_file)
    raw_logging = section(raw_companion, "logging", path=config_file, default={})
    return CompanionConfig(
        stop_distance=resolve_float_value(raw_companion, "stop_distance", DEFAULT_STOP_DISTANCE),
        console_log_level=resolve_str_value(raw_logging, "console_level", DEFAULT_CONSOLE_LOG_LEVEL),
        file_log_level=resolve_str_value(raw_logging, "file_level", DEFAULT_FILE_LOG_LEVEL),
    )
