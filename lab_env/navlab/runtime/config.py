from __future__ import annotations

import os
import shlex
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import tomllib

DEFAULT_PROFILE = Path("profiles/navlab-gazebo.toml")


@dataclass(frozen=True, slots=True)
class NodeConfig:
    autostart: bool
    endpoint: str = ""
    args: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class RuntimeConfig:
    path: Path
    imu_source_label: str
    world_markers: NodeConfig
    scan_features: NodeConfig
    pose_mirror: NodeConfig
    imu_bridge: NodeConfig
    external_nav_sender: NodeConfig
    mission: NodeConfig

    @classmethod
    def load(cls, path: str | Path | None = None) -> RuntimeConfig:
        profile_path = resolve_profile_path(path)
        data = tomllib.loads(profile_path.read_text(encoding="utf-8"))
        runtime = _section(data, "runtime")
        world_markers = _section(runtime, "world_markers")
        scan_features = _section(runtime, "scan_features")
        pose_mirror = _section(runtime, "pose_mirror")
        imu_bridge = _section(runtime, "imu_bridge")
        external_nav_sender = _section(runtime, "external_nav_sender")
        mission = _section(runtime, "mission")
        return cls(
            path=profile_path,
            imu_source_label=_as_str(runtime.get("imu_source_label"), "fcu_mavlink_navlab"),
            world_markers=NodeConfig(
                autostart=_as_bool(world_markers.get("autostart"), True),
                args=_as_args(world_markers.get("args")),
            ),
            scan_features=NodeConfig(
                autostart=_as_bool(scan_features.get("autostart"), True),
                args=_as_args(scan_features.get("args")),
            ),
            pose_mirror=NodeConfig(
                autostart=_as_bool(pose_mirror.get("autostart"), True),
                endpoint=_as_str(pose_mirror.get("endpoint"), "tcp:mavlink-router:5760"),
                args=_as_args(pose_mirror.get("args")),
            ),
            imu_bridge=NodeConfig(
                autostart=_as_bool(imu_bridge.get("autostart"), False),
                endpoint=_as_str(imu_bridge.get("endpoint"), "tcp:mavlink-router:5760"),
                args=_as_args(imu_bridge.get("args")),
            ),
            external_nav_sender=NodeConfig(
                autostart=_as_bool(external_nav_sender.get("autostart"), True),
                endpoint=_as_str(external_nav_sender.get("endpoint"), "tcp:mavlink-router:5760"),
                args=_as_args(external_nav_sender.get("args")),
            ),
            mission=NodeConfig(
                autostart=_as_bool(mission.get("autostart"), False),
                endpoint=_as_str(mission.get("endpoint"), "tcp:mavlink-router:5760"),
                args=_as_args(mission.get("args")),
            ),
        )


def resolve_profile_path(path: str | Path | None = None) -> Path:
    raw = path or os.environ.get("NAVLAB_CONFIG") or DEFAULT_PROFILE
    return Path(raw).expanduser()


def _as_str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    return str(value)


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).lower() in {"1", "true", "yes", "on"}


def _as_args(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return tuple(shlex.split(value))
    if isinstance(value, list | tuple):
        return tuple(str(item) for item in value)
    raise TypeError(f"expected args list or string, got {type(value).__name__}")


def _section(data: dict[str, Any], *keys: str) -> dict[str, Any]:
    value: Any = data
    for key in keys:
        value = value.get(key, {}) if isinstance(value, dict) else {}
    if not isinstance(value, dict):
        return {}
    return value
