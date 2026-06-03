from __future__ import annotations

import os
import shlex
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import tomllib

from lab_env.config import load_navlab_images_config, load_runtime_config

DEFAULT_PROFILE = Path("profiles/navlab-gazebo.toml")
NAVLAB_PROFILES = ("base_env", "sim_p1")
NAVLAB_SERVICES = (
    "gazebo",
    "scan-bridge",
    "foxglove",
    "mavlink-router",
    "sitl",
)
NAVLAB_STOP_SERVICES = (
    "scan-bridge",
    "gazebo",
    "sitl",
    "mavlink-router",
    "foxglove",
)


@dataclass(frozen=True, slots=True)
class FoxgloveUploadConfig:
    enabled: bool
    api_url: str
    token_env: str
    project_id: str
    device_id: str
    device_name: str
    key_prefix: str
    filename_prefix: str


@dataclass(frozen=True, slots=True)
class SlamContainerConfig:
    autostart: bool
    image: str
    backend: str
    imu_source_topic: str
    imu_source_label: str
    imu_min_input_rate_hz: str
    args: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class OrchestrationConfig:
    path: Path
    session_id: str
    ros_domain_id: str
    gazebo_world: str
    rosbag_profile: str
    companion_image: str
    sitl_image: str
    sitl_model: str
    sitl_speedup: str
    sitl_instance: str
    sitl_home: str
    sitl_upstream_endpoint: str
    sitl_router_only: str
    sitl_extra_args: tuple[str, ...]
    router_image: str
    router_downstream_endpoints: str
    router_listen: str
    router_tcp_port: str
    slam: SlamContainerConfig
    foxglove_upload: FoxgloveUploadConfig

    @classmethod
    def load(cls, path: str | Path | None = None) -> OrchestrationConfig:
        profile_path = resolve_profile_path(path)
        data = tomllib.loads(profile_path.read_text(encoding="utf-8"))
        runtime_config = load_runtime_config()
        image_config = load_navlab_images_config(runtime_config)
        orchestration = _section(data, "orchestration")
        sitl = _section(orchestration, "sitl")
        router = _section(orchestration, "router")
        slam = _section(orchestration, "slam")
        foxglove_upload = _section(orchestration, "foxglove_upload")
        return cls(
            path=profile_path,
            session_id=_as_str(orchestration.get("session_id"), "navlab_companion_sitl_gazebo"),
            ros_domain_id=_as_str(orchestration.get("ros_domain_id"), "85"),
            gazebo_world=_as_str(orchestration.get("gazebo_world"), "/workspace/worlds/uav_obstacle_5m.sdf"),
            rosbag_profile=_as_str(orchestration.get("rosbag_profile"), "profiles/navlab-rosbag-topics.txt"),
            companion_image=_as_str(
                orchestration.get("companion_image"),
                image_config.companion.image(cwd=runtime_config.lab_root),
            ),
            sitl_image=_as_str(sitl.get("image"), "remote-sitl-lab/ardupilot-sitl:stage1-f10500ae45aa"),
            sitl_model=_as_str(sitl.get("model"), "quad"),
            sitl_speedup=_as_str(sitl.get("speedup"), "1"),
            sitl_instance=_as_str(sitl.get("instance"), "0"),
            sitl_home=_as_str(sitl.get("home"), ""),
            sitl_upstream_endpoint=_as_str(sitl.get("upstream_endpoint"), "mavlink-router:14550"),
            sitl_router_only=_as_str(sitl.get("router_only"), "false"),
            sitl_extra_args=_as_args(sitl.get("extra_args")),
            router_image=_as_str(router.get("image"), "remote-sitl-lab/mavlink-router:stage1-4ee567d97525"),
            router_downstream_endpoints=_as_str(router.get("downstream_endpoints"), "127.0.0.1:14552"),
            router_listen=_as_str(router.get("listen"), "0.0.0.0:14550"),
            router_tcp_port=_as_str(router.get("tcp_port"), "5760"),
            slam=SlamContainerConfig(
                autostart=_as_bool(slam.get("autostart"), True),
                image=_as_str(slam.get("image"), image_config.slam.image(cwd=runtime_config.lab_root)),
                backend=_as_str(slam.get("backend"), "cartographer"),
                imu_source_topic=_as_str(slam.get("imu_source_topic"), "/navlab/fcu_imu/data"),
                imu_source_label=_as_str(slam.get("imu_source_label"), "fcu_mavlink_navlab"),
                imu_min_input_rate_hz=_as_str(slam.get("imu_min_input_rate_hz"), "2.0"),
                args=_as_args(slam.get("args")),
            ),
            foxglove_upload=FoxgloveUploadConfig(
                enabled=_as_bool(foxglove_upload.get("enabled"), True),
                api_url=_as_str(foxglove_upload.get("api_url"), "https://api.foxglove.dev/v1"),
                token_env=_as_str(foxglove_upload.get("token_env"), "FOXGLOVE_API_TOKEN"),
                project_id=_as_str(foxglove_upload.get("project_id"), ""),
                device_id=_as_str(foxglove_upload.get("device_id"), ""),
                device_name=_as_str(foxglove_upload.get("device_name"), "navlab_companion_sitl_gazebo"),
                key_prefix=_as_str(foxglove_upload.get("key_prefix"), "navlab"),
                filename_prefix=_as_str(foxglove_upload.get("filename_prefix"), "navlab"),
            ),
        )

    def compose_env(self) -> dict[str, str]:
        return {
            "SESSION_ID": self.session_id,
            "SITL_IMAGE": self.sitl_image,
            "ROUTER_IMAGE": self.router_image,
            "ROS_DOMAIN_ID": self.ros_domain_id,
            "ROUTER_DOWNSTREAM_ENDPOINTS": self.router_downstream_endpoints,
            "ROUTER_LISTEN": self.router_listen,
            "ROUTER_TCP_PORT": self.router_tcp_port,
            "SITL_UPSTREAM_ENDPOINT": self.sitl_upstream_endpoint,
            "SITL_ROUTER_ONLY": self.sitl_router_only,
            "SITL_MODEL": self.sitl_model,
            "SITL_SPEEDUP": self.sitl_speedup,
            "SITL_INSTANCE": self.sitl_instance,
            "SITL_HOME": self.sitl_home,
            "SITL_EXTRA_ARGS": shlex.join(self.sitl_extra_args),
            "GAZEBO_WORLD": self.gazebo_world,
            "NAVLAB_COMPANION_IMAGE": self.companion_image,
            "NAVLAB_SLAM_IMAGE": self.slam.image,
            "NAVLAB_CONFIG": str(self.path),
        }


@dataclass(frozen=True, slots=True)
class RunConfig:
    orchestration: OrchestrationConfig
    duration_sec: float = 90.0
    run_id: str = ""
    artifact_dir: Path = Path()

    @property
    def ros_domain_id(self) -> str:
        return self.orchestration.ros_domain_id

    @property
    def session_id(self) -> str:
        return self.orchestration.session_id

    @property
    def companion_image(self) -> str:
        return self.orchestration.companion_image

    @property
    def slam_image(self) -> str:
        return self.orchestration.slam.image

    @property
    def rosbag_profile(self) -> str:
        return self.orchestration.rosbag_profile

    @classmethod
    def from_profile(
        cls,
        *,
        profile_path: str | Path | None = None,
        duration_sec: float = 90.0,
        artifact_dir: str | Path | None = None,
        run_id: str | None = None,
    ) -> RunConfig:
        orchestration = OrchestrationConfig.load(profile_path)
        final_run_id = run_id or os.environ.get("RUN_ID") or datetime.now().strftime("%Y%m%d_%H%M%S")
        final_artifact_dir = Path(
            artifact_dir or os.environ.get("ARTIFACT_DIR") or f"artifacts/ros/{orchestration.session_id}/{final_run_id}"
        )
        return cls(
            orchestration=orchestration,
            duration_sec=duration_sec,
            run_id=final_run_id,
            artifact_dir=final_artifact_dir,
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
