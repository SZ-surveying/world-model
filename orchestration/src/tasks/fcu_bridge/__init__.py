from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class FcuBridgeModeSpec:
    name: str
    description: str
    prepare_service_names: tuple[str, ...]
    prepare_required_topics: tuple[str, ...]
    preflight_required_ros_packages: tuple[str, ...] = ()
    preflight_required_python_modules: tuple[str, ...] = ()
    preflight_required_command_groups: tuple[tuple[str, ...], ...] = ()


NAVLAB_MAVLINK = FcuBridgeModeSpec(
    name="navlab_mavlink",
    description="NavLab MAVLink router plus NavLab MAVLink bridge topics; no MAVROS or /ap/v1 DDS required.",
    prepare_service_names=("mavlink_router", "navlab_mavlink_bridge", "lidar", "slam", "rangefinder_bridge"),
    prepare_required_topics=(
        "/navlab/mavlink/status",
        "/navlab/fcu/local_position_pose",
        "/mavlink_external_nav/status",
        "/external_nav/status",
        "/rangefinder/down/range",
        "/rangefinder/down/status",
    ),
    preflight_required_ros_packages=(
        "cartographer_ros",
        "navlab_slam_bringup",
        "navlab_cartographer_adapter",
        "navlab_external_nav_bridge",
        "navlab_slam_imu_bridge",
        "ydlidar_ros2_driver",
    ),
    preflight_required_python_modules=("navlab.companion.cli", "navlab.slam.cli"),
    preflight_required_command_groups=(("mavlink-routerd", "mavlink-router"), ("ros2",)),
)

FCU_BRIDGE_MODES = {NAVLAB_MAVLINK.name: NAVLAB_MAVLINK}


def get_fcu_bridge_mode(name: str) -> FcuBridgeModeSpec:
    normalized = name.strip().lower()
    try:
        return FCU_BRIDGE_MODES[normalized]
    except KeyError as exc:
        supported = ",".join(sorted(FCU_BRIDGE_MODES))
        raise ValueError(f"fcu_bridge_mode_unknown:{name}:supported={supported}") from exc


def registered_fcu_bridge_modes() -> tuple[str, ...]:
    return tuple(sorted(FCU_BRIDGE_MODES))


__all__ = ["FCU_BRIDGE_MODES", "FcuBridgeModeSpec", "get_fcu_bridge_mode", "registered_fcu_bridge_modes"]
