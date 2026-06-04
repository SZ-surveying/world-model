from __future__ import annotations

import tomllib

from navlab.companion.config import RuntimeConfig, load_config


def test_companion_config_reads_companion_section() -> None:
    config = load_config()

    assert config.stop_distance.value == 0.5
    assert config.console_log_level.value == "DEBUG"
    assert config.file_log_level.value == "INFO"


def test_runtime_config_world_markers_follow_navlab_quad_root() -> None:
    config = RuntimeConfig.load()

    assert "--root-model-name" in config.world_markers.args
    assert "navlab_iq_quad" in config.world_markers.args
    assert "--frame-id" in config.world_markers.args
    assert "navlab_world" in config.world_markers.args
    assert "--set-gazebo-pose" not in config.pose_mirror.args
    assert "--world-name" not in config.pose_mirror.args
    assert "--model-name" not in config.pose_mirror.args
    assert "--pose-frame-id" in config.pose_mirror.args
    assert "--map-frame-id" in config.pose_mirror.args
    assert "--sensor-base-frame-id" in config.pose_mirror.args
    assert "--laser-frame-id" in config.pose_mirror.args
    assert "--front-center-deg" in config.scan_features.args
    front_flag = config.scan_features.args.index("--front-center-deg")
    rear_flag = config.scan_features.args.index("--rear-center-deg")
    assert config.scan_features.args[front_flag + 1] == "0"
    assert config.scan_features.args[rear_flag + 1] == "180"
    assert config.gazebo_truth_bridge.autostart is True
    assert "dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" in config.gazebo_truth_bridge.args[0]
    assert config.gazebo_truth_odom.autostart is True
    assert "/gazebo/truth/odom" in config.gazebo_truth_odom.args
    index_flag = config.gazebo_truth_odom.args.index("--transform-index")
    assert config.gazebo_truth_odom.args[index_flag + 1] == "0"
    assert "--pass-x-m" in config.mission.args
    pass_x_flag = config.mission.args.index("--pass-x-m")
    avoid_distance_flag = config.mission.args.index("--obstacle-avoid-distance-m")
    assert config.mission.args[pass_x_flag + 1] == "1.25"
    assert config.mission.args[avoid_distance_flag + 1] == "1.2"


def test_companion_dependency_group_includes_numpy_for_ros_python() -> None:
    pyproject = tomllib.loads(open("navlab/pyproject.toml", encoding="utf-8").read())
    companion_deps = pyproject["dependency-groups"]["companion"]

    assert any(dep.startswith("numpy") for dep in companion_deps)
