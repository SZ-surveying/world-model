from __future__ import annotations

from pathlib import Path


def test_navlab_cartographer_lua_follows_official_ardupilot_baseline() -> None:
    config = Path(
        "navlab/common/slam/ros/localization/navlab_cartographer_adapter/config/navlab_cartographer_2d.lua"
    ).read_text(encoding="utf-8")

    assert 'tracking_frame = "imu_link"' in config
    assert 'published_frame = "base_link"' in config
    assert "provide_odom_frame = true" in config
    assert "publish_frame_projected_to_2d = false" in config
    assert "use_odometry = true" in config
    assert "TRAJECTORY_BUILDER_2D.max_range = 30" in config
    assert "TRAJECTORY_BUILDER_2D.use_imu_data = false" in config
    assert "POSE_GRAPH.optimize_every_n_nodes = 30" in config
