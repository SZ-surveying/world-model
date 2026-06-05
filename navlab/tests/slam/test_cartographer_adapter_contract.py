from __future__ import annotations

from pathlib import Path


def test_cartographer_2d_adapter_marks_vertical_and_tilt_uncertain() -> None:
    source = Path(
        "navlab/slam/ros/localization/navlab_cartographer_adapter/src/"
        "navlab_cartographer_adapter_node.cpp"
    ).read_text(encoding="utf-8")

    for index in (14, 21, 28):
        assert f"odom.pose.covariance[{index}] = 9999.0" in source
        assert f"odom.twist.covariance[{index}] = 9999.0" in source
