from __future__ import annotations

from navlab.companion.acceptance import _scan_publisher_summary


def test_scan_publisher_summary_reads_only_publishers() -> None:
    samples = """SCAN_TOPIC_INFO
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Node name: ydlidar_ros2_driver_node
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Subscription count: 1
Node name: scan_features_publisher
Node namespace: /
"""

    summary = _scan_publisher_summary(samples)

    assert summary["publisher_nodes"] == ["ydlidar_ros2_driver_node"]
    assert summary["vendor_driver_publisher"] is True
    assert summary["emulator_publisher"] is False
