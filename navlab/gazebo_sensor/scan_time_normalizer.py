from __future__ import annotations

import copy
import time

from navlab.common.logging import configure_sim_logging, logger
from navlab.gazebo_sensor.config import X2SensorRuntimeConfig


def run() -> int:
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
    except ModuleNotFoundError as exc:
        raise SystemExit("scan time normalizer requires ROS2 Python packages.") from exc

    config = X2SensorRuntimeConfig.load()
    if config.vendor_scan_topic == config.scan_topic:
        logger.error("vendor_scan_topic and scan_topic must be different: {}", config.scan_topic)
        return 22

    class ScanTimeNormalizerNode(Node):
        def __init__(self) -> None:
            super().__init__("navlab_x2_scan_time_normalizer")
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            self._latest_clock: Clock | None = None
            self._count = 0
            self._started_at = time.monotonic()
            self._publisher = self.create_publisher(LaserScan, config.scan_topic, qos_profile_sensor_data)
            self.create_subscription(Clock, "/clock", self._handle_clock, 10)
            self.create_subscription(LaserScan, config.vendor_scan_topic, self._handle_scan, qos_profile_sensor_data)
            self.create_timer(2.0, self._log_status)
            logger.info(
                "normalizing X2 scan timestamps input={} output={}",
                config.vendor_scan_topic,
                config.scan_topic,
            )

        def _handle_clock(self, message: Clock) -> None:
            self._latest_clock = message

        def _handle_scan(self, message: LaserScan) -> None:
            output = copy.deepcopy(message)
            if self._latest_clock is not None:
                output.header.stamp = self._latest_clock.clock
            else:
                output.header.stamp = self.get_clock().now().to_msg()
            self._publisher.publish(output)
            self._count += 1

        def _log_status(self) -> None:
            elapsed = max(0.001, time.monotonic() - self._started_at)
            logger.info(
                "scan time normalizer count={} rate_hz={:.2f} clock_seen={}",
                self._count,
                self._count / elapsed,
                self._latest_clock is not None,
            )

    configure_sim_logging()
    rclpy.init(args=None)
    node = ScanTimeNormalizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
