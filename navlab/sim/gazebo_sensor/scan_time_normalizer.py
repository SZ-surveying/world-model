from __future__ import annotations

import copy
import time

from navlab.common.logging import configure_sim_logging, logger
from navlab.sim.gazebo_sensor.config import X2SensorRuntimeConfig


def stamp_to_nanoseconds(stamp: object) -> int:
    return int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))


def monotonic_scan_stamp_ns(
    *,
    preferred_ns: int,
    fallback_elapsed_sec: float,
    previous_ns: int | None,
    min_increment_ns: int = 1,
) -> int:
    fallback_ns = 1_000_000_000 + max(0, int(fallback_elapsed_sec * 1_000_000_000))
    candidate_ns = preferred_ns if preferred_ns > 0 else fallback_ns
    if previous_ns is None:
        return candidate_ns
    return max(candidate_ns, previous_ns + max(1, min_increment_ns))


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
            self._last_stamp_ns: int | None = None
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
            preferred_ns = 0
            if self._latest_clock is not None:
                preferred_ns = stamp_to_nanoseconds(self._latest_clock.clock)
            if preferred_ns <= 0:
                preferred_ns = stamp_to_nanoseconds(message.header.stamp)
            scan_duration_sec = max(
                float(message.scan_time),
                float(message.time_increment) * len(message.ranges),
                0.0,
            )
            scan_duration_ns = int(scan_duration_sec * 1_000_000_000)
            stamp_ns = monotonic_scan_stamp_ns(
                preferred_ns=preferred_ns,
                fallback_elapsed_sec=time.monotonic() - self._started_at,
                previous_ns=self._last_stamp_ns,
                min_increment_ns=scan_duration_ns,
            )
            output.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            output.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
            self._last_stamp_ns = stamp_ns
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
