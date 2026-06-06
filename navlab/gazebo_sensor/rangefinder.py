from __future__ import annotations

import argparse
import json
import math
import time
from collections.abc import Sequence
from dataclasses import dataclass

from navlab.common.logging import configure_sim_logging, logger
from navlab.gazebo_sensor.config import DownRangefinderRuntimeConfig


@dataclass(frozen=True, slots=True)
class RangefinderReading:
    distance_m: float
    stamp_monotonic: float


def select_down_range_m(ranges: Sequence[float], *, min_m: float, max_m: float) -> float | None:
    valid = [float(value) for value in ranges if min_m <= float(value) <= max_m and math.isfinite(float(value))]
    if not valid:
        return None
    return min(valid)


def meters_to_centimeters(distance_m: float, *, min_m: float, max_m: float) -> int:
    clamped = min(max(distance_m, min_m), max_m)
    return int(round(clamped * 100.0))


def mavlink_constant(module: object, name: str) -> int:
    value = getattr(module, name, None)
    if not isinstance(value, int):
        raise ValueError(f"Unknown MAVLink constant: {name}")
    return value


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send Gazebo down rangefinder readings as MAVLink DISTANCE_SENSOR.")
    parser.add_argument("--log-file")
    return parser


def run(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    configure_sim_logging(log_file=args.log_file)
    config = DownRangefinderRuntimeConfig.load()

    try:
        import rclpy
        from pymavlink import mavutil
        from pymavlink.dialects.v20 import ardupilotmega as mavlink
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan, Range
        from std_msgs.msg import String
    except ModuleNotFoundError as exc:
        raise SystemExit("down rangefinder runtime requires ROS2 and pymavlink packages.") from exc

    class DownRangefinderNode(Node):
        def __init__(self) -> None:
            super().__init__("down_rangefinder_mavlink_sender")
            self._mavlink_orientation = mavlink_constant(mavlink, config.mavlink_orientation)
            self._connection = mavutil.mavlink_connection(
                config.endpoint,
                source_system=config.source_system,
                source_component=config.source_component,
                dialect="ardupilotmega",
            )
            self._mavlink_peer_observed = self._wait_for_mavlink_peer()
            self._latest: RangefinderReading | None = None
            self._input_count = 0
            self._sent_count = 0
            self._last_sent_monotonic = 0.0
            self._range_pub = self.create_publisher(Range, config.range_topic, 10)
            self._status_pub = self.create_publisher(String, config.status_topic, 10)
            self.create_subscription(LaserScan, config.scan_ideal_topic, self._handle_scan, 10)
            self.create_timer(1.0 / max(config.rate_hz, 0.1), self._send_distance_sensor)
            self.create_timer(0.5, self._publish_status)
            logger.info(
                "down rangefinder started scan_topic={} range_topic={} endpoint={} rate={}Hz",
                config.scan_ideal_topic,
                config.range_topic,
                config.endpoint,
                config.rate_hz,
            )

        def _wait_for_mavlink_peer(self) -> bool:
            if not config.endpoint.startswith(("udpin:", "tcp:")):
                return True
            logger.info("down rangefinder waiting for MAVLink peer on {}", config.endpoint)
            try:
                heartbeat = self._connection.wait_heartbeat(timeout=5)
            except Exception as exc:
                logger.warning("down rangefinder did not observe MAVLink heartbeat: {}", exc)
                return False
            observed = heartbeat is not None
            logger.info("down rangefinder MAVLink peer observed={}", observed)
            return observed

        def _handle_scan(self, msg: LaserScan) -> None:
            distance = select_down_range_m(msg.ranges, min_m=config.min_distance_m, max_m=config.max_distance_m)
            if distance is None:
                return
            self._input_count += 1
            now = time.monotonic()
            self._latest = RangefinderReading(distance_m=distance, stamp_monotonic=now)
            output = Range()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = config.frame_id
            output.radiation_type = Range.INFRARED
            output.field_of_view = 0.0
            output.min_range = config.min_distance_m
            output.max_range = config.max_distance_m
            output.range = distance
            self._range_pub.publish(output)

        def _send_distance_sensor(self) -> None:
            reading = self._latest
            if reading is None:
                return
            distance_cm = meters_to_centimeters(
                reading.distance_m,
                min_m=config.min_distance_m,
                max_m=config.max_distance_m,
            )
            self._connection.mav.distance_sensor_send(
                int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
                meters_to_centimeters(config.min_distance_m, min_m=config.min_distance_m, max_m=config.max_distance_m),
                meters_to_centimeters(config.max_distance_m, min_m=config.min_distance_m, max_m=config.max_distance_m),
                distance_cm,
                mavlink.MAV_DISTANCE_SENSOR_LASER,
                config.sensor_id,
                self._mavlink_orientation,
                config.covariance_cm,
            )
            self._sent_count += 1
            self._last_sent_monotonic = time.monotonic()

        def _publish_status(self) -> None:
            now = time.monotonic()
            latest_age = None if self._latest is None else now - self._latest.stamp_monotonic
            sent_age = None if self._last_sent_monotonic <= 0 else now - self._last_sent_monotonic
            state = "sending" if self._sent_count > 0 and latest_age is not None and latest_age <= 1.0 else "waiting"
            message = String()
            message.data = json.dumps(
                {
                    "state": state,
                    "ready": state == "sending",
                    "source": "gazebo_down_rangefinder",
                    "scan_ideal_topic": config.scan_ideal_topic,
                    "range_topic": config.range_topic,
                    "endpoint": config.endpoint,
                    "frame_id": config.frame_id,
                    "mavlink_orientation": config.mavlink_orientation,
                    "source_system": config.source_system,
                    "source_component": config.source_component,
                    "sensor_id": config.sensor_id,
                    "mavlink_peer_observed": self._mavlink_peer_observed,
                    "input_count": self._input_count,
                    "sent_count": self._sent_count,
                    "latest_distance_m": None if self._latest is None else self._latest.distance_m,
                    "latest_input_age_sec": latest_age,
                    "latest_sent_age_sec": sent_age,
                    "mavlink_message": "DISTANCE_SENSOR",
                    "min_distance_m": config.min_distance_m,
                    "max_distance_m": config.max_distance_m,
                    "covariance_cm": config.covariance_cm,
                },
                ensure_ascii=True,
                sort_keys=True,
            )
            self._status_pub.publish(message)

    rclpy.init(args=None)
    node = DownRangefinderNode()
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
