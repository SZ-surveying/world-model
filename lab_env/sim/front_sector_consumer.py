from __future__ import annotations

import argparse
import json
from typing import NoReturn

from lab_env.sim.contract import DEFAULT_SCAN_CONTRACT
from lab_env.sim.front_sector import classify_front_sector


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Consume /scan and report front-sector state.")
    parser.add_argument(
        "--front-half-width-deg",
        type=float,
        default=15.0,
        help="Half width of the front sector in degrees.",
    )
    parser.add_argument(
        "--obstacle-seen-distance",
        type=float,
        default=6.0,
        help="Threshold for reporting obstacle_seen.",
    )
    parser.add_argument(
        "--avoid-distance",
        type=float,
        default=1.0,
        help="Threshold for reporting avoid_required.",
    )
    return parser


def main() -> NoReturn:
    args = _build_arg_parser().parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "front_sector_consumer requires ROS2 Python packages. "
            "Source your ROS environment before running this command."
        ) from exc

    class FrontSectorConsumer(Node):
        def __init__(self) -> None:
            super().__init__("front_sector_consumer")
            self.create_subscription(
                LaserScan,
                DEFAULT_SCAN_CONTRACT.topic_name,
                self._handle_scan,
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"subscribed to {DEFAULT_SCAN_CONTRACT.topic_name}")

        def _handle_scan(self, message: LaserScan) -> None:
            report = classify_front_sector(
                list(message.ranges),
                contract=DEFAULT_SCAN_CONTRACT,
                front_half_width_deg=args.front_half_width_deg,
                obstacle_seen_distance=args.obstacle_seen_distance,
                avoid_distance=args.avoid_distance,
            )
            payload = {
                "frame_id": message.header.frame_id,
                "state": report.state,
                "front_min": report.front_min,
                "valid_points": report.valid_points,
            }
            self.get_logger().info(json.dumps(payload, ensure_ascii=True))

    rclpy.init(args=None)
    node = FrontSectorConsumer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0)


if __name__ == "__main__":
    main()
