from __future__ import annotations

import argparse
import json
from math import isfinite

from lab_env.config import load_runtime_config, load_sim_config
from lab_env.sim.nodes.scan_features_publisher import DEFAULT_SCAN_FEATURES_TOPIC
from lab_env.sim.perception.contract import DEFAULT_SCAN_CONTRACT
from lab_env.sim.perception.front_sector import ForwardStopStateMachine, classify_front_min, classify_front_sector

DEFAULT_CMD_VEL_TOPIC = "/planner/cmd_vel"


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Consume /scan_features and report front-sector state.")
    parser.add_argument(
        "--source",
        choices=("auto", "features", "scan"),
        default="auto",
        help="Prefer /scan_features, or force /scan_features or raw /scan.",
    )
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
    parser.add_argument(
        "--stop-distance",
        type=float,
        default=None,
        help="Distance threshold for switching from forward to stop.",
    )
    parser.add_argument(
        "--forward-speed",
        type=float,
        default=0.2,
        help="Linear X command while the state machine is in forward.",
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default=DEFAULT_CMD_VEL_TOPIC,
        help="Twist output topic for the minimal forward/stop controller.",
    )
    return parser


def run(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if args.stop_distance is None:
        args.stop_distance = load_sim_config(load_runtime_config()).stop_distance.value

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "front_sector_consumer requires ROS2 Python packages. "
            "Source your ROS environment before running this command."
        ) from exc

    scan_features_cls = None
    if args.source != "scan":
        try:
            from ydlidar_interfaces.msg import ScanFeatures
        except ModuleNotFoundError as exc:
            if args.source == "features":
                raise SystemExit(
                    "front_sector_consumer source=features requires ydlidar_interfaces. "
                    "Run it through the NavLab companion runtime or source the overlay first."
                ) from exc
        else:
            scan_features_cls = ScanFeatures

    class FrontSectorConsumer(Node):
        def __init__(self) -> None:
            super().__init__("front_sector_consumer")
            self._controller = ForwardStopStateMachine(
                forward_speed=args.forward_speed,
                stop_distance=args.stop_distance,
            )
            self._features_subscription = None
            self._scan_subscription = None
            self._cmd_publisher = self.create_publisher(Twist, args.cmd_vel_topic, 10)

            if scan_features_cls is not None:
                self._features_subscription = self.create_subscription(
                    scan_features_cls,
                    DEFAULT_SCAN_FEATURES_TOPIC,
                    self._handle_features,
                    qos_profile_sensor_data,
                )

            if args.source in {"auto", "scan"}:
                self._scan_subscription = self.create_subscription(
                    LaserScan,
                    DEFAULT_SCAN_CONTRACT.topic_name,
                    self._handle_scan,
                    qos_profile_sensor_data,
                )

            if args.source == "features":
                self.get_logger().info(f"subscribed to {DEFAULT_SCAN_FEATURES_TOPIC}")
            elif args.source == "scan":
                self.get_logger().info(f"subscribed to {DEFAULT_SCAN_CONTRACT.topic_name}")
            elif scan_features_cls is None:
                self.get_logger().info(
                    f"{DEFAULT_SCAN_FEATURES_TOPIC} type unavailable locally, "
                    f"subscribed to {DEFAULT_SCAN_CONTRACT.topic_name}"
                )
            else:
                self.get_logger().info(
                    f"preferring {DEFAULT_SCAN_FEATURES_TOPIC}, falling back to {DEFAULT_SCAN_CONTRACT.topic_name}"
                )
            self.get_logger().info(f"publishing minimal forward/stop commands on {args.cmd_vel_topic}")

        def _handle_front_observation(
            self,
            *,
            frame_id: str,
            source: str,
            front_min: float | None,
            valid_points: int,
        ) -> None:
            perception = classify_front_min(
                front_min,
                valid_points=valid_points,
                obstacle_seen_distance=args.obstacle_seen_distance,
                avoid_distance=args.avoid_distance,
            )
            decision = self._controller.update(front_min)
            self._publish_cmd(decision.linear_x)
            payload = {
                "frame_id": frame_id,
                "source": source,
                "scan_state": perception.state,
                "motion_state": decision.motion_state,
                "cmd_linear_x": decision.linear_x,
                "decision_reason": decision.reason,
                "transitioned": decision.transitioned,
                "front_min": perception.front_min,
                "valid_points": perception.valid_points,
            }
            self.get_logger().info(json.dumps(payload, ensure_ascii=True))

        def _publish_cmd(self, linear_x: float) -> None:
            message = Twist()
            message.linear.x = linear_x
            self._cmd_publisher.publish(message)

        def _handle_features(self, message) -> None:
            self._handle_front_observation(
                frame_id=message.header.frame_id,
                source="scan_features",
                front_min=message.front_min if isfinite(message.front_min) else None,
                valid_points=message.valid_count,
            )

        def _handle_scan(self, message: LaserScan) -> None:
            if args.source == "auto" and self.count_publishers(DEFAULT_SCAN_FEATURES_TOPIC) > 0:
                return
            report = classify_front_sector(
                list(message.ranges),
                contract=DEFAULT_SCAN_CONTRACT,
                front_half_width_deg=args.front_half_width_deg,
                obstacle_seen_distance=args.obstacle_seen_distance,
                avoid_distance=args.avoid_distance,
            )
            self._handle_front_observation(
                frame_id=message.header.frame_id,
                source="scan",
                front_min=report.front_min,
                valid_points=report.valid_points,
            )

    rclpy.init(args=None)
    node = FrontSectorConsumer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
