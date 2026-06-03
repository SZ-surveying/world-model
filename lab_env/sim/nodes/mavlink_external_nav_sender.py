from __future__ import annotations

import argparse
import json
import math
import os
import time
from typing import Sequence

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


MAVLINK_TIME_SOURCE = "sender_monotonic_clock_us"
MAVLINK_POSITION_FRAME = "MAV_FRAME_LOCAL_FRD"
MAVLINK_VELOCITY_FRAME = "MAV_FRAME_BODY_FRD"
MAVLINK_ESTIMATOR_TYPE = "MAV_ESTIMATOR_TYPE_VIO"
ROS_ODOM_SEMANTICS = "ROS ENU position with FLU body twist"


def _upper_triangular_covariance(covariance: Sequence[float]) -> list[float]:
    if len(covariance) != 36:
        return [0.0] * 21

    values: list[float] = []
    for row in range(6):
        for col in range(row, 6):
            value = covariance[(row * 6) + col]
            values.append(0.0 if math.isnan(value) else float(value))
    return values


def _ros_quat_to_frd(q: object) -> list[float]:
    # ROS odom is treated as ENU/FLU. MAVLink output is LOCAL_FRD/BODY_FRD.
    return [float(q.w), float(q.x), -float(q.y), -float(q.z)]


def _odometry_mapping_status(
    *, input_topic: str, quality: int, reset_counter: int, rate_hz: float, source_system: int
) -> dict[str, object]:
    return {
        "input": {
            "topic": input_topic,
            "message": "nav_msgs/msg/Odometry",
            "semantics": ROS_ODOM_SEMANTICS,
        },
        "output": {
            "message": "MAVLink v2 ODOMETRY",
            "position_frame": MAVLINK_POSITION_FRAME,
            "velocity_frame": MAVLINK_VELOCITY_FRAME,
            "estimator_type": MAVLINK_ESTIMATOR_TYPE,
            "component": "MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY",
            "source_system": source_system,
            "rate_hz": rate_hz,
            "quality": quality,
            "reset_counter": reset_counter,
            "time_usec_source": MAVLINK_TIME_SOURCE,
        },
        "field_map": {
            "time_usec": MAVLINK_TIME_SOURCE,
            "x": "odom.pose.pose.position.x",
            "y": "-odom.pose.pose.position.y",
            "z": "-odom.pose.pose.position.z",
            "q": "[w, x, -y, -z] from odom.pose.pose.orientation",
            "vx": "odom.twist.twist.linear.x",
            "vy": "-odom.twist.twist.linear.y",
            "vz": "-odom.twist.twist.linear.z",
            "rollspeed": "odom.twist.twist.angular.x",
            "pitchspeed": "-odom.twist.twist.angular.y",
            "yawspeed": "-odom.twist.twist.angular.z",
            "pose_covariance": "upper triangular odom.pose.covariance",
            "velocity_covariance": "upper triangular odom.twist.covariance",
        },
    }


class MavlinkExternalNavSender(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mavlink_external_nav_sender")
        self._endpoint = args.endpoint
        self._rate_hz = args.rate_hz
        self._quality = args.quality
        self._reset_counter = args.reset_counter
        self._source_system = args.source_system
        self._odom_topic = args.odom_topic
        self._connection = mavutil.mavlink_connection(
            self._endpoint,
            source_system=self._source_system,
            source_component=mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
            dialect="ardupilotmega",
        )
        self._last_odom: Odometry | None = None
        self._last_odom_rx_monotonic = 0.0
        self._sent_count = 0
        self._last_heartbeat_monotonic = 0.0

        self.create_subscription(Odometry, args.odom_topic, self._handle_odom, 10)
        self._status_pub = self.create_publisher(String, args.status_topic, 10)
        self.create_timer(1.0 / self._rate_hz, self._send_tick)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            "mavlink_external_nav_sender started "
            f"endpoint={self._endpoint} odom_topic={args.odom_topic} rate={self._rate_hz:.3f}Hz "
            f"quality={self._quality} reset_counter={self._reset_counter}"
        )

    def _handle_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        self._last_odom_rx_monotonic = time.monotonic()

    def _send_tick(self) -> None:
        now_monotonic = time.monotonic()
        if now_monotonic - self._last_heartbeat_monotonic >= 1.0:
            self._connection.mav.heartbeat_send(
                mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavlink.MAV_STATE_ACTIVE,
            )
            self._last_heartbeat_monotonic = now_monotonic

        if self._last_odom is None:
            return

        odom = self._last_odom
        pose = odom.pose.pose
        twist = odom.twist.twist
        time_usec = int(time.monotonic() * 1000000)

        self._connection.mav.odometry_send(
            int(time_usec),
            mavlink.MAV_FRAME_LOCAL_FRD,
            mavlink.MAV_FRAME_BODY_FRD,
            float(pose.position.x),
            -float(pose.position.y),
            -float(pose.position.z),
            _ros_quat_to_frd(pose.orientation),
            float(twist.linear.x),
            -float(twist.linear.y),
            -float(twist.linear.z),
            float(twist.angular.x),
            -float(twist.angular.y),
            -float(twist.angular.z),
            _upper_triangular_covariance(odom.pose.covariance),
            _upper_triangular_covariance(odom.twist.covariance),
            int(self._reset_counter),
            mavlink.MAV_ESTIMATOR_TYPE_VIO,
            int(self._quality),
        )
        self._sent_count += 1

    def _publish_status(self) -> None:
        age_ms = -1.0
        if self._last_odom is not None:
            age_ms = (time.monotonic() - self._last_odom_rx_monotonic) * 1000.0

        status = {
            "state": "sending" if self._last_odom is not None else "waiting_for_external_nav_odom",
            "endpoint": self._endpoint,
            "input_topic": self._odom_topic,
            "sent_count": self._sent_count,
            "rate_hz": self._rate_hz,
            "odom_age_ms": round(age_ms, 3),
            "frame_id": self._last_odom.header.frame_id if self._last_odom else "",
            "child_frame_id": self._last_odom.child_frame_id if self._last_odom else "",
            "mav_frame_id": MAVLINK_POSITION_FRAME,
            "mav_child_frame_id": MAVLINK_VELOCITY_FRAME,
            "quality": self._quality,
            "reset_counter": self._reset_counter,
            "estimator_type": MAVLINK_ESTIMATOR_TYPE,
            "time_usec_source": MAVLINK_TIME_SOURCE,
            "mapping": _odometry_mapping_status(
                input_topic=self._odom_topic,
                quality=self._quality,
                reset_counter=self._reset_counter,
                rate_hz=self._rate_hz,
                source_system=self._source_system,
            ),
        }
        msg = String()
        msg.data = json.dumps(status, separators=(",", ":"))
        self._status_pub.publish(msg)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send ROS ExternalNav odom as MAVLink ODOMETRY.")
    parser.add_argument("--endpoint", default="udpout:mavlink-router:14550")
    parser.add_argument("--odom-topic", default="/external_nav/odom")
    parser.add_argument("--status-topic", default="/mavlink_external_nav/status")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--quality", type=int, default=100)
    parser.add_argument("--reset-counter", type=int, default=0)
    parser.add_argument("--source-system", type=int, default=191)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init(args=None)
    node = MavlinkExternalNavSender(args)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
