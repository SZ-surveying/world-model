from __future__ import annotations

import argparse
import json
import os
import signal
import time
from dataclasses import dataclass

from lab_env.config import load_runtime_config, load_sim_config
from lab_env.sim.nodes.cmd_vel_executor import DEFAULT_CMD_VEL_TOPIC, DEFAULT_POSE_TOPIC
from lab_env.sim.nodes.scan_features_publisher import DEFAULT_SCAN_FEATURES_TOPIC
from lab_env.sim.status import DEFAULT_SIM_LOG_TOPIC, encode_sim_log
from lab_env.sim.waypoints import StraightLineMission, Waypoint, load_straight_line_mission

_STOP_DISTANCE_TOLERANCE = 1e-3


@dataclass(frozen=True, slots=True)
class AutoRunDecision:
    mission_state: str
    linear_x: float
    state: str
    reason: str
    active_waypoint_index: int | None


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Execute a minimal straight-line waypoint mission in sim.")
    parser.add_argument(
        "--waypoint-file",
        default=os.environ.get("SIM_AUTO_WAYPOINT_FILE"),
        help="Path to the mission yaml file mounted inside the runtime container.",
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default=DEFAULT_CMD_VEL_TOPIC,
        help="Twist output topic.",
    )
    parser.add_argument(
        "--pose-topic",
        default=DEFAULT_POSE_TOPIC,
        help="Pose topic used to track waypoint progress.",
    )
    parser.add_argument(
        "--scan-features-topic",
        default=DEFAULT_SCAN_FEATURES_TOPIC,
        help="Structured scan topic used for the stop guard.",
    )
    parser.add_argument(
        "--forward-speed",
        type=float,
        default=None,
        help="Override the mission forward speed in m/s.",
    )
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=None,
        help="Override the mission position tolerance in meters.",
    )
    parser.add_argument(
        "--stop-distance",
        type=float,
        default=None,
        help="Override the configured stop distance in meters.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Command publish rate in Hz.",
    )
    parser.add_argument(
        "--status-topic",
        default=DEFAULT_SIM_LOG_TOPIC,
        help="JSON log topic for mission state updates.",
    )
    return parser


def compute_auto_run_decision(
    *,
    mission: StraightLineMission,
    current_x: float | None,
    front_min: float | None,
    stop_distance: float,
    forward_speed: float,
    position_tolerance: float,
) -> AutoRunDecision:
    if current_x is None:
        return AutoRunDecision("ready", 0.0, "stop", "waiting_for_pose", 0)

    for index, waypoint in enumerate(mission.waypoints):
        if current_x + position_tolerance < waypoint.x:
            if front_min is None:
                return AutoRunDecision("ready", 0.0, "stop", "waiting_for_scan_features", index)
            if front_min <= stop_distance + _STOP_DISTANCE_TOLERANCE:
                return AutoRunDecision("blocked_by_stop_guard", 0.0, "stop", "stop_distance_reached", index)
            return AutoRunDecision("running", forward_speed, "forward", "heading_to_waypoint", index)

    return AutoRunDecision("complete", 0.0, "stop", "mission_complete", None)


def _stop_auto_rosbag_if_configured(timeout_sec: float = 10.0) -> None:
    pid_text = os.environ.get("SIM_AUTO_ROSBAG_PID", "").strip()
    if not pid_text:
        return

    try:
        pid = int(pid_text)
    except ValueError:
        return

    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        return

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            os.killpg(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.1)


def run(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if not args.waypoint_file:
        raise SystemExit("waypoint_follower requires --waypoint-file or SIM_AUTO_WAYPOINT_FILE")

    mission = load_straight_line_mission(args.waypoint_file)
    runtime = load_runtime_config()
    sim_config = load_sim_config(runtime)
    stop_distance = args.stop_distance if args.stop_distance is not None else sim_config.stop_distance.value
    forward_speed = args.forward_speed if args.forward_speed is not None else mission.forward_speed
    position_tolerance = args.position_tolerance if args.position_tolerance is not None else mission.position_tolerance

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, Twist
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        from ydlidar_interfaces.msg import ScanFeatures
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "waypoint_follower requires ROS2 Python packages and ydlidar_interfaces. "
            "Run it through the NavLab companion runtime or source the overlay first."
        ) from exc

    class WaypointFollower(Node):
        def __init__(self) -> None:
            super().__init__("waypoint_follower")
            self._current_x: float | None = None
            self._front_min: float | None = None
            self._last_payload = ""
            self._shutdown_requested = False
            self._publisher = self.create_publisher(Twist, args.cmd_vel_topic, 10)
            self._status_publisher = self.create_publisher(String, args.status_topic, 10)
            self.create_subscription(PoseStamped, args.pose_topic, self._handle_pose, 10)
            self.create_subscription(
                ScanFeatures,
                args.scan_features_topic,
                self._handle_scan_features,
                qos_profile_sensor_data,
            )
            self.create_timer(1.0 / args.rate, self._tick)
            self._publish_status(
                event="mission_loaded",
                mission_state="ready",
                start_x=mission.start.x,
                start_y=mission.start.y,
                start_z=mission.start.z,
                goal_x=mission.goal.x,
                goal_y=mission.goal.y,
                goal_z=mission.goal.z,
                waypoint_count=len(mission.waypoints),
                stop_distance=stop_distance,
                forward_speed=forward_speed,
            )
            self.get_logger().info(
                f"loaded straight-line mission from {args.waypoint_file}; "
                f"start=({mission.start.x:.3f}, {mission.start.y:.3f}, {mission.start.z:.3f}) "
                f"goal=({mission.goal.x:.3f}, {mission.goal.y:.3f}, {mission.goal.z:.3f}) "
                f"publishing {args.cmd_vel_topic} with stop_distance={stop_distance:.3f}"
            )

        @property
        def shutdown_requested(self) -> bool:
            return self._shutdown_requested

        def _publish_status(self, *, event: str, **fields) -> None:
            payload = encode_sim_log(source="waypoint_follower", event=event, **fields)
            message = String()
            message.data = payload
            self._status_publisher.publish(message)
            self.get_logger().info(payload)

        def _handle_pose(self, message: PoseStamped) -> None:
            self._current_x = message.pose.position.x

        def _handle_scan_features(self, message: ScanFeatures) -> None:
            value = float(message.front_min)
            self._front_min = value if value == value else None

        def _tick(self) -> None:
            decision = compute_auto_run_decision(
                mission=mission,
                current_x=self._current_x,
                front_min=self._front_min,
                stop_distance=stop_distance,
                forward_speed=forward_speed,
                position_tolerance=position_tolerance,
            )
            msg = Twist()
            msg.linear.x = decision.linear_x
            self._publisher.publish(msg)

            active_waypoint = (
                None
                if decision.active_waypoint_index is None
                else mission.waypoints[decision.active_waypoint_index]
            )
            payload = json.dumps(
                {
                    "mission_state": decision.mission_state,
                    "state": decision.state,
                    "reason": decision.reason,
                    "current_x": self._current_x,
                    "front_min": self._front_min,
                    "active_waypoint_index": decision.active_waypoint_index,
                    "active_waypoint_x": None if active_waypoint is None else active_waypoint.x,
                    "start_x": mission.start.x,
                    "goal_x": mission.goal.x,
                    "cmd_linear_x": decision.linear_x,
                },
                ensure_ascii=True,
            )
            if payload != self._last_payload:
                self._last_payload = payload
                self._publish_status(
                    event=decision.reason,
                    state=decision.state,
                    mission_state=decision.mission_state,
                    current_x=self._current_x,
                    front_min=self._front_min,
                    active_waypoint_index=decision.active_waypoint_index,
                    active_waypoint_x=None if active_waypoint is None else active_waypoint.x,
                    start_x=mission.start.x,
                    goal_x=mission.goal.x,
                    cmd_linear_x=decision.linear_x,
                )
            if decision.mission_state in {"blocked_by_stop_guard", "complete"} and not self._shutdown_requested:
                self._shutdown_requested = True
                _stop_auto_rosbag_if_configured()
                rclpy.try_shutdown()

    rclpy.init(args=None)
    node = WaypointFollower()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
