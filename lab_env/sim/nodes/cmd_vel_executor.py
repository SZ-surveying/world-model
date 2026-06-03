from __future__ import annotations

import argparse
import subprocess
from dataclasses import dataclass
from math import cos, sin

from lab_env.config import load_runtime_config, load_sim_config
from lab_env.sim.status import DEFAULT_SIM_LOG_TOPIC, encode_sim_log
from lab_env.sim.world.world_markers import compute_forward_clearance, load_world_obstacle_boxes

DEFAULT_WORLD_NAME = "uav_obstacle_5m"
DEFAULT_VISUAL_MODEL_NAME = "uav_start_marker"
DEFAULT_CMD_VEL_TOPIC = "/planner/cmd_vel"
DEFAULT_POSE_TOPIC = "/sim/uav_pose"
DEFAULT_WORLD_FILE = "docker/worlds/uav_obstacle_5m.sdf"


@dataclass(slots=True)
class PlanarPoseState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True, slots=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


def integrate_planar_pose(state: PlanarPoseState, command: VelocityCommand, dt: float) -> PlanarPoseState:
    return PlanarPoseState(
        x=state.x + command.linear_x * cos(state.yaw) * dt,
        y=state.y + command.linear_x * sin(state.yaw) * dt,
        z=state.z,
        yaw=state.yaw + command.angular_z * dt,
    )


def clamp_forward_speed_for_clearance(
    linear_x: float,
    *,
    dt: float,
    front_min: float | None,
    min_front_distance: float,
) -> float:
    if linear_x <= 0.0 or front_min is None or dt <= 0.0:
        return linear_x

    remaining_clearance = max(front_min - min_front_distance, 0.0)
    max_linear_x = remaining_clearance / dt
    return min(linear_x, max_linear_x)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, sin(half_yaw), cos(half_yaw))


def _build_set_pose_request(model_name: str, pose: PlanarPoseState) -> str:
    qx, qy, qz, qw = _quaternion_from_yaw(pose.yaw)
    return (
        f'name: "{model_name}" '
        f'position {{ x: {pose.x} y: {pose.y} z: {pose.z} }} '
        f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}'
    )


class GazeboPoseCommander:
    def __init__(self, *, world_name: str, timeout_ms: int = 1000) -> None:
        self._service_name = f"/world/{world_name}/set_pose"
        self._timeout_ms = timeout_ms

    @property
    def service_name(self) -> str:
        return self._service_name

    def set_pose(self, *, model_name: str, pose: PlanarPoseState) -> None:
        request = _build_set_pose_request(model_name, pose)
        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                self._service_name,
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                str(self._timeout_ms),
                "--req",
                request,
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Consume /planner/cmd_vel and move the sim lidar rig in Gazebo.")
    parser.add_argument(
        "--cmd-vel-topic",
        default=DEFAULT_CMD_VEL_TOPIC,
        help="Twist input topic.",
    )
    parser.add_argument(
        "--world-name",
        default=DEFAULT_WORLD_NAME,
        help="Gazebo world name used for the /set_pose service.",
    )
    parser.add_argument(
        "--visual-model-name",
        default=DEFAULT_VISUAL_MODEL_NAME,
        help="Gazebo model name to reposition so the UAV body visibly advances in sim.",
    )
    parser.add_argument(
        "--visual-model-z",
        type=float,
        default=0.1,
        help="Z position used for the visual UAV marker model.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Pose integration and set_pose push rate in Hz.",
    )
    parser.add_argument(
        "--cmd-timeout-sec",
        type=float,
        default=0.5,
        help="Stop the rig if no Twist arrives within this timeout.",
    )
    parser.add_argument(
        "--pose-topic",
        default=DEFAULT_POSE_TOPIC,
        help="Pose topic consumed by sim-side world-model publishers.",
    )
    parser.add_argument(
        "--world-file",
        default=DEFAULT_WORLD_FILE,
        help="World SDF used by the minimum-clearance gate.",
    )
    parser.add_argument(
        "--min-front-distance",
        type=float,
        default=None,
        help="Minimum allowed forward obstacle clearance in meters.",
    )
    parser.add_argument(
        "--status-topic",
        default=DEFAULT_SIM_LOG_TOPIC,
        help="JSON log topic for executor state updates.",
    )
    parser.add_argument(
        "--status-period-sec",
        type=float,
        default=0.5,
        help="Heartbeat period for executor status logs.",
    )
    return parser


def run(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if args.min_front_distance is None:
        args.min_front_distance = load_sim_config(load_runtime_config()).stop_distance.value

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, Twist
        from rclpy.node import Node
        from std_msgs.msg import String
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "cmd_vel_executor requires ROS2 Python packages. "
            "Source your ROS environment before running this command."
        ) from exc

    class CmdVelExecutor(Node):
        def __init__(self) -> None:
            super().__init__("cmd_vel_executor")
            self._pose = PlanarPoseState()
            self._command = VelocityCommand()
            self._last_command_time = None
            self._last_tick_time = self.get_clock().now()
            self._last_status_time = self.get_clock().now()
            self._last_status_reason = ""
            self._world_boxes = load_world_obstacle_boxes(args.world_file)
            self._pose_commander = GazeboPoseCommander(world_name=args.world_name)
            self._pose_publisher = self.create_publisher(PoseStamped, args.pose_topic, 10)
            self._status_publisher = self.create_publisher(String, args.status_topic, 10)
            self.create_subscription(Twist, args.cmd_vel_topic, self._handle_cmd_vel, 10)
            self.create_timer(1.0 / args.rate, self._tick)
            self._push_pose()
            self._publish_status(
                event="executor_ready",
                reason="waiting_for_cmd",
                requested_linear_x=0.0,
                applied_linear_x=0.0,
                front_clearance=None,
                force=True,
            )
            self.get_logger().info(
                f"subscribed to {args.cmd_vel_topic}, publishing {args.pose_topic}, "
                f"controlling {args.visual_model_name} via {self._pose_commander.service_name}"
            )

        def _handle_cmd_vel(self, message: Twist) -> None:
            self._command = VelocityCommand(
                linear_x=message.linear.x,
                angular_z=message.angular.z,
            )
            self._last_command_time = self.get_clock().now()
            self._publish_status(
                event="cmd_received",
                reason="cmd_received",
                requested_linear_x=self._command.linear_x,
                applied_linear_x=self._command.linear_x,
                front_clearance=None,
                force=True,
            )

        def _publish_status(
            self,
            *,
            event: str,
            reason: str,
            requested_linear_x: float,
            applied_linear_x: float,
            front_clearance: float | None,
            force: bool = False,
        ) -> None:
            now = self.get_clock().now()
            elapsed_sec = (now - self._last_status_time).nanoseconds / 1_000_000_000
            if not force and reason == self._last_status_reason and elapsed_sec < args.status_period_sec:
                return

            payload = encode_sim_log(
                source="cmd_vel_executor",
                event=event,
                sim_mode="runtime",
                reason=reason,
                current_x=self._pose.x,
                current_y=self._pose.y,
                current_z=self._pose.z,
                yaw=self._pose.yaw,
                requested_linear_x=requested_linear_x,
                applied_linear_x=applied_linear_x,
                front_clearance=front_clearance,
                min_front_distance=args.min_front_distance,
            )
            message = String()
            message.data = payload
            self._status_publisher.publish(message)
            self._last_status_time = now
            self._last_status_reason = reason
            self.get_logger().info(payload)

        def _active_command(self) -> VelocityCommand:
            if self._last_command_time is None:
                return VelocityCommand()
            age_sec = (self.get_clock().now() - self._last_command_time).nanoseconds / 1_000_000_000
            if age_sec > args.cmd_timeout_sec:
                return VelocityCommand()
            return self._command

        def _tick(self) -> None:
            now = self.get_clock().now()
            dt = (now - self._last_tick_time).nanoseconds / 1_000_000_000
            self._last_tick_time = now
            if dt <= 0.0:
                return

            command = self._active_command()
            front_clearance = compute_forward_clearance(
                self._world_boxes,
                origin_x=self._pose.x,
                origin_y=self._pose.y,
                origin_z=args.visual_model_z,
                yaw=self._pose.yaw,
            )
            linear_x = clamp_forward_speed_for_clearance(
                command.linear_x,
                dt=dt,
                front_min=front_clearance,
                min_front_distance=args.min_front_distance,
            )
            safe_command = VelocityCommand(
                linear_x=linear_x,
                angular_z=command.angular_z,
            )
            if self._last_command_time is None:
                status_reason = "waiting_for_cmd"
            else:
                age_sec = (now - self._last_command_time).nanoseconds / 1_000_000_000
                if age_sec > args.cmd_timeout_sec:
                    status_reason = "cmd_timeout"
                elif safe_command.linear_x + 1e-9 < command.linear_x:
                    status_reason = "stop_guard_clamped"
                elif abs(safe_command.linear_x) > 1e-9 or abs(safe_command.angular_z) > 1e-9:
                    status_reason = "executing_cmd"
                else:
                    status_reason = "idle"
            self._pose = integrate_planar_pose(self._pose, safe_command, dt)
            self._push_pose()
            self._publish_status(
                event="executor_state",
                reason=status_reason,
                requested_linear_x=command.linear_x,
                applied_linear_x=safe_command.linear_x,
                front_clearance=front_clearance,
            )

        def _push_pose(self) -> None:
            visual_pose = PlanarPoseState(
                x=self._pose.x,
                y=self._pose.y,
                z=args.visual_model_z,
                yaw=self._pose.yaw,
            )
            self._pose_commander.set_pose(model_name=args.visual_model_name, pose=visual_pose)

            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "map"
            message.pose.position.x = self._pose.x
            message.pose.position.y = self._pose.y
            message.pose.position.z = args.visual_model_z
            qx, qy, qz, qw = _quaternion_from_yaw(self._pose.yaw)
            message.pose.orientation.x = qx
            message.pose.orientation.y = qy
            message.pose.orientation.z = qz
            message.pose.orientation.w = qw
            self._pose_publisher.publish(message)

    rclpy.init(args=None)
    node = CmdVelExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
