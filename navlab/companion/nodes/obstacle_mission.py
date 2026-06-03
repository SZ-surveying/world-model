from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from navlab.sim.status import DEFAULT_SIM_LOG_TOPIC, encode_sim_log

os.environ.setdefault("MAVLINK20", "1")

XY_VELOCITY_Z_POSITION_TYPE_MASK = 3555


@dataclass(frozen=True, slots=True)
class MissionInputs:
    current_x: float | None
    current_y: float | None
    current_z_ned: float | None
    front_min: float | None
    external_nav_ready: bool
    imu_ready: bool
    scan_fresh: bool
    expected_mode_seen: bool
    armed_seen: bool
    airborne_seen: bool
    airborne_elapsed_sec: float | None


@dataclass(frozen=True, slots=True)
class MissionConfig:
    takeoff_alt_m: float = 0.8
    hover_settle_sec: float = 4.0
    forward_speed_mps: float = 0.15
    avoid_forward_speed_mps: float = 0.05
    avoid_lateral_speed_mps: float = 0.15
    obstacle_seen_distance_m: float = 2.0
    avoid_y_m: float = 2.6
    pass_x_m: float = 6.5
    return_y_m: float = 0.1
    final_hold_sec: float = 5.0


@dataclass(frozen=True, slots=True)
class MissionDecision:
    phase: str
    mission_state: str
    reason: str
    vx_mps: float
    vy_mps: float
    should_set_guided: bool = False
    should_arm: bool = False
    should_takeoff: bool = False
    terminal: bool = False


def decide_obstacle_mission(inputs: MissionInputs, config: MissionConfig) -> MissionDecision:
    ready = inputs.external_nav_ready and inputs.imu_ready and inputs.scan_fresh
    if not ready:
        return MissionDecision("wait_ready", "ready", "waiting_for_inputs", 0.0, 0.0)
    if not inputs.expected_mode_seen:
        return MissionDecision("guided", "arming", "setting_guided", 0.0, 0.0, should_set_guided=True)
    if not inputs.armed_seen:
        return MissionDecision("arm", "arming", "arming_vehicle", 0.0, 0.0, should_arm=True)
    if not inputs.airborne_seen:
        return MissionDecision("takeoff", "takeoff", "taking_off", 0.0, 0.0, should_takeoff=True)
    if inputs.airborne_elapsed_sec is None or inputs.airborne_elapsed_sec < config.hover_settle_sec:
        return MissionDecision("hover_settle", "running", "hover_settle", 0.0, 0.0)

    x = inputs.current_x or 0.0
    y = inputs.current_y or 0.0
    front_min = inputs.front_min
    obstacle_seen = front_min is not None and front_min <= config.obstacle_seen_distance_m

    if x >= config.pass_x_m and y > config.return_y_m:
        return MissionDecision(
            "return_track",
            "running",
            "return_to_track",
            config.avoid_forward_speed_mps,
            -config.avoid_lateral_speed_mps,
        )
    if y < config.avoid_y_m and (obstacle_seen or x >= 3.0):
        return MissionDecision(
            "avoid",
            "running",
            "avoid_left",
            config.avoid_forward_speed_mps,
            config.avoid_lateral_speed_mps,
        )
    if x < config.pass_x_m:
        if y >= config.avoid_y_m:
            return MissionDecision("pass_obstacle", "running", "pass_obstacle", config.forward_speed_mps, 0.0)
        return MissionDecision("forward", "running", "forward_progress", config.forward_speed_mps, 0.0)
    if inputs.airborne_elapsed_sec < config.hover_settle_sec + config.final_hold_sec:
        return MissionDecision("final_hold", "running", "final_hold", 0.0, 0.0)
    return MissionDecision("complete", "complete", "mission_complete", 0.0, 0.0, terminal=True)


def encode_mission_status(
    *,
    decision: MissionDecision,
    inputs: MissionInputs,
    setpoints_sent_count: int,
    obstacle_detected: bool,
) -> str:
    return json.dumps(
        {
            "phase": decision.phase,
            "mission_state": decision.mission_state,
            "reason": decision.reason,
            "cmd": {
                "vx_mps": decision.vx_mps,
                "vy_mps": decision.vy_mps,
            },
            "position": {
                "x": inputs.current_x,
                "y": inputs.current_y,
                "z_ned": inputs.current_z_ned,
            },
            "front_min": inputs.front_min,
            "external_nav_ready": inputs.external_nav_ready,
            "imu_ready": inputs.imu_ready,
            "scan_fresh": inputs.scan_fresh,
            "expected_mode_seen": inputs.expected_mode_seen,
            "armed_seen": inputs.armed_seen,
            "airborne_seen": inputs.airborne_seen,
            "setpoints_sent_count": setpoints_sent_count,
            "obstacle_detected": obstacle_detected,
        },
        separators=(",", ":"),
        sort_keys=True,
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run NavLab hover/forward/avoid mission via MAVLink setpoints.")
    parser.add_argument("--endpoint", default="tcp:sitl:5765")
    parser.add_argument("--duration-sec", type=float, default=90.0)
    parser.add_argument("--summary-file", default="")
    parser.add_argument("--mode", default="GUIDED")
    parser.add_argument("--takeoff-alt-m", type=float, default=0.8)
    parser.add_argument("--min-airborne-alt-m", type=float, default=0.25)
    parser.add_argument("--status-topic", default="/navlab/mission/status")
    parser.add_argument("--sim-log-topic", default=DEFAULT_SIM_LOG_TOPIC)
    parser.add_argument("--scan-features-topic", default="/scan_features")
    parser.add_argument("--external-nav-status-topic", default="/external_nav/status")
    parser.add_argument("--imu-status-topic", default="/imu/status")
    parser.add_argument("--pose-topic", default="/sim/uav_pose")
    parser.add_argument("--mavlink-status-topic", default="/navlab/mavlink/status")
    parser.add_argument("--scan-timeout-sec", type=float, default=1.0)
    parser.add_argument("--status-timeout-sec", type=float, default=1.0)
    parser.add_argument("--setpoint-rate-hz", type=float, default=5.0)
    parser.add_argument("--disable-arming-checks", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--force-arm", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--simulate-mode-arm", action=argparse.BooleanOptionalAction, default=False)
    return parser


def _mode_number(mode_name: str) -> int:
    from pymavlink import mavutil

    requested = mode_name.upper()
    for number, name in mavutil.mode_mapping_acm.items():
        if name == requested:
            return int(number)
    supported = ", ".join(sorted(mavutil.mode_mapping_acm.values()))
    raise ValueError(f"unsupported ArduCopter mode {mode_name!r}; supported: {supported}")


def _send_message_interval(connection, target_system: int, target_component: int, message_id: int, hz: float) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1_000_000.0 / hz),
        0,
        0,
        0,
        0,
        0,
    )


def _request_streams(connection, target_system: int, target_component: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    for message_id, hz in (
        (mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2.0),
        (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0),
        (mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 4.0),
        (mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 4.0),
        (mavlink.MAVLINK_MSG_ID_STATUSTEXT, 2.0),
    ):
        _send_message_interval(connection, target_system, target_component, message_id, hz)


def _set_mode(connection, target_system: int, mode_number: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.set_mode_send(target_system, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_number)


def _set_arming_check(connection, target_system: int, target_component: int, value: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.param_set_send(
        target_system,
        target_component,
        b"ARMING_CHECK",
        float(value),
        mavlink.MAV_PARAM_TYPE_INT32,
    )


def _command_arm(connection, target_system: int, target_component: int, force_arm: bool) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        21196 if force_arm else 0,
        0,
        0,
        0,
        0,
        0,
    )


def _command_takeoff(connection, target_system: int, target_component: int, altitude_m: float) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        math.nan,
        0,
        0,
        altitude_m,
    )


def _send_velocity_z_hold_setpoint(
    connection,
    target_system: int,
    target_component: int,
    vx_mps: float,
    vy_mps: float,
    z_ned_m: float,
) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000),
        target_system,
        target_component,
        mavlink.MAV_FRAME_LOCAL_NED,
        XY_VELOCITY_Z_POSITION_TYPE_MASK,
        0.0,
        0.0,
        z_ned_m,
        vx_mps,
        vy_mps,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def run(argv: Sequence[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from pymavlink import mavutil
        from pymavlink.dialects.v20 import ardupilotmega as mavlink
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        from ydlidar_interfaces.msg import ScanFeatures
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "mavlink_obstacle_mission_controller requires ROS2, ydlidar_interfaces, and pymavlink. "
            "Run it from the NavLab companion image."
        ) from exc

    class MavlinkObstacleMissionController(Node):
        def __init__(self) -> None:
            super().__init__("mavlink_obstacle_mission_controller")
            self._connection = mavutil.mavlink_connection(args.endpoint, dialect="ardupilotmega")
            self._status_pub = self.create_publisher(String, args.status_topic, 10)
            self._sim_log_pub = self.create_publisher(String, args.sim_log_topic, 10)
            self.create_subscription(
                ScanFeatures,
                args.scan_features_topic,
                self._handle_scan_features,
                qos_profile_sensor_data,
            )
            self.create_subscription(String, args.external_nav_status_topic, self._handle_external_nav_status, 10)
            self.create_subscription(String, args.imu_status_topic, self._handle_imu_status, 10)
            self.create_subscription(PoseStamped, args.pose_topic, self._handle_pose, 10)
            self.create_subscription(String, args.mavlink_status_topic, self._handle_mavlink_status, 10)
            self._mode_number = _mode_number(args.mode)
            self._target_system: int | None = None
            self._target_component: int | None = None
            self._front_min: float | None = None
            self._last_scan_monotonic = 0.0
            self._external_nav_ready = False
            self._last_external_status_monotonic = 0.0
            self._imu_ready = False
            self._last_imu_status_monotonic = 0.0
            self._current_x: float | None = None
            self._current_y: float | None = None
            self._current_z: float | None = None
            self._expected_mode_seen = False
            self._armed_seen = False
            self._external_expected_mode_seen = False
            self._external_armed_seen = False
            self._airborne_seen = False
            self._airborne_started: float | None = None
            self._next_request = 0.0
            self._next_mode_command = 0.0
            self._next_arm_command = 0.0
            self._next_takeoff_command = 0.0
            self._next_setpoint = 0.0
            self._setpoints_sent = 0
            self._obstacle_detected = False
            self._avoidance_setpoint_sent = False
            self._phases_seen: set[str] = set()
            self._message_counts: dict[str, int] = {}
            self._command_acks: list[dict[str, int]] = []
            self._ekf_flags: list[int] = []
            self._started = time.monotonic()
            self._config = MissionConfig(takeoff_alt_m=args.takeoff_alt_m)
            self.create_timer(0.05, self._tick)
            self.get_logger().info(f"stage1.5 obstacle mission controller started endpoint={args.endpoint}")

        def _handle_scan_features(self, msg) -> None:
            value = float(msg.front_min)
            self._front_min = value if math.isfinite(value) else None
            self._last_scan_monotonic = time.monotonic()

        def _handle_external_nav_status(self, msg: String) -> None:
            self._external_nav_ready = self._status_ready(msg.data)
            self._last_external_status_monotonic = time.monotonic()

        def _handle_imu_status(self, msg: String) -> None:
            self._imu_ready = self._status_ready(msg.data)
            self._last_imu_status_monotonic = time.monotonic()

        def _handle_pose(self, msg) -> None:
            self._current_x = float(msg.pose.position.x)
            self._current_y = float(msg.pose.position.y)
            self._current_z = -float(msg.pose.position.z)
            if msg.pose.position.z >= args.min_airborne_alt_m:
                self._airborne_seen = True
                if self._airborne_started is None:
                    self._airborne_started = time.monotonic()

        def _handle_mavlink_status(self, msg: String) -> None:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            mode_number = payload.get("mode_number")
            self._external_expected_mode_seen = mode_number == self._mode_number
            self._external_armed_seen = payload.get("armed") is True

        @staticmethod
        def _status_ready(data: str) -> bool:
            try:
                payload = json.loads(data)
            except json.JSONDecodeError:
                return False
            return payload.get("ready") is True

        def _tick(self) -> None:
            now = time.monotonic()
            if now - self._started >= args.duration_sec:
                self._write_summary(ok=False, reason="duration_timeout")
                rclpy.try_shutdown()
                return
            self._drain_mavlink()
            if self._target_system is not None and self._target_component is not None and now >= self._next_request:
                _request_streams(self._connection, self._target_system, self._target_component)
                if args.disable_arming_checks:
                    _set_arming_check(self._connection, self._target_system, self._target_component, 0)
                self._next_request = now + 2.0

            inputs = self._build_inputs(now)
            decision = decide_obstacle_mission(inputs, self._config)
            self._phases_seen.add(decision.phase)
            if inputs.front_min is not None and inputs.front_min <= self._config.obstacle_seen_distance_m:
                self._obstacle_detected = True

            if self._target_system is not None and self._target_component is not None:
                if decision.should_set_guided and now >= self._next_mode_command:
                    _set_mode(self._connection, self._target_system, self._mode_number)
                    self._next_mode_command = now + 1.0
                if decision.should_arm and now >= self._next_arm_command:
                    _command_arm(self._connection, self._target_system, self._target_component, args.force_arm)
                    self._next_arm_command = now + 2.0
                if decision.should_takeoff and now >= self._next_takeoff_command:
                    _command_takeoff(self._connection, self._target_system, self._target_component, args.takeoff_alt_m)
                    self._next_takeoff_command = now + 2.0
                if inputs.airborne_seen and now >= self._next_setpoint:
                    _send_velocity_z_hold_setpoint(
                        self._connection,
                        self._target_system,
                        self._target_component,
                        decision.vx_mps,
                        decision.vy_mps,
                        -args.takeoff_alt_m,
                    )
                    self._setpoints_sent += 1
                    if decision.phase == "avoid":
                        self._avoidance_setpoint_sent = True
                    self._next_setpoint = now + (1.0 / args.setpoint_rate_hz)

            self._publish_status(decision, inputs)
            if decision.terminal:
                self._write_summary(ok=True, reason=decision.reason)
                rclpy.try_shutdown()

        def _drain_mavlink(self) -> None:
            while True:
                msg = self._connection.recv_match(blocking=False)
                if msg is None:
                    return
                msg_type = msg.get_type()
                self._message_counts[msg_type] = self._message_counts.get(msg_type, 0) + 1
                if msg_type == "HEARTBEAT" and int(msg.autopilot) != mavlink.MAV_AUTOPILOT_INVALID:
                    self._target_system = msg.get_srcSystem()
                    self._target_component = msg.get_srcComponent()
                    self._expected_mode_seen = int(msg.custom_mode) == self._mode_number
                    self._armed_seen = bool(int(msg.base_mode) & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif msg_type == "COMMAND_ACK":
                    if len(self._command_acks) < 80:
                        self._command_acks.append({"command": int(msg.command), "result": int(msg.result)})
                elif msg_type == "LOCAL_POSITION_NED":
                    self._current_x = float(msg.x)
                    self._current_y = float(msg.y)
                    self._current_z = float(msg.z)
                    if self._current_z <= -args.min_airborne_alt_m:
                        self._airborne_seen = True
                elif msg_type == "GLOBAL_POSITION_INT":
                    if float(msg.relative_alt) / 1000.0 >= args.min_airborne_alt_m:
                        self._airborne_seen = True
                elif msg_type == "EXTENDED_SYS_STATE":
                    if int(msg.landed_state) in (mavlink.MAV_LANDED_STATE_TAKEOFF, mavlink.MAV_LANDED_STATE_IN_AIR):
                        self._airborne_seen = True
                elif msg_type == "EKF_STATUS_REPORT":
                    self._ekf_flags.append(int(msg.flags))
                if self._airborne_seen and self._airborne_started is None:
                    self._airborne_started = time.monotonic()

        def _build_inputs(self, now: float) -> MissionInputs:
            scan_fresh = self._last_scan_monotonic > 0.0 and now - self._last_scan_monotonic <= args.scan_timeout_sec
            external_nav_fresh = (
                self._last_external_status_monotonic > 0.0
                and now - self._last_external_status_monotonic <= args.status_timeout_sec
            )
            imu_fresh = (
                self._last_imu_status_monotonic > 0.0
                and now - self._last_imu_status_monotonic <= args.status_timeout_sec
            )
            airborne_elapsed = None if self._airborne_started is None else now - self._airborne_started
            return MissionInputs(
                current_x=self._current_x,
                current_y=self._current_y,
                current_z_ned=self._current_z,
                front_min=self._front_min,
                external_nav_ready=self._external_nav_ready and external_nav_fresh,
                imu_ready=self._imu_ready and imu_fresh,
                scan_fresh=scan_fresh,
                expected_mode_seen=(
                    args.simulate_mode_arm or self._expected_mode_seen or self._external_expected_mode_seen
                ),
                armed_seen=args.simulate_mode_arm or self._armed_seen or self._external_armed_seen,
                airborne_seen=self._airborne_seen,
                airborne_elapsed_sec=airborne_elapsed,
            )

        def _publish_status(self, decision: MissionDecision, inputs: MissionInputs) -> None:
            msg = String()
            msg.data = encode_mission_status(
                decision=decision,
                inputs=inputs,
                setpoints_sent_count=self._setpoints_sent,
                obstacle_detected=self._obstacle_detected,
            )
            self._status_pub.publish(msg)
            sim_log = String()
            sim_log.data = encode_sim_log(
                source="mavlink_obstacle_mission_controller",
                event=decision.reason,
                mission_state=decision.mission_state,
                phase=decision.phase,
                current_x=inputs.current_x,
                current_y=inputs.current_y,
                current_z_ned=inputs.current_z_ned,
                front_min=inputs.front_min,
                cmd_vx_mps=decision.vx_mps,
                cmd_vy_mps=decision.vy_mps,
                obstacle_detected=self._obstacle_detected,
                setpoints_sent_count=self._setpoints_sent,
            )
            self._sim_log_pub.publish(sim_log)

        def _write_summary(self, *, ok: bool, reason: str) -> None:
            if not args.summary_file:
                return
            summary = {
                "ok": ok,
                "reason": reason,
                "setpoints_sent_count": self._setpoints_sent,
                "obstacle_detected": self._obstacle_detected,
                "avoidance_setpoint_sent": self._avoidance_setpoint_sent,
                "phases_seen": sorted(self._phases_seen),
                "last_position": {"x": self._current_x, "y": self._current_y, "z_ned": self._current_z},
                "message_counts": self._message_counts,
                "command_acks": self._command_acks[-40:],
                "ekf_flags_seen": sorted(set(self._ekf_flags)),
            }
            path = Path(args.summary_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

    rclpy.init(args=None)
    node = MavlinkObstacleMissionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
