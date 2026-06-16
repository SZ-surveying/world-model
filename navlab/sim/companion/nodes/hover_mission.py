from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from navlab.sim.companion.nodes.obstacle_mission import (
    DEFAULT_ORIGIN_ALT_M,
    DEFAULT_ORIGIN_LAT_DEG,
    DEFAULT_ORIGIN_LON_DEG,
    command_arm,
    command_takeoff,
    mode_number,
    send_gcs_heartbeat,
    send_local_position_yaw_setpoint,
    set_arming_check,
    set_ekf_origin,
    set_home_position,
    set_mode,
)
from navlab.sim.companion.runtime.status import DEFAULT_SIM_LOG_TOPIC, encode_sim_log

os.environ.setdefault("MAVLINK20", "1")
HOVER_DURATION_TOLERANCE_SEC = 0.25


@dataclass(frozen=True, slots=True)
class HoverInputs:
    external_nav_ready: bool
    imu_ready: bool
    ready_elapsed_sec: float
    current_yaw_rad: float | None
    expected_mode_seen: bool
    armed_seen: bool
    airborne_seen: bool
    takeoff_ack_ok: bool
    airborne_elapsed_sec: float
    hover_elapsed_sec: float
    current_x: float | None
    current_y: float | None
    current_z_ned: float | None
    current_height_m: float | None
    target_z_ned: float | None


@dataclass(frozen=True, slots=True)
class HoverRequirements:
    require_external_nav: bool = True
    require_imu_status: bool = True


@dataclass(frozen=True, slots=True)
class HoverDecision:
    phase: str
    reason: str
    should_set_guided: bool = False
    should_arm: bool = False
    should_takeoff: bool = False
    terminal: bool = False


@dataclass(frozen=True, slots=True)
class HoverDriftSummary:
    sample_count: int
    duration_sec: float
    horizontal_span_m: float
    z_span_m: float
    horizontal_drift_m: float
    z_drift_m: float

    @property
    def ok(self) -> bool:
        return self.sample_count >= 2


def decide_hover(
    inputs: HoverInputs,
    *,
    requirements: HoverRequirements | None = None,
    preflight_ready_sec: float,
    hover_settle_sec: float,
    hover_hold_sec: float,
    takeoff_alt_m: float,
    hover_altitude_tolerance_m: float,
) -> HoverDecision:
    requirements = requirements or HoverRequirements()
    if requirements.require_external_nav and not inputs.external_nav_ready:
        return HoverDecision("wait_ready", "waiting_for_external_nav_and_imu")
    if requirements.require_imu_status and not inputs.imu_ready:
        return HoverDecision("wait_ready", "waiting_for_external_nav_and_imu")
    if inputs.current_yaw_rad is None:
        return HoverDecision("wait_ready", "waiting_for_fcu_attitude")
    if inputs.ready_elapsed_sec < preflight_ready_sec:
        return HoverDecision("wait_ready", "waiting_for_stable_external_nav_and_imu")
    if not inputs.expected_mode_seen:
        return HoverDecision("guided", "setting_guided", should_set_guided=True)
    if not inputs.armed_seen:
        return HoverDecision("arm", "arming_vehicle", should_arm=True)
    if not inputs.airborne_seen:
        return HoverDecision("takeoff", "taking_off", should_takeoff=True)
    if inputs.airborne_elapsed_sec < hover_settle_sec:
        return HoverDecision("hover_settle", "settling_before_position_hold")
    target_z_ned = inputs.target_z_ned if inputs.target_z_ned is not None else -takeoff_alt_m
    if inputs.current_z_ned is not None:
        if abs(inputs.current_z_ned - target_z_ned) > hover_altitude_tolerance_m:
            return HoverDecision("hover_settle", "settling_until_target_altitude")
    elif inputs.current_height_m is not None:
        if abs(inputs.current_height_m - takeoff_alt_m) > hover_altitude_tolerance_m:
            return HoverDecision("hover_settle", "settling_until_target_altitude")
    else:
        return HoverDecision("hover_settle", "settling_until_target_altitude")
    if inputs.hover_elapsed_sec < hover_hold_sec:
        return HoverDecision("hover_hold", "holding_position")
    return HoverDecision("complete", "hover_complete", terminal=True)


def summarize_hover_drift(samples: list[tuple[float, float, float, float]]) -> HoverDriftSummary:
    if len(samples) < 2:
        return HoverDriftSummary(
            sample_count=len(samples),
            duration_sec=0.0,
            horizontal_span_m=math.inf,
            z_span_m=math.inf,
            horizontal_drift_m=math.inf,
            z_drift_m=math.inf,
        )
    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    zs = [sample[3] for sample in samples]
    start = samples[0]
    end = samples[-1]
    return HoverDriftSummary(
        sample_count=len(samples),
        duration_sec=max(0.0, end[0] - start[0]),
        horizontal_span_m=math.hypot(max(xs) - min(xs), max(ys) - min(ys)),
        z_span_m=max(zs) - min(zs),
        horizontal_drift_m=math.hypot(end[1] - start[1], end[2] - start[2]),
        z_drift_m=abs(end[3] - start[3]),
    )


def json_safe_number(value: float | int | None) -> float | int | None:
    if value is None:
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def hold_axis_or_current(hold_value: float | None, current_value: float | None) -> float:
    if hold_value is not None:
        return hold_value
    if current_value is not None:
        return current_value
    return 0.0


def command_ack_success(command_acks: list[dict[str, int]], command_id: int) -> bool:
    return any(ack.get("command") == command_id and ack.get("result") == 0 for ack in command_acks)


def command_ack_rejected(command_acks: list[dict[str, int]], command_id: int) -> bool:
    return any(ack.get("command") == command_id and ack.get("result") not in (0, None) for ack in command_acks)


def _request_hover_streams(connection, target_system: int, target_component: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    for message_id, hz in (
        (mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2.0),
        (mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0),
        (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0),
        (mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 4.0),
        (mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 4.0),
        (mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10.0),
        (mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 1.0),
        (mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1.0),
        (mavlink.MAVLINK_MSG_ID_STATUSTEXT, 2.0),
    ):
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


def _command_disarm(connection, target_system: int, target_component: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def _command_land(connection, target_system: int, target_component: int) -> None:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run NavLab FCU-controlled hover mission via MAVLink setpoints.")
    parser.add_argument("--endpoint", default="tcp:sitl:5765")
    parser.add_argument("--duration-sec", type=float, default=90.0)
    parser.add_argument("--summary-file", default="")
    parser.add_argument("--mode", default="GUIDED")
    parser.add_argument("--takeoff-alt-m", type=float, default=0.45)
    parser.add_argument("--min-airborne-alt-m", type=float, default=0.10)
    parser.add_argument("--preflight-ready-sec", type=float, default=5.0)
    parser.add_argument("--hover-settle-sec", type=float, default=2.0)
    parser.add_argument("--hover-altitude-tolerance-m", type=float, default=0.18)
    parser.add_argument("--hover-hold-sec", type=float, default=20.0)
    parser.add_argument("--max-horizontal-drift-m", type=float, default=1.0)
    parser.add_argument("--max-altitude-drift-m", type=float, default=0.6)
    parser.add_argument("--origin-lat-deg", type=float, default=DEFAULT_ORIGIN_LAT_DEG)
    parser.add_argument("--origin-lon-deg", type=float, default=DEFAULT_ORIGIN_LON_DEG)
    parser.add_argument("--origin-alt-m", type=float, default=DEFAULT_ORIGIN_ALT_M)
    parser.add_argument("--source-system", type=int, default=255)
    parser.add_argument("--source-component", type=int, default=190)
    parser.add_argument("--status-topic", default="/navlab/hover/status")
    parser.add_argument("--landing-status-topic", default="/navlab/landing/status")
    parser.add_argument("--landing-intent-topic", default="/navlab/landing/intent")
    parser.add_argument("--sim-log-topic", default=DEFAULT_SIM_LOG_TOPIC)
    parser.add_argument("--external-nav-status-topic", default="/external_nav/status")
    parser.add_argument("--imu-status-topic", default="/imu/status")
    parser.add_argument("--mavlink-status-topic", default="/navlab/mavlink/status")
    parser.add_argument("--status-timeout-sec", type=float, default=1.0)
    parser.add_argument("--setpoint-rate-hz", type=float, default=5.0)
    parser.add_argument("--pre-land-hold-sec", type=float, default=2.0)
    parser.add_argument("--max-landing-duration-sec", type=float, default=35.0)
    parser.add_argument("--touchdown-altitude-m", type=float, default=0.12)
    parser.add_argument("--touchdown-vertical-speed-mps", type=float, default=0.08)
    parser.add_argument("--require-disarm", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-motors-safe", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-external-nav", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-imu-status", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--send-position-setpoints", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--disable-arming-checks", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--force-arm", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--simulate-mode-arm", action=argparse.BooleanOptionalAction, default=False)
    # Compatibility with MissionNodeConfig argv; hover ignores these obstacle-demo fields.
    parser.add_argument("--forward-speed-mps", type=float, default=0.0)
    parser.add_argument("--avoid-forward-speed-mps", type=float, default=0.0)
    parser.add_argument("--obstacle-detect-distance-m", type=float, default=0.0)
    parser.add_argument("--obstacle-avoid-distance-m", type=float, default=0.0)
    parser.add_argument("--scan-yaw-deg", type=float, default=0.0)
    parser.add_argument("--scan-dwell-sec", type=float, default=0.0)
    parser.add_argument("--pass-x-m", type=float, default=0.0)
    parser.add_argument("--return-y-m", type=float, default=0.0)
    parser.add_argument("--final-hold-sec", type=float, default=0.0)
    parser.add_argument("--scan-features-topic", default="/scan_features")
    parser.add_argument("--pose-topic", default="/sim/uav_pose")
    parser.add_argument("--scan-timeout-sec", type=float, default=1.0)
    parser.add_argument("--setpoint-lookahead-sec", type=float, default=0.0)
    return parser


def run(argv: Sequence[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)

    try:
        import rclpy
        from pymavlink import mavutil
        from pymavlink.dialects.v20 import ardupilotmega as mavlink
        from rclpy.node import Node
        from std_msgs.msg import String
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "mavlink_hover_mission_controller requires ROS2 and pymavlink. Run it from the NavLab companion image."
        ) from exc

    class MavlinkHoverMissionController(Node):
        def __init__(self) -> None:
            super().__init__("mavlink_hover_mission_controller")
            self._connection = mavutil.mavlink_connection(
                args.endpoint,
                source_system=args.source_system,
                source_component=args.source_component,
                dialect="ardupilotmega",
            )
            self._status_pub = self.create_publisher(String, args.status_topic, 10)
            self._landing_status_pub = self.create_publisher(String, args.landing_status_topic, 10)
            self._landing_intent_pub = self.create_publisher(String, args.landing_intent_topic, 10)
            self._sim_log_pub = self.create_publisher(String, args.sim_log_topic, 10)
            self.create_subscription(String, args.external_nav_status_topic, self._handle_external_nav_status, 10)
            self.create_subscription(String, args.imu_status_topic, self._handle_imu_status, 10)
            self.create_subscription(String, args.mavlink_status_topic, self._handle_mavlink_status, 10)
            self.mode_number = mode_number(args.mode)
            self._target_system: int | None = None
            self._target_component: int | None = None
            self._external_nav_ready = False
            self._last_external_status_monotonic = 0.0
            self._last_external_nav_state = ""
            self._external_nav_status_history: list[dict[str, object]] = []
            self._imu_ready = False
            self._last_imu_status_monotonic = 0.0
            self._ready_started: float | None = None
            self._expected_mode_seen = False
            self._armed_seen = False
            self._guided_seen_ever = False
            self._armed_seen_ever = False
            self._external_expected_mode_seen = False
            self._external_armed_seen = False
            self._airborne_seen = False
            self._airborne_started: float | None = None
            self._hover_started: float | None = None
            self._hold_x: float | None = None
            self._hold_y: float | None = None
            self._hold_yaw_rad = 0.0
            self._current_x: float | None = None
            self._current_y: float | None = None
            self._current_z: float | None = None
            self._ground_z_ned: float | None = None
            self._current_vz: float | None = None
            self._current_range_m: float | None = None
            self._current_yaw_rad: float | None = None
            self._next_request = 0.0
            self._next_heartbeat = 0.0
            self._next_origin_command = 0.0
            self._next_mode_command = 0.0
            self._next_arm_command = 0.0
            self._next_takeoff_command = 0.0
            self._next_setpoint = 0.0
            self._next_land_command = 0.0
            self._next_disarm_command = 0.0
            self._started = time.monotonic()
            self._phases_seen: set[str] = set()
            self._phase_counts: dict[str, int] = {}
            self._status_history: list[dict[str, object]] = []
            self._setpoints_sent = 0
            self._message_counts: dict[str, int] = {}
            self._command_acks: list[dict[str, int]] = []
            self._statustext: list[dict[str, object]] = []
            self._crash_detected = False
            self._ekf_flags: list[int] = []
            self._sent_commands: dict[str, int] = {}
            self._gps_global_origin_seen = False
            self._home_position_seen = False
            self._hover_samples: list[tuple[float, float, float, float]] = []
            self._landing_started: float | None = None
            self._hover_body_ok = False
            self._hover_body_reason = ""
            self._landing_state = "not_started"
            self._landed_state: int | None = None
            self._touchdown_confirmed = False
            self._land_command_sent = False
            self._landing_blockers: list[str] = []
            self.create_timer(0.05, self._tick)
            self.get_logger().info(f"hover mission controller started endpoint={args.endpoint}")

        def _stop_vehicle(self) -> None:
            if self._target_system is None or self._target_component is None:
                return
            _command_disarm(self._connection, self._target_system, self._target_component)
            self._count_sent_command("disarm")

        def _handle_external_nav_status(self, msg: String) -> None:
            previous_ready = self._external_nav_ready
            previous_state = self._last_external_nav_state
            payload = self._parse_status_payload(msg.data)
            self._external_nav_ready = payload.get("ready") is True
            self._last_external_nav_state = str(payload.get("state") or "")
            self._last_external_status_monotonic = time.monotonic()
            if self._external_nav_ready != previous_ready or self._last_external_nav_state != previous_state:
                odom = payload.get("odom") if isinstance(payload.get("odom"), dict) else {}
                event = {
                    "elapsed_sec": round(self._last_external_status_monotonic - self._started, 3),
                    "ready": self._external_nav_ready,
                    "state": self._last_external_nav_state,
                    "input_topic": odom.get("input_topic"),
                    "rate_hz": odom.get("rate_hz"),
                    "rate_ok": odom.get("rate_ok"),
                    "frame_ok": odom.get("frame_ok"),
                    "age_ms": odom.get("age_ms"),
                }
                self._external_nav_status_history.append(event)
                self._external_nav_status_history = self._external_nav_status_history[-40:]
                self.get_logger().info(
                    "external_nav status "
                    f"ready={self._external_nav_ready} "
                    f"state={self._last_external_nav_state} "
                    f"input={event['input_topic']} "
                    f"rate_hz={event['rate_hz']} "
                    f"rate_ok={event['rate_ok']} "
                    f"frame_ok={event['frame_ok']}"
                )

        def _handle_imu_status(self, msg: String) -> None:
            self._imu_ready = self._status_ready(msg.data)
            self._last_imu_status_monotonic = time.monotonic()

        def _handle_mavlink_status(self, msg: String) -> None:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            self._external_expected_mode_seen = payload.get("mode_number") == self.mode_number
            self._external_armed_seen = payload.get("armed") is True

        @staticmethod
        def _status_ready(data: str) -> bool:
            return MavlinkHoverMissionController._parse_status_payload(data).get("ready") is True

        @staticmethod
        def _parse_status_payload(data: str) -> dict[str, object]:
            try:
                payload = json.loads(data)
            except json.JSONDecodeError:
                return {}
            return payload if isinstance(payload, dict) else {}

        def _tick(self) -> None:
            now = time.monotonic()
            if now - self._started >= args.duration_sec:
                self._stop_vehicle()
                self.write_summary(ok=False, reason="duration_timeout", landing_ok=False)
                rclpy.try_shutdown()
                return
            self._drain_mavlink()
            if self._crash_detected:
                self._stop_vehicle()
                self.write_summary(ok=False, reason="crash_detected", landing_ok=False)
                rclpy.try_shutdown()
                return
            if self._landing_started is not None:
                self._tick_landing(now)
                return
            if now >= self._next_heartbeat:
                send_gcs_heartbeat(self._connection)
                self._next_heartbeat = now + 1.0
            if self._target_system is not None and self._target_component is not None and now >= self._next_request:
                _request_hover_streams(self._connection, self._target_system, self._target_component)
                if args.disable_arming_checks:
                    set_arming_check(self._connection, self._target_system, self._target_component, 0)
                self._next_request = now + 2.0
            if (
                self._target_system is not None
                and self._target_component is not None
                and now >= self._next_origin_command
            ):
                if not self._gps_global_origin_seen:
                    set_ekf_origin(
                        self._connection,
                        self._target_system,
                        args.origin_lat_deg,
                        args.origin_lon_deg,
                        args.origin_alt_m,
                    )
                    self._count_sent_command("set_gps_global_origin")
                if not self._home_position_seen:
                    set_home_position(
                        self._connection,
                        self._target_system,
                        self._target_component,
                        args.origin_lat_deg,
                        args.origin_lon_deg,
                        args.origin_alt_m,
                    )
                    self._count_sent_command("set_home_position")
                self._next_origin_command = now + 2.0

            inputs = self._build_inputs(now)
            decision = decide_hover(
                inputs,
                requirements=HoverRequirements(
                    require_external_nav=args.require_external_nav,
                    require_imu_status=args.require_imu_status,
                ),
                preflight_ready_sec=args.preflight_ready_sec,
                hover_settle_sec=args.hover_settle_sec,
                hover_hold_sec=args.hover_hold_sec,
                takeoff_alt_m=args.takeoff_alt_m,
                hover_altitude_tolerance_m=args.hover_altitude_tolerance_m,
            )
            self._phases_seen.add(decision.phase)
            self._phase_counts[decision.phase] = self._phase_counts.get(decision.phase, 0) + 1
            if decision.phase != "hover_hold" and self._hover_started is not None and not decision.terminal:
                self._hover_started = None
                self._hold_x = None
                self._hold_y = None
                self._hover_samples.clear()
            if decision.phase == "hover_hold" and self._hover_started is None:
                self._hover_started = now
                self._hold_x = self._current_x if self._current_x is not None else 0.0
                self._hold_y = self._current_y if self._current_y is not None else 0.0
                self._hold_yaw_rad = self._current_yaw_rad if self._current_yaw_rad is not None else 0.0
            if decision.phase == "hover_hold" and self._current_x is not None and self._current_y is not None:
                self._hover_samples.append((now, self._current_x, self._current_y, self._current_z or 0.0))

            if self._target_system is not None and self._target_component is not None:
                if decision.should_set_guided and now >= self._next_mode_command:
                    set_mode(self._connection, self._target_system, self.mode_number)
                    self._count_sent_command("set_mode_guided")
                    self._next_mode_command = now + 1.0
                if decision.should_arm and now >= self._next_arm_command:
                    command_arm(self._connection, self._target_system, self._target_component, args.force_arm)
                    self._count_sent_command("arm")
                    self._next_arm_command = now + 2.0
                if decision.should_takeoff and now >= self._next_takeoff_command:
                    command_takeoff(self._connection, self._target_system, self._target_component, args.takeoff_alt_m)
                    self._count_sent_command("takeoff")
                    self._next_takeoff_command = now + 2.0
                should_send_hold_setpoint = args.send_position_setpoints and inputs.airborne_seen
                if should_send_hold_setpoint and now >= self._next_setpoint:
                    send_local_position_yaw_setpoint(
                        self._connection,
                        self._target_system,
                        self._target_component,
                        hold_axis_or_current(self._hold_x, self._current_x),
                        hold_axis_or_current(self._hold_y, self._current_y),
                        self._target_z_ned(),
                        self._hold_yaw_rad
                        if self._hold_x is not None and self._hold_y is not None
                        else (self._current_yaw_rad or 0.0),
                    )
                    self._setpoints_sent += 1
                    self._count_sent_command("local_position_yaw_setpoint")
                    self._next_setpoint = now + (1.0 / args.setpoint_rate_hz)

            self._publish_status(decision, inputs)
            if decision.terminal:
                drift = summarize_hover_drift(self._hover_samples)
                self._hover_body_ok = (
                    drift.ok
                    and drift.duration_sec >= args.hover_hold_sec - HOVER_DURATION_TOLERANCE_SEC
                    and drift.horizontal_drift_m <= args.max_horizontal_drift_m
                    and drift.z_span_m <= args.max_altitude_drift_m
                    and self._message_counts.get("LOCAL_POSITION_NED", 0) > 0
                    and not self._crash_detected
                )
                self._hover_body_reason = "hover_complete" if self._hover_body_ok else "hover_unstable"
                self._start_landing(now)

        def _start_landing(self, now: float) -> None:
            self._landing_started = now
            self._landing_state = "task_body_complete"
            intent = String()
            intent.data = json.dumps(
                {
                    "source": "mavlink_hover_mission_controller",
                    "kind": "land_in_place",
                    "policy": "land_in_place",
                    "reason": self._hover_body_reason,
                    "updated_ms": int(time.time() * 1000),
                },
                separators=(",", ":"),
                sort_keys=True,
            )
            self._landing_intent_pub.publish(intent)
            self._publish_landing_status()

        def _tick_landing(self, now: float) -> None:
            if self._target_system is None or self._target_component is None:
                self._landing_blockers.append("landing_target_system_missing")
                self.write_summary(ok=False, reason="landing_target_system_missing", landing_ok=False)
                rclpy.try_shutdown()
                return
            self._drain_mavlink()
            elapsed = 0.0 if self._landing_started is None else now - self._landing_started
            if elapsed > args.max_landing_duration_sec:
                self._landing_blockers.append("landing_timeout")
                self._stop_vehicle()
                self.write_summary(ok=False, reason="landing_timeout", landing_ok=False)
                rclpy.try_shutdown()
                return
            if elapsed < args.pre_land_hold_sec:
                self._landing_state = "pre_land_hold"
                self._send_hold_setpoint(now)
                self._publish_landing_status()
                return
            if not self._land_command_sent or now >= self._next_land_command:
                self._landing_state = "land_command_sent"
                _command_land(self._connection, self._target_system, self._target_component)
                self._count_sent_command("land")
                self._land_command_sent = True
                self._next_land_command = now + 2.0
            if command_ack_rejected(self._command_acks, mavlink.MAV_CMD_NAV_LAND):
                self._landing_blockers.append("landing_command_rejected")
                self.write_summary(ok=False, reason="landing_command_rejected", landing_ok=False)
                rclpy.try_shutdown()
                return
            self._touchdown_confirmed = self._touchdown_confirmed or self._touchdown_candidate()
            self._landing_state = "touchdown_candidate" if self._touchdown_confirmed else "descent_monitoring"
            if self._touchdown_confirmed and args.require_disarm and now >= self._next_disarm_command:
                self._landing_state = "disarm_requested"
                _command_disarm(self._connection, self._target_system, self._target_component)
                self._count_sent_command("disarm")
                self._next_disarm_command = now + 2.0
            disarmed = not self._armed_seen
            motors_safe = disarmed if args.require_motors_safe else True
            landing_ok = self._touchdown_confirmed and (disarmed if args.require_disarm else True) and motors_safe
            self._publish_landing_status()
            if landing_ok:
                self._landing_state = "landing_complete"
                self.write_summary(
                    ok=self._hover_body_ok and landing_ok, reason=self._hover_body_reason, landing_ok=True
                )
                rclpy.try_shutdown()

        def _send_hold_setpoint(self, now: float) -> None:
            if (
                not args.send_position_setpoints
                or self._target_system is None
                or self._target_component is None
                or now < self._next_setpoint
            ):
                return
            send_local_position_yaw_setpoint(
                self._connection,
                self._target_system,
                self._target_component,
                hold_axis_or_current(self._hold_x, self._current_x),
                hold_axis_or_current(self._hold_y, self._current_y),
                -args.takeoff_alt_m,
                self._hold_yaw_rad
                if self._hold_x is not None and self._hold_y is not None
                else (self._current_yaw_rad or 0.0),
            )
            self._setpoints_sent += 1
            self._count_sent_command("local_position_yaw_setpoint")
            self._next_setpoint = now + (1.0 / args.setpoint_rate_hz)

        def _touchdown_candidate(self) -> bool:
            if self._landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND:
                return True
            range_ok = self._current_range_m is not None and self._current_range_m <= args.touchdown_altitude_m
            z_ok = self._current_z is not None and self._current_z >= -args.touchdown_altitude_m
            vz_ok = self._current_vz is None or abs(self._current_vz) <= args.touchdown_vertical_speed_mps
            return bool((range_ok or z_ok) and vz_ok)

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
                    self._expected_mode_seen = int(msg.custom_mode) == self.mode_number
                    self._armed_seen = bool(int(msg.base_mode) & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._guided_seen_ever = self._guided_seen_ever or self._expected_mode_seen
                    self._armed_seen_ever = self._armed_seen_ever or self._armed_seen
                elif msg_type == "COMMAND_ACK":
                    if len(self._command_acks) < 120:
                        self._command_acks.append({"command": int(msg.command), "result": int(msg.result)})
                elif msg_type == "STATUSTEXT":
                    if len(self._statustext) < 120:
                        text = str(msg.text).rstrip("\x00")
                        if "Crash:" in text:
                            self._crash_detected = True
                        self._statustext.append({"severity": int(msg.severity), "text": text})
                elif msg_type == "LOCAL_POSITION_NED":
                    self._current_x = float(msg.x)
                    self._current_y = float(msg.y)
                    self._current_z = float(msg.z)
                    self._current_vz = float(getattr(msg, "vz", 0.0))
                    if self._ground_z_ned is None:
                        self._ground_z_ned = self._current_z
                    if self._ground_z_ned - self._current_z >= args.min_airborne_alt_m:
                        self._airborne_seen = True
                elif msg_type == "ATTITUDE":
                    self._current_yaw_rad = float(msg.yaw)
                elif msg_type == "GLOBAL_POSITION_INT":
                    if float(msg.relative_alt) / 1000.0 >= args.min_airborne_alt_m:
                        self._airborne_seen = True
                elif msg_type == "EKF_STATUS_REPORT":
                    self._ekf_flags.append(int(msg.flags))
                elif msg_type == "GPS_GLOBAL_ORIGIN":
                    self._gps_global_origin_seen = True
                elif msg_type == "HOME_POSITION":
                    self._home_position_seen = True
                elif msg_type in {"DISTANCE_SENSOR", "RANGEFINDER"}:
                    if hasattr(msg, "current_distance"):
                        self._current_range_m = float(msg.current_distance) / 100.0
                    elif hasattr(msg, "distance"):
                        self._current_range_m = float(msg.distance)
                elif msg_type == "EXTENDED_SYS_STATE":
                    self._landed_state = int(msg.landed_state)
                if self._airborne_seen and self._airborne_started is None:
                    self._airborne_started = time.monotonic()

        def _build_inputs(self, now: float) -> HoverInputs:
            external_nav_fresh = (
                self._last_external_status_monotonic > 0.0
                and now - self._last_external_status_monotonic <= args.status_timeout_sec
            )
            imu_fresh = (
                self._last_imu_status_monotonic > 0.0
                and now - self._last_imu_status_monotonic <= args.status_timeout_sec
            )
            hover_elapsed = 0.0 if self._hover_started is None else now - self._hover_started
            airborne_elapsed = 0.0 if self._airborne_started is None else now - self._airborne_started
            takeoff_ack_ok = command_ack_success(self._command_acks, mavlink.MAV_CMD_NAV_TAKEOFF)
            external_ready = self._external_nav_ready and external_nav_fresh
            imu_ready = self._imu_ready and imu_fresh
            ready_for_preflight = (external_ready or not args.require_external_nav) and (
                imu_ready or not args.require_imu_status
            )
            if ready_for_preflight:
                if self._ready_started is None:
                    self._ready_started = now
            else:
                self._ready_started = None
            ready_elapsed = 0.0 if self._ready_started is None else now - self._ready_started
            return HoverInputs(
                external_nav_ready=external_ready,
                imu_ready=imu_ready,
                ready_elapsed_sec=ready_elapsed,
                current_yaw_rad=self._current_yaw_rad,
                expected_mode_seen=(
                    args.simulate_mode_arm or self._expected_mode_seen or self._external_expected_mode_seen
                ),
                armed_seen=args.simulate_mode_arm or self._armed_seen or self._external_armed_seen,
                airborne_seen=self._airborne_seen,
                takeoff_ack_ok=takeoff_ack_ok,
                airborne_elapsed_sec=airborne_elapsed,
                hover_elapsed_sec=hover_elapsed,
                current_x=self._current_x,
                current_y=self._current_y,
                current_z_ned=self._current_z,
                current_height_m=self._current_range_m,
                target_z_ned=self._target_z_ned(),
            )

        def _target_z_ned(self) -> float:
            ground_z = self._ground_z_ned if self._ground_z_ned is not None else 0.0
            return ground_z - args.takeoff_alt_m

        def _publish_status(self, decision: HoverDecision, inputs: HoverInputs) -> None:
            status_payload = {
                "phase": decision.phase,
                "reason": decision.reason,
                "external_nav_ready": inputs.external_nav_ready,
                "imu_ready": inputs.imu_ready,
                "ready_elapsed_sec": inputs.ready_elapsed_sec,
                "expected_mode_seen": inputs.expected_mode_seen,
                "armed_seen": inputs.armed_seen,
                "airborne_seen": inputs.airborne_seen,
                "takeoff_ack_ok": inputs.takeoff_ack_ok,
                "airborne_elapsed_sec": inputs.airborne_elapsed_sec,
                "hover_elapsed_sec": inputs.hover_elapsed_sec,
                "setpoints_sent_count": self._setpoints_sent,
                "local_position_count": self._message_counts.get("LOCAL_POSITION_NED", 0),
                "rangefinder_count": self._rangefinder_count(),
                "position": {
                    "x": inputs.current_x,
                    "y": inputs.current_y,
                    "z_ned": inputs.current_z_ned,
                    "height_m": inputs.current_height_m,
                    "target_z_ned": inputs.target_z_ned,
                    "yaw_rad": self._current_yaw_rad,
                },
            }
            self._status_history.append(status_payload)
            self._status_history = self._status_history[-80:]
            msg = String()
            msg.data = json.dumps(status_payload, separators=(",", ":"), sort_keys=True)
            self._status_pub.publish(msg)
            sim_log = String()
            sim_log.data = encode_sim_log(
                source="mavlink_hover_mission_controller",
                event=decision.reason,
                mission_state="complete" if decision.terminal else "running",
                phase=decision.phase,
                current_x=inputs.current_x,
                current_y=inputs.current_y,
                current_z_ned=inputs.current_z_ned,
                current_yaw_rad=self._current_yaw_rad,
                setpoints_sent_count=self._setpoints_sent,
            )
            self._sim_log_pub.publish(sim_log)

        def _landing_summary(self) -> dict[str, object]:
            land_command_accepted = command_ack_success(self._command_acks, mavlink.MAV_CMD_NAV_LAND)
            disarmed = not self._armed_seen
            motors_safe = disarmed if args.require_motors_safe else True
            ok = bool(self._touchdown_confirmed and (disarmed if args.require_disarm else True) and motors_safe)
            blockers = list(dict.fromkeys(self._landing_blockers))
            if self._landing_started is not None:
                if not self._touchdown_confirmed:
                    blockers.append("touchdown_not_confirmed")
                if args.require_disarm and not disarmed:
                    blockers.append("disarm_not_confirmed")
                if args.require_motors_safe and not motors_safe:
                    blockers.append("motors_not_safe")
            else:
                blockers.append("landing_not_started")
            return {
                "ok": ok,
                "claim": "evaluated" if self._landing_started is not None else "not_evaluated",
                "policy": "land_in_place",
                "state": self._landing_state,
                "return_home": {
                    "required": False,
                    "ok": True,
                    "state": "not_required",
                    "distance_to_home_m": None,
                    "duration_sec": None,
                },
                "land_command_accepted": land_command_accepted,
                "landing_duration_sec": None
                if self._landing_started is None
                else max(0.0, time.monotonic() - self._landing_started),
                "landed_confirmed": self._touchdown_confirmed,
                "touchdown_confirmed": self._touchdown_confirmed,
                "disarmed": disarmed,
                "motors_safe": motors_safe,
                "require_disarm": args.require_disarm,
                "require_motors_safe": args.require_motors_safe,
                "uses_gazebo_truth_as_input": False,
                "last_range_m": self._current_range_m,
                "last_z_ned": self._current_z,
                "last_vz_mps": self._current_vz,
                "landed_state": self._landed_state,
                "blockers": sorted(set(blockers)) if not ok else [],
            }

        def _publish_landing_status(self) -> None:
            msg = String()
            msg.data = json.dumps(self._landing_summary(), separators=(",", ":"), sort_keys=True)
            self._landing_status_pub.publish(msg)

        def _rangefinder_count(self) -> int:
            return self._message_counts.get("DISTANCE_SENSOR", 0) + self._message_counts.get("RANGEFINDER", 0)

        def _count_sent_command(self, name: str) -> None:
            self._sent_commands[name] = self._sent_commands.get(name, 0) + 1

        def write_summary(self, *, ok: bool, reason: str, landing_ok: bool) -> None:
            if not args.summary_file:
                return
            drift = summarize_hover_drift(self._hover_samples)
            hover_z_ned = self._hover_samples[-1][3] if self._hover_samples else self._current_z
            target_z_ned = self._target_z_ned()
            if hover_z_ned is not None:
                altitude_error_m = abs(float(hover_z_ned) - float(target_z_ned))
            elif self._current_range_m is not None:
                altitude_error_m = abs(float(self._current_range_m) - float(args.takeoff_alt_m))
            else:
                altitude_error_m = None
            landing_summary = self._landing_summary()
            external_nav_age_sec = -1.0
            if self._last_external_status_monotonic > 0.0:
                external_nav_age_sec = time.monotonic() - self._last_external_status_monotonic
            external_nav_ready = (
                self._external_nav_ready
                and self._last_external_status_monotonic > 0.0
                and external_nav_age_sec <= args.status_timeout_sec
            )
            summary = {
                "ok": ok,
                "reason": reason,
                "hover_body_ok": self._hover_body_ok,
                "landing_ok": landing_ok,
                "phases_seen": sorted(self._phases_seen),
                "phase_counts": dict(sorted(self._phase_counts.items())),
                "status_history": self._status_history[-40:],
                "mode": args.mode,
                "mode_number": self.mode_number,
                "guided_seen": self._guided_seen_ever or self._expected_mode_seen or self._external_expected_mode_seen,
                "armed_seen": self._armed_seen_ever or self._armed_seen or self._external_armed_seen,
                "airborne_seen": self._airborne_seen,
                "takeoff_ack_ok": command_ack_success(self._command_acks, mavlink.MAV_CMD_NAV_TAKEOFF),
                "arm_ack_ok": command_ack_success(self._command_acks, mavlink.MAV_CMD_COMPONENT_ARM_DISARM),
                "crash_detected": self._crash_detected,
                "setpoints_sent_count": self._setpoints_sent,
                "local_position_count": self._message_counts.get("LOCAL_POSITION_NED", 0),
                "rangefinder_count": self._rangefinder_count(),
                "target_alt_m": args.takeoff_alt_m,
                "takeoff_alt_m": args.takeoff_alt_m,
                "ground_z_ned": self._ground_z_ned,
                "target_z_ned": target_z_ned,
                "current_z_ned": hover_z_ned,
                "current_height_m": self._current_range_m,
                "altitude_error_m": altitude_error_m,
                "preflight_ready_sec": args.preflight_ready_sec,
                "hover_settle_sec": args.hover_settle_sec,
                "hover_altitude_tolerance_m": args.hover_altitude_tolerance_m,
                "hover_hold_sec": args.hover_hold_sec,
                "hover_hold_duration_sec": drift.duration_sec,
                "require_external_nav": args.require_external_nav,
                "external_nav_ready": external_nav_ready,
                "external_nav_status_age_sec": external_nav_age_sec,
                "external_nav_status_history": self._external_nav_status_history[-40:],
                "require_imu_status": args.require_imu_status,
                "send_position_setpoints": args.send_position_setpoints,
                "hover_drift": {
                    "sample_count": drift.sample_count,
                    "duration_sec": drift.duration_sec,
                    "horizontal_span_m": json_safe_number(drift.horizontal_span_m),
                    "z_span_m": json_safe_number(drift.z_span_m),
                    "horizontal_drift_m": json_safe_number(drift.horizontal_drift_m),
                    "z_drift_m": json_safe_number(drift.z_drift_m),
                    "max_horizontal_drift_m": args.max_horizontal_drift_m,
                    "max_altitude_drift_m": args.max_altitude_drift_m,
                    "duration_tolerance_sec": HOVER_DURATION_TOLERANCE_SEC,
                    "horizontal_span_ok": drift.horizontal_span_m <= args.max_horizontal_drift_m,
                    "horizontal_drift_ok": drift.horizontal_drift_m <= args.max_horizontal_drift_m,
                    "z_span_ok": drift.z_span_m <= args.max_altitude_drift_m,
                    "duration_ok": drift.duration_sec >= args.hover_hold_sec - HOVER_DURATION_TOLERANCE_SEC,
                    "ok": (
                        drift.ok
                        and drift.duration_sec >= args.hover_hold_sec - HOVER_DURATION_TOLERANCE_SEC
                        and drift.horizontal_drift_m <= args.max_horizontal_drift_m
                        and drift.z_span_m <= args.max_altitude_drift_m
                    ),
                },
                "last_position": {"x": self._current_x, "y": self._current_y, "z_ned": self._current_z},
                "last_yaw_rad": self._current_yaw_rad,
                "hold_yaw_rad": self._hold_yaw_rad,
                "message_counts": self._message_counts,
                "sent_commands": dict(sorted(self._sent_commands.items())),
                "command_acks": self._command_acks[-60:],
                "statustext": self._statustext[-60:],
                "ekf_flags_seen": sorted(set(self._ekf_flags)),
                "gps_global_origin_seen": self._gps_global_origin_seen,
                "home_position_seen": self._home_position_seen,
                "land_command_accepted": landing_summary["land_command_accepted"],
                "touchdown_confirmed": landing_summary["touchdown_confirmed"],
                "disarmed": landing_summary["disarmed"],
                "motors_safe": landing_summary["motors_safe"],
                "landing": landing_summary,
            }
            path = Path(args.summary_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = path.with_name(path.name + ".tmp")
            tmp_path.write_text(json.dumps(summary, allow_nan=False, indent=2, sort_keys=True), encoding="utf-8")
            tmp_path.replace(path)

    rclpy.init(args=None)
    node = MavlinkHoverMissionController()
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
