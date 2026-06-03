from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Sequence

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink


XY_VELOCITY_Z_POSITION_TYPE_MASK = 3555


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a minimal GUIDED local velocity setpoint check.")
    parser.add_argument("--endpoint", default="tcp:sitl:5763")
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--summary-file", required=True)
    parser.add_argument("--mode", default="GUIDED")
    parser.add_argument("--takeoff-alt-m", type=float, default=0.8)
    parser.add_argument("--min-airborne-alt-m", type=float, default=0.25)
    parser.add_argument("--move-start-sec", type=float, default=12.0)
    parser.add_argument("--move-duration-sec", type=float, default=5.0)
    parser.add_argument("--stop-duration-sec", type=float, default=10.0)
    parser.add_argument("--velocity-x-mps", type=float, default=0.1)
    parser.add_argument("--min-horizontal-span-m", type=float, default=0.25)
    parser.add_argument("--max-stop-drift-m", type=float, default=0.12)
    parser.add_argument("--allow-startup-unhealthy-sec", type=float, default=10.0)
    parser.add_argument("--status-topic", default="/stage1/control/status")
    parser.add_argument("--disable-arming-checks", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--force-arm", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args(argv)


def _mode_number(mode_name: str) -> int:
    requested = mode_name.upper()
    for number, name in mavutil.mode_mapping_acm.items():
        if name == requested:
            return int(number)
    supported = ", ".join(sorted(mavutil.mode_mapping_acm.values()))
    raise ValueError(f"unsupported ArduCopter mode {mode_name!r}; supported: {supported}")


def _connect(endpoint: str, summary_path: Path) -> mavutil.mavfile | None:
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        try:
            return mavutil.mavlink_connection(endpoint, dialect="ardupilotmega")
        except OSError:
            time.sleep(1.0)
    summary_path.write_text(
        json.dumps({"ok": False, "reason": "mavlink_connection_refused", "endpoint": endpoint}, indent=2),
        encoding="utf-8",
    )
    return None


class ControlStatusPublisher:
    def __init__(self, topic: str) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
        except ModuleNotFoundError:
            self._enabled = False
            return

        rclpy.init(args=None)
        self._rclpy = rclpy
        self._message_type = String
        self._node: Node | None = rclpy.create_node("stage1_local_setpoint_control_status")
        self._publisher = self._node.create_publisher(String, topic, 10)
        self._enabled = True

    def publish(self, payload: dict[str, Any]) -> None:
        if not self._enabled or self._node is None:
            return
        message = self._message_type()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self._rclpy.spin_once(self._node, timeout_sec=0)

    def close(self) -> None:
        if not self._enabled or self._node is None:
            return
        self._node.destroy_node()
        self._rclpy.shutdown()
        self._node = None


def _wait_autopilot_heartbeat(connection: mavutil.mavfile, timeout_sec: float) -> Any | None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        candidate = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if candidate is None:
            continue
        if candidate.get_srcSystem() == 1 and int(candidate.autopilot) != mavlink.MAV_AUTOPILOT_INVALID:
            return candidate
    return None


def _send_message_interval(
    connection: mavutil.mavfile, target_system: int, target_component: int, message_id: int, hz: float
) -> None:
    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1000000.0 / hz),
        0,
        0,
        0,
        0,
        0,
    )


def _request_streams(connection: mavutil.mavfile, target_system: int, target_component: int) -> None:
    connection.mav.request_data_stream_send(
        target_system,
        target_component,
        mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )
    for message_id, hz in (
        (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0),
        (mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 4.0),
        (mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0),
        (mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 4.0),
        (mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 4.0),
        (mavlink.MAVLINK_MSG_ID_STATUSTEXT, 2.0),
        (mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 5.0),
    ):
        _send_message_interval(connection, target_system, target_component, message_id, hz)


def _set_mode(connection: mavutil.mavfile, target_system: int, mode_number: int) -> None:
    connection.mav.set_mode_send(
        target_system,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_number,
    )


def _set_arming_check(
    connection: mavutil.mavfile, target_system: int, target_component: int, value: int
) -> None:
    connection.mav.param_set_send(
        target_system,
        target_component,
        b"ARMING_CHECK",
        float(value),
        mavlink.MAV_PARAM_TYPE_INT32,
    )


def _command_arm(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    force_arm: bool,
) -> None:
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


def _command_takeoff(
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    altitude_m: float,
) -> None:
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
    connection: mavutil.mavfile,
    target_system: int,
    target_component: int,
    vx_mps: float,
    vy_mps: float,
    z_ned_m: float,
) -> None:
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


def _horizontal_span(samples: list[dict[str, float]]) -> float:
    if len(samples) < 2:
        return 0.0
    xs = [sample["x"] for sample in samples]
    ys = [sample["y"] for sample in samples]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def _horizontal_distance(first: dict[str, float] | None, last: dict[str, float] | None) -> float:
    if first is None or last is None:
        return 0.0
    return math.hypot(last["x"] - first["x"], last["y"] - first["y"])


def run(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    summary_path = Path(args.summary_file)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    status_publisher = ControlStatusPublisher(args.status_topic)

    mode_number = _mode_number(args.mode)
    connection = _connect(args.endpoint, summary_path)
    if connection is None:
        status_publisher.publish(
            {"ok": False, "phase": "connect_failed", "endpoint": args.endpoint, "reason": "mavlink_connection_refused"}
        )
        status_publisher.close()
        return 2
    heartbeat = _wait_autopilot_heartbeat(connection, timeout_sec=25.0)
    if heartbeat is None:
        summary_path.write_text(
            json.dumps({"ok": False, "reason": "autopilot_heartbeat_timeout"}, indent=2),
            encoding="utf-8",
        )
        status_publisher.publish(
            {"ok": False, "phase": "connect_failed", "endpoint": args.endpoint, "reason": "autopilot_heartbeat_timeout"}
        )
        status_publisher.close()
        return 2

    target_system = heartbeat.get_srcSystem()
    target_component = heartbeat.get_srcComponent()
    _request_streams(connection, target_system, target_component)
    if args.disable_arming_checks:
        _set_arming_check(connection, target_system, target_component, 0)

    started = time.monotonic()
    next_request = started + 1.0
    next_mode_command = started
    next_arm_command = started + 2.0
    next_setpoint = started
    next_status_publish = started
    takeoff_command_sent = False
    airborne_started: float | None = None
    latest_visodom_unhealthy_sec: float | None = None
    move_started_at: float | None = None
    stop_started_at: float | None = None

    counts: dict[str, int] = {}
    command_acks: list[dict[str, Any]] = []
    statustext: list[dict[str, Any]] = []
    local_positions: list[dict[str, float]] = []
    move_positions: list[dict[str, float]] = []
    stop_positions: list[dict[str, float]] = []
    relative_alts_m: list[float] = []
    ekf_flags: list[int] = []
    modes_seen: list[int] = []
    setpoints_sent: list[dict[str, float]] = []
    armed_seen = False
    expected_mode_seen = False
    airborne_seen = False
    airborne_state_seen = False
    last_mode = int(getattr(heartbeat, "custom_mode", -1))
    phase = "preflight"

    while time.monotonic() - started < args.duration_sec:
        now = time.monotonic()
        elapsed = now - started
        if now >= next_status_publish:
            status_publisher.publish(
                {
                    "elapsed_sec": round(elapsed, 3),
                    "phase": phase,
                    "mode": args.mode.upper(),
                    "mode_number": mode_number,
                    "last_mode_number": last_mode,
                    "expected_mode_seen": expected_mode_seen,
                    "armed_seen": armed_seen,
                    "airborne_seen": airborne_seen,
                    "setpoints_sent_count": len(setpoints_sent),
                    "last_setpoint_vx_mps": setpoints_sent[-1]["vx_mps"] if setpoints_sent else None,
                    "z_hold_target_ned_m": -args.takeoff_alt_m,
                }
            )
            next_status_publish = now + 1.0
        if now >= next_request:
            _request_streams(connection, target_system, target_component)
            next_request = now + 2.0
        if not expected_mode_seen and now >= next_mode_command:
            _set_mode(connection, target_system, mode_number)
            next_mode_command = now + 1.0
        if expected_mode_seen and not armed_seen and now >= next_arm_command:
            _command_arm(connection, target_system, target_component, args.force_arm)
            next_arm_command = now + 2.0
        if expected_mode_seen and armed_seen and not takeoff_command_sent:
            _command_takeoff(connection, target_system, target_component, args.takeoff_alt_m)
            takeoff_command_sent = True

        if airborne_started is not None:
            motion_elapsed = now - airborne_started
            move_end = args.move_start_sec + args.move_duration_sec
            stop_end = move_end + args.stop_duration_sec
            if motion_elapsed < args.move_start_sec:
                phase = "initial_hold"
                desired_vx = 0.0
            elif motion_elapsed < move_end:
                phase = "move"
                desired_vx = args.velocity_x_mps
                if move_started_at is None:
                    move_started_at = now
            elif motion_elapsed < stop_end:
                phase = "stop_hold"
                desired_vx = 0.0
                if stop_started_at is None:
                    stop_started_at = now
            else:
                phase = "final_hold"
                desired_vx = 0.0
            if now >= next_setpoint:
                _send_velocity_z_hold_setpoint(
                    connection,
                    target_system,
                    target_component,
                    desired_vx,
                    0.0,
                    -args.takeoff_alt_m,
                )
                setpoints_sent.append({"elapsed_sec": round(elapsed, 3), "vx_mps": desired_vx})
                next_setpoint = now + 0.2

        msg = connection.recv_match(blocking=True, timeout=0.05)
        if msg is None:
            continue

        msg_type = msg.get_type()
        counts[msg_type] = counts.get(msg_type, 0) + 1
        if msg_type == "HEARTBEAT":
            if msg.get_srcSystem() != target_system or int(msg.autopilot) == mavlink.MAV_AUTOPILOT_INVALID:
                continue
            last_mode = int(msg.custom_mode)
            modes_seen.append(last_mode)
            expected_mode_seen = expected_mode_seen or last_mode == mode_number
            armed_seen = armed_seen or bool(int(msg.base_mode) & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif msg_type == "COMMAND_ACK":
            command_acks.append(
                {
                    "elapsed_sec": round(elapsed, 3),
                    "command": int(msg.command),
                    "result": int(msg.result),
                }
            )
        elif msg_type == "LOCAL_POSITION_NED":
            sample = {"elapsed_sec": round(elapsed, 3), "x": float(msg.x), "y": float(msg.y), "z": float(msg.z)}
            local_positions.append(sample)
            if float(msg.z) <= -args.min_airborne_alt_m:
                airborne_seen = True
            if phase == "move":
                move_positions.append(sample)
            elif phase in {"stop_hold", "final_hold"}:
                stop_positions.append(sample)
        elif msg_type == "GLOBAL_POSITION_INT":
            relative_alt_m = float(msg.relative_alt) / 1000.0
            relative_alts_m.append(relative_alt_m)
            if relative_alt_m >= args.min_airborne_alt_m:
                airborne_seen = True
        elif msg_type == "EXTENDED_SYS_STATE":
            if int(msg.landed_state) in (
                mavlink.MAV_LANDED_STATE_TAKEOFF,
                mavlink.MAV_LANDED_STATE_IN_AIR,
            ):
                airborne_state_seen = True
        elif msg_type == "EKF_STATUS_REPORT":
            ekf_flags.append(int(msg.flags))
        elif msg_type == "STATUSTEXT":
            text = str(msg.text)
            if "VisOdom: not healthy" in text:
                latest_visodom_unhealthy_sec = elapsed
            if len(statustext) < 60:
                statustext.append({"elapsed_sec": round(elapsed, 3), "text": text})

        if airborne_started is None and expected_mode_seen and armed_seen and airborne_seen:
            airborne_started = time.monotonic()

    first_position = local_positions[0] if local_positions else None
    last_position = local_positions[-1] if local_positions else None
    horizontal_span_m = _horizontal_span(local_positions)
    move_span_m = _horizontal_span(move_positions)
    stop_drift_m = _horizontal_distance(stop_positions[0], stop_positions[-1]) if len(stop_positions) >= 2 else 0.0
    max_relative_alt_m = max(relative_alts_m) if relative_alts_m else None
    persistent_visodom_unhealthy = (
        latest_visodom_unhealthy_sec is not None
        and latest_visodom_unhealthy_sec > args.allow_startup_unhealthy_sec
    )
    rejected_commands = [
        ack
        for ack in command_acks
        if ack["command"] in {mavlink.MAV_CMD_COMPONENT_ARM_DISARM, mavlink.MAV_CMD_NAV_TAKEOFF}
        and ack["result"] not in {mavlink.MAV_RESULT_ACCEPTED, mavlink.MAV_RESULT_IN_PROGRESS}
    ]
    sent_move_setpoints = [sample for sample in setpoints_sent if sample["vx_mps"] != 0.0]
    sent_stop_setpoints = [sample for sample in setpoints_sent if sample["vx_mps"] == 0.0 and stop_started_at is not None]
    ok = (
        expected_mode_seen
        and last_mode == mode_number
        and armed_seen
        and takeoff_command_sent
        and airborne_seen
        and len(sent_move_setpoints) >= 5
        and len(sent_stop_setpoints) >= 5
        and horizontal_span_m >= args.min_horizontal_span_m
        and stop_drift_m <= args.max_stop_drift_m
        and counts.get("LOCAL_POSITION_NED", 0) > 0
        and counts.get("EKF_STATUS_REPORT", 0) > 0
        and not persistent_visodom_unhealthy
        and not rejected_commands
    )
    summary = {
        "ok": ok,
        "endpoint": args.endpoint,
        "status_topic": args.status_topic,
        "duration_sec": args.duration_sec,
        "mode": args.mode.upper(),
        "mode_number": mode_number,
        "last_mode_number": last_mode,
        "expected_mode_seen": expected_mode_seen,
        "armed_seen": armed_seen,
        "takeoff_command_sent": takeoff_command_sent,
        "airborne_seen": airborne_seen,
        "airborne_state_seen": airborne_state_seen,
        "velocity_x_mps": args.velocity_x_mps,
        "z_hold_target_ned_m": -args.takeoff_alt_m,
        "move_duration_sec": args.move_duration_sec,
        "stop_duration_sec": args.stop_duration_sec,
        "setpoints_sent_count": len(setpoints_sent),
        "move_setpoints_sent_count": len(sent_move_setpoints),
        "stop_setpoints_sent_count": len(sent_stop_setpoints),
        "horizontal_span_m": round(horizontal_span_m, 4),
        "move_span_m": round(move_span_m, 4),
        "stop_drift_m": round(stop_drift_m, 4),
        "min_horizontal_span_m": args.min_horizontal_span_m,
        "max_stop_drift_m": args.max_stop_drift_m,
        "max_relative_alt_m": None if max_relative_alt_m is None else round(max_relative_alt_m, 4),
        "message_counts": counts,
        "first_local_position_ned": first_position,
        "last_local_position_ned": last_position,
        "modes_seen": sorted(set(modes_seen)),
        "ekf_flags_seen": sorted(set(ekf_flags)),
        "command_acks": command_acks[-40:],
        "rejected_commands": rejected_commands,
        "statustext_sample": statustext,
        "latest_visodom_unhealthy_sec": latest_visodom_unhealthy_sec,
        "allow_startup_unhealthy_sec": args.allow_startup_unhealthy_sec,
        "disable_arming_checks": args.disable_arming_checks,
        "force_arm": args.force_arm,
    }
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    status_publisher.publish(
        {
            "ok": ok,
            "phase": "complete",
            "horizontal_span_m": round(horizontal_span_m, 4),
            "stop_drift_m": round(stop_drift_m, 4),
            "setpoints_sent_count": len(setpoints_sent),
            "move_setpoints_sent_count": len(sent_move_setpoints),
            "stop_setpoints_sent_count": len(sent_stop_setpoints),
            "latest_visodom_unhealthy_sec": latest_visodom_unhealthy_sec,
        }
    )
    status_publisher.close()
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if ok else 3


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
