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


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a minimal ArduPilot SITL hold/hover check.")
    parser.add_argument("--endpoint", default="tcp:sitl:5763")
    parser.add_argument("--duration-sec", type=float, default=45.0)
    parser.add_argument("--summary-file", required=True)
    parser.add_argument("--mode", default="GUIDED")
    parser.add_argument("--takeoff-alt-m", type=float, default=0.8)
    parser.add_argument("--min-airborne-alt-m", type=float, default=0.25)
    parser.add_argument("--max-xy-drift-m", type=float, default=0.35)
    parser.add_argument("--max-yaw-drift-rad", type=float, default=0.35)
    parser.add_argument("--allow-startup-unhealthy-sec", type=float, default=10.0)
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


def _send_message_interval(
    connection: mavutil.mavfile, target_system: int, target_component: int, message_id: int, hz: float
) -> None:
    interval_us = int(1000000.0 / hz)
    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
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
    ):
        _send_message_interval(connection, target_system, target_component, message_id, hz)


def _wait_autopilot_heartbeat(connection: mavutil.mavfile, timeout_sec: float) -> Any | None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        candidate = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if candidate is None:
            continue
        if candidate.get_srcSystem() == 1 and int(candidate.autopilot) != mavlink.MAV_AUTOPILOT_INVALID:
            return candidate
    return None


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


def _span(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    return max(values) - min(values)


def run(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    summary_path = Path(args.summary_file)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    mode_number = _mode_number(args.mode)
    connection = None
    connect_deadline = time.monotonic() + 20.0
    while time.monotonic() < connect_deadline:
        try:
            connection = mavutil.mavlink_connection(args.endpoint, dialect="ardupilotmega")
            break
        except OSError:
            time.sleep(1.0)
    if connection is None:
        summary_path.write_text(
            json.dumps({"ok": False, "reason": "mavlink_connection_refused", "endpoint": args.endpoint}, indent=2),
            encoding="utf-8",
        )
        return 2
    heartbeat = _wait_autopilot_heartbeat(connection, timeout_sec=25.0)
    if heartbeat is None:
        summary_path.write_text(
            json.dumps({"ok": False, "reason": "autopilot_heartbeat_timeout"}, indent=2),
            encoding="utf-8",
        )
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
    takeoff_command_sent = False
    hold_started: float | None = None
    latest_visodom_unhealthy_sec: float | None = None

    counts: dict[str, int] = {}
    command_acks: list[dict[str, Any]] = []
    statustext: list[dict[str, Any]] = []
    local_positions: list[dict[str, float]] = []
    hold_x: list[float] = []
    hold_y: list[float] = []
    hold_yaw: list[float] = []
    relative_alts_m: list[float] = []
    ekf_flags: list[int] = []
    modes_seen: list[int] = []
    landed_states_seen: list[int] = []
    armed_seen = False
    expected_mode_seen = False
    airborne_seen = False
    last_mode = int(getattr(heartbeat, "custom_mode", -1))

    while time.monotonic() - started < args.duration_sec:
        now = time.monotonic()
        elapsed = now - started
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

        msg = connection.recv_match(blocking=True, timeout=0.25)
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
            sample = {"x": float(msg.x), "y": float(msg.y), "z": float(msg.z)}
            local_positions.append(sample)
            if float(msg.z) <= -args.min_airborne_alt_m:
                airborne_seen = True
            if hold_started is not None:
                hold_x.append(sample["x"])
                hold_y.append(sample["y"])
        elif msg_type == "GLOBAL_POSITION_INT":
            relative_alt_m = float(msg.relative_alt) / 1000.0
            relative_alts_m.append(relative_alt_m)
            if relative_alt_m >= args.min_airborne_alt_m:
                airborne_seen = True
        elif msg_type == "ATTITUDE":
            if hold_started is not None:
                hold_yaw.append(float(msg.yaw))
        elif msg_type == "EXTENDED_SYS_STATE":
            landed_state = int(msg.landed_state)
            landed_states_seen.append(landed_state)
            if landed_state in (
                mavlink.MAV_LANDED_STATE_TAKEOFF,
                mavlink.MAV_LANDED_STATE_IN_AIR,
            ):
                airborne_seen = True
        elif msg_type == "EKF_STATUS_REPORT":
            ekf_flags.append(int(msg.flags))
        elif msg_type == "STATUSTEXT":
            text = str(msg.text)
            if "VisOdom: not healthy" in text:
                latest_visodom_unhealthy_sec = elapsed
            if len(statustext) < 60:
                statustext.append({"elapsed_sec": round(elapsed, 3), "text": text})

        if hold_started is None and expected_mode_seen and armed_seen and airborne_seen:
            hold_started = time.monotonic()

    first_position = local_positions[0] if local_positions else None
    last_position = local_positions[-1] if local_positions else None
    max_relative_alt_m = max(relative_alts_m) if relative_alts_m else None
    min_local_z_m = min((sample["z"] for sample in local_positions), default=None)
    hold_x_span_m = _span(hold_x)
    hold_y_span_m = _span(hold_y)
    hold_yaw_span_rad = _span(hold_yaw)
    hold_duration_sec = 0.0 if hold_started is None else max(0.0, time.monotonic() - hold_started)
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
    ok = (
        expected_mode_seen
        and last_mode == mode_number
        and armed_seen
        and takeoff_command_sent
        and airborne_seen
        and hold_duration_sec >= max(5.0, args.duration_sec * 0.25)
        and hold_x_span_m <= args.max_xy_drift_m
        and hold_y_span_m <= args.max_xy_drift_m
        and hold_yaw_span_rad <= args.max_yaw_drift_rad
        and counts.get("LOCAL_POSITION_NED", 0) > 0
        and counts.get("EKF_STATUS_REPORT", 0) > 0
        and not persistent_visodom_unhealthy
        and not rejected_commands
    )
    summary = {
        "ok": ok,
        "endpoint": args.endpoint,
        "duration_sec": args.duration_sec,
        "mode": args.mode.upper(),
        "mode_number": mode_number,
        "last_mode_number": last_mode,
        "expected_mode_seen": expected_mode_seen,
        "armed_seen": armed_seen,
        "takeoff_command_sent": takeoff_command_sent,
        "airborne_seen": airborne_seen,
        "hold_duration_sec": round(hold_duration_sec, 3),
        "hold_x_span_m": round(hold_x_span_m, 4),
        "hold_y_span_m": round(hold_y_span_m, 4),
        "hold_yaw_span_rad": round(hold_yaw_span_rad, 4),
        "max_xy_drift_m": args.max_xy_drift_m,
        "max_yaw_drift_rad": args.max_yaw_drift_rad,
        "takeoff_alt_m": args.takeoff_alt_m,
        "min_airborne_alt_m": args.min_airborne_alt_m,
        "max_relative_alt_m": None if max_relative_alt_m is None else round(max_relative_alt_m, 4),
        "min_local_z_m": None if min_local_z_m is None else round(min_local_z_m, 4),
        "message_counts": counts,
        "first_local_position_ned": first_position,
        "last_local_position_ned": last_position,
        "modes_seen": sorted(set(modes_seen)),
        "landed_states_seen": sorted(set(landed_states_seen)),
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
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if ok else 3


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
