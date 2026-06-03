from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Sequence

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Observe ArduPilot MAVLink state for Stage 1 P0.4.")
    parser.add_argument("--endpoint", default="tcp:mavlink-router:5760")
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument("--summary-file", required=True)
    parser.add_argument("--min-local-position-span-m", type=float, default=0.2)
    parser.add_argument("--allow-startup-unhealthy-sec", type=float, default=10.0)
    return parser.parse_args(argv)


def _request_streams(connection: mavutil.mavfile, target_system: int, target_component: int) -> None:
    connection.mav.request_data_stream_send(
        target_system,
        target_component,
        mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )
    for message_id in (
        mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        mavlink.MAVLINK_MSG_ID_ATTITUDE,
    ):
        connection.mav.command_long_send(
            target_system,
            target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            250000,
            0,
            0,
            0,
            0,
            0,
        )


def run(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    summary_path = Path(args.summary_file)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    connection = mavutil.mavlink_connection(args.endpoint, dialect="ardupilotmega")
    heartbeat = None
    heartbeat_deadline = time.monotonic() + 20.0
    while time.monotonic() < heartbeat_deadline:
        candidate = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if candidate is None:
            continue
        if (
            candidate.get_srcSystem() == 1
            and int(candidate.autopilot) != mavlink.MAV_AUTOPILOT_INVALID
        ):
            heartbeat = candidate
            break

    if heartbeat is None:
        summary_path.write_text(
            json.dumps({"ok": False, "reason": "autopilot_heartbeat_timeout"}, indent=2),
            encoding="utf-8",
        )
        return 2

    target_system = heartbeat.get_srcSystem()
    target_component = heartbeat.get_srcComponent()
    _request_streams(connection, target_system, target_component)

    counts: dict[str, int] = {}
    first_local_position: dict[str, float] | None = None
    last_local_position: dict[str, float] | None = None
    first_global_position: dict[str, float] | None = None
    last_global_position: dict[str, float] | None = None
    statustext: list[dict[str, float | str]] = []
    latest_visodom_unhealthy_sec: float | None = None
    ekf_flags: list[int] = []
    started = time.monotonic()
    next_request = started + 1.0

    while time.monotonic() - started < args.duration_sec:
        now = time.monotonic()
        if now >= next_request:
            _request_streams(connection, target_system, target_component)
            next_request = now + 2.0

        msg = connection.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue

        msg_type = msg.get_type()
        counts[msg_type] = counts.get(msg_type, 0) + 1

        if msg_type == "LOCAL_POSITION_NED":
            sample = {"x": float(msg.x), "y": float(msg.y), "z": float(msg.z)}
            if first_local_position is None:
                first_local_position = sample
            last_local_position = sample
        elif msg_type == "GLOBAL_POSITION_INT":
            sample = {
                "lat": float(msg.lat) / 10000000.0,
                "lon": float(msg.lon) / 10000000.0,
                "relative_alt_m": float(msg.relative_alt) / 1000.0,
            }
            if first_global_position is None:
                first_global_position = sample
            last_global_position = sample
        elif msg_type == "STATUSTEXT":
            text = str(msg.text)
            elapsed_sec = time.monotonic() - started
            if "VisOdom: not healthy" in text:
                latest_visodom_unhealthy_sec = elapsed_sec
            if len(statustext) < 40:
                statustext.append({"elapsed_sec": round(elapsed_sec, 3), "text": text})
        elif msg_type == "EKF_STATUS_REPORT":
            ekf_flags.append(int(msg.flags))

    local_span_m = 0.0
    if first_local_position is not None and last_local_position is not None:
        local_span_m = abs(last_local_position["x"] - first_local_position["x"])

    ok = (
        counts.get("HEARTBEAT", 0) > 0
        and counts.get("EKF_STATUS_REPORT", 0) > 0
        and counts.get("GLOBAL_POSITION_INT", 0) > 0
        and counts.get("LOCAL_POSITION_NED", 0) > 0
        and local_span_m >= args.min_local_position_span_m
        and (
            latest_visodom_unhealthy_sec is None
            or latest_visodom_unhealthy_sec <= args.allow_startup_unhealthy_sec
        )
    )
    summary = {
        "ok": ok,
        "endpoint": args.endpoint,
        "duration_sec": args.duration_sec,
        "target_system": target_system,
        "target_component": target_component,
        "message_counts": counts,
        "first_local_position_ned": first_local_position,
        "last_local_position_ned": last_local_position,
        "first_global_position_int": first_global_position,
        "last_global_position_int": last_global_position,
        "local_position_x_span_m": round(local_span_m, 4),
        "min_local_position_span_m": args.min_local_position_span_m,
        "ekf_flags_seen": sorted(set(ekf_flags)),
        "statustext_sample": statustext,
        "latest_visodom_unhealthy_sec": latest_visodom_unhealthy_sec,
        "allow_startup_unhealthy_sec": args.allow_startup_unhealthy_sec,
    }
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if ok else 3


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
