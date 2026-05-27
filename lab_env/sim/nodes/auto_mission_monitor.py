from __future__ import annotations

import argparse
import json
import os
import time

from lab_env.config import load_runtime_config, load_sim_config
from lab_env.logging_utils import configure_sim_logging, logger
from lab_env.sim.status import DEFAULT_SIM_LOG_TOPIC

_TERMINAL_STATES = {"blocked_by_stop_guard", "complete"}


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Watch /sim/log and report auto-mission progress.")
    parser.add_argument(
        "--status-topic",
        default=DEFAULT_SIM_LOG_TOPIC,
        help="JSON log topic emitted by sim components.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=300.0,
        help="Maximum time to wait for the auto mission to reach a terminal state.",
    )
    parser.add_argument(
        "--progress-period-sec",
        type=float,
        default=1.0,
        help="Minimum interval between running-state progress lines.",
    )
    return parser


def run(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    sim_config = load_sim_config(load_runtime_config())
    configure_sim_logging(
        log_file=os.environ.get("SIM_AUTO_LOG_FILE", "").strip() or None,
        console_level=sim_config.console_log_level.value,
        file_level=sim_config.file_log_level.value,
    )

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "auto_mission_monitor requires ROS2 Python packages. "
            "Run it through the sim-runtime ROS env wrapper or source the overlay first."
        ) from exc

    class AutoMissionMonitor(Node):
        def __init__(self) -> None:
            super().__init__("auto_mission_monitor")
            self._terminal_state: str | None = None
            self._last_line = ""
            self._last_progress_time = 0.0
            self._seen_executor_ready = False
            self._seen_mission_loaded = False
            self.create_subscription(String, args.status_topic, self._handle_status, 10)

        @property
        def terminal_state(self) -> str | None:
            return self._terminal_state

        def _emit(self, level: str, line: str, *, force: bool = False) -> None:
            if not force and line == self._last_line:
                return
            self._last_line = line
            getattr(logger, level.lower())(line)

        def _handle_status(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                self._emit("warning", f"[sim] ignoring non-json status payload: {message.data}", force=True)
                return

            source = str(payload.get("source") or "")
            event = str(payload.get("event") or "")

            if source == "cmd_vel_executor" and event == "executor_ready" and not self._seen_executor_ready:
                self._seen_executor_ready = True
                self._emit("info", "[sim] executor ready")
                return

            if source != "waypoint_follower":
                return

            mission_state = str(payload.get("mission_state") or "")
            current_x = payload.get("current_x")
            front_min = payload.get("front_min")
            goal_x = payload.get("goal_x")
            cmd_linear_x = payload.get("cmd_linear_x")

            if event == "mission_loaded" and not self._seen_mission_loaded:
                self._seen_mission_loaded = True
                start_x = payload.get("start_x")
                stop_distance = payload.get("stop_distance")
                forward_speed = payload.get("forward_speed")
                self._emit(
                    "info",
                    "[sim] mission loaded: "
                    f"start_x={start_x} goal_x={goal_x} stop_distance={stop_distance} forward_speed={forward_speed}",
                )
                return

            if mission_state == "ready":
                self._emit("info", f"[sim] waiting: {event}")
                return

            if mission_state == "running":
                now = time.monotonic()
                if now - self._last_progress_time < args.progress_period_sec:
                    return
                self._last_progress_time = now
                self._emit(
                    "info",
                    "[sim] running: "
                    f"event={event} current_x={current_x} goal_x={goal_x} front_min={front_min} cmd_linear_x={cmd_linear_x}",
                )
                return

            if mission_state in _TERMINAL_STATES:
                self._terminal_state = mission_state
                level = "warning" if mission_state == "blocked_by_stop_guard" else "info"
                self._emit(
                    level,
                    "[sim] mission finished: "
                    f"state={mission_state} event={event} current_x={current_x} goal_x={goal_x} front_min={front_min}",
                    force=True,
                )
                rclpy.try_shutdown()

    deadline = time.monotonic() + args.timeout_sec
    rclpy.init(args=None)
    node = AutoMissionMonitor()
    try:
        while rclpy.ok() and node.terminal_state is None:
            if time.monotonic() >= deadline:
                logger.error("[sim] timeout waiting for auto mission after {:.1f}s", args.timeout_sec)
                return 3
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    return 0 if node.terminal_state == "complete" else 2
