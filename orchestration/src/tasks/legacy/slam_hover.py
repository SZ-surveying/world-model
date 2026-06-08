from __future__ import annotations

import json
import math
import os
import shlex
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import tomli_w
from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.config import RunConfig
from src.tasks.legacy.fcu_controller import (
    P4_CONTROLLER_CONTAINER,
    _append_controller_blockers,
    _append_owner_blockers,
    _build_p4_doctor_summary,
    _source_official_setup,
    _start_p4_controller_container,
    _wait_for_controller_summary,
    _write_controller_runtime_script,
    _write_p4_runtime_config,
)
from src.tasks.legacy.frame_contract import (
    _append_p5_blockers,
    _build_p5_doctor_summary,
    _run_frame_probe,
    _write_frame_probe_script,
    _write_p5_runtime_config,
)
from src.tasks.legacy.official_baseline import (
    _build_doctor_summary,
    _collect_official_dds_probe,
    _collect_ros_graph,
    _load_rosbag_metadata_counts,
    _validate_official_rosbag_profile,
    _write_json,
    _write_text,
)
from src.tasks.legacy.official_maze_x2 import (
    GAZEBO_SENSOR_CONTAINER,
    OFFICIAL_IRIS_3D_BRIDGE_CONFIG,
    _capture_container_log,
    _collect_topic_info,
    _file_sha256,
    _profile_topics,
    _remove_container,
    _start_gazebo_sensor_container,
    _write_p1_bridge_override,
    _write_p1_vendor_profile,
)
from src.tasks.legacy.rangefinder_imu import (
    OFFICIAL_GAZEBO_IRIS_PARAMS,
    OFFICIAL_IRIS_WITH_LIDAR_MODEL,
    _collect_imu_probe,
    _collect_rangefinder_probe,
    _write_p2_model_overlay,
    _write_p2_param_overlay,
    _write_p2_sensor_config,
)
from src.tasks.legacy.slam_backend import (
    SLAM_BACKEND_CONTAINER,
    _append_slam_odom_quality_blockers,
    _build_p3_doctor_summary,
    _collect_odometry_probe,
    _start_p3_slam_container,
    _write_p3_slam_runtime_config,
)

P6_ROSBAG_CONTAINER = "navlab-p6-rosbag"
P6_VEHICLE_MARKER_CONTAINER = "navlab-p6-vehicle-markers"


def _baseline_env(config: RunConfig) -> dict[str, str]:
    baseline = config.orchestration.official_baseline
    return {
        "DDS_ENABLE": baseline.dds_enable,
        "DDS_DOMAIN_ID": baseline.dds_domain_id,
        "ROS_DOMAIN_ID": baseline.dds_domain_id,
        "RMW_IMPLEMENTATION": baseline.rmw_implementation,
    }


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return _load_rosbag_metadata_counts(metadata)


def _write_p6_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    p6 = config.orchestration.slam_hover
    data = {
        "slam_hover": {
            "runtime": {
                "slam_odom_topic": p6.slam_odom_topic,
                "slam_status_topic": p6.slam_status_topic,
                "external_nav_status_topic": p6.external_nav_status_topic,
                "fcu_pose_topic": p6.fcu_pose_topic,
                "fcu_twist_topic": p6.fcu_twist_topic,
                "fcu_status_topic": p6.fcu_status_topic,
                "cmd_vel_topic": p6.cmd_vel_topic,
                "rangefinder_range_topic": p6.rangefinder_range_topic,
                "rangefinder_status_topic": p6.rangefinder_status_topic,
                "imu_topic": p6.imu_topic,
                "truth_diagnostic_topic": p6.truth_diagnostic_topic,
                "controller_status_topic": p6.controller_status_topic,
                "setpoint_intent_topic": p6.setpoint_intent_topic,
                "setpoint_output_topic": p6.setpoint_output_topic,
                "owner_status_topic": p6.owner_status_topic,
                "hover_status_topic": p6.hover_status_topic,
                "vehicle_marker_topic": p6.vehicle_marker_topic,
                "vehicle_marker_pose_topic": p6.vehicle_marker_pose_topic,
                "vehicle_marker_frame_id": p6.vehicle_marker_frame_id,
                "vehicle_marker_rate_hz": p6.vehicle_marker_rate_hz,
                "record_visualization_markers": p6.record_visualization_markers,
                "settle_window_sec": p6.settle_window_sec,
                "hover_window_sec": p6.hover_window_sec,
                "final_hold_window_sec": p6.final_hold_window_sec,
                "max_hover_horizontal_drift_m": p6.max_hover_horizontal_drift_m,
                "max_hover_altitude_error_m": p6.max_hover_altitude_error_m,
                "max_hover_yaw_drift_rad": p6.max_hover_yaw_drift_rad,
                "max_stop_drift_m": p6.max_stop_drift_m,
                "min_slam_odom_rate_hz": p6.min_slam_odom_rate_hz,
                "min_external_nav_rate_hz": p6.min_external_nav_rate_hz,
                "min_fcu_local_position_rate_hz": p6.min_fcu_local_position_rate_hz,
                "max_latest_age_sec": p6.max_latest_age_sec,
                "uses_gazebo_truth_as_input": p6.uses_gazebo_truth_as_input,
                "hover_claim": p6.hover_claim,
                "exploration_claim": p6.exploration_claim,
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _hover_probe_script(spec: dict[str, Any]) -> str:
    spec_json = json.dumps(spec, sort_keys=True)
    return f"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String

SPEC = json.loads({spec_json!r})


def now_ms() -> int:
    return int(time.time() * 1000)


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_from_pose_msg(msg: PoseStamped) -> tuple[float, float, float, float]:
    p = msg.pose.position
    return float(p.x), float(p.y), float(p.z), yaw_from_quat(msg.pose.orientation)


def point_from_odom_msg(msg: Odometry) -> tuple[float, float, float, float]:
    p = msg.pose.pose.position
    return float(p.x), float(p.y), float(p.z), yaw_from_quat(msg.pose.pose.orientation)


def horizontal_span(samples: list[tuple[float, float, float, float, float]]) -> float | None:
    if len(samples) < 2:
        return None
    xs = [item[1] for item in samples]
    ys = [item[2] for item in samples]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def z_span(samples: list[tuple[float, float, float, float, float]]) -> float | None:
    if len(samples) < 2:
        return None
    values = [abs(item[3]) for item in samples]
    return max(values) - min(values)


def yaw_span(samples: list[tuple[float, float, float, float, float]]) -> float | None:
    if len(samples) < 2:
        return None
    yaws = [item[4] for item in samples]
    return max(yaws) - min(yaws)


def rate(count: int, duration: float) -> float:
    return count / max(duration, 0.001)


class SlamHoverProbe:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("navlab_slam_hover_probe")
        self.started = time.monotonic()
        self.ready_at = None
        self.phase = "wait_ready"
        self.counts = {{
            "fcu_pose": 0,
            "fcu_twist": 0,
            "slam_odom": 0,
            "truth_odom": 0,
            "range": 0,
            "range_status": 0,
            "imu": 0,
            "slam_status": 0,
            "external_nav_status": 0,
            "controller_status": 0,
            "setpoint_intent": 0,
            "setpoint_output": 0,
            "owner_status": 0,
        }}
        self.latest = {{}}
        self.samples = {{"fcu_pose": [], "slam_odom": [], "truth_odom": []}}
        self.hover_samples = {{"fcu_pose": [], "slam_odom": [], "truth_odom": []}}
        self.final_samples = {{"fcu_pose": [], "slam_odom": [], "truth_odom": []}}
        self.status_pub = self.node.create_publisher(String, SPEC["hover_status_topic"], 10)
        self.fcu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.node.create_subscription(PoseStamped, SPEC["fcu_pose_topic"], self._pose_cb, self.fcu_qos)
        self.node.create_subscription(TwistStamped, SPEC["fcu_twist_topic"], self._touch_cb("fcu_twist"), self.fcu_qos)
        self.node.create_subscription(Odometry, SPEC["slam_odom_topic"], self._odom_cb("slam_odom"), qos_profile_sensor_data)
        self.node.create_subscription(
            Odometry,
            SPEC["truth_diagnostic_topic"],
            self._odom_cb("truth_odom"),
            qos_profile_sensor_data,
        )
        self.node.create_subscription(Range, SPEC["rangefinder_range_topic"], self._range_cb, qos_profile_sensor_data)
        self.node.create_subscription(String, SPEC["rangefinder_status_topic"], self._string_cb("range_status"), 10)
        self.node.create_subscription(Imu, SPEC["imu_topic"], self._touch_cb("imu"), qos_profile_sensor_data)
        self.node.create_subscription(String, SPEC["slam_status_topic"], self._string_cb("slam_status"), 10)
        self.node.create_subscription(String, SPEC["external_nav_status_topic"], self._string_cb("external_nav_status"), 10)
        self.node.create_subscription(String, SPEC["controller_status_topic"], self._string_cb("controller_status"), 10)
        self.node.create_subscription(String, SPEC["setpoint_intent_topic"], self._string_cb("setpoint_intent"), 10)
        self.node.create_subscription(String, SPEC["setpoint_output_topic"], self._string_cb("setpoint_output"), 10)
        self.node.create_subscription(String, SPEC["owner_status_topic"], self._string_cb("owner_status"), 10)

    def _record_motion_sample(self, key: str, sample: tuple[float, float, float, float]) -> None:
        stamped = (time.monotonic(), *sample)
        self.samples[key].append(stamped)
        if self.phase == "hover":
            self.hover_samples[key].append(stamped)
        elif self.phase == "final_hold":
            self.final_samples[key].append(stamped)

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.counts["fcu_pose"] += 1
        sample = point_from_pose_msg(msg)
        self.latest["fcu_pose"] = {{"frame_id": msg.header.frame_id, "position": sample[:3], "yaw": sample[3], "monotonic": time.monotonic()}}
        self._record_motion_sample("fcu_pose", sample)

    def _odom_cb(self, key: str):
        def callback(msg: Odometry) -> None:
            self.counts[key] += 1
            sample = point_from_odom_msg(msg)
            self.latest[key] = {{
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "position": sample[:3],
                "yaw": sample[3],
                "monotonic": time.monotonic(),
            }}
            self._record_motion_sample(key, sample)
        return callback

    def _range_cb(self, msg: Range) -> None:
        self.counts["range"] += 1
        self.latest["range"] = {{"frame_id": msg.header.frame_id, "range": float(msg.range), "monotonic": time.monotonic()}}

    def _touch_cb(self, key: str):
        def callback(msg) -> None:
            self.counts[key] += 1
            self.latest[key] = {{"frame_id": getattr(getattr(msg, "header", None), "frame_id", ""), "monotonic": time.monotonic()}}
        return callback

    def _string_cb(self, key: str):
        def callback(msg: String) -> None:
            self.counts[key] += 1
            sample = {{"raw": msg.data, "monotonic": time.monotonic()}}
            try:
                sample["json"] = json.loads(msg.data)
            except json.JSONDecodeError:
                sample["json"] = None
            self.latest[key] = sample
        return callback

    def controller_ready(self) -> bool:
        payload = self.latest.get("controller_status", {{}}).get("json") or {{}}
        return bool(payload.get("ready") or payload.get("state") in {{"hold_ready", "complete"}})

    def _latest_age(self, key: str) -> float | None:
        monotonic = self.latest.get(key, {{}}).get("monotonic")
        return None if monotonic is None else time.monotonic() - float(monotonic)

    def _external_nav(self) -> dict:
        payload = self.latest.get("external_nav_status", {{}}).get("json") or {{}}
        odom = payload.get("odom") if isinstance(payload.get("odom"), dict) else {{}}
        input_topic = odom.get("input_topic") or payload.get("input_topic") or SPEC["slam_odom_topic"]
        rate_hz = float(odom.get("rate_hz") or payload.get("rate_hz") or 0.0)
        ready = bool(payload.get("ready") or payload.get("state") == "healthy" or odom.get("fresh"))
        return {{
            "ok": ready and input_topic == SPEC["slam_odom_topic"] and rate_hz >= float(SPEC["min_external_nav_rate_hz"]),
            "state": payload.get("state", "unknown"),
            "ready": ready,
            "input_topic": input_topic,
            "rate_hz": rate_hz,
            "sent_count": payload.get("sent_count") or payload.get("output_count") or self.counts["external_nav_status"],
            "latest_age_sec": self._latest_age("external_nav_status"),
            "uses_gazebo_truth_as_input": bool(SPEC["uses_gazebo_truth_as_input"]),
            "sample": payload,
        }}

    def _window_summary(self, samples: dict[str, list]) -> dict:
        fcu = samples["fcu_pose"]
        slam = samples["slam_odom"]
        truth = samples["truth_odom"]
        altitude_error = None
        if fcu:
            target = float(SPEC["takeoff_alt_m"])
            altitude_error = max(abs(abs(item[3]) - target) for item in fcu)
        return {{
            "fcu_sample_count": len(fcu),
            "slam_sample_count": len(slam),
            "truth_sample_count": len(truth),
            "fcu_horizontal_drift_m": horizontal_span(fcu),
            "slam_horizontal_drift_m": horizontal_span(slam),
            "truth_horizontal_drift_m": horizontal_span(truth),
            "altitude_error_m": altitude_error,
            "fcu_z_span_m": z_span(fcu),
            "yaw_drift_rad": yaw_span(fcu),
        }}

    def summary(self) -> dict:
        elapsed = time.monotonic() - self.started
        external_nav = self._external_nav()
        hover = self._window_summary(self.hover_samples)
        final_hold = self._window_summary(self.final_samples)
        fcu_rate = rate(self.counts["fcu_pose"], elapsed)
        slam_rate = rate(self.counts["slam_odom"], elapsed)
        hover_ok = (
            hover["fcu_sample_count"] > 0
            and hover["slam_sample_count"] > 0
            and hover["fcu_horizontal_drift_m"] is not None
            and hover["fcu_horizontal_drift_m"] <= float(SPEC["max_hover_horizontal_drift_m"])
            and hover["altitude_error_m"] is not None
            and hover["altitude_error_m"] <= float(SPEC["max_hover_altitude_error_m"])
            and (hover["yaw_drift_rad"] is None or hover["yaw_drift_rad"] <= float(SPEC["max_hover_yaw_drift_rad"]))
        )
        stop_drift = final_hold["fcu_horizontal_drift_m"]
        final_ok = stop_drift is None or stop_drift <= float(SPEC["max_stop_drift_m"])
        owner = self.latest.get("owner_status", {{}}).get("json") or {{}}
        blockers = []
        if not self.ready_at:
            blockers.append("P6 controller never reached ready state")
        if self.counts["range"] <= 0 or self.counts["range_status"] <= 0:
            blockers.append("P6 rangefinder was not observed")
        if self.counts["imu"] <= 0:
            blockers.append("P6 IMU was not observed")
        if fcu_rate < float(SPEC["min_fcu_local_position_rate_hz"]):
            blockers.append("P6 FCU local position rate is below minimum")
        if slam_rate < float(SPEC["min_slam_odom_rate_hz"]):
            blockers.append("P6 SLAM odom rate is below minimum")
        if self._latest_age("fcu_pose") is None or self._latest_age("fcu_pose") > float(SPEC["max_latest_age_sec"]):
            blockers.append("P6 FCU local position latest age is too high")
        if self._latest_age("slam_odom") is None or self._latest_age("slam_odom") > float(SPEC["max_latest_age_sec"]):
            blockers.append("P6 SLAM odom latest age is too high")
        if not external_nav["ok"]:
            blockers.append("P6 ExternalNav is not healthy")
        if not hover_ok:
            blockers.append("P6 hover drift gate did not pass")
        if not final_ok:
            blockers.append("P6 stop drift gate did not pass")
        if owner and not owner.get("unique", True):
            blockers.append("P6 setpoint owner is not unique")
        if owner.get("set_pose_count", 0) != 0:
            blockers.append("P6 direct set pose count is non-zero")
        if SPEC["uses_gazebo_truth_as_input"]:
            blockers.append("P6 is configured to use Gazebo truth as input")
        return {{
            "ok": not blockers,
            "blockers": blockers,
            "phase": self.phase,
            "elapsed_sec": elapsed,
            "p6_slam_hover": {{
                "ok": not blockers,
                "hover_claim": SPEC["hover_claim"],
                "exploration_claim": SPEC["exploration_claim"],
                "control_route": "unique_fcu_controller",
                "external_nav_input_topic": SPEC["slam_odom_topic"],
                "uses_gazebo_truth_as_input": bool(SPEC["uses_gazebo_truth_as_input"]),
            }},
            "fcu": {{
                "local_position_ok": self.counts["fcu_pose"] > 0,
                "local_position_count": self.counts["fcu_pose"],
                "local_position_rate_hz": fcu_rate,
                "latest_age_sec": self._latest_age("fcu_pose"),
            }},
            "slam_odom": {{
                "ok": self.counts["slam_odom"] > 0 and slam_rate >= float(SPEC["min_slam_odom_rate_hz"]),
                "topic": SPEC["slam_odom_topic"],
                "count": self.counts["slam_odom"],
                "rate_hz": slam_rate,
                "latest_age_sec": self._latest_age("slam_odom"),
            }},
            "external_nav": external_nav,
            "hover": {{
                "ok": hover_ok and final_ok,
                "settle_window_sec": float(SPEC["settle_window_sec"]),
                "window_sec": float(SPEC["hover_window_sec"]),
                "final_hold_window_sec": float(SPEC["final_hold_window_sec"]),
                "horizontal_drift_m": hover["fcu_horizontal_drift_m"],
                "altitude_error_m": hover["altitude_error_m"],
                "yaw_drift_rad": hover["yaw_drift_rad"],
                "stop_drift_m": stop_drift,
                "hover_window": hover,
                "final_hold_window": final_hold,
            }},
            "owner": {{
                "unique": owner.get("unique", True),
                "owner": owner.get("owner"),
                "owner_id": owner.get("owner_id"),
                "set_pose_count": owner.get("set_pose_count", 0),
                "competing_publishers": owner.get("competing_publishers", []),
            }},
            "counts": self.counts,
            "latest": self.latest,
        }}

    def publish_status(self, final: bool = False) -> dict:
        payload = self.summary()
        payload["source"] = "navlab_slam_hover_probe"
        payload["final"] = final
        payload["updated_ms"] = now_ms()
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)
        return payload

    def run(self) -> dict:
        ready_deadline = time.monotonic() + float(SPEC["ready_timeout_sec"])
        next_status = 0.0
        while time.monotonic() < ready_deadline and not self.controller_ready():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if time.monotonic() >= next_status:
                self.publish_status(final=False)
                next_status = time.monotonic() + 0.5
        if self.controller_ready():
            self.ready_at = time.monotonic()
        else:
            summary = self.publish_status(final=True)
            Path(SPEC["summary_file"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
            self.node.destroy_node()
            rclpy.shutdown()
            return summary
        phases = [
            ("settle", float(SPEC["settle_window_sec"])),
            ("hover", float(SPEC["hover_window_sec"])),
            ("final_hold", float(SPEC["final_hold_window_sec"])),
        ]
        for phase, duration in phases:
            self.phase = phase
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if time.monotonic() >= next_status:
                    self.publish_status(final=False)
                    next_status = time.monotonic() + 0.5
        self.phase = "complete"
        summary = self.publish_status(final=True)
        Path(SPEC["summary_file"]).parent.mkdir(parents=True, exist_ok=True)
        Path(SPEC["summary_file"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
        print(json.dumps(summary, sort_keys=True))
        self.node.destroy_node()
        rclpy.shutdown()
        return summary


raise SystemExit(0 if SlamHoverProbe().run()["ok"] else 30)
"""


def _write_hover_probe_script(config: RunConfig, script_path: Path) -> dict[str, Any]:
    p6 = config.orchestration.slam_hover
    p4 = config.orchestration.fcu_controller
    summary_file = config.artifact_dir / "slam_hover_summary.json"
    spec = {
        "summary_file": host._workspace_path(summary_file),
        "slam_odom_topic": p6.slam_odom_topic,
        "slam_status_topic": p6.slam_status_topic,
        "external_nav_status_topic": p6.external_nav_status_topic,
        "fcu_pose_topic": p6.fcu_pose_topic,
        "fcu_twist_topic": p6.fcu_twist_topic,
        "fcu_status_topic": p6.fcu_status_topic,
        "cmd_vel_topic": p6.cmd_vel_topic,
        "rangefinder_range_topic": p6.rangefinder_range_topic,
        "rangefinder_status_topic": p6.rangefinder_status_topic,
        "imu_topic": p6.imu_topic,
        "truth_diagnostic_topic": p6.truth_diagnostic_topic,
        "controller_status_topic": p6.controller_status_topic,
        "setpoint_intent_topic": p6.setpoint_intent_topic,
        "setpoint_output_topic": p6.setpoint_output_topic,
        "owner_status_topic": p6.owner_status_topic,
        "hover_status_topic": p6.hover_status_topic,
        "settle_window_sec": p6.settle_window_sec,
        "hover_window_sec": p6.hover_window_sec,
        "final_hold_window_sec": p6.final_hold_window_sec,
        "max_hover_horizontal_drift_m": p6.max_hover_horizontal_drift_m,
        "max_hover_altitude_error_m": p6.max_hover_altitude_error_m,
        "max_hover_yaw_drift_rad": p6.max_hover_yaw_drift_rad,
        "max_stop_drift_m": p6.max_stop_drift_m,
        "min_slam_odom_rate_hz": p6.min_slam_odom_rate_hz,
        "min_external_nav_rate_hz": p6.min_external_nav_rate_hz,
        "min_fcu_local_position_rate_hz": p6.min_fcu_local_position_rate_hz,
        "max_latest_age_sec": p6.max_latest_age_sec,
        "uses_gazebo_truth_as_input": p6.uses_gazebo_truth_as_input,
        "hover_claim": p6.hover_claim,
        "exploration_claim": p6.exploration_claim,
        "takeoff_alt_m": p4.takeoff_alt_m,
        "ready_timeout_sec": p4.readiness_timeout_sec + 25.0,
    }
    script_path.parent.mkdir(parents=True, exist_ok=True)
    _write_text(script_path, _hover_probe_script(spec))
    return {
        "path": str(script_path),
        "workspace_path": host._workspace_path(script_path),
        "sha256": _file_sha256(script_path),
        "summary_file": str(summary_file),
        "spec": spec,
    }


def _run_hover_probe(config: RunConfig, *, script_path: Path) -> dict[str, Any]:
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.orchestration.official_baseline.runtime_image,
        shell_command=f"python3 {shlex.quote(host._workspace_path(script_path))}",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "slam_hover_probe.txt", output)
    summary = _load_json(config.artifact_dir / "slam_hover_summary.json")
    if not summary:
        summary = {"ok": False, "blockers": [f"hover probe failed rc={rc}"], "output": output}
    summary["rc"] = rc
    return summary


def _start_p6_vehicle_marker_container(config: RunConfig) -> None:
    p6 = config.orchestration.slam_hover
    if not p6.record_visualization_markers:
        _remove_container(P6_VEHICLE_MARKER_CONTAINER)
        _write_text(
            config.artifact_dir / "vehicle_markers_tail.log",
            "visualization marker recording disabled by slam_hover.record_visualization_markers=false\n",
        )
        return
    _remove_container(P6_VEHICLE_MARKER_CONTAINER)
    args = [
        "python3",
        "-m",
        "navlab.companion.nodes.vehicle_markers",
        "--pose-topic",
        p6.vehicle_marker_pose_topic,
        "--topic",
        p6.vehicle_marker_topic,
        "--rate",
        f"{p6.vehicle_marker_rate_hz:g}",
    ]
    if p6.vehicle_marker_frame_id:
        args.extend(["--frame-id", p6.vehicle_marker_frame_id])
    command = " ".join(shlex.quote(item) for item in args)
    DockerClient().run(
        config.orchestration.official_baseline.runtime_image,
        ["bash", "-lc", _source_official_setup(command)],
        detach=True,
        name=P6_VEHICLE_MARKER_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={**_baseline_env(config), "PYTHONPATH": "/workspace"},
    )


def _dedupe_topics(topics: list[str]) -> list[str]:
    seen: set[str] = set()
    deduped: list[str] = []
    for topic in topics:
        if topic in seen:
            continue
        seen.add(topic)
        deduped.append(topic)
    return deduped


def _p6_effective_profile_path(config: RunConfig) -> Path:
    return config.artifact_dir / "p6_effective_rosbag_profile.txt"


def _write_p6_effective_rosbag_profile(config: RunConfig) -> tuple[Path, list[str], list[str], list[str]]:
    p6 = config.orchestration.slam_hover
    source_profile = Path(config.slam_hover_rosbag_profile)
    required, optional, _topics = _profile_topics(source_profile)
    if p6.record_visualization_markers:
        required = _dedupe_topics([*required, p6.vehicle_marker_topic])
        optional = [topic for topic in optional if topic not in required]
    profile_path = _p6_effective_profile_path(config)
    profile_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Generated by NavLab P6 acceptance.",
        f"# Source profile: {source_profile}",
        f"# record_visualization_markers: {str(p6.record_visualization_markers).lower()}",
        "",
        "# Required topics.",
        *(f"required {topic}" for topic in required),
    ]
    if optional:
        lines.extend(["", "# Optional topics.", *(f"optional {topic}" for topic in optional)])
    profile_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return profile_path, required, optional, [*required, *optional]


def _p6_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    source_profile = Path(config.slam_hover_rosbag_profile)
    profile_path, required, optional, topics = _write_p6_effective_rosbag_profile(config)
    if not source_profile.is_file() or not topics:
        return profile_path, required, optional, ""
    container_rosbag = Path("/workspace") / config.artifact_dir / "rosbag"
    topic_args = " ".join(shlex.quote(topic) for topic in topics)
    command = (
        f"rm -rf {shlex.quote(str(container_rosbag))} && "
        f"mkdir -p {shlex.quote(str(container_rosbag.parent))} && "
        "set +e; "
        f"timeout --signal=INT {duration_sec:g} "
        f"ros2 bag record -s mcap -o {shlex.quote(str(container_rosbag))} --topics {topic_args}; "
        "rc=$?; "
        "set -e; "
        'if [ "$rc" != "0" ] && [ "$rc" != "124" ] && [ "$rc" != "130" ]; then exit "$rc"; fi; '
        f"for i in $(seq 1 40); do [ -f {shlex.quote(str(container_rosbag / 'metadata.yaml'))} ] && exit 0; "
        "sleep 0.25; done; exit 2"
    )
    return profile_path, required, optional, command


def _start_p6_rosbag_recording(config: RunConfig, *, duration_sec: float) -> None:
    _remove_container(P6_ROSBAG_CONTAINER)
    profile_path, required, optional, command = _p6_rosbag_shell_command(config, duration_sec=duration_sec)
    if not command:
        _write_json(
            config.artifact_dir / "rosbag_profile_summary.json",
            {
                "ok": False,
                "recorded": False,
                "profile": str(profile_path),
                "required_topics": required,
                "optional_topics": optional,
                "reason": "rosbag profile missing or empty",
            },
        )
        return
    DockerClient().run(
        config.orchestration.official_baseline.runtime_image,
        ["bash", "-lc", _source_official_setup(command)],
        detach=True,
        name=P6_ROSBAG_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={**_baseline_env(config), "PYTHONPATH": "/workspace"},
    )


def _finish_p6_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    profile_path = _p6_effective_profile_path(config)
    required, optional, _topics = _profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    try:
        rc = DockerClient().wait(P6_ROSBAG_CONTAINER)
    except DockerException as exc:
        rc = exc.return_code or 1
    try:
        output = DockerClient().logs(P6_ROSBAG_CONTAINER, tail=2000)
    except DockerException as exc:
        output = str(exc)
    _write_text(config.artifact_dir / "rosbag_record.txt", str(output))
    for _ in range(160):
        if metadata.is_file():
            break
        time.sleep(0.25)
    if rc != 0 or not metadata.is_file():
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "reason": f"rosbag record failed rc={rc}",
            "record_output": str(output),
        }
        _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
        return summary
    summary = _validate_official_rosbag_profile(
        profile=profile_path,
        metadata=metadata,
        required=required,
        optional=optional,
    )
    summary["rosbag_path"] = str(config.artifact_dir / "rosbag")
    summary["mcap_path"] = str(config.artifact_dir / "rosbag" / "rosbag_0.mcap")
    _write_json(config.artifact_dir / "rosbag_profile_summary.json", summary)
    return summary


def _build_p6_doctor_summary(
    config: RunConfig,
    *,
    runtime_config: Path,
    include_dependencies: bool = True,
) -> dict[str, Any]:
    p6 = config.orchestration.slam_hover
    if include_dependencies:
        baseline_doctor = _build_doctor_summary(config)
        p3_runtime_config = config.artifact_dir / "p6_doctor_p3_slam_runtime.toml"
        _write_p3_slam_runtime_config(config, p3_runtime_config)
        p3_doctor = _build_p3_doctor_summary(config, runtime_config=p3_runtime_config)
        p4_runtime_config = config.artifact_dir / "p6_doctor_p4_fcu_controller_runtime.toml"
        _write_p4_runtime_config(config, p4_runtime_config)
        p4_doctor = _build_p4_doctor_summary(config, runtime_config=p4_runtime_config)
        p5_runtime_config = config.artifact_dir / "p6_doctor_p5_frame_contract_runtime.toml"
        _write_p5_runtime_config(config, p5_runtime_config)
        p5_doctor = _build_p5_doctor_summary(config, runtime_config=p5_runtime_config)
    else:
        baseline_doctor = {"ok": True, "blockers": [], "skipped": "acceptance already launched official stack"}
        p3_doctor = {"ok": True, "blockers": [], "skipped": "acceptance already launched SLAM backend"}
        p4_doctor = {"ok": True, "blockers": [], "skipped": "acceptance already launched FCU controller"}
        p5_doctor = {"ok": True, "blockers": [], "skipped": "acceptance already ran frame probe"}
    profile_path = Path(p6.rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    if p6.record_visualization_markers:
        required = _dedupe_topics([*required, p6.vehicle_marker_topic])
        optional = [topic for topic in optional if topic not in required]
        topics = [*required, *optional]
    blockers = [
        *[str(item) for item in baseline_doctor.get("blockers", [])],
        *[str(item) for item in p3_doctor.get("blockers", [])],
        *[str(item) for item in p4_doctor.get("blockers", [])],
        *[str(item) for item in p5_doctor.get("blockers", [])],
    ]
    if not profile_path.is_file() or not topics:
        blockers.append("P6 rosbag profile is missing or empty")
    if p6.uses_gazebo_truth_as_input:
        blockers.append("P6 must not use Gazebo truth as a control/planning/SLAM/ExternalNav input")
    if p6.slam_odom_topic != config.orchestration.slam_backend.slam_odom_topic:
        blockers.append("P6 SLAM odom topic must match P3 canonical SLAM odom topic")
    if p6.slam_odom_topic == p6.truth_diagnostic_topic:
        blockers.append("P6 SLAM odom topic must not be the Gazebo truth diagnostic topic")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "p6_slam_hover_doctor": {
            "runtime_config": str(runtime_config),
            "runtime_config_sha256": _file_sha256(runtime_config) if runtime_config.is_file() else "",
            "dependency_checks_included": include_dependencies,
            "slam_odom_topic": p6.slam_odom_topic,
            "external_nav_status_topic": p6.external_nav_status_topic,
            "hover_status_topic": p6.hover_status_topic,
            "vehicle_marker_topic": p6.vehicle_marker_topic,
            "vehicle_marker_pose_topic": p6.vehicle_marker_pose_topic,
            "uses_gazebo_truth_as_input": p6.uses_gazebo_truth_as_input,
            "record_visualization_markers": p6.record_visualization_markers,
            "hover_claim": p6.hover_claim,
            "exploration_claim": p6.exploration_claim,
            "thresholds": {
                "settle_window_sec": p6.settle_window_sec,
                "hover_window_sec": p6.hover_window_sec,
                "final_hold_window_sec": p6.final_hold_window_sec,
                "max_hover_horizontal_drift_m": p6.max_hover_horizontal_drift_m,
                "max_hover_altitude_error_m": p6.max_hover_altitude_error_m,
                "max_hover_yaw_drift_rad": p6.max_hover_yaw_drift_rad,
                "max_stop_drift_m": p6.max_stop_drift_m,
            },
            "rosbag_profile": {
                "profile": str(profile_path),
                "required_topics": required,
                "optional_topics": optional,
            },
        },
        "official_baseline_doctor": baseline_doctor,
        "p3_slam_backend_doctor": p3_doctor,
        "p4_fcu_controller_doctor": p4_doctor,
        "p5_frame_contract_doctor": p5_doctor,
    }


def _append_p6_blockers(
    *,
    blockers: list[str],
    hover_summary: dict[str, Any],
    rosbag_profile: dict[str, Any],
    counts: dict[str, int],
    p6: Any,
) -> None:
    if not hover_summary:
        blockers.append("P6 hover summary is missing")
        return
    if not hover_summary.get("ok"):
        blockers.extend(str(item) for item in hover_summary.get("blockers", []))
    if not hover_summary.get("slam_odom", {}).get("ok"):
        blockers.append("P6 SLAM odom gate did not pass")
    if not hover_summary.get("external_nav", {}).get("ok"):
        blockers.append("P6 ExternalNav gate did not pass")
    if not hover_summary.get("fcu", {}).get("local_position_ok"):
        blockers.append("P6 FCU local position gate did not pass")
    if not hover_summary.get("hover", {}).get("ok"):
        blockers.append("P6 hover drift gate did not pass")
    if p6.uses_gazebo_truth_as_input:
        blockers.append("P6 is configured to use Gazebo truth as input")
    if not rosbag_profile.get("ok"):
        blockers.append("P6 rosbag profile did not pass")
    if counts.get(p6.hover_status_topic, 0) <= 0:
        blockers.append(f"{p6.hover_status_topic} was not recorded")
    if p6.record_visualization_markers and counts.get(p6.vehicle_marker_topic, 0) <= 0:
        blockers.append(f"{p6.vehicle_marker_topic} was not recorded")


def _write_foxglove_notes(config: RunConfig) -> None:
    p6 = config.orchestration.slam_hover
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P6 SLAM hover replay notes",
                "",
                "P6 validates real SLAM ExternalNav hover. It is not a forward/yaw/exploration gate.",
                "",
                "- Fixed frame: `map`.",
                f"- SLAM odom: `{p6.slam_odom_topic}`.",
                f"- ExternalNav status: `{p6.external_nav_status_topic}`.",
                f"- FCU pose/twist: `{p6.fcu_pose_topic}`, `{p6.fcu_twist_topic}`.",
                f"- Hover status: `{p6.hover_status_topic}`.",
                f"- Vehicle shell markers enabled: `{p6.record_visualization_markers}`.",
                (
                    f"- Vehicle shell markers: `{p6.vehicle_marker_topic}` follows `{p6.vehicle_marker_pose_topic}`."
                    if p6.record_visualization_markers
                    else "- Vehicle shell markers are not recorded by default; set `slam_hover.record_visualization_markers=true` for a Foxglove visual shell."
                ),
                "- When enabled, the vehicle shell is primitive MarkerArray geometry, not file:// mesh resources, so cloud replay is portable.",
                f"- Diagnostic truth only: `{p6.truth_diagnostic_topic}`.",
                "- Do not use Gazebo truth as a SLAM, ExternalNav, planning, or control input.",
            ]
        )
        + "\n",
    )




