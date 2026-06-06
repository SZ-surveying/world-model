from __future__ import annotations

import json
import math
import os
import shlex
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

import tomli_w
from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.config import RunConfig
from src.tasks.base import OrchestrationTask
from src.tasks.fcu_controller import (
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
from src.tasks.frame_contract import (
    _append_p5_blockers,
    _build_p5_doctor_summary,
    _run_frame_probe,
    _write_frame_probe_script,
    _write_p5_runtime_config,
)
from src.tasks.official_baseline import (
    _build_doctor_summary,
    _collect_official_dds_probe,
    _collect_ros_graph,
    _load_rosbag_metadata_counts,
    _validate_official_rosbag_profile,
    _write_json,
    _write_text,
)
from src.tasks.official_maze_x2 import (
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
from src.tasks.rangefinder_imu import (
    OFFICIAL_GAZEBO_IRIS_PARAMS,
    OFFICIAL_IRIS_WITH_LIDAR_MODEL,
    _collect_imu_probe,
    _collect_rangefinder_probe,
    _write_p2_model_overlay,
    _write_p2_param_overlay,
    _write_p2_sensor_config,
)
from src.tasks.registry import TaskRegistry
from src.tasks.slam_backend import (
    SLAM_BACKEND_CONTAINER,
    _append_slam_odom_quality_blockers,
    _build_p3_doctor_summary,
    _collect_odometry_probe,
    _start_p3_slam_container,
    _write_p3_slam_runtime_config,
)

P6_ROSBAG_CONTAINER = "navlab-p6-rosbag"


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


def _p6_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.slam_hover_rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    if not profile_path.is_file() or not topics:
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
    profile_path = Path(config.slam_hover_rosbag_profile)
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


def _build_p6_doctor_summary(config: RunConfig, *, runtime_config: Path) -> dict[str, Any]:
    p6 = config.orchestration.slam_hover
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
    profile_path = Path(p6.rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
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
            "slam_odom_topic": p6.slam_odom_topic,
            "external_nav_status_topic": p6.external_nav_status_topic,
            "hover_status_topic": p6.hover_status_topic,
            "uses_gazebo_truth_as_input": p6.uses_gazebo_truth_as_input,
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
                f"- Diagnostic truth only: `{p6.truth_diagnostic_topic}`.",
                "- Do not use Gazebo truth as a SLAM, ExternalNav, planning, or control input.",
            ]
        )
        + "\n",
    )


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class SlamHoverDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "slam-hover-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check P6 real SLAM hover prerequisites."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_slam_hover_doctor/{config.run_id}"))
        artifact_dir.mkdir(parents=True, exist_ok=True)
        config = RunConfig.from_config(config_path=config_path, artifact_dir=artifact_dir, run_id=config.run_id)
        runtime_config = artifact_dir / "p6_slam_hover_runtime.toml"
        _write_p6_runtime_config(config, runtime_config)
        console.print("[bold cyan]Checking P6 real SLAM hover prerequisites[/bold cyan]")
        summary = _build_p6_doctor_summary(config, runtime_config=runtime_config)
        _write_json(artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P6 SLAM hover doctor rc={0 if summary['ok'] else 20}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 20


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class SlamHoverAcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "slam-hover-acceptance"
    TASK_DESCRIPTION: ClassVar[str] = "Run P6 real SLAM hover acceptance."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        duration_sec: float = 90.0,
        console: Console | None = None,
    ) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
        config.artifact_dir.mkdir(parents=True, exist_ok=True)
        host._render_run_config(console, config)
        baseline = config.orchestration.official_baseline
        p2 = config.orchestration.rangefinder_imu
        p3 = config.orchestration.slam_backend
        p4 = config.orchestration.fcu_controller
        p5 = config.orchestration.frame_contract
        p6 = config.orchestration.slam_hover
        bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p6.yaml"
        model_overlay = config.artifact_dir / "iris_with_lidar_p6_rangefinder_x2.sdf"
        param_overlay = config.artifact_dir / "gazebo-iris-p6-rangefinder.parm"
        sensor_config = config.artifact_dir / "p6_gazebo_sensor_runtime.toml"
        vendor_profile = config.artifact_dir / "x2_vendor_driver_p6.yaml"
        slam_runtime_config = config.artifact_dir / "p6_slam_runtime.toml"
        p4_runtime_config = config.artifact_dir / "p6_fcu_controller_runtime.toml"
        p5_runtime_config = config.artifact_dir / "p6_frame_contract_runtime.toml"
        p6_runtime_config = config.artifact_dir / "p6_slam_hover_runtime.toml"
        controller_script = config.artifact_dir / "p6_fcu_controller_runtime.py"
        frame_probe_script = config.artifact_dir / "p6_frame_contract_probe.py"
        hover_probe_script = config.artifact_dir / "p6_slam_hover_probe.py"
        _write_p1_bridge_override(bridge_override)
        _write_p1_vendor_profile(vendor_profile, virtual_serial_link=p2.x2_virtual_serial_link)

        summary: dict[str, Any] | None = None
        try:
            model_overlay_summary = _write_p2_model_overlay(config, model_overlay)
            param_overlay_summary = _write_p2_param_overlay(config, param_overlay)
            sensor_config_summary = _write_p2_sensor_config(config, sensor_config, vendor_profile=vendor_profile)
            slam_runtime_summary = _write_p3_slam_runtime_config(config, slam_runtime_config)
            p4_runtime_summary = _write_p4_runtime_config(config, p4_runtime_config)
            p5_runtime_summary = _write_p5_runtime_config(config, p5_runtime_config)
            p6_runtime_summary = _write_p6_runtime_config(config, p6_runtime_config)
            controller_hold_sec = p6.settle_window_sec + p6.hover_window_sec + p6.final_hold_window_sec + 6.0
            controller_script_summary = _write_controller_runtime_script(
                config,
                controller_script,
                duration_sec=max(55.0, min(duration_sec, 95.0)),
                hold_after_ready_sec=controller_hold_sec,
            )
            frame_probe_script_summary = _write_frame_probe_script(config, frame_probe_script)
            hover_probe_script_summary = _write_hover_probe_script(config, hover_probe_script)

            console.print("[bold cyan]Starting official maze + P6 real SLAM hover gate[/bold cyan]")
            try:
                host._compose_stop(config)
            except DockerException:
                pass
            host._start_official_baseline_container(
                config,
                volume_overrides=[
                    (bridge_override.resolve(), OFFICIAL_IRIS_3D_BRIDGE_CONFIG),
                    (model_overlay.resolve(), OFFICIAL_IRIS_WITH_LIDAR_MODEL),
                    (param_overlay.resolve(), OFFICIAL_GAZEBO_IRIS_PARAMS),
                ],
            )
            time.sleep(min(max(duration_sec, 1.0), 10.0))
            _start_gazebo_sensor_container(config, sensor_config=sensor_config)
            time.sleep(8.0)
            _start_p3_slam_container(config, runtime_config=slam_runtime_config)
            time.sleep(4.0)
            _start_p6_rosbag_recording(config, duration_sec=max(70.0, min(duration_sec, 115.0)))
            time.sleep(2.0)
            _start_p4_controller_container(config, script_path=controller_script)
            time.sleep(4.0)
            frame_summary = _run_frame_probe(config, script_path=frame_probe_script)
            hover_summary = _run_hover_probe(config, script_path=hover_probe_script)
            controller_summary = _wait_for_controller_summary(config, timeout_sec=max(45.0, duration_sec + 30.0))
            rosbag_profile = _finish_p6_rosbag_recording(config)
            counts = _message_counts(config)

            graph = _collect_ros_graph(config, config.artifact_dir, image=baseline.runtime_image, network="host")
            probe = _collect_official_dds_probe(config, config.artifact_dir, image=baseline.runtime_image, network="host")
            rangefinder_probe = _collect_rangefinder_probe(config, image=baseline.runtime_image)
            imu_probe = _collect_imu_probe(config, image=baseline.runtime_image)
            slam_odom_probe = _collect_odometry_probe(
                config,
                image=baseline.runtime_image,
                topic=p6.slam_odom_topic,
                artifact_name="p6_slam_odom_probe.txt",
            )
            topic_info = _collect_topic_info(
                config,
                image=baseline.runtime_image,
                topics=(
                    p6.slam_odom_topic,
                    p6.slam_status_topic,
                    p6.external_nav_status_topic,
                    p6.fcu_pose_topic,
                    p6.fcu_twist_topic,
                    p6.fcu_status_topic,
                    p6.cmd_vel_topic,
                    p6.rangefinder_range_topic,
                    p6.rangefinder_status_topic,
                    p6.imu_topic,
                    p6.truth_diagnostic_topic,
                    p6.controller_status_topic,
                    p6.setpoint_intent_topic,
                    p6.setpoint_output_topic,
                    p6.owner_status_topic,
                    p6.hover_status_topic,
                ),
            )
            doctor = _build_p6_doctor_summary(config, runtime_config=p6_runtime_config)
            blockers: list[str] = []
            if not doctor.get("ok"):
                blockers.extend(str(item) for item in doctor.get("blockers", []))
            if not probe.get("result", {}).get("time_received"):
                blockers.append("official DDS probe did not receive /ap/v1/time")
            if not rangefinder_probe.get("result", {}).get("range_received"):
                blockers.append("P6 did not receive rangefinder")
            if not imu_probe.get("result", {}).get("received"):
                blockers.append("P6 did not receive IMU")
            _append_slam_odom_quality_blockers(
                blockers=blockers,
                p3=p3,
                slam_odom_result=slam_odom_probe.get("result", {}),
            )
            _append_controller_blockers(blockers=blockers, controller=controller_summary)
            owner_summary = controller_summary.get("owner", {}) if controller_summary else {}
            cmd_vel_publishers = topic_info.get(p4.cmd_vel_topic, {}).get("publisher_nodes", [])
            _append_owner_blockers(
                blockers=blockers,
                owner_summary=owner_summary,
                cmd_vel_publishers=cmd_vel_publishers,
                p4=p4,
            )
            _append_p5_blockers(
                blockers=blockers,
                frame_summary=frame_summary,
                rosbag_profile={"ok": True},
                counts={p5.status_topic: max(1, counts.get(p5.status_topic, 0))},
                p5=p5,
            )
            _append_p6_blockers(
                blockers=blockers,
                hover_summary=hover_summary,
                rosbag_profile=rosbag_profile,
                counts=counts,
                p6=p6,
            )
            for topic in (
                p6.slam_odom_topic,
                p6.external_nav_status_topic,
                p6.fcu_pose_topic,
                p6.rangefinder_range_topic,
                p6.imu_topic,
                p6.hover_status_topic,
            ):
                if counts.get(topic, 0) <= 0:
                    blockers.append(f"{topic} was not recorded")

            summary = {
                "ok": not blockers,
                "blocked": bool(blockers),
                "blockers": blockers,
                "p6_slam_hover": {
                    "runtime_config": p6_runtime_summary,
                    "hover_probe_script": hover_probe_script_summary,
                    "hover_probe": hover_summary,
                    "model_overlay": model_overlay_summary,
                    "param_overlay": param_overlay_summary,
                    "sensor_config": sensor_config_summary,
                    "slam_runtime_config": slam_runtime_summary,
                    "p4_runtime_config": p4_runtime_summary,
                    "p5_runtime_config": p5_runtime_summary,
                    "controller_script": controller_script_summary,
                    "frame_probe_script": frame_probe_script_summary,
                    "controller_runtime": controller_summary,
                    "owner": owner_summary,
                    "external_nav_input_topic": p6.slam_odom_topic,
                    "uses_gazebo_truth_as_input": p6.uses_gazebo_truth_as_input,
                    "hover_claim": p6.hover_claim,
                    "exploration_claim": p6.exploration_claim,
                    "rosbag_path": str(config.artifact_dir / "rosbag"),
                    "mcap_path": str(config.artifact_dir / "rosbag" / "rosbag_0.mcap"),
                },
                "fcu": hover_summary.get("fcu", {}),
                "slam_odom": hover_summary.get("slam_odom", {}),
                "slam_odom_probe": slam_odom_probe.get("result", {}),
                "external_nav": hover_summary.get("external_nav", {}),
                "hover": hover_summary.get("hover", {}),
                "owner": hover_summary.get("owner", {}),
                "frame_contract": frame_summary,
                "official_dds_probe": probe,
                "rangefinder_probe": rangefinder_probe.get("result", {}),
                "imu_probe": imu_probe.get("result", {}),
                "topic_info": topic_info,
                "ros_graph": graph,
                "message_counts": counts,
                "rosbag_profile": rosbag_profile,
                "hover_claim": p6.hover_claim,
                "exploration_claim": p6.exploration_claim,
            }
            _write_json(config.artifact_dir / "summary.json", summary)
            _write_foxglove_notes(config)
        finally:
            host._capture_official_baseline_log(config=config)
            _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_tail.log")
            _capture_container_log(config, container=SLAM_BACKEND_CONTAINER, output_name="slam_backend_tail.log")
            _capture_container_log(config, container=P4_CONTROLLER_CONTAINER, output_name="fcu_controller_tail.log")
            _capture_container_log(config, container=P6_ROSBAG_CONTAINER, output_name="rosbag_tail.log")
            _remove_container(P6_ROSBAG_CONTAINER)
            _remove_container(P4_CONTROLLER_CONTAINER)
            _remove_container(SLAM_BACKEND_CONTAINER)
            _remove_container(GAZEBO_SENSOR_CONTAINER)
            host._remove_official_baseline_container()
            try:
                host._compose_stop(config)
            except DockerException:
                pass
        if summary is None:
            summary = {
                "ok": False,
                "blocked": True,
                "blockers": ["P6 SLAM hover acceptance did not produce a summary"],
            }
            _write_json(config.artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P6 SLAM hover acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 30
