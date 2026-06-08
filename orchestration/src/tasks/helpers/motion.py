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
from src.tasks.helpers.fcu import (
    P4_CONTROLLER_CONTAINER,
    _append_controller_blockers,
    _append_owner_blockers,
    _start_p4_controller_container,
    _wait_for_controller_summary,
    _write_controller_runtime_script,
    _write_p4_runtime_config,
)
from src.tasks.helpers.frame_contract import _append_p5_blockers, _run_frame_probe, _write_frame_probe_script, _write_p5_runtime_config
from src.tasks.helpers.official_stack import (
    _collect_official_dds_probe,
    _collect_ros_graph,
    _load_rosbag_metadata_counts,
    _validate_official_rosbag_profile,
    _write_json,
    _write_text,
)
from src.tasks.helpers.navlab_models import (
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
from src.tasks.helpers.sensors import (
    OFFICIAL_GAZEBO_IRIS_PARAMS,
    OFFICIAL_IRIS_WITH_LIDAR_MODEL,
    _collect_imu_probe,
    _collect_rangefinder_probe,
    _write_p2_model_overlay,
    _write_p2_param_overlay,
    _write_p2_sensor_config,
)
from src.tasks.helpers.slam import (
    SLAM_BACKEND_CONTAINER,
    _append_slam_odom_quality_blockers,
    _collect_odometry_probe,
    _start_p3_slam_container,
    _write_p3_slam_runtime_config,
)
from src.tasks.helpers.slam_hover import (
    _baseline_env,
    _build_p6_doctor_summary,
    _load_json,
    _source_official_setup,
)

P7_ROSBAG_CONTAINER = "navlab-p7-rosbag"


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return _load_rosbag_metadata_counts(metadata)


def _write_p7_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    p7 = config.orchestration.motion_gate
    data = {
        "motion_gate": {
            "runtime": {
                "slam_odom_topic": p7.slam_odom_topic,
                "slam_status_topic": p7.slam_status_topic,
                "external_nav_status_topic": p7.external_nav_status_topic,
                "fcu_pose_topic": p7.fcu_pose_topic,
                "fcu_twist_topic": p7.fcu_twist_topic,
                "fcu_status_topic": p7.fcu_status_topic,
                "cmd_vel_topic": p7.cmd_vel_topic,
                "rangefinder_range_topic": p7.rangefinder_range_topic,
                "rangefinder_status_topic": p7.rangefinder_status_topic,
                "imu_topic": p7.imu_topic,
                "scan_topic": p7.scan_topic,
                "truth_diagnostic_topic": p7.truth_diagnostic_topic,
                "controller_status_topic": p7.controller_status_topic,
                "setpoint_intent_topic": p7.setpoint_intent_topic,
                "setpoint_output_topic": p7.setpoint_output_topic,
                "owner_status_topic": p7.owner_status_topic,
                "hover_status_topic": p7.hover_status_topic,
                "motion_status_topic": p7.motion_status_topic,
                "settle_window_sec": p7.settle_window_sec,
                "forward_window_sec": p7.forward_window_sec,
                "back_window_sec": p7.back_window_sec,
                "yaw_window_sec": p7.yaw_window_sec,
                "stop_hold_window_sec": p7.stop_hold_window_sec,
                "final_hold_window_sec": p7.final_hold_window_sec,
                "motion_distance_m": p7.motion_distance_m,
                "motion_speed_mps": p7.motion_speed_mps,
                "yaw_scan_rad": p7.yaw_scan_rad,
                "yaw_rate_radps": p7.yaw_rate_radps,
                "min_forward_displacement_m": p7.min_forward_displacement_m,
                "max_forward_displacement_m": p7.max_forward_displacement_m,
                "min_back_displacement_m": p7.min_back_displacement_m,
                "max_back_displacement_m": p7.max_back_displacement_m,
                "min_yaw_delta_rad": p7.min_yaw_delta_rad,
                "max_yaw_delta_rad": p7.max_yaw_delta_rad,
                "max_lateral_error_m": p7.max_lateral_error_m,
                "max_motion_altitude_error_m": p7.max_motion_altitude_error_m,
                "max_stop_drift_m": p7.max_stop_drift_m,
                "min_clearance_m": p7.min_clearance_m,
                "min_slam_odom_rate_hz": p7.min_slam_odom_rate_hz,
                "min_external_nav_rate_hz": p7.min_external_nav_rate_hz,
                "min_fcu_local_position_rate_hz": p7.min_fcu_local_position_rate_hz,
                "max_latest_age_sec": p7.max_latest_age_sec,
                "uses_gazebo_truth_as_input": p7.uses_gazebo_truth_as_input,
                "hover_claim": p7.hover_claim,
                "motion_claim": p7.motion_claim,
                "exploration_claim": p7.exploration_claim,
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _motion_probe_script(spec: dict[str, Any]) -> str:
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
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import String

SPEC = json.loads({spec_json!r})


def now_ms() -> int:
    return int(time.time() * 1000)


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_from_pose(msg: PoseStamped) -> tuple[float, float, float, float]:
    p = msg.pose.position
    return float(p.x), float(p.y), float(p.z), yaw_from_quat(msg.pose.orientation)


def point_from_odom(msg: Odometry) -> tuple[float, float, float, float]:
    p = msg.pose.pose.position
    return float(p.x), float(p.y), float(p.z), yaw_from_quat(msg.pose.pose.orientation)


def angle_delta(end: float, start: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


def rate(count: int, duration: float) -> float:
    return count / max(duration, 0.001)


def sample_rate(samples: list, count: int, duration: float) -> float:
    if len(samples) >= 2:
        return (len(samples) - 1) / max(float(samples[-1][0]) - float(samples[0][0]), 0.001)
    return rate(count, duration)


class MotionGateProbe:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("navlab_p7_motion_gate_coordinator")
        self.started = time.monotonic()
        self.phase = "wait_ready"
        self.ready = False
        self.seq = 0
        self.counts = {{
            "fcu_pose": 0,
            "fcu_twist": 0,
            "slam_odom": 0,
            "truth_odom": 0,
            "scan": 0,
            "range": 0,
            "range_status": 0,
            "imu": 0,
            "slam_status": 0,
            "external_nav_status": 0,
            "controller_status": 0,
            "setpoint_intent": 0,
            "setpoint_output": 0,
            "owner_status": 0,
            "cmd_vel": 0,
            "hover_status": 0,
        }}
        self.latest = {{}}
        self.samples = {{"fcu_pose": [], "slam_odom": [], "truth_odom": []}}
        self.phase_samples = {{}}
        self.scan_min_ranges = []
        self.motion_pub = self.node.create_publisher(String, SPEC["motion_status_topic"], 10)
        self.intent_pub = self.node.create_publisher(String, SPEC["setpoint_intent_topic"], 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.node.create_subscription(PoseStamped, SPEC["fcu_pose_topic"], self._pose_cb, qos)
        self.node.create_subscription(TwistStamped, SPEC["fcu_twist_topic"], self._touch_cb("fcu_twist"), qos)
        self.node.create_subscription(TwistStamped, SPEC["cmd_vel_topic"], self._touch_cb("cmd_vel"), qos)
        self.node.create_subscription(Odometry, SPEC["slam_odom_topic"], self._odom_cb("slam_odom"), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, SPEC["truth_diagnostic_topic"], self._odom_cb("truth_odom"), qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, SPEC["scan_topic"], self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(Range, SPEC["rangefinder_range_topic"], self._range_cb, qos_profile_sensor_data)
        self.node.create_subscription(String, SPEC["rangefinder_status_topic"], self._string_cb("range_status"), 10)
        self.node.create_subscription(Imu, SPEC["imu_topic"], self._touch_cb("imu"), qos_profile_sensor_data)
        self.node.create_subscription(String, SPEC["slam_status_topic"], self._string_cb("slam_status"), 10)
        self.node.create_subscription(String, SPEC["external_nav_status_topic"], self._string_cb("external_nav_status"), 10)
        self.node.create_subscription(String, SPEC["controller_status_topic"], self._string_cb("controller_status"), 10)
        self.node.create_subscription(String, SPEC["setpoint_intent_topic"], self._string_cb("setpoint_intent"), 10)
        self.node.create_subscription(String, SPEC["setpoint_output_topic"], self._string_cb("setpoint_output"), 10)
        self.node.create_subscription(String, SPEC["owner_status_topic"], self._string_cb("owner_status"), 10)
        self.node.create_subscription(String, SPEC["hover_status_topic"], self._string_cb("hover_status"), 10)

    def _json_msg(self, payload: dict) -> String:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        return msg

    def _sample(self, key: str, sample: tuple[float, float, float, float]) -> None:
        stamped = (time.monotonic(), *sample)
        self.samples[key].append(stamped)
        self.phase_samples.setdefault(self.phase, {{"fcu_pose": [], "slam_odom": [], "truth_odom": []}})[key].append(stamped)

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.counts["fcu_pose"] += 1
        sample = point_from_pose(msg)
        self.latest["fcu_pose"] = {{"frame_id": msg.header.frame_id, "position": sample[:3], "yaw": sample[3], "monotonic": time.monotonic()}}
        self._sample("fcu_pose", sample)

    def _odom_cb(self, key: str):
        def callback(msg: Odometry) -> None:
            self.counts[key] += 1
            sample = point_from_odom(msg)
            self.latest[key] = {{"frame_id": msg.header.frame_id, "position": sample[:3], "yaw": sample[3], "monotonic": time.monotonic()}}
            self._sample(key, sample)
        return callback

    def _scan_cb(self, msg: LaserScan) -> None:
        self.counts["scan"] += 1
        ranges = [float(item) for item in msg.ranges if math.isfinite(float(item)) and float(item) > 0.0]
        min_range = min(ranges) if ranges else None
        if min_range is not None:
            self.scan_min_ranges.append(min_range)
        self.latest["scan"] = {{"frame_id": msg.header.frame_id, "min_range_m": min_range, "monotonic": time.monotonic()}}

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
            "input_topic": input_topic,
            "rate_hz": rate_hz,
            "latest_age_sec": self._latest_age("external_nav_status"),
            "uses_gazebo_truth_as_input": bool(SPEC["uses_gazebo_truth_as_input"]),
            "sample": payload,
        }}

    def _action_summary(self, phase: str, *, kind: str) -> dict:
        fcu = self.phase_samples.get(phase, {{}}).get("fcu_pose", [])
        if len(fcu) < 2:
            return {{"ok": False, "sample_count": len(fcu), "displacement_m": None, "signed_displacement_m": None, "direction_ok": False, "lateral_error_m": None, "yaw_delta_rad": None, "stop_drift_m": None}}
        start = fcu[0]
        end = fcu[-1]
        dx = end[1] - start[1]
        dy = end[2] - start[2]
        start_yaw = start[4]
        axis_displacement = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        lateral_error = abs(-dx * math.sin(start_yaw) + dy * math.cos(start_yaw))
        displacement = abs(axis_displacement) if kind in {{"forward", "back"}} else math.hypot(dx, dy)
        direction_ok = (axis_displacement >= 0.0) if kind == "forward" else (axis_displacement <= 0.0)
        if kind == "yaw":
            direction_ok = True
        yaw_delta = abs(angle_delta(end[4], start[4]))
        altitude_error = max(abs(abs(item[3]) - float(SPEC["takeoff_alt_m"])) for item in fcu)
        if kind == "yaw":
            lateral_error = displacement
        return {{
            "ok": True,
            "sample_count": len(fcu),
            "displacement_m": displacement,
            "signed_displacement_m": axis_displacement,
            "direction_ok": direction_ok,
            "lateral_error_m": lateral_error,
            "yaw_delta_rad": yaw_delta,
            "altitude_error_m": altitude_error,
            "start": start[1:],
            "end": end[1:],
        }}

    def _stop_summary(self, phase: str) -> dict:
        fcu = self.phase_samples.get(phase, {{}}).get("fcu_pose", [])
        if len(fcu) < 2:
            return {{"sample_count": len(fcu), "stop_drift_m": None}}
        xs = [item[1] for item in fcu]
        ys = [item[2] for item in fcu]
        return {{"sample_count": len(fcu), "stop_drift_m": math.hypot(max(xs) - min(xs), max(ys) - min(ys))}}

    def publish_status(self, final: bool = False) -> None:
        payload = {{
            "source": "navlab_p7_motion_gate_coordinator",
            "phase": self.phase,
            "final": final,
            "counts": self.counts,
            "ready": self.ready,
            "hover_claim": SPEC["hover_claim"],
            "motion_claim": SPEC["motion_claim"],
            "exploration_claim": SPEC["exploration_claim"],
            "updated_ms": now_ms(),
        }}
        self.motion_pub.publish(self._json_msg(payload))

    def publish_intent(self, *, linear_x: float = 0.0, angular_z: float = 0.0, kind: str = "hold") -> None:
        self.seq += 1
        intent = {{
            "source": "navlab_p7_motion_gate_coordinator",
            "target_owner": SPEC["owner_name"],
            "target_owner_id": SPEC["owner_id"],
            "sequence_id": self.seq,
            "kind": kind,
            "linear_x_mps": linear_x,
            "angular_z_radps": angular_z,
            "reason": "p7_motion_gate_intent",
            "updated_ms": now_ms(),
        }}
        self.intent_pub.publish(self._json_msg(intent))

    def run_phase(self, phase: str, duration: float, *, linear_x: float = 0.0, angular_z: float = 0.0, kind: str = "hold") -> None:
        self.phase = phase
        end = time.monotonic() + duration
        next_status = 0.0
        while time.monotonic() < end:
            self.publish_intent(linear_x=linear_x, angular_z=angular_z, kind=kind)
            spin_until = time.monotonic() + 0.05
            while time.monotonic() < spin_until:
                rclpy.spin_once(self.node, timeout_sec=0.005)
            if time.monotonic() >= next_status:
                self.publish_status(final=False)
                next_status = time.monotonic() + 0.5
            time.sleep(0.02)

    def wait_ready(self) -> bool:
        deadline = time.monotonic() + float(SPEC["ready_timeout_sec"])
        next_status = 0.0
        while time.monotonic() < deadline:
            spin_until = time.monotonic() + 0.05
            while time.monotonic() < spin_until:
                rclpy.spin_once(self.node, timeout_sec=0.005)
            if time.monotonic() >= next_status:
                self.publish_status(final=False)
                next_status = time.monotonic() + 0.5
            controller = self.latest.get("controller_status", {{}}).get("json") or {{}}
            controller_ready = bool(controller.get("ready"))
            if (
                self.counts["fcu_pose"] > 0
                and self.counts["slam_odom"] > 0
                and self.counts["range"] > 0
                and self.counts["imu"] > 0
                and controller_ready
            ):
                self.ready = True
                return True
            time.sleep(0.02)
        return False

    def summary(self) -> dict:
        elapsed = time.monotonic() - self.started
        fcu_rate = sample_rate(self.samples["fcu_pose"], self.counts["fcu_pose"], elapsed)
        slam_rate = sample_rate(self.samples["slam_odom"], self.counts["slam_odom"], elapsed)
        external_nav = self._external_nav()
        forward = self._action_summary("forward", kind="forward")
        back = self._action_summary("back", kind="back")
        yaw = self._action_summary("yaw_scan", kind="yaw")
        forward_stop = self._stop_summary("forward_stop")
        back_stop = self._stop_summary("back_stop")
        yaw_stop = self._stop_summary("yaw_stop")
        final_hold = self._stop_summary("final_hold")
        min_scan = min(self.scan_min_ranges) if self.scan_min_ranges else None
        blockers = []
        if fcu_rate < float(SPEC["min_fcu_local_position_rate_hz"]):
            blockers.append("P7 FCU local position rate is below minimum")
        if slam_rate < float(SPEC["min_slam_odom_rate_hz"]):
            blockers.append("P7 SLAM odom rate is below minimum")
        if self._latest_age("fcu_pose") is None or self._latest_age("fcu_pose") > float(SPEC["max_latest_age_sec"]):
            blockers.append("P7 FCU local position latest age is too high")
        if self._latest_age("slam_odom") is None or self._latest_age("slam_odom") > float(SPEC["max_latest_age_sec"]):
            blockers.append("P7 SLAM odom latest age is too high")
        if not external_nav["ok"]:
            blockers.append("P7 ExternalNav is not healthy")
        if forward["displacement_m"] is None or forward["displacement_m"] < float(SPEC["min_forward_displacement_m"]):
            blockers.append("P7 forward displacement below threshold")
        if not forward.get("direction_ok"):
            blockers.append("P7 forward displacement direction is inconsistent")
        if forward["displacement_m"] is not None and forward["displacement_m"] > float(SPEC["max_forward_displacement_m"]):
            blockers.append("P7 forward displacement exceeded threshold")
        if forward.get("lateral_error_m") is not None and forward["lateral_error_m"] > float(SPEC["max_lateral_error_m"]):
            blockers.append("P7 forward lateral error exceeded threshold")
        if back["displacement_m"] is None or back["displacement_m"] < float(SPEC["min_back_displacement_m"]):
            blockers.append("P7 back displacement below threshold")
        if not back.get("direction_ok"):
            blockers.append("P7 back displacement direction is inconsistent")
        if back["displacement_m"] is not None and back["displacement_m"] > float(SPEC["max_back_displacement_m"]):
            blockers.append("P7 back displacement exceeded threshold")
        if back.get("lateral_error_m") is not None and back["lateral_error_m"] > float(SPEC["max_lateral_error_m"]):
            blockers.append("P7 back lateral error exceeded threshold")
        if yaw["yaw_delta_rad"] is None or yaw["yaw_delta_rad"] < float(SPEC["min_yaw_delta_rad"]):
            blockers.append("P7 yaw scan delta below threshold")
        if yaw["yaw_delta_rad"] is not None and yaw["yaw_delta_rad"] > float(SPEC["max_yaw_delta_rad"]):
            blockers.append("P7 yaw scan delta exceeded threshold")
        for label, stop in (("forward", forward_stop), ("back", back_stop), ("yaw", yaw_stop), ("final", final_hold)):
            if stop["stop_drift_m"] is not None and stop["stop_drift_m"] > float(SPEC["max_stop_drift_m"]):
                blockers.append(f"P7 {{label}} stop drift exceeded threshold")
        if min_scan is None or min_scan < float(SPEC["min_clearance_m"]):
            blockers.append("P7 scan clearance below threshold")
        if SPEC["uses_gazebo_truth_as_input"]:
            blockers.append("P7 is configured to use Gazebo truth as input")
        forward_ok = not any("forward" in item for item in blockers)
        back_ok = not any("back" in item for item in blockers)
        yaw_ok = not any("yaw" in item for item in blockers)
        owner_payload = self.latest.get("owner_status", {{}}).get("json") or {{}}
        owner = {{
            "unique": bool(owner_payload.get("unique")),
            "owner": owner_payload.get("owner"),
            "owner_id": owner_payload.get("owner_id"),
            "set_pose_count": int(owner_payload.get("set_pose_count", -1)),
            "competing_publishers": owner_payload.get("competing_publishers", []),
        }}
        return {{
            "ok": not blockers,
            "blockers": blockers,
            "p7_motion_gate": {{
                "ok": not blockers,
                "hover_claim": SPEC["hover_claim"],
                "motion_claim": SPEC["motion_claim"],
                "exploration_claim": SPEC["exploration_claim"],
                "control_route": "unique_fcu_controller",
                "external_nav_input_topic": SPEC["slam_odom_topic"],
                "uses_gazebo_truth_as_input": bool(SPEC["uses_gazebo_truth_as_input"]),
            }},
            "p6_hover_prerequisite": {{"ok": self.ready, "source": "P7 waits for FCU/SLAM readiness after P4 bootstrap"}},
            "motion_actions": {{
                "forward": {{**forward, "ok": forward_ok, "stop_drift_m": forward_stop["stop_drift_m"]}},
                "back": {{**back, "ok": back_ok, "stop_drift_m": back_stop["stop_drift_m"]}},
                "yaw_scan": {{**yaw, "ok": yaw_ok, "stop_drift_m": yaw_stop["stop_drift_m"]}},
            }},
            "clearance": {{"ok": min_scan is not None and min_scan >= float(SPEC["min_clearance_m"]), "min_scan_range_m": min_scan, "min_clearance_m": float(SPEC["min_clearance_m"])}},
            "fcu": {{"local_position_ok": self.counts["fcu_pose"] > 0, "local_position_rate_hz": fcu_rate, "latest_age_sec": self._latest_age("fcu_pose")}},
            "slam_odom": {{"ok": self.counts["slam_odom"] > 0 and slam_rate >= float(SPEC["min_slam_odom_rate_hz"]), "topic": SPEC["slam_odom_topic"], "rate_hz": slam_rate, "latest_age_sec": self._latest_age("slam_odom")}},
            "external_nav": external_nav,
            "owner": owner,
            "counts": self.counts,
            "latest": self.latest,
        }}

    def run(self) -> dict:
        if not self.wait_ready():
            summary = {{"ok": False, "blockers": ["P7 readiness timeout"], "counts": self.counts}}
        else:
            self.run_phase("settle", float(SPEC["settle_window_sec"]), kind="hold")
            self.run_phase("forward", float(SPEC["forward_window_sec"]), linear_x=float(SPEC["motion_speed_mps"]), kind="forward")
            self.run_phase("forward_stop", float(SPEC["stop_hold_window_sec"]), kind="stop")
            self.run_phase("back", float(SPEC["back_window_sec"]), linear_x=-float(SPEC["motion_speed_mps"]), kind="back")
            self.run_phase("back_stop", float(SPEC["stop_hold_window_sec"]), kind="stop")
            self.run_phase("yaw_scan", float(SPEC["yaw_window_sec"]), angular_z=float(SPEC["yaw_rate_radps"]), kind="yaw_scan")
            self.run_phase("yaw_stop", float(SPEC["stop_hold_window_sec"]), kind="stop")
            self.run_phase("final_hold", float(SPEC["final_hold_window_sec"]), kind="final_hold")
            self.phase = "complete"
            summary = self.summary()
            for _ in range(5):
                self.publish_intent(kind="complete")
                rclpy.spin_once(self.node, timeout_sec=0.05)
                time.sleep(0.05)
        self.publish_status(final=True)
        Path(SPEC["summary_file"]).parent.mkdir(parents=True, exist_ok=True)
        Path(SPEC["summary_file"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
        print(json.dumps(summary, sort_keys=True))
        self.node.destroy_node()
        rclpy.shutdown()
        return summary


raise SystemExit(0 if MotionGateProbe().run()["ok"] else 30)
"""


def _write_motion_probe_script(config: RunConfig, script_path: Path) -> dict[str, Any]:
    p4 = config.orchestration.fcu_controller
    p7 = config.orchestration.motion_gate
    summary_file = config.artifact_dir / "motion_gate_summary.json"
    spec = {
        "summary_file": host._workspace_path(summary_file),
        "owner_name": p4.owner_name,
        "owner_id": p4.owner_id,
        "takeoff_alt_m": p4.takeoff_alt_m,
        "ready_timeout_sec": p4.readiness_timeout_sec,
        "slam_odom_topic": p7.slam_odom_topic,
        "slam_status_topic": p7.slam_status_topic,
        "external_nav_status_topic": p7.external_nav_status_topic,
        "fcu_pose_topic": p7.fcu_pose_topic,
        "fcu_twist_topic": p7.fcu_twist_topic,
        "fcu_status_topic": p7.fcu_status_topic,
        "cmd_vel_topic": p7.cmd_vel_topic,
        "rangefinder_range_topic": p7.rangefinder_range_topic,
        "rangefinder_status_topic": p7.rangefinder_status_topic,
        "imu_topic": p7.imu_topic,
        "scan_topic": p7.scan_topic,
        "truth_diagnostic_topic": p7.truth_diagnostic_topic,
        "controller_status_topic": p7.controller_status_topic,
        "setpoint_intent_topic": p7.setpoint_intent_topic,
        "setpoint_output_topic": p7.setpoint_output_topic,
        "owner_status_topic": p7.owner_status_topic,
        "hover_status_topic": p7.hover_status_topic,
        "motion_status_topic": p7.motion_status_topic,
        "settle_window_sec": p7.settle_window_sec,
        "forward_window_sec": p7.forward_window_sec,
        "back_window_sec": p7.back_window_sec,
        "yaw_window_sec": p7.yaw_window_sec,
        "stop_hold_window_sec": p7.stop_hold_window_sec,
        "final_hold_window_sec": p7.final_hold_window_sec,
        "motion_speed_mps": p7.motion_speed_mps,
        "yaw_rate_radps": p7.yaw_rate_radps,
        "min_forward_displacement_m": p7.min_forward_displacement_m,
        "max_forward_displacement_m": p7.max_forward_displacement_m,
        "min_back_displacement_m": p7.min_back_displacement_m,
        "max_back_displacement_m": p7.max_back_displacement_m,
        "min_yaw_delta_rad": p7.min_yaw_delta_rad,
        "max_yaw_delta_rad": p7.max_yaw_delta_rad,
        "max_lateral_error_m": p7.max_lateral_error_m,
        "max_motion_altitude_error_m": p7.max_motion_altitude_error_m,
        "max_stop_drift_m": p7.max_stop_drift_m,
        "min_clearance_m": p7.min_clearance_m,
        "min_slam_odom_rate_hz": p7.min_slam_odom_rate_hz,
        "min_external_nav_rate_hz": p7.min_external_nav_rate_hz,
        "min_fcu_local_position_rate_hz": p7.min_fcu_local_position_rate_hz,
        "max_latest_age_sec": p7.max_latest_age_sec,
        "uses_gazebo_truth_as_input": p7.uses_gazebo_truth_as_input,
        "hover_claim": p7.hover_claim,
        "motion_claim": p7.motion_claim,
        "exploration_claim": p7.exploration_claim,
    }
    script_path.parent.mkdir(parents=True, exist_ok=True)
    _write_text(script_path, _motion_probe_script(spec))
    return {
        "path": str(script_path),
        "workspace_path": host._workspace_path(script_path),
        "sha256": _file_sha256(script_path),
        "summary_file": str(summary_file),
        "spec": spec,
    }


def _run_motion_probe(config: RunConfig, *, script_path: Path) -> dict[str, Any]:
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.orchestration.official_baseline.runtime_image,
        shell_command=f"python3 {shlex.quote(host._workspace_path(script_path))}",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "motion_gate_probe.txt", output)
    summary = _load_json(config.artifact_dir / "motion_gate_summary.json")
    if not summary:
        summary = {"ok": False, "blockers": [f"motion probe failed rc={rc}"], "output": output}
    summary["rc"] = rc
    return summary


def _p7_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.motion_gate_rosbag_profile)
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


def _start_p7_rosbag_recording(config: RunConfig, *, duration_sec: float) -> None:
    _remove_container(P7_ROSBAG_CONTAINER)
    profile_path, required, optional, command = _p7_rosbag_shell_command(config, duration_sec=duration_sec)
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
        name=P7_ROSBAG_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={**_baseline_env(config), "PYTHONPATH": "/workspace"},
    )


def _finish_p7_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    profile_path = Path(config.motion_gate_rosbag_profile)
    required, optional, _topics = _profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    try:
        rc = DockerClient().wait(P7_ROSBAG_CONTAINER)
    except DockerException as exc:
        rc = exc.return_code or 1
    try:
        output = DockerClient().logs(P7_ROSBAG_CONTAINER, tail=2000)
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


def _build_p7_doctor_summary(
    config: RunConfig,
    *,
    runtime_config: Path,
    include_dependencies: bool = True,
) -> dict[str, Any]:
    p7 = config.orchestration.motion_gate
    p6_runtime = config.artifact_dir / "p7_doctor_p6_slam_hover_runtime.toml"
    p6_doctor = (
        _build_p6_doctor_summary(config, runtime_config=p6_runtime)
        if include_dependencies
        else {"ok": True, "blockers": [], "skipped": "acceptance already launched P6 prerequisites"}
    )
    profile_path = Path(p7.rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    blockers = [str(item) for item in p6_doctor.get("blockers", [])]
    if not profile_path.is_file() or not topics:
        blockers.append("P7 rosbag profile is missing or empty")
    if p7.uses_gazebo_truth_as_input:
        blockers.append("P7 must not use Gazebo truth as a control/planning/SLAM/ExternalNav input")
    if p7.slam_odom_topic != config.orchestration.slam_backend.slam_odom_topic:
        blockers.append("P7 SLAM odom topic must match P3 canonical SLAM odom topic")
    if p7.slam_odom_topic == p7.truth_diagnostic_topic:
        blockers.append("P7 SLAM odom topic must not be the Gazebo truth diagnostic topic")
    if p7.cmd_vel_topic != config.orchestration.fcu_controller.cmd_vel_topic:
        blockers.append("P7 cmd_vel topic must match the P4 FCU controller output topic")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "p7_motion_gate_doctor": {
            "runtime_config": str(runtime_config),
            "runtime_config_sha256": _file_sha256(runtime_config) if runtime_config.is_file() else "",
            "dependency_checks_included": include_dependencies,
            "slam_odom_topic": p7.slam_odom_topic,
            "external_nav_status_topic": p7.external_nav_status_topic,
            "motion_status_topic": p7.motion_status_topic,
            "uses_gazebo_truth_as_input": p7.uses_gazebo_truth_as_input,
            "hover_claim": p7.hover_claim,
            "motion_claim": p7.motion_claim,
            "exploration_claim": p7.exploration_claim,
            "thresholds": {
                "motion_distance_m": p7.motion_distance_m,
                "motion_speed_mps": p7.motion_speed_mps,
                "yaw_scan_rad": p7.yaw_scan_rad,
                "max_stop_drift_m": p7.max_stop_drift_m,
                "min_clearance_m": p7.min_clearance_m,
            },
            "rosbag_profile": {
                "profile": str(profile_path),
                "required_topics": required,
                "optional_topics": optional,
            },
        },
        "p6_slam_hover_doctor": p6_doctor,
    }


def _append_p7_blockers(
    *,
    blockers: list[str],
    motion_summary: dict[str, Any],
    rosbag_profile: dict[str, Any],
    counts: dict[str, int],
    p7: Any,
) -> None:
    if not motion_summary:
        blockers.append("P7 motion summary is missing")
        return
    if not motion_summary.get("ok"):
        blockers.extend(str(item) for item in motion_summary.get("blockers", []))
    if not motion_summary.get("motion_actions", {}).get("forward", {}).get("ok"):
        blockers.append("P7 forward motion gate did not pass")
    if not motion_summary.get("motion_actions", {}).get("back", {}).get("ok"):
        blockers.append("P7 back motion gate did not pass")
    if not motion_summary.get("motion_actions", {}).get("yaw_scan", {}).get("ok"):
        blockers.append("P7 yaw scan gate did not pass")
    if not motion_summary.get("clearance", {}).get("ok"):
        blockers.append("P7 clearance gate did not pass")
    if not motion_summary.get("slam_odom", {}).get("ok"):
        blockers.append("P7 SLAM odom gate did not pass")
    if not motion_summary.get("external_nav", {}).get("ok"):
        blockers.append("P7 ExternalNav gate did not pass")
    if not motion_summary.get("fcu", {}).get("local_position_ok"):
        blockers.append("P7 FCU local position gate did not pass")
    if p7.uses_gazebo_truth_as_input:
        blockers.append("P7 is configured to use Gazebo truth as input")
    if not rosbag_profile.get("ok"):
        blockers.append("P7 rosbag profile did not pass")
    for topic in (
        p7.controller_status_topic,
        p7.setpoint_intent_topic,
        p7.setpoint_output_topic,
        p7.owner_status_topic,
    ):
        if counts.get(topic, 0) <= 0:
            blockers.append(f"{topic} was not recorded")
    if counts.get(p7.motion_status_topic, 0) <= 0:
        blockers.append(f"{p7.motion_status_topic} was not recorded")


def _write_foxglove_notes(config: RunConfig) -> None:
    p7 = config.orchestration.motion_gate
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P7 motion gate replay notes",
                "",
                "P7 validates forward/back/yaw/stop motion after real SLAM hover. It is not an exploration gate.",
                "",
                "- Fixed frame: `map`.",
                f"- Motion status: `{p7.motion_status_topic}`.",
                f"- Setpoint output: `{p7.setpoint_output_topic}`.",
                f"- SLAM odom: `{p7.slam_odom_topic}`.",
                f"- FCU pose/twist: `{p7.fcu_pose_topic}`, `{p7.fcu_twist_topic}`.",
                f"- Diagnostic truth only: `{p7.truth_diagnostic_topic}`.",
                "- Do not use Gazebo truth as a SLAM, ExternalNav, planning, or control input.",
            ]
        )
        + "\n",
    )




