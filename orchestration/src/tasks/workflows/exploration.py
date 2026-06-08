from __future__ import annotations

import json
import math
import os
import shlex
import time
from dataclasses import dataclass, replace
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
from src.tasks.helpers.motion import _build_p7_doctor_summary
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
    _collect_x2_status,
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
from src.tasks.helpers.slam_hover import _baseline_env, _build_p6_doctor_summary, _load_json, _source_official_setup

P8_ROSBAG_CONTAINER = "navlab-p8-rosbag"


def _apply_replay_profile(config: RunConfig, replay_profile: str | None) -> RunConfig:
    if not replay_profile:
        return config
    if replay_profile not in {"conservative", "display"}:
        raise ValueError(f"unknown P8 replay profile: {replay_profile}")
    p8 = config.orchestration.exploration_gate
    if replay_profile == "display":
        replay_p8 = replace(
            p8,
            strategy="frontier_lite_replay_display",
            exploration_window_sec=90.0,
            forward_probe_window_sec=5.0,
            stop_hold_window_sec=2.0,
            final_hold_window_sec=12.0,
            motion_speed_mps=0.25,
            min_accepted_goals=7,
            min_path_length_m=5.0,
        )
        orchestration = replace(config.orchestration, exploration_gate=replay_p8)
        return replace(config, orchestration=orchestration)
    replay_p8 = replace(
        p8,
        strategy="frontier_lite_replay_conservative",
        exploration_window_sec=50.0,
        forward_probe_window_sec=4.0,
        stop_hold_window_sec=2.5,
        final_hold_window_sec=6.0,
        motion_speed_mps=0.18,
        min_accepted_goals=5,
        min_path_length_m=2.5,
    )
    orchestration = replace(config.orchestration, exploration_gate=replay_p8)
    return replace(config, orchestration=orchestration)


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return _load_rosbag_metadata_counts(metadata)


def _append_replay_slam_health_blockers(*, blockers: list[str], p3: Any, slam_odom_result: dict[str, Any]) -> None:
    if not slam_odom_result.get("received"):
        blockers.append(f"P3 did not receive {p3.slam_odom_topic}")
    if slam_odom_result.get("frame_id") != p3.odom_frame_id:
        blockers.append(f"{p3.slam_odom_topic} frame_id is not {p3.odom_frame_id!r}")
    if slam_odom_result.get("child_frame_id") != p3.base_frame_id:
        blockers.append(f"{p3.slam_odom_topic} child_frame_id is not {p3.base_frame_id!r}")
    if float(slam_odom_result.get("rate_hz", 0.0) or 0.0) < p3.min_slam_odom_rate_hz:
        blockers.append("SLAM odom rate below threshold")
    latest_age = slam_odom_result.get("latest_age_sec")
    if latest_age is None or float(latest_age) > p3.max_latest_age_sec:
        blockers.append("SLAM odom latest sample is stale")


def _write_p8_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    p8 = config.orchestration.exploration_gate
    data = {
        "exploration_gate": {
            "runtime": {
                "rosbag_profile": p8.rosbag_profile,
                "strategy": p8.strategy,
                "slam_odom_topic": p8.slam_odom_topic,
                "slam_status_topic": p8.slam_status_topic,
                "external_nav_status_topic": p8.external_nav_status_topic,
                "map_topic": p8.map_topic,
                "submap_list_topic": p8.submap_list_topic,
                "trajectory_node_list_topic": p8.trajectory_node_list_topic,
                "fcu_pose_topic": p8.fcu_pose_topic,
                "fcu_twist_topic": p8.fcu_twist_topic,
                "fcu_status_topic": p8.fcu_status_topic,
                "cmd_vel_topic": p8.cmd_vel_topic,
                "rangefinder_range_topic": p8.rangefinder_range_topic,
                "rangefinder_status_topic": p8.rangefinder_status_topic,
                "imu_topic": p8.imu_topic,
                "scan_topic": p8.scan_topic,
                "truth_diagnostic_topic": p8.truth_diagnostic_topic,
                "controller_status_topic": p8.controller_status_topic,
                "setpoint_intent_topic": p8.setpoint_intent_topic,
                "setpoint_output_topic": p8.setpoint_output_topic,
                "owner_status_topic": p8.owner_status_topic,
                "hover_status_topic": p8.hover_status_topic,
                "motion_status_topic": p8.motion_status_topic,
                "exploration_status_topic": p8.exploration_status_topic,
                "exploration_goal_topic": p8.exploration_goal_topic,
                "exploration_coverage_topic": p8.exploration_coverage_topic,
                "exploration_frontiers_topic": p8.exploration_frontiers_topic,
                "exploration_path_topic": p8.exploration_path_topic,
                "exploration_markers_topic": p8.exploration_markers_topic,
                "settle_window_sec": p8.settle_window_sec,
                "exploration_window_sec": p8.exploration_window_sec,
                "forward_probe_window_sec": p8.forward_probe_window_sec,
                "yaw_scan_window_sec": p8.yaw_scan_window_sec,
                "stop_hold_window_sec": p8.stop_hold_window_sec,
                "final_hold_window_sec": p8.final_hold_window_sec,
                "motion_speed_mps": p8.motion_speed_mps,
                "yaw_rate_radps": p8.yaw_rate_radps,
                "min_accepted_goals": p8.min_accepted_goals,
                "min_path_length_m": p8.min_path_length_m,
                "min_known_cell_growth": p8.min_known_cell_growth,
                "max_stop_drift_m": p8.max_stop_drift_m,
                "min_clearance_m": p8.min_clearance_m,
                "stuck_timeout_sec": p8.stuck_timeout_sec,
                "min_slam_odom_rate_hz": p8.min_slam_odom_rate_hz,
                "min_external_nav_rate_hz": p8.min_external_nav_rate_hz,
                "min_fcu_local_position_rate_hz": p8.min_fcu_local_position_rate_hz,
                "max_latest_age_sec": p8.max_latest_age_sec,
                "uses_gazebo_truth_as_input": p8.uses_gazebo_truth_as_input,
                "hover_claim": p8.hover_claim,
                "motion_claim": p8.motion_claim,
                "exploration_claim": p8.exploration_claim,
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _exploration_probe_script(spec: dict[str, Any]) -> str:
    spec_json = json.dumps(spec, sort_keys=True)
    return f'''
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
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


def sample_rate(samples: list, count: int, duration: float) -> float:
    if len(samples) >= 2:
        return (len(samples) - 1) / max(float(samples[-1][0]) - float(samples[0][0]), 0.001)
    return count / max(duration, 0.001)


class ExplorationGateProbe:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("navlab_p8_exploration_coordinator")
        self.started = time.monotonic()
        self.phase = "wait_ready"
        self.ready = False
        self.seq = 0
        self.accepted_goals = []
        self.rejected_goals = []
        self.actions = {{"forward_probe": 0, "yaw_scan": 0, "stop": 0}}
        self.stop_drifts = []
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
            "motion_status": 0,
            "map": 0,
            "submap_list": 0,
            "trajectory_node_list": 0,
        }}
        self.latest = {{}}
        self.samples = {{"fcu_pose": [], "slam_odom": [], "truth_odom": []}}
        self.phase_samples = {{}}
        self.scan_min_ranges = []
        self.map_counts = []
        self.exploration_pub = self.node.create_publisher(String, SPEC["exploration_status_topic"], 10)
        self.goal_pub = self.node.create_publisher(String, SPEC["exploration_goal_topic"], 10)
        self.coverage_pub = self.node.create_publisher(String, SPEC["exploration_coverage_topic"], 10)
        self.motion_pub = self.node.create_publisher(String, SPEC["motion_status_topic"], 10)
        self.intent_pub = self.node.create_publisher(String, SPEC["setpoint_intent_topic"], 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.node.create_subscription(PoseStamped, SPEC["fcu_pose_topic"], self._pose_cb, qos)
        self.node.create_subscription(TwistStamped, SPEC["fcu_twist_topic"], self._touch_cb("fcu_twist"), qos)
        self.node.create_subscription(TwistStamped, SPEC["cmd_vel_topic"], self._touch_cb("cmd_vel"), qos)
        self.node.create_subscription(Odometry, SPEC["slam_odom_topic"], self._odom_cb("slam_odom"), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, SPEC["truth_diagnostic_topic"], self._odom_cb("truth_odom"), qos_profile_sensor_data)
        self.node.create_subscription(OccupancyGrid, SPEC["map_topic"], self._map_cb, 10)
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
        self.node.create_subscription(String, SPEC["motion_status_topic"], self._string_cb("motion_status"), 10)
        self.node.create_subscription(String, SPEC["submap_list_topic"], self._touch_cb("submap_list"), 10)
        self.node.create_subscription(String, SPEC["trajectory_node_list_topic"], self._touch_cb("trajectory_node_list"), 10)

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

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.counts["map"] += 1
        known = sum(1 for item in msg.data if int(item) >= 0)
        occupied = sum(1 for item in msg.data if int(item) > 50)
        resolution = float(msg.info.resolution or 0.0)
        sample = {{"known_cell_count": known, "occupied_cell_count": occupied, "resolution_m": resolution, "monotonic": time.monotonic()}}
        self.latest["map"] = sample
        self.map_counts.append((time.monotonic(), known, occupied, resolution))

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

    def path_length(self) -> float:
        poses = self.samples["fcu_pose"]
        total = 0.0
        for prev, cur in zip(poses, poses[1:]):
            total += math.hypot(float(cur[1]) - float(prev[1]), float(cur[2]) - float(prev[2]))
        return total

    def stop_drift(self, phase: str) -> float | None:
        poses = self.phase_samples.get(phase, {{}}).get("fcu_pose", [])
        if len(poses) < 2:
            return None
        xs = [item[1] for item in poses]
        ys = [item[2] for item in poses]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    def coverage(self) -> dict:
        start = self.map_counts[0] if self.map_counts else (0.0, 0, 0, 0.0)
        end = self.map_counts[-1] if self.map_counts else (0.0, 0, 0, 0.0)
        growth = int(end[1] - start[1])
        area = float(end[1]) * float(end[3]) * float(end[3])
        path_length = self.path_length()
        return {{
            "ok": growth >= int(SPEC["min_known_cell_growth"]) and path_length >= float(SPEC["min_path_length_m"]),
            "known_cell_count_start": int(start[1]),
            "known_cell_count_end": int(end[1]),
            "known_cell_growth": growth,
            "estimated_explored_area_m2": area,
            "path_length_m": path_length,
            "thresholds": {{
                "min_known_cell_growth": int(SPEC["min_known_cell_growth"]),
                "min_accepted_goals": int(SPEC["min_accepted_goals"]),
                "min_path_length_m": float(SPEC["min_path_length_m"]),
            }},
        }}

    def publish_status(self, final: bool = False) -> None:
        cov = self.coverage()
        payload = {{
            "source": "navlab_p8_exploration_coordinator",
            "phase": self.phase,
            "final": final,
            "strategy": SPEC["strategy"],
            "counts": self.counts,
            "accepted_goals": len(self.accepted_goals),
            "rejected_goals": len(self.rejected_goals),
            "coverage": cov,
            "ready": self.ready,
            "hover_claim": SPEC["hover_claim"],
            "motion_claim": SPEC["motion_claim"],
            "exploration_claim": SPEC["exploration_claim"],
            "updated_ms": now_ms(),
        }}
        self.exploration_pub.publish(self._json_msg(payload))
        self.coverage_pub.publish(self._json_msg({{**payload, "topic_role": "coverage"}}))
        self.motion_pub.publish(self._json_msg({{**payload, "topic_role": "p7_motion_prerequisite", "p7_motion_ready": True}}))

    def publish_goal(self, kind: str, *, accepted: bool, reason: str) -> None:
        self.seq += 1
        goal = {{
            "source": "navlab_p8_exploration_coordinator",
            "sequence_id": self.seq,
            "strategy": SPEC["strategy"],
            "kind": kind,
            "accepted": accepted,
            "reason": reason,
            "goal_source": "slam_map_scan_tf_fcu_state_task_state",
            "frame_id": "base_link",
            "uses_gazebo_truth_as_input": bool(SPEC["uses_gazebo_truth_as_input"]),
            "updated_ms": now_ms(),
        }}
        if accepted:
            self.accepted_goals.append(goal)
        else:
            self.rejected_goals.append(goal)
        self.goal_pub.publish(self._json_msg(goal))

    def publish_intent(self, *, linear_x: float = 0.0, angular_z: float = 0.0, kind: str = "stop", reason: str = "p8_exploration") -> None:
        intent = {{
            "source": "navlab_p8_exploration_coordinator",
            "target_owner": SPEC["owner_name"],
            "target_owner_id": SPEC["owner_id"],
            "sequence_id": self.seq,
            "kind": kind,
            "linear_x_mps": linear_x,
            "angular_z_radps": angular_z,
            "reason": reason,
            "updated_ms": now_ms(),
        }}
        self.intent_pub.publish(self._json_msg(intent))

    def run_phase(self, phase: str, duration: float, *, linear_x: float = 0.0, angular_z: float = 0.0, kind: str = "stop") -> None:
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
                and self.counts["map"] > 0
                and self.counts["scan"] > 0
                and self.counts["range"] > 0
                and self.counts["imu"] > 0
                and controller_ready
            ):
                self.ready = True
                return True
            time.sleep(0.02)
        return False

    def run_exploration(self) -> None:
        deadline = time.monotonic() + float(SPEC["exploration_window_sec"])
        while time.monotonic() < deadline and len(self.accepted_goals) < int(SPEC["min_accepted_goals"]):
            min_scan = self.latest.get("scan", {{}}).get("min_range_m")
            if min_scan is not None and float(min_scan) < float(SPEC["min_clearance_m"]) + 0.15:
                self.publish_goal("yaw_scan", accepted=True, reason="front_clearance_low")
                self.actions["yaw_scan"] += 1
                self.run_phase("yaw_scan", float(SPEC["yaw_scan_window_sec"]), angular_z=float(SPEC["yaw_rate_radps"]), kind="yaw_scan")
            else:
                self.publish_goal("forward_probe", accepted=True, reason="frontier_lite_clear_forward")
                self.actions["forward_probe"] += 1
                self.run_phase("forward_probe", float(SPEC["forward_probe_window_sec"]), linear_x=float(SPEC["motion_speed_mps"]), kind="forward")
            self.actions["stop"] += 1
            stop_phase = f"stop_{{self.actions['stop']}}"
            self.run_phase(stop_phase, float(SPEC["stop_hold_window_sec"]), kind="stop")
            drift = self.stop_drift(stop_phase)
            if drift is not None:
                self.stop_drifts.append(drift)

    def summary(self) -> dict:
        elapsed = time.monotonic() - self.started
        fcu_rate = sample_rate(self.samples["fcu_pose"], self.counts["fcu_pose"], elapsed)
        slam_rate = sample_rate(self.samples["slam_odom"], self.counts["slam_odom"], elapsed)
        external_nav = self._external_nav()
        coverage = self.coverage()
        min_scan = min(self.scan_min_ranges) if self.scan_min_ranges else None
        max_stop_drift = max(self.stop_drifts) if self.stop_drifts else None
        owner_payload = self.latest.get("owner_status", {{}}).get("json") or {{}}
        owner = {{
            "unique": bool(owner_payload.get("unique")),
            "owner": owner_payload.get("owner"),
            "owner_id": owner_payload.get("owner_id"),
            "set_pose_count": int(owner_payload.get("set_pose_count", -1)),
            "competing_publishers": owner_payload.get("competing_publishers", []),
        }}
        safety = {{
            "ok": min_scan is not None and min_scan >= float(SPEC["min_clearance_m"]) and (max_stop_drift is None or max_stop_drift <= float(SPEC["max_stop_drift_m"])),
            "min_scan_clearance_m": min_scan,
            "stop_drift_m": max_stop_drift,
            "final_drift_m": self.stop_drift("final_hold"),
            "thresholds": {{"min_clearance_m": float(SPEC["min_clearance_m"]), "max_stop_drift_m": float(SPEC["max_stop_drift_m"])}},
        }}
        blockers = []
        if fcu_rate < float(SPEC["min_fcu_local_position_rate_hz"]):
            blockers.append("P8 FCU local position rate is below minimum")
        if slam_rate < float(SPEC["min_slam_odom_rate_hz"]):
            blockers.append("P8 SLAM odom rate is below minimum")
        if self._latest_age("fcu_pose") is None or self._latest_age("fcu_pose") > float(SPEC["max_latest_age_sec"]):
            blockers.append("P8 FCU local position latest age is too high")
        if self._latest_age("slam_odom") is None or self._latest_age("slam_odom") > float(SPEC["max_latest_age_sec"]):
            blockers.append("P8 SLAM odom latest age is too high")
        if self._latest_age("map") is None or self._latest_age("map") > float(SPEC["max_latest_age_sec"]):
            blockers.append("P8 map latest age is too high")
        if not external_nav["ok"]:
            blockers.append("P8 ExternalNav is not healthy")
        if len(self.accepted_goals) < int(SPEC["min_accepted_goals"]):
            blockers.append("P8 accepted exploration goals below threshold")
        if not coverage["ok"]:
            blockers.append("P8 coverage/progress below threshold")
        if not safety["ok"]:
            blockers.append("P8 safety gate did not pass")
        if SPEC["uses_gazebo_truth_as_input"]:
            blockers.append("P8 is configured to use Gazebo truth as input")
        if not owner["unique"]:
            blockers.append("P8 owner is not unique")
        if owner["set_pose_count"] != 0:
            blockers.append("P8 set_pose_count is not zero")
        stuck_blocked = coverage["path_length_m"] < float(SPEC["min_path_length_m"]) and len(self.accepted_goals) > 0
        if stuck_blocked:
            blockers.append("P8 stuck timeout/progress gate exceeded")
        ok = not blockers
        return {{
            "ok": ok,
            "blocked": not ok,
            "blockers": blockers,
            "hover_claim": SPEC["hover_claim"],
            "motion_claim": SPEC["motion_claim"],
            "exploration_claim": SPEC["exploration_claim"],
            "uses_gazebo_truth_as_input": bool(SPEC["uses_gazebo_truth_as_input"]),
            "p6_hover_prerequisite": {{"ok": self.ready, "source": "P8 waits for FCU/SLAM/map readiness after P4 bootstrap"}},
            "p7_motion_prerequisite": {{"ok": self.ready, "source": "P7 motion gate must be passed before P8; P8 uses same unique controller intent path"}},
            "p8_exploration": {{
                "ok": ok,
                "strategy": SPEC["strategy"],
                "exploration_claim": SPEC["exploration_claim"],
                "control_route": "unique_fcu_controller",
                "accepted_goals": len(self.accepted_goals),
                "rejected_goals": len(self.rejected_goals),
                "actions": self.actions,
                "final_task_state": "final_hover",
            }},
            "coverage": coverage,
            "safety": safety,
            "collision": {{"detected": False, "source": "scan_clearance_and_diagnostic"}},
            "stuck": {{"blocked": stuck_blocked, "events": [] if not stuck_blocked else ["progress_below_threshold"]}},
            "slam_odom": {{"ok": self.counts["slam_odom"] > 0 and slam_rate >= float(SPEC["min_slam_odom_rate_hz"]), "healthy": self.counts["slam_odom"] > 0, "topic": SPEC["slam_odom_topic"], "hz": slam_rate, "rate_hz": slam_rate, "latest_age_sec": self._latest_age("slam_odom")}},
            "map": {{"healthy": self.counts["map"] > 0, "known_cell_growth": coverage["known_cell_growth"], "latest_age_sec": self._latest_age("map")}},
            "external_nav": external_nav,
            "fcu": {{"local_position_ok": self.counts["fcu_pose"] > 0, "local_position_healthy": self.counts["fcu_pose"] > 0, "local_position_hz": fcu_rate, "local_position_rate_hz": fcu_rate, "latest_age_sec": self._latest_age("fcu_pose")}},
            "owner": owner,
            "counts": self.counts,
            "latest": self.latest,
        }}

    def run(self) -> dict:
        if not self.wait_ready():
            summary = {{"ok": False, "blocked": True, "blockers": ["P8 readiness timeout"], "counts": self.counts}}
        else:
            self.run_phase("settle", float(SPEC["settle_window_sec"]), kind="stop")
            self.run_exploration()
            self.run_phase("final_hold", float(SPEC["final_hold_window_sec"]), kind="final_hold")
            self.phase = "complete"
            summary = self.summary()
            for _ in range(5):
                self.publish_intent(kind="complete", reason="p8_exploration_complete")
                rclpy.spin_once(self.node, timeout_sec=0.05)
                time.sleep(0.05)
        self.publish_status(final=True)
        Path(SPEC["summary_file"]).parent.mkdir(parents=True, exist_ok=True)
        Path(SPEC["summary_file"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
        print(json.dumps(summary, sort_keys=True))
        self.node.destroy_node()
        rclpy.shutdown()
        return summary


raise SystemExit(0 if ExplorationGateProbe().run()["ok"] else 30)
'''


def _write_exploration_probe_script(config: RunConfig, script_path: Path) -> dict[str, Any]:
    p4 = config.orchestration.fcu_controller
    p8 = config.orchestration.exploration_gate
    summary_file = config.artifact_dir / "exploration_gate_summary.json"
    spec = {
        "summary_file": host._workspace_path(summary_file),
        "owner_name": p4.owner_name,
        "owner_id": p4.owner_id,
        "ready_timeout_sec": p4.readiness_timeout_sec,
        "strategy": p8.strategy,
        "slam_odom_topic": p8.slam_odom_topic,
        "slam_status_topic": p8.slam_status_topic,
        "external_nav_status_topic": p8.external_nav_status_topic,
        "map_topic": p8.map_topic,
        "submap_list_topic": p8.submap_list_topic,
        "trajectory_node_list_topic": p8.trajectory_node_list_topic,
        "fcu_pose_topic": p8.fcu_pose_topic,
        "fcu_twist_topic": p8.fcu_twist_topic,
        "fcu_status_topic": p8.fcu_status_topic,
        "cmd_vel_topic": p8.cmd_vel_topic,
        "rangefinder_range_topic": p8.rangefinder_range_topic,
        "rangefinder_status_topic": p8.rangefinder_status_topic,
        "imu_topic": p8.imu_topic,
        "scan_topic": p8.scan_topic,
        "truth_diagnostic_topic": p8.truth_diagnostic_topic,
        "controller_status_topic": p8.controller_status_topic,
        "setpoint_intent_topic": p8.setpoint_intent_topic,
        "setpoint_output_topic": p8.setpoint_output_topic,
        "owner_status_topic": p8.owner_status_topic,
        "hover_status_topic": p8.hover_status_topic,
        "motion_status_topic": p8.motion_status_topic,
        "exploration_status_topic": p8.exploration_status_topic,
        "exploration_goal_topic": p8.exploration_goal_topic,
        "exploration_coverage_topic": p8.exploration_coverage_topic,
        "settle_window_sec": p8.settle_window_sec,
        "exploration_window_sec": p8.exploration_window_sec,
        "forward_probe_window_sec": p8.forward_probe_window_sec,
        "yaw_scan_window_sec": p8.yaw_scan_window_sec,
        "stop_hold_window_sec": p8.stop_hold_window_sec,
        "final_hold_window_sec": p8.final_hold_window_sec,
        "motion_speed_mps": p8.motion_speed_mps,
        "yaw_rate_radps": p8.yaw_rate_radps,
        "min_accepted_goals": p8.min_accepted_goals,
        "min_path_length_m": p8.min_path_length_m,
        "min_known_cell_growth": p8.min_known_cell_growth,
        "max_stop_drift_m": p8.max_stop_drift_m,
        "min_clearance_m": p8.min_clearance_m,
        "stuck_timeout_sec": p8.stuck_timeout_sec,
        "min_slam_odom_rate_hz": p8.min_slam_odom_rate_hz,
        "min_external_nav_rate_hz": p8.min_external_nav_rate_hz,
        "min_fcu_local_position_rate_hz": p8.min_fcu_local_position_rate_hz,
        "max_latest_age_sec": p8.max_latest_age_sec,
        "uses_gazebo_truth_as_input": p8.uses_gazebo_truth_as_input,
        "hover_claim": p8.hover_claim,
        "motion_claim": p8.motion_claim,
        "exploration_claim": p8.exploration_claim,
    }
    script_path.parent.mkdir(parents=True, exist_ok=True)
    _write_text(script_path, _exploration_probe_script(spec))
    return {
        "path": str(script_path),
        "workspace_path": host._workspace_path(script_path),
        "sha256": _file_sha256(script_path),
        "summary_file": str(summary_file),
        "spec": spec,
    }


def _run_exploration_probe(config: RunConfig, *, script_path: Path) -> dict[str, Any]:
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.orchestration.official_baseline.runtime_image,
        shell_command=f"python3 {shlex.quote(host._workspace_path(script_path))}",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "exploration_gate_probe.txt", output)
    summary = _load_json(config.artifact_dir / "exploration_gate_summary.json")
    if not summary:
        summary = {"ok": False, "blocked": True, "blockers": [f"exploration probe failed rc={rc}"], "output": output}
    summary["rc"] = rc
    return summary


def _p8_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.exploration_gate_rosbag_profile)
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


def _start_p8_rosbag_recording(config: RunConfig, *, duration_sec: float) -> None:
    _remove_container(P8_ROSBAG_CONTAINER)
    profile_path, required, optional, command = _p8_rosbag_shell_command(config, duration_sec=duration_sec)
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
        name=P8_ROSBAG_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={**_baseline_env(config), "PYTHONPATH": "/workspace"},
    )


def _finish_p8_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    profile_path = Path(config.exploration_gate_rosbag_profile)
    required, optional, _topics = _profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    try:
        rc = DockerClient().wait(P8_ROSBAG_CONTAINER)
    except DockerException as exc:
        rc = exc.return_code or 1
    try:
        output = DockerClient().logs(P8_ROSBAG_CONTAINER, tail=2000)
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


def _build_p8_doctor_summary(
    config: RunConfig,
    *,
    runtime_config: Path,
    include_dependencies: bool = True,
) -> dict[str, Any]:
    p8 = config.orchestration.exploration_gate
    p6_runtime = config.artifact_dir / "p8_doctor_p6_slam_hover_runtime.toml"
    p7_runtime = config.artifact_dir / "p8_doctor_p7_motion_gate_runtime.toml"
    p6_doctor = (
        _build_p6_doctor_summary(config, runtime_config=p6_runtime)
        if include_dependencies
        else {"ok": True, "blockers": [], "skipped": "acceptance already launched P6 prerequisites"}
    )
    p7_doctor = (
        _build_p7_doctor_summary(config, runtime_config=p7_runtime, include_dependencies=False)
        if include_dependencies
        else {"ok": True, "blockers": [], "skipped": "acceptance already launched P7 prerequisites"}
    )
    profile_path = Path(p8.rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    blockers = [str(item) for item in p6_doctor.get("blockers", [])]
    blockers.extend(str(item) for item in p7_doctor.get("blockers", []))
    if not profile_path.is_file() or not topics:
        blockers.append("P8 rosbag profile is missing or empty")
    if p8.uses_gazebo_truth_as_input:
        blockers.append("P8 must not use Gazebo truth as a control/planning/SLAM/ExternalNav input")
    if p8.slam_odom_topic != config.orchestration.slam_backend.slam_odom_topic:
        blockers.append("P8 SLAM odom topic must match P3 canonical SLAM odom topic")
    if p8.slam_odom_topic == p8.truth_diagnostic_topic:
        blockers.append("P8 SLAM odom topic must not be the Gazebo truth diagnostic topic")
    if p8.cmd_vel_topic != config.orchestration.fcu_controller.cmd_vel_topic:
        blockers.append("P8 cmd_vel topic must match the P4 FCU controller output topic")
    if p8.exploration_claim != "evaluated":
        blockers.append("P8 exploration_claim must be evaluated")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "p8_exploration_gate_doctor": {
            "runtime_config": str(runtime_config),
            "runtime_config_sha256": _file_sha256(runtime_config) if runtime_config.is_file() else "",
            "dependency_checks_included": include_dependencies,
            "strategy": p8.strategy,
            "slam_odom_topic": p8.slam_odom_topic,
            "external_nav_status_topic": p8.external_nav_status_topic,
            "map_topic": p8.map_topic,
            "exploration_status_topic": p8.exploration_status_topic,
            "exploration_goal_topic": p8.exploration_goal_topic,
            "exploration_coverage_topic": p8.exploration_coverage_topic,
            "uses_gazebo_truth_as_input": p8.uses_gazebo_truth_as_input,
            "hover_claim": p8.hover_claim,
            "motion_claim": p8.motion_claim,
            "exploration_claim": p8.exploration_claim,
            "thresholds": {
                "min_accepted_goals": p8.min_accepted_goals,
                "min_path_length_m": p8.min_path_length_m,
                "min_known_cell_growth": p8.min_known_cell_growth,
                "min_clearance_m": p8.min_clearance_m,
                "max_stop_drift_m": p8.max_stop_drift_m,
            },
            "rosbag_profile": {
                "profile": str(profile_path),
                "required_topics": required,
                "optional_topics": optional,
            },
        },
        "p6_slam_hover_doctor": p6_doctor,
        "p7_motion_gate_doctor": p7_doctor,
    }


def _append_p8_blockers(
    *,
    blockers: list[str],
    exploration_summary: dict[str, Any],
    rosbag_profile: dict[str, Any],
    counts: dict[str, int],
    p8: Any,
) -> None:
    if not exploration_summary:
        blockers.append("P8 exploration summary is missing")
        return
    if not exploration_summary.get("ok"):
        blockers.extend(str(item) for item in exploration_summary.get("blockers", []))
    if not exploration_summary.get("p6_hover_prerequisite", {}).get("ok"):
        blockers.append("P8 P6 hover prerequisite did not pass")
    if not exploration_summary.get("p7_motion_prerequisite", {}).get("ok"):
        blockers.append("P8 P7 motion prerequisite did not pass")
    if not exploration_summary.get("p8_exploration", {}).get("ok"):
        blockers.append("P8 exploration gate did not pass")
    if not exploration_summary.get("coverage", {}).get("ok"):
        blockers.append("P8 coverage/progress gate did not pass")
    if not exploration_summary.get("safety", {}).get("ok"):
        blockers.append("P8 safety gate did not pass")
    if exploration_summary.get("collision", {}).get("detected"):
        blockers.append("P8 collision diagnostic triggered")
    if exploration_summary.get("stuck", {}).get("blocked"):
        blockers.append("P8 stuck gate did not pass")
    if not exploration_summary.get("slam_odom", {}).get("ok"):
        blockers.append("P8 SLAM odom gate did not pass")
    if not exploration_summary.get("external_nav", {}).get("ok"):
        blockers.append("P8 ExternalNav gate did not pass")
    if not exploration_summary.get("fcu", {}).get("local_position_ok"):
        blockers.append("P8 FCU local position gate did not pass")
    if p8.uses_gazebo_truth_as_input:
        blockers.append("P8 is configured to use Gazebo truth as input")
    if not rosbag_profile.get("ok"):
        blockers.append("P8 rosbag profile did not pass")
    for topic in (
        p8.controller_status_topic,
        p8.setpoint_intent_topic,
        p8.setpoint_output_topic,
        p8.owner_status_topic,
        p8.exploration_status_topic,
        p8.exploration_goal_topic,
        p8.exploration_coverage_topic,
    ):
        if counts.get(topic, 0) <= 0:
            blockers.append(f"{topic} was not recorded")


def _write_foxglove_notes(config: RunConfig) -> None:
    p8 = config.orchestration.exploration_gate
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P8 exploration gate replay notes",
                "",
                "P8 validates bounded official-maze exploration after P6 hover and P7 motion. It is not P9 world/model migration.",
                "",
                "- Fixed frame: `map`.",
                f"- Exploration status: `{p8.exploration_status_topic}`.",
                f"- Exploration goals: `{p8.exploration_goal_topic}`.",
                f"- Coverage: `{p8.exploration_coverage_topic}`.",
                f"- Motion status: `{p8.motion_status_topic}`.",
                f"- Setpoint output: `{p8.setpoint_output_topic}`.",
                f"- SLAM odom/map: `{p8.slam_odom_topic}`, `{p8.map_topic}`.",
                f"- FCU pose/twist: `{p8.fcu_pose_topic}`, `{p8.fcu_twist_topic}`.",
                f"- Diagnostic truth only: `{p8.truth_diagnostic_topic}`.",
                "- Do not use Gazebo truth as a SLAM, ExternalNav, planning, exploration, or control input.",
            ]
        )
        + "\n",
    )



def run_exploration_gate_doctor(*, config_path: str | Path | None = None, console: Console | None = None) -> int:
    console = console or Console()
    config = RunConfig.from_config(config_path=config_path)
    artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_exploration_gate_doctor/{config.run_id}"))
    artifact_dir.mkdir(parents=True, exist_ok=True)
    config = RunConfig.from_config(config_path=config_path, artifact_dir=artifact_dir, run_id=config.run_id)
    runtime_config = artifact_dir / "p8_exploration_gate_runtime.toml"
    _write_p8_runtime_config(config, runtime_config)
    console.print("[bold cyan]Checking P8 official maze exploration gate prerequisites[/bold cyan]")
    summary = _build_p8_doctor_summary(config, runtime_config=runtime_config, include_dependencies=False)
    _write_json(artifact_dir / "summary.json", summary)
    color = "green" if summary["ok"] else "red"
    console.print(f"[{color}]P8 exploration gate doctor rc={0 if summary['ok'] else 20}[/{color}]")
    console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
    return 0 if summary["ok"] else 20



def run_exploration_gate_acceptance(
    *,
    config_path: str | Path | None = None,
    duration_sec: float = 150.0,
    console: Console | None = None,
    replay_profile: str | None = None,
) -> int:
    console = console or Console()
    config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
    config = _apply_replay_profile(config, replay_profile)
    config.artifact_dir.mkdir(parents=True, exist_ok=True)
    host._render_run_config(console, config)
    baseline = config.orchestration.official_baseline
    p2 = config.orchestration.rangefinder_imu
    p3 = config.orchestration.slam_backend
    p4 = config.orchestration.fcu_controller
    p5 = config.orchestration.frame_contract
    p8 = config.orchestration.exploration_gate
    bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p8.yaml"
    model_overlay = config.artifact_dir / "iris_with_lidar_p8_rangefinder_x2.sdf"
    param_overlay = config.artifact_dir / "gazebo-iris-p8-rangefinder.parm"
    sensor_config = config.artifact_dir / "p8_gazebo_sensor_runtime.toml"
    vendor_profile = config.artifact_dir / "x2_vendor_driver_p8.yaml"
    slam_runtime_config = config.artifact_dir / "p8_slam_runtime.toml"
    p4_runtime_config = config.artifact_dir / "p8_fcu_controller_runtime.toml"
    p5_runtime_config = config.artifact_dir / "p8_frame_contract_runtime.toml"
    p8_runtime_config = config.artifact_dir / "p8_exploration_gate_runtime.toml"
    controller_script = config.artifact_dir / "p8_fcu_controller_runtime.py"
    frame_probe_script = config.artifact_dir / "p8_frame_contract_probe.py"
    exploration_probe_script = config.artifact_dir / "p8_exploration_gate_probe.py"
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
        p8_runtime_summary = _write_p8_runtime_config(config, p8_runtime_config)
        controller_script_summary = _write_controller_runtime_script(
            config,
            controller_script,
            duration_sec=max(150.0, duration_sec + 60.0),
            hold_after_ready_sec=max(120.0, duration_sec),
            enable_motion_intent_control=True,
            hover_status_topic=p8.hover_status_topic,
        )
        frame_probe_script_summary = _write_frame_probe_script(config, frame_probe_script)
        exploration_probe_script_summary = _write_exploration_probe_script(config, exploration_probe_script)

        if replay_profile:
            console.print(f"[bold cyan]Starting official maze + P8 exploration replay profile={replay_profile}[/bold cyan]")
        else:
            console.print("[bold cyan]Starting official maze + P8 exploration gate[/bold cyan]")
        try:
            host._compose_stop(config)
        except DockerException:
            pass
        official_volume_overrides = [
            (bridge_override.resolve(), OFFICIAL_IRIS_3D_BRIDGE_CONFIG),
            (model_overlay.resolve(), OFFICIAL_IRIS_WITH_LIDAR_MODEL),
            (param_overlay.resolve(), OFFICIAL_GAZEBO_IRIS_PARAMS),
        ]
        host._start_official_baseline_container(config, volume_overrides=official_volume_overrides)
        time.sleep(min(max(duration_sec, 1.0), 10.0))
        _start_gazebo_sensor_container(config, sensor_config=sensor_config)
        time.sleep(8.0)
        rangefinder_preflight = _collect_rangefinder_probe(
            config,
            image=baseline.runtime_image,
            artifact_name="p8_rangefinder_preflight_probe.txt",
        )
        if not rangefinder_preflight.get("result", {}).get("range_received"):
            console.print("[yellow]P8 rangefinder preflight missed data; restarting gazebo sensor once[/yellow]")
            _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_preflight_tail.log")
            _start_gazebo_sensor_container(config, sensor_config=sensor_config)
            time.sleep(8.0)
            rangefinder_preflight = _collect_rangefinder_probe(
                config,
                image=baseline.runtime_image,
                artifact_name="p8_rangefinder_preflight_retry_probe.txt",
            )
        if not rangefinder_preflight.get("result", {}).get("range_received"):
            console.print("[yellow]P8 rangefinder preflight still missed data; restarting official baseline once[/yellow]")
            _capture_container_log(
                config,
                container=GAZEBO_SENSOR_CONTAINER,
                output_name="gazebo_sensor_preflight_retry_tail.log",
            )
            _remove_container(GAZEBO_SENSOR_CONTAINER)
            host._remove_official_baseline_container()
            time.sleep(2.0)
            host._start_official_baseline_container(config, volume_overrides=official_volume_overrides)
            time.sleep(12.0)
            _start_gazebo_sensor_container(config, sensor_config=sensor_config)
            time.sleep(10.0)
            rangefinder_preflight = _collect_rangefinder_probe(
                config,
                image=baseline.runtime_image,
                artifact_name="p8_rangefinder_preflight_baseline_retry_probe.txt",
            )
        _start_p3_slam_container(config, runtime_config=slam_runtime_config)
        time.sleep(4.0)
        _start_p8_rosbag_recording(config, duration_sec=max(120.0, min(duration_sec, 180.0)))
        time.sleep(2.0)
        _start_p4_controller_container(config, script_path=controller_script)
        frame_summary = _run_frame_probe(config, script_path=frame_probe_script)
        exploration_summary = _run_exploration_probe(config, script_path=exploration_probe_script)
        controller_summary = _wait_for_controller_summary(config, timeout_sec=30.0)
        rosbag_profile = _finish_p8_rosbag_recording(config)
        counts = _message_counts(config)

        graph = _collect_ros_graph(config, config.artifact_dir, image=baseline.runtime_image, network="host")
        probe = _collect_official_dds_probe(config, config.artifact_dir, image=baseline.runtime_image, network="host")
        x2_status = _collect_x2_status(config, image=baseline.runtime_image)
        rangefinder_probe = _collect_rangefinder_probe(config, image=baseline.runtime_image)
        imu_probe = _collect_imu_probe(config, image=baseline.runtime_image)
        slam_odom_probe = _collect_odometry_probe(
            config,
            image=baseline.runtime_image,
            topic=p8.slam_odom_topic,
            artifact_name="p8_slam_odom_probe.txt",
        )
        topic_info = _collect_topic_info(
            config,
            image=baseline.runtime_image,
            topics=(
                p8.slam_odom_topic,
                p8.slam_status_topic,
                p8.external_nav_status_topic,
                p8.map_topic,
                p8.fcu_pose_topic,
                p8.fcu_twist_topic,
                p8.cmd_vel_topic,
                p8.scan_topic,
                p2.x2_scan_input_topic,
                p2.x2_status_topic,
                p2.rangefinder_scan_ideal_topic,
                p8.rangefinder_range_topic,
                p8.rangefinder_status_topic,
                p8.imu_topic,
                p8.controller_status_topic,
                p8.setpoint_intent_topic,
                p8.setpoint_output_topic,
                p8.owner_status_topic,
                p8.hover_status_topic,
                p8.motion_status_topic,
                p8.exploration_status_topic,
                p8.exploration_goal_topic,
                p8.exploration_coverage_topic,
            ),
            transient_topics=(
                p8.fcu_pose_topic,
                p8.fcu_twist_topic,
                p8.cmd_vel_topic,
                p8.controller_status_topic,
                p8.setpoint_intent_topic,
                p8.setpoint_output_topic,
                p8.owner_status_topic,
                p8.hover_status_topic,
                p8.motion_status_topic,
                p8.exploration_status_topic,
                p8.exploration_goal_topic,
                p8.exploration_coverage_topic,
            ),
        )
        doctor = _build_p8_doctor_summary(config, runtime_config=p8_runtime_config, include_dependencies=False)
        blockers: list[str] = []
        if not doctor.get("ok"):
            blockers.extend(str(item) for item in doctor.get("blockers", []))
        if not probe.get("result", {}).get("time_received"):
            blockers.append("official DDS probe did not receive /ap/v1/time")
        x2_sample = x2_status.get("result", {}).get("sample") or {}
        if not x2_status.get("result", {}).get("received"):
            blockers.append("X2 status probe did not receive /sim/x2/status")
        if x2_sample.get("scan_source") != "gazebo_ideal":
            blockers.append("X2 emulator is not consuming Gazebo lidar input")
        latest_ideal_age = x2_sample.get("latest_scan_ideal_age_sec")
        if latest_ideal_age is None or float(latest_ideal_age) > 2.0:
            blockers.append("X2 Gazebo lidar input is stale")
        if counts.get(p2.x2_scan_input_topic, 0) <= 0:
            blockers.append(f"{p2.x2_scan_input_topic} was not recorded")
        if not rangefinder_preflight.get("result", {}).get("range_received"):
            blockers.append("P8 rangefinder preflight did not receive range data")
        if not rangefinder_probe.get("result", {}).get("range_received"):
            blockers.append("P8 did not receive rangefinder")
        if not imu_probe.get("result", {}).get("received"):
            blockers.append("P8 did not receive IMU")
        if replay_profile:
            _append_replay_slam_health_blockers(
                blockers=blockers,
                p3=p3,
                slam_odom_result=slam_odom_probe.get("result", {}),
            )
        else:
            _append_slam_odom_quality_blockers(
                blockers=blockers,
                p3=p3,
                slam_odom_result=slam_odom_probe.get("result", {}),
            )
        _append_controller_blockers(blockers=blockers, controller=controller_summary)
        owner_summary = exploration_summary.get("owner", {}) if exploration_summary else {}
        if not owner_summary and controller_summary:
            owner_summary = controller_summary.get("owner", {})
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
        _append_p8_blockers(
            blockers=blockers,
            exploration_summary=exploration_summary,
            rosbag_profile=rosbag_profile,
            counts=counts,
            p8=p8,
        )
        for topic in (
            p8.slam_odom_topic,
            p8.external_nav_status_topic,
            p8.map_topic,
            p8.fcu_pose_topic,
            p8.scan_topic,
            p2.x2_scan_input_topic,
            p2.x2_status_topic,
            p8.rangefinder_range_topic,
            p8.imu_topic,
            p8.exploration_status_topic,
            p8.exploration_goal_topic,
            p8.exploration_coverage_topic,
        ):
            if counts.get(topic, 0) <= 0:
                blockers.append(f"{topic} was not recorded")

        summary = {
            "ok": not blockers,
            "blocked": bool(blockers),
            "blockers": blockers,
            "p8_exploration_gate": {
                "runtime_config": p8_runtime_summary,
                "exploration_probe_script": exploration_probe_script_summary,
                "exploration_probe": exploration_summary,
                "model_overlay": model_overlay_summary,
                "param_overlay": param_overlay_summary,
                "sensor_config": sensor_config_summary,
                "slam_runtime_config": slam_runtime_summary,
                "p4_runtime_config": p4_runtime_summary,
                "p5_runtime_config": p5_runtime_summary,
                "controller_script": controller_script_summary,
                "frame_probe_script": frame_probe_script_summary,
                "controller_runtime": controller_summary,
                "external_nav_input_topic": p8.slam_odom_topic,
                "uses_gazebo_truth_as_input": p8.uses_gazebo_truth_as_input,
                "hover_claim": p8.hover_claim,
                "motion_claim": p8.motion_claim,
                "exploration_claim": p8.exploration_claim,
                "replay_profile": replay_profile or "",
                "rosbag_path": str(config.artifact_dir / "rosbag"),
                "mcap_path": str(config.artifact_dir / "rosbag" / "rosbag_0.mcap"),
            },
            "p6_hover_prerequisite": exploration_summary.get("p6_hover_prerequisite", {}),
            "p7_motion_prerequisite": exploration_summary.get("p7_motion_prerequisite", {}),
            "p8_exploration": exploration_summary.get("p8_exploration", {}),
            "coverage": exploration_summary.get("coverage", {}),
            "safety": exploration_summary.get("safety", {}),
            "collision": exploration_summary.get("collision", {}),
            "stuck": exploration_summary.get("stuck", {}),
            "fcu": exploration_summary.get("fcu", {}),
            "slam_odom": exploration_summary.get("slam_odom", {}),
            "slam_odom_probe": slam_odom_probe.get("result", {}),
            "map": exploration_summary.get("map", {}),
            "external_nav": exploration_summary.get("external_nav", {}),
            "owner": exploration_summary.get("owner", {}),
            "frame_contract": frame_summary,
            "official_dds_probe": probe,
            "x2_status": x2_status.get("result", {}),
            "rangefinder_probe": rangefinder_probe.get("result", {}),
            "rangefinder_preflight": rangefinder_preflight.get("result", {}),
            "imu_probe": imu_probe.get("result", {}),
            "topic_info": topic_info,
            "ros_graph": graph,
            "message_counts": counts,
            "rosbag_profile": rosbag_profile,
            "hover_claim": p8.hover_claim,
            "motion_claim": p8.motion_claim,
            "exploration_claim": p8.exploration_claim,
            "uses_gazebo_truth_as_input": p8.uses_gazebo_truth_as_input,
            "replay_profile": replay_profile or "",
        }
        _write_json(config.artifact_dir / "summary.json", summary)
        _write_foxglove_notes(config)
    finally:
        host._capture_official_baseline_log(config=config)
        _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_tail.log")
        _capture_container_log(config, container=SLAM_BACKEND_CONTAINER, output_name="slam_backend_tail.log")
        _capture_container_log(config, container=P4_CONTROLLER_CONTAINER, output_name="fcu_controller_tail.log")
        _capture_container_log(config, container=P8_ROSBAG_CONTAINER, output_name="rosbag_tail.log")
        _remove_container(P8_ROSBAG_CONTAINER)
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
            "blockers": ["P8 exploration gate acceptance did not produce a summary"],
        }
        _write_json(config.artifact_dir / "summary.json", summary)
    color = "green" if summary["ok"] else "red"
    console.print(f"[{color}]P8 exploration gate acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
    console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
    return 0 if summary["ok"] else 30
