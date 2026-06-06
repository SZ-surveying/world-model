from __future__ import annotations

import json
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
    _start_p4_controller_container,
    _wait_for_controller_summary,
    _write_controller_runtime_script,
    _write_p4_runtime_config,
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
    _build_p3_doctor_summary,
    _start_p3_slam_container,
    _write_p3_slam_runtime_config,
)

P5_ROSBAG_CONTAINER = "navlab-p5-rosbag"


def _baseline_env(config: RunConfig) -> dict[str, str]:
    baseline = config.orchestration.official_baseline
    return {
        "DDS_ENABLE": baseline.dds_enable,
        "DDS_DOMAIN_ID": baseline.dds_domain_id,
        "ROS_DOMAIN_ID": baseline.dds_domain_id,
        "RMW_IMPLEMENTATION": baseline.rmw_implementation,
    }


def _write_p5_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    p5 = config.orchestration.frame_contract
    data = {
        "frame_contract": {
            "runtime": {
                "required_frames": list(p5.required_frames),
                "map_frame_id": p5.map_frame_id,
                "odom_frame_id": p5.odom_frame_id,
                "base_frame_id": p5.base_frame_id,
                "imu_frame_id": p5.imu_frame_id,
                "laser_frame_id": p5.laser_frame_id,
                "rangefinder_frame_id": p5.rangefinder_frame_id,
                "scan_topic": p5.scan_topic,
                "imu_topic": p5.imu_topic,
                "rangefinder_range_topic": p5.rangefinder_range_topic,
                "rangefinder_status_topic": p5.rangefinder_status_topic,
                "fcu_pose_topic": p5.fcu_pose_topic,
                "fcu_twist_topic": p5.fcu_twist_topic,
                "fcu_status_topic": p5.fcu_status_topic,
                "cmd_vel_topic": p5.cmd_vel_topic,
                "slam_odom_topic": p5.slam_odom_topic,
                "slam_status_topic": p5.slam_status_topic,
                "truth_diagnostic_topic": p5.truth_diagnostic_topic,
                "controller_status_topic": p5.controller_status_topic,
                "setpoint_output_topic": p5.setpoint_output_topic,
                "owner_status_topic": p5.owner_status_topic,
                "status_topic": p5.status_topic,
                "max_dynamic_tf_age_sec": p5.max_dynamic_tf_age_sec,
                "min_scan_valid_ratio": p5.min_scan_valid_ratio,
                "max_rangefinder_height_error_m": p5.max_rangefinder_height_error_m,
                "max_direction_error_rad": p5.max_direction_error_rad,
                "probe_duration_sec": p5.probe_duration_sec,
                "require_motion_direction_check": p5.require_motion_direction_check,
                "hover_claim": p5.hover_claim,
                "exploration_claim": p5.exploration_claim,
                "uses_gazebo_truth_as_input": p5.uses_gazebo_truth_as_input,
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _frame_probe_script(spec: dict[str, Any]) -> str:
    spec_json = json.dumps(spec, sort_keys=True)
    return f"""
from __future__ import annotations

import json
import math
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

SPEC = json.loads({spec_json!r})


def now_ms() -> int:
    return int(time.time() * 1000)


def quat_norm(q) -> float:
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)


def point_from_pose(msg) -> tuple[float, float, float]:
    p = msg.pose.position if hasattr(msg, "pose") and hasattr(msg.pose, "position") else msg.pose.pose.position
    return float(p.x), float(p.y), float(p.z)


def finite(value: float) -> bool:
    return math.isfinite(value)


class FrameContractProbe:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("navlab_frame_contract_probe")
        self.started = time.monotonic()
        self.frames = set()
        self.parents = defaultdict(set)
        self.edges = set()
        self.static_edges = set()
        self.dynamic_edges = set()
        self.dynamic_seen_monotonic = []
        self.parent_conflicts = []
        self.quaternion_norm_errors = []
        self.counts = {{
            "tf": 0,
            "tf_static": 0,
            "scan": 0,
            "imu": 0,
            "range": 0,
            "range_status": 0,
            "fcu_pose": 0,
            "fcu_twist": 0,
            "slam_odom": 0,
            "truth_odom": 0,
            "slam_status": 0,
            "controller_status": 0,
            "setpoint_output": 0,
            "owner_status": 0,
        }}
        self.latest = {{}}
        self.first_points = {{}}
        self.latest_points = {{}}
        self.status_pub = self.node.create_publisher(String, SPEC["status_topic"], 10)
        self.tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.static_tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.fcu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.node.create_subscription(TFMessage, "/tf", self._tf_cb, self.tf_qos)
        self.node.create_subscription(TFMessage, "/tf_static", self._tf_static_cb, self.static_tf_qos)
        self.node.create_subscription(LaserScan, SPEC["scan_topic"], self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(Imu, SPEC["imu_topic"], self._imu_cb, qos_profile_sensor_data)
        self.node.create_subscription(Range, SPEC["rangefinder_range_topic"], self._range_cb, qos_profile_sensor_data)
        self.node.create_subscription(String, SPEC["rangefinder_status_topic"], self._string_cb("range_status"), 10)
        self.node.create_subscription(PoseStamped, SPEC["fcu_pose_topic"], self._pose_cb("fcu_pose"), self.fcu_qos)
        self.node.create_subscription(TwistStamped, SPEC["fcu_twist_topic"], self._twist_cb, self.fcu_qos)
        self.node.create_subscription(Odometry, SPEC["slam_odom_topic"], self._odom_cb("slam_odom"), qos_profile_sensor_data)
        self.node.create_subscription(
            Odometry,
            SPEC["truth_diagnostic_topic"],
            self._odom_cb("truth_odom"),
            qos_profile_sensor_data,
        )
        self.node.create_subscription(String, SPEC["slam_status_topic"], self._string_cb("slam_status"), 10)
        self.node.create_subscription(String, SPEC["controller_status_topic"], self._string_cb("controller_status"), 10)
        self.node.create_subscription(String, SPEC["setpoint_output_topic"], self._string_cb("setpoint_output"), 10)
        self.node.create_subscription(String, SPEC["owner_status_topic"], self._string_cb("owner_status"), 10)

    def _tf_cb(self, msg: TFMessage) -> None:
        self.counts["tf"] += len(msg.transforms)
        self.dynamic_seen_monotonic.append(time.monotonic())
        self._handle_tf(msg, is_static=False)

    def _tf_static_cb(self, msg: TFMessage) -> None:
        self.counts["tf_static"] += len(msg.transforms)
        self._handle_tf(msg, is_static=True)

    def _handle_tf(self, msg: TFMessage, *, is_static: bool) -> None:
        for transform in msg.transforms:
            parent = transform.header.frame_id.strip()
            child = transform.child_frame_id.strip()
            if not parent or not child:
                continue
            self.frames.add(parent)
            self.frames.add(child)
            self.parents[child].add(parent)
            edge = (parent, child)
            self.edges.add(edge)
            if is_static:
                self.static_edges.add(edge)
            else:
                self.dynamic_edges.add(edge)
            norm = quat_norm(transform.transform.rotation)
            if abs(norm - 1.0) > 0.05:
                self.quaternion_norm_errors.append({{"parent": parent, "child": child, "norm": norm}})

    def _scan_cb(self, msg: LaserScan) -> None:
        self.counts["scan"] += 1
        finite_ranges = [float(value) for value in msg.ranges if finite(float(value))]
        valid = [
            value
            for value in finite_ranges
            if float(msg.range_min) <= value <= float(msg.range_max)
        ]
        front_range = None
        front_index = None
        if msg.ranges and msg.angle_increment:
            front_index = int(round((0.0 - float(msg.angle_min)) / float(msg.angle_increment)))
            if 0 <= front_index < len(msg.ranges):
                raw = float(msg.ranges[front_index])
                front_range = raw if finite(raw) else None
        self.frames.add(msg.header.frame_id)
        self.latest["scan"] = {{
            "frame_id": msg.header.frame_id,
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "range_count": len(msg.ranges),
            "valid_range_count": len(valid),
            "valid_range_ratio": len(valid) / max(len(msg.ranges), 1),
            "front_index": front_index,
            "front_range": front_range,
            "front_angle_available": float(msg.angle_min) <= 0.0 <= float(msg.angle_max),
        }}

    def _imu_cb(self, msg: Imu) -> None:
        self.counts["imu"] += 1
        self.frames.add(msg.header.frame_id)
        self.latest["imu"] = {{"frame_id": msg.header.frame_id, "monotonic": time.monotonic()}}

    def _range_cb(self, msg: Range) -> None:
        self.counts["range"] += 1
        self.frames.add(msg.header.frame_id)
        self.latest["range"] = {{
            "frame_id": msg.header.frame_id,
            "range": float(msg.range),
            "radiation_type": int(msg.radiation_type),
            "monotonic": time.monotonic(),
        }}

    def _pose_cb(self, key: str):
        def callback(msg: PoseStamped) -> None:
            self.counts[key] += 1
            self.frames.add(msg.header.frame_id)
            point = point_from_pose(msg)
            self.first_points.setdefault(key, point)
            self.latest_points[key] = point
            self.latest[key] = {{"frame_id": msg.header.frame_id, "position": point, "monotonic": time.monotonic()}}
        return callback

    def _odom_cb(self, key: str):
        def callback(msg: Odometry) -> None:
            self.counts[key] += 1
            self.frames.add(msg.header.frame_id)
            self.frames.add(msg.child_frame_id)
            point = point_from_pose(msg)
            self.first_points.setdefault(key, point)
            self.latest_points[key] = point
            self.latest[key] = {{
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "position": point,
                "monotonic": time.monotonic(),
            }}
        return callback

    def _twist_cb(self, msg: TwistStamped) -> None:
        self.counts["fcu_twist"] += 1
        self.frames.add(msg.header.frame_id)
        self.latest["fcu_twist"] = {{"frame_id": msg.header.frame_id, "monotonic": time.monotonic()}}

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

    def has_path(self, source: str, target: str) -> bool:
        graph = defaultdict(set)
        for parent, child in self.edges:
            graph[parent].add(child)
        seen = set()
        stack = [source]
        while stack:
            node = stack.pop()
            if node == target:
                return True
            if node in seen:
                continue
            seen.add(node)
            stack.extend(graph[node] - seen)
        return False

    def cycles(self) -> list[list[str]]:
        graph = defaultdict(set)
        for parent, child in self.edges:
            graph[parent].add(child)
        cycles = []
        visiting = set()
        visited = set()

        def visit(node: str, path: list[str]) -> None:
            if node in visiting:
                if node in path:
                    cycles.append(path[path.index(node):] + [node])
                return
            if node in visited:
                return
            visiting.add(node)
            for child in graph[node]:
                visit(child, [*path, child])
            visiting.remove(node)
            visited.add(node)

        for node in list(graph):
            visit(node, [node])
        return cycles

    def displacement(self, key: str) -> tuple[float, float, float] | None:
        first = self.first_points.get(key)
        latest = self.latest_points.get(key)
        if not first or not latest:
            return None
        return latest[0] - first[0], latest[1] - first[1], latest[2] - first[2]

    def direction_match(self, a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> bool | None:
        if a is None or b is None:
            return None
        a_norm = math.hypot(a[0], a[1])
        b_norm = math.hypot(b[0], b[1])
        if a_norm < 0.03 or b_norm < 0.03:
            return None
        dot = a[0] * b[0] + a[1] * b[1]
        cos_angle = max(-1.0, min(1.0, dot / (a_norm * b_norm)))
        return math.acos(cos_angle) <= float(SPEC["max_direction_error_rad"])

    def summary(self) -> dict:
        required_frames = list(SPEC["required_frames"])
        missing_frames = [frame for frame in required_frames if frame not in self.frames]
        parent_conflicts = [
            {{"child": child, "parents": sorted(parents)}}
            for child, parents in self.parents.items()
            if len(parents) > 1
        ]
        cycles = self.cycles()
        expected_edges = {{
            "map_to_odom": (SPEC["map_frame_id"], SPEC["odom_frame_id"]),
            "odom_to_base": (SPEC["odom_frame_id"], SPEC["base_frame_id"]),
            "base_to_imu": (SPEC["base_frame_id"], SPEC["imu_frame_id"]),
            "base_to_laser": (SPEC["base_frame_id"], SPEC["laser_frame_id"]),
            "base_to_rangefinder": (SPEC["base_frame_id"], SPEC["rangefinder_frame_id"]),
        }}
        edge_results = {{
            name: {{
                "edge": [parent, child],
                "direct": (parent, child) in self.edges,
                "connected": self.has_path(parent, child),
            }}
            for name, (parent, child) in expected_edges.items()
        }}
        latest_dynamic_age = None
        if self.dynamic_seen_monotonic:
            latest_dynamic_age = time.monotonic() - max(self.dynamic_seen_monotonic)
        tf_ok = (
            not missing_frames
            and not parent_conflicts
            and not cycles
            and not self.quaternion_norm_errors
            and all(item["connected"] for item in edge_results.values())
            and latest_dynamic_age is not None
            and latest_dynamic_age <= float(SPEC["max_dynamic_tf_age_sec"])
        )
        scan = self.latest.get("scan", {{}})
        scan_ok = (
            self.counts["scan"] > 0
            and scan.get("frame_id") == SPEC["laser_frame_id"]
            and scan.get("front_angle_available") is True
            and scan.get("valid_range_ratio", 0.0) >= float(SPEC["min_scan_valid_ratio"])
            and scan.get("angle_min", 0.0) < scan.get("angle_max", 0.0)
            and scan.get("angle_increment", 0.0) > 0.0
        )
        imu = self.latest.get("imu", {{}})
        imu_ok = self.counts["imu"] > 0 and imu.get("frame_id") == SPEC["imu_frame_id"]
        range_sample = self.latest.get("range", {{}})
        range_height_error = None
        fcu_pose = self.latest.get("fcu_pose", {{}})
        if range_sample.get("range") is not None and fcu_pose.get("position"):
            range_height_error = abs(float(range_sample["range"]) - abs(float(fcu_pose["position"][2])))
        range_ok = (
            self.counts["range"] > 0
            and range_sample.get("frame_id") == SPEC["rangefinder_frame_id"]
            and (range_height_error is None or range_height_error <= float(SPEC["max_rangefinder_height_error_m"]))
        )
        fcu_disp = self.displacement("fcu_pose")
        slam_disp = self.displacement("slam_odom")
        truth_disp = self.displacement("truth_odom")
        fcu_vs_slam = self.direction_match(fcu_disp, slam_disp)
        fcu_vs_truth = self.direction_match(fcu_disp, truth_disp)
        motion_required = bool(SPEC["require_motion_direction_check"])
        direction_ok = True
        if motion_required:
            direction_ok = fcu_vs_slam is True and fcu_vs_truth is True
        blockers = []
        if not tf_ok:
            if missing_frames:
                blockers.append("required frames are missing")
            if parent_conflicts:
                blockers.append("TF child frame has multiple parents")
            if cycles:
                blockers.append("TF graph has a cycle")
            if self.quaternion_norm_errors:
                blockers.append("TF quaternion norm is invalid")
            for name, item in edge_results.items():
                if not item["connected"]:
                    blockers.append(f"TF missing {{name}}")
            if latest_dynamic_age is None or latest_dynamic_age > float(SPEC["max_dynamic_tf_age_sec"]):
                blockers.append("dynamic TF latest age is too high")
        if not scan_ok:
            blockers.append("scan frame or forward direction contract failed")
        if not imu_ok:
            blockers.append("IMU frame contract failed")
        if not range_ok:
            blockers.append("rangefinder frame or height contract failed")
        if motion_required and not direction_ok:
            blockers.append("motion direction consistency failed")
        if SPEC["uses_gazebo_truth_as_input"]:
            blockers.append("Gazebo truth is configured as an input")
        return {{
            "ok": not blockers,
            "blockers": blockers,
            "claim": "frame_contract_evaluated",
            "counts": self.counts,
            "tf": {{
                "ok": tf_ok,
                "required_frames": required_frames,
                "frames_seen": sorted(self.frames),
                "missing_frames": missing_frames,
                "parent_conflicts": parent_conflicts,
                "cycles": cycles,
                "edge_results": edge_results,
                "latest_dynamic_age_sec": latest_dynamic_age,
                "quaternion_norm_errors": self.quaternion_norm_errors,
            }},
            "scan": {{
                "ok": scan_ok,
                "topic": SPEC["scan_topic"],
                **scan,
                "forward_matches_base_link_x": scan_ok,
            }},
            "imu": {{"ok": imu_ok, **imu}},
            "rangefinder": {{
                "ok": range_ok,
                **range_sample,
                "height_error_m": range_height_error,
            }},
            "direction_consistency": {{
                "ok": direction_ok,
                "motion_required": motion_required,
                "direction_motion_claim": "evaluated" if motion_required else "not_evaluated",
                "fcu_displacement": fcu_disp,
                "slam_displacement": slam_disp,
                "truth_displacement": truth_disp,
                "fcu_vs_slam_direction_match": fcu_vs_slam,
                "fcu_vs_truth_direction_match": fcu_vs_truth,
                "yaw_direction_match": None,
            }},
            "latest": self.latest,
            "claims": {{
                "hover_claim": SPEC["hover_claim"],
                "exploration_claim": SPEC["exploration_claim"],
                "gazebo_truth_control_claim": False,
                "gazebo_truth_input_claim": bool(SPEC["uses_gazebo_truth_as_input"]),
            }},
        }}

    def publish_status(self, final: bool = False) -> dict:
        payload = self.summary()
        payload["source"] = "navlab_frame_contract_probe"
        payload["final"] = final
        payload["updated_ms"] = now_ms()
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)
        return payload

    def run(self) -> dict:
        deadline = time.monotonic() + float(SPEC["probe_duration_sec"])
        next_status = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if time.monotonic() >= next_status:
                self.publish_status(final=False)
                next_status = time.monotonic() + 0.5
        summary = self.publish_status(final=True)
        Path(SPEC["summary_file"]).parent.mkdir(parents=True, exist_ok=True)
        Path(SPEC["summary_file"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
        print(json.dumps(summary, sort_keys=True))
        self.node.destroy_node()
        rclpy.shutdown()
        return summary


raise SystemExit(0 if FrameContractProbe().run()["ok"] else 30)
"""


def _write_frame_probe_script(config: RunConfig, script_path: Path) -> dict[str, Any]:
    p5 = config.orchestration.frame_contract
    summary_file = config.artifact_dir / "frame_contract_summary.json"
    spec = {
        "summary_file": host._workspace_path(summary_file),
        "required_frames": list(p5.required_frames),
        "map_frame_id": p5.map_frame_id,
        "odom_frame_id": p5.odom_frame_id,
        "base_frame_id": p5.base_frame_id,
        "imu_frame_id": p5.imu_frame_id,
        "laser_frame_id": p5.laser_frame_id,
        "rangefinder_frame_id": p5.rangefinder_frame_id,
        "scan_topic": p5.scan_topic,
        "imu_topic": p5.imu_topic,
        "rangefinder_range_topic": p5.rangefinder_range_topic,
        "rangefinder_status_topic": p5.rangefinder_status_topic,
        "fcu_pose_topic": p5.fcu_pose_topic,
        "fcu_twist_topic": p5.fcu_twist_topic,
        "fcu_status_topic": p5.fcu_status_topic,
        "cmd_vel_topic": p5.cmd_vel_topic,
        "slam_odom_topic": p5.slam_odom_topic,
        "slam_status_topic": p5.slam_status_topic,
        "truth_diagnostic_topic": p5.truth_diagnostic_topic,
        "controller_status_topic": p5.controller_status_topic,
        "setpoint_output_topic": p5.setpoint_output_topic,
        "owner_status_topic": p5.owner_status_topic,
        "status_topic": p5.status_topic,
        "max_dynamic_tf_age_sec": p5.max_dynamic_tf_age_sec,
        "min_scan_valid_ratio": p5.min_scan_valid_ratio,
        "max_rangefinder_height_error_m": p5.max_rangefinder_height_error_m,
        "max_direction_error_rad": p5.max_direction_error_rad,
        "probe_duration_sec": p5.probe_duration_sec,
        "require_motion_direction_check": p5.require_motion_direction_check,
        "hover_claim": p5.hover_claim,
        "exploration_claim": p5.exploration_claim,
        "uses_gazebo_truth_as_input": p5.uses_gazebo_truth_as_input,
    }
    script_path.parent.mkdir(parents=True, exist_ok=True)
    _write_text(script_path, _frame_probe_script(spec))
    return {
        "path": str(script_path),
        "workspace_path": host._workspace_path(script_path),
        "sha256": _file_sha256(script_path),
        "summary_file": str(summary_file),
        "spec": spec,
    }


def _run_frame_probe(config: RunConfig, *, script_path: Path) -> dict[str, Any]:
    command = f"python3 {shlex.quote(host._workspace_path(script_path))}"
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.orchestration.official_baseline.runtime_image,
        shell_command=command,
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "frame_contract_probe.txt", output)
    summary = _load_json(config.artifact_dir / "frame_contract_summary.json")
    if not summary:
        summary = {"ok": False, "blockers": [f"frame probe failed rc={rc}"], "output": output}
    summary["rc"] = rc
    return summary


def _p5_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.frame_contract_rosbag_profile)
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


def _start_p5_rosbag_recording(config: RunConfig, *, duration_sec: float) -> None:
    _remove_container(P5_ROSBAG_CONTAINER)
    profile_path, required, optional, command = _p5_rosbag_shell_command(config, duration_sec=duration_sec)
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
        [
            "bash",
            "-lc",
            (
                "source /opt/ros/jazzy/setup.bash && "
                "source /opt/navlab_official_ws/install/setup.bash && "
                f"{command}"
            ),
        ],
        detach=True,
        name=P5_ROSBAG_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={**_baseline_env(config), "PYTHONPATH": "/workspace"},
    )


def _finish_p5_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    profile_path = Path(config.frame_contract_rosbag_profile)
    required, optional, _topics = _profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    try:
        rc = DockerClient().wait(P5_ROSBAG_CONTAINER)
    except DockerException as exc:
        rc = exc.return_code or 1
    try:
        output = DockerClient().logs(P5_ROSBAG_CONTAINER, tail=2000)
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


def _build_p5_doctor_summary(config: RunConfig, *, runtime_config: Path) -> dict[str, Any]:
    p5 = config.orchestration.frame_contract
    baseline_doctor = _build_doctor_summary(config)
    p3_runtime_config = config.artifact_dir / "p5_doctor_p3_slam_runtime.toml"
    _write_p3_slam_runtime_config(config, p3_runtime_config)
    p3_doctor = _build_p3_doctor_summary(config, runtime_config=p3_runtime_config)
    p4_runtime_config = config.artifact_dir / "p5_doctor_p4_fcu_controller_runtime.toml"
    _write_p4_runtime_config(config, p4_runtime_config)
    p4_doctor = _build_p4_doctor_summary(config, runtime_config=p4_runtime_config)
    profile_path = Path(p5.rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    blockers = [
        *[str(item) for item in baseline_doctor.get("blockers", [])],
        *[str(item) for item in p3_doctor.get("blockers", [])],
        *[str(item) for item in p4_doctor.get("blockers", [])],
    ]
    if not profile_path.is_file() or not topics:
        blockers.append("P5 rosbag profile is missing or empty")
    if p5.uses_gazebo_truth_as_input:
        blockers.append("P5 must not use Gazebo truth as a control/planning/ExternalNav input")
    if not p5.required_frames:
        blockers.append("P5 required_frames is empty")
    summary = {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "p5_frame_contract_doctor": {
            "runtime_config": str(runtime_config),
            "runtime_config_sha256": _file_sha256(runtime_config) if runtime_config.is_file() else "",
            "required_frames": list(p5.required_frames),
            "tf_chain": [p5.map_frame_id, p5.odom_frame_id, p5.base_frame_id],
            "sensor_frames": [p5.imu_frame_id, p5.laser_frame_id, p5.rangefinder_frame_id],
            "scan_topic": p5.scan_topic,
            "status_topic": p5.status_topic,
            "rosbag_profile": {
                "profile": str(profile_path),
                "required_topics": required,
                "optional_topics": optional,
            },
            "thresholds": {
                "max_dynamic_tf_age_sec": p5.max_dynamic_tf_age_sec,
                "min_scan_valid_ratio": p5.min_scan_valid_ratio,
                "max_rangefinder_height_error_m": p5.max_rangefinder_height_error_m,
                "max_direction_error_rad": p5.max_direction_error_rad,
            },
            "hover_claim": p5.hover_claim,
            "exploration_claim": p5.exploration_claim,
        },
        "official_baseline_doctor": baseline_doctor,
        "p3_slam_backend_doctor": p3_doctor,
        "p4_fcu_controller_doctor": p4_doctor,
    }
    return summary


def _append_p5_blockers(
    *,
    blockers: list[str],
    frame_summary: dict[str, Any],
    rosbag_profile: dict[str, Any],
    counts: dict[str, int],
    p5: Any,
) -> None:
    if not frame_summary:
        blockers.append("P5 frame probe summary is missing")
        return
    if not frame_summary.get("ok"):
        blockers.extend(str(item) for item in frame_summary.get("blockers", []))
    if not frame_summary.get("tf", {}).get("ok"):
        blockers.append("P5 TF contract did not pass")
    if not frame_summary.get("scan", {}).get("ok"):
        blockers.append("P5 scan contract did not pass")
    if not frame_summary.get("imu", {}).get("ok"):
        blockers.append("P5 IMU contract did not pass")
    if not frame_summary.get("rangefinder", {}).get("ok"):
        blockers.append("P5 rangefinder contract did not pass")
    if p5.require_motion_direction_check and not frame_summary.get("direction_consistency", {}).get("ok"):
        blockers.append("P5 direction consistency did not pass")
    if p5.uses_gazebo_truth_as_input:
        blockers.append("P5 is configured to use Gazebo truth as input")
    if not rosbag_profile.get("ok"):
        blockers.append("P5 rosbag profile did not pass")
    if counts.get(p5.status_topic, 0) <= 0:
        blockers.append(f"{p5.status_topic} was not recorded")


def _write_foxglove_notes(config: RunConfig) -> None:
    p5 = config.orchestration.frame_contract
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P5 frame contract replay notes",
                "",
                "P5 validates TF, scan, sensor frames, and direction diagnostics. It is not a hover completion gate.",
                "",
                f"- Fixed frame: `{p5.map_frame_id}` when available; otherwise inspect the summary blocker before switching.",
                f"- Frame status: `{p5.status_topic}`.",
                f"- Scan: `{p5.scan_topic}` in `{p5.laser_frame_id}`.",
                f"- FCU pose/twist: `{p5.fcu_pose_topic}`, `{p5.fcu_twist_topic}`.",
                f"- SLAM odom: `{p5.slam_odom_topic}`.",
                f"- Diagnostic truth only: `{p5.truth_diagnostic_topic}`.",
                "- Do not interpret this bag as P6 SLAM hover completion or exploration completion.",
            ]
        )
        + "\n",
    )


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class FrameContractDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "frame-contract-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check P5 frame contract prerequisites."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_frame_contract_doctor/{config.run_id}"))
        artifact_dir.mkdir(parents=True, exist_ok=True)
        config = RunConfig.from_config(config_path=config_path, artifact_dir=artifact_dir, run_id=config.run_id)
        runtime_config = artifact_dir / "p5_frame_contract_runtime.toml"
        _write_p5_runtime_config(config, runtime_config)
        console.print("[bold cyan]Checking P5 frame contract prerequisites[/bold cyan]")
        summary = _build_p5_doctor_summary(config, runtime_config=runtime_config)
        _write_json(artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P5 frame contract doctor rc={0 if summary['ok'] else 20}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 20


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class FrameContractAcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "frame-contract-acceptance"
    TASK_DESCRIPTION: ClassVar[str] = "Run P5 frame contract acceptance."

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
        p4 = config.orchestration.fcu_controller
        p5 = config.orchestration.frame_contract
        bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p5.yaml"
        model_overlay = config.artifact_dir / "iris_with_lidar_p5_rangefinder_x2.sdf"
        param_overlay = config.artifact_dir / "gazebo-iris-p5-rangefinder.parm"
        sensor_config = config.artifact_dir / "p5_gazebo_sensor_runtime.toml"
        vendor_profile = config.artifact_dir / "x2_vendor_driver_p5.yaml"
        slam_runtime_config = config.artifact_dir / "p5_slam_runtime.toml"
        p4_runtime_config = config.artifact_dir / "p5_fcu_controller_runtime.toml"
        p5_runtime_config = config.artifact_dir / "p5_frame_contract_runtime.toml"
        controller_script = config.artifact_dir / "p5_fcu_controller_runtime.py"
        frame_probe_script = config.artifact_dir / "p5_frame_contract_probe.py"
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
            controller_script_summary = _write_controller_runtime_script(
                config,
                controller_script,
                duration_sec=max(35.0, min(duration_sec, 75.0)),
            )
            frame_probe_script_summary = _write_frame_probe_script(config, frame_probe_script)

            console.print("[bold cyan]Starting official maze + P5 frame contract gate[/bold cyan]")
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
            _start_p5_rosbag_recording(config, duration_sec=max(60.0, min(duration_sec, 105.0)))
            time.sleep(2.0)
            _start_p4_controller_container(config, script_path=controller_script)
            time.sleep(8.0)
            frame_summary = _run_frame_probe(config, script_path=frame_probe_script)
            controller_summary = _wait_for_controller_summary(config, timeout_sec=max(45.0, duration_sec + 20.0))
            rosbag_profile = _finish_p5_rosbag_recording(config)
            counts = _message_counts(config)

            graph = _collect_ros_graph(config, config.artifact_dir, image=baseline.runtime_image, network="host")
            probe = _collect_official_dds_probe(config, config.artifact_dir, image=baseline.runtime_image, network="host")
            rangefinder_probe = _collect_rangefinder_probe(config, image=baseline.runtime_image)
            imu_probe = _collect_imu_probe(config, image=baseline.runtime_image)
            topic_info = _collect_topic_info(
                config,
                image=baseline.runtime_image,
                topics=(
                    p5.scan_topic,
                    p5.imu_topic,
                    p5.rangefinder_range_topic,
                    p5.rangefinder_status_topic,
                    p5.fcu_pose_topic,
                    p5.fcu_twist_topic,
                    p5.fcu_status_topic,
                    p5.cmd_vel_topic,
                    p5.slam_odom_topic,
                    p5.slam_status_topic,
                    p5.truth_diagnostic_topic,
                    p5.controller_status_topic,
                    p5.setpoint_output_topic,
                    p5.owner_status_topic,
                    p5.status_topic,
                ),
            )
            doctor = _build_p5_doctor_summary(config, runtime_config=p5_runtime_config)
            blockers: list[str] = []
            if not doctor.get("ok"):
                blockers.extend(str(item) for item in doctor.get("blockers", []))
            if not probe.get("result", {}).get("time_received"):
                blockers.append("official DDS probe did not receive /ap/v1/time")
            if not rangefinder_probe.get("result", {}).get("range_received"):
                blockers.append("P5 did not receive rangefinder")
            if not imu_probe.get("result", {}).get("received"):
                blockers.append("P5 did not receive IMU")
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
                rosbag_profile=rosbag_profile,
                counts=counts,
                p5=p5,
            )
            for topic in (
                p5.scan_topic,
                p5.imu_topic,
                p5.rangefinder_range_topic,
                p5.fcu_pose_topic,
                p5.slam_odom_topic,
                p5.status_topic,
            ):
                if counts.get(topic, 0) <= 0:
                    blockers.append(f"{topic} was not recorded")

            summary = {
                "ok": not blockers,
                "blocked": bool(blockers),
                "blockers": blockers,
                "p5_frame_contract": {
                    "runtime_config": p5_runtime_summary,
                    "frame_probe_script": frame_probe_script_summary,
                    "frame_probe": frame_summary,
                    "model_overlay": model_overlay_summary,
                    "param_overlay": param_overlay_summary,
                    "sensor_config": sensor_config_summary,
                    "slam_runtime_config": slam_runtime_summary,
                    "p4_runtime_config": p4_runtime_summary,
                    "controller_script": controller_script_summary,
                    "controller_runtime": controller_summary,
                    "owner": owner_summary,
                    "set_pose_count": owner_summary.get("set_pose_count"),
                    "uses_gazebo_truth_as_input": p5.uses_gazebo_truth_as_input,
                    "hover_claim": p5.hover_claim,
                    "exploration_claim": p5.exploration_claim,
                    "rosbag_path": str(config.artifact_dir / "rosbag"),
                    "mcap_path": str(config.artifact_dir / "rosbag" / "rosbag_0.mcap"),
                },
                "tf": frame_summary.get("tf", {}),
                "scan": frame_summary.get("scan", {}),
                "imu": frame_summary.get("imu", {}),
                "rangefinder": frame_summary.get("rangefinder", {}),
                "direction_consistency": frame_summary.get("direction_consistency", {}),
                "official_dds_probe": probe,
                "rangefinder_probe": rangefinder_probe.get("result", {}),
                "imu_probe": imu_probe.get("result", {}),
                "topic_info": topic_info,
                "ros_graph": graph,
                "message_counts": counts,
                "rosbag_profile": rosbag_profile,
                "hover_claim": p5.hover_claim,
                "exploration_claim": p5.exploration_claim,
            }
            _write_json(config.artifact_dir / "summary.json", summary)
            _write_foxglove_notes(config)
        finally:
            host._capture_official_baseline_log(config=config)
            _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_tail.log")
            _capture_container_log(config, container=SLAM_BACKEND_CONTAINER, output_name="slam_backend_tail.log")
            _capture_container_log(config, container=P4_CONTROLLER_CONTAINER, output_name="fcu_controller_tail.log")
            _capture_container_log(config, container=P5_ROSBAG_CONTAINER, output_name="rosbag_tail.log")
            _remove_container(P5_ROSBAG_CONTAINER)
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
                "blockers": ["P5 frame contract acceptance did not produce a summary"],
            }
            _write_json(config.artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P5 frame contract acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 30
