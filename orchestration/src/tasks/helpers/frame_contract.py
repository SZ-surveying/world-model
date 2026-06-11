from __future__ import annotations

import json
import shlex
from pathlib import Path
from typing import Any

import tomli_w

from src import host
from src.configs.run_config import RunConfig
from src.tasks.helpers.artifacts import file_sha256, write_text
from src.tasks.helpers.fcu import (
    build_p4_doctor_summary,
    write_p4_runtime_config,
)
from src.tasks.helpers.official_stack import (
    build_doctor_summary,
)
from src.tasks.helpers.rosbag_profiles import load_rosbag_metadata_counts, profile_topics
from src.tasks.helpers.slam import (
    build_p3_doctor_summary,
    write_p3_slam_runtime_config,
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


def write_p5_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
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
    return {"path": str(path), "workspace_path": host.workspace_path(path), "sha256": file_sha256(path), "data": data}


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


def write_frame_probe_script(config: RunConfig, script_path: Path) -> dict[str, Any]:
    p5 = config.orchestration.frame_contract
    summary_file = config.artifact_dir / "frame_contract_summary.json"
    spec = {
        "summary_file": host.workspace_path(summary_file),
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
    write_text(script_path, _frame_probe_script(spec))
    return {
        "path": str(script_path),
        "workspace_path": host.workspace_path(script_path),
        "sha256": file_sha256(script_path),
        "summary_file": str(summary_file),
        "spec": spec,
    }


def run_frame_probe(config: RunConfig, *, script_path: Path) -> dict[str, Any]:
    command = f"python3 {shlex.quote(host.workspace_path(script_path))}"
    rc, output = host.docker_run_ros_shell_capture(
        config=config,
        image=config.orchestration.official_baseline.runtime_image,
        shell_command=command,
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    write_text(config.artifact_dir / "frame_contract_probe.txt", output)
    summary = _load_json(config.artifact_dir / "frame_contract_summary.json")
    if not summary:
        summary = {"ok": False, "blockers": [f"frame probe failed rc={rc}"], "output": output}
    summary["rc"] = rc
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
    return load_rosbag_metadata_counts(metadata)


def build_p5_doctor_summary(config: RunConfig, *, runtime_config: Path) -> dict[str, Any]:
    p5 = config.orchestration.frame_contract
    baseline_doctor = build_doctor_summary(config)
    p3_runtime_config = config.artifact_dir / "p5_doctor_p3_slam_runtime.toml"
    write_p3_slam_runtime_config(config, p3_runtime_config)
    p3_doctor = build_p3_doctor_summary(config, runtime_config=p3_runtime_config)
    p4_runtime_config = config.artifact_dir / "p5_doctor_p4_fcu_controller_runtime.toml"
    write_p4_runtime_config(config, p4_runtime_config)
    p4_doctor = build_p4_doctor_summary(config, runtime_config=p4_runtime_config)
    profile_path = Path(p5.rosbag_profile)
    required, optional, topics = profile_topics(profile_path)
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
            "runtime_config_sha256": file_sha256(runtime_config) if runtime_config.is_file() else "",
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


def append_p5_blockers(
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
    write_text(
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
