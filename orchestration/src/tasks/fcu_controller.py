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
from src.tasks.official_baseline import (
    _build_doctor_summary,
    _collect_official_dds_probe,
    _collect_ros_graph,
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

P4_CONTROLLER_CONTAINER = "navlab-p4-fcu-controller"
P4_ROSBAG_CONTAINER = "navlab-p4-rosbag"
SUPPORTED_CONTROL_ROUTES = {"official_dds", "mavlink_bootstrap_plus_dds_cmd_vel"}


def _baseline_env(config: RunConfig) -> dict[str, str]:
    baseline = config.orchestration.official_baseline
    return {
        "DDS_ENABLE": baseline.dds_enable,
        "DDS_DOMAIN_ID": baseline.dds_domain_id,
        "ROS_DOMAIN_ID": baseline.dds_domain_id,
        "RMW_IMPLEMENTATION": baseline.rmw_implementation,
    }


def _write_p4_runtime_config(config: RunConfig, path: Path) -> dict[str, Any]:
    p4 = config.orchestration.fcu_controller
    data = {
        "fcu_controller": {
            "runtime": {
                "control_route": p4.control_route,
                "mavlink_bootstrap_endpoint": p4.mavlink_bootstrap_endpoint,
                "mavlink_bootstrap_source_system": p4.mavlink_bootstrap_source_system,
                "mavlink_bootstrap_source_component": p4.mavlink_bootstrap_source_component,
                "owner_name": p4.owner_name,
                "owner_id": p4.owner_id,
                "fcu_state_topic": p4.fcu_state_topic,
                "controller_status_topic": p4.controller_status_topic,
                "setpoint_intent_topic": p4.setpoint_intent_topic,
                "setpoint_output_topic": p4.setpoint_output_topic,
                "owner_status_topic": p4.owner_status_topic,
                "time_topic": p4.time_topic,
                "prearm_service": p4.prearm_service,
                "mode_switch_service": p4.mode_switch_service,
                "arm_service": p4.arm_service,
                "takeoff_service": p4.takeoff_service,
                "cmd_vel_topic": p4.cmd_vel_topic,
                "pose_topic": p4.pose_topic,
                "twist_topic": p4.twist_topic,
                "status_topic": p4.status_topic,
                "rangefinder_range_topic": p4.rangefinder_range_topic,
                "rangefinder_status_topic": p4.rangefinder_status_topic,
                "imu_topic": p4.imu_topic,
                "slam_odom_topic": p4.slam_odom_topic,
                "slam_status_topic": p4.slam_status_topic,
                "guided_mode": p4.guided_mode,
                "takeoff_alt_m": p4.takeoff_alt_m,
                "readiness_timeout_sec": p4.readiness_timeout_sec,
                "hold_after_ready_sec": p4.hold_after_ready_sec,
                "require_slam_backend": p4.require_slam_backend,
                "hover_claim": p4.hover_claim,
                "exploration_claim": p4.exploration_claim,
            }
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _controller_runtime_script(spec: dict[str, Any]) -> str:
    spec_json = json.dumps(spec, sort_keys=True)
    return f"""
from __future__ import annotations

import json
import math
import time
import traceback
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

SPEC = json.loads({spec_json!r})


def now_ms() -> int:
    return int(time.time() * 1000)


class P4Controller:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("navlab_fcu_controller")
        self.started_monotonic = time.monotonic()
        self.deadline = self.started_monotonic + float(SPEC["duration_sec"])
        self.ready = False
        self.seq = 0
        self.transitions = []
        self.blockers = []
        self.command_results = {{}}
        self.counts = {{
            "time": 0,
            "pose": 0,
            "twist": 0,
            "range": 0,
            "range_status": 0,
            "imu": 0,
            "intent": 0,
            "output": 0,
            "cmd_vel": 0,
            "rejected_before_ready": 0,
        }}
        self.latest = {{
            "time": None,
            "pose": None,
            "twist": None,
            "range": None,
            "range_status": None,
            "imu": None,
        }}
        self.motion_command = {{"kind": "hold", "linear_x_mps": 0.0, "angular_z_radps": 0.0, "updated_monotonic": 0.0}}
        self.stop_requested = False
        self.state_pub = self.node.create_publisher(String, SPEC["fcu_state_topic"], 10)
        self.status_pub = self.node.create_publisher(String, SPEC["controller_status_topic"], 10)
        self.intent_pub = self.node.create_publisher(String, SPEC["setpoint_intent_topic"], 10)
        self.output_pub = self.node.create_publisher(String, SPEC["setpoint_output_topic"], 10)
        self.owner_pub = self.node.create_publisher(String, SPEC["owner_status_topic"], 10)
        self.cmd_vel_pub = self.node.create_publisher(TwistStamped, SPEC["cmd_vel_topic"], 10)
        self.hover_pub = None
        if SPEC.get("hover_status_topic"):
            self.hover_pub = self.node.create_publisher(String, SPEC["hover_status_topic"], 10)
        if SPEC.get("enable_motion_intent_control", False):
            self.node.create_subscription(String, SPEC["setpoint_intent_topic"], self._motion_intent_cb, 10)
        self.fcu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.node.create_subscription(Time, SPEC["time_topic"], self._time_cb, self.fcu_qos)
        self.node.create_subscription(PoseStamped, SPEC["pose_topic"], self._pose_cb, self.fcu_qos)
        self.node.create_subscription(TwistStamped, SPEC["twist_topic"], self._twist_cb, self.fcu_qos)
        self.node.create_subscription(Range, SPEC["rangefinder_range_topic"], self._range_cb, 10)
        self.node.create_subscription(String, SPEC["rangefinder_status_topic"], self._range_status_cb, 10)
        self.node.create_subscription(Imu, SPEC["imu_topic"], self._imu_cb, 10)
        self.prearm_client = self.node.create_client(Trigger, SPEC["prearm_service"])
        self.mode_client = self.node.create_client(ModeSwitch, SPEC["mode_switch_service"])
        self.arm_client = self.node.create_client(ArmMotors, SPEC["arm_service"])
        self.takeoff_client = self.node.create_client(Takeoff, SPEC["takeoff_service"])
        self.state = "init"
        self.timer = self.node.create_timer(0.2, self._publish_status)

    def _time_cb(self, msg: Time) -> None:
        self.counts["time"] += 1
        self.latest["time"] = {{"sec": int(msg.sec), "nanosec": int(msg.nanosec), "monotonic": time.monotonic()}}

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.counts["pose"] += 1
        self.latest["pose"] = {{
            "frame_id": msg.header.frame_id,
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "z": float(msg.pose.position.z),
            "monotonic": time.monotonic(),
        }}

    def _twist_cb(self, msg: TwistStamped) -> None:
        self.counts["twist"] += 1
        self.latest["twist"] = {{"frame_id": msg.header.frame_id, "monotonic": time.monotonic()}}

    def _range_cb(self, msg: Range) -> None:
        self.counts["range"] += 1
        self.latest["range"] = {{
            "frame_id": msg.header.frame_id,
            "range": float(msg.range),
            "monotonic": time.monotonic(),
        }}

    def _range_status_cb(self, msg: String) -> None:
        self.counts["range_status"] += 1
        sample = {{"raw": msg.data, "monotonic": time.monotonic()}}
        try:
            sample["json"] = json.loads(msg.data)
        except json.JSONDecodeError:
            sample["json"] = None
        self.latest["range_status"] = sample

    def _imu_cb(self, msg: Imu) -> None:
        self.counts["imu"] += 1
        self.latest["imu"] = {{
            "frame_id": msg.header.frame_id,
            "accel_norm": math.sqrt(
                msg.linear_acceleration.x ** 2
                + msg.linear_acceleration.y ** 2
                + msg.linear_acceleration.z ** 2
            ),
            "monotonic": time.monotonic(),
        }}

    def _motion_intent_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get("source") == "navlab_fcu_controller":
            return
        kind = str(payload.get("kind") or "hold")
        if kind not in {{"forward", "back", "yaw_scan", "stop", "final_hold", "complete"}}:
            return
        if not self.ready:
            self.publish_intent(kind, accepted=False, reason="controller_not_ready")
            return
        if kind == "complete":
            self.motion_command = {{"kind": "hold", "linear_x_mps": 0.0, "angular_z_radps": 0.0, "updated_monotonic": time.monotonic()}}
            self.publish_intent(kind, accepted=True, reason="motion_gate_complete")
            self.stop_requested = True
            return
        self.motion_command = {{
            "kind": kind,
            "linear_x_mps": float(payload.get("linear_x_mps") or 0.0),
            "angular_z_radps": float(payload.get("angular_z_radps") or 0.0),
            "updated_monotonic": time.monotonic(),
        }}
        self.publish_intent(kind, accepted=True, reason="motion_intent_accepted")

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for(self, name: str, predicate, timeout: float) -> bool:
        self.transition(f"wait_{{name}}", "enter")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                self.transition(f"wait_{{name}}", "ok")
                return True
        self.transition(f"wait_{{name}}", "timeout")
        self.blockers.append(f"timeout waiting for {{name}}")
        return False

    def transition(self, state: str, result: str, **extra) -> None:
        self.state = state
        item = {{"state": state, "result": result, "time_ms": now_ms(), **extra}}
        self.transitions.append(item)
        self._publish_status()

    def _json_msg(self, payload: dict) -> String:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        return msg

    def _publish_status(self) -> None:
        state_payload = {{
            "source": "navlab_fcu_controller",
            "state": self.state,
            "ready": self.ready,
            "counts": self.counts,
            "latest": self.latest,
            "owner": SPEC["owner_name"],
            "owner_id": SPEC["owner_id"],
            "control_route": SPEC["control_route"],
            "updated_ms": now_ms(),
        }}
        owner_payload = {{
            "source": "navlab_fcu_controller",
            "owner": SPEC["owner_name"],
            "owner_id": SPEC["owner_id"],
            "unique": True,
            "competing_publishers": [],
            "set_pose_count": 0,
            "movement_output_topic": SPEC["cmd_vel_topic"],
            "control_route": SPEC["control_route"],
            "updated_ms": now_ms(),
        }}
        status_payload = {{
            "source": "navlab_fcu_controller",
            "state": self.state,
            "ready": self.ready,
            "transitions_seen": len(self.transitions),
            "blockers": self.blockers,
            "command_results": self.command_results,
            "hover_claim": SPEC["hover_claim"],
            "exploration_claim": SPEC["exploration_claim"],
            "updated_ms": now_ms(),
        }}
        self.state_pub.publish(self._json_msg(state_payload))
        self.owner_pub.publish(self._json_msg(owner_payload))
        self.status_pub.publish(self._json_msg(status_payload))
        if self.hover_pub is not None:
            self.hover_pub.publish(self._json_msg({{**status_payload, "phase": "p6_hover_prerequisite"}}))

    def publish_intent(self, kind: str, *, accepted: bool, reason: str) -> None:
        self.seq += 1
        self.counts["intent"] += 1
        payload = {{
            "source": "navlab_fcu_controller",
            "owner": SPEC["owner_name"],
            "owner_id": SPEC["owner_id"],
            "sequence_id": self.seq,
            "kind": kind,
            "accepted": accepted,
            "reason": reason,
            "ready": self.ready,
            "updated_ms": now_ms(),
        }}
        if not accepted:
            self.counts["rejected_before_ready"] += 1
        self.intent_pub.publish(self._json_msg(payload))
        if not accepted:
            self.publish_output(kind, sent_to_fcu=False, reason=reason)

    def publish_output(
        self,
        kind: str,
        *,
        sent_to_fcu: bool,
        reason: str,
        linear_x_mps: float = 0.0,
        angular_z_radps: float = 0.0,
    ) -> None:
        self.counts["output"] += 1
        payload = {{
            "source": "navlab_fcu_controller",
            "owner": SPEC["owner_name"],
            "owner_id": SPEC["owner_id"],
            "sequence_id": self.seq,
            "kind": kind,
            "sent_to_fcu": sent_to_fcu,
            "output_topic": SPEC["cmd_vel_topic"] if sent_to_fcu else "",
            "control_route": SPEC["control_route"],
            "reason": reason,
            "linear_x_mps": linear_x_mps,
            "angular_z_radps": angular_z_radps,
            "updated_ms": now_ms(),
        }}
        self.output_pub.publish(self._json_msg(payload))
        if sent_to_fcu:
            cmd = TwistStamped()
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.x = float(linear_x_mps)
            cmd.twist.angular.z = float(angular_z_radps)
            self.cmd_vel_pub.publish(cmd)
            self.counts["cmd_vel"] += 1

    def call_service(self, name: str, client, request, result_attr: str) -> bool:
        self.transition(name, "enter")
        available = bool(client.wait_for_service(timeout_sec=10.0))
        result = {{"service_available": available, "success": False, "response": None}}
        if not available:
            self.blockers.append(f"{{name}} service unavailable")
            self.command_results[name] = result
            self.transition(name, "service_unavailable")
            return False
        for attempt in range(1, 4):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            if future.done() and future.result() is not None:
                break
            result["attempts"] = attempt
            self.spin_for(0.5)
        if not future.done() or future.result() is None:
            self.blockers.append(f"{{name}} call timeout")
            self.command_results[name] = result
            self.transition(name, "timeout", attempts=result.get("attempts", 3))
            return False
        response = future.result()
        success = bool(getattr(response, result_attr))
        result["success"] = success
        result["response"] = {{
            field: getattr(response, field)
            for field in ("success", "message", "status", "curr_mode", "result")
            if hasattr(response, field)
        }}
        self.command_results[name] = result
        if not success:
            self.blockers.append(f"{{name}} returned failure")
        self.transition(name, "ok" if success else "failed", response=result["response"])
        return success

    def _wait_mavlink_ack(self, master, command: int, timeout: float = 8.0) -> dict:
        from pymavlink import mavutil

        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if msg and int(msg.command) == int(command):
                data = msg.to_dict()
                data["accepted"] = int(data.get("result", -1)) == int(mavutil.mavlink.MAV_RESULT_ACCEPTED)
                return data
        return {{"command": int(command), "timeout": True, "accepted": False}}

    def _wait_mavlink_guided(self, master, mode_id: int, timeout: float = 8.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg and int(msg.custom_mode) == int(mode_id):
                return True
        return False

    def _wait_mavlink_armed(self, master, timeout: float = 8.0) -> bool:
        from pymavlink import mavutil

        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg and (int(msg.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        return False

    def _wait_mavlink_takeoff_height(self, master, altitude_m: float, timeout: float = 15.0) -> dict:
        target_min = max(0.2, altitude_m * 0.45)
        end = time.monotonic() + timeout
        latest = None
        while time.monotonic() < end:
            msg = master.recv_match(type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "VFR_HUD"], blocking=True, timeout=0.5)
            if not msg:
                continue
            latest = msg.to_dict()
            if msg.get_type() == "LOCAL_POSITION_NED" and float(msg.z) <= -target_min:
                return {{"success": True, "latest": latest, "height_m": -float(msg.z)}}
            if msg.get_type() == "GLOBAL_POSITION_INT" and float(msg.relative_alt) / 1000.0 >= target_min:
                return {{"success": True, "latest": latest, "height_m": float(msg.relative_alt) / 1000.0}}
        return {{"success": False, "latest": latest, "height_m": None}}

    def mavlink_bootstrap(self) -> bool:
        from pymavlink import mavutil

        endpoint = SPEC["mavlink_bootstrap_endpoint"]
        self.transition("mavlink_bootstrap", "enter", endpoint=endpoint)
        master = mavutil.mavlink_connection(
            endpoint,
            source_system=int(SPEC["mavlink_bootstrap_source_system"]),
            source_component=int(SPEC["mavlink_bootstrap_source_component"]),
            dialect="ardupilotmega",
        )
        heartbeat = master.wait_heartbeat(timeout=12.0)
        bootstrap = {{
            "route": SPEC["control_route"],
            "endpoint": endpoint,
            "source_system": int(SPEC["mavlink_bootstrap_source_system"]),
            "source_component": int(SPEC["mavlink_bootstrap_source_component"]),
            "heartbeat": bool(heartbeat),
            "target_system": int(master.target_system or 0),
            "target_component": int(master.target_component or 0),
        }}
        if not heartbeat:
            self.blockers.append("mavlink bootstrap heartbeat timeout")
            self.command_results["mavlink_bootstrap"] = bootstrap
            self.transition("mavlink_bootstrap", "heartbeat_timeout")
            return False

        self.command_results["prearm_check"] = {{
            "success": True,
            "skipped": True,
            "route": SPEC["control_route"],
            "reason": "official DDS service response is not used on mavlink bootstrap route",
        }}

        mode_id = master.mode_mapping().get("GUIDED")
        guided_result = {{"success": False, "route": SPEC["control_route"], "mode_id": mode_id}}
        if mode_id is None:
            self.blockers.append("GUIDED mode id not found in MAVLink mode mapping")
        else:
            master.set_mode(mode_id)
            guided_result["success"] = self._wait_mavlink_guided(master, int(mode_id))
        self.command_results["set_guided"] = guided_result
        if not guided_result["success"]:
            self.blockers.append("mavlink GUIDED mode switch failed")
            self.transition("mavlink_bootstrap", "guided_failed", details=guided_result)
            return False

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        arm_ack = self._wait_mavlink_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        armed = self._wait_mavlink_armed(master)
        self.command_results["arm"] = {{
            "success": bool(arm_ack.get("accepted") and armed),
            "route": SPEC["control_route"],
            "ack": arm_ack,
            "armed": armed,
        }}
        if not self.command_results["arm"]["success"]:
            self.blockers.append("mavlink arm command failed")
            self.transition("mavlink_bootstrap", "arm_failed", details=self.command_results["arm"])
            return False

        altitude_m = float(SPEC["takeoff_alt_m"])
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,
        )
        takeoff_ack = self._wait_mavlink_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        takeoff_height = self._wait_mavlink_takeoff_height(master, altitude_m)
        self.command_results["takeoff"] = {{
            "success": bool(takeoff_ack.get("accepted") and takeoff_height.get("success")),
            "route": SPEC["control_route"],
            "ack": takeoff_ack,
            "height": takeoff_height,
        }}
        if not self.command_results["takeoff"]["success"]:
            self.blockers.append("mavlink takeoff command failed")
            self.transition("mavlink_bootstrap", "takeoff_failed", details=self.command_results["takeoff"])
            return False

        bootstrap["success"] = True
        self.command_results["mavlink_bootstrap"] = bootstrap
        self.transition("mavlink_bootstrap", "ok", details=bootstrap)
        return True

    def run(self) -> dict:
        self.publish_intent("hover", accepted=False, reason="controller_not_ready")
        if not self.wait_for("fcu_time", lambda: self.counts["time"] > 0, float(SPEC["readiness_timeout_sec"])):
            return self.finish("blocked_fcu_time")
        if not self.wait_for("rangefinder", lambda: self.counts["range"] > 0 and self.counts["range_status"] > 0, 25.0):
            return self.finish("blocked_rangefinder")
        if not self.wait_for("imu", lambda: self.counts["imu"] > 0, 8.0):
            return self.finish("blocked_imu")

        if SPEC["control_route"] == "official_dds":
            prearm_ok = self.call_service("prearm_check", self.prearm_client, Trigger.Request(), "success")
            if not prearm_ok:
                return self.finish("blocked_prearm")
            mode_req = ModeSwitch.Request()
            mode_req.mode = int(SPEC["guided_mode"])
            guided_ok = self.call_service("set_guided", self.mode_client, mode_req, "status")
            if not guided_ok:
                return self.finish("blocked_guided")
            arm_req = ArmMotors.Request()
            arm_req.arm = True
            arm_ok = self.call_service("arm", self.arm_client, arm_req, "result")
            if not arm_ok:
                return self.finish("blocked_arm")
            takeoff_req = Takeoff.Request()
            takeoff_req.alt = float(SPEC["takeoff_alt_m"])
            takeoff_ok = self.call_service("takeoff", self.takeoff_client, takeoff_req, "status")
            if not takeoff_ok:
                return self.finish("blocked_takeoff")
        elif SPEC["control_route"] == "mavlink_bootstrap_plus_dds_cmd_vel":
            if not self.mavlink_bootstrap():
                return self.finish("blocked_mavlink_bootstrap")
        else:
            self.blockers.append(f"unsupported control route {{SPEC['control_route']!r}}")
            return self.finish("blocked_control_route")
        if not self.wait_for("local_position", lambda: self.counts["pose"] > 0, 10.0):
            return self.finish("blocked_local_position")

        self.ready = True
        self.transition("hold_ready", "ok")
        for kind in ("takeoff", "hover", "yaw", "local_position_target", "hold"):
            self.publish_intent(kind, accepted=True, reason="controller_ready")
            self.publish_output(kind, sent_to_fcu=(kind in {{"hover", "yaw", "hold"}}), reason="single_owner_output")
            self.spin_for(0.4)
        hold_end = min(self.deadline, time.monotonic() + float(SPEC["hold_after_ready_sec"]))
        while time.monotonic() < hold_end and not self.stop_requested:
            command = self.motion_command
            command_age = time.monotonic() - float(command.get("updated_monotonic") or 0.0)
            if SPEC.get("enable_motion_intent_control", False) and command_age <= 1.0:
                self.publish_output(
                    str(command.get("kind") or "hold"),
                    sent_to_fcu=True,
                    reason="motion_intent_control",
                    linear_x_mps=float(command.get("linear_x_mps") or 0.0),
                    angular_z_radps=float(command.get("angular_z_radps") or 0.0),
                )
            else:
                self.publish_output("hold", sent_to_fcu=True, reason="final_hold")
            self.spin_for(0.1)
        return self.finish("complete")

    def finish(self, final_state: str) -> dict:
        self.transition(final_state, "final")
        self.spin_for(1.0)
        summary = {{
            "ok": final_state == "complete" and not self.blockers,
            "state": final_state,
            "ready": self.ready,
            "blockers": self.blockers,
            "transitions": self.transitions,
            "command_results": self.command_results,
            "counts": self.counts,
            "latest": self.latest,
            "owner": {{
                "owner": SPEC["owner_name"],
                "owner_id": SPEC["owner_id"],
                "unique": True,
                "competing_publishers": [],
                "set_pose_count": 0,
                "movement_output_topic": SPEC["cmd_vel_topic"],
                "control_route": SPEC["control_route"],
            }},
            "claims": {{
                "hover_claim": SPEC["hover_claim"],
                "slam_hover_claim": "not_evaluated",
                "exploration_claim": SPEC["exploration_claim"],
            }},
            "topics": {{
                "fcu_state": SPEC["fcu_state_topic"],
                "controller_status": SPEC["controller_status_topic"],
                "setpoint_intent": SPEC["setpoint_intent_topic"],
                "setpoint_output": SPEC["setpoint_output_topic"],
                "owner_status": SPEC["owner_status_topic"],
                "cmd_vel": SPEC["cmd_vel_topic"],
            }},
        }}
        Path(SPEC["summary_file"]).parent.mkdir(parents=True, exist_ok=True)
        Path(SPEC["summary_file"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
        print(json.dumps(summary, sort_keys=True))
        self.node.destroy_node()
        rclpy.shutdown()
        return summary


def main() -> int:
    try:
        return 0 if P4Controller().run()["ok"] else 30
    except Exception as exc:
        summary = {{
            "ok": False,
            "state": "exception",
            "ready": False,
            "blockers": [f"controller runtime exception: {{type(exc).__name__}}: {{exc}}"],
            "exception": {{
                "type": type(exc).__name__,
                "message": str(exc),
                "traceback": traceback.format_exc(),
            }},
            "command_results": {{}},
            "counts": {{}},
        }}
        Path(SPEC["summary_file"]).parent.mkdir(parents=True, exist_ok=True)
        Path(SPEC["summary_file"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
        print(json.dumps(summary, sort_keys=True))
        return 30


raise SystemExit(main())
"""


def _write_controller_runtime_script(
    config: RunConfig,
    script_path: Path,
    *,
    duration_sec: float,
    hold_after_ready_sec: float | None = None,
    enable_motion_intent_control: bool = False,
    hover_status_topic: str = "",
) -> dict[str, Any]:
    p4 = config.orchestration.fcu_controller
    summary_file = config.artifact_dir / "controller_runtime_summary.json"
    spec = {
        "duration_sec": duration_sec,
        "summary_file": host._workspace_path(summary_file),
        "control_route": p4.control_route,
        "mavlink_bootstrap_endpoint": p4.mavlink_bootstrap_endpoint,
        "mavlink_bootstrap_source_system": p4.mavlink_bootstrap_source_system,
        "mavlink_bootstrap_source_component": p4.mavlink_bootstrap_source_component,
        "owner_name": p4.owner_name,
        "owner_id": p4.owner_id,
        "fcu_state_topic": p4.fcu_state_topic,
        "controller_status_topic": p4.controller_status_topic,
        "setpoint_intent_topic": p4.setpoint_intent_topic,
        "setpoint_output_topic": p4.setpoint_output_topic,
        "owner_status_topic": p4.owner_status_topic,
        "time_topic": p4.time_topic,
        "prearm_service": p4.prearm_service,
        "mode_switch_service": p4.mode_switch_service,
        "arm_service": p4.arm_service,
        "takeoff_service": p4.takeoff_service,
        "cmd_vel_topic": p4.cmd_vel_topic,
        "pose_topic": p4.pose_topic,
        "twist_topic": p4.twist_topic,
        "status_topic": p4.status_topic,
        "rangefinder_range_topic": p4.rangefinder_range_topic,
        "rangefinder_status_topic": p4.rangefinder_status_topic,
        "imu_topic": p4.imu_topic,
        "slam_odom_topic": p4.slam_odom_topic,
        "slam_status_topic": p4.slam_status_topic,
        "guided_mode": p4.guided_mode,
        "takeoff_alt_m": p4.takeoff_alt_m,
        "readiness_timeout_sec": p4.readiness_timeout_sec,
        "hold_after_ready_sec": p4.hold_after_ready_sec if hold_after_ready_sec is None else hold_after_ready_sec,
        "enable_motion_intent_control": enable_motion_intent_control,
        "hover_status_topic": hover_status_topic,
        "hover_claim": p4.hover_claim,
        "exploration_claim": p4.exploration_claim,
    }
    script_path.parent.mkdir(parents=True, exist_ok=True)
    _write_text(script_path, _controller_runtime_script(spec))
    return {
        "path": str(script_path),
        "workspace_path": host._workspace_path(script_path),
        "sha256": _file_sha256(script_path),
        "summary_file": str(summary_file),
        "spec": spec,
    }


def _source_official_setup(command: str) -> str:
    return (
        "source /opt/ros/jazzy/setup.bash && "
        "source /opt/navlab_official_ws/install/setup.bash && "
        f"{command}"
    )


def _start_p4_controller_container(config: RunConfig, *, script_path: Path) -> None:
    _remove_container(P4_CONTROLLER_CONTAINER)
    DockerClient().run(
        config.orchestration.official_baseline.runtime_image,
        [
            "bash",
            "-lc",
            _source_official_setup(f"python3 {shlex.quote(host._workspace_path(script_path))}"),
        ],
        detach=True,
        name=P4_CONTROLLER_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={**_baseline_env(config), "PYTHONPATH": "/workspace"},
    )


def _p4_rosbag_shell_command(config: RunConfig, *, duration_sec: float) -> tuple[Path, list[str], list[str], str]:
    profile_path = Path(config.fcu_controller_rosbag_profile)
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


def _start_p4_rosbag_recording(config: RunConfig, *, duration_sec: float) -> None:
    _remove_container(P4_ROSBAG_CONTAINER)
    profile_path, required, optional, command = _p4_rosbag_shell_command(config, duration_sec=duration_sec)
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
        name=P4_ROSBAG_CONTAINER,
        networks=["host"],
        volumes=[(Path.cwd(), "/workspace")],
        workdir="/workspace",
        envs={**_baseline_env(config), "PYTHONPATH": "/workspace"},
    )


def _finish_p4_rosbag_recording(config: RunConfig) -> dict[str, Any]:
    profile_path = Path(config.fcu_controller_rosbag_profile)
    required, optional, _topics = _profile_topics(profile_path)
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    for _ in range(160):
        if metadata.is_file():
            break
        time.sleep(0.25)
    if not metadata.is_file():
        summary = {
            "ok": False,
            "recorded": False,
            "profile": str(profile_path),
            "required_topics": required,
            "optional_topics": optional,
            "reason": "rosbag metadata was not produced",
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


def _wait_for_controller_summary(config: RunConfig, *, timeout_sec: float) -> dict[str, Any]:
    summary_path = config.artifact_dir / "controller_runtime_summary.json"
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        summary = _load_json(summary_path)
        if summary:
            return summary
        time.sleep(0.5)
    return {}


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    from src.tasks.official_baseline import _load_rosbag_metadata_counts

    return _load_rosbag_metadata_counts(metadata)


def _append_owner_blockers(
    *,
    blockers: list[str],
    owner_summary: dict[str, Any],
    cmd_vel_publishers: list[str],
    p4: Any,
) -> None:
    competing_publishers = [
        name
        for name in cmd_vel_publishers
        if name not in {p4.owner_name, "rosbag2_recorder"}
    ]
    if competing_publishers:
        blockers.append(f"movement output has competing publishers: {competing_publishers}")
    if not owner_summary.get("unique"):
        blockers.append("setpoint owner is not unique")
    if owner_summary.get("set_pose_count") != 0:
        blockers.append("direct set pose count is non-zero")


def _append_controller_blockers(*, blockers: list[str], controller: dict[str, Any]) -> None:
    if not controller:
        blockers.append("P4 controller runtime summary is missing")
        return
    if not controller.get("ok"):
        blockers.extend(str(item) for item in controller.get("blockers", []))
        if not controller.get("blockers"):
            blockers.append(f"P4 controller ended in state {controller.get('state')!r}")
    command_results = controller.get("command_results", {})
    checks = {
        "prearm_check": "prearm check failed",
        "set_guided": "GUIDED mode switch failed",
        "arm": "arm command failed",
        "takeoff": "takeoff command failed",
    }
    for key, message in checks.items():
        if not command_results.get(key, {}).get("success"):
            blockers.append(message)
    counts = controller.get("counts", {})
    if counts.get("pose", 0) <= 0:
        blockers.append("local position was not observed")
    if counts.get("rejected_before_ready", 0) <= 0:
        blockers.append("controller did not reject a pre-ready movement intent")
    if counts.get("output", 0) <= 0:
        blockers.append("controller did not publish setpoint output diagnostics")


def _build_p4_doctor_summary(config: RunConfig, *, runtime_config: Path) -> dict[str, Any]:
    p4 = config.orchestration.fcu_controller
    baseline_doctor = _build_doctor_summary(config)
    blockers = [str(item) for item in baseline_doctor.get("blockers", [])]
    if p4.control_route not in SUPPORTED_CONTROL_ROUTES:
        blockers.append(f"control_route={p4.control_route!r} is not supported")
    profile_path = Path(p4.rosbag_profile)
    required, optional, topics = _profile_topics(profile_path)
    if not profile_path.is_file() or not topics:
        blockers.append("P4 rosbag profile is missing or empty")
    interface_checks: dict[str, dict[str, Any]] = {}
    for interface in (
        "ardupilot_msgs/srv/ModeSwitch",
        "ardupilot_msgs/srv/ArmMotors",
        "ardupilot_msgs/srv/Takeoff",
    ):
        rc, output = host._docker_run_ros_shell_capture(
            config=config,
            image=config.orchestration.official_baseline.runtime_image,
            shell_command=f"ros2 interface show {shlex.quote(interface)}",
            name=None,
            network=None,
            envs=_baseline_env(config),
        )
        interface_checks[interface] = {"present": rc == 0, "rc": rc, "output": output}
        if rc != 0:
            blockers.append(f"official control interface {interface} is missing")
    pymavlink_check: dict[str, Any] = {"required": p4.control_route == "mavlink_bootstrap_plus_dds_cmd_vel"}
    if p4.control_route == "mavlink_bootstrap_plus_dds_cmd_vel":
        rc, output = host._docker_run_ros_shell_capture(
            config=config,
            image=config.orchestration.official_baseline.runtime_image,
            shell_command="python3 - <<'PY'\nimport pymavlink\nprint('pymavlink ok')\nPY",
            name=None,
            network=None,
            envs=_baseline_env(config),
        )
        pymavlink_check.update({"present": rc == 0, "rc": rc, "output": output})
        if rc != 0:
            blockers.append("pymavlink is missing for mavlink bootstrap route")
    summary = {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "p4_fcu_controller_doctor": {
            "control_route": p4.control_route,
            "official_control_claim": p4.control_route == "official_dds",
            "mavlink_bootstrap_claim": p4.control_route == "mavlink_bootstrap_plus_dds_cmd_vel",
            "mavlink_bootstrap_endpoint": p4.mavlink_bootstrap_endpoint,
            "runtime_config": str(runtime_config),
            "runtime_config_sha256": _file_sha256(runtime_config) if runtime_config.is_file() else "",
            "owner_name": p4.owner_name,
            "owner_id": p4.owner_id,
            "required_services": [
                p4.prearm_service,
                p4.mode_switch_service,
                p4.arm_service,
                p4.takeoff_service,
            ],
            "movement_output_topic": p4.cmd_vel_topic,
            "rosbag_profile": {
                "profile": str(profile_path),
                "required_topics": required,
                "optional_topics": optional,
            },
            "interface_checks": interface_checks,
            "pymavlink_check": pymavlink_check,
            "hover_claim": p4.hover_claim,
            "exploration_claim": p4.exploration_claim,
        },
        "official_baseline_doctor": baseline_doctor,
    }
    return summary


def _write_foxglove_notes(config: RunConfig) -> None:
    p4 = config.orchestration.fcu_controller
    _write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P4 FCU controller replay notes",
                "",
                "P4 validates FCU readiness and unique setpoint ownership. It is not a hover completion gate.",
                "",
                "- Fixed frame: `odom` or `map` if SLAM is enabled.",
                f"- FCU state: `{p4.fcu_state_topic}`.",
                f"- Controller status: `{p4.controller_status_topic}`.",
                f"- Owner status: `{p4.owner_status_topic}`.",
                f"- Setpoint intent/output: `{p4.setpoint_intent_topic}`, `{p4.setpoint_output_topic}`.",
                f"- Movement output topic: `{p4.cmd_vel_topic}`.",
                "- Use Raw Messages for controller JSON topics and Plot for local position/rangefinder.",
                "- Do not interpret this bag as P6 SLAM hover completion or exploration completion.",
            ]
        )
        + "\n",
    )


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class FcuControllerDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "fcu-controller-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check P4 FCU controller prerequisites."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_fcu_controller_doctor/{config.run_id}"))
        artifact_dir.mkdir(parents=True, exist_ok=True)
        runtime_config = artifact_dir / "p4_fcu_controller_runtime.toml"
        _write_p4_runtime_config(config, runtime_config)
        console.print("[bold cyan]Checking P4 FCU controller prerequisites[/bold cyan]")
        summary = _build_p4_doctor_summary(config, runtime_config=runtime_config)
        _write_json(artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P4 FCU controller doctor rc={0 if summary['ok'] else 20}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 20


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class FcuControllerAcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "fcu-controller-acceptance"
    TASK_DESCRIPTION: ClassVar[str] = "Run P4 FCU state machine and unique controller acceptance."

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
        bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_p4.yaml"
        model_overlay = config.artifact_dir / "iris_with_lidar_p4_rangefinder_x2.sdf"
        param_overlay = config.artifact_dir / "gazebo-iris-p4-rangefinder.parm"
        sensor_config = config.artifact_dir / "p4_gazebo_sensor_runtime.toml"
        vendor_profile = config.artifact_dir / "x2_vendor_driver_p4.yaml"
        slam_runtime_config = config.artifact_dir / "p4_slam_runtime.toml"
        p4_runtime_config = config.artifact_dir / "p4_fcu_controller_runtime.toml"
        controller_script = config.artifact_dir / "p4_fcu_controller_runtime.py"
        _write_p1_bridge_override(bridge_override)
        _write_p1_vendor_profile(vendor_profile, virtual_serial_link=p2.x2_virtual_serial_link)

        summary: dict[str, Any] | None = None
        try:
            model_overlay_summary = _write_p2_model_overlay(config, model_overlay)
            param_overlay_summary = _write_p2_param_overlay(config, param_overlay)
            sensor_config_summary = _write_p2_sensor_config(config, sensor_config, vendor_profile=vendor_profile)
            slam_runtime_summary = _write_p3_slam_runtime_config(config, slam_runtime_config)
            p4_runtime_summary = _write_p4_runtime_config(config, p4_runtime_config)
            controller_script_summary = _write_controller_runtime_script(
                config,
                controller_script,
                duration_sec=max(35.0, min(duration_sec, 75.0)),
            )

            console.print("[bold cyan]Starting official maze + P4 FCU controller gate[/bold cyan]")
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
            if p4.require_slam_backend:
                _start_p3_slam_container(config, runtime_config=slam_runtime_config)
                time.sleep(4.0)
            _start_p4_rosbag_recording(config, duration_sec=max(55.0, min(duration_sec, 100.0)))
            time.sleep(2.0)
            _start_p4_controller_container(config, script_path=controller_script)
            time.sleep(12.0)

            topic_info = _collect_topic_info(
                config,
                image=baseline.runtime_image,
                topics=(
                    p4.cmd_vel_topic,
                    p4.fcu_state_topic,
                    p4.controller_status_topic,
                    p4.setpoint_intent_topic,
                    p4.setpoint_output_topic,
                    p4.owner_status_topic,
                    p4.time_topic,
                    p4.pose_topic,
                    p4.twist_topic,
                    p4.status_topic,
                    p4.rangefinder_range_topic,
                    p4.rangefinder_status_topic,
                    p4.imu_topic,
                    p4.slam_odom_topic,
                    p4.slam_status_topic,
                    "/scan",
                ),
            )
            graph = _collect_ros_graph(config, config.artifact_dir, image=baseline.runtime_image, network="host")
            probe = _collect_official_dds_probe(config, config.artifact_dir, image=baseline.runtime_image, network="host")
            rangefinder_probe = _collect_rangefinder_probe(config, image=baseline.runtime_image)
            imu_probe = _collect_imu_probe(config, image=baseline.runtime_image)
            controller_summary = _wait_for_controller_summary(config, timeout_sec=max(45.0, duration_sec + 20.0))
            rosbag_profile = _finish_p4_rosbag_recording(config)
            counts = _message_counts(config)
            doctor = _build_p4_doctor_summary(config, runtime_config=p4_runtime_config)

            blockers: list[str] = []
            if not doctor.get("ok"):
                blockers.extend(str(item) for item in doctor.get("blockers", []))
            if not probe.get("result", {}).get("time_received"):
                blockers.append("official DDS probe did not receive /ap/v1/time")
            if not probe.get("result", {}).get("prearm_service_available"):
                blockers.append("official DDS probe did not find /ap/v1/prearm_check service")
            if p4.control_route == "official_dds" and probe.get("result", {}).get("prearm_success") is not True:
                blockers.append("official DDS /ap/v1/prearm_check service call did not return success")
            if not rangefinder_probe.get("result", {}).get("range_received"):
                blockers.append("P4 did not receive rangefinder")
            if not imu_probe.get("result", {}).get("received"):
                blockers.append("P4 did not receive IMU")
            _append_controller_blockers(blockers=blockers, controller=controller_summary)
            owner_summary = controller_summary.get("owner", {}) if controller_summary else {}
            cmd_vel_publishers = topic_info.get(p4.cmd_vel_topic, {}).get("publisher_nodes", [])
            _append_owner_blockers(
                blockers=blockers,
                owner_summary=owner_summary,
                cmd_vel_publishers=cmd_vel_publishers,
                p4=p4,
            )
            if p4.require_slam_backend:
                slam_publishers = topic_info.get(p4.slam_odom_topic, {}).get("publisher_nodes", [])
                if "navlab_cartographer_adapter_node" not in slam_publishers:
                    blockers.append("P4 required SLAM backend is not publishing /slam/odom")
            if not rosbag_profile.get("ok"):
                blockers.append("P4 rosbag profile did not pass")
            for topic in (
                p4.fcu_state_topic,
                p4.controller_status_topic,
                p4.setpoint_intent_topic,
                p4.setpoint_output_topic,
                p4.owner_status_topic,
            ):
                if counts.get(topic, 0) <= 0:
                    blockers.append(f"{topic} was not recorded")

            setpoint_flow = {
                "intent_topic": p4.setpoint_intent_topic,
                "output_topic": p4.setpoint_output_topic,
                "movement_output_topic": p4.cmd_vel_topic,
                "intent_count": controller_summary.get("counts", {}).get("intent", 0) if controller_summary else 0,
                "output_count": controller_summary.get("counts", {}).get("output", 0) if controller_summary else 0,
                "cmd_vel_count": controller_summary.get("counts", {}).get("cmd_vel", 0) if controller_summary else 0,
                "rejected_before_ready": (
                    controller_summary.get("counts", {}).get("rejected_before_ready", 0)
                    if controller_summary
                    else 0
                ),
                "cmd_vel_publishers": cmd_vel_publishers,
                "uses_gazebo_truth": False,
                "uses_slam_for_planning": False,
            }
            fcu_state = {
                "time_received": bool(controller_summary.get("counts", {}).get("time", 0) > 0)
                if controller_summary
                else False,
                "prearm_ok": bool(controller_summary.get("command_results", {}).get("prearm_check", {}).get("success"))
                if controller_summary
                else False,
                "guided_ok": bool(controller_summary.get("command_results", {}).get("set_guided", {}).get("success"))
                if controller_summary
                else False,
                "arm_ok": bool(controller_summary.get("command_results", {}).get("arm", {}).get("success"))
                if controller_summary
                else False,
                "takeoff_ok": bool(controller_summary.get("command_results", {}).get("takeoff", {}).get("success"))
                if controller_summary
                else False,
                "local_position_ready": bool(controller_summary.get("counts", {}).get("pose", 0) > 0)
                if controller_summary
                else False,
                "rangefinder_ready": bool(controller_summary.get("counts", {}).get("range", 0) > 0)
                if controller_summary
                else False,
                "imu_ready": bool(controller_summary.get("counts", {}).get("imu", 0) > 0)
                if controller_summary
                else False,
            }
            p4_summary = {
                "official_bringup": True,
                "control_route": p4.control_route,
                "official_control_claim": p4.control_route == "official_dds",
                "mavlink_bootstrap_claim": p4.control_route == "mavlink_bootstrap_plus_dds_cmd_vel",
                "mavlink_bootstrap_endpoint": p4.mavlink_bootstrap_endpoint,
                "runtime_config": p4_runtime_summary,
                "controller_script": controller_script_summary,
                "controller_runtime": controller_summary,
                "model_overlay": model_overlay_summary,
                "param_overlay": param_overlay_summary,
                "sensor_config": sensor_config_summary,
                "slam_runtime_config": slam_runtime_summary if p4.require_slam_backend else {},
                "owner": owner_summary,
                "set_pose_count": owner_summary.get("set_pose_count"),
                "hover_claim": p4.hover_claim,
                "exploration_claim": p4.exploration_claim,
                "slam_hover_claim": "not_evaluated",
                "rosbag_path": str(config.artifact_dir / "rosbag"),
                "mcap_path": str(config.artifact_dir / "rosbag" / "rosbag_0.mcap"),
            }
            summary = {
                "ok": not blockers,
                "blocked": bool(blockers),
                "blockers": blockers,
                "p4_fcu_controller": p4_summary,
                "fcu_state": fcu_state,
                "setpoint_owner": owner_summary,
                "setpoint_flow": setpoint_flow,
                "official_dds_probe": probe,
                "rangefinder": rangefinder_probe.get("result", {}),
                "imu": imu_probe.get("result", {}),
                "topic_info": topic_info,
                "ros_graph": graph,
                "message_counts": counts,
                "rosbag_profile": rosbag_profile,
                "hover_claim": p4.hover_claim,
                "exploration_claim": p4.exploration_claim,
            }
            _write_json(config.artifact_dir / "summary.json", summary)
            _write_foxglove_notes(config)
        finally:
            host._capture_official_baseline_log(config=config)
            _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_tail.log")
            _capture_container_log(config, container=SLAM_BACKEND_CONTAINER, output_name="slam_backend_tail.log")
            _capture_container_log(config, container=P4_CONTROLLER_CONTAINER, output_name="fcu_controller_tail.log")
            _capture_container_log(config, container=P4_ROSBAG_CONTAINER, output_name="rosbag_tail.log")
            _remove_container(P4_ROSBAG_CONTAINER)
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
                "blockers": ["P4 FCU controller acceptance did not produce a summary"],
            }
            _write_json(config.artifact_dir / "summary.json", summary)
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]P4 FCU controller acceptance completed rc={0 if summary['ok'] else 30}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 30
