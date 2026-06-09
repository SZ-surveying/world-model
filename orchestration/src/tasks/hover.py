from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

import tomli_w
from navlab.gazebo_sensor.airframe_disturbance import (
    AirframeDisturbanceProfile,
    apply_profile_to_iris_sdf,
    estimate_disturbance_metrics,
    profile_from_library,
)
from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.artifacts import finalize_navlab_artifact
from src.config import RunConfig, load_task_invocation_config
from src.foxglove_upload import upload_acceptance_rosbag
from src.tasks.base import OrchestrationTask
from src.tasks.helpers.artifacts import _write_json
from src.tasks.helpers.fcu import (
    P4_CONTROLLER_CONTAINER,
    _append_controller_blockers,
    _append_owner_blockers,
    _start_p4_controller_container,
    _wait_for_controller_summary,
    _write_controller_runtime_script,
    _write_p4_runtime_config,
)
from src.tasks.helpers.landing import apply_landing_gate
from src.tasks.helpers.navlab_models import (
    GAZEBO_SENSOR_CONTAINER,
    OFFICIAL_IRIS_3D_BRIDGE_CONFIG,
    _capture_container_log,
    _collect_topic_info,
    _file_sha256,
    _remove_container,
    _start_gazebo_sensor_container,
    _write_p1_bridge_override,
    _write_p1_vendor_profile,
)
from src.tasks.helpers.official_stack import _collect_official_dds_probe, _collect_ros_graph, _write_text
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
    P6_ROSBAG_CONTAINER,
    _append_p6_blockers,
    _baseline_env,
    _build_p6_doctor_summary,
    _finish_p6_rosbag_recording,
    _message_counts,
    _run_hover_probe,
    _start_p6_rosbag_recording,
    _start_p6_vehicle_marker_container,
    _write_foxglove_notes,
    _write_hover_probe_script,
    _write_p6_runtime_config,
)
from src.tasks.registry import TaskRegistry

SUPPORTED_SIMULATION_PROFILES = ("ideal", "mild_disturbance")
MILD_DISTURBANCE_AIRFRAME_PROFILE = "mild_bias"


def _landing_budget_sec(config: RunConfig) -> float:
    landing = config.orchestration.landing
    return landing.pre_land_hold_sec + landing.max_landing_duration_sec + 20.0


def _normalize_simulation_profile(value: str) -> str:
    profile = value.strip().lower()
    if profile not in SUPPORTED_SIMULATION_PROFILES:
        supported = ", ".join(SUPPORTED_SIMULATION_PROFILES)
        raise ValueError(f"unsupported hover simulation profile {value!r}; supported: {supported}")
    return profile


def _airframe_profile_for_simulation_profile(simulation_profile: str) -> AirframeDisturbanceProfile | None:
    if simulation_profile == "mild_disturbance":
        return profile_from_library(MILD_DISTURBANCE_AIRFRAME_PROFILE)
    return None


def _hover_airframe_runtime_config(config: RunConfig, *, profile: AirframeDisturbanceProfile) -> dict[str, Any]:
    p12 = config.orchestration.airframe_disturbance
    return {
        "enabled": True,
        "profile": profile.name,
        "injection_layer": p12.injection_layer,
        "seed": str(profile.seed),
        "motor_count": str(profile.motor_count),
        "thrust_multipliers": ",".join(str(value) for value in profile.thrust_multipliers),
        "esc_lag_ms": ",".join(str(value) for value in profile.esc_lag_ms),
        "esc_lag_model": p12.esc_lag_model,
        "thrust_noise_std": profile.thrust_noise_std,
        "thrust_noise_correlation_ms": profile.thrust_noise_correlation_ms,
        "motor_jitter_hz": profile.motor_jitter_hz,
        "imu_vibration_enabled": profile.imu_vibration_enabled,
        "imu_input_topic": p12.imu_input_topic,
        "imu_output_topic": p12.imu_output_topic,
        "imu_gyro_noise_std_dps": profile.imu_gyro_noise_std_dps,
        "imu_accel_noise_std_mps2": profile.imu_accel_noise_std_mps2,
        "imu_vibration_freq_hz": profile.imu_vibration_freq_hz,
        "imu_vibration_roll_pitch_amp_deg": profile.imu_vibration_roll_pitch_amp_deg,
        "status_topic": p12.status_topic,
        "events_topic": p12.events_topic,
    }


def _write_hover_model_overlay(
    config: RunConfig,
    path: Path,
    *,
    airframe_profile: AirframeDisturbanceProfile | None,
) -> dict[str, Any]:
    base_summary = _write_p2_model_overlay(config, path)
    if airframe_profile is None:
        return base_summary
    rendered, disturbance_summary = apply_profile_to_iris_sdf(path.read_text(encoding="utf-8"), airframe_profile)
    path.write_text(rendered, encoding="utf-8")
    return {
        **base_summary,
        "overlay_sha256": _file_sha256(path),
        "airframe_disturbance": disturbance_summary,
        "disturbance_injection_layer": config.orchestration.airframe_disturbance.injection_layer,
        "esc_lag_claim": config.orchestration.airframe_disturbance.esc_lag_model,
    }


def _write_hover_sensor_config(
    config: RunConfig,
    path: Path,
    *,
    vendor_profile: Path,
    airframe_profile: AirframeDisturbanceProfile | None,
) -> dict[str, Any]:
    summary = _write_p2_sensor_config(config, path, vendor_profile=vendor_profile)
    if airframe_profile is None:
        return summary
    data = summary["data"]
    data.setdefault("gazebo_sensor", {})["airframe_disturbance"] = _hover_airframe_runtime_config(
        config,
        profile=airframe_profile,
    )
    path.write_bytes(tomli_w.dumps(data).encode("utf-8"))
    return {"path": str(path), "workspace_path": host._workspace_path(path), "sha256": _file_sha256(path), "data": data}


def _publish_hover_landing_intent(config: RunConfig) -> dict[str, Any]:
    landing = config.orchestration.landing
    p4 = config.orchestration.fcu_controller
    payload = {
        "source": "navlab_hover_acceptance",
        "target_owner": p4.owner_name,
        "target_owner_id": p4.owner_id,
        "kind": "land_in_place",
        "policy": "land_in_place",
        "home_required": False,
        "reason": "hover_stage1_complete",
        "updated_ms": int(time.time() * 1000),
    }
    script = f"""
import json
import time

import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("navlab_hover_landing_intent_publisher")
pub = node.create_publisher(String, {landing.landing_intent_topic!r}, 10)
payload = json.loads({json.dumps(payload, sort_keys=True)!r})
msg = String()
msg.data = json.dumps(payload, sort_keys=True)
for _ in range(20):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.02)
    time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
print(msg.data)
"""
    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image=config.orchestration.official_baseline.runtime_image,
        shell_command=f"python3 - <<'PY'\n{script}\nPY",
        name=None,
        network="host",
        envs=_baseline_env(config),
    )
    _write_text(config.artifact_dir / "hover_landing_intent.txt", output)
    return {"ok": rc == 0, "rc": rc, "topic": landing.landing_intent_topic, "payload": payload, "output": output}


def _write_hover_runtime_diff(config: RunConfig, *, latest_failed_artifact: str = "") -> dict[str, Any]:
    p6 = config.orchestration.slam_hover
    p4 = config.orchestration.fcu_controller
    diff = {
        "baseline": "P12/P6 successful runtime contract",
        "reference_artifact": "artifacts/ros/navlab_companion_sitl_gazebo/20260608_200320/p12_live_clean_replay/summary.json",
        "latest_failed_legacy_hover_artifact": latest_failed_artifact,
        "legacy_hover_missing_items": [
            "P4 FCU controller status on /navlab/fcu/controller/status",
            "official DDS pose/status on /ap/v1/pose/filtered and /ap/v1/status",
            "SLAM odometry input/output continuity while Cartographer use_odometry=true",
            "unique setpoint owner diagnostics",
            "landing intent/status topics recorded in the same acceptance bag",
        ],
        "current_hover_contract": {
            "control_route": p4.control_route,
            "fcu_pose_topic": p6.fcu_pose_topic,
            "slam_odom_topic": p6.slam_odom_topic,
            "external_nav_status_topic": p6.external_nav_status_topic,
            "controller_status_topic": p6.controller_status_topic,
            "landing_policy": config.orchestration.landing.policy_for_task("hover"),
            "uses_gazebo_truth_as_input": p6.uses_gazebo_truth_as_input,
        },
    }
    _write_json(config.artifact_dir / "hover_p12_runtime_contract_diff.json", diff)
    return diff


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class HoverAcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "hover"
    TASK_DESCRIPTION: ClassVar[str] = "Run NavLab P12-aligned FCU/SLAM hover + land acceptance."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        task_config_path: str | Path | None = None,
        duration_sec: float | None = None,
        simulation_profile: str | None = None,
        console: Console | None = None,
    ) -> int:
        console = console or Console()
        task_config = load_task_invocation_config(
            "hover",
            task_config_path=task_config_path,
            cli_duration_sec=duration_sec,
            default_duration_sec=90.0,
            cli_simulation_profile=simulation_profile,
            default_simulation_profile="ideal",
        )
        duration_sec = task_config.duration_sec
        simulation_profile = _normalize_simulation_profile(task_config.simulation_profile)
        airframe_profile = _airframe_profile_for_simulation_profile(simulation_profile)
        config = RunConfig.from_config(
            config_path=config_path,
            task_name="hover",
            task_config_path=task_config_path,
            duration_sec=duration_sec,
        )
        config.artifact_dir.mkdir(parents=True, exist_ok=True)
        host._render_run_config(console, config)
        baseline = config.orchestration.official_baseline
        p2 = config.orchestration.rangefinder_imu
        p3 = config.orchestration.slam_backend
        p4 = config.orchestration.fcu_controller
        p6 = config.orchestration.slam_hover
        bridge_override = config.artifact_dir / "official_iris_3Dlidar_bridge_hover.yaml"
        model_overlay = config.artifact_dir / "iris_with_lidar_hover_rangefinder_x2.sdf"
        param_overlay = config.artifact_dir / "gazebo-iris-hover-rangefinder.parm"
        sensor_config = config.artifact_dir / "hover_gazebo_sensor_runtime.toml"
        vendor_profile = config.artifact_dir / "x2_vendor_driver_hover.yaml"
        slam_runtime_config = config.artifact_dir / "hover_slam_runtime.toml"
        p4_runtime_config = config.artifact_dir / "hover_fcu_controller_runtime.toml"
        p6_runtime_config = config.artifact_dir / "hover_slam_hover_runtime.toml"
        controller_script = config.artifact_dir / "hover_fcu_controller_runtime.py"
        hover_probe_script = config.artifact_dir / "hover_slam_hover_probe.py"
        _write_p1_bridge_override(bridge_override)
        _write_p1_vendor_profile(vendor_profile, virtual_serial_link=p2.x2_virtual_serial_link)

        summary: dict[str, Any] | None = None
        try:
            model_overlay_summary = _write_hover_model_overlay(
                config,
                model_overlay,
                airframe_profile=airframe_profile,
            )
            param_overlay_summary = _write_p2_param_overlay(config, param_overlay)
            sensor_config_summary = _write_hover_sensor_config(
                config,
                sensor_config,
                vendor_profile=vendor_profile,
                airframe_profile=airframe_profile,
            )
            slam_runtime_summary = _write_p3_slam_runtime_config(config, slam_runtime_config)
            p4_runtime_summary = _write_p4_runtime_config(config, p4_runtime_config)
            p6_runtime_summary = _write_p6_runtime_config(config, p6_runtime_config)
            landing_budget_sec = _landing_budget_sec(config)
            controller_script_summary = _write_controller_runtime_script(
                config,
                controller_script,
                duration_sec=max(120.0, duration_sec + landing_budget_sec + 60.0),
                hold_after_ready_sec=max(90.0, duration_sec + landing_budget_sec),
                enable_motion_intent_control=False,
                enable_landing_intent_control=True,
                hover_status_topic=p6.hover_status_topic,
            )
            hover_probe_script_summary = _write_hover_probe_script(config, hover_probe_script)
            runtime_diff = _write_hover_runtime_diff(
                config,
                latest_failed_artifact="artifacts/ros/navlab_companion_sitl_gazebo/20260609_101615/summary.json",
            )

            console.print(
                "[bold cyan]Starting P12-aligned official FCU/SLAM hover gate "
                f"profile={simulation_profile}[/bold cyan]"
            )
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
                artifact_name="hover_rangefinder_preflight_probe.txt",
            )
            if not rangefinder_preflight.get("result", {}).get("range_received"):
                console.print("[yellow]Hover rangefinder preflight missed data; restarting gazebo sensor once[/yellow]")
                _capture_container_log(config, container=GAZEBO_SENSOR_CONTAINER, output_name="gazebo_sensor_preflight_tail.log")
                _start_gazebo_sensor_container(config, sensor_config=sensor_config)
                time.sleep(8.0)
                rangefinder_preflight = _collect_rangefinder_probe(
                    config,
                    image=baseline.runtime_image,
                    artifact_name="hover_rangefinder_preflight_retry_probe.txt",
                )
            _start_p3_slam_container(config, runtime_config=slam_runtime_config)
            time.sleep(4.0)
            _start_p6_vehicle_marker_container(config)
            _start_p6_rosbag_recording(config, duration_sec=max(120.0, min(duration_sec + landing_budget_sec + 30.0, 240.0)))
            time.sleep(2.0)
            _start_p4_controller_container(config, script_path=controller_script)
            hover_summary = _run_hover_probe(config, script_path=hover_probe_script)
            landing_intent = _publish_hover_landing_intent(config)
            controller_summary = _wait_for_controller_summary(config, timeout_sec=landing_budget_sec + 20.0)
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
                artifact_name="hover_slam_odom_probe.txt",
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
                    p4.cmd_vel_topic,
                    p6.rangefinder_range_topic,
                    p6.rangefinder_status_topic,
                    p6.imu_topic,
                    p6.controller_status_topic,
                    p6.setpoint_intent_topic,
                    p6.setpoint_output_topic,
                    p6.owner_status_topic,
                    p6.hover_status_topic,
                    config.orchestration.landing.landing_status_topic,
                    config.orchestration.landing.landing_intent_topic,
                    config.orchestration.airframe_disturbance.status_topic,
                ),
                transient_topics=(
                    p6.fcu_pose_topic,
                    p6.fcu_twist_topic,
                    p4.cmd_vel_topic,
                    p6.controller_status_topic,
                    p6.setpoint_intent_topic,
                    p6.setpoint_output_topic,
                    p6.owner_status_topic,
                    p6.hover_status_topic,
                    config.orchestration.landing.landing_status_topic,
                    config.orchestration.landing.landing_intent_topic,
                    config.orchestration.airframe_disturbance.status_topic,
                ),
            )
            doctor = _build_p6_doctor_summary(config, runtime_config=p6_runtime_config, include_dependencies=False)
            blockers: list[str] = []
            if not doctor.get("ok"):
                blockers.extend(str(item) for item in doctor.get("blockers", []))
            if not probe.get("result", {}).get("time_received"):
                blockers.append("official DDS probe did not receive /ap/v1/time")
            if not rangefinder_preflight.get("result", {}).get("range_received"):
                blockers.append("hover rangefinder preflight did not receive range data")
            if not rangefinder_probe.get("result", {}).get("range_received"):
                blockers.append("hover did not receive rangefinder")
            if not imu_probe.get("result", {}).get("received"):
                blockers.append("hover did not receive IMU")
            _append_slam_odom_quality_blockers(
                blockers=blockers,
                p3=p3,
                slam_odom_result=slam_odom_probe.get("result", {}),
            )
            _append_controller_blockers(blockers=blockers, controller=controller_summary)
            owner_summary = hover_summary.get("owner", {}) if hover_summary else {}
            if not owner_summary and controller_summary:
                owner_summary = controller_summary.get("owner", {})
            cmd_vel_publishers = topic_info.get(p4.cmd_vel_topic, {}).get("publisher_nodes", [])
            _append_owner_blockers(
                blockers=blockers,
                owner_summary=owner_summary,
                cmd_vel_publishers=cmd_vel_publishers,
                p4=p4,
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
                config.orchestration.landing.landing_status_topic,
            ):
                if counts.get(topic, 0) <= 0:
                    blockers.append(f"{topic} was not recorded")
            if not landing_intent.get("ok"):
                blockers.append("hover landing intent publisher failed")
            airframe_disturbance_summary: dict[str, Any] = {
                "enabled": airframe_profile is not None,
                "simulation_profile": simulation_profile,
                "profile": airframe_profile.to_summary() if airframe_profile is not None else None,
                "estimated_metrics": (
                    estimate_disturbance_metrics(airframe_profile) if airframe_profile is not None else None
                ),
                "status_topic": config.orchestration.airframe_disturbance.status_topic,
                "status_count": counts.get(config.orchestration.airframe_disturbance.status_topic, 0),
            }
            if airframe_profile is not None and airframe_disturbance_summary["status_count"] <= 0:
                blockers.append(f"{config.orchestration.airframe_disturbance.status_topic} was not recorded")

            summary = {
                "ok": not blockers,
                "blocked": bool(blockers),
                "blockers": blockers,
                "acceptance_stage": "simulation",
                "simulation_profile": simulation_profile,
                "config_sources": config.config_sources_summary(),
                "task_parameters": task_config.to_summary(),
                "hover_gate": {
                    "runtime_config": p6_runtime_summary,
                    "hover_probe_script": hover_probe_script_summary,
                    "hover_probe": hover_summary,
                    "model_overlay": model_overlay_summary,
                    "param_overlay": param_overlay_summary,
                    "sensor_config": sensor_config_summary,
                    "slam_runtime_config": slam_runtime_summary,
                    "p4_runtime_config": p4_runtime_summary,
                    "controller_script": controller_script_summary,
                    "controller_runtime": controller_summary,
                    "landing_intent": landing_intent,
                    "external_nav_input_topic": p6.slam_odom_topic,
                    "uses_gazebo_truth_as_input": p6.uses_gazebo_truth_as_input,
                    "rosbag_path": str(config.artifact_dir / "rosbag"),
                    "mcap_path": str(config.artifact_dir / "rosbag" / "rosbag_0.mcap"),
                    "runtime_contract_diff": runtime_diff,
                    "airframe_disturbance": airframe_disturbance_summary,
                },
                "airframe_disturbance": airframe_disturbance_summary,
                "p6_hover_prerequisite": hover_summary.get("p6_slam_hover", {}),
                "hover": hover_summary.get("hover", {}),
                "fcu": hover_summary.get("fcu", {}),
                "slam_odom": hover_summary.get("slam_odom", {}),
                "slam_odom_probe": slam_odom_probe.get("result", {}),
                "external_nav": hover_summary.get("external_nav", {}),
                "owner": hover_summary.get("owner", {}),
                "official_dds_probe": probe,
                "rangefinder_probe": rangefinder_probe.get("result", {}),
                "rangefinder_preflight": rangefinder_preflight.get("result", {}),
                "imu_probe": imu_probe.get("result", {}),
                "topic_info": topic_info,
                "ros_graph": graph,
                "message_counts": counts,
                "rosbag_profile": rosbag_profile,
                "hover_claim": p6.hover_claim,
                "uses_gazebo_truth_as_input": p6.uses_gazebo_truth_as_input,
            }
            landing_summary = controller_summary.get("landing") if isinstance(controller_summary.get("landing"), dict) else None
            summary = apply_landing_gate(summary, config, task_name="hover", landing=landing_summary)
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
                "blockers": ["hover acceptance did not produce a summary"],
                "acceptance_stage": "simulation",
                "simulation_profile": simulation_profile,
                "config_sources": config.config_sources_summary(),
                "task_parameters": task_config.to_summary(),
            }
            summary = apply_landing_gate(summary, config, task_name="hover", landing=None)
            _write_json(config.artifact_dir / "summary.json", summary)
        finalize_navlab_artifact(
            artifact_dir=config.artifact_dir,
            session_id=config.session_id,
            run_id=config.run_id,
            duration_sec=duration_sec,
            ros_domain_id=config.ros_domain_id,
            rosbag_profile=config.slam_hover_rosbag_profile,
            session_log_dir=host._session_log_dir(config),
            stage_label="P12-aligned FCU/SLAM hover + landing acceptance",
            control_mode=f"official_fcu_controller_slam_hover_land_in_place_{simulation_profile}",
        )
        upload = upload_acceptance_rosbag(config)
        upload_color = "green" if upload.ok else "yellow"
        console.print(f"[{upload_color}]Foxglove upload:[/{upload_color}] {upload.state} ({upload.reason})")
        rc = 0 if summary["ok"] else 30
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]NavLab hover acceptance completed rc={rc}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        return rc
