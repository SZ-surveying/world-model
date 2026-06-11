from __future__ import annotations

from pathlib import Path
from typing import Any

from src.configs.run_config import RunConfig
from src.tasks.helpers.artifacts import file_sha256, write_text
from src.tasks.helpers.rosbag_profiles import profile_topics
from src.tasks.helpers.slam_hover import (
    build_p6_doctor_summary,
)

P7_ROSBAG_CONTAINER = "navlab-p7-rosbag"

def build_p7_doctor_summary(
    config: RunConfig,
    *,
    runtime_config: Path,
    include_dependencies: bool = True,
) -> dict[str, Any]:
    p7 = config.orchestration.motion_gate
    p6_runtime = config.artifact_dir / "p7_doctor_p6_slam_hover_runtime.toml"
    p6_doctor = (
        build_p6_doctor_summary(config, runtime_config=p6_runtime)
        if include_dependencies
        else {"ok": True, "blockers": [], "skipped": "acceptance already launched P6 prerequisites"}
    )
    profile_path = Path(p7.rosbag_profile)
    required, optional, topics = profile_topics(profile_path)
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
            "runtime_config_sha256": file_sha256(runtime_config) if runtime_config.is_file() else "",
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


def _write_foxglove_notes(config: RunConfig) -> None:
    p7 = config.orchestration.motion_gate
    write_text(
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
