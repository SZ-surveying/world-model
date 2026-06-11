from __future__ import annotations

from typing import Any

from src.configs.run_config import RunConfig
from src.tasks.helpers.artifacts import write_text
from src.tasks.helpers.rosbag_profiles import load_rosbag_metadata_counts

P10_ROSBAG_CONTAINER = "navlab-p10-rosbag"


def _message_counts(config: RunConfig) -> dict[str, int]:
    metadata = config.artifact_dir / "rosbag" / "metadata.yaml"
    if not metadata.is_file():
        return {}
    return load_rosbag_metadata_counts(metadata)


def motor_output_summary(*, ros_graph: dict[str, Any]) -> dict[str, Any]:
    topics = ros_graph.get("ros2_topic_list", {}).get("lines", [])
    motor_keywords = ("motor", "servo", "actuator", "esc", "rpm", "pwm")
    excluded = {"/robot_description"}
    candidates = sorted(
        topic
        for topic in topics
        if topic not in excluded
        if any(keyword in topic.lower() for keyword in motor_keywords) and "support_motor" not in topic.lower()
    )
    if not candidates:
        return {
            "motor_output_claim": "not_available",
            "available": False,
            "candidate_topics": [],
            "motor_pwm_min": None,
            "motor_pwm_max": None,
            "motor_pwm_spread": None,
            "motor_rpm_min": None,
            "motor_rpm_max": None,
            "motor_rpm_spread": None,
            "motor_thrust_bias_estimate": None,
            "reason": "no motor/servo/actuator/ESC output topic is exposed in the ROS graph",
        }
    return {
        "motor_output_claim": "candidate_topics_present",
        "available": False,
        "candidate_topics": candidates,
        "motor_pwm_min": None,
        "motor_pwm_max": None,
        "motor_pwm_spread": None,
        "motor_rpm_min": None,
        "motor_rpm_max": None,
        "motor_rpm_spread": None,
        "motor_thrust_bias_estimate": None,
        "reason": "candidate topics exist, but P10.1 does not parse motor output message schemas yet",
    }


def _write_foxglove_notes(config: RunConfig) -> None:
    p10 = config.orchestration.scan_integrity_gate
    write_text(
        config.artifact_dir / "foxglove_notes.md",
        "\n".join(
            [
                "# NavLab P10 scan integrity replay notes",
                "",
                "P10 validates body-fixed 2D lidar scan integrity before real-machine flight.",
                "",
                "- Fixed frame: `map`.",
                f"- Raw scan: `{p10.raw_scan_topic}`.",
                f"- Normalized scan: `{p10.normalized_scan_topic}`.",
                f"- Validated SLAM scan: `{p10.validated_scan_topic}`.",
                f"- Scan integrity status: `{p10.status_topic}`.",
                f"- Attitude source: `{p10.attitude_source_topic}`.",
                "- Gazebo truth is diagnostic only and is not an integrity input.",
            ]
        )
        + "\n",
    )
