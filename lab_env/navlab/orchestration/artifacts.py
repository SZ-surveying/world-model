from __future__ import annotations

import json
import os
import shutil
from pathlib import Path
from typing import Any

import tomli_w
from jinja2 import Environment, FileSystemLoader, Template, select_autoescape

TEMPLATE_DIR = Path(__file__).parent / "templates"


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _copy_if_exists(source: Path, target: Path) -> None:
    if source.exists() and source.resolve() != target.resolve():
        shutil.copyfile(source, target)


def _status(value: bool | None) -> str:
    if value is True:
        return "PASS"
    if value is False:
        return "FAIL"
    return "UNKNOWN"


def _summarize_counts(counts: dict[str, Any], limit: int = 16) -> str:
    if not counts:
        return "none"
    parts = [f"`{topic}={count}`" for topic, count in sorted(counts.items())[:limit]]
    if len(counts) > limit:
        parts.append(f"... +{len(counts) - limit} more")
    return ", ".join(parts)


def _json_pretty(value: Any) -> str:
    return json.dumps(value, indent=2, sort_keys=True)


def _summary_template() -> Template:
    environment = Environment(
        autoescape=select_autoescape(disabled_extensions=("md", "j2"), default_for_string=False, default=False),
        loader=FileSystemLoader(TEMPLATE_DIR),
        lstrip_blocks=True,
        trim_blocks=True,
    )
    environment.filters["status"] = _status
    environment.filters["summarize_counts"] = _summarize_counts
    environment.filters["json_pretty"] = _json_pretty
    return environment.get_template("summary.md.j2")


def finalize_navlab_artifact(
    *,
    artifact_dir: Path,
    session_id: str,
    run_id: str,
    duration_sec: float,
    ros_domain_id: str,
    rosbag_profile: str,
) -> None:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    _copy_if_exists(artifact_dir / "navlab_stack_tail.log", artifact_dir / "sitl.log")
    _copy_if_exists(artifact_dir / "navlab_stack_tail.log", artifact_dir / "external_nav_bridge.log")
    with (artifact_dir / "run_config.toml").open("wb") as output:
        tomli_w.dump(
            {
                "run": {
                    "session_id": session_id,
                    "run_id": run_id,
                    "artifact_dir": str(artifact_dir),
                    "stage_id": "NavLab",
                    "stage_label": "Companion SITL Gazebo obstacle acceptance",
                    "stage_gate": "NavLab",
                    "duration_sec": duration_sec,
                    "ros_domain_id": ros_domain_id,
                },
                "inputs": {
                    "odom_source": "gazebo_scan_fcu_imu_cartographer",
                    "control_mode": "companion_mavlink_hover_forward_avoid",
                    "rosbag_profile": rosbag_profile,
                },
                "host": {
                    "cwd": os.getcwd(),
                },
                "outputs": {
                    "summary_json": str(artifact_dir / "summary.json"),
                    "summary_md": str(artifact_dir / "summary.md"),
                    "rosbag": str(artifact_dir / "rosbag" / "rosbag_0.mcap"),
                    "sitl_log": str(artifact_dir / "sitl.log"),
                    "external_nav_bridge_log": str(artifact_dir / "external_nav_bridge.log"),
                },
            },
            output,
        )
    _write_summary_md(artifact_dir)


def _write_summary_md(artifact_dir: Path) -> None:
    summary = _load_json(artifact_dir / "summary.json")
    rosbag_profile = _load_json(artifact_dir / "rosbag_profile_summary.json")
    sections = [
        (title, summary.get(section_key, {}) if summary else {})
        for section_key, title in (
            ("mission_summary", "Mission"),
            ("imu_status", "IMU"),
            ("external_nav_status", "ExternalNav"),
            ("cartographer_status", "Cartographer"),
            ("x2_status", "X2 Sensor"),
            ("scan_publisher", "Scan Publisher"),
        )
    ]
    rendered = _summary_template().render(
        artifact_dir=artifact_dir,
        summary=summary,
        rosbag_profile=rosbag_profile,
        summary_ok=summary.get("ok") if summary else None,
        rosbag_ok=rosbag_profile.get("ok") if rosbag_profile else None,
        message_counts=rosbag_profile.get("message_counts", {}) if rosbag_profile else {},
        zero_count_required=rosbag_profile.get("zero_count_required_topics", []) if rosbag_profile else [],
        sections=sections,
    )
    (artifact_dir / "summary.md").write_text(rendered.rstrip() + "\n", encoding="utf-8")
