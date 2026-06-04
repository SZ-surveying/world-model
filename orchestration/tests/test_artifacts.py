from __future__ import annotations

import json

import tomllib
from src.artifacts import finalize_navlab_artifact


def test_finalize_navlab_artifact_writes_valid_toml(tmp_path) -> None:
    session_log_dir = tmp_path / "session_logs"
    session_log_dir.mkdir()
    (session_log_dir / "sitl.log").write_text("Starting ArduPilot SITL\n", encoding="utf-8")
    (session_log_dir / "router.log").write_text("Starting mavlink-router\n", encoding="utf-8")
    (tmp_path / "summary.json").write_text(
        json.dumps(
            {
                "ok": True,
                "lidar_chain": {
                    "scan_count": 7,
                    "scan_ideal_count": 10,
                    "x2_status_count": 2,
                    "x2_status_fresh": True,
                },
                "observation_mode": {
                    "pose_mirror_observation_only": True,
                },
            }
        ),
        encoding="utf-8",
    )
    (tmp_path / "rosbag_profile_summary.json").write_text(
        json.dumps({"ok": True, "message_counts": {"/scan": 7, "/scan_ideal": 10, "/sim/x2/status": 2}}),
        encoding="utf-8",
    )

    finalize_navlab_artifact(
        artifact_dir=tmp_path,
        session_id="navlab_test",
        run_id="20260603_000000",
        duration_sec=90.0,
        ros_domain_id="85",
        rosbag_profile="profiles/navlab-rosbag-topics.txt",
        session_log_dir=session_log_dir,
    )

    run_config = tomllib.loads((tmp_path / "run_config.toml").read_text(encoding="utf-8"))
    assert run_config["run"]["session_id"] == "navlab_test"
    assert run_config["run"]["duration_sec"] == 90.0
    assert run_config["inputs"]["rosbag_profile"] == "profiles/navlab-rosbag-topics.txt"
    assert run_config["inputs"]["control_authority"] == "sitl_mavlink_only"
    assert run_config["inputs"]["gazebo_direct_set_pose"] is False
    assert (tmp_path / "sitl.log").read_text(encoding="utf-8") == "Starting ArduPilot SITL\n"
    assert (tmp_path / "router.log").read_text(encoding="utf-8") == "Starting mavlink-router\n"

    summary_md = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "# NavLab Companion SITL Gazebo obstacle acceptance" in summary_md
    assert "- Result: `PASS`" in summary_md
    assert f"- SITL log: `{tmp_path / 'sitl.log'}`" in summary_md
    assert "- Lidar chain: `/scan=7`, `/scan_ideal=10`, `/sim/x2/status=2`" in summary_md
    assert "- Pose mirror observation-only: `PASS`" in summary_md
