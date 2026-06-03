from __future__ import annotations

import tomllib

from lab_env.navlab.orchestration.artifacts import finalize_navlab_artifact


def test_finalize_navlab_artifact_writes_valid_toml(tmp_path) -> None:
    finalize_navlab_artifact(
        artifact_dir=tmp_path,
        session_id="navlab_test",
        run_id="20260603_000000",
        duration_sec=90.0,
        ros_domain_id="85",
        rosbag_profile="profiles/navlab-rosbag-topics.txt",
    )

    run_config = tomllib.loads((tmp_path / "run_config.toml").read_text(encoding="utf-8"))
    assert run_config["run"]["session_id"] == "navlab_test"
    assert run_config["run"]["duration_sec"] == 90.0
    assert run_config["inputs"]["rosbag_profile"] == "profiles/navlab-rosbag-topics.txt"

    summary_md = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "# NavLab Companion SITL Gazebo obstacle acceptance" in summary_md
    assert "- Result: `UNKNOWN`" in summary_md
