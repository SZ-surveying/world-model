from __future__ import annotations

from navlab.companion.acceptance import (
    MINIMUM_ROSBAG_TOPICS,
    _build_rosbag_topic_plan,
    _record_profile_topics,
    _write_effective_rosbag_profile,
    _write_foxglove_notes,
)


def test_write_foxglove_notes_uses_template(tmp_path) -> None:
    _write_foxglove_notes(artifact_dir=tmp_path)

    notes = (tmp_path / "foxglove_notes.md").read_text(encoding="utf-8")
    assert "# NavLab Foxglove replay notes" in notes
    assert f"- Rosbag: `{tmp_path / 'rosbag'}`" in notes
    assert "`/navlab/replay/markers`" not in notes
    assert "`/navlab/replay/constraint_markers`" not in notes
    assert "`/submap_list`" not in notes
    assert "3D panel fixed frame: `navlab_world`" in notes
    assert "ArduPilot JSON plugin" in notes
    assert "excludes Cartographer custom-schema debug topics" in notes
    assert "wait_ready -> guided -> arm" in notes


def test_navlab_minimum_rosbag_topics_cover_replay_and_feedback() -> None:
    topics = list(MINIMUM_ROSBAG_TOPICS)

    assert "/scan" in topics
    assert "/scan_ideal" in topics
    assert "/sim/x2/status" in topics
    assert "/gazebo/truth/odom" in topics
    assert "/gazebo/truth/status" in topics
    assert "/navlab/fcu/local_position_pose" in topics
    assert "/submap_list" not in topics
    assert "/constraint_list" not in topics
    assert "/trajectory_node_list" not in topics
    assert "/ap/tf" not in topics
    assert "/tf_static" in topics


def test_rosbag_topic_plan_adds_configured_extras(tmp_path) -> None:
    profile = tmp_path / "extra-topics.txt"
    profile.write_text(
        "\n".join(
            [
                "required /map",
                "optional /odom",
                "optional /scan",
                "",
            ]
        ),
        encoding="utf-8",
    )

    topics = _build_rosbag_topic_plan(profile)
    effective_profile = _write_effective_rosbag_profile(
        artifact_dir=tmp_path,
        source_profile=profile,
        topics=topics,
    )
    effective = effective_profile.read_text(encoding="utf-8")

    assert "/scan" in topics
    assert "/map" in topics
    assert "/odom" in topics
    assert topics.count("/scan") == 1
    assert "required /scan" in effective
    assert "required /map" in effective
    assert "optional /odom" in effective
    assert "optional /scan" not in effective


def test_rosbag_record_uses_profile_topic_whitelist(tmp_path) -> None:
    calls = []

    class FakeManager:
        def start_subprocess(self, name, command, *, log_path):  # noqa: ANN001
            calls.append({"name": name, "command": command, "log_path": log_path})
            return object()

    _record_profile_topics(manager=FakeManager(), artifact_dir=tmp_path, topics=["/scan", "/sim/uav_pose"])

    command = calls[0]["command"]
    assert "--topics" in command
    assert "--all-topics" not in command
    assert "/scan" in command
    assert "/sim/uav_pose" in command
    assert "/submap_list" not in command
