from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


def _load_script(name: str, path: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


replay = _load_script("foxglove_replay", "scripts/command/src/foxglove/replay.py")
upload = _load_script("foxglove_upload", "scripts/command/src/foxglove/upload.py")


def test_p9_parse_walls_handles_degree_yaw(tmp_path: Path) -> None:
    sdf = tmp_path / "maze.sdf"
    sdf.write_text(
        """
<sdf version="1.9">
  <world name="maze">
    <model name="maze">
      <link name="Wall_0">
        <pose degrees="true">0 0 0 0 0 90</pose>
        <collision name="collision">
          <geometry><box><size>4 0.2 2</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
""".strip(),
        encoding="utf-8",
    )

    walls = replay.parse_walls(sdf)
    extent = replay.walls_extent(walls)

    assert len(walls) == 1
    assert round(extent.xmax - extent.xmin, 1) == 0.2
    assert round(extent.ymax - extent.ymin, 1) == 4.0


def test_p9_rasterizes_official_maze_overlay() -> None:
    walls = [replay.Wall("wall", 0.0, 0.0, 0.0, 2.0, 0.2, 2.0)]
    overlay = replay.rasterize_walls(walls, replay.BBox(-1.0, -1.0, 1.0, 1.0), resolution_m=0.1)

    assert overlay["header"]["frame_id"] == "map"
    assert overlay["info"]["width"] == 20
    assert overlay["info"]["height"] == 20
    assert any(value == 100 for value in overlay["data"])


def test_p9_auto_crop_unions_slam_and_trajectory_then_clamps() -> None:
    crop, mode = replay.choose_crop_bbox(
        maze_extent=replay.BBox(-10.0, -10.0, 10.0, 10.0),
        map_bbox=replay.BBox(-1.0, -2.0, 2.0, 3.0),
        trajectory_bbox=replay.BBox(5.0, 6.0, 5.5, 6.5),
        start_xy=(0.0, 0.0),
        margin_m=4.0,
        bbox_override=None,
        full=False,
    )

    assert mode == "auto_slam_and_trajectory_bbox"
    assert crop == replay.BBox(-5.0, -6.0, 9.5, 10.0)


def test_p9_replay_quality_marks_minimal_runs() -> None:
    quality = replay.replay_quality_from_summary(
        {
            "coverage": {"path_length_m": 0.96, "known_cell_growth": 100, "estimated_explored_area_m2": 5.0},
            "p8_exploration": {"accepted_goals": 3},
            "safety": {"min_scan_clearance_m": 0.8, "stop_drift_m": 0.05},
        }
    )

    assert quality["publishable"] is False
    assert quality["profile"] == "minimal_run"


def test_p9_replay_quality_accepts_representative_runs() -> None:
    quality = replay.replay_quality_from_summary(
        {
            "replay_profile": "conservative",
            "coverage": {"path_length_m": 2.6, "known_cell_growth": 100, "estimated_explored_area_m2": 8.0},
            "p8_exploration": {"accepted_goals": 5},
            "safety": {"min_scan_clearance_m": 0.8, "stop_drift_m": 0.05},
        }
    )

    assert quality["publishable"] is True
    assert quality["profile"] == "p8_replay_conservative"


def test_p9_replay_quality_preserves_display_profile() -> None:
    quality = replay.replay_quality_from_summary(
        {
            "replay_profile": "display",
            "coverage": {"path_length_m": 6.0, "known_cell_growth": 100, "estimated_explored_area_m2": 8.0},
            "p8_exploration": {"accepted_goals": 6},
            "safety": {"min_scan_clearance_m": 0.8, "stop_drift_m": 0.05},
        }
    )

    assert quality["publishable"] is True
    assert quality["profile"] == "p8_replay_display"


def test_upload_targets_default_to_raw_even_when_foxglove_mcap_exists(tmp_path: Path) -> None:
    run_dir = tmp_path / "20260607_185314"
    (run_dir / "rosbag").mkdir(parents=True)
    (run_dir / "rosbag_foxglove").mkdir()
    (run_dir / "rosbag" / "rosbag_0.mcap").write_bytes(b"raw")
    (run_dir / "rosbag_foxglove" / "rosbag_foxglove_0.mcap").write_bytes(b"lite")
    (run_dir / "summary.json").write_text("{}", encoding="utf-8")
    (run_dir / "foxglove_replay_summary.json").write_text("{}", encoding="utf-8")

    targets = upload._build_targets(run_dir)

    assert targets[0].kind == "mcap"
    assert targets[0].path == run_dir / "rosbag" / "rosbag_0.mcap"
    assert [target.kind for target in targets] == ["mcap", "summary", "replay_summary"]


def test_upload_targets_lite_uses_foxglove_mcap_and_attach_replay_summary(tmp_path: Path) -> None:
    run_dir = tmp_path / "20260607_185314"
    (run_dir / "rosbag").mkdir(parents=True)
    (run_dir / "rosbag_foxglove").mkdir()
    (run_dir / "rosbag" / "rosbag_0.mcap").write_bytes(b"raw")
    (run_dir / "rosbag_foxglove" / "rosbag_foxglove_0.mcap").write_bytes(b"lite")
    (run_dir / "summary.json").write_text("{}", encoding="utf-8")
    (run_dir / "foxglove_replay_summary.json").write_text("{}", encoding="utf-8")

    targets = upload._build_targets(run_dir, lite=True)

    assert targets[0].kind == "mcap"
    assert targets[0].path == run_dir / "rosbag_foxglove" / "rosbag_foxglove_0.mcap"
    assert [target.kind for target in targets] == ["mcap", "summary", "replay_summary"]


def test_upload_targets_full_when_foxglove_mcap_missing(tmp_path: Path) -> None:
    run_dir = tmp_path / "20260607_185314"
    (run_dir / "rosbag").mkdir(parents=True)
    (run_dir / "rosbag" / "rosbag_0.mcap").write_bytes(b"raw")
    (run_dir / "summary.json").write_text("{}", encoding="utf-8")

    targets = upload._build_targets(run_dir)

    assert targets[0].kind == "mcap"
    assert targets[0].path == run_dir / "rosbag" / "rosbag_0.mcap"
    assert [target.kind for target in targets] == ["mcap", "summary"]


def test_upload_generate_lite_invokes_replay_builder(monkeypatch, tmp_path: Path) -> None:  # noqa: ANN001
    run_dir = tmp_path / "20260607_185314"
    run_dir.mkdir()
    calls = []

    def fake_run(cmd, cwd, check):  # noqa: ANN001
        calls.append((cmd, cwd, check))
        (run_dir / "rosbag_foxglove").mkdir()
        (run_dir / "rosbag_foxglove" / "rosbag_foxglove_0.mcap").write_bytes(b"lite")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(upload.subprocess, "run", fake_run)

    assert upload._generate_lite_mcap(run_dir) is True
    assert calls
    assert calls[0][0][-4:] == ["scripts/command/main.py", "foxglove", "build-replay", str(run_dir)]



def test_upload_uses_binary_content_type_for_signed_urls() -> None:
    assert upload._content_type(Path("summary.json")) == "application/octet-stream"
    assert upload._content_type(Path("replay.mcap")) == "application/octet-stream"

def test_foxglove_lite_profile_lists_required_and_optional_topics() -> None:
    profile = Path("profiles/navlab-exploration-foxglove-lite-topics.txt")
    parsed = replay.load_lite_topic_profile(profile)
    required = replay.load_lite_required_topics(profile)

    assert parsed.overlay_topic == "/navlab/official_maze/map"
    assert "/navlab/official_maze/map" in required
    assert "/slam/odom" in parsed.required_topics
    assert parsed.retain_intervals["/tf_static"] is None
    assert parsed.retain_intervals["/scan"] == 0.10
    assert "/external_nav/odom" in parsed.retain_intervals
    assert "/external_nav/odom" not in parsed.required_topics
    assert "/imu" in parsed.dropped_topics


def test_foxglove_lite_profile_missing_file_is_fatal(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        replay.load_lite_topic_profile(tmp_path / "missing-topics.txt")


def test_foxglove_lite_profile_requires_overlay(tmp_path: Path) -> None:
    profile = tmp_path / "topics.txt"
    profile.write_text("required /tf interval=0.05\ndrop /imu\n", encoding="utf-8")

    with pytest.raises(ValueError, match="missing overlay topic"):
        replay.load_lite_topic_profile(profile)


def test_p9_summary_template_records_truth_boundary(tmp_path: Path) -> None:
    run_dir = tmp_path / "20260607_185314"
    run_dir.mkdir()
    raw_mcap = run_dir / "rosbag/rosbag_0.mcap"
    output_mcap = run_dir / "rosbag_foxglove/rosbag_foxglove_0.mcap"

    summary = replay._summary_template(
        run_dir,
        raw_mcap,
        output_mcap,
        tmp_path / "maze.sdf",
        {"ok": True},
        {"profile": "minimal_run", "publishable": False},
        [],
    )

    assert summary["ok"] is True
    assert summary["p8_prerequisite"]["ok"] is True
    assert summary["truth_boundary"]["uses_official_maze_as_input"] is False
    assert summary["truth_boundary"]["uses_gazebo_truth_as_input"] is False
    assert summary["truth_boundary"]["official_maze_layer_role"] == "visualization_only"
