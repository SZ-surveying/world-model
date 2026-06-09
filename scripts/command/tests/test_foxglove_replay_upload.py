from __future__ import annotations

import json

from src.foxglove import replay, upload


def test_hover_summary_uses_hover_lite_profile_by_default() -> None:
    summary = {"ok": True, "hover_gate": {}, "hover": {"ok": True}, "landing": {"ok": True}}

    profile = replay.resolve_lite_topic_profile(summary, replay.FOXGLOVE_LITE_PROFILE)

    assert profile == replay.HOVER_FOXGLOVE_LITE_PROFILE


def test_upload_targets_use_hover_prefix(tmp_path) -> None:
    run_dir = tmp_path / "20260609_110400"
    run_dir.mkdir()
    (run_dir / "summary.json").write_text(
        json.dumps({"ok": True, "hover_gate": {}, "hover": {"ok": True}, "landing": {"ok": True}}),
        encoding="utf-8",
    )

    targets = upload._build_targets(run_dir, lite=True)

    assert targets[0].filename == "navlab_hover_20260609_110400.mcap"
    assert targets[1].filename == "navlab_hover_20260609_110400_summary.json"
