from __future__ import annotations

from lab_env.navlab.runtime.acceptance import _write_foxglove_notes


def test_write_foxglove_notes_uses_template(tmp_path) -> None:
    _write_foxglove_notes(artifact_dir=tmp_path)

    notes = (tmp_path / "foxglove_notes.md").read_text(encoding="utf-8")
    assert "# NavLab Foxglove replay notes" in notes
    assert f"- Rosbag: `{tmp_path / 'rosbag'}`" in notes
    assert "`/navlab/replay/constraint_markers`" in notes
    assert "wait_ready -> guided -> arm" in notes
