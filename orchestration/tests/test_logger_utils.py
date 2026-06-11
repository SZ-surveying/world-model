from __future__ import annotations

from src.logger_utils import close_process_loggers, start_process_logger


def test_process_loggers_write_to_separate_files(tmp_path) -> None:
    companion = start_process_logger(process_name="companion", log_path=tmp_path / "companion.log")
    slam = start_process_logger(process_name="slam", log_path=tmp_path / "slam.log")

    companion.logger.info("companion entry")
    slam.logger.info("slam entry")

    stats = close_process_loggers([companion, slam])

    companion_text = (tmp_path / "companion.log").read_text(encoding="utf-8")
    slam_text = (tmp_path / "slam.log").read_text(encoding="utf-8")
    assert "companion entry" in companion_text
    assert "slam entry" not in companion_text
    assert "slam entry" in slam_text
    assert "companion entry" not in slam_text
    assert stats[0]["process"] == "companion"
    assert stats[0]["entries"] == 1
    assert stats[1]["process"] == "slam"
    assert stats[1]["entries"] == 1
