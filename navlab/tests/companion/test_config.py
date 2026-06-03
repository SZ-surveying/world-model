from __future__ import annotations

from navlab.companion.config import load_config


def test_companion_config_reads_companion_section() -> None:
    config = load_config()

    assert config.stop_distance.value == 0.5
    assert config.console_log_level.value == "DEBUG"
    assert config.file_log_level.value == "INFO"
