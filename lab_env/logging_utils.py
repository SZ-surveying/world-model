from __future__ import annotations

import sys
from pathlib import Path

from loguru import logger

_CONSOLE_FORMAT = "<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <7}</level> | <level>{message}</level>"
_FILE_FORMAT = "{time:YYYY-MM-DD HH:mm:ss.SSS} | {level: <7} | {message}"


def configure_sim_logging(
    *,
    log_file: str | Path | None = None,
    console: bool = True,
    console_level: str = "DEBUG",
    file_level: str = "INFO",
) -> None:
    logger.remove()
    if console:
        logger.add(sys.stderr, format=_CONSOLE_FORMAT, colorize=True, level=console_level.upper())
    if log_file is not None:
        path = Path(log_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        logger.add(path, format=_FILE_FORMAT, encoding="utf-8", enqueue=False, level=file_level.upper())


__all__ = ["configure_sim_logging", "logger"]
