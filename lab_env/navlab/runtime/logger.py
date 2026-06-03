"""Singleton loguru logger for NavLab runtime modules."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any


class RuntimeLogger:
    """Thin wrapper around loguru.logger with console DEBUG and file INFO sinks."""

    def __init__(self, log_path: Path | str | None = None) -> None:
        from loguru import logger as loguru_logger

        self.logger = loguru_logger
        self.log_path: Path | None = None
        self.configure(log_path)

    def configure(self, log_path: Path | str | None = None) -> None:
        self.logger.remove()
        self.log_path = Path(log_path) if log_path is not None else None

        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.logger.add(
                self.log_path,
                rotation="10 MB",
                retention="7 days",
                level="INFO",
                enqueue=True,
                format="{time:YYYY-MM-DD HH:mm:ss} | {level} | {message}",
            )

        self.logger.add(
            sys.stderr,
            level="DEBUG",
            format="<cyan>{time:HH:mm:ss}</cyan> | <level>{level}</level> | {message}",
            colorize=True,
        )


class _LoggerProxy:
    """Lazy proxy so modules can import `logger` before init_logger() runs."""

    def __getattr__(self, name: str) -> Any:
        return getattr(get_logger().logger, name)


_instance: RuntimeLogger | None = None
logger = _LoggerProxy()


def init_logger(log_path: Path | str | None = None) -> RuntimeLogger:
    """Initialize or reconfigure the singleton logger."""
    global _instance
    if _instance is None:
        _instance = RuntimeLogger(log_path)
    else:
        _instance.configure(log_path)
    return _instance


def get_logger() -> RuntimeLogger:
    """Return the singleton logger instance."""
    global _instance
    if _instance is None:
        _instance = RuntimeLogger()
    return _instance
