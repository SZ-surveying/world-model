from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from loguru import logger as loguru_logger

_PROCESS_LOG_FORMAT = "{time:YYYY-MM-DD HH:mm:ss.SSS} | {level:<7} | {extra[process_name]} | {message}"
_configured = False


@dataclass(slots=True)
class ProcessLogHandle:
    process_name: str
    log_path: Path
    sink_id: int
    logger: Any

    def close(self) -> dict[str, Any]:
        loguru_logger.remove(self.sink_id)
        return {
            "process": self.process_name,
            "path": str(self.log_path),
            "entries": _count_log_entries(self.log_path),
            "bytes": self.log_path.stat().st_size if self.log_path.is_file() else 0,
        }


def start_process_logger(*, process_name: str, log_path: str | Path) -> ProcessLogHandle:
    _ensure_process_logging_configured()
    path = Path(log_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    sink_id = loguru_logger.add(
        path,
        format=_PROCESS_LOG_FORMAT,
        filter=lambda record, name=process_name: record["extra"].get("process_name") == name,
        level="INFO",
        enqueue=False,
        encoding="utf-8",
    )
    return ProcessLogHandle(
        process_name=process_name,
        log_path=path,
        sink_id=sink_id,
        logger=loguru_logger.bind(process_name=process_name),
    )


def close_process_loggers(handles: list[ProcessLogHandle]) -> list[dict[str, Any]]:
    return [handle.close() for handle in handles]


def _ensure_process_logging_configured() -> None:
    global _configured
    if not _configured:
        loguru_logger.remove()
        _configured = True


def _count_log_entries(path: Path) -> int:
    if not path.is_file():
        return 0
    return sum(1 for line in path.read_text(encoding="utf-8", errors="replace").splitlines() if line.strip())
