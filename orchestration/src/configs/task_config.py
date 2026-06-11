from __future__ import annotations

from pathlib import Path
from typing import Any

import tomllib

from src.configs.project_config import PROJECT_PATH

DEFAULT_TASK_CONFIG_DIR = PROJECT_PATH / "configs"

TASK_CONFIG_NAMES = {
    "hover": "hover",
    "exploration": "exploration",
    "exploration-doctor": "exploration",
    "scan-robustness": "scan_robustness",
    "scan-robustness-doctor": "scan_robustness",
    "real-preflight-doctor": "real_preflight",
    "real-prepare": "real_prepare",
}


def default_task_config_path(task_name: str) -> Path:
    return DEFAULT_TASK_CONFIG_DIR / f"{TASK_CONFIG_NAMES.get(task_name, task_name.replace('-', '_'))}.toml"


def resolve_task_config_path(task_name: str | None, path: str | Path | None = None) -> tuple[Path | None, str]:
    if path:
        return Path(path).expanduser(), "CLI --task-config"
    if not task_name:
        return None, "none"
    return default_task_config_path(task_name), "task default"


def load_task_config_data(task_name: str | None, *, task_config_path: str | Path | None = None) -> dict[str, Any]:
    path, source = resolve_task_config_path(task_name, task_config_path)
    if path is None:
        return {}
    if not path.is_file():
        if source == "CLI --task-config":
            raise FileNotFoundError(f"Task config does not exist: {path}")
        return {}
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Invalid task config in {path}")
    return data


def optional_task_table(data: dict[str, Any], path: Path | None) -> dict[str, Any]:
    raw_task = data.get("task", {})
    if raw_task is None:
        raw_task = {}
    if not isinstance(raw_task, dict):
        location = str(path) if path else "task config"
        raise ValueError(f"Invalid [task] section in {location}")
    return raw_task


def resolve_task_float(
    task: dict[str, Any],
    key: str,
    cli_value: float | None,
    default: float,
) -> tuple[float, str]:
    if cli_value is not None:
        return float(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return float(task[key]), "task config"
    return default, "default"


def resolve_task_int(
    task: dict[str, Any],
    key: str,
    cli_value: int | None,
    default: int,
) -> tuple[int, str]:
    if cli_value is not None:
        return int(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return int(task[key]), "task config"
    return default, "default"


def resolve_task_str(
    task: dict[str, Any],
    key: str,
    cli_value: str | None,
    default: str,
) -> tuple[str, str]:
    if cli_value not in (None, ""):
        return str(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return str(task[key]), "task config"
    return default, "default"


def resolve_task_bool(
    task: dict[str, Any],
    key: str,
    cli_value: bool | None,
    default: bool,
) -> tuple[bool, str]:
    if cli_value is not None:
        return bool(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return as_bool(task[key], default), "task config"
    return default, "default"


def resolve_task_str_tuple(
    task: dict[str, Any],
    key: str,
    cli_value: tuple[str, ...] | None,
    default: tuple[str, ...],
) -> tuple[tuple[str, ...], str]:
    if cli_value is not None:
        return tuple(cli_value), "CLI"
    if task.get(key) not in (None, ""):
        return as_str_tuple(task[key], default), "task config"
    return default, "default"


def as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).lower() in {"1", "true", "yes", "on"}


def as_str_tuple(value: Any, default: tuple[str, ...] = ()) -> tuple[str, ...]:
    if value is None:
        return default
    if isinstance(value, str):
        if not value.strip():
            return default
        return tuple(item.strip() for item in value.split(",") if item.strip())
    if isinstance(value, list | tuple):
        return tuple(str(item) for item in value)
    return (str(value),)
