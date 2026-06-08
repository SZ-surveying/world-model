from __future__ import annotations

import fnmatch
import json
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any, ClassVar

from rich.console import Console

from src import host
from src.config import RunConfig
from src.project_config import RuntimeConfig as ProjectRuntimeConfig
from src.project_config import load_orchestration_runtime_backend_config, load_runtime_config
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


def _load_runtime_selection(config: RunConfig):
    runtime = load_runtime_config()
    return load_orchestration_runtime_backend_config(
        ProjectRuntimeConfig(
            lab_root=runtime.lab_root,
            ardupilot_root=runtime.ardupilot_root,
            mavlink_router_root=runtime.mavlink_router_root,
            venv_path=runtime.venv_path,
            config_file=config.orchestration.path,
            config_loaded=config.orchestration.path.is_file(),
        )
    )


def _build_real_preflight_summary(
    config: RunConfig,
    *,
    topics: tuple[str, ...] | None,
    topic_probe_error: str | None,
) -> dict[str, Any]:
    blockers: list[str] = []
    try:
        selected = _load_runtime_selection(config)
    except Exception as exc:
        return {
            "ok": False,
            "blocked": True,
            "blockers": [f"runtime_config_invalid:{exc}"],
            "runtime_backend": "unknown",
            "runtime_mode": "unknown",
        }

    if selected.backend.value != "process":
        blockers.append(f"runtime_backend_must_be_process:{selected.backend.value}")
    if selected.mode.value != "real":
        blockers.append(f"runtime_mode_must_be_real:{selected.mode.value}")

    required_topics = selected.real_sources.required_real_topics
    forbidden_topics = selected.real_sources.forbidden_simulation_input_topics
    if topics is None:
        blockers.append(f"real_topic_probe_failed:{topic_probe_error or 'unknown'}")
        seen_topics: tuple[str, ...] = ()
    else:
        seen_topics = topics
        missing = [topic for topic in required_topics if topic not in seen_topics]
        blockers.extend(f"required_real_topic_missing:{topic}" for topic in missing)
        forbidden_seen = [
            topic
            for topic in seen_topics
            if any(fnmatch.fnmatch(topic, pattern) for pattern in forbidden_topics)
        ]
        blockers.extend(f"forbidden_simulation_topic_present:{topic}" for topic in forbidden_seen)

    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "runtime_backend": selected.backend.value,
        "runtime_mode": selected.mode.value,
        "runtime_backend_summary": host._runtime_backend_summary(config),
        "source_claims": host._runtime_source_claims(config),
        "real_preflight": {
            "required_real_topics": list(required_topics),
            "forbidden_simulation_input_topics": list(forbidden_topics),
            "topic_probe_error": topic_probe_error or "",
            "topic_count": len(seen_topics),
        },
    }


def _collect_ros2_topics(*, timeout_sec: float = 8.0) -> tuple[tuple[str, ...] | None, str | None]:
    if shutil.which("ros2") is None:
        return None, "ros2_not_found"
    env = os.environ.copy()
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            env=env,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)
    if result.returncode != 0:
        return None, (result.stderr or result.stdout or f"rc={result.returncode}").strip()
    topics = tuple(line.strip() for line in result.stdout.splitlines() if line.strip())
    return topics, None


@TaskRegistry.register
class RealPreflightDoctorTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "real-preflight-doctor"
    TASK_DESCRIPTION: ClassVar[str] = "Check process+real runtime topic and source preflight contract."

    def run(self, *, config_path: str | Path | None = None, console: Console | None = None) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path)
        artifact_dir = Path(os.environ.get("ARTIFACT_DIR", f"artifacts/ros/navlab_real_preflight_doctor/{config.run_id}"))
        config = RunConfig.from_config(config_path=config_path, run_id=config.run_id, artifact_dir=artifact_dir)
        console.print("Checking real runtime preflight contract")
        topics, topic_error = _collect_ros2_topics()
        summary = _build_real_preflight_summary(config, topics=topics, topic_probe_error=topic_error)
        artifact_dir.mkdir(parents=True, exist_ok=True)
        (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        color = "green" if summary["ok"] else "red"
        console.print(f"[{color}]Real preflight doctor rc={0 if summary['ok'] else 20}[/{color}]")
        console.print(f"Summary: {artifact_dir / 'summary.json'}")
        return 0 if summary["ok"] else 20
