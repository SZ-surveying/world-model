from __future__ import annotations

import importlib
import os
from pathlib import Path

from lab_env.sim.rosbag import RosbagOptions, with_rosbag_recording

_SIM_ROSBAG_TOPIC_FILE = Path("/workspace/profiles/sim-rosbag-topics.txt")


def build_rosbag_options(
    *,
    enabled: bool,
    label: str,
    session_id: str | None = None,
    topic_file: str | Path = _SIM_ROSBAG_TOPIC_FILE,
) -> RosbagOptions:
    return RosbagOptions(
        enabled=enabled,
        label=label,
        topic_file=Path(topic_file),
        session_id=session_id or os.environ.get("SESSION_ID", "manual"),
    )


@with_rosbag_recording
def invoke_python_target(target: str, argv: list[str] | None = None) -> int:
    module_name, function_name = target.split(":", 1)
    module = importlib.import_module(module_name)
    func = getattr(module, function_name)
    result = func(argv or [])
    return 0 if result is None else int(result)
