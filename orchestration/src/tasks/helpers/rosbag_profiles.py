from __future__ import annotations

import re
from pathlib import Path
from typing import Any


def profile_topics(profile_path: Path) -> tuple[list[str], list[str], list[str]]:
    required: list[str] = []
    optional: list[str] = []
    if profile_path.is_file():
        for raw_line in profile_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(maxsplit=1)
            if len(parts) != 2:
                continue
            kind, topic = parts
            if kind == "required":
                required.append(topic)
            elif kind == "optional":
                optional.append(topic)
    return required, optional, [*required, *optional]


def load_rosbag_metadata_counts(metadata: Path) -> dict[str, int]:
    content = metadata.read_text(encoding="utf-8")
    counts: dict[str, int] = {}
    topic_matches = list(re.finditer(r"name: (/[^\n]+)", content))
    for index, match in enumerate(topic_matches):
        topic = match.group(1).strip()
        end = topic_matches[index + 1].start() if index + 1 < len(topic_matches) else len(content)
        block = content[match.end() : end]
        count_match = re.search(r"message_count:\s*(\d+)", block)
        counts[topic] = int(count_match.group(1)) if count_match else 0
    return counts


def validate_official_rosbag_profile(
    *,
    profile: Path,
    metadata: Path,
    required: list[str],
    optional: list[str],
) -> dict[str, Any]:
    counts = load_rosbag_metadata_counts(metadata)
    missing_required = [topic for topic in required if topic not in counts]
    zero_count_required = [topic for topic in required if counts.get(topic, 0) <= 0]
    present_optional = [topic for topic in optional if counts.get(topic, 0) > 0]
    missing_optional = [topic for topic in optional if topic not in counts]
    return {
        "ok": not missing_required and not zero_count_required,
        "recorded": True,
        "profile": str(profile),
        "metadata": str(metadata),
        "required_topics": required,
        "optional_topics": optional,
        "present_topics": sorted(counts),
        "message_counts": counts,
        "missing_required_topics": missing_required,
        "zero_count_required_topics": zero_count_required,
        "present_optional_topics": present_optional,
        "missing_optional_topics": missing_optional,
    }
