from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

DEFAULT_AUTO_FRAME_ID = "map"
DEFAULT_AUTO_FORWARD_SPEED = 0.2
DEFAULT_AUTO_POSITION_TOLERANCE = 0.05


@dataclass(frozen=True, slots=True)
class Waypoint:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class StraightLineMission:
    version: int
    mode: str
    frame_id: str
    forward_speed: float
    position_tolerance: float
    start: Waypoint
    waypoints: tuple[Waypoint, ...]

    @property
    def goal(self) -> Waypoint:
        return self.waypoints[-1]


def load_straight_line_mission(path: str | Path) -> StraightLineMission:
    mission_path = Path(path)
    data = _load_waypoint_document(mission_path)
    return _parse_straight_line_mission(data, mission_path=mission_path)


def _load_waypoint_document(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"Waypoint file not found: {path}")

    text = path.read_text(encoding="utf-8")
    suffix = path.suffix.lower()
    if suffix in {".yaml", ".yml"}:
        data = _parse_minimal_yaml(text)
    else:
        raise ValueError(f"Unsupported waypoint file format '{path.suffix}': expected .yaml or .yml")

    if not isinstance(data, dict):
        raise ValueError(f"Invalid waypoint file {path}: expected a top-level mapping")
    return data


def _parse_straight_line_mission(data: dict[str, Any], *, mission_path: Path) -> StraightLineMission:
    version = _parse_int(data.get("version", 1), field_name="version")
    mode = str(data.get("mode", "straight_line"))
    if mode != "straight_line":
        raise ValueError(f"Unsupported waypoint mode '{mode}' in {mission_path}: only 'straight_line' is supported")

    frame_id = str(data.get("frame_id", DEFAULT_AUTO_FRAME_ID))
    if frame_id != DEFAULT_AUTO_FRAME_ID:
        raise ValueError(f"Unsupported frame_id '{frame_id}' in {mission_path}: only 'map' is supported")

    forward_speed = _parse_float(
        data.get("forward_speed", DEFAULT_AUTO_FORWARD_SPEED),
        field_name="forward_speed",
    )
    if forward_speed <= 0.0:
        raise ValueError(f"Invalid forward_speed in {mission_path}: expected > 0")

    position_tolerance = _parse_float(
        data.get("position_tolerance", DEFAULT_AUTO_POSITION_TOLERANCE),
        field_name="position_tolerance",
    )
    if position_tolerance <= 0.0:
        raise ValueError(f"Invalid position_tolerance in {mission_path}: expected > 0")

    start = _parse_optional_waypoint(
        data.get("start"),
        mission_path=mission_path,
        field_name="start",
        default=Waypoint(x=0.0, y=0.0, z=0.0),
    )
    waypoints = _parse_mission_waypoints(data, mission_path=mission_path)

    _validate_straight_line_waypoints(start=start, waypoints=waypoints, mission_path=mission_path)
    return StraightLineMission(
        version=version,
        mode=mode,
        frame_id=frame_id,
        forward_speed=forward_speed,
        position_tolerance=position_tolerance,
        start=start,
        waypoints=waypoints,
    )


def _parse_waypoint(item: Any, *, index: int, mission_path: Path) -> Waypoint:
    if not isinstance(item, dict):
        raise ValueError(f"Invalid waypoint #{index + 1} in {mission_path}: expected a mapping")

    return Waypoint(
        x=_parse_float(item.get("x"), field_name=f"waypoints[{index}].x"),
        y=_parse_float(item.get("y", 0.0), field_name=f"waypoints[{index}].y"),
        z=_parse_float(item.get("z", 0.0), field_name=f"waypoints[{index}].z"),
    )


def _parse_named_waypoint(item: Any, *, field_name: str, mission_path: Path) -> Waypoint:
    if not isinstance(item, dict):
        raise ValueError(f"Invalid {field_name} in {mission_path}: expected a mapping")
    return Waypoint(
        x=_parse_float(item.get("x"), field_name=f"{field_name}.x"),
        y=_parse_float(item.get("y", 0.0), field_name=f"{field_name}.y"),
        z=_parse_float(item.get("z", 0.0), field_name=f"{field_name}.z"),
    )


def _parse_optional_waypoint(
    item: Any,
    *,
    mission_path: Path,
    field_name: str,
    default: Waypoint,
) -> Waypoint:
    if item is None:
        return default
    return _parse_named_waypoint(item, field_name=field_name, mission_path=mission_path)


def _parse_mission_waypoints(data: dict[str, Any], *, mission_path: Path) -> tuple[Waypoint, ...]:
    raw_waypoints = data.get("waypoints")
    raw_goal = data.get("goal")

    if raw_waypoints not in (None, "") and raw_goal not in (None, ""):
        raise ValueError(f"Invalid mission in {mission_path}: use either 'goal' or 'waypoints', not both")

    if raw_waypoints not in (None, ""):
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError(f"Invalid waypoints in {mission_path}: expected a non-empty list")
        return tuple(
            _parse_waypoint(item, index=index, mission_path=mission_path) for index, item in enumerate(raw_waypoints)
        )

    if raw_goal in (None, ""):
        raise ValueError(f"Invalid mission in {mission_path}: expected either 'goal' or 'waypoints'")

    return (_parse_named_waypoint(raw_goal, field_name="goal", mission_path=mission_path),)


def _validate_straight_line_waypoints(
    *,
    start: Waypoint,
    waypoints: tuple[Waypoint, ...],
    mission_path: Path,
) -> None:
    previous_x = start.x
    reference_y = start.y
    reference_z = start.z
    for index, waypoint in enumerate(waypoints):
        if abs(waypoint.y - reference_y) > 1e-9 or abs(waypoint.z - reference_z) > 1e-9:
            raise ValueError(
                f"Waypoint #{index + 1} in {mission_path} leaves the straight line: "
                "start/goal and all waypoints must keep the same y/z in the first auto-mode slice"
            )
        if waypoint.x < previous_x:
            raise ValueError(
                f"Waypoint #{index + 1} in {mission_path} moves backward in x: "
                "auto mode currently only supports monotonic +X motion from start to goal"
            )
        previous_x = waypoint.x


def _parse_int(value: Any, *, field_name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {field_name}: expected an integer") from exc


def _parse_float(value: Any, *, field_name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {field_name}: expected a number") from exc


def _parse_minimal_yaml(text: str) -> dict[str, Any]:
    data: dict[str, Any] = {}
    current_block_key: str | None = None
    current_block_list: list[dict[str, Any]] | None = None
    current_block_mapping: dict[str, Any] | None = None
    current_item: dict[str, Any] | None = None

    for raw_line in text.splitlines():
        line = _strip_yaml_comment(raw_line).rstrip()
        if not line:
            continue

        indent = len(line) - len(line.lstrip(" "))
        content = line.lstrip(" ")
        if "\t" in raw_line:
            raise ValueError("Tabs are not supported in waypoint YAML files")

        if indent == 0:
            current_item = None
            if content.endswith(":"):
                current_block_key = content[:-1].strip()
                current_block_list = []
                current_block_mapping = {}
                data[current_block_key] = current_block_mapping
                continue
            key, value = _split_yaml_pair(content)
            data[key] = _parse_yaml_scalar(value)
            current_block_key = None
            current_block_list = None
            current_block_mapping = None
            continue

        if current_block_key is None:
            raise ValueError(f"Unsupported YAML structure near: {raw_line}")

        if indent == 2 and content.startswith("- "):
            if current_block_list is None:
                current_block_list = []
                data[current_block_key] = current_block_list
                current_block_mapping = None
            current_item = {}
            current_block_list.append(current_item)
            remainder = content[2:].strip()
            if remainder:
                key, value = _split_yaml_pair(remainder)
                current_item[key] = _parse_yaml_scalar(value)
            continue

        if indent == 2 and current_block_mapping is not None:
            key, value = _split_yaml_pair(content)
            current_block_mapping[key] = _parse_yaml_scalar(value)
            continue

        if indent == 4 and current_item is not None:
            key, value = _split_yaml_pair(content)
            current_item[key] = _parse_yaml_scalar(value)
            continue

        raise ValueError(f"Unsupported YAML structure near: {raw_line}")

    return data


def _strip_yaml_comment(line: str) -> str:
    if "#" not in line:
        return line
    content, _, _comment = line.partition("#")
    return content


def _split_yaml_pair(content: str) -> tuple[str, str]:
    key, separator, value = content.partition(":")
    if not separator:
        raise ValueError(f"Invalid YAML line: {content}")
    return key.strip(), value.strip()


def _parse_yaml_scalar(value: str) -> Any:
    if not value:
        return ""
    if value[0] == value[-1] and value[0] in {'"', "'"}:
        return value[1:-1]
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value
