from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[4]
ARTIFACT_ROOT = REPO_ROOT / "artifacts/ros/navlab_companion_sitl_gazebo"
DEFAULT_ARDUPILOT_GZ = REPO_ROOT.parent / "ardupilot_gz"
DEFAULT_MAZE = DEFAULT_ARDUPILOT_GZ / "ardupilot_gz_gazebo/worlds/maze.sdf"
RAW_MCAP_RELATIVE = Path("rosbag/rosbag_0.mcap")
FOXGLOVE_MCAP_RELATIVE = Path("rosbag_foxglove/rosbag_foxglove_0.mcap")
SUMMARY_FILENAME = "foxglove_replay_summary.json"
DEFAULT_RESOLUTION_M = 0.10
DEFAULT_MARGIN_M = 4.0
PUBLISHABLE_MIN_PATH_LENGTH_M = 2.5
PUBLISHABLE_MIN_ACCEPTED_GOALS = 5
EXPLORATION_FOXGLOVE_LITE_PROFILE = REPO_ROOT / "profiles/navlab-exploration-foxglove-lite-topics.txt"
HOVER_FOXGLOVE_LITE_PROFILE = REPO_ROOT / "profiles/navlab-hover-foxglove-lite-topics.txt"
FOXGLOVE_LITE_PROFILE = EXPLORATION_FOXGLOVE_LITE_PROFILE


@dataclass(frozen=True, slots=True)
class Wall:
    name: str
    x: float
    y: float
    yaw: float
    length: float
    thickness: float
    height: float


@dataclass(frozen=True, slots=True)
class BBox:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    @property
    def valid(self) -> bool:
        return self.xmax > self.xmin and self.ymax > self.ymin

    def expand(self, margin: float) -> BBox:
        return BBox(self.xmin - margin, self.ymin - margin, self.xmax + margin, self.ymax + margin)

    def union(self, other: BBox | None) -> BBox:
        if other is None or not other.valid:
            return self
        return BBox(min(self.xmin, other.xmin), min(self.ymin, other.ymin), max(self.xmax, other.xmax), max(self.ymax, other.ymax))

    def as_dict(self) -> dict[str, float]:
        return {"xmin": self.xmin, "ymin": self.ymin, "xmax": self.xmax, "ymax": self.ymax}


@dataclass(frozen=True, slots=True)
class TopicProfile:
    path: Path
    overlay_topic: str
    required_topics: set[str]
    retain_intervals: dict[str, float | None]
    dropped_topics: set[str]

    @property
    def output_required_topics(self) -> set[str]:
        return set(self.required_topics) | {self.overlay_topic}

def resolve_run_dir(value: str | None) -> Path:
    if value:
        path = Path(value).expanduser()
        if path.is_dir():
            return path
        repo_relative = REPO_ROOT / path
        if repo_relative.is_dir():
            return repo_relative
        run_dir = ARTIFACT_ROOT / value
        if run_dir.is_dir():
            return run_dir
        raise FileNotFoundError(f"run directory not found: {run_dir}")
    if not ARTIFACT_ROOT.is_dir():
        raise FileNotFoundError(f"artifact root not found: {ARTIFACT_ROOT}")
    candidates = [path for path in ARTIFACT_ROOT.iterdir() if path.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"no run directories under: {ARTIFACT_ROOT}")
    return max(candidates, key=lambda path: path.name)


def parse_bbox(value: str) -> BBox:
    parts = [float(item) for item in value.split(",")]
    if len(parts) != 4:
        raise ValueError("--bbox must be xmin,ymin,xmax,ymax")
    bbox = BBox(parts[0], parts[1], parts[2], parts[3])
    if not bbox.valid:
        raise ValueError(f"invalid --bbox: {value}")
    return bbox


def build_replay(
    *,
    run_dir: Path,
    maze_path: Path,
    topic_profile_path: Path = FOXGLOVE_LITE_PROFILE,
    resolution_m: float = DEFAULT_RESOLUTION_M,
    margin_m: float = DEFAULT_MARGIN_M,
    bbox_override: BBox | None = None,
    full: bool = False,
    dry_run: bool = False,
) -> dict[str, Any]:
    raw_mcap = run_dir / RAW_MCAP_RELATIVE
    output_mcap = run_dir / FOXGLOVE_MCAP_RELATIVE
    p8_summary_path = run_dir / "summary.json"
    blockers: list[str] = []
    if not p8_summary_path.is_file():
        blockers.append("run summary missing")
    if not raw_mcap.is_file():
        blockers.append("raw MCAP missing")
    if not maze_path.is_file():
        blockers.append("official maze.sdf missing")
    if resolution_m <= 0:
        blockers.append("overlay resolution must be positive")
    p8_summary = _read_json(p8_summary_path) if p8_summary_path.is_file() else {}
    task_kind = task_kind_from_summary(p8_summary)
    topic_profile = load_lite_topic_profile(resolve_lite_topic_profile(p8_summary, topic_profile_path))
    if p8_summary and not bool(p8_summary.get("ok")):
        blockers.append(f"{task_kind} summary is not ok")

    walls: list[Wall] = []
    maze_extent: BBox | None = None
    if maze_path.is_file():
        walls = parse_walls(maze_path)
        if not walls:
            blockers.append("official maze has no walls")
        else:
            maze_extent = walls_extent(walls)

    replay_quality = replay_quality_from_summary(p8_summary)
    if not replay_quality["publishable"]:
        replay_quality["warnings"].append("minimal_run_or_insufficient_path_length")

    if blockers:
        summary = _summary_template(run_dir, raw_mcap, output_mcap, maze_path, p8_summary, replay_quality, blockers)
        _write_json(run_dir / SUMMARY_FILENAME, summary)
        return summary

    assert maze_extent is not None
    try:
        mcap_modules = _load_mcap_modules()
        trajectory_topics = sorted(topic for topic in topic_profile.retain_intervals if topic.endswith("/odom"))
        scan = scan_replay_inputs(raw_mcap, mcap_modules, trajectory_topics=trajectory_topics)
        crop_bbox, crop_mode = choose_crop_bbox(
            maze_extent=maze_extent,
            map_bbox=scan["map_bbox"],
            trajectory_bbox=scan["trajectory_bbox"],
            start_xy=scan["start_xy"],
            margin_m=margin_m,
            bbox_override=bbox_override,
            full=full,
        )
        overlay = rasterize_walls(walls, crop_bbox, resolution_m=resolution_m)
        missing_raw = sorted(topic for topic in topic_profile.required_topics if topic not in scan["present_topics"])
        if missing_raw:
            blockers.append(f"raw MCAP missing replay topics: {', '.join(missing_raw)}")
        if not scan["map_schema"]:
            blockers.append("raw MCAP has no /map schema for OccupancyGrid overlay")
        if blockers:
            summary = _summary_template(run_dir, raw_mcap, output_mcap, maze_path, p8_summary, replay_quality, blockers)
            summary["crop"] = {"mode": crop_mode, "margin_m": margin_m, "bbox_m": crop_bbox.as_dict()}
            _write_json(run_dir / SUMMARY_FILENAME, summary)
            return summary
        if not dry_run:
            output_mcap.parent.mkdir(parents=True, exist_ok=True)
            write_lite_mcap(raw_mcap, output_mcap, overlay, scan["map_schema"], topic_profile, mcap_modules)
        output_counts = inspect_output_counts(output_mcap, mcap_modules) if output_mcap.is_file() else {}
    except Exception as exc:  # noqa: BLE001 - convert implementation failures to blockers.
        blockers.append(str(exc))
        summary = _summary_template(run_dir, raw_mcap, output_mcap, maze_path, p8_summary, replay_quality, blockers)
        _write_json(run_dir / SUMMARY_FILENAME, summary)
        return summary

    raw_size = raw_mcap.stat().st_size
    output_size = output_mcap.stat().st_size if output_mcap.is_file() else 0
    missing_output = sorted(topic for topic in topic_profile.output_required_topics if output_counts.get(topic, 0) <= 0)
    if missing_output and not dry_run:
        blockers.append(f"Foxglove-lite MCAP missing required topics: {', '.join(missing_output)}")
    if not dry_run and output_size <= 0:
        blockers.append("Foxglove-lite MCAP not generated")

    summary = _summary_template(run_dir, raw_mcap, output_mcap, maze_path, p8_summary, replay_quality, blockers)
    summary.update(
        {
            "official_maze": {
                "source": str(maze_path),
                "sha256": file_sha256(maze_path),
                "wall_count": len(walls),
                "extent_m": maze_extent.as_dict(),
            },
            "overlay": {
                "topic": topic_profile.overlay_topic,
                "frame_id": "map",
                "resolution_m": resolution_m,
                "scale": 1.0,
                "role": "visualization_only",
                "width": overlay["info"]["width"],
                "height": overlay["info"]["height"],
                "occupied_cells": sum(1 for value in overlay["data"] if value == 100),
            },
            "crop": {"mode": crop_mode, "margin_m": margin_m, "bbox_m": crop_bbox.as_dict()},
            "replay_mcap": {
                "path": str(output_mcap),
                "topic_profile": str(topic_profile.path),
                "raw_mcap_size_bytes": raw_size,
                "foxglove_mcap_size_bytes": output_size,
                "size_reduction_ratio": (raw_size / output_size) if output_size else 0.0,
                "required_topics": sorted(topic_profile.output_required_topics),
                "missing_topics": missing_output if not dry_run else [],
                "present_topics": sorted(output_counts),
                "message_counts": output_counts,
                "configured_drop_topics": sorted(topic_profile.dropped_topics),
                "dropped_topics": sorted(scan["present_topics"] - set(topic_profile.retain_intervals)),
                "retained_topics": sorted(topic_profile.retain_intervals),
                "downsampled_topics": {k: v for k, v in topic_profile.retain_intervals.items() if v is not None},
                "dry_run": dry_run,
            },
        }
    )
    summary["ok"] = not blockers
    summary["blocked"] = bool(blockers)
    summary["blockers"] = blockers
    _write_json(run_dir / SUMMARY_FILENAME, summary)
    return summary


def _load_mcap_modules() -> dict[str, Any]:
    try:
        from mcap.reader import make_reader
        from mcap.writer import Writer
        from mcap_ros2._dynamic import serialize_dynamic
        from mcap_ros2.decoder import DecoderFactory
    except ImportError as exc:
        raise RuntimeError(
            "missing MCAP dependencies; run with `uv run --project scripts/command python scripts/command/main.py foxglove build-replay`"
        ) from exc
    return {"make_reader": make_reader, "Writer": Writer, "serialize_dynamic": serialize_dynamic, "DecoderFactory": DecoderFactory}


def parse_walls(path: Path) -> list[Wall]:
    root = ET.parse(path).getroot()
    model = root.find(".//model[@name='maze']")
    if model is None:
        raise ValueError(f"maze model not found in {path}")
    walls: list[Wall] = []
    for link in model.findall("link"):
        name = link.get("name") or "wall"
        pose_el = link.find("pose")
        link_pose = _pose_values(pose_el.text if pose_el is not None else None)
        link_yaw = link_pose[5]
        if pose_el is not None and pose_el.get("degrees") == "true":
            link_yaw = math.radians(link_yaw)
        collision = link.find("collision")
        if collision is None:
            continue
        collision_pose = _pose_values(collision.findtext("pose"))
        size_text = collision.findtext(".//box/size")
        if not size_text:
            continue
        length, thickness, height = (float(item) for item in size_text.split()[:3])
        cx, cy = collision_pose[0], collision_pose[1]
        x = link_pose[0] + math.cos(link_yaw) * cx - math.sin(link_yaw) * cy
        y = link_pose[1] + math.sin(link_yaw) * cx + math.cos(link_yaw) * cy
        walls.append(Wall(name=name, x=x, y=y, yaw=link_yaw, length=length, thickness=thickness, height=height))
    return walls


def _pose_values(text: str | None) -> tuple[float, float, float, float, float, float]:
    values = [float(item) for item in (text or "0 0 0 0 0 0").split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])  # type: ignore[return-value]


def walls_extent(walls: list[Wall]) -> BBox:
    xs: list[float] = []
    ys: list[float] = []
    for wall in walls:
        half_l = wall.length / 2.0
        half_t = wall.thickness / 2.0
        for lx, ly in ((-half_l, -half_t), (half_l, -half_t), (half_l, half_t), (-half_l, half_t)):
            xs.append(wall.x + math.cos(wall.yaw) * lx - math.sin(wall.yaw) * ly)
            ys.append(wall.y + math.sin(wall.yaw) * lx + math.cos(wall.yaw) * ly)
    return BBox(min(xs), min(ys), max(xs), max(ys))


def scan_replay_inputs(raw_mcap: Path, mcap_modules: dict[str, Any], *, trajectory_topics: list[str]) -> dict[str, Any]:
    make_reader = mcap_modules["make_reader"]
    DecoderFactory = mcap_modules["DecoderFactory"]
    map_bbox: BBox | None = None
    trajectory_bbox: BBox | None = None
    start_xy: tuple[float, float] | None = None
    map_schema: Any = None
    present_topics: set[str] = set()

    with raw_mcap.open("rb") as handle:
        reader = make_reader(handle)
        for schema, channel, _message in reader.iter_messages(log_time_order=False):
            present_topics.add(channel.topic)
            if channel.topic == "/map" and map_schema is None:
                map_schema = schema
    with raw_mcap.open("rb") as handle:
        reader = make_reader(handle, decoder_factories=[DecoderFactory()])
        for _schema, channel, _message, decoded in reader.iter_decoded_messages(
            topics=["/map", *trajectory_topics], log_time_order=False
        ):
            if channel.topic == "/map":
                bbox = occupancy_known_bbox(decoded)
                if bbox is not None:
                    map_bbox = bbox if map_bbox is None else map_bbox.union(bbox)
            else:
                xy = odom_xy(decoded)
                if xy is None:
                    continue
                if start_xy is None:
                    start_xy = xy
                point_bbox = BBox(xy[0], xy[1], xy[0] + 1e-6, xy[1] + 1e-6)
                trajectory_bbox = point_bbox if trajectory_bbox is None else trajectory_bbox.union(point_bbox)
    return {
        "map_bbox": map_bbox,
        "trajectory_bbox": trajectory_bbox,
        "start_xy": start_xy,
        "map_schema": map_schema,
        "present_topics": present_topics,
    }


def occupancy_known_bbox(msg: Any) -> BBox | None:
    info = _get(msg, "info")
    data = _get(msg, "data") or []
    width = int(_get(info, "width") or 0)
    height = int(_get(info, "height") or 0)
    resolution = float(_get(info, "resolution") or 0.0)
    origin = _get(info, "origin")
    pos = _get(origin, "position")
    ox = float(_get(pos, "x") or 0.0)
    oy = float(_get(pos, "y") or 0.0)
    if width <= 0 or height <= 0 or resolution <= 0:
        return None
    min_col = width
    min_row = height
    max_col = -1
    max_row = -1
    for index, value in enumerate(data):
        if int(value) == -1:
            continue
        row, col = divmod(index, width)
        min_col = min(min_col, col)
        max_col = max(max_col, col)
        min_row = min(min_row, row)
        max_row = max(max_row, row)
    if max_col < 0:
        return None
    return BBox(ox + min_col * resolution, oy + min_row * resolution, ox + (max_col + 1) * resolution, oy + (max_row + 1) * resolution)


def odom_xy(msg: Any) -> tuple[float, float] | None:
    pose = _get(_get(msg, "pose"), "pose")
    position = _get(pose, "position")
    if position is None:
        return None
    return float(_get(position, "x") or 0.0), float(_get(position, "y") or 0.0)


def _get(obj: Any, key: str) -> Any:
    if obj is None:
        return None
    if isinstance(obj, dict):
        return obj.get(key)
    return getattr(obj, key, None)


def choose_crop_bbox(
    *,
    maze_extent: BBox,
    map_bbox: BBox | None,
    trajectory_bbox: BBox | None,
    start_xy: tuple[float, float] | None,
    margin_m: float,
    bbox_override: BBox | None,
    full: bool,
) -> tuple[BBox, str]:
    if full:
        return maze_extent, "full_official_maze"
    if bbox_override is not None:
        return bbox_override, "explicit_bbox"
    combined = map_bbox.union(trajectory_bbox) if map_bbox is not None else trajectory_bbox
    if combined is not None and combined.valid:
        return clamp_bbox(combined.expand(margin_m), maze_extent), "auto_slam_and_trajectory_bbox"
    sx, sy = start_xy or (0.0, 0.0)
    return clamp_bbox(BBox(sx - 5.0, sy - 5.0, sx + 5.0, sy + 5.0), maze_extent), "fallback_start_10m_window"


def clamp_bbox(bbox: BBox, extent: BBox) -> BBox:
    return BBox(max(bbox.xmin, extent.xmin), max(bbox.ymin, extent.ymin), min(bbox.xmax, extent.xmax), min(bbox.ymax, extent.ymax))


def rasterize_walls(walls: list[Wall], bbox: BBox, *, resolution_m: float) -> dict[str, Any]:
    width = max(1, int(math.ceil((bbox.xmax - bbox.xmin) / resolution_m)))
    height = max(1, int(math.ceil((bbox.ymax - bbox.ymin) / resolution_m)))
    data = [-1] * (width * height)
    for row in range(height):
        y = bbox.ymin + (row + 0.5) * resolution_m
        for col in range(width):
            x = bbox.xmin + (col + 0.5) * resolution_m
            if point_inside_any_wall(x, y, walls):
                data[row * width + col] = 100
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
        "info": {
            "map_load_time": {"sec": 0, "nanosec": 0},
            "resolution": float(resolution_m),
            "width": width,
            "height": height,
            "origin": {
                "position": {"x": bbox.xmin, "y": bbox.ymin, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
        "data": data,
    }


def point_inside_any_wall(x: float, y: float, walls: list[Wall]) -> bool:
    for wall in walls:
        dx = x - wall.x
        dy = y - wall.y
        lx = math.cos(wall.yaw) * dx + math.sin(wall.yaw) * dy
        ly = -math.sin(wall.yaw) * dx + math.cos(wall.yaw) * dy
        if abs(lx) <= wall.length / 2.0 and abs(ly) <= wall.thickness / 2.0:
            return True
    return False


def write_lite_mcap(
    raw_mcap: Path,
    output_mcap: Path,
    overlay_msg: dict[str, Any],
    map_schema: Any,
    topic_profile: TopicProfile,
    mcap_modules: dict[str, Any],
) -> None:
    make_reader = mcap_modules["make_reader"]
    Writer = mcap_modules["Writer"]
    serialize_dynamic = mcap_modules["serialize_dynamic"]
    schema_ids: dict[int, int] = {}
    channel_ids: dict[int, int] = {}
    last_written: dict[str, int] = {}
    overlay_written = False
    overlay_encoder = serialize_dynamic(map_schema.name, map_schema.data.decode())[map_schema.name]

    with raw_mcap.open("rb") as src, output_mcap.open("wb") as dst:
        reader = make_reader(src)
        writer = Writer(dst)
        writer.start()
        overlay_schema_id = writer.register_schema(map_schema.name, map_schema.encoding, map_schema.data)
        overlay_channel_id = writer.register_channel(topic_profile.overlay_topic, "cdr", overlay_schema_id)
        for schema, channel, message in reader.iter_messages(log_time_order=True):
            interval = topic_profile.retain_intervals.get(channel.topic)
            if channel.topic not in topic_profile.retain_intervals:
                continue
            if interval is not None:
                previous = last_written.get(channel.topic)
                if previous is not None and message.log_time - previous < int(interval * 1e9):
                    continue
            if not overlay_written:
                writer.add_message(
                    channel_id=overlay_channel_id,
                    log_time=message.log_time,
                    publish_time=message.publish_time,
                    sequence=0,
                    data=overlay_encoder(overlay_msg),
                )
                overlay_written = True
            if schema.id not in schema_ids:
                schema_ids[schema.id] = writer.register_schema(schema.name, schema.encoding, schema.data)
            if channel.id not in channel_ids:
                channel_ids[channel.id] = writer.register_channel(
                    topic=channel.topic,
                    message_encoding=channel.message_encoding,
                    schema_id=schema_ids[schema.id],
                    metadata=channel.metadata,
                )
            writer.add_message(
                channel_id=channel_ids[channel.id],
                log_time=message.log_time,
                publish_time=message.publish_time,
                sequence=message.sequence,
                data=message.data,
            )
            last_written[channel.topic] = message.log_time
        writer.finish()


def inspect_output_counts(path: Path, mcap_modules: dict[str, Any]) -> dict[str, int]:
    if not path.is_file():
        return {}
    make_reader = mcap_modules["make_reader"]
    counts: dict[str, int] = {}
    with path.open("rb") as handle:
        reader = make_reader(handle)
        for _schema, channel, _message in reader.iter_messages(log_time_order=False):
            counts[channel.topic] = counts.get(channel.topic, 0) + 1
    return counts


def replay_quality_from_summary(summary: dict[str, Any]) -> dict[str, Any]:
    task_kind = task_kind_from_summary(summary)
    if task_kind == "hover":
        hover = summary.get("hover") or {}
        landing = summary.get("landing") or {}
        return {
            "profile": "hover_landing",
            "publishable": bool(summary.get("ok")) and bool(hover.get("ok")) and bool(landing.get("ok")),
            "path_length_m": 0.0,
            "accepted_goals": 0,
            "known_cell_growth": 0,
            "estimated_explored_area_m2": 0.0,
            "min_scan_clearance_m": None,
            "stop_drift_m": hover.get("stop_drift_m"),
            "warnings": [],
        }
    coverage = summary.get("coverage") or {}
    exploration = summary.get("p8_exploration") or {}
    safety = summary.get("safety") or {}
    path_length = float(coverage.get("path_length_m") or 0.0)
    accepted_goals = int(exploration.get("accepted_goals") or 0)
    publishable = path_length >= PUBLISHABLE_MIN_PATH_LENGTH_M and accepted_goals >= PUBLISHABLE_MIN_ACCEPTED_GOALS
    replay_profile = str(summary.get("replay_profile") or "conservative")
    quality_profile = f"p8_replay_{replay_profile}" if publishable else "minimal_run"
    return {
        "profile": quality_profile,
        "publishable": publishable,
        "path_length_m": path_length,
        "accepted_goals": accepted_goals,
        "known_cell_growth": int(coverage.get("known_cell_growth") or 0),
        "estimated_explored_area_m2": float(coverage.get("estimated_explored_area_m2") or 0.0),
        "min_scan_clearance_m": safety.get("min_scan_clearance_m"),
        "stop_drift_m": safety.get("stop_drift_m"),
        "warnings": [],
    }


def task_kind_from_summary(summary: dict[str, Any]) -> str:
    if "hover_gate" in summary or ("hover" in summary and "p8_exploration" not in summary):
        return "hover"
    if "p8_exploration" in summary or "p8_exploration_gate" in summary:
        return "p8"
    if "p12_airframe_disturbance_gate" in summary or "scan_robustness" in summary:
        return "scan_robustness"
    return "run"


def resolve_lite_topic_profile(summary: dict[str, Any], topic_profile_path: Path) -> Path:
    if topic_profile_path != FOXGLOVE_LITE_PROFILE:
        return topic_profile_path
    if task_kind_from_summary(summary) == "hover":
        return HOVER_FOXGLOVE_LITE_PROFILE
    return topic_profile_path


def _summary_template(
    run_dir: Path,
    raw_mcap: Path,
    output_mcap: Path,
    maze_path: Path,
    p8_summary: dict[str, Any],
    replay_quality: dict[str, Any],
    blockers: list[str],
) -> dict[str, Any]:
    task_kind = task_kind_from_summary(p8_summary)
    prerequisite = {"task": task_kind, "ok": bool(p8_summary.get("ok")), "summary": str(run_dir / "summary.json")}
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "blockers": blockers,
        "run_id": run_dir.name,
        "task": task_kind,
        "task_prerequisite": prerequisite,
        "p8_prerequisite": prerequisite,
        "replay_quality": replay_quality,
        "official_maze": {"source": str(maze_path), "sha256": file_sha256(maze_path) if maze_path.is_file() else ""},
        "replay_mcap": {
            "path": str(output_mcap),
            "raw_mcap_path": str(raw_mcap),
            "raw_mcap_size_bytes": raw_mcap.stat().st_size if raw_mcap.is_file() else 0,
            "foxglove_mcap_size_bytes": output_mcap.stat().st_size if output_mcap.is_file() else 0,
        },
        "truth_boundary": {
            "uses_official_maze_as_input": False,
            "uses_gazebo_truth_as_input": False,
            "official_maze_layer_role": "visualization_only",
        },
    }


def load_lite_required_topics(profile_path: Path = FOXGLOVE_LITE_PROFILE) -> set[str]:
    return load_lite_topic_profile(profile_path).output_required_topics


def load_lite_topic_profile(profile_path: Path = FOXGLOVE_LITE_PROFILE) -> TopicProfile:
    if not profile_path.is_file():
        raise FileNotFoundError(f"Foxglove-lite topic profile missing: {profile_path}")

    overlay_topic = ""
    required_topics: set[str] = set()
    retain_intervals: dict[str, float | None] = {}
    dropped_topics: set[str] = set()
    seen_topics: set[str] = set()

    for line_number, raw_line in enumerate(profile_path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        role = parts[0]
        if role == "overlay":
            if len(parts) != 2:
                raise ValueError(f"{profile_path}:{line_number}: overlay line must be `overlay TOPIC`")
            if overlay_topic:
                raise ValueError(f"{profile_path}:{line_number}: duplicate overlay topic")
            overlay_topic = _validate_topic(profile_path, line_number, parts[1])
            continue
        if role in {"required", "optional"}:
            if len(parts) != 3 or not parts[2].startswith("interval="):
                raise ValueError(f"{profile_path}:{line_number}: {role} line must be `{role} TOPIC interval=SECONDS|all`")
            topic = _validate_topic(profile_path, line_number, parts[1])
            if topic in seen_topics:
                raise ValueError(f"{profile_path}:{line_number}: duplicate topic: {topic}")
            seen_topics.add(topic)
            retain_intervals[topic] = _parse_interval(profile_path, line_number, parts[2].split("=", 1)[1])
            if role == "required":
                required_topics.add(topic)
            continue
        if role == "drop":
            if len(parts) != 2:
                raise ValueError(f"{profile_path}:{line_number}: drop line must be `drop TOPIC`")
            topic = _validate_topic(profile_path, line_number, parts[1])
            if topic in seen_topics:
                raise ValueError(f"{profile_path}:{line_number}: duplicate topic: {topic}")
            seen_topics.add(topic)
            dropped_topics.add(topic)
            continue
        raise ValueError(f"{profile_path}:{line_number}: unknown topic profile role: {role}")

    if not overlay_topic:
        raise ValueError(f"{profile_path}: missing overlay topic")
    if not required_topics:
        raise ValueError(f"{profile_path}: missing required topics")
    if overlay_topic in retain_intervals or overlay_topic in dropped_topics:
        raise ValueError(f"{profile_path}: overlay topic must not also be retained or dropped: {overlay_topic}")
    if not dropped_topics:
        raise ValueError(f"{profile_path}: missing explicit drop topics")
    return TopicProfile(
        path=profile_path,
        overlay_topic=overlay_topic,
        required_topics=required_topics,
        retain_intervals=retain_intervals,
        dropped_topics=dropped_topics,
    )


def _parse_interval(profile_path: Path, line_number: int, value: str) -> float | None:
    if value == "all":
        return None
    try:
        interval = float(value)
    except ValueError as exc:
        raise ValueError(f"{profile_path}:{line_number}: interval must be a positive number or all: {value}") from exc
    if interval <= 0:
        raise ValueError(f"{profile_path}:{line_number}: interval must be positive: {value}")
    return interval


def _validate_topic(profile_path: Path, line_number: int, topic: str) -> str:
    if not topic.startswith("/"):
        raise ValueError(f"{profile_path}:{line_number}: topic must be absolute: {topic}")
    return topic


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
