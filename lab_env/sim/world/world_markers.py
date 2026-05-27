from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from math import cos, inf, sin
from pathlib import Path

from lab_env.sim.perception.contract import DEFAULT_SCAN_CONTRACT, ScanContract


@dataclass(frozen=True, slots=True)
class MarkerColor:
    r: float
    g: float
    b: float
    a: float = 1.0


@dataclass(frozen=True, slots=True)
class MarkerPose:
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True, slots=True)
class MarkerScale:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class MarkerSpec:
    marker_id: int
    namespace: str
    shape: str
    pose: MarkerPose
    scale: MarkerScale
    color: MarkerColor
    frame_id: str = "map"


@dataclass(frozen=True, slots=True)
class WorldObstacleBox:
    name: str
    pose: MarkerPose
    scale: MarkerScale


VISIBLE_MODEL_NAMES = (
    "uav_start_marker",
    "obstacle_5m_ahead",
)


def _build_uav_body_markers(uav_marker: MarkerSpec) -> list[MarkerSpec]:
    half_length = uav_marker.scale.x / 2.0
    nose_length = 0.2
    tail_length = max(uav_marker.scale.x - nose_length, 0.0)
    tail_center_x = uav_marker.pose.x - (nose_length / 2.0)
    nose_center_x = uav_marker.pose.x + (half_length - nose_length / 2.0)
    return [
        MarkerSpec(
            marker_id=uav_marker.marker_id,
            namespace="uav_body_tail_marker",
            shape="cube",
            pose=MarkerPose(
                x=tail_center_x,
                y=uav_marker.pose.y,
                z=uav_marker.pose.z,
                roll=uav_marker.pose.roll,
                pitch=uav_marker.pose.pitch,
                yaw=uav_marker.pose.yaw,
            ),
            scale=MarkerScale(tail_length, uav_marker.scale.y, uav_marker.scale.z),
            color=MarkerColor(0.1, 0.35, 0.85, 1.0),
            frame_id=uav_marker.frame_id,
        ),
        MarkerSpec(
            marker_id=uav_marker.marker_id + 1,
            namespace="uav_body_nose_marker",
            shape="cube",
            pose=MarkerPose(
                x=nose_center_x,
                y=uav_marker.pose.y,
                z=uav_marker.pose.z,
                roll=uav_marker.pose.roll,
                pitch=uav_marker.pose.pitch,
                yaw=uav_marker.pose.yaw,
            ),
            scale=MarkerScale(nose_length, uav_marker.scale.y, uav_marker.scale.z),
            color=MarkerColor(0.96, 0.82, 0.32, 1.0),
            frame_id=uav_marker.frame_id,
        ),
    ]


def _parse_pose(text: str | None) -> MarkerPose:
    if not text:
        return MarkerPose(0.0, 0.0, 0.0)
    values = [float(value) for value in text.split()]
    while len(values) < 6:
        values.append(0.0)
    return MarkerPose(*values[:6])


def _parse_color(visual: ET.Element) -> MarkerColor:
    ambient = visual.findtext("material/ambient")
    if not ambient:
        return MarkerColor(0.8, 0.8, 0.8, 1.0)
    values = [float(value) for value in ambient.split()]
    while len(values) < 4:
        values.append(1.0)
    return MarkerColor(*values[:4])


def _visual_geometry(visual: ET.Element) -> tuple[str, MarkerScale]:
    if (box := visual.find("geometry/box/size")) is not None and box.text:
        size = [float(value) for value in box.text.split()]
        return "cube", MarkerScale(*size[:3])
    if (cylinder := visual.find("geometry/cylinder")) is not None:
        radius = float(cylinder.findtext("radius", "0"))
        length = float(cylinder.findtext("length", "0"))
        diameter = radius * 2.0
        return "cylinder", MarkerScale(diameter, diameter, length)
    raise ValueError("unsupported marker geometry")


def load_world_marker_specs(world_file: str | Path) -> list[MarkerSpec]:
    root = ET.parse(world_file).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("world element not found")

    specs: list[MarkerSpec] = []
    marker_id = 0
    for model in world.findall("model"):
        name = model.get("name", "")
        if name not in VISIBLE_MODEL_NAMES:
            continue

        pose = _parse_pose(model.findtext("pose"))
        visual = model.find("link/visual")
        if visual is None:
            raise ValueError(f"visual not found for model {name}")

        shape, scale = _visual_geometry(visual)
        color = _parse_color(visual)
        specs.append(
            MarkerSpec(
                marker_id=marker_id,
                namespace=name,
                shape=shape,
                pose=pose,
                scale=scale,
                color=color,
            )
        )
        marker_id += 1

    expanded_specs: list[MarkerSpec] = []
    for spec in specs:
        if spec.namespace == "uav_start_marker" and spec.shape == "cube":
            expanded_specs.extend(_build_uav_body_markers(spec))
        else:
            expanded_specs.append(spec)

    for index, spec in enumerate(expanded_specs):
        expanded_specs[index] = MarkerSpec(
            marker_id=index,
            namespace=spec.namespace,
            shape=spec.shape,
            pose=spec.pose,
            scale=spec.scale,
            color=spec.color,
            frame_id=spec.frame_id,
        )

    return expanded_specs


def load_world_model_pose(world_file: str | Path, model_name: str) -> MarkerPose:
    root = ET.parse(world_file).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("world element not found")

    for model in world.findall("model"):
        if model.get("name", "") == model_name:
            return _parse_pose(model.findtext("pose"))

    raise ValueError(f"model '{model_name}' not found in {world_file}")


def load_world_obstacle_boxes(world_file: str | Path) -> list[WorldObstacleBox]:
    root = ET.parse(world_file).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("world element not found")

    boxes: list[WorldObstacleBox] = []
    for model in world.findall("model"):
        name = model.get("name", "")
        if name.startswith("uav_") or name == "ground_plane":
            continue

        collision = model.find("link/collision/geometry/box/size")
        if collision is None or not collision.text:
            continue

        size = [float(value) for value in collision.text.split()]
        boxes.append(
            WorldObstacleBox(
                name=name,
                pose=_parse_pose(model.findtext("pose")),
                scale=MarkerScale(*size[:3]),
            )
        )

    return boxes


def _ray_box_distance(
    *,
    origin_x: float,
    origin_y: float,
    dir_x: float,
    dir_y: float,
    box: WorldObstacleBox,
) -> float | None:
    min_x = box.pose.x - box.scale.x / 2.0
    max_x = box.pose.x + box.scale.x / 2.0
    min_y = box.pose.y - box.scale.y / 2.0
    max_y = box.pose.y + box.scale.y / 2.0

    if min_x <= origin_x <= max_x and min_y <= origin_y <= max_y:
        return 0.0

    t_min = -inf
    t_max = inf

    for origin, direction, lower, upper in (
        (origin_x, dir_x, min_x, max_x),
        (origin_y, dir_y, min_y, max_y),
    ):
        if abs(direction) < 1e-9:
            if origin < lower or origin > upper:
                return None
            continue

        axis_t1 = (lower - origin) / direction
        axis_t2 = (upper - origin) / direction
        if axis_t1 > axis_t2:
            axis_t1, axis_t2 = axis_t2, axis_t1
        t_min = max(t_min, axis_t1)
        t_max = min(t_max, axis_t2)
        if t_min > t_max:
            return None

    if t_max < 0.0:
        return None
    return t_min if t_min >= 0.0 else t_max


def synthesize_planar_scan(
    obstacle_boxes: list[WorldObstacleBox],
    *,
    origin_x: float,
    origin_y: float,
    origin_z: float,
    yaw: float,
    contract: ScanContract = DEFAULT_SCAN_CONTRACT,
) -> list[float]:
    ranges = [contract.invalid_range_value] * contract.beam_count

    for index in range(contract.beam_count):
        beam_angle = yaw + contract.angle_min + contract.angle_increment * index
        dir_x = cos(beam_angle)
        dir_y = sin(beam_angle)
        beam_min: float | None = None

        for obstacle in obstacle_boxes:
            min_z = obstacle.pose.z - obstacle.scale.z / 2.0
            max_z = obstacle.pose.z + obstacle.scale.z / 2.0
            if origin_z < min_z or origin_z > max_z:
                continue

            distance = _ray_box_distance(
                origin_x=origin_x,
                origin_y=origin_y,
                dir_x=dir_x,
                dir_y=dir_y,
                box=obstacle,
            )
            if distance == 0.0:
                distance = contract.range_min
            if distance is None or distance < contract.range_min or distance > contract.range_max:
                continue
            beam_min = distance if beam_min is None else min(beam_min, distance)

        if beam_min is not None:
            ranges[index] = beam_min

    return ranges


def compute_forward_clearance(
    obstacle_boxes: list[WorldObstacleBox],
    *,
    origin_x: float,
    origin_y: float,
    origin_z: float,
    yaw: float,
) -> float | None:
    dir_x = cos(yaw)
    dir_y = sin(yaw)
    nearest_distance: float | None = None

    for obstacle in obstacle_boxes:
        min_z = obstacle.pose.z - obstacle.scale.z / 2.0
        max_z = obstacle.pose.z + obstacle.scale.z / 2.0
        if origin_z < min_z or origin_z > max_z:
            continue

        distance = _ray_box_distance(
            origin_x=origin_x,
            origin_y=origin_y,
            dir_x=dir_x,
            dir_y=dir_y,
            box=obstacle,
        )
        if distance is None:
            continue
        nearest_distance = distance if nearest_distance is None else min(nearest_distance, distance)

    return nearest_distance
