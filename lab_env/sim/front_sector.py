from __future__ import annotations

from dataclasses import dataclass
from math import degrees

from lab_env.sim.contract import DEFAULT_SCAN_CONTRACT, ScanContract


@dataclass(frozen=True, slots=True)
class FrontSectorReport:
    front_min: float | None
    state: str
    valid_points: int


def _is_valid_range(value: float, contract: ScanContract) -> bool:
    return contract.range_min <= value <= contract.range_max


def compute_front_min(
    ranges: list[float],
    *,
    contract: ScanContract = DEFAULT_SCAN_CONTRACT,
    front_half_width_deg: float = 15.0,
) -> float | None:
    front_values: list[float] = []
    for index, value in enumerate(ranges):
        angle_deg = degrees(contract.angle_min + contract.angle_increment * index)
        if abs(angle_deg) > front_half_width_deg:
            continue
        if _is_valid_range(value, contract):
            front_values.append(value)
    if not front_values:
        return None
    return min(front_values)


def classify_front_sector(
    ranges: list[float],
    *,
    contract: ScanContract = DEFAULT_SCAN_CONTRACT,
    front_half_width_deg: float = 15.0,
    obstacle_seen_distance: float = 6.0,
    avoid_distance: float = 1.0,
) -> FrontSectorReport:
    front_min = compute_front_min(
        ranges,
        contract=contract,
        front_half_width_deg=front_half_width_deg,
    )
    valid_points = sum(1 for value in ranges if _is_valid_range(value, contract))

    if front_min is None:
        return FrontSectorReport(front_min=None, state="clear", valid_points=valid_points)
    if front_min < avoid_distance:
        return FrontSectorReport(front_min=front_min, state="avoid_required", valid_points=valid_points)
    if front_min < obstacle_seen_distance:
        return FrontSectorReport(front_min=front_min, state="obstacle_seen", valid_points=valid_points)
    return FrontSectorReport(front_min=front_min, state="clear", valid_points=valid_points)

