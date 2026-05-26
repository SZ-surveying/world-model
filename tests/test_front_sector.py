from __future__ import annotations

from lab_env.sim.contract import DEFAULT_SCAN_CONTRACT
from lab_env.sim.front_sector import classify_front_sector, compute_front_min


def _ranges_with_front_obstacle(distance: float) -> list[float]:
    ranges = [DEFAULT_SCAN_CONTRACT.invalid_range_value] * DEFAULT_SCAN_CONTRACT.beam_count
    center = DEFAULT_SCAN_CONTRACT.beam_count // 2
    for offset in range(-5, 6):
        ranges[center + offset] = distance
    return ranges


def test_front_min_is_none_for_empty_scan() -> None:
    ranges = [DEFAULT_SCAN_CONTRACT.invalid_range_value] * DEFAULT_SCAN_CONTRACT.beam_count
    assert compute_front_min(ranges) is None
    assert classify_front_sector(ranges).state == "clear"


def test_front_obstacle_is_reported() -> None:
    ranges = _ranges_with_front_obstacle(5.0)
    assert compute_front_min(ranges) == 5.0
    assert classify_front_sector(ranges).state == "obstacle_seen"


def test_close_obstacle_triggers_avoid_required() -> None:
    ranges = _ranges_with_front_obstacle(0.5)
    report = classify_front_sector(ranges)
    assert report.front_min == 0.5
    assert report.state == "avoid_required"
